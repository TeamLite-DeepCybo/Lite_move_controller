#!/usr/bin/env python3
"""Offline DDS regression for the actual gripper executable; no hardware topics."""

import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not (os.environ.get("ROS_DOMAIN_ID") == "173"
            and os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST"):
        parser.error("Use dedicated ROS_DOMAIN_ID=173 and LOCALHOST discovery")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    prefix = "/lite_gripper_regression"
    report = {"hardware_started": False, "status": "failed", "checks": []}
    rclpy.init()
    node = rclpy.create_node("gripper_regression_probe")
    frames = []
    subscription = node.create_subscription(
        JointState, prefix + "/output", lambda msg: frames.append(dict(zip(msg.name, msg.position))), 10
    )
    state = node.create_publisher(JointState, prefix + "/joint_states", 10)
    command = node.create_publisher(JointState, prefix + "/command", 10)
    opening = node.create_publisher(Float64, prefix + "/opening", 10)
    child = None
    log = args.output.with_suffix(".node.log").open("w")

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if child is not None:
                assert child.poll() is None, "gripper node exited"
            rclpy.spin_once(node, timeout_sec=0.02)

    def check(condition, label):
        if not condition:
            raise AssertionError(label)
        report["checks"].append(label)

    def stop():
        nonlocal child
        if child is not None:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=10)
            check(child.returncode == 0, "node exits gracefully")
            child = None

    def start():
        nonlocal child
        options = [str(args.executable.resolve()), "--ros-args"]
        for key, topic in {"command_topic": "command", "opening_topic": "opening",
                           "joint_states_topic": "joint_states", "output_topic": "output"}.items():
            options += ["-p", f"{key}:={prefix}/{topic}"]
        child = subprocess.Popen(options, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 10
        while node.count_publishers(prefix + "/output") == 0 or command.get_subscription_count() == 0:
            assert time.monotonic() < deadline, "DDS discovery timed out"
            spin(0.1)
        spin(0.25)

    def js(names, positions):
        msg = JointState()
        msg.name, msg.position = names, positions
        return msg

    def send(publisher, msg):
        for _ in range(3):
            publisher.publish(msg)
            spin(0.06)
        spin(0.15)
        return frames[-1] if frames else {}

    def equal(actual, expected):
        return actual.keys() == expected.keys() and all(
            math.isclose(actual[k], v, abs_tol=1e-9) for k, v in expected.items()
        )

    left, right = "left_gripper", "right_gripper"
    try:
        start()
        check(not frames, "no target before feedback or explicit command")
        actual = send(state, js([left], [0.031]))
        check(equal(actual, {left: 0.031}), "single measured side never invents other side")
        actual = send(command, js([left], [0.0]))
        check(equal(actual, {left: 0.0}), "explicit zero closes initialized side")
        actual = send(state, js([right], [0.017]))
        check(equal(actual, {left: 0.0, right: 0.017}), "late right measurement retains left command")
        actual = send(command, js([left, right], [1.0, 0.5]))
        expected = {left: 0.047, right: 0.0235}
        check(equal(actual, expected), "independent normalized targets map to metres")
        actual = send(command, js([right], [0.0]))
        expected[right] = 0.0
        check(equal(actual, expected), "omitted side retains confirmed target")
        for names, positions, label in [
            ([left, right], [0.2, float("nan")], "NaN"),
            ([left, right], [0.2, float("inf")], "Inf"),
            ([left, right], [0.2], "short positions"),
            ([left], [0.2, 0.4], "long positions"),
            ([left, "unknown"], [0.2, 0.4], "unknown name"),
            ([left, left], [0.2, 0.4], "duplicate name"),
            ([left, right], [0.2, 1.5], "above range"),
            ([left, right], [0.2, -0.1], "below range"),
            ([], [], "empty command"),
        ]:
            actual = send(command, js(names, positions))
            check(equal(actual, expected), f"atomic rejection: {label}")
        for value in [float("nan"), float("inf"), -0.1, 1.5]:
            msg = Float64()
            msg.data = value
            check(equal(send(opening, msg), expected), "invalid global opening leaves both targets")
        msg = Float64()
        msg.data = 0.5
        check(equal(send(opening, msg), {left: 0.0235, right: 0.0235}), "global half opening")
        stop()
        spin(0.3)
        frames.clear()
        start()
        check(not frames, "restart does not emit default zero")
        actual = send(state, js([left, right], [0.03, 0.02]))
        check(equal(actual, {left: 0.03, right: 0.02}), "restart holds new measurements")
        stop()
        spin(0.2)
        frames.clear()
        start()
        check(not frames, "fresh instance remains silent")
        actual = send(command, js([right], [0.0]))
        check(equal(actual, {right: 0.0}), "explicit zero before feedback is valid; other side omitted")
        forbidden = {"/joint_states", "/gripper/command", "/gripper/opening",
                     "/lite_move_controller/gripper_command"}
        check(not forbidden.intersection(name for name, _ in node.get_topic_names_and_types()),
              "no production hardware topics")
        report["status"] = "passed"
    finally:
        try:
            stop()
        except Exception:
            report["status"] = "failed"
            raise
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
            log.close()
            args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"{len(report['checks'])} DDS checks passed")


if __name__ == "__main__":
    main()
