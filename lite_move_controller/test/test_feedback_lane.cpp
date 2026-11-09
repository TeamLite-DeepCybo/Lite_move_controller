#include "lite_move_controller/bridge_client.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

static std::int64_t steady_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void check(bool ok, const char * message)
{
  if (!ok) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv)
{
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  const char * isolated = std::getenv("LITE_DCH_FAKE_ONLY");
  if (!domain || std::string(domain) != "186" || !isolated || std::string(isolated) != "1") {
    std::cerr << "requires isolated domain 186 and LITE_DCH_FAKE_ONLY=1\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("feedback_lane_test", "/lite_feedback_test");
    auto source = std::make_shared<rclcpp::Node>("feedback_source", "/lite_feedback_test");
    auto publisher = source->create_publisher<sensor_msgs::msg::JointState>(
      "/lite_feedback_test/joints", rclcpp::QoS(1).reliable());
    lite_move_controller::BridgeClient client(node.get(), "fixture", {"joint"},
      "/lite_feedback_test/command", "/lite_feedback_test/joints", 200.0);
    std::string error;
    check(client.start(error), "receiver failed to start");
    sensor_msgs::msg::JointState msg;
    msg.name = {"joint"}; msg.position = {0.1}; msg.velocity = {0.0}; msg.effort = {0.0};
    auto publish_for = [&](int ms, bool fresh) {
        const auto end = steady_ms() + ms;
        while (steady_ms() < end) {
          if (fresh) {msg.header.stamp = source->now();}
          publisher->publish(msg);
          std::this_thread::sleep_for(5ms);
        }
      };
    // Never spin the main/controller node: its general executor is entirely
    // unavailable. The dedicated lane must nevertheless receive sensor data.
    publish_for(1200, true);
    check(client.stateReady(), "feedback depends on main executor");
    check(client.stateAgeMs(steady_ms()) < 100, "healthy independent feedback became stale");
    // The receiver must not acquire command or service authority as a side
    // effect of isolation. This checks the actual DDS graph, not only options.
    const std::string receiver_name = "feedback_lane_test_feedback";
    auto graph = source->get_node_graph_interface();
    const auto node_names = graph->get_node_names();
    check(std::find(node_names.begin(), node_names.end(),
      "/lite_feedback_test/" + receiver_name) != node_names.end(), "receiver node missing from isolated graph");
    check(graph->get_publisher_names_and_types_by_node(receiver_name, "/lite_feedback_test").empty(),
      "feedback receiver has publisher authority");
    check(graph->get_service_names_and_types_by_node(receiver_name, "/lite_feedback_test").empty(),
      "feedback receiver exposes a service");
    const auto last_stamp = client.snapshot().source_stamp_ns;
    publish_for(180, false);
    check(client.snapshot().source_stamp_ns == last_stamp, "duplicate source timestamp refreshed sample");
    check(client.stateAgeMs(steady_ms()) >= 100, "replayed feedback bypassed unchanged watchdog");
    publish_for(100, true);
    check(client.stateAgeMs(steady_ms()) < 100, "fresh samples did not recover receiver");
    msg.header.stamp = (source->now() + rclcpp::Duration::from_seconds(2.0));
    publish_for(180, false);
    check(client.stateAgeMs(steady_ms()) >= 100, "future sample bypassed watchdog");
    // The receiver must not share the caller's RMW context. Shutting down the
    // caller context cannot silence the passive receiver while it is alive.
    rclcpp::shutdown();
    auto source_context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions options;
    options.set_domain_id(186);
    options.auto_initialize_logging(false);
    source_context->init(0, nullptr, options);
    source = std::make_shared<rclcpp::Node>("independent_source", "/lite_feedback_test",
      rclcpp::NodeOptions().context(source_context));
    publisher = source->create_publisher<sensor_msgs::msg::JointState>(
      "/lite_feedback_test/joints", rclcpp::QoS(1).reliable());
    publish_for(3000, true);
    if (client.stateAgeMs(steady_ms()) >= 100 || client.stateAgeMs(steady_ms()) < 0) {
      std::cerr << "age=" << client.stateAgeMs(steady_ms()) << " source=" << client.snapshot().source_stamp_ns << " now=" << source->now().nanoseconds() << " matches=" << publisher->get_subscription_count() << std::endl;
    }
    check(client.stateAgeMs(steady_ms()) < 100 && client.stateAgeMs(steady_ms()) >= 0, "receiver did not recover after caller context shutdown and producer replacement");
    source_context->shutdown("fixture finished");
    std::cout << "PASS: independent context/receiver; stale/replayed/future samples remain fenced\n";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; result = 1;
  }
  rclcpp::shutdown();
  return result;
}
