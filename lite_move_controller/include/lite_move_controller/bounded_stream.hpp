#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace lite_move_controller
{
// Caller serializes access. Clock is steady milliseconds; sample age is checked
// separately against the ROS clock. No ROS, CAN, enabling or mode changes here.
struct BoundedStream
{
  std::string owner, command, rejection;
  std::uint64_t accepted{0}, applied{0};
  std::int64_t received_ms{0};
  double timeout_ms{200.0};
  bool holding{true}, expired{false}, pending{false};
  std::vector<double> positions;

  bool claim(const std::string & expected, const std::string & next, std::int64_t now)
  {
    if (next.empty() || owner != expected || !owner.empty()) {
      rejection = "live owner or invalid claim; wait for watchdog hold";
      return false;
    }
    owner = next; command.clear(); accepted = applied = 0;
    received_ms = now; expired = false; holding = true; pending = false;
    rejection.clear();
    return true;
  }

  bool accept(const std::string & token, const std::string & id, std::uint64_t seq,
    double sample_age_ms, const std::vector<std::string> & names,
    const std::vector<double> & q, const std::vector<std::string> & canonical,
    const std::vector<double> & lo, const std::vector<double> & hi,
    bool hold, std::int64_t now)
  {
    auto reject = [this](const char * why) {rejection = why; return false;};
    if (owner.empty() || token != owner || expired) {return reject("stream fenced");}
    if (now - received_ms > timeout_ms) {return reject("stream expired");}
    if (seq <= accepted || id.empty()) {return reject("old sequence or empty command");}
    if (!std::isfinite(sample_age_ms) || sample_age_ms < -10.0 ||
        sample_age_ms > timeout_ms) {return reject("batch timestamp stale/future");}
    if (names != canonical || q.size() != canonical.size() ||
        lo.size() != q.size() || hi.size() != q.size()) {
      return reject("full canonical batch required");
    }
    for (std::size_t i = 0; i < q.size(); ++i) {
      if (!std::isfinite(q[i]) || q[i] < lo[i] || q[i] > hi[i]) {
        return reject("nonfinite or out-of-limit batch");
      }
    }
    positions = q; command = id; accepted = seq; holding = hold;
    received_ms = now; pending = true; rejection.clear();
    return true;
  }

  bool watchdog(std::int64_t now)
  {
    if (owner.empty() || now - received_ms <= timeout_ms) {return false;}
    owner.clear(); pending = false; holding = true; expired = true;
    rejection = "stream timeout: measured hold latched; old token revoked; last rejection: " + rejection;
    return true;
  }
};
}  // namespace lite_move_controller
