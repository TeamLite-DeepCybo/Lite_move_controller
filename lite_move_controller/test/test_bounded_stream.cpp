#include <cassert>
#include <limits>
#include "lite_move_controller/bounded_stream.hpp"
using lite_move_controller::BoundedStream;
int main()
{
  BoundedStream s;
  std::vector<std::string> names{"arm", "gripper"};
  std::vector<double> lo{-1, 0}, hi{1, .047};
  assert(s.claim("", "s1", 100));
  assert(!s.claim("s1", "s2", 101)); // never steals a live stream
  auto send = [&](std::string token, unsigned seq, std::vector<double> q, double age=0) {
    return s.accept(token, "cmd", seq, age, names, q, names, lo, hi, false, 120);
  };
  assert(send("s1", 1, {.1, 0})); // explicit zero
  const auto previous = s.positions;
  assert(!send("s1", 2, {.2, .05}));
  assert(s.positions == previous && s.accepted == 1); // atomic validation
  assert(!send("s1", 2, {std::numeric_limits<double>::quiet_NaN(), 0}));
  assert(!send("s0", 2, {.2, 0}));
  assert(!send("s1", 1, {.2, 0}));
  assert(!send("s1", 2, {.2, 0}, 201));
  assert(!send("s1", 2, {.2, 0}, -11));
  assert(!s.accept("s1", "cmd", 2, 0, {"gripper", "arm"}, {0,.2}, names,
    lo, hi, false, 120));
  assert(s.watchdog(321));
  assert(s.owner.empty() && s.expired && s.holding && !s.pending);
  assert(!send("s1", 3, {.3, 0}));
  assert(s.claim("", "s2", 330));
  assert(s.accept("s2", "hold", 1, 0, names, {.1,.01}, names, lo, hi, true, 340));
  assert(s.holding);
  return 0;
}
