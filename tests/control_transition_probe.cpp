#include "flight/core.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;

// Held-body controls-only trace. No new actuator law or flight-force path.
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe required");
    auto d = Definition::load(argv[1]);
    std::cout << std::setprecision(17)
              << "hz,fault,event_s,time_s,phase,left_rad,right_rad,left_mid_rad,"
                 "right_mid_rad,left_filtered_rad,right_filtered_rad,left_rate_bound,"
                 "right_rate_bound,left_connected_target,right_connected_target,left_connected,left_jammed,"
                 "body_event_equal\n";
    for (double hz : {120., 240., 480.})
      for (const std::string fault : {"none", "disconnect", "jam"})
        for (double event : {1. / 60, .125}) {
          if (fault == "none" && event != 1. / 60)
            continue;
          Simulation s(d, 1 / hz);
          s.instructor_mode = InstructorMode::Off;
          s.density = 1.225 * std::exp(-1000. / 8500);
          auto index = [&](const std::string &name) {
            auto p = std::find_if(s.def.chains.begin(), s.def.chains.end(),
                                   [&](const auto &c) { return c.name == name; });
            if (p == s.def.chains.end())
              throw std::runtime_error("missing actuator");
            return std::size_t(p - s.def.chains.begin());
          };
          auto &left = s.def.chains[index("left_aileron")];
          auto &right = s.def.chains[index("right_aileron")];
          double q = .5 * s.density * 70 * 70;
          auto effort = [&](const auto &c) { return 1 / (1 + q / c.q_limit); };
          auto rate = [&](const auto &c) {
            return c.slew * effort(c) / (1 + std::max(c.friction, 0.));
          };
          auto target = [&](const auto &c) {
            return .2 * c.limit * clamp(c.range, 0, 1) * effort(c);
          };
          if (left.lag != 0 || right.lag != 0 || left.friction != 0 || right.friction != 0)
            throw std::runtime_error("analytic bench requires this zero-lag/friction fixture");
          auto emit = [&](double time, const char *phase) {
            std::cout << hz << ',' << fault << ',' << (fault == "none" ? -1 : event)
                      << ',' << time << ',' << phase << ',' << left.deflection
                      << ',' << right.deflection << ',' << left.midpoint_deflection
                      << ',' << right.midpoint_deflection << ',' << left.filtered
                      << ',' << right.filtered << ',' << rate(left) << ','
                      << rate(right) << ',' << target(left) << ',' << target(right)
                      << ',' << left.connected << ',' << left.jammed << ",1\n";
          };
          PilotIntent p;
          p.active[Roll] = true;
          p.direct[Roll] = .2; // Same held input before and after every event.
          int event_tick = int(std::llround(event * hz));
          emit(0, "sample");
          for (int i = 0; i < int(.25 * hz); ++i) {
            if (fault != "none" && i == event_tick) {
              emit(i / hz, "pre_event");
              auto body_before = s.body;
              double before = left.deflection;
              if (fault == "disconnect")
                s.disconnect("left_aileron");
              else {
                left.jam_position = left.deflection;
                left.jammed = true;
              }
              auto a = s.body.attitude, b = body_before.attitude;
              if (left.deflection != before || norm(s.body.position - body_before.position) != 0 ||
                  norm(s.body.velocity - body_before.velocity) != 0 ||
                  norm(s.body.omega - body_before.omega) != 0 ||
                  a.w != b.w || a.x != b.x || a.y != b.y || a.z != b.z)
                throw std::runtime_error("event changed physical angle or held body state");
              emit(i / hz, "post_event");
            }
            double old_left = left.deflection, old_right = right.deflection;
            s.update_controls(p);
            if (s.ownership[Roll].direct_weight != 1 ||
                s.ownership[Pitch].direct_weight != 0 || s.ownership[Yaw].direct_weight != 0)
              throw std::runtime_error("held direct-input ownership changed");
            if (std::abs(left.deflection - old_left) > rate(left) / hz + 1e-12 ||
                std::abs(right.deflection - old_right) > rate(right) / hz + 1e-12)
              throw std::runtime_error("physical surface exceeded existing slew bound");
            emit((i + 1) / hz, "sample");
          }
        }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
