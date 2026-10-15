#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
void hold_guidance(Simulation &s) {
  s.guidance.heading_gain = 0;
  s.guidance.path_gain = 0;
  s.guidance.normal_gain = 0;
}
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe required");
    auto d = Definition::load(argv[1]);
    const std::array<double, 3> kp{.9, 1.4, 1.}, ki{.8, 1.2, .5};
    double worst = 0, spread = 0;
    for (int axis = 0; axis < 3; ++axis)
      for (double sign : {-1., 1.}) {
        double first = 0;
        for (double hz : {60., 120., 240., 480.}) {
          Simulation s(d, 1 / hz);
          hold_guidance(s);
          PilotIntent p;
          p.mouse = true;
          s.body.omega[axis] = -sign * .1008 / kp[axis];
          s.instructor.rate_bias[axis] = sign * .899;
          for (int i = 0; i < int(hz / 60); ++i) {
            s.update_controls(p);
            double raw = kp[axis] * (s.instructor.desired_rates[axis] -
                                     s.body.omega[axis]) +
                         s.instructor.rate_bias[axis];
            worst = std::max(worst, std::max(std::abs(raw) - 1, 0.));
            require(std::abs(raw) <= 1 + 1e-13,
                    "integration cannot create command overshoot at a "
                    "saturation crossing");
          }
          require(std::abs(s.instructor.rate_bias[axis] - sign * .8992) < 2e-14,
                  "constant-rate boundary lands on analytic reachable integral "
                  "bias");
          if (hz == 60)
            first = s.instructor.rate_bias[axis];
          spread =
              std::max(spread, std::abs(s.instructor.rate_bias[axis] - first));
        }
        Simulation linear(d);
        hold_guidance(linear);
        PilotIntent p;
        p.mouse = true;
        linear.body.omega[axis] = -sign * .02;
        linear.update_controls(p);
        require(std::abs(linear.instructor.rate_bias[axis] -
                         ki[axis] * sign * .02 * linear.dt) < 1e-15,
                "unsaturated PI increment remains unchanged");
        for (int reason = 0; reason < 3; ++reason) {
          Simulation frozen(d);
          hold_guidance(frozen);
          frozen.body.omega[axis] = -sign * .1008 / kp[axis];
          frozen.instructor.rate_bias[axis] = sign * .899;
          p.active = {};
          p.direct = {};
          if (reason == 0) {
            p.active[axis] = true;
            p.direct[axis] = sign;
          }
          if (reason == 1) {
            p.active[axis] = true;
            p.direct[axis] = sign;
            frozen.update_controls(p);
            p.active[axis] = false;
            frozen.update_controls(p); // Actual release/handoff history.
          }
          if (reason == 2)
            for (auto &chain : frozen.def.chains)
              if (chain.axis == axis)
                frozen.disconnect(chain.name);
          frozen.update_controls(p);
          require(frozen.instructor.integral_paused[axis] &&
                      frozen.instructor.rate_bias[axis] == sign * .899,
                  "direct input, handoff and absent physical controls keep the "
                  "integral frozen");
          if (reason == 0)
            require(frozen.ownership[axis].direct_weight == 1 &&
                        frozen.ownership[axis].command == sign,
                    "keyboard retains independent full axis authority");
        }
      }
    std::cout << std::setprecision(13)
              << "{\"checks_passed\":true,\"max_boundary_overshoot\":" << worst
              << ",\"held_error_bias_dt_spread\":" << spread
              << ",\"rates_hz\":[60,120,240,480]}\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
