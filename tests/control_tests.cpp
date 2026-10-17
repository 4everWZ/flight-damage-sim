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
void path_load_projection(const Definition &d) {
  // Controlled sensor states, not prescribed flight motion. A balanced total
  // force must produce zero path-normal error regardless of body/flow axes or
  // how the same force is divided between airframe and propeller.
  for (double yaw : {-1., 0., 1.})
    for (double pitch : {-.2, .2})
      for (double bank : {-.4, 0., .4})
        for (double path : {-.1, 0., .1}) {
          Simulation s(d);
          s.body.attitude = Quat::axis({0, 0, 1}, yaw) *
                            Quat::axis({0, 1, 0}, pitch) *
                            Quat::axis({1, 0, 0}, bank);
          s.wind_world = {2, -3, 1};
          PilotIntent p;
          p.mouse = true;
          p.aim_world = {std::cos(path) * std::cos(yaw),
                         std::cos(path) * std::sin(yaw), -std::sin(path)};
          s.body.velocity = s.wind_world + p.aim_world * 70;
          double weight = s.mass.mass * gravity;
          s.propulsion.force = {weight * .1, weight * .03, -weight * .02};
          s.aero.force = s.body.attitude.conjugate().rotate({0, 0, -weight}) -
                         s.propulsion.force;
          s.load_factor = -s.aero.force.z / weight;
          s.update_controls(p);
          require(std::abs(s.instructor.desired_normal_g - s.load_factor) < 1e-12,
                  "force-balanced flight has no fictitious normal-load deficit");
        }
  Simulation trimmed(d);
  auto tr = trimmed.trim(70, 1000);
  require(tr.converged, "path-loop regression requires actual trimmed entry");
  auto p = tr.intent;
  p.mouse = true;
  p.aim_world = unit(trimmed.body.velocity - trimmed.wind_world);
  trimmed.update_controls(p);
  // Trim may include a small bank for propeller asymmetry; the separate
  // coordination-rate request need not vanish. Its NORMAL-LOAD error must.
  require(std::abs(trimmed.instructor.desired_normal_g - trimmed.load_factor) < 1e-9,
          "actual force-balanced trim has no fictitious normal-load deficit");
}
void moving_aim(const Definition &d) {
  // Prescribed sensor bench: verify the rate observer against a continuous
  // rotating direction (including atan2 wrap), independently of flight dynamics.
  for (double hz : {120., 240.})
    for (double sign : {-1., 1.}) {
      Simulation s(d, 1 / hz);
      PilotIntent p;
      p.mouse = true;
      for (int i = 0; i <= int(2 * hz); ++i) {
        double heading = sign * (3 + .08 * i / hz);
        p.aim_world = {std::cos(heading), std::sin(heading), 0};
        s.body.velocity = p.aim_world * 70;
        s.update_controls(p);
        double expected = sign * .08 * (1 - std::exp(-i / (hz * .15)));
        require(std::abs(s.instructor.aim_heading_rate - expected) < 2e-13,
                "constant rotating aim matches analytic filtered rate across wrap");
        require(std::abs(s.instructor.desired_bank -
                         std::atan(70 * expected / gravity)) < 2e-12,
                "zero heading error still requests the coordinated turn rate");
      }
      auto bias = s.instructor.rate_bias[Roll];
      p.active[Roll] = true;
      p.direct[Roll] = -1;
      s.update_controls(p);
      require(s.ownership[Roll].command == -1 &&
                  s.ownership[Roll].direct_weight == 1 &&
                  s.ownership[Pitch].direct_weight == 0 &&
                  s.ownership[Yaw].direct_weight == 0 &&
                  s.instructor.rate_bias[Roll] == bias,
              "moving aim cannot dilute direct-axis authority or unfreeze its PI");
      p.active[Roll] = false;
      s.update_controls(p);
      require(s.ownership[Roll].command == -1,
              "moving aim release retains initial command continuity");
      for (int i = 0; i <= int(d.handoff * hz); ++i)
        s.update_controls(p);
      require(s.ownership[Roll].direct_weight == 0,
              "moving aim handoff ends at configured duration");
      for (int reason = 0; reason < 5; ++reason) {
        auto reset = s;
        if (reason == 0) p.mouse = false;
        if (reason == 1) reset.instructor_mode = InstructorMode::Off;
        if (reason == 2) reset.instructor_mode = InstructorMode::Legacy;
        if (reason == 3) reset.guidance.aim_rate_feedforward = false;
        if (reason == 4) p.aim_world = {0, 0, -1};
        reset.update_controls(p);
        require(!reset.instructor.aim_history_valid &&
                    reset.instructor.aim_heading_rate == 0,
                "inactive guidance and vertical aim invalidate rate history");
        p.mouse = true;
        p.aim_world = {0, 1, 0};
        reset.instructor_mode = InstructorMode::FlightPath;
        reset.guidance.aim_rate_feedforward = true;
        reset.update_controls(p);
        require(reset.instructor.aim_heading_rate == 0,
                "reactivation does not differentiate a stale aim");
      }
    }
  Simulation jump(d);
  PilotIntent p;
  p.mouse = true;
  jump.update_controls(p);
  p.aim_world = {0, 1, 0};
  jump.update_controls(p);
  double ceiling = gravity / 70 * std::tan(jump.guidance.max_bank);
  require(std::abs(jump.instructor.aim_heading_rate) <= ceiling &&
              std::abs(jump.instructor.desired_bank) <= jump.guidance.max_bank,
          "an aim jump cannot charge unbounded feedforward memory");
  double before = jump.instructor.aim_heading_rate;
  for (int i = 0; i < 120; ++i) jump.update_controls(p);
  require(std::abs(jump.instructor.aim_heading_rate - before * std::exp(-1 / .15)) < 1e-14,
          "held aim rate decays in physical time");

  // Actual 6DoF counterfactual: guidance must not move an aircraft differently
  // when every pilot linkage is disconnected. No body state is prescribed here.
  Simulation on(d);
  p = on.trim(70, 1000).intent;
  auto off = on;
  off.guidance.aim_rate_feedforward = false;
  for (const auto &c : d.chains) {
    on.disconnect(c.name);
    off.disconnect(c.name);
  }
  p.mouse = true;
  for (int i = 0; i < 180; ++i) {
    double heading = .08 * i * on.dt;
    p.aim_world = {std::cos(heading), std::sin(heading), 0};
    on.step(p);
    off.step(p);
    require(norm(on.body.position - off.body.position) < 1e-12 &&
                norm(on.body.velocity - off.body.velocity) < 1e-12 &&
                norm(on.body.omega - off.body.omega) < 1e-12,
            "rate feedforward cannot bypass disconnected surfaces in flight");
  }
}
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe required");
    auto d = Definition::load(argv[1]);
    path_load_projection(d);
    moving_aim(d);
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
