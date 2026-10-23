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
struct AvailabilityCheck {
  int cases = 0, immobile = 0;
  double frozen_drift = 0, moving_error = 0, immobile_motion = 0;
};
AvailabilityCheck control_availability(const Definition &d) {
  // Held sensor states isolate the PI/hardware boundary, not a flight model.
  // Zero slew is already an immobile runtime state in advance_surface.
  AvailabilityCheck check;
  constexpr std::array<double,3> ki{.8,1.2,.5};
  for (int hz : {120,240}) for (int axis=0; axis<3; ++axis)
    for (double sign : {-1.,1.}) for (int kind=0; kind<3; ++kind) {
      Simulation s(d,1./hz); hold_guidance(s);
      std::vector<std::size_t> branches;
      for (std::size_t i=0; i<s.def.chains.size(); ++i)
        if (s.def.chains[i].axis==axis) {
          branches.push_back(i);
          s.def.chains[i].slew = kind==2 ? 1e-9 : 0;
        }
      require(!branches.empty(),"availability bench needs an existing branch");
      if (kind==1) // One surviving branch is enough, even on a shared axis.
        s.def.chains[branches.back()].slew=d.chains[branches.back()].slew;
      double initial=sign*.2;
      s.instructor.rate_bias[axis]=initial;
      s.body.omega[axis]=-sign*.02;
      PilotIntent p; p.mouse=true;
      for (int tick=0; tick<hz; ++tick) s.update_controls(p);
      ++check.cases;
      if (kind==0) {
        ++check.immobile;
        check.frozen_drift=std::max(check.frozen_drift,
                                  std::abs(s.instructor.rate_bias[axis]-initial));
        for (auto id:branches)
          check.immobile_motion=std::max(check.immobile_motion,
              std::max(std::abs(s.def.chains[id].deflection),
                       std::abs(s.def.chains[id].midpoint_deflection)));
        require(s.instructor.integral_paused[axis] && check.frozen_drift==0,
                "immobile control chains must freeze Instructor PI bias");
        require(check.immobile_motion==0,"zero-slew surfaces remain immobile");
        p.active[axis]=true; p.direct[axis]=sign;
        s.update_controls(p);
        require(s.ownership[axis].command==sign && s.ownership[axis].direct_weight==1,
                "immobile hardware does not dilute keyboard ownership");
        p.active[axis]=false; s.update_controls(p);
        require(s.ownership[axis].command==sign,
                "release retains request continuity on immobile hardware");
        for (int tick=0; tick<=int(d.handoff*hz); ++tick) s.update_controls(p);
        require(s.ownership[axis].direct_weight==0 && s.instructor.rate_bias[axis]==initial,
                "unavailable PI stays frozen after handoff ends");
        // A bench counterfactual, not a gameplay repair feature: the next
        // available sample resumes the old bias with the ordinary increment.
        s.def.chains[branches.back()].slew=d.chains[branches.back()].slew;
        s.update_controls(p);
        require(!s.instructor.integral_paused[axis] &&
                    std::abs(s.def.chains[branches.back()].deflection)>0 &&
                    std::abs(s.instructor.rate_bias[axis]-(initial+sign*ki[axis]*.02/hz))<1e-14,
                "a surviving movable branch resumes PI without resetting bias");
      } else {
        double expected=initial+sign*ki[axis]*.02;
        check.moving_error=std::max(check.moving_error,
                                   std::abs(s.instructor.rate_bias[axis]-expected));
        require(!s.instructor.integral_paused[axis] && check.moving_error<1e-13,
                "one movable branch or strictly positive slew retains ordinary PI");
        require(std::abs(s.def.chains[branches.back()].deflection)>0,
                "positive slew has nonzero actual surface response");
      }
      for (int other=0; other<3; ++other) if (other!=axis)
        require(s.ownership[other].direct_weight==0 && s.instructor.rate_bias[other]==0,
                "availability and direct ownership remain per-axis");
    }
  return check;
}
struct AvailabilityTransitions {
  int cases = 0, resets = 0, resumed = 0;
  double bias_error = 0, frozen_motion = 0, slew_excess = 0, weight_error = 0;
};
AvailabilityTransitions availability_transitions(const Definition &d) {
  AvailabilityTransitions check;
  constexpr std::array<double,3> ki{.8,1.2,.5};
  for (int hz:{120,240}) for (int axis=0; axis<3; ++axis)
    for (double sign:{-1.,1.}) for (bool negative_zero:{false,true}) {
      Simulation s(d,1./hz); hold_guidance(s);
      s.body.omega={-.01,.015,-.012};
      s.body.omega[axis]=-sign*.02;
      s.instructor.rate_bias={.1,-.11,.12};
      s.instructor.rate_bias[axis]=sign*.2;
      std::vector<std::size_t> branches;
      for (std::size_t id=0; id<s.def.chains.size(); ++id)
        if (s.def.chains[id].axis==axis) {
          branches.push_back(id);
          s.def.chains[id].lag=.15; s.def.chains[id].friction=4;
        }
      require(!branches.empty(),"transition bench needs an existing branch");
      PilotIntent p; p.mouse=true;
      for (int tick=0; tick<hz/4; ++tick) s.update_controls(p);
      std::vector<double> held;
      for (auto id:branches) {
        held.push_back(s.def.chains[id].deflection);
        require(held.back()!=0 && s.def.chains[id].filtered!=0,
                "fault starts from actual nonzero actuator/filter history");
        s.def.chains[id].slew=negative_zero ? -0. : 0.;
      }
      // Change held error sign and pressure after immobilization. No surface
      // position or filter is assigned; only runtime hardware state changes.
      s.body.omega[axis]=sign*.02;
      s.body.velocity={140,0,0};
      auto restored=branches[negative_zero ? branches.size()-1 : 0];
      bool recovered=false, resumed=false;
      auto advance=[&] {
        auto bias=s.instructor.rate_bias;
        double old_weight=s.ownership[axis].direct_weight;
        std::vector<double> before;
        for (auto id:branches) before.push_back(s.def.chains[id].deflection);
        s.update_controls(p);
        require(norm(s.instructor.desired_rates)<1e-14,
                "held sensor bench keeps a known zero rate target");
        for (int a=0; a<3; ++a) {
          bool pause=a==axis && (!recovered || p.active[a] || old_weight>.01);
          double expected=bias[a]+(pause ? 0 : -ki[a]*s.body.omega[a]*s.dt);
          double error=std::abs(s.instructor.rate_bias[a]-expected);
          check.bias_error=std::max(check.bias_error,error);
          require(error<1e-14 && s.instructor.integral_paused[a]==pause,
                  "transition preserves exact PI pause policy and other-axis integration");
          if (a!=axis) require(s.ownership[a].direct_weight==0,
                               "fault/recovery never takes ownership of another axis");
          if (a==axis && !pause) resumed=true;
        }
        double q=.5*s.density*dot(s.body.velocity,s.body.velocity);
        for (std::size_t n=0; n<branches.size(); ++n) {
          auto id=branches[n]; const auto &c=s.def.chains[id];
          if (!recovered || id!=restored) {
            double motion=std::max(std::abs(c.deflection-held[n]),
                                   std::abs(c.midpoint_deflection-held[n]));
            check.frozen_motion=std::max(check.frozen_motion,motion);
            require(motion==0,"zero-slew branch retains its captured nonzero angle");
          } else {
            double rate=c.slew/(1+q/c.q_limit)/(1+c.friction);
            double excess=std::max({0.,std::abs(c.deflection-before[n])-rate*s.dt,
                                    std::abs(c.midpoint_deflection-before[n])-rate*s.dt/2});
            check.slew_excess=std::max(check.slew_excess,excess);
            require(excess<1e-14,"recovery cannot snap endpoint or midpoint beyond actual slew");
          }
        }
      };
      for (int tick=0; tick<hz/8; ++tick) advance();
      p.active[axis]=true; p.direct[axis]=-sign; advance();
      require(s.ownership[axis].command==-sign && s.ownership[axis].direct_weight==1,
              "nonzero-angle immobilization retains full direct request authority");
      p.active[axis]=false;
      int release_tick=0;
      auto release=[&] {
        advance();
        double t=std::min(1.,release_tick++*s.dt/d.handoff);
        double expected=1-t*t*(3-2*t);
        check.weight_error=std::max(check.weight_error,
                                   std::abs(s.ownership[axis].direct_weight-expected));
        require(check.weight_error<1e-14,"hardware transition preserves handoff timing");
      };
      release();
      require(s.ownership[axis].command==-sign,"first release sample is continuous");
      for (int tick=0; tick<int(d.handoff*hz/2); ++tick) release();
      require(s.ownership[axis].direct_weight>.01,"restore hardware inside active handoff");
      s.def.chains[restored].slew=d.chains[restored].slew; recovered=true;
      for (int tick=0; tick<=int(d.handoff*hz/2)+2; ++tick) release();
      require(resumed && s.ownership[axis].direct_weight==0,
              "PI resumes from retained state after the existing ownership gate permits it");
      require(s.def.chains[restored].deflection!=held[negative_zero ? held.size()-1 : 0],
              "restored branch has actual physical motion");
      ++check.cases; ++check.resumed;
      for (auto id:branches) s.def.chains[id].slew=0;
      auto bias=s.instructor.rate_bias;
      p.mouse=false; s.update_controls(p);
      require(s.instructor.rate_bias==bias,"inactive mouse preserves existing PI memory policy");
      p.mouse=true;
      for (int reason=0; reason<3; ++reason) {
        auto reset=s;
        if (reason==0) reset.instructor_mode=InstructorMode::Off;
        if (reason==1) reset.instructor_mode=InstructorMode::Legacy;
        if (reason==2) reset.guidance.rate_integral=false;
        reset.update_controls(p);
        require(reset.instructor.rate_bias==std::array<double,3>{},
                "mode/PI disable still clears stale biases despite zero-slew pause");
        reset.instructor_mode=InstructorMode::FlightPath; reset.guidance.rate_integral=true;
        reset.update_controls(p);
        for (int a=0; a<3; ++a) {
          double expected=a==axis ? 0 : -ki[a]*reset.body.omega[a]*reset.dt;
          require(std::abs(reset.instructor.rate_bias[a]-expected)<1e-14 &&
                      reset.instructor.integral_paused[a]==(a==axis),
                  "reactivation does not resurrect cleared bias or freeze healthy axes");
        }
        ++check.resets;
      }
    }
  return check;
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
    auto availability = control_availability(d);
    auto transitions = availability_transitions(d);
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
              << ",\"availability_cases\":" << availability.cases
              << ",\"availability_rates_hz\":[120,240]"
              << ",\"immobile_cases\":" << availability.immobile
              << ",\"max_immobile_bias_drift\":" << availability.frozen_drift
              << ",\"max_moving_bias_error\":" << availability.moving_error
              << ",\"max_immobile_surface_motion_rad\":" << availability.immobile_motion
              << ",\"availability_transition_cases\":" << transitions.cases
              << ",\"availability_state_reset_cases\":" << transitions.resets
              << ",\"availability_resumed_cases\":" << transitions.resumed
              << ",\"max_transition_bias_error\":" << transitions.bias_error
              << ",\"max_captured_angle_motion_rad\":" << transitions.frozen_motion
              << ",\"max_recovery_slew_excess_rad\":" << transitions.slew_excess
              << ",\"max_transition_handoff_weight_error\":" << transitions.weight_error
              << ",\"rates_hz\":[60,120,240,480]}\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
