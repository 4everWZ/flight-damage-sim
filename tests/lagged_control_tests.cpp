#include "flight/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace flight;
namespace {
void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
// Independent closed-form reversal bench. The positive step stays on the
// slew bound until reversal; after catching the falling exponential, the
// surface reverses at the same finite speed. Neither leg settles by 0.8 s.
struct Reversal {
  double rate, amplitude, lag = .15, event = .2, end = .8, turn;
  Reversal(double r, double a) : rate(r), amplitude(a) {
    double filtered = amplitude * (-std::expm1(-event / lag));
    double lo = 0, hi = .2;
    for (int i = 0; i < 60; ++i) {
      double t = (lo + hi) * .5;
      double target = -amplitude + (filtered + amplitude) * std::exp(-t / lag);
      if (target > rate * (event + t))
        lo = t;
      else
        hi = t;
    }
    turn = event + (lo + hi) * .5;
    require(filtered > rate * event, "first leg must still be slew limited");
    require(-amplitude + (filtered + amplitude) * std::exp(-(end-event)/lag)
                < angle(end), "second leg must still be slew limited");
  }
  double angle(double t) const {
    return rate * (t <= turn ? t : 2 * turn - t);
  }
  double integral() const {
    double after = end - turn;
    return .5 * rate * turn * turn + rate * turn * after - .5 * rate * after * after;
  }
};
}
int main(int argc, char **argv) {
  try {
    if (argc < 2 || argc > 3)
      throw std::runtime_error("airframe required [summary.json]");
    auto original = Definition::load(argv[1]);
    double max_angle = 0, max_mid = 0, max_moment = 0, max_impulse = 0,
           max_composition = 0;
    bool corner_bounds_passed = true;
    int cases = 0;
    double example_gain = 0, example_integral = 0, example_exact = 0;
    for (double speed : {40., 70., 110.})
      for (double sign : {-1., 1.})
        for (bool damaged : {false, true})
          for (double hz : {120., 240., 480.}) {
            Simulation s(original, 1/hz);
            s.instructor_mode = InstructorMode::Off;
            s.body.position = {0, 0, -1000};
            s.body.velocity = {speed, 0, 0};
            s.density = 1.225 * std::exp(-1000./8500);
            s.def.engine.model = PropellerModel::Legacy;
            s.def.engine.prop_fraction = s.def.engine.thrust = s.def.engine.torque = 0;
            // Declared synthetic linear polar isolates the actuator -> actual
            // cell wrench boundary; no production table/default is changed.
            for (auto &table : s.def.curves)
              for (auto &point : table.second) {
                point.cl = 5.7 * point.alpha;
                point.cd = .02;
                point.cm = 0;
              }
            auto &chain = s.def.chains[0];
            require(chain.name == "left_aileron", "fixture left control chain");
            chain.lag = .15;
            chain.friction = 4;
            chain.range = .6;
            chain.hinge = damaged ? .7 : 1;
            int removed = 0;
            for (auto &cell : s.def.cells) {
              if (cell.chain != 0)
                cell.control_gain = 0; // Only the measured branch varies.
              if (damaged && cell.node == 2 && removed < 8) {
                s.damage_cell(cell.id, 0, 1);
                ++removed;
              }
            }
            require(!damaged || removed == 8, "declared local cell loss");
            double q = .5 * s.density * speed * speed;
            double effort = 1 / (1 + q / chain.q_limit);
            Reversal reference(chain.slew * effort / (1 + chain.friction),
                               chain.limit * chain.range * effort);
            double gain = 0, compression = 1 / std::sqrt(1-speed*speed/(340*340));
            for (const auto &cell : s.def.cells)
              if (cell.chain == 0)
                gain -= (cell.position.y-s.mass.cg.y) * q * cell.area *
                        cell.area_fraction * cell.lift_efficiency * 5.7 *
                        cell.control_gain * cell.control_sign * chain.hinge * compression;
            s.evaluate(s.body, true);
            double neutral = s.aero.moment.x, impulse = 0;
            PilotIntent p;
            p.active[Roll] = true;
            for (int i = 0; i < int(reference.end*hz); ++i) {
              p.direct[Roll] = sign * (i < int(reference.event*hz) ? 1 : -1);
              double previous = chain.deflection;
              s.update_controls(p);
              require(s.ownership[Roll].command == p.direct[Roll] &&
                          s.ownership[Pitch].direct_weight == 0 &&
                          s.ownership[Yaw].direct_weight == 0,
                      "damage cannot dilute direct axis authority");
              require(std::abs(chain.deflection-previous) <= reference.rate/hz+1e-13,
                      "actual surface respects pressure/friction slew bound");
              double end_error = std::abs(chain.deflection-sign*reference.angle((i+1)/hz));
              double mid_error = std::abs(chain.midpoint_deflection-sign*reference.angle((i+.5)/hz));
              max_angle = std::max(max_angle, end_error);
              max_mid = std::max(max_mid, mid_error);
              // Evaluate the authoritative cell law at the already-generated
              // physical midpoint sample, in a copy of the held-body bench.
              auto sample = s;
              sample.def.chains[0].deflection = chain.midpoint_deflection;
              sample.evaluate(sample.body, true);
              double delta = sample.aero.moment.x-neutral;
              max_moment = std::max(max_moment,
                  std::abs(delta-gain*sign*reference.angle((i+.5)/hz)));
              impulse += delta/hz;
            }
            double error = std::abs(impulse-gain*sign*reference.integral());
            max_impulse = std::max(max_impulse, error);
            // Composite midpoint error for one corner with slope jump 2*rate.
            if (speed == 70 && sign == 1 && damaged && hz == 120) {
              example_gain = gain;
              example_integral = impulse;
              example_exact = gain*reference.integral();
            }
            corner_bounds_passed &= error <= std::abs(gain)*reference.rate/(4*hz*hz)+1e-7;
            ++cases;
          }
    // Smooth tracking, reversal/disconnect from a settled state, no artificial
    // motion with a jam or zero available slew, plus a changing pressure.
    for (double sign : {-1., 1.}) {
      Simulation s(original);
      auto &c = s.def.chains[0];
      c.lag = .2;
      c.slew = rad(10000);
      PilotIntent p;
      p.active[Roll] = true;
      p.direct[Roll] = sign*.2;
      double q = .5*s.density*70*70, effort = 1/(1+q/c.q_limit);
      for (int i=0; i<120; ++i) s.update_controls(p);
      require(std::abs(c.deflection-sign*.2*c.limit*effort*(-std::expm1(-1/.2)))<1e-13,
              "unlimited-rate lag follows its exponential exactly");
      double before=c.deflection;
      s.disconnect(c.name);
      require(c.deflection==before, "disconnect cannot teleport the surface");
      s.update_controls(p);
      require(std::abs(c.deflection-before*std::exp(-s.dt/c.lag))<1e-13,
              "existing neutral-seeking disconnect retains its lag");
      c.jammed=true; c.jam_position=c.deflection;
      s.update_controls(p);
      require(c.deflection==c.jam_position, "jam retains its angle");
      c.jammed=false; c.slew=0;
      before=c.deflection;
      s.body.velocity={110,0,0};
      s.update_controls(p);
      require(c.deflection==before, "zero slew cannot move when pressure changes");
    }
    // A held command/pressure defines one continuous mechanical trajectory.
    // Splitting the same interval must not change its endpoint, including a
    // catch, reversal and subsequent smooth tracking inside that interval.
    for (double position : {-.15, 0., .15})
      for (double filtered : {-.2, 0., .2})
        for (double command : {-1., 1.}) {
          Simulation fine(original);
          auto &c=fine.def.chains[0];
          c.lag=.15; c.friction=4; c.range=.6;
          c.deflection=position; c.filtered=filtered;
          auto coarse=fine;
          coarse.dt=.8;
          PilotIntent p;
          p.active[Roll]=true; p.direct[Roll]=command;
          coarse.update_controls(p);
          for (int i=0;i<96;++i) fine.update_controls(p);
          max_composition=std::max({max_composition,
              std::abs(c.deflection-coarse.def.chains[0].deflection),
              std::abs(c.filtered-coarse.def.chains[0].filtered)});
        }
    // Unmodified production polar/structure/propeller in actual 6DoF flight.
    // These are short response measurements, not a historical handling or a
    // universal convergence requirement. No body/surface is prescribed after
    // the common trimmed initial condition.
    std::array<std::array<Body, 3>, 3> flights;
    for (int condition=0; condition<3; ++condition)
      for (int sample=0; sample<3; ++sample) {
        double hz = 120 * (1 << sample);
        Simulation s(original, 1/hz);
        auto trim = s.trim(70, 1000);
        require(trim.converged, "flight comparison requires genuine trim");
        auto p = trim.intent;
        s.instructor_mode = InstructorMode::Off;
        p.active[Roll] = true;
        if (condition > 0) {
          auto &chain = s.def.chains[0];
          chain.lag=.15; chain.friction=4; chain.range=.6; chain.hinge=.7;
        }
        if (condition == 2) {
          int removed=0;
          for (const auto &cell:s.def.cells)
            if (cell.node==2 && removed++<8) s.damage_cell(cell.id,0,1);
        }
        for (int i=0; i<int(.8*hz); ++i) {
          p.direct[Roll] = i<int(.2*hz) ? 1 : -1;
          s.step(p);
          require(s.ownership[Roll].command==p.direct[Roll] &&
                      s.ownership[Roll].direct_weight==1,
                  "damaged flight retains full direct roll ownership");
          for (const auto &cell:s.def.cells)
            if (cell.attached && cell.chain>=0) {
              const auto &c=s.def.chains[cell.chain];
              require(std::abs(cell.deflection-c.midpoint_deflection*c.hinge*cell.control_sign)<1e-13,
                      "production flight consumes actual mechanical midpoint");
            }
        }
        require(std::isfinite(norm(s.body.velocity)) && s.debris.empty(),
                "short damaged response stays finite and structurally attached");
        flights[condition][sample]=s.body;
      }
    bool passed = max_angle < 2e-12 && max_mid < 2e-12 && max_moment < 1e-7 &&
                  max_composition < 2e-12 && corner_bounds_passed;
    std::ostringstream report;
    report << std::setprecision(15)
              << "{\"passed\":" << (passed ? "true" : "false") << ",\"reversal_cases\":" << cases
              << ",\"max_endpoint_error_rad\":" << max_angle
              << ",\"max_midpoint_error_rad\":" << max_mid
              << ",\"max_cell_moment_error_Nm\":" << max_moment
              << ",\"max_composition_error_rad\":" << max_composition
              << ",\"max_midpoint_impulse_error_Nms\":" << max_impulse
              << ",\"damaged_70ms_gain_Nm_per_rad\":" << example_gain
              << ",\"damaged_70ms_120hz_impulse_Nms\":" << example_integral
              << ",\"damaged_70ms_exact_impulse_Nms\":" << example_exact
              << ",\"flights\":[";
    for (int condition=0; condition<3; ++condition) {
      if (condition) report << ',';
      report << "{\"condition\":\"" << (condition==0 ? "intact" : condition==1 ? "linkage_damage" : "linkage_and_cell_loss")
             << "\",\"roll_rate_120hz_rad_s\":" << flights[condition][0].omega.x
             << ",\"position_difference_120_480_m\":" << norm(flights[condition][0].position-flights[condition][2].position)
             << ",\"position_difference_240_480_m\":" << norm(flights[condition][1].position-flights[condition][2].position) << '}';
    }
    report << "]}\n";
    std::cout << report.str();
    if (argc == 3) {
      std::ofstream file(argv[2]);
      if (!(file << report.str()))
        throw std::runtime_error("cannot write summary");
    }
    require(passed, "lag/slew reversal and cell moment must match the analytic trajectory");
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
