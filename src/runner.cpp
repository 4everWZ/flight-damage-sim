#include "flight/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace flight;
namespace {
void row(std::ostream &out, const Simulation &s) {
  auto vb = s.body.attitude.conjugate().rotate(s.body.velocity);
  Vec forward = s.body.attitude.rotate({1, 0, 0});
  double max_u = 0;
  int failed = 0, stalled = 0;
  for (auto &e : s.def.edges) {
    max_u = std::max(max_u, e.utilization);
    failed += e.failed;
  }
  for (auto &c : s.def.cells)
    stalled += c.stall > 0;
  out << s.tick * s.dt << ',' << s.body.position.x << ',' << s.body.position.y
      << ',' << -s.body.position.z << ',' << norm(s.body.velocity) << ','
      << std::atan2(vb.z, vb.x) << ',' << std::atan2(forward.y, forward.x)
      << ',' << s.body.omega.x << ',' << s.body.omega.y << ',' << s.body.omega.z
      << ',' << s.load_factor;
  for (auto &o : s.ownership)
    out << ',' << o.command << ',' << o.direct_weight;
  for (auto &c : s.def.chains)
    out << ',' << c.deflection;
  out << ',' << s.def.engine.rpm << ',' << s.def.engine.thrust << ','
      << s.aero.force.z << ',' << s.aero.moment.x << ',' << max_u << ','
      << failed << ',' << s.debris.size() << ',' << s.mass.mass << ','
      << stalled;
  Vec air_world = s.body.velocity - s.wind_world;
  Vec down = s.body.attitude.conjugate().rotate({0, 0, 1});
  out << ',' << std::atan2(-air_world.z, std::hypot(air_world.x, air_world.y))
      << ',' << std::atan2(down.y, down.z) << ',' << s.instructor.desired_bank
      << ',' << s.instructor.desired_normal_g << ','
      << s.instructor.flight_path_error;
  for (double rate :
       {s.instructor.desired_rates.x, s.instructor.desired_rates.y,
        s.instructor.desired_rates.z})
    out << ',' << rate;
  for (bool sat : s.instructor.saturated)
    out << ',' << sat;
  out << ',' << std::atan2(air_world.y, air_world.x) << ','
      << std::atan2(vb.y, std::hypot(vb.x, vb.z));
  for (double bias : s.instructor.rate_bias)
    out << ',' << bias;
  for (bool paused : s.instructor.integral_paused)
    out << ',' << paused;
  out << '\n';
}
void debug(const Simulation &s, const std::string &prefix) {
  std::ofstream c(prefix + ".cells.csv");
  if (!c)
    throw std::runtime_error("cannot write cell telemetry");
  c << "id,node,attached,area_fraction,alpha,q,stall,fx,fy,fz,deflection\n";
  for (auto &a : s.def.cells)
    c << a.id << ',' << a.node << ',' << a.attached << ',' << a.area_fraction
      << ',' << a.alpha << ',' << a.q << ',' << a.stall << ',' << a.force.x
      << ',' << a.force.y << ',' << a.force.z << ',' << a.deflection << '\n';
  std::ofstream e(prefix + ".edges.csv");
  if (!e)
    throw std::runtime_error("cannot write structural telemetry");
  e << "name,section,stiffness,bending,torsion,utilization,plastic_strain,"
       "failed\n";
  for (auto &a : s.def.edges)
    e << a.name << ',' << a.remaining_section << ',' << a.stiffness() << ','
      << a.bend_load << ',' << a.twist_load << ',' << a.utilization << ','
      << a.plastic_strain << ',' << a.failed << '\n';
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc < 4) {
      std::cerr
          << "usage: flight-run AIRFRAME SCENARIO OUTPUT_PREFIX [hz=120] "
             "[seconds=10] [controller=flightpath|flightpath-p|legacy|off] "
             "[bank_deg=40]\n";
      return 2;
    }
    std::string scenario = argv[2], prefix = argv[3];
    double hz = argc > 4 ? std::stod(argv[4]) : 120,
           seconds = argc > 5 ? std::stod(argv[5]) : 10;
    const std::vector<std::string> names{"clean",
                                         "roll",
                                         "pull",
                                         "mouse",
                                         "mixed",
                                         "aero_damage",
                                         "symmetric_damage",
                                         "control_damage",
                                         "compensate",
                                         "structure",
                                         "weakened_pull",
                                         "highspeed_pull",
                                         "highspeed_weakened",
                                         "stall",
                                         "mouse_damaged",
                                         "mouse_disconnected",
                                         "mouse_climb",
                                         "mouse_circle",
                                         "mouse_circle_damaged"};
    if (std::find(names.begin(), names.end(), scenario) == names.end() ||
        hz <= 0 || seconds <= 0 || !std::isfinite(hz) ||
        !std::isfinite(seconds) || seconds * hz > 10000000)
      throw std::runtime_error("invalid scenario or duration");
    Simulation s(Definition::load(argv[1]), 1 / hz);
    std::string controller = argc > 6 ? argv[6] : "flightpath";
    if (controller == "legacy")
      s.instructor_mode = InstructorMode::Legacy;
    else if (controller == "off")
      s.instructor_mode = InstructorMode::Off;
    else if (controller == "flightpath-p")
      s.guidance.rate_integral = false;
    else if (controller != "flightpath")
      throw std::runtime_error("unknown controller");
    if (argc > 7) {
      double bank = std::stod(argv[7]);
      if (!std::isfinite(bank) || bank < 5 || bank > 75)
        throw std::runtime_error("invalid bank limit");
      s.guidance.max_bank = rad(bank);
    }
    auto trim = s.trim(
        scenario == "highspeed_pull" || scenario == "highspeed_weakened" ? 140
                                                                         : 70,
        1000);
    PilotIntent p = trim.intent;
    std::cout << std::setprecision(12) << "trim_residual=" << trim.residual
              << " alpha=" << trim.alpha << " throttle=" << p.throttle
              << " roll_trim=" << p.trim[0] << " pitch_trim=" << p.trim[1]
              << " yaw_trim=" << p.trim[2]
              << " force_residual_N=" << norm(trim.force_residual_N)
              << " moment_residual_Nm=" << norm(trim.moment_residual_Nm)
              << " rotor_accel=" << trim.rotor_acceleration
              << " converged=" << trim.converged << "\n";
    if (scenario == "structure") {
      s.weaken("left_root_front", .30);
      std::vector<double> b(s.def.nodes.size()), t(b.size());
      std::ofstream bench(prefix + ".csv");
      if (!bench)
        throw std::runtime_error("cannot write bench telemetry");
      bench << "time,load_g,front,rear,skin,failed,debris,mass\n";
      for (int i = 0; i < int(seconds * hz); ++i) {
        double time = i / hz, load = time < 3 ? 1 : 5;
        std::fill(b.begin(), b.end(), 0);
        b[1] =
            -s.mass.mass * gravity / 2 * 2.4 *
            load; // Independent imposed-load bench, not an aircraft maneuver.
        s.solve_structure(b, t, 1 / hz);
        int fail = 0;
        for (auto &e : s.def.edges)
          fail += e.failed;
        bench << time << ',' << load << ',' << s.def.edges[0].bend_load << ','
              << s.def.edges[1].bend_load << ',' << s.def.edges[2].bend_load
              << ',' << fail << ',' << s.debris.size() << ',' << s.mass.mass
              << '\n';
      }
      debug(s, prefix);
      return 0;
    }
    if (trim.residual > 1e-6)
      std::cout << "entry_equilibrium=false (high-speed transient; fixed-pitch "
                   "propulsion cannot sustain this speed)\n";
    std::ofstream out(prefix + ".csv");
    if (!out)
      throw std::runtime_error("cannot write flight telemetry");
    out << std::setprecision(15)
        << "time,x,y,altitude,speed,alpha,heading,p,q,r,g,roll_cmd,roll_owner,"
           "pitch_cmd,pitch_owner,yaw_cmd,yaw_owner,left_aileron,right_aileron,"
           "elevator,rudder,rpm,thrust,lift_z,roll_moment,max_utilization,"
           "failed,debris,mass,stalled,path_angle,bank,desired_bank,desired_g,"
           "path_error,desired_p,desired_q,desired_r,roll_saturated,pitch_"
           "saturated,yaw_saturated,air_heading,beta,roll_bias,pitch_bias,yaw_"
           "bias,roll_integral_paused,pitch_integral_paused,yaw_integral_"
           "paused\n";
    bool damaged = false;
    p.mouse = scenario == "mouse" || scenario == "mixed" ||
              scenario == "compensate" || scenario == "weakened_pull" ||
              scenario == "mouse_damaged" || scenario == "mouse_disconnected" ||
              scenario == "mouse_climb" || scenario == "mouse_circle" ||
              scenario == "mouse_circle_damaged";
    if (scenario == "stall") {
      s.body.velocity = {40, 0, 0};
      s.evaluate(s.body, true);
    }
    row(out, s);
    for (int i = 0; i < int(seconds * hz); ++i) {
      double time = i / hz;
      if ((scenario == "aero_damage" || scenario == "symmetric_damage" ||
           scenario == "compensate") &&
          !damaged && time >= 1) {
        int count = 0;
        for (auto &c : s.def.cells)
          if (c.node == 2 && count++ < 8)
            s.damage_cell(c.id, 0, 1);
        if (scenario == "symmetric_damage") {
          count = 0;
          for (auto &c : s.def.cells)
            if (c.node == 4 && count++ < 8)
              s.damage_cell(c.id, 0, 1);
        }
        damaged = true;
      }
      if (scenario == "control_damage" && !damaged && time >= 1) {
        s.disconnect("left_aileron");
        damaged = true;
      }
      if ((scenario == "weakened_pull" || scenario == "highspeed_weakened") &&
          !damaged && time >= 1) {
        s.weaken("left_root_front", .3);
        damaged = true;
      }
      if (scenario == "roll" || scenario == "control_damage") {
        p.active[Roll] = time >= 1 && time < 2;
        p.direct[Roll] = 1;
      }
      if (scenario == "pull" || scenario == "weakened_pull" ||
          scenario == "highspeed_pull" || scenario == "highspeed_weakened") {
        p.active[Pitch] = time >= 3 && time < 5;
        p.direct[Pitch] = 1;
      }
      if (scenario == "stall") {
        p.throttle = 1;
        p.active[Pitch] = time >= 1 && time < 8;
        p.direct[Pitch] = time < 4 ? 1 : -1;
      }
      if (time >= 1 && !damaged &&
          (scenario == "mouse_damaged" || scenario == "mouse_circle_damaged")) {
        int count = 0;
        for (auto &c : s.def.cells)
          if (c.node == 2 && count++ < 8)
            s.damage_cell(c.id, 0, 1);
        damaged = true;
      }
      if (time >= 1 && !damaged && scenario == "mouse_disconnected") {
        s.disconnect("elevator");
        damaged = true;
      }
      if (scenario == "mouse_circle" || scenario == "mouse_circle_damaged")
        p.aim_world = {std::cos(.08 * time), std::sin(.08 * time), 0};
      if (scenario == "mouse_climb")
        p.aim_world = {std::cos(rad(8)), 0, -std::sin(rad(8))};
      if (scenario == "mouse" || scenario == "mouse_damaged" ||
          scenario == "mouse_disconnected")
        p.aim_world = {0, 1, 0};
      if (scenario == "mixed") {
        p.aim_world = {1, .12, -.02};
        p.active[Roll] = time >= 1 && time < 2;
        p.direct[Roll] = -1;
      }
      s.step(p);
      row(out, s);
      if (!std::isfinite(norm(s.body.velocity)) ||
          !std::isfinite(norm(s.body.omega)))
        throw std::runtime_error("non-finite flight state");
    }
    debug(s, prefix);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
