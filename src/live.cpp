#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace flight;
namespace {
void vec(Vec v) { std::cout << '[' << v.x << ',' << v.y << ',' << v.z << ']'; }
void snapshot(const Simulation &s, const PilotIntent &p) {
  const char *mode = s.instructor_mode == InstructorMode::Off      ? "off"
                     : s.instructor_mode == InstructorMode::Legacy ? "legacy"
                     : s.guidance.rate_integral ? "flightpath"
                                                : "flightpath-p";
  std::cout << std::setprecision(12) << "{\"controller\":" << std::quoted(mode)
            << ",\"tick\":" << s.tick << ",\"dt\":" << s.dt << ",\"position\":";
  vec(s.body.position);
  std::cout << ",\"wind\":";
  vec(s.wind_world);
  std::cout << ",\"velocity\":";
  vec(s.body.velocity);
  std::cout << ",\"omega\":";
  vec(s.body.omega);
  auto q = s.body.attitude;
  std::cout << ",\"quaternion\":[" << q.w << ',' << q.x << ',' << q.y << ','
            << q.z << "],\"cg\":";
  vec(s.mass.cg);
  std::cout << ",\"mass\":" << s.mass.mass << ",\"g\":" << s.load_factor
            << ",\"rpm\":" << s.def.engine.rpm
            << ",\"thrust\":" << s.def.engine.thrust
            << ",\"throttle\":" << p.throttle << ",\"trim\":[" << p.trim[0]
            << ',' << p.trim[1] << ',' << p.trim[2] << "],\"desired_rates\":";
  vec(s.instructor.desired_rates);
  std::cout << ",\"desired_bank\":" << s.instructor.desired_bank
            << ",\"desired_normal_g\":" << s.instructor.desired_normal_g
            << ",\"path_error\":" << s.instructor.flight_path_error
            << ",\"aero_force\":";
  vec(s.aero.force);
  std::cout << ",\"prop_force\":";
  vec(s.propulsion.force);
  std::cout << ",\"rate_bias\":[" << s.instructor.rate_bias[0] << ','
            << s.instructor.rate_bias[1] << ',' << s.instructor.rate_bias[2]
            << ']';
  std::cout << ",\"propeller_model\":"
            << std::quoted(s.def.engine.model == PropellerModel::BladeElement
                               ? "blade_element"
                               : "legacy")
            << ",\"prop_pitch\":" << s.def.engine.pitch
            << ",\"target_rpm\":" << s.def.engine.target_rpm
            << ",\"governor\":" << (s.def.engine.governor ? "true" : "false")
            << ",\"governor_saturated\":"
            << (s.def.engine.governor_saturated ? "true" : "false")
            << ",\"pitch_jammed\":"
            << (s.def.engine.pitch_jammed ? "true" : "false")
            << ",\"shaft_power\":" << s.def.engine.shaft_power
            << ",\"tip_mach\":" << s.def.engine.tip_mach;
  std::cout << ",\"materials\":[";
  bool first_material = true;
  for (const auto &m : s.def.masses)
    if (!m.material_name.empty()) {
      if (!first_material)
        std::cout << ',';
      first_material = false;
      std::cout << "{\"name\":" << std::quoted(m.material_name)
                << ",\"node\":" << m.node << ",\"initial_mass\":" << m.mass
                << ",\"remaining_fraction\":" << m.remaining
                << ",\"remaining_mass\":" << m.mass * m.remaining << '}';
    }
  std::cout << ']';
  std::cout << ",\"ownership\":[";
  for (int i = 0; i < 3; ++i) {
    if (i)
      std::cout << ',';
    std::cout << "{\"command\":" << s.ownership[i].command
              << ",\"direct_weight\":" << s.ownership[i].direct_weight
              << ",\"saturated\":"
              << (s.instructor.saturated[i] ? "true" : "false")
              << ",\"integral_paused\":"
              << (s.instructor.integral_paused[i] ? "true" : "false") << '}';
  }
  std::cout << "],\"chains\":[";
  for (std::size_t i = 0; i < s.def.chains.size(); ++i) {
    auto &c = s.def.chains[i];
    if (i)
      std::cout << ',';
    std::cout << "{\"name\":" << std::quoted(c.name)
              << ",\"connected\":" << (c.connected ? "true" : "false")
              << ",\"deflection\":" << c.deflection << '}';
  }
  std::cout << "],\"cells\":[";
  for (std::size_t i = 0; i < s.def.cells.size(); ++i) {
    auto &c = s.def.cells[i];
    if (i)
      std::cout << ',';
    std::cout << "{\"id\":" << c.id << ",\"node\":" << c.node
              << ",\"plane\":" << c.plane << ",\"position\":";
    vec(c.position);
    std::cout << ",\"flow\":";
    vec(c.flow);
    std::cout << ",\"force\":";
    vec(c.force);
    std::cout << ",\"attached\":" << (c.attached ? "true" : "false")
              << ",\"area_fraction\":" << c.area_fraction
              << ",\"alpha\":" << c.alpha << ",\"q\":" << c.q
              << ",\"stall\":" << c.stall << '}';
  }
  std::cout << "],\"edges\":[";
  for (std::size_t i = 0; i < s.def.edges.size(); ++i) {
    auto &e = s.def.edges[i];
    if (i)
      std::cout << ',';
    std::cout << "{\"name\":" << std::quoted(e.name)
              << ",\"section\":" << e.remaining_section
              << ",\"stiffness_fraction\":" << e.stiffness()
              << ",\"bending_Nm\":" << e.bend_load
              << ",\"torsion_Nm\":" << e.twist_load
              << ",\"ultimate_utilization\":" << e.utilization
              << ",\"plastic_strain\":" << e.plastic_strain
              << ",\"failed\":" << (e.failed ? "true" : "false") << '}';
  }
  std::cout << "],\"debris\":" << s.debris.size() << ",\"detached\":[";
  for (std::size_t i = 0; i < s.debris.size(); ++i) {
    if (i)
      std::cout << ',';
    const auto &piece = s.debris[i];
    std::cout << "{\"material\":" << std::quoted(piece.material_name)
              << ",\"mass\":" << piece.mass.mass << ",\"cg\":";
    vec(piece.mass.cg);
    std::cout << ",\"position\":";
    vec(piece.state.position);
    std::cout << ",\"velocity\":";
    vec(piece.state.velocity);
    std::cout << ",\"omega\":";
    vec(piece.state.omega);
    auto q = piece.state.attitude;
    std::cout << ",\"quaternion\":[" << q.w << ',' << q.x << ',' << q.y << ','
              << q.z << "]";
    std::cout << ",\"aero_force\":";
    vec(piece.aero.force);
    std::cout << ",\"aero_moment\":";
    vec(piece.aero.moment);
    std::cout << ",\"chains\":[";
    for (std::size_t j = 0; j < piece.chains.size(); ++j) {
      if (j)
        std::cout << ',';
      const auto &c = piece.chains[j];
      std::cout << "{\"name\":" << std::quoted(c.name)
                << ",\"connected\":" << (c.connected ? "true" : "false")
                << ",\"jammed\":" << (c.jammed ? "true" : "false")
                << ",\"deflection\":" << c.deflection << '}';
    }
    std::cout << "],\"cells\":[";
    for (std::size_t j = 0; j < piece.aero_cells.size(); ++j) {
      if (j)
        std::cout << ',';
      const auto &c = piece.aero_cells[j].cell;
      std::cout << "{\"id\":" << c.id
                << ",\"area_fraction\":" << c.area_fraction << ",\"flow\":";
      vec(c.flow);
      std::cout << ",\"force\":";
      vec(c.force);
      std::cout << ",\"alpha\":" << c.alpha << ",\"q\":" << c.q
                << ",\"deflection\":" << c.deflection << '}';
    }
    std::cout << "]}";
  }
  std::cout << "]}\n" << std::flush;
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("usage: flight-live AIRFRAME");
    auto fixture = Definition::load(argv[1]);
    Simulation s(fixture);
    auto tr = s.trim(70, 1000);
    PilotIntent p = tr.intent;
    p.mouse = true;
    snapshot(s, p);
    std::string line;
    while (std::getline(std::cin, line)) {
      try {
        std::istringstream input(line);
        std::string op;
        input >> op;
        if (op == "quit")
          break;
        if (op == "reset") {
          bool governed = s.def.engine.governor;
          auto mode = s.instructor_mode;
          auto parameters = s.guidance;
          s = Simulation(fixture);
          s.def.engine.governor = governed;
          s.instructor_mode = mode;
          s.guidance = parameters;
          tr = s.trim(70, 1000);
          p = tr.intent;
          p.mouse = true;
        } else if (op == "governor") {
          if (s.def.engine.model != PropellerModel::BladeElement)
            throw std::runtime_error("legacy reference has no pitch governor");
          std::string mode;
          input >> mode;
          if (mode == "on")
            s.def.engine.governor = true;
          else if (mode == "off")
            s.def.engine.governor = false;
          else
            throw std::runtime_error("invalid governor mode");
        } else if (op == "controller") {
          std::string mode;
          input >> mode;
          if (mode == "off")
            s.instructor_mode = InstructorMode::Off;
          else if (mode == "legacy")
            s.instructor_mode = InstructorMode::Legacy;
          else if (mode == "flightpath" || mode == "flightpath-p") {
            s.instructor_mode = InstructorMode::FlightPath;
            s.guidance.rate_integral = mode == "flightpath";
          } else
            throw std::runtime_error("unknown controller");
          s.instructor = {};
        } else if (op == "step") {
          int steps, mouse, r, pitch, yaw;
          PilotIntent next;
          input >> steps >> next.aim_world.x >> next.aim_world.y >>
              next.aim_world.z >> mouse >> next.direct[0] >> next.direct[1] >>
              next.direct[2] >> r >> pitch >> yaw >> next.throttle >>
              next.trim[0] >> next.trim[1] >> next.trim[2];
          if (!input || steps < 0 || steps > 120 ||
              !std::isfinite(norm(next.aim_world)) ||
              norm(next.aim_world) < 1e-9)
            throw std::runtime_error("invalid step input");
          for (double v : next.direct)
            if (!std::isfinite(v))
              throw std::runtime_error("non-finite direct input");
          for (double v : next.trim)
            if (!std::isfinite(v))
              throw std::runtime_error("non-finite trim");
          if (!std::isfinite(next.throttle))
            throw std::runtime_error("non-finite throttle");
          input >> std::ws;
          if (!input.eof()) {
            input >> next.prop_rpm_ratio;
            if (!input || !std::isfinite(next.prop_rpm_ratio))
              throw std::runtime_error("invalid prop RPM ratio");
          }
          next.mouse = mouse;
          next.active = {bool(r), bool(pitch), bool(yaw)};
          p = next;
          for (int i = 0; i < steps; ++i)
            s.step(p);
        } else if (op == "damage") {
          std::string target, name;
          input >> target;
          if (target == "governor") {
            s.def.engine.pitch_jammed = true;
          } else if (target == "cell") {
            int id;
            double area, roughness;
            input >> id >> area >> roughness;
            if (!input || !std::isfinite(area) || !std::isfinite(roughness))
              throw std::runtime_error("invalid cell damage");
            s.damage_cell(id, area, roughness);
          } else if (target == "material") {
            double remaining;
            input >> name >> remaining;
            if (!input)
              throw std::runtime_error("invalid material event");
            s.remove_material(name, remaining);
          } else if (target == "spar") {
            double section;
            input >> name >> section;
            if (!input || !std::isfinite(section))
              throw std::runtime_error("invalid section");
            s.weaken(name, section);
          } else if (target == "link") {
            input >> name;
            if (!input)
              throw std::runtime_error("missing link");
            s.disconnect(name);
          } else if (target == "engine" || target == "prop") {
            double fraction;
            input >> fraction;
            if (!input || !std::isfinite(fraction) || fraction < 0 ||
                fraction > 1)
              throw std::runtime_error("invalid propulsion fraction");
            if (target == "engine")
              s.def.engine.power_fraction = fraction;
            else
              s.def.engine.prop_fraction = fraction;
          } else
            throw std::runtime_error("unknown damage target");
          s.evaluate(s.body, true);
        } else
          throw std::runtime_error("unknown command");
        snapshot(s, p);
      } catch (const std::exception &e) {
        std::cout << "{\"error\":" << std::quoted(std::string(e.what()))
                  << "}\n"
                  << std::flush;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
