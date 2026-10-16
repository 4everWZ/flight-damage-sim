#include "flight/core.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;

// Static hardware bench only. Use the real arbitration/actuator update; no
// flight/aero step or writes to actual surface state. Existing aero tables are
// loaded but never used for force evaluation or interpreted as section data.
int main(int argc, char **argv) {
  try {
    if (argc < 2)
      throw std::runtime_error("airframe required, followed by optional inputs");
    auto definition = Definition::load(argv[1]);
    std::string fault = "none";
    int first_command = 2;
    if (argc > 2 && std::string(argv[2]).starts_with("--fault=")) {
      fault = std::string(argv[2]).substr(8);
      first_command = 3;
      if (fault != "left_disconnected" && fault != "left_jammed")
        throw std::runtime_error("unknown control-bench fault");
    }
    std::vector<double> commands{-.2, 0, .2};
    if (argc > first_command) {
      commands.clear();
      for (int i = first_command; i < argc; ++i)
        commands.push_back(std::stod(argv[i]));
    }
    std::cout << std::setprecision(17)
              << "input,left_rad,right_rad,q_Pa,roll_ownership,"
                 "pre_failure_left_rad,immediate_left_rad,left_connected,"
                 "left_jammed,jam_position_rad\n";
    for (double command : commands) {
      if (!std::isfinite(command) || std::abs(command) > 1)
        throw std::runtime_error("normalized command outside [-1,1]");
      Simulation s(definition);
      s.instructor_mode = InstructorMode::Off;
      s.density = 1.225 * std::exp(-1000. / 8500);
      PilotIntent p;
      p.active[Roll] = true;
      double before_fault = 0, immediate = 0;
      if (fault != "none") {
        // Same physical prehistory for both faults. Disconnect must not be
        // replaced by an instantaneous surface-zero write; jam captures the
        // actual angle rather than teleporting to a prescribed position.
        p.direct[Roll] = .2;
        for (int i = 0; i < 120; ++i)
          s.update_controls(p);
        auto left = std::find_if(s.def.chains.begin(), s.def.chains.end(),
                                 [](const auto &c) { return c.name == "left_aileron"; });
        if (left == s.def.chains.end())
          throw std::runtime_error("missing left aileron");
        before_fault = left->deflection;
        if (fault == "left_disconnected")
          s.disconnect("left_aileron");
        else {
          left->jam_position = left->deflection;
          left->jammed = true;
        }
        immediate = left->deflection;
        if (immediate != before_fault)
          throw std::runtime_error("fault event moved surface instantaneously");
      }
      p.direct[Roll] = command;
      for (int i = 0; i < 120; ++i)
        s.update_controls(p);
      auto previous = s.def.chains;
      for (int i = 0; i < 120; ++i)
        s.update_controls(p);
      double left = 0, right = 0;
      bool connected = true, jammed = false;
      double jam_position = 0;
      bool found_left = false, found_right = false;
      for (std::size_t i = 0; i < s.def.chains.size(); ++i) {
        const auto &c = s.def.chains[i];
        if (std::abs(c.deflection - previous[i].deflection) > 1e-12)
          throw std::runtime_error("control bench not settled by one second");
        if (c.name == "left_aileron") {
          left = c.deflection;
          connected = c.connected;
          jammed = c.jammed;
          jam_position = c.jam_position;
          found_left = true;
        }
        if (c.name == "right_aileron") {
          right = c.deflection;
          found_right = true;
        }
      }
      if (!found_left || !found_right || s.ownership[Roll].direct_weight != 1 ||
          s.ownership[Pitch].direct_weight != 0 ||
          s.ownership[Yaw].direct_weight != 0)
        throw std::runtime_error("control bench axis/hardware contract");
      std::cout << command << ',' << left << ',' << right << ','
                << .5 * s.density * 70 * 70 << ','
                << s.ownership[Roll].direct_weight << ',' << before_fault << ','
                << immediate << ',' << connected << ',' << jammed << ','
                << jam_position << '\n';
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
