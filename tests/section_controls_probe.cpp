#include "flight/core.hpp"
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
    std::vector<double> commands{-.2, 0, .2};
    if (argc > 2) {
      commands.clear();
      for (int i = 2; i < argc; ++i)
        commands.push_back(std::stod(argv[i]));
    }
    std::cout << std::setprecision(17)
              << "input,left_rad,right_rad,q_Pa,roll_ownership\n";
    for (double command : commands) {
      if (!std::isfinite(command) || std::abs(command) > 1)
        throw std::runtime_error("normalized command outside [-1,1]");
      Simulation s(definition);
      s.instructor_mode = InstructorMode::Off;
      s.density = 1.225 * std::exp(-1000. / 8500);
      PilotIntent p;
      p.active[Roll] = true;
      p.direct[Roll] = command;
      for (int i = 0; i < 120; ++i)
        s.update_controls(p);
      auto previous = s.def.chains;
      for (int i = 0; i < 120; ++i)
        s.update_controls(p);
      double left = 0, right = 0;
      bool found_left = false, found_right = false;
      for (std::size_t i = 0; i < s.def.chains.size(); ++i) {
        const auto &c = s.def.chains[i];
        if (std::abs(c.deflection - previous[i].deflection) > 1e-12)
          throw std::runtime_error("control bench not settled by one second");
        if (c.name == "left_aileron") {
          left = c.deflection;
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
                << s.ownership[Roll].direct_weight << '\n';
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
