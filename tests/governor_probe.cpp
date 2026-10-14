#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
using namespace flight;
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("BEM fixture required");
    auto d = Definition::load(argv[1]);
    std::cout << std::setprecision(13)
              << "case,hz,time,speed,rpm,target,pitch,thrust,motor_torque,aero_"
                 "torque,shaft_power,loss,induced,saturated\n";
    for (double hz : {120., 240.})
      for (std::string scenario :
           {"fixed", "governed", "windmill", "jammed", "limited"}) {
        auto e = d.engine;
        e.governor = scenario != "fixed";
        e.running = scenario != "windmill";
        e.pitch_jammed = scenario == "jammed";
        if (scenario == "limited")
          e.pitch_range = .12;
        if (scenario == "windmill")
          e.rpm = 0;
        for (int i = 0; i < int(40 * hz); ++i) {
          double t = i / hz, v = t < 15 ? 70 : 110;
          if (scenario == "windmill")
            v = 80;
          double throttle = t < 25 ? .75 : .95;
          advance_propulsion(e, 1.1, 1000, v, throttle, 1 / hz);
          if (i % int(hz / 10) == 0 || i == int(40 * hz) - 1)
            std::cout << scenario << ',' << hz << ',' << (i + 1) / hz << ','
                      << v << ',' << e.rpm << ',' << e.target_rpm << ','
                      << e.pitch << ',' << e.thrust << ',' << e.torque << ','
                      << e.aero_torque << ',' << e.shaft_power << ','
                      << e.profile_loss + e.induced_loss << ','
                      << e.induced_velocity << ',' << e.governor_saturated
                      << '\n';
        }
      }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
