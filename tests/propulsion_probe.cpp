#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
#include <string>
using namespace flight;
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe required");
    auto fixture = Definition::load(argv[1]);
    double altitude = 1000, rho = 1.225 * std::exp(-altitude / 8500);
    std::cout << std::setprecision(14)
              << "case,hz,speed,rpm,thrust,shaft_power,useful_power,prop_aero_"
                 "torque,rotor_accel,max_torque_pair_error_Nm\n";
    auto bench = [&](std::string name, Engine e, double speed, double throttle,
                     double hz, double duration) {
      double worst = 0, qa = 0;
      for (int i = 0; i < int(duration * hz); ++i) {
        double before = e.rpm * 2 * pi / 60;
        advance_propulsion(e, rho, altitude, speed, throttle, 1 / hz);
        qa = e.aero_torque;
        double rotor_moment =
            e.rotor_inertia * (e.rpm * 2 * pi / 60 - before) * hz;
        worst = std::max(worst, std::abs(rotor_moment + qa - e.torque));
      }
      std::cout << name << ',' << hz << ',' << speed << ',' << e.rpm << ','
                << e.thrust << ',' << e.shaft_power << ',' << e.thrust * speed
                << ',' << qa << ',' << e.acceleration << ',' << worst << '\n';
      return e;
    };
    for (double hz : {120., 240.}) {
      for (double speed : {0., 40., 70., 100., 140.})
        bench("powered", fixture.engine, speed, 1, hz, 10);
      Engine accelerating = fixture.engine;
      accelerating.rpm = 600;
      accelerating.throttle_state = 1;
      bench("spinup", accelerating, 70, 1, hz, 2);
      Engine shutdown = fixture.engine;
      shutdown.running = false;
      bench("shutdown", shutdown, 80, 0, hz, 30);
      Engine stopped = fixture.engine;
      stopped.rpm = 0;
      stopped.running = false;
      bench("stopped_in_flow", stopped, 80, 0, hz, 5);
      Engine absent = stopped;
      absent.prop_fraction = 0;
      bench("removed_prop", absent, 80, 0, hz, 5);
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
