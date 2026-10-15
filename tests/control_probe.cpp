#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
using namespace flight;
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("material airframe required");
    auto d = Definition::load(argv[1]);
    std::cout << std::setprecision(15)
              << "refresh,hz,material_removed,x,y,z,pitch_saturated_s,pitch_"
                 "integral_paused_s,max_alpha\n";
    for (int refresh : {0, 1})
      for (double hz : {120., 240., 480., 960.})
        for (int removed : {0, 1}) {
          Simulation s(d, 1 / hz);
          auto tr = s.trim(70, 1000);
          if (!tr.converged)
            throw std::runtime_error("entry trim failed");
          auto p = tr.intent;
          p.mouse = true;
          p.aim_world = {1, .3, 0};
          double saturated = 0, paused = 0, alpha = 0;
          for (int i = 0; i < int(5 * hz); ++i) {
            double t = i / hz;
            if (removed && i == int(hz))
              s.remove_material("left_outer_forward", .2);
            p.active[Roll] = t >= 1 && t < 2;
            p.direct[Roll] = 1;
            // Read-only force evaluation ablates previous-tick normal-load
            // feedback. It advances neither body nor actuator; every case calls
            // the same step.
            if (refresh)
              s.evaluate(s.body, true);
            s.step(p);
            saturated += s.instructor.saturated[Pitch] / hz;
            paused += s.instructor.integral_paused[Pitch] / hz;
            Vec v = s.body.attitude.conjugate().rotate(s.body.velocity -
                                                       s.wind_world);
            alpha = std::max(alpha, std::atan2(v.z, v.x));
          }
          auto x = s.body.position;
          std::cout << refresh << ',' << hz << ',' << removed << ',' << x.x
                    << ',' << x.y << ',' << x.z << ',' << saturated << ','
                    << paused << ',' << alpha << '\n';
        }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
