#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
Definition passive(Definition d, double cd) {
  d.engine.max_power = 0;
  d.engine.engine_friction = 0;
  d.engine.prop_fraction = 0;
  d.engine.rpm = 0;
  d.body_drag_area = 0;
  for (auto &table : d.curves)
    for (auto &point : table.second) {
      point.cl = .5;
      point.cd = cd;
      point.cm = 0;
    }
  for (auto &edge : d.edges) {
    edge.yield_stress = 1e20;
    edge.ultimate_stress = 1e21;
  }
  return d;
}
double piece_energy(const DetachedBody &p) {
  Engine off;
  return mechanical_energy(p.state, p.mass, off);
}
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe required");
    auto d = Definition::load(argv[1]);
    double worst_lift = 0, worst_piece = 0, min_drag_loss = 1e100;
    for (double hz : {120., 240., 480.}) {
      for (auto model :
           {PropellerModel::BladeElement, PropellerModel::Legacy}) {
        Simulation s(passive(d, 0), 1 / hz);
        s.def.engine.model = model;
        s.body.omega = {.1, -.06, .08};
        s.body.attitude =
            (Quat::axis({0, 0, 1}, .4) * Quat::axis({0, 1, 0}, .1))
                .normalized();
        s.body.velocity = s.body.attitude.rotate({70, 4, 3});
        s.evaluate(s.body, true);
        auto vb = s.body.attitude.conjugate().rotate(s.body.velocity);
        require(std::abs(dot(s.aero.force, vb) +
                         dot(s.aero.moment, s.body.omega)) < 1e-7,
                "distributed lift has zero instantaneous power");
        double before = mechanical_energy(s.body, s.mass, s.def.engine);
        for (int i = 0; i < int(hz / 2); ++i)
          s.step({});
        double error =
            std::abs(mechanical_energy(s.body, s.mass, s.def.engine) - before);
        worst_lift = std::max(worst_lift, error);
        // Absolute work bound: < 1 mJ, against a megajoule-scale trajectory.
        require(error < .001,
                "midpoint pure lift cannot inject discrete energy");
      }
      Simulation s(passive(d, 0), 1 / hz);
      for (auto &edge : s.def.edges)
        if (edge.a == 0 && edge.b == 1)
          edge.failed = true;
      std::vector<double> zero(s.def.nodes.size());
      s.solve_structure(zero, zero, 0);
      require(s.debris.size() == 1 && !s.debris[0].aero_cells.empty(),
              "wing component bench");
      auto &piece = s.debris[0];
      piece.state.omega = {.1, -.06, .08};
      piece.state.velocity = {70, 4, 3};
      double before = piece_energy(piece);
      for (int i = 0; i < int(hz / 2); ++i)
        s.step({});
      double error = std::abs(piece_energy(s.debris[0]) - before);
      worst_piece = std::max(worst_piece, error);
      require(error < .001,
              "detached distributed pure lift has no numerical energy source");
      Simulation drag(passive(d, .08), 1 / hz);
      before = mechanical_energy(drag.body, drag.mass, drag.def.engine);
      for (int i = 0; i < int(hz / 2); ++i)
        drag.step({});
      double loss =
          before - mechanical_energy(drag.body, drag.mass, drag.def.engine);
      min_drag_loss = std::min(min_drag_loss, loss);
      require(loss > 100,
              "unpowered positive cell drag dissipates mechanical energy");
    }
    std::cout << std::setprecision(15)
              << "{\"pure_lift_max_energy_error_J\":" << worst_lift
              << ",\"detached_pure_lift_max_energy_error_J\":" << worst_piece
              << ",\"min_drag_loss_J\":" << min_drag_loss << "}\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
