#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;
void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe required");
    auto d = Definition::load(argv[1]);
    double worst = 0;
    for (auto model : {PropellerModel::BladeElement, PropellerModel::Legacy})
      for (int hz : {60, 120, 240, 480})
        for (int kind = 0; kind < 8; ++kind) {
          Simulation s(d, 1. / hz);
          s.def.engine.model = model;
          s.def.engine.max_power = 0;
          s.def.engine.prop_fraction = 0;
          auto &c = s.def.chains[0];
          c.q_limit = 1e100;
          c.limit = 1;
          c.slew = 1;
          c.lag = 0;
          c.deflection = c.filtered = 0;
          PilotIntent p;
          p.active = {true, true, true};
          p.direct[Roll] = 1;
          double end = s.dt, middle = s.dt / 2;
          if (kind == 1) {
            c.limit = s.dt * .2;
            end = middle = c.limit;
          }
          if (kind == 2) {
            c.lag = .2;
            c.slew = 1e6;
            end = 1 - std::exp(-s.dt / c.lag);
            middle = 1 - std::exp(-s.dt / (2 * c.lag));
          }
          if (kind == 3) {
            c.friction = 3;
            c.range = .4;
            end /= 4;
            middle /= 4;
          }
          if (kind == 4) {
            c.jammed = true;
            c.jam_position = .07;
            end = middle = .07;
          }
          if (kind == 5) {
            c.connected = false;
            c.deflection = c.filtered = .1;
            end = .1 - s.dt;
            middle = .1 - s.dt / 2;
          }
          if (kind == 6) {
            c.range = s.dt * .2;
            c.hinge = .35;
            end = middle = c.range;
          }
          if (kind == 7) {
            c.q_limit =
                .5 * 1.225 * std::exp(s.body.position.z / 8500.) * 70 * 70;
            end = s.dt / 2;
            middle = s.dt / 4;
          }
          s.step(p);
          worst = std::max(worst, std::abs(c.deflection - end));
          require(std::abs(c.deflection - end) < 1e-14,
                  "actual endpoint remains the physical actuator endpoint");
          require(std::abs(c.midpoint_deflection - middle) < 1e-14,
                  "rate, arrival, lag, friction, jam and disconnect use the "
                  "correct interval midpoint");
          bool seen = false;
          for (const auto &cell : s.def.cells)
            if (cell.chain == 0) {
              seen = true;
              require(std::abs(cell.deflection -
                               middle * c.hinge * cell.control_sign) < 1e-14,
                      "integrated cell force must use the mechanical midpoint");
            }
          require(seen, "force-bearing control mapping required");
          s.evaluate(s.body, true);
          for (const auto &cell : s.def.cells)
            if (cell.chain == 0)
              require(std::abs(cell.deflection -
                               end * c.hinge * cell.control_sign) < 1e-14,
                      "instantaneous evaluate must read current endpoint, not "
                      "an old interval sample");
        }
    for (int hz : {120, 240}) {
      Simulation s(d, 1. / hz);
      auto &c = s.def.chains[0];
      c.deflection = c.filtered = .1;
      c.q_limit = 1e100;
      c.slew = 1;
      c.lag = 0;
      for (auto &e : s.def.edges)
        if (e.a == 0 && e.b == 1)
          e.failed = true;
      std::vector<double> zero(s.def.nodes.size());
      s.solve_structure(zero, zero, 0);
      require(s.debris.size() == 1, "detached wing fixture");
      s.step({});
      const auto &piece = s.debris[0];
      bool seen = false;
      for (const auto &element : piece.aero_cells)
        if (element.cell.chain >= 0) {
          const auto &local = piece.chains[element.cell.chain];
          if (local.name == c.name) {
            seen = true;
            require(std::abs(local.deflection - (.1 - s.dt)) < 1e-14,
                    "detached endpoint neutral relaxation");
            require(std::abs(element.cell.deflection -
                             (.1 - s.dt / 2) * local.hinge *
                                 element.cell.control_sign) < 1e-14,
                    "detached loads use their own neutral-relaxation midpoint");
          }
        }
      require(seen, "detached control cells retained");
    }
    std::cout << std::setprecision(15)
              << "{\"checks_passed\":true,\"max_endpoint_error_rad\":" << worst
              << ",\"cases\":66}\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
