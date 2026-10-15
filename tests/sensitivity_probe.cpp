#include "flight/core.hpp"
#include <array>
#include <iomanip>
#include <iostream>
using namespace flight;
using Commands = std::vector<std::array<double, 3>>;
struct Run {
  Body body;
  Commands commands;
};
Run fly(const Definition &d, int hz, int removed,
        const Commands *commands = nullptr, int event_tick = -1) {
  Simulation s(d, 1. / hz);
  auto tr = s.trim(70, 1000);
  if (!tr.converged)
    throw std::runtime_error("entry trim failed");
  auto p = tr.intent;
  p.mouse = true;
  p.aim_world = {1, .3, 0};
  Run r;
  if (event_tick < 0)
    event_tick = hz;
  for (int i = 0; i < 5 * hz; ++i) {
    if (removed && i == event_tick)
      s.remove_material("left_outer_forward", .2);
    p.active[Roll] = i >= hz && i < 2 * hz;
    p.direct[Roll] = 1;
    if (commands) {
      // Zero-order hold of normalized 120 Hz requests. Preserve actual trim
      // fade, damaged chains, hinge effort, filtering and slew in step().
      p.mouse = false;
      p.active = {true, true, true};
      p.direct = (*commands)[i * 120 / hz];
    }
    s.step(p);
    std::array<double, 3> request{};
    for (int a = 0; a < 3; ++a)
      request[a] = s.ownership[a].command;
    r.commands.push_back(request);
  }
  r.body = s.body;
  return r;
}
int main(int argc, char **argv) {
  if (argc != 2)
    throw std::runtime_error("material airframe required");
  auto d = Definition::load(argv[1]);
  std::cout << std::setprecision(15);
  auto c = fly(d, 120, 0), m = fly(d, 120, 1);
  for (int mode = 0; mode < 3; ++mode)
    for (int hz : {120, 240, 480}) {
      const Commands *cc = mode ? &c.commands : nullptr,
                     *mc = mode == 1   ? &m.commands
                           : mode == 2 ? &c.commands
                                       : nullptr;
      auto a = fly(d, hz, 0, cc), b = fly(d, hz, 1, mc);
      auto e = b.body.position - a.body.position;
      std::cout << "effect " << mode << ' ' << hz << ' ' << e.x << ' ' << e.y
                << ' ' << e.z << " norm " << norm(e) << " replay_identity "
                << (mode == 1 && hz == 120
                        ? norm(a.body.position - c.body.position) +
                              norm(b.body.position - m.body.position)
                        : 0)
                << '\n';
    }
  auto fine = fly(d, 480, 0);
  for (int tick : {476, 478, 480, 482, 484}) {
    auto a = fly(d, 480, 1, nullptr, tick);
    auto e = a.body.position - fine.body.position;
    std::cout << "event " << tick / 480. << ' ' << e.x << ' ' << e.y << ' '
              << e.z << '\n';
  }
  auto lift = d;
  lift.engine.max_power = 0;
  lift.engine.engine_friction = 0;
  lift.engine.prop_fraction = 0;
  lift.body_drag_area = 0;
  for (auto &t : lift.curves)
    for (auto &p : t.second) {
      p.cl = .5;
      p.cd = 0;
      p.cm = 0;
    }
  for (auto &e : lift.edges) {
    e.yield_stress = 1e20;
    e.ultimate_stress = 1e21;
  }
  for (int hz : {120, 240, 480}) {
    Simulation s(lift, 1. / hz);
    PilotIntent p;
    double old = mechanical_energy(s.body, s.mass, s.def.engine);
    s.evaluate(s.body, true);
    auto v = s.body.attitude.conjugate().rotate(s.body.velocity);
    double power = dot(s.aero.force, v) + dot(s.aero.moment, s.body.omega);
    for (int i = 0; i < hz / 2; ++i)
      s.step(p);
    std::cout << "purelift " << hz << " initial_power " << power
              << " energy_error "
              << mechanical_energy(s.body, s.mass, s.def.engine) - old << '\n';
  }
}
