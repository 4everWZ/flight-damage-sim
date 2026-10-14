#include "flight/core.hpp"
#include <iostream>
#include <stdexcept>
using namespace flight;
void require(bool b, const char *message) {
  if (!b)
    throw std::runtime_error(message);
}
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe path required");
    Wrench w;
    w.add({0, 0, -100}, {0, -2, 0});
    require(std::abs(w.moment.x - 200) < 1e-10,
            "left upward load must roll right");
    // Analytic dynamics verify the real integration boundary, including gravity
    // and torque.
    MassProperties analytic;
    analytic.mass = 2;
    analytic.inertia = Mat::diagonal({2, 3, 4});
    Body ballistic;
    ballistic.position = {};
    ballistic.velocity = {3, 0, 0};
    for (int i = 0; i < 120; ++i)
      integrate_body(ballistic, analytic, Wrench{}, 1. / 120);
    require(std::abs(ballistic.position.x - 3) < 1e-10 &&
                std::abs(ballistic.position.z - .5 * gravity) < 1e-10,
            "analytic ballistic motion");
    double energy =
        .5 * analytic.mass * dot(ballistic.velocity, ballistic.velocity) -
        analytic.mass * gravity * ballistic.position.z;
    require(std::abs(energy - 9) < 1e-9, "ballistic energy conservation");
    Body rotation;
    rotation.position = {};
    rotation.velocity = {};
    Wrench moment;
    moment.moment = {4, 0, 0};
    for (int i = 0; i < 120; ++i)
      integrate_body(rotation, analytic, moment, 1. / 120);
    require(std::abs(rotation.omega.x - 2) < 1e-10 &&
                std::abs(rotation.attitude.w - std::cos(.5)) < 1e-5,
            "torque integrates angular acceleration and attitude");
    auto d = Definition::load(argv[1]);
    Simulation s(d);
    require(s.def.cells.size() == 152, "representative distributed fixture");
    auto tr = s.trim(70, 1000);
    require(tr.residual < 1e-6, "trim force/moment residual");
    auto p = tr.intent;
    p.mouse = true;
    p.aim_world = {1, .3, 0};
    p.active[Roll] = true;
    p.direct[Roll] = -1;
    s.update_controls(p);
    require(s.ownership[Roll].command == -1 &&
                s.ownership[Roll].direct_weight == 1,
            "keyboard full authority");
    require(s.ownership[Pitch].direct_weight == 0 &&
                s.ownership[Yaw].direct_weight == 0,
            "independent ownership");
    p.active[Roll] = false;
    s.update_controls(p);
    require(s.ownership[Roll].command == -1, "handoff initial continuity");
    for (int i = 0; i < 31; ++i)
      s.update_controls(p);
    require(s.ownership[Roll].direct_weight == 0, "handoff finite duration");
    s.disconnect("left_aileron");
    for (int i = 0; i < 120; ++i)
      s.update_controls(p);
    require(std::abs(s.def.chains[0].deflection) < 1e-5,
            "disconnected linkage cannot be bypassed");
    Simulation clean(d), damaged(d);
    clean.trim(70, 1000);
    damaged.trim(70, 1000);
    clean.evaluate(clean.body, true);
    int count = 0;
    for (auto &c : damaged.def.cells)
      if (c.node == 2 && count++ < 8)
        damaged.damage_cell(c.id, 0, 1);
    damaged.evaluate(damaged.body, true);
    require(damaged.aero.force.z > clean.aero.force.z, "loss of local lift");
    require(damaged.aero.moment.x < clean.aero.moment.x - 100,
            "asymmetric moment follows force locations");
    require(damaged.aero.force.x < clean.aero.force.x,
            "skin gap increases drag");
    require(damaged.debris.empty(), "aero loss cannot detach structure");
    // Local angular flow: right/left wings see different incidence during roll.
    clean.body.omega.x = 1;
    clean.evaluate(clean.body, true);
    double left = 0, right = 0;
    for (auto &c : clean.def.cells) {
      if (c.node == 2)
        left += c.alpha;
      if (c.node == 4)
        right += c.alpha;
    }
    require(right > left, "angular velocity must affect local incidence");
    // Rotated mass properties and separation inherit the same velocity field
    // and conserve momentum.
    Simulation split(d);
    split.body.omega = {.2, .3, .1};
    split.body.velocity = {70, 2, 3};
    auto old = split.mass;
    Vec origin = split.body.position;
    Vec momentum = split.body.velocity * old.mass;
    Vec angular = split.body.attitude.rotate(old.inertia * split.body.omega);
    for (auto &e : split.def.edges)
      if (e.a == 0 && e.b == 1)
        e.failed = true;
    std::vector<double> zero(d.nodes.size());
    split.solve_structure(zero, zero, 0);
    Vec after = split.body.velocity * split.mass.mass;
    Vec h = split.body.attitude.rotate(split.mass.inertia * split.body.omega) +
            cross(split.body.position - origin,
                  split.body.velocity * split.mass.mass);
    for (auto &piece : split.debris) {
      after += piece.state.velocity * piece.mass.mass;
      h += piece.state.attitude.rotate(piece.mass.inertia * piece.state.omega) +
           cross(piece.state.position - origin,
                 piece.state.velocity * piece.mass.mass);
    }
    require(norm(after - momentum) < 1e-8, "split linear momentum");
    require(norm(h - angular) < 1e-7, "split angular momentum");
    require(split.debris.size() == 1 && split.mass.mass < old.mass,
            "graph disconnection updates mass");
    // A jam is a physical surface position, independent of q and ownership.
    s.def.chains[1].jammed = true;
    s.def.chains[1].jam_position = .12;
    s.update_controls(p);
    require(std::abs(s.def.chains[1].deflection - .12) < 1e-12,
            "jam retains physical position");
    // Loss of one parallel path changes load distribution; material plasticity
    // needs load and time.
    Simulation bench(d);
    std::vector<double> bend(d.nodes.size()), torsion(d.nodes.size());
    bend[1] = -old.mass * gravity / 2 * 2.4;
    bench.solve_structure(bend, torsion, 0);
    double intact_rear = std::abs(bench.def.edges[1].bend_load);
    bench.weaken("left_root_front", .3);
    bench.solve_structure(bend, torsion, 0);
    require(std::abs(bench.def.edges[1].bend_load) > intact_rear * 1.5,
            "stiffness-dependent redistribution");
    for (int i = 0; i < 360; ++i)
      bench.solve_structure(bend, torsion, 1. / 120);
    require(bench.debris.empty(), "weak spar must survive low load");
    bend[1] *= 5;
    bench.solve_structure(bend, torsion, 1. / 120);
    require(bench.debris.empty(), "finite overload exposure before failure");
    for (int i = 0; i < 600 && bench.debris.empty(); ++i)
      bench.solve_structure(bend, torsion, 1. / 120);
    require(!bench.debris.empty(),
            "high load must cause progressive material failure");
    std::cout
        << "PASS: wrench, trim, ownership/release, linkage, local airflow, "
           "aero loss, split momentum, redistribution and overload\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
