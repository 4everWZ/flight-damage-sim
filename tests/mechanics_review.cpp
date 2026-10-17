#include "flight/core.hpp"
#include <iostream>
#include <numeric>
using namespace flight;
namespace {
int failures = 0;
void check(bool okay, const char *name, double value) {
  std::cout << (okay ? "PASS " : "FAIL ") << name << " value=" << value << '\n';
  failures += !okay;
}
struct Drift {
  double energy, momentum;
  Quat attitude;
};
Drift torque_free(double hz) {
  MassProperties m;
  m.mass = 2;
  m.inertia = Mat::diagonal({2, 3, 5});
  m.inertia.a[0][2] = m.inertia.a[2][0] = .35;
  Body b;
  b.omega = {1.3, .8, 1.1};
  double energy = .5 * dot(b.omega, m.inertia * b.omega);
  Vec momentum = b.attitude.rotate(m.inertia * b.omega);
  for (int i = 0; i < int(5 * hz); ++i)
    integrate_body(b, m, Wrench{}, 1 / hz);
  return {std::abs(.5 * dot(b.omega, m.inertia * b.omega) - energy) / energy,
          norm(b.attitude.rotate(m.inertia * b.omega) - momentum) /
              norm(momentum),
          b.attitude};
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe required");
    auto fixture = Definition::load(argv[1]);
    auto coarse = torque_free(120), fine = torque_free(240);
    check(coarse.energy < 1e-5, "torque-free full-inertia energy drift",
          coarse.energy);
    check(coarse.momentum < 2e-4, "torque-free world angular momentum drift",
          coarse.momentum);
    auto reference = torque_free(960);
    auto angle_error = [](Quat a, Quat b) {
      auto diff = a.conjugate() * b;
      return 2 *
             std::atan2(norm(Vec{diff.x, diff.y, diff.z}), std::abs(diff.w));
    };
    double coarse_angle = angle_error(coarse.attitude, reference.attitude),
           fine_angle = angle_error(fine.attitude, reference.attitude);
    check(fine_angle < coarse_angle * .3,
          "torque-free attitude second-order convergence",
          fine_angle / coarse_angle);
    MassProperties slender;
    slender.mass = 10;
    slender.inertia = Mat::diagonal({.2, 3, 5});
    Body spinning;
    spinning.omega = {10, 15, 20};
    double initial_energy =
        .5 * dot(spinning.omega, slender.inertia * spinning.omega);
    for (int i = 0; i < 120; ++i)
      integrate_body(spinning, slender, Wrench{}, 1. / 120);
    check(std::abs(.5 * dot(spinning.omega, slender.inertia * spinning.omega) -
                   initial_energy) /
                  initial_energy <
              1e-9,
          "anisotropic high-rate free body remains finite/conservative",
          std::abs(.5 * dot(spinning.omega, slender.inertia * spinning.omega) -
                   initial_energy) /
              initial_energy);
    // Pure lift on a flexed wing must do no work along its local relative
    // velocity.
    auto pure = fixture;
    pure.body_drag_area = 0;
    for (auto &table : pure.curves)
      for (auto &row : table.second) {
        row.cd = 0;
        row.cm = 0;
      }
    Simulation lift(pure);
    lift.body.velocity = {70, 20, 7};
    lift.def.nodes[1].bend = lift.def.nodes[2].bend = .25;
    lift.def.nodes[3].bend = lift.def.nodes[4].bend = -.25;
    lift.evaluate(lift.body, true);
    double power = 0, relative = 0;
    for (auto &c : lift.def.cells) {
      power += dot(c.force, c.flow);
      relative =
          std::max(relative, std::abs(dot(c.force, c.flow)) /
                                 std::max(norm(c.force) * norm(c.flow), 1.));
    }
    check(relative < 1e-10, "flexed-cell lift orthogonal to local flow",
          relative);
    std::cout << "lift_only_net_power_W=" << power << '\n';
    // Refining each cell into two half-area coincident cells must leave total
    // wrench unchanged.
    Simulation original(fixture);
    original.body.velocity = {70, 3, 5};
    original.body.omega = {.2, .1, -.3};
    auto base = original.evaluate(original.body, true);
    auto refined = fixture;
    refined.cells.clear();
    for (auto c : fixture.cells) {
      c.area *= .5;
      c.id *= 2;
      refined.cells.push_back(c);
      ++c.id;
      refined.cells.push_back(c);
    }
    Simulation doubled(refined);
    doubled.body = original.body;
    auto split = doubled.evaluate(doubled.body);
    double split_err =
        norm(split.force - base.force) + norm(split.moment - base.moment);
    check(split_err < 1e-8, "cell refinement does not double forces",
          split_err);
    Wrench sum;
    for (auto &c : original.def.cells)
      sum.add(c.force, c.position - original.mass.cg, c.moment);
    Vec vb = original.body.velocity;
    sum.add(-unit(vb) *
                (.5 * 1.225 * std::exp(original.body.position.z / 8500) *
                 dot(vb, vb) * fixture.body_drag_area),
            -original.mass.cg);
    check(norm(sum.force - original.aero.force) +
                  norm(sum.moment - original.aero.moment) <
              1e-8,
          "cell forces accumulated once",
          norm(sum.force - original.aero.force));
    // Global yaw rotation and a Galilean translation (same airspeed) change
    // neither local wrench nor airflow.
    Simulation rotated(fixture);
    rotated.body = original.body;
    auto yaw = Quat::axis({0, 0, 1}, 1.2);
    rotated.body.attitude = yaw;
    rotated.wind_world = {12, -7, 0};
    rotated.body.velocity =
        yaw.rotate(original.body.velocity) + rotated.wind_world;
    auto turned = rotated.evaluate(rotated.body);
    check(norm(turned.force - base.force) + norm(turned.moment - base.moment) <
              1e-8,
          "frame rotation / wind covariance", norm(turned.force - base.force));
    // Reflection of a symmetric airframe reverses only lateral force and
    // roll/yaw moments.
    Simulation mirror(fixture);
    mirror.body = original.body;
    mirror.body.velocity.y *= -1;
    mirror.body.omega.x *= -1;
    mirror.body.omega.z *= -1;
    auto reflected = mirror.evaluate(mirror.body);
    double mirror_err =
        norm(reflected.force - Vec{base.force.x, -base.force.y, base.force.z}) +
        norm(reflected.moment -
             Vec{-base.moment.x, base.moment.y, -base.moment.z});
    check(mirror_err < 1e-8, "symmetric airframe reflection", mirror_err);
    auto unpowered = fixture;
    unpowered.engine.running = false;
    unpowered.engine.prop_fraction = 0;
    Simulation symmetric(unpowered);
    symmetric.body.velocity = {70, 0, 7};
    symmetric.step(PilotIntent{});
    double bending_error =
        std::abs(symmetric.node_bending[1] + symmetric.node_bending[3]) +
        std::abs(symmetric.node_bending[2] + symmetric.node_bending[4]);
    check(bending_error < 1e-8,
          "symmetric actual aero/inertial structural loads", bending_error);
    // A weak structure with zero applied aero/prop loads may not fail because a
    // scenario clock advanced.
    auto empty = unpowered;
    empty.body_drag_area = 0;
    for (auto &c : empty.cells) {
      c.area_fraction = 0;
      c.roughness = 0;
    }
    Simulation freefall(empty);
    freefall.weaken("left_root_front", .3);
    PilotIntent full;
    full.active[Pitch] = true;
    full.direct[Pitch] = 1;
    for (int i = 0; i < 1200; ++i)
      freefall.step(full);
    double strain = 0;
    for (auto &e : freefall.def.edges)
      strain += e.plastic_strain;
    check(freefall.debris.empty() && strain == 0,
          "no loads: no time/G-scripted failure", strain);
    // Simultaneous pieces, tilted attitude and nonzero products of inertia
    // preserve split energy/momentum and datum.
    auto asym = fixture;
    for (std::size_t i = 0; i < asym.masses.size(); ++i)
      asym.masses[i].position.z = .06 * i;
    Simulation partition(asym);
    partition.body.attitude =
        (Quat::axis({0, 0, 1}, .7) * Quat::axis({1, 0, 0}, .4)).normalized();
    partition.body.omega = {.7, -.3, .2};
    partition.body.velocity = {60, 4, -2};
    auto old = partition.mass;
    auto initial = partition.body;
    Vec momentum = initial.velocity * old.mass,
        angular = initial.attitude.rotate(old.inertia * initial.omega);
    double kinetic = .5 * old.mass * dot(initial.velocity, initial.velocity) +
                     .5 * dot(initial.omega, old.inertia * initial.omega);
    Vec datum = initial.position - initial.attitude.rotate(old.cg);
    for (auto &e : partition.def.edges)
      if (e.a == 0 && (e.b == 1 || e.b == 5))
        e.failed = true;
    std::vector<double> zero(asym.nodes.size());
    partition.solve_structure(zero, zero, 0);
    Vec after{}, h{};
    double energy = 0, total_mass = 0;
    auto sum_piece = [&](const Body &b, const MassProperties &m) {
      after += b.velocity * m.mass;
      h += b.attitude.rotate(m.inertia * b.omega) +
           cross(b.position - initial.position, b.velocity * m.mass);
      energy += .5 * m.mass * dot(b.velocity, b.velocity) +
                .5 * dot(b.omega, m.inertia * b.omega);
      total_mass += m.mass;
    };
    sum_piece(partition.body, partition.mass);
    for (auto &piece : partition.debris)
      sum_piece(piece.state, piece.mass);
    check(partition.debris.size() == 2 &&
              std::abs(total_mass - old.mass) < 1e-10,
          "simultaneous connected-component mass partition",
          total_mass - old.mass);
    check(norm(after - momentum) < 1e-8 && norm(h - angular) < 1e-7,
          "rotated asymmetric split momentum", norm(h - angular));
    check(std::abs(energy - kinetic) / kinetic < 1e-12, "split kinetic energy",
          std::abs(energy - kinetic) / kinetic);
    check(norm(partition.body.position -
               partition.body.attitude.rotate(partition.mass.cg) - datum) <
              1e-10,
          "CG change leaves surviving datum continuous",
          norm(partition.body.position -
               partition.body.attitude.rotate(partition.mass.cg) - datum));
    // No key was ever pressed: any configured handoff time must start with
    // complete Instructor ownership.
    auto long_handoff = fixture;
    long_handoff.handoff = .5;
    Simulation owner(long_handoff);
    PilotIntent input;
    input.mouse = true;
    input.aim_world = {1, .3, 0};
    owner.update_controls(input);
    check(owner.ownership[Roll].direct_weight == 0,
          "initial Instructor ownership with 0.5s handoff",
          owner.ownership[Roll].direct_weight);
    Simulation coordinated(fixture);
    coordinated.body.velocity = {70, 7, 0};
    coordinated.load_factor = 1;
    PilotIntent aim;
    aim.mouse = true;
    aim.aim_world = unit(coordinated.body.velocity);
    coordinated.update_controls(aim);
    check(coordinated.instructor.desired_rates.z > 0,
          "positive sideslip requests stabilizing positive yaw",
          coordinated.instructor.desired_rates.z);
    auto initial_bias = coordinated.instructor.rate_bias[Roll];
    aim.active[Roll] = true;
    aim.direct[Roll] = -1;
    for (int i = 0; i < 60; ++i)
      coordinated.update_controls(aim);
    check(coordinated.ownership[Roll].command == -1 &&
              coordinated.instructor.rate_bias[Roll] == initial_bias,
          "direct axis freezes Instructor integrator",
          coordinated.instructor.rate_bias[Roll] - initial_bias);
    Simulation level(fixture);
    level.body.attitude = Quat::axis({0, 1, 0}, rad(10));
    // Force-balanced sensors: body Z alone is cos(pitch), not 1g. Split a
    // nonzero normal propeller contribution from the aerodynamic sensor too.
    double weight = level.mass.mass * gravity;
    level.propulsion.force = {weight * .1, 0, -weight * .02};
    level.aero.force = level.body.attitude.conjugate().rotate({0, 0, -weight}) -
                       level.propulsion.force;
    level.load_factor = -level.aero.force.z / weight;
    aim = {};
    aim.mouse = true;
    level.update_controls(aim);
    check(std::abs(level.instructor.desired_rates.y) < 1e-10,
          "level air-path aim does not command a nose attitude",
          level.instructor.desired_rates.y);
    // A trim solver may not manufacture a disconnected or jammed surface
    // position.
    Simulation damaged(fixture);
    damaged.disconnect("elevator");
    auto damaged_trim = damaged.trim(70, 1000);
    check(damaged_trim.residual > 1e-4,
          "unachievable damaged trim has nonzero residual",
          damaged_trim.residual);
    check(std::abs(damaged.def.chains[2].deflection) < 1e-12,
          "trim respects disconnected elevator",
          damaged.def.chains[2].deflection);
    Simulation jam(fixture);
    jam.def.chains[2].jammed = true;
    jam.def.chains[2].jam_position = -.07;
    jam.trim(70, 1000);
    check(std::abs(jam.def.chains[2].deflection + .07) < 1e-12,
          "trim respects jam position", jam.def.chains[2].deflection);
    std::cout << "review_failures=" << failures << '\n';
    return failures ? 1 : 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
