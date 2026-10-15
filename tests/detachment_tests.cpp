#include "flight/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
void break_root(Simulation &s, int root) {
  for (auto &edge : s.def.edges)
    if (edge.a == 0 && edge.b == root)
      edge.failed = true;
  std::vector<double> zero(s.def.nodes.size());
  s.solve_structure(zero, zero,
                    0); // Explicit severed-connection component bench.
}
double energy(const DetachedBody &p) {
  return .5 * p.mass.mass * dot(p.state.velocity, p.state.velocity) -
         p.mass.mass * gravity * p.state.position.z +
         .5 * dot(p.state.omega, p.mass.inertia * p.state.omega);
}
Simulation component(const Definition &d, double hz) {
  Simulation s(d, 1 / hz);
  s.def.engine.max_power = 0;
  s.def.engine.engine_friction = 0;
  s.def.engine.prop_fraction = 0;
  s.def.body_drag_area = 0;
  s.body.velocity = {70, 0, 0};
  s.def.chains[0].deflection = .15;
  s.def.nodes[1].bend = .02;
  s.def.nodes[2].twist = .01;
  break_root(s, 1);
  return s;
}
int main(int argc, char **argv) {
  try {
    if (argc < 2)
      throw std::runtime_error("airframe path required [trace.csv]");
    auto def = Definition::load(argv[1]);
    // Compare the original cells' wrench with their disjoint owner reductions.
    Simulation split(def);
    split.def.engine.prop_fraction = 0;
    split.def.body_drag_area = 0;
    split.wind_world = {3, -2, 0};
    split.body.attitude =
        (Quat::axis({0, 0, 1}, .6) * Quat::axis({0, 1, 0}, .1)).normalized();
    split.body.velocity =
        split.body.attitude.rotate({70, 3, 2}) + split.wind_world;
    split.body.omega = {.1, .2, -.15};
    split.def.nodes[1].bend = .03;
    split.def.nodes[2].twist = -.02;
    split.def.chains[0].deflection = .12;
    auto old_mass = split.mass;
    auto original = split.evaluate(split.body, true);
    Vec origin = split.body.position;
    auto world_force = split.body.attitude.rotate(original.force);
    auto world_moment = split.body.attitude.rotate(original.moment);
    // Connected nodes' shape is held equal for the instant partition
    // comparison.
    break_root(split, 1);
    require(split.debris.size() == 1, "one disconnected graph component");
    auto &piece = split.debris.front();
    require(piece.aero_cells.size() == 64, "whole left wing owns its 64 cells");
    auto parent = split.evaluate(split.body, true);
    auto load = split.evaluate_detached(piece);
    // Existing airframe atmosphere samples density at each body's CG. A tilted
    // split changes those sample altitudes slightly. Normalize to original rho
    // to isolate cell ownership/reduction from this one-point approximation.
    double root_scale = std::exp((origin.z - split.body.position.z) / 8500.);
    double piece_scale = std::exp((origin.z - piece.state.position.z) / 8500.);
    double density_jump =
        norm(split.body.attitude.rotate(parent.force) +
             piece.state.attitude.rotate(load.force) - world_force);
    parent.force = parent.force * root_scale;
    parent.moment = parent.moment * root_scale;
    load.force = load.force * piece_scale;
    load.moment = load.moment * piece_scale;
    Vec f = split.body.attitude.rotate(parent.force) +
            piece.state.attitude.rotate(load.force);
    Vec m = split.body.attitude.rotate(parent.moment) +
            cross(split.body.position - origin,
                  split.body.attitude.rotate(parent.force)) +
            piece.state.attitude.rotate(load.moment) +
            cross(piece.state.position - origin,
                  piece.state.attitude.rotate(load.force));
    double force_error = norm(f - world_force),
           moment_error = norm(m - world_moment);
    require(force_error < 1e-7 && moment_error < 1e-6,
            "partition cells once, moments about consistent CGs");
    Vec linear = split.body.velocity * split.mass.mass +
                 piece.state.velocity * piece.mass.mass;
    require(norm(linear -
                 (split.body.attitude.rotate({70, 3, 2}) + split.wind_world) *
                     old_mass.mass) < 1e-7,
            "birth preserves linear momentum");
    require(!split.def.chains[0].connected && split.def.chains[1].connected,
            "only missing parent control branch disconnects");
    require(std::all_of(piece.chains.begin(), piece.chains.end(),
                        [](auto &c) { return !c.connected; }),
            "detached surface has no pilot path");
    require(piece.chains.size() == 1 && piece.chains[0].name == "left_aileron",
            "piece owns only its actual mapped surfaces");
    auto shared_def = def;
    for (auto &cell : shared_def.cells)
      if (cell.chain == 1)
        cell.chain = 0;
    Simulation shared(shared_def);
    break_root(shared, 1);
    require(shared.def.chains[0].connected &&
                !shared.debris[0].chains[0].connected,
            "a surviving shared control branch stays piloted, the separated "
            "branch does not");
    Simulation tail(def);
    break_root(tail, 5);
    require(!tail.def.chains[2].connected && !tail.def.chains[3].connected &&
                tail.def.chains[0].connected && tail.def.chains[1].connected &&
                tail.debris[0].chains.size() == 2,
            "tail split partitions both elevator and rudder without cutting "
            "ailerons");
    tail.evaluate_detached(tail.debris[0]);
    auto a = component(def, 120), b = a;
    PilotIntent left, right;
    left.active[Roll] = right.active[Roll] = true;
    left.direct[Roll] = -1;
    right.direct[Roll] = 1;
    double before = energy(a.debris[0]);
    for (int i = 0; i < 60; ++i) {
      a.step(left);
      b.step(right);
    }
    auto &pa = a.debris[0];
    auto &pb = b.debris[0];
    require(norm(pa.state.position - pb.state.position) < 1e-10 &&
                norm(pa.state.omega - pb.state.omega) < 1e-10,
            "opposite pilot commands cannot drive a detached piece");
    require(std::abs(pa.chains[0].deflection) < 1e-12,
            "unpowered intact hinge relaxes to neutral");
    require(energy(pa) < before,
            "unpowered cell drag dissipates mechanical energy");
    auto jam = component(def, 120);
    jam.debris[0].chains[0].jammed = true;
    jam.debris[0].chains[0].jam_position = .15;
    for (int i = 0; i < 60; ++i)
      jam.step(left);
    require(std::abs(jam.debris[0].chains[0].deflection - .15) < 1e-12,
            "mechanical jam survives lost pilot linkage");
    require(norm(jam.debris[0].state.omega - pa.state.omega) > .01,
            "actual residual surface changes piece motion");
    // Own-body flow, rotational flow, zero-airflow, wind/frame covariance and
    // passive work.
    auto probe = component(def, 120);
    auto &p = probe.debris[0];
    probe.wind_world = {5, -3, 0};
    p.state.velocity = probe.wind_world;
    p.state.omega = {};
    require(norm(probe.evaluate_detached(p).force) < 1e-12,
            "no ghost parent flow on a stationary-in-air piece");
    p.state.velocity = probe.wind_world + Vec{65, 2, 3};
    p.state.omega = {.2, -.1, .15};
    auto pl = probe.evaluate_detached(p);
    Vec v = p.state.attitude.conjugate().rotate(p.state.velocity -
                                                probe.wind_world);
    double power = dot(pl.force, v) + dot(pl.moment, p.state.omega);
    require(power < 0,
            "passive piece wrench cannot produce net air-relative work");
    double flow_error = 0;
    for (auto &e : p.aero_cells)
      flow_error = std::max(
          flow_error, norm(e.cell.flow - v -
                           cross(p.state.omega, e.cell.position - p.mass.cg)));
    require(flow_error < 1e-12, "cell flow uses piece CG and angular velocity");
    auto damaged = probe;
    int missing_id = damaged.debris[0].aero_cells.front().cell.id;
    auto root_before = damaged.evaluate(damaged.body);
    damaged.damage_cell(missing_id, 0, 0);
    auto dl = damaged.evaluate_detached(damaged.debris[0]);
    require(damaged.debris[0].aero_cells.front().cell.area_fraction == 0 &&
                norm(dl.force - pl.force) > 1,
            "damage by stable cell id reaches its current detached owner");
    require(norm(damaged.evaluate(damaged.body).force - root_before.force) == 0,
            "detached cell damage cannot leak into the parent wrench");
    auto rotated = probe;
    Quat yaw = Quat::axis({0, 0, 1}, .8);
    rotated.wind_world = yaw.rotate(probe.wind_world);
    rotated.debris[0].state.velocity = yaw.rotate(p.state.velocity);
    rotated.debris[0].state.attitude = (yaw * p.state.attitude).normalized();
    auto rl = rotated.evaluate_detached(rotated.debris[0]);
    require(norm(rl.force - pl.force) < 1e-8 &&
                norm(rl.moment - pl.moment) < 1e-8,
            "world yaw/wind change preserves piece body loads");
    // Halving dt against a finer reference; no aerodynamic coefficients change.
    std::vector<DetachedBody> final;
    for (double hz : {120., 240., 480.}) {
      auto s = component(def, hz);
      for (int i = 0; i < int(.5 * hz); ++i)
        s.step(left);
      final.push_back(s.debris[0]);
    }
    double coarse = norm(final[0].state.velocity - final[2].state.velocity),
           fine = norm(final[1].state.velocity - final[2].state.velocity);
    require(fine < coarse * .8,
            "piece flight improves with timestep refinement");
    // Real free-flight failure: section weakening, then actual full-authority
    // pull.
    Simulation flight(def);
    auto trim = flight.trim(140, 1000);
    auto intent = trim.intent;
    Body ballistic;
    MassProperties ballistic_mass;
    bool detached = false;
    double first = -1, separation = -1;
    std::ofstream trace;
    if (argc > 2) {
      trace.open(argv[2]);
      trace << "time,x,y,z,vx,vy,vz,p,q,r,fx,fy,fz,ballistic_z\n";
    }
    for (int i = 0; i < 1200; ++i) {
      double t = i * flight.dt;
      if (i == 120)
        flight.weaken("left_root_front", .3);
      intent.active[Pitch] = t >= 3 && t < 5;
      intent.direct[Pitch] = 1;
      if (detached)
        integrate_body(ballistic, ballistic_mass, Wrench{}, flight.dt);
      flight.step(intent);
      if (first < 0 && std::any_of(
                           flight.def.edges.begin(), flight.def.edges.end(),
                           [](auto &e) { return e.failed; }))
        first = (i + 1) * flight.dt;
      if (!detached && !flight.debris.empty()) {
        detached = true;
        separation = (i + 1) * flight.dt;
        ballistic = flight.debris[0].state;
        ballistic_mass = flight.debris[0].mass;
      }
      if (detached && trace && i % 12 == 0) {
        auto &d = flight.debris[0];
        auto x = d.state.position, vv = d.state.velocity, w = d.state.omega,
             ff = d.aero.force;
        trace << std::setprecision(13) << (i + 1) * flight.dt << ',' << x.x
              << ',' << x.y << ',' << x.z << ',' << vv.x << ',' << vv.y << ','
              << vv.z << ',' << w.x << ',' << w.y << ',' << w.z << ',' << ff.x
              << ',' << ff.y << ',' << ff.z << ',' << ballistic.position.z
              << '\n';
      }
    }
    require(first > 3 && separation > first,
            "low-load survival then delayed load-driven graph failure");
    double distance =
        norm(flight.debris[0].state.position - ballistic.position);
    require(distance > 1 && std::isfinite(distance),
            "separated body flight differs from ballistic continuation");
    std::cout << std::setprecision(13)
              << "{\"checks_passed\":true,\"force_partition_error_N\":"
              << force_error
              << ",\"moment_partition_error_Nm\":" << moment_error
              << ",\"cg_density_sample_force_jump_N\":" << density_jump
              << ",\"passive_power_W\":" << power
              << ",\"component_energy_loss_J\":" << before - energy(pa)
              << ",\"coarse_velocity_error\":" << coarse
              << ",\"fine_velocity_error\":" << fine
              << ",\"refinement_ratio\":" << fine / coarse
              << ",\"first_failure_s\":" << first
              << ",\"separation_s\":" << separation
              << ",\"distance_from_ballistic_at_10s_m\":" << distance
              << ",\"high_speed_trim_converged\":"
              << (trim.converged ? "true" : "false") << "}\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
