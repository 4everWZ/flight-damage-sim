#include "flight/core.hpp"
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>
namespace flight {
namespace {
MassProperties properties(const Definition &d, const std::vector<int> &nodes) {
  MassProperties m;
  for (auto &p : d.masses)
    if (std::find(nodes.begin(), nodes.end(), p.node) != nodes.end()) {
      m.mass += p.mass;
      m.cg += p.position * p.mass;
    }
  if (m.mass <= 0)
    throw std::runtime_error("empty mass group");
  m.cg = m.cg / m.mass;
  for (auto &p : d.masses)
    if (std::find(nodes.begin(), nodes.end(), p.node) != nodes.end()) {
      m.inertia += Mat::diagonal(p.inertia);
      m.inertia += parallel_axis(p.mass, p.position - m.cg);
    }
  return m;
}
std::vector<int> reachable(const Definition &d, int root) {
  std::vector<int> r{root};
  for (std::size_t i = 0; i < r.size(); ++i)
    for (auto &e : d.edges)
      if (!e.failed) {
        int n = e.a == r[i] ? e.b : (e.b == r[i] ? e.a : -1);
        if (n >= 0 && std::find(r.begin(), r.end(), n) == r.end())
          r.push_back(n);
      }
  return r;
}
std::vector<double> linear_solve(std::vector<std::vector<double>> a,
                                 std::vector<double> b) {
  int n = int(b.size());
  for (int i = 0; i < n; ++i) {
    int p = i;
    for (int j = i + 1; j < n; ++j)
      if (std::abs(a[j][i]) > std::abs(a[p][i]))
        p = j;
    if (std::abs(a[p][i]) < 1e-12)
      throw std::runtime_error("singular structural graph");
    std::swap(a[i], a[p]);
    std::swap(b[i], b[p]);
    double div = a[i][i];
    for (int k = i; k < n; ++k)
      a[i][k] /= div;
    b[i] /= div;
    for (int j = 0; j < n; ++j)
      if (j != i) {
        double f = a[j][i];
        for (int k = i; k < n; ++k)
          a[j][k] -= f * a[i][k];
        b[j] -= f * b[i];
      }
  }
  return b;
}
CurvePoint lookup(const Definition &d, const AeroCell &c, double alpha) {
  auto it = std::find_if(d.curves.begin(), d.curves.end(),
                         [&](const auto &p) { return p.first == c.table; });
  if (it == d.curves.end())
    throw std::runtime_error("unknown aero table: " + c.table);
  const auto &tab = it->second;
  alpha = clamp(alpha, tab.front().alpha, tab.back().alpha);
  auto upper = std::upper_bound(
      tab.begin(), tab.end(), alpha,
      [](double a, const CurvePoint &p) { return a < p.alpha; });
  if (upper == tab.begin())
    return *upper;
  if (upper == tab.end())
    return tab.back();
  auto lo = upper - 1;
  double w = (alpha - lo->alpha) / (upper->alpha - lo->alpha);
  return {alpha, lo->cl + w * (upper->cl - lo->cl),
          lo->cd + w * (upper->cd - lo->cd), lo->cm + w * (upper->cm - lo->cm)};
}
Quat orientation(double alpha) { return Quat::axis({0, 1, 0}, alpha); }
} // namespace
Definition Definition::load(const std::string &path) {
  Definition d;
  std::ifstream file(path);
  if (!file)
    throw std::runtime_error("cannot read airframe: " + path);
  std::string line;
  int line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    std::istringstream s(line);
    std::string tag;
    s >> tag;
    if (tag.empty() || tag[0] == '#')
      continue;
    if (tag == "mass") {
      MassPart p;
      s >> p.node >> p.mass >> p.position.x >> p.position.y >> p.position.z >>
          p.inertia.x >> p.inertia.y >> p.inertia.z;
      d.masses.push_back(p);
    } else if (tag == "node") {
      StructuralNode n;
      s >> n.name;
      d.nodes.push_back(n);
    } else if (tag == "edge") {
      StructuralEdge e;
      s >> e.name >> e.a >> e.b >> e.bend_k >> e.twist_k >> e.section_modulus >>
          e.torsion_modulus >> e.yield_stress >> e.ultimate_stress >>
          e.failure_strain >> e.plastic_rate;
      d.edges.push_back(e);
    } else if (tag == "chain") {
      ControlChain c;
      int axis;
      double lim, rate;
      s >> c.name >> axis >> lim >> rate >> c.q_limit;
      c.axis = Axis(axis);
      c.limit = rad(lim);
      c.slew = rad(rate);
      d.chains.push_back(c);
    } else if (tag == "cell") {
      AeroCell c;
      double inc;
      s >> c.id >> c.node >> c.chain >> c.plane >> c.position.x >>
          c.position.y >> c.position.z >> c.area >> c.chord >> c.span >> inc >>
          c.control_sign >> c.control_gain >> c.table;
      c.incidence = rad(inc);
      d.cells.push_back(c);
    } else if (tag == "curve") {
      std::string name;
      CurvePoint p;
      double deg;
      s >> name >> deg >> p.cl >> p.cd >> p.cm;
      p.alpha = rad(deg);
      auto it = std::find_if(d.curves.begin(), d.curves.end(),
                             [&](auto &v) { return v.first == name; });
      if (it == d.curves.end()) {
        d.curves.push_back({name, {}});
        it = d.curves.end() - 1;
      }
      it->second.push_back(p);
    } else if (tag == "engine") {
      auto &e = d.engine;
      s >> e.max_power >> e.rated_rpm >> e.prop_radius >> e.rotor_inertia >>
          e.ct >> e.cq >> e.advance_loss >> e.altitude_scale >> e.response;
      e.rpm = e.rated_rpm;
    } else if (tag == "prop_position") {
      s >> d.engine.position.x >> d.engine.position.y >> d.engine.position.z;
    } else if (tag == "body_drag")
      s >> d.body_drag_area;
    else if (tag == "handoff")
      s >> d.handoff;
    else if (tag == "trim") {
      int r, p, y;
      s >> r >> p >> y;
      d.trim_enabled = {bool(r), bool(p), bool(y)};
    } else
      throw std::runtime_error("unknown definition row " +
                               std::to_string(line_no));
    if (!s)
      throw std::runtime_error("malformed definition row " +
                               std::to_string(line_no));
  }
  if (d.nodes.empty() || d.masses.empty() || d.cells.empty() ||
      d.engine.max_power <= 0 || d.handoff <= 0)
    throw std::runtime_error("incomplete airframe");
  auto node_valid = [&](int n) { return n >= 0 && n < int(d.nodes.size()); };
  for (auto &e : d.edges)
    if (!node_valid(e.a) || !node_valid(e.b) || e.a == e.b || e.bend_k <= 0 ||
        e.twist_k <= 0 || e.section_modulus <= 0 || e.torsion_modulus <= 0 ||
        e.yield_stress <= 0 || e.ultimate_stress <= e.yield_stress ||
        e.failure_strain <= 0 || e.plastic_rate <= 0)
      throw std::runtime_error("invalid structural edge");
  for (auto &m : d.masses)
    if (!node_valid(m.node) || m.mass <= 0 || m.inertia.x <= 0 ||
        m.inertia.y <= 0 || m.inertia.z <= 0)
      throw std::runtime_error("invalid mass");
  for (auto &c : d.chains)
    if (c.axis < Roll || c.axis > Yaw || c.limit <= 0 || c.slew <= 0 ||
        c.q_limit <= 0)
      throw std::runtime_error("invalid chain");
  std::vector<int> ids;
  for (auto &c : d.cells) {
    if (!node_valid(c.node) || c.area <= 0 || c.chord <= 0 || c.span <= 0 ||
        c.chain < -1 || c.chain >= int(d.chains.size()) ||
        (c.plane != 0 && c.plane != 1) ||
        std::find(ids.begin(), ids.end(), c.id) != ids.end())
      throw std::runtime_error("invalid aero cell");
    ids.push_back(c.id);
  }
  for (auto &tab : d.curves) {
    if (tab.second.size() < 3)
      throw std::runtime_error("aero curve needs 3+ points");
    for (std::size_t i = 1; i < tab.second.size(); ++i)
      if (tab.second[i].alpha <= tab.second[i - 1].alpha)
        throw std::runtime_error("unsorted aero curve");
  }
  if (reachable(d, 0).size() != d.nodes.size())
    throw std::runtime_error("initial structure disconnected");
  for (auto &c : d.cells)
    (void)lookup(d, c, 0);
  return d;
}
Simulation::Simulation(Definition d, double timestep)
    : def(std::move(d)), dt(timestep) {
  if (!std::isfinite(dt) || dt <= 0 || dt > 1. / 60)
    throw std::runtime_error("unsupported timestep");
  std::vector<int> nodes(def.nodes.size());
  std::iota(nodes.begin(), nodes.end(), 0);
  mass = properties(def, nodes);
  node_bending.resize(nodes.size());
  node_torsion.resize(nodes.size());
}
void Simulation::update_controls(const PilotIntent &p) {
  Vec target = body.attitude.conjugate().rotate(unit(p.aim_world));
  instructor.aim_body = target;
  // Guidance: bank toward horizontal aim error; pitch rate aligns flight path
  // and supports a banked turn.
  double heading_error = std::atan2(target.y, target.x),
         pitch_error = std::atan2(-target.z, std::hypot(target.x, target.y));
  Vec down = body.attitude.conjugate().rotate({0, 0, 1});
  double bank = std::atan2(down.y, down.z);
  Vec vb = body.attitude.conjugate().rotate(body.velocity - wind_world);
  double speed = std::max(norm(vb), 15.), alpha = std::atan2(vb.z, vb.x),
         beta = std::atan2(vb.y, std::hypot(vb.x, vb.z));
  instructor.desired_bank = clamp(1.2 * heading_error, -rad(65), rad(65));
  instructor.desired_rates = {
      clamp(2. * (instructor.desired_bank - bank), -1.4, 1.4),
      clamp(1.3 * (pitch_error + alpha) +
                gravity / speed * (1. / std::max(std::cos(bank), .4) - 1),
            -.6, .6),
      clamp(.4 * heading_error - 1.5 * beta, -.4, .4)};
  if (alpha > rad(13))
    instructor.desired_rates.y =
        std::min(instructor.desired_rates.y, -2 * (alpha - rad(13)));
  std::array<double, 3> gains{.9, 1.4, 1.0};
  for (int axis = 0; axis < 3; ++axis) {
    double ic = p.mouse ? clamp(gains[axis] * (instructor.desired_rates[axis] -
                                               body.omega[axis]),
                                -1, 1)
                        : 0;
    instructor.command[axis] = ic;
    auto &a = ownership[axis];
    if (p.active[axis]) {
      a.direct_weight = 1;
      a.last_direct = clamp(p.direct[axis], -1, 1);
      a.release_from = a.last_direct;
      a.release_elapsed = 0;
      a.command = a.last_direct;
    } else {
      if (a.was_active)
        a.release_elapsed = 0;
      else
        a.release_elapsed = std::min(def.handoff, a.release_elapsed + dt);
      double t = clamp(a.release_elapsed / def.handoff, 0, 1);
      a.direct_weight = 1 - t * t * (3 - 2 * t);
      a.command = a.direct_weight * a.release_from + (1 - a.direct_weight) * ic;
    }
    a.was_active = p.active[axis];
  }
  double q = .5 * density * dot(vb, vb);
  for (auto &c : def.chains) {
    double direct_command = ownership[c.axis].command;
    double command =
        direct_command + (def.trim_enabled[c.axis]
                              ? p.trim[c.axis] * (1 - std::abs(direct_command))
                              : 0);
    command = clamp(command, -1, 1);
    if (c.jammed) {
      c.deflection = clamp(c.jam_position, -c.limit, c.limit);
      c.filtered = c.deflection;
      continue;
    }
    double target_def =
        c.connected ? command * c.limit * clamp(c.range, 0, 1) : 0;
    if (c.lag > 0)
      c.filtered += (target_def - c.filtered) * (1 - std::exp(-dt / c.lag));
    else
      c.filtered = target_def;
    // Mechanical effort limits response/range as hinge load rises; disconnected
    // surfaces relax to zero.
    double effort = 1 / (1 + q / c.q_limit);
    double max_step = c.slew * effort * dt / (1 + std::max(c.friction, 0.));
    c.deflection +=
        clamp(c.filtered * effort - c.deflection, -max_step, max_step);
  }
}
void Simulation::update_engine(const PilotIntent &p) {
  auto &e = def.engine;
  double v = std::max(
      body.attitude.conjugate().rotate(body.velocity - wind_world).x, 0.);
  density = 1.225 * std::exp(body.position.z / 8500.);
  e.throttle_state += (clamp(p.throttle, 0, 1) - e.throttle_state) *
                      (1 - std::exp(-dt / e.response));
  double omega = std::max(e.rpm * 2 * pi / 60, 0.);
  double rpm_factor = clamp(e.rpm / e.rated_rpm, 0, 1.15);
  e.shaft_power =
      e.running ? e.max_power * e.throttle_state * e.power_fraction *
                      std::exp(body.position.z / e.altitude_scale) * rpm_factor
                : 0;
  double n = e.rpm / 60, diameter = 2 * e.prop_radius,
         J = v / std::max(n * diameter, 1.);
  double prop_q =
      e.cq * density * n * n * std::pow(diameter, 5) * e.prop_fraction;
  double engine_q = e.shaft_power / std::max(omega, 20.);
  omega = std::max(0., omega + (engine_q - prop_q) * dt / e.rotor_inertia);
  e.rpm = omega * 60 / (2 * pi);
  n = e.rpm / 60;
  J = v / std::max(n * diameter, 1.);
  e.thrust = e.ct * density * n * n * std::pow(diameter, 4) *
             (1 - e.advance_loss * J) * e.prop_fraction;
  e.torque = e.shaft_power / std::max(omega, 20.);
}
Wrench Simulation::evaluate(const Body &b, bool store) {
  double rho = 1.225 * std::exp(b.position.z / 8500.);
  Vec vb = b.attitude.conjugate().rotate(b.velocity - wind_world);
  Wrench a;
  for (auto &c : def.cells) {
    if (!c.attached)
      continue;
    Vec r = c.position - mass.cg, local = vb + cross(b.omega, r);
    // Axial actuator-disk slipstream, limited to the propeller/tail footprint.
    if (c.position.x < def.engine.position.x &&
        std::hypot(c.position.y - def.engine.position.y,
                   c.position.z - def.engine.position.z) <
            def.engine.prop_radius) {
      double disk = pi * def.engine.prop_radius * def.engine.prop_radius;
      local.x +=
          .5 * (std::sqrt(vb.x * vb.x +
                          2 * std::max(def.engine.thrust, 0.) / (rho * disk)) -
                std::abs(vb.x));
    }
    double speed = norm(local), q = .5 * rho * speed * speed;
    Vec drag_dir = -unit(local);
    double raw_alpha = c.plane == 0 ? std::atan2(local.z, local.x)
                                    : std::atan2(local.y, local.x);
    double deflection = c.chain >= 0
                            ? def.chains[c.chain].deflection *
                                  def.chains[c.chain].hinge * c.control_sign
                            : 0;
    double alpha = raw_alpha + c.incidence + def.nodes[c.node].twist +
                   c.control_gain * deflection;
    auto coeff = lookup(def, c, alpha);
    double mach = speed / 340.;
    double compression = 1. / std::sqrt(std::max(1 - mach * mach, .35));
    double effective_area = c.area * c.area_fraction;
    double lift =
        q * effective_area * coeff.cl * c.lift_efficiency * compression;
    // Gap drag persists on the missing skin footprint, independently of its
    // lost lifting area.
    double drag = q * (effective_area * coeff.cd + c.area * c.roughness * .08);
    Vec lift_dir = c.plane == 0 ? unit(Vec{local.z, 0, -local.x})
                                : unit(Vec{local.y, -local.x, 0});
    if (c.plane == 0)
      lift_dir = Quat::axis({1, 0, 0}, def.nodes[c.node].bend).rotate(lift_dir);
    Vec force = lift_dir * lift + drag_dir * drag;
    Vec moment = c.plane == 0
                     ? Vec{0, q * effective_area * c.chord * coeff.cm, 0}
                     : Vec{0, 0, q * effective_area * c.chord * coeff.cm};
    a.add(force, r, moment);
    if (store) {
      c.flow = local;
      c.alpha = alpha;
      c.beta = std::atan2(local.y, std::hypot(local.x, local.z));
      c.q = q;
      c.stall = std::abs(alpha) > rad(15) ? 1 : 0;
      c.force = force;
      c.moment = moment;
      c.deflection = deflection;
    }
  }
  a.add(-unit(vb) * (.5 * rho * dot(vb, vb) * def.body_drag_area), -mass.cg);
  Wrench prop;
  prop.add({def.engine.thrust, 0, 0}, def.engine.position - mass.cg,
           {-def.engine.torque, 0, 0});
  // First-order asymmetric blade-loading approximation: thrust offset grows
  // with disk incidence.
  prop.moment.z -= def.engine.thrust * .15 * def.engine.prop_radius *
                   std::sin(std::atan2(vb.z, vb.x));
  if (store) {
    aero = a;
    propulsion = prop;
    load_factor = -a.force.z / (mass.mass * gravity);
  }
  a.force += prop.force;
  a.moment += prop.moment;
  return a;
}
void integrate_body(Body &b, const MassProperties &m, const Wrench &force,
                    double elapsed) {
  Vec alpha =
      m.inertia.solve(force.moment - cross(b.omega, m.inertia * b.omega));
  Vec mid_omega = b.omega + alpha * (elapsed / 2);
  Quat mid = (b.attitude + b.attitude *
                               Quat{0, mid_omega.x, mid_omega.y, mid_omega.z} *
                               (elapsed / 4))
                 .normalized();
  Vec accel = mid.rotate(force.force) / m.mass + Vec{0, 0, gravity};
  b.position += b.velocity * elapsed + accel * (.5 * elapsed * elapsed);
  b.velocity += accel * elapsed;
  b.attitude = (b.attitude +
                b.attitude * Quat{0, mid_omega.x, mid_omega.y, mid_omega.z} *
                    (elapsed / 2))
                   .normalized();
  b.omega += alpha * elapsed;
}
void Simulation::solve_structure(const std::vector<double> &bending,
                                 const std::vector<double> &torsion,
                                 double elapsed) {
  if (bending.size() != def.nodes.size() || torsion.size() != bending.size())
    throw std::runtime_error("invalid nodal loads");
  auto connected = reachable(def, 0);
  std::vector<int> index(def.nodes.size(), -1);
  int n = 0;
  for (int id : connected)
    if (id != 0)
      index[id] = n++;
  auto solve = [&](bool twist) {
    std::vector<std::vector<double>> k(n, std::vector<double>(n));
    std::vector<double> f(n);
    for (int id : connected)
      if (id != 0)
        f[index[id]] = twist ? torsion[id] : bending[id];
    for (auto &e : def.edges)
      if (!e.failed && def.nodes[e.a].attached && def.nodes[e.b].attached) {
        int i = index[e.a], j = index[e.b];
        double stiff = (twist ? e.twist_k : e.bend_k) * e.stiffness();
        if (i >= 0)
          k[i][i] += stiff;
        if (j >= 0)
          k[j][j] += stiff;
        if (i >= 0 && j >= 0) {
          k[i][j] -= stiff;
          k[j][i] -= stiff;
        }
      }
    auto x = linear_solve(k, f);
    for (int id : connected) {
      double v = id == 0 ? 0 : x[index[id]];
      if (twist)
        def.nodes[id].twist = v;
      else
        def.nodes[id].bend = v;
    }
  };
  solve(false);
  solve(true);
  for (auto &e : def.edges)
    if (!e.failed && def.nodes[e.a].attached && def.nodes[e.b].attached) {
      e.bend_load = e.bend_k * e.stiffness() *
                    (def.nodes[e.b].bend - def.nodes[e.a].bend);
      e.twist_load = e.twist_k * e.stiffness() *
                     (def.nodes[e.b].twist - def.nodes[e.a].twist);
      double zscale = std::pow(std::max(e.remaining_section, 1e-6), 1.5);
      double stress = std::hypot(e.bend_load / (e.section_modulus * zscale),
                                 e.twist_load / (e.torsion_modulus * zscale));
      e.utilization = stress / e.ultimate_stress;
      // Accumulated plastic strain above yield; overload accelerates strain
      // until ductility is exhausted.
      e.plastic_strain +=
          elapsed * e.plastic_rate *
          std::pow(std::max(stress / e.yield_stress - 1., 0.), 2);
      if (e.plastic_strain >= e.failure_strain)
        e.failed = true;
    }
  update_mass_and_detach();
}
void Simulation::update_mass_and_detach() {
  auto still = reachable(def, 0);
  std::vector<int> newly;
  for (int i = 1; i < int(def.nodes.size()); ++i)
    if (def.nodes[i].attached &&
        std::find(still.begin(), still.end(), i) == still.end())
      newly.push_back(i);
  if (newly.empty())
    return;
  Vec old_cg = mass.cg;
  while (!newly.empty()) {
    auto group = reachable(def, newly.front());
    group.erase(std::remove_if(group.begin(), group.end(),
                               [&](int id) {
                                 return std::find(newly.begin(), newly.end(),
                                                  id) == newly.end();
                               }),
                group.end());
    DetachedBody piece;
    piece.mass = properties(def, group);
    piece.state = body;
    Vec offset = piece.mass.cg - old_cg;
    piece.state.position += body.attitude.rotate(offset);
    piece.state.velocity += body.attitude.rotate(cross(body.omega, offset));
    piece.nodes = group;
    for (int id : group) {
      def.nodes[id].attached = false;
      newly.erase(std::remove(newly.begin(), newly.end(), id), newly.end());
    }
    for (int i = 0; i < int(def.cells.size()); ++i)
      if (std::find(group.begin(), group.end(), def.cells[i].node) !=
          group.end()) {
        def.cells[i].attached = false;
        def.cells[i].force = {};
        def.cells[i].moment = {};
        def.cells[i].q = 0;
        def.cells[i].stall = 0;
        piece.cells.push_back(i);
      }
    debris.push_back(piece);
  }
  mass = properties(def, still);
  Vec offset = mass.cg - old_cg;
  body.position += body.attitude.rotate(offset);
  body.velocity += body.attitude.rotate(cross(body.omega, offset));
  // Every piece inherits the pre-separation rigid velocity field; aggregate
  // linear/angular momentum is conserved at the split.
}
void Simulation::assemble_structural_loads(const Wrench &forces) {
  std::fill(node_bending.begin(), node_bending.end(), 0);
  std::fill(node_torsion.begin(), node_torsion.end(), 0);
  for (auto &c : def.cells)
    if (c.attached) {
      Vec local_moment = cross(c.position, c.force) + c.moment;
      node_bending[c.node] += local_moment.x;
      node_torsion[c.node] += local_moment.y;
    }
  // D'Alembert inertial relief: structural mass shares translational and
  // rotational acceleration. Uniform gravity cancels against the gravity
  // contribution to rigid-body acceleration.
  Vec angular_accel = mass.inertia.solve(
      forces.moment - cross(body.omega, mass.inertia * body.omega));
  for (const auto &part : def.masses)
    if (def.nodes[part.node].attached) {
      Vec r = part.position - mass.cg;
      Vec relief = -(forces.force / mass.mass + cross(angular_accel, r) +
                     cross(body.omega, cross(body.omega, r))) *
                   part.mass;
      Vec moment = cross(part.position, relief);
      node_bending[part.node] += moment.x;
      node_torsion[part.node] += moment.y;
    }
}
void Simulation::step(const PilotIntent &p) {
  update_engine(p);
  update_controls(p);
  auto forces = evaluate(body, true);
  assemble_structural_loads(forces);
  // Existing detached pieces advance once; new pieces inherit the end-of-tick
  // state. Per-piece aero evaluation is a later task; currently pieces coast
  // ballistically.
  for (auto &d : debris)
    integrate_body(d.state, d.mass, Wrench{}, dt);
  integrate_body(body, mass, forces, dt);
  solve_structure(node_bending, node_torsion, dt);
  ++tick;
}
void Simulation::damage_cell(int id, double area, double roughness) {
  if (!std::isfinite(area) || !std::isfinite(roughness))
    throw std::runtime_error("non-finite aero damage");
  auto c = std::find_if(def.cells.begin(), def.cells.end(),
                        [&](auto &x) { return x.id == id; });
  if (c == def.cells.end())
    throw std::runtime_error("unknown cell");
  c->area_fraction = clamp(area, 0, 1);
  c->roughness = std::max(roughness, 0.);
}
void Simulation::weaken(const std::string &name, double section) {
  auto e = std::find_if(def.edges.begin(), def.edges.end(),
                        [&](auto &x) { return x.name == name; });
  if (e == def.edges.end() || !std::isfinite(section) || section <= 0 ||
      section > 1)
    throw std::runtime_error("invalid section event");
  e->remaining_section = section;
}
void Simulation::disconnect(const std::string &name) {
  auto c = std::find_if(def.chains.begin(), def.chains.end(),
                        [&](auto &x) { return x.name == name; });
  if (c == def.chains.end())
    throw std::runtime_error("unknown chain");
  c->connected = false;
}
void Simulation::reset_trim(const TrimResult &t, double speed,
                            double altitude) {
  body = Body{};
  body.position.z = -altitude;
  body.velocity = {speed, 0, 0};
  body.attitude =
      (orientation(t.alpha) * Quat::axis({1, 0, 0}, t.bank)).normalized();
  for (auto &c : def.chains)
    c.deflection = clamp(t.intent.trim[c.axis], -1, 1) * c.limit /
                   (1 + .5 * 1.225 * std::exp(-altitude / 8500) * speed *
                            speed / c.q_limit);
  def.engine.throttle_state = t.intent.throttle;
  // Solve shaft/prop torque equilibrium for the trim's throttle.
  auto &e = def.engine;
  double rho = 1.225 * std::exp(-altitude / 8500), D = 2 * e.prop_radius;
  double power =
      e.max_power * t.intent.throttle * std::exp(-altitude / e.altitude_scale);
  e.rpm = std::sqrt(std::max(power, 0.) * 60 /
                    (2 * pi * e.rated_rpm * e.cq * rho * std::pow(D, 5))) *
          60;
  e.rpm = std::min(e.rpm, e.rated_rpm * 1.15);
  update_engine(t.intent);
}
TrimResult Simulation::trim(double speed, double altitude) {
  // Six residuals: alpha, roll/pitch/yaw trim, throttle, bank. Use
  // finite-difference Newton on the same cell model.
  if (speed <= 10 || altitude < 0)
    throw std::runtime_error("invalid trim condition");
  std::array<double, 6> x{rad(3), 0, 0, 0, .55, 0};
  TrimResult out;
  auto residual = [&](const std::array<double, 6> &v) {
    TrimResult t;
    t.alpha = v[0];
    t.bank = v[5];
    t.intent.trim = {v[1], v[2], v[3]};
    t.intent.throttle = v[4];
    reset_trim(t, speed, altitude);
    body.attitude =
        (orientation(v[0]) * Quat::axis({1, 0, 0}, v[5])).normalized();
    // Include the same elastic feedback used during flight, without advancing
    // material strain.
    for (int elastic = 0; elastic < 6; ++elastic) {
      auto trial = evaluate(body, true);
      assemble_structural_loads(trial);
      solve_structure(node_bending, node_torsion, 0);
    }
    auto w = evaluate(body);
    Vec gravity_body =
        body.attitude.conjugate().rotate({0, 0, mass.mass * gravity});
    Vec f = w.force + gravity_body;
    return std::vector<double>{
        f.x / (mass.mass * gravity), f.y / (mass.mass * gravity),
        f.z / (mass.mass * gravity), w.moment.x / 10000,
        w.moment.y / 10000,          w.moment.z / 10000};
  };
  for (int iter = 0; iter < 35; ++iter) {
    auto f = residual(x);
    double err = 0;
    for (double v : f)
      err += v * v;
    out.residual = std::sqrt(err);
    out.iterations = iter;
    if (out.residual < 1e-8)
      break;
    std::vector<std::vector<double>> J(6, std::vector<double>(6));
    for (int j = 0; j < 6; ++j) {
      auto y = x;
      y[j] += 1e-5;
      auto fy = residual(y);
      for (int i = 0; i < 6; ++i)
        J[i][j] = (fy[i] - f[i]) / 1e-5;
    }
    for (double &v : f)
      v = -v;
    auto dx = linear_solve(J, f);
    for (int j = 0; j < 6; ++j)
      x[j] += clamp(dx[j], -.2, .2);
    x[0] = clamp(x[0], rad(-5), rad(13));
    for (int j = 1; j < 4; ++j)
      x[j] = clamp(x[j], -.9, .9);
    x[4] = clamp(x[4], .01, 1);
    x[5] = clamp(x[5], -.15, .15);
  }
  auto final_residual = residual(x);
  out.residual = 0;
  for (double v : final_residual)
    out.residual += v * v;
  out.residual = std::sqrt(out.residual);
  out.alpha = x[0];
  out.bank = x[5];
  out.intent.trim = {x[1], x[2], x[3]};
  out.intent.throttle = x[4];
  body.attitude =
      (orientation(x[0]) * Quat::axis({1, 0, 0}, x[5])).normalized();
  evaluate(body, true);
  return out;
}
} // namespace flight
