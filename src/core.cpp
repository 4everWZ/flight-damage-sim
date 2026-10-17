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
      m.mass += p.mass * p.remaining;
      m.cg += p.position * (p.mass * p.remaining);
    }
  if (m.mass <= 0)
    throw std::runtime_error("empty mass group");
  m.cg = m.cg / m.mass;
  for (auto &p : d.masses)
    if (std::find(nodes.begin(), nodes.end(), p.node) != nodes.end()) {
      m.inertia += Mat::diagonal(p.inertia * p.remaining);
      m.inertia += parallel_axis(p.mass * p.remaining, p.position - m.cg);
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
Quat cell_frame(const Definition &def, const AeroCell &c) {
  Quat incidence = c.plane == 0 ? Quat::axis({0, 1, 0}, c.incidence)
                                : Quat::axis({0, 0, 1}, -c.incidence);
  return Quat::axis({1, 0, 0}, def.nodes[c.node].bend) *
         Quat::axis({0, 1, 0}, def.nodes[c.node].twist) * incidence;
}
Wrench cell_load(const Definition &def, AeroCell &c, Vec local, Vec r,
                 double rho, Quat frame, double deflection, bool store) {
  double speed = norm(local), q = .5 * rho * speed * speed;
  Vec drag_dir = -unit(local);
  Vec chord = frame.rotate({1, 0, 0});
  Vec span = frame.rotate(c.plane == 0 ? Vec{0, 1, 0} : Vec{0, 0, -1});
  Vec normal = frame.rotate(c.plane == 0 ? Vec{0, 0, 1} : Vec{0, 1, 0});
  double raw_alpha = std::atan2(dot(local, normal), dot(local, chord));
  double alpha = raw_alpha + c.control_gain * deflection;
  auto coeff = lookup(def, c, alpha);
  double mach = speed / 340.;
  double compression = 1. / std::sqrt(std::max(1 - mach * mach, .35));
  double effective_area = c.area * c.area_fraction;
  double lift = q * effective_area * coeff.cl * c.lift_efficiency * compression;
  // Gap drag persists on the missing skin footprint, independently of its
  // lost lifting area.
  double drag = q * (effective_area * coeff.cd + c.area * c.roughness * .08);
  Vec lift_dir = unit(cross(span, local));
  Vec force = lift_dir * lift + drag_dir * drag;
  Vec moment = span * (q * effective_area * c.chord * coeff.cm *
                       (c.plane == 0 ? 1 : -1));
  Wrench result;
  result.add(force, r, moment);
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
  return result;
}
double trimmed_command(double axis_command, double trim, bool enabled) {
  return clamp(axis_command +
                   (enabled ? trim * (1 - std::abs(axis_command)) : 0),
               -1, 1);
}
void advance_surface(ControlChain &c, double command, double q,
                     double elapsed) {
  if (c.jammed) {
    c.deflection = c.filtered = clamp(c.jam_position, -c.limit, c.limit);
    return;
  }
  double target = c.connected ? command * c.limit * clamp(c.range, 0, 1) : 0;
  if (c.lag > 0)
    c.filtered += (target - c.filtered) * (1 - std::exp(-elapsed / c.lag));
  else
    c.filtered = target;
  double effort = 1 / (1 + q / c.q_limit);
  double travel = c.slew * effort * elapsed / (1 + std::max(c.friction, 0.));
  c.deflection += clamp(c.filtered * effort - c.deflection, -travel, travel);
}
void sample_surface(ControlChain &c, double command, double q, double elapsed) {
  // Both samples start from the same actual linkage state. Advancing twice by
  // half a step would change the existing lag+slew endpoint approximation.
  auto midpoint = c;
  advance_surface(midpoint, command, q, elapsed * .5);
  advance_surface(c, command, q, elapsed);
  c.midpoint_deflection = midpoint.deflection;
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
    if (tag == "mass" || tag == "material") {
      MassPart p;
      s >> p.node;
      if (tag == "material")
        s >> p.material_name;
      s >> p.mass >> p.position.x >> p.position.y >> p.position.z >>
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
    } else if (tag == "propeller") {
      std::string model;
      int blades, radial, azimuth;
      double pitch, low, high, slew;
      s >> model >> blades >> radial >> azimuth >> pitch >> low >> high >>
          slew >> d.engine.pump_rpm >> d.engine.engine_friction;
      if (model != "blade_element")
        throw std::runtime_error("unknown propeller model");
      auto &e = d.engine;
      e.model = PropellerModel::BladeElement;
      e.blades = blades;
      e.radial_samples = radial;
      e.azimuth_samples = azimuth;
      e.pitch = rad(pitch);
      e.pitch_min = rad(low);
      e.pitch_max = rad(high);
      e.pitch_slew = rad(slew);
    } else if (tag == "blade") {
      BladeStation b;
      double twist;
      s >> b.radius_fraction >> b.chord >> twist;
      b.twist = rad(twist);
      d.engine.stations.push_back(b);
    } else if (tag == "prop_polar") {
      CurvePoint p;
      double a;
      s >> a >> p.cl >> p.cd;
      p.alpha = rad(a);
      d.engine.polar.push_back(p);
    } else if (tag == "prop_axis") {
      s >> d.engine.axis.x >> d.engine.axis.y >> d.engine.axis.z;
    } else if (tag == "governor") {
      int enabled;
      double kp, ki;
      s >> enabled >> d.engine.target_rpm >> kp >> ki;
      d.engine.governor = enabled;
      d.engine.governor_kp = rad(kp);
      d.engine.governor_ki = rad(ki);
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
  std::vector<std::string> materials;
  for (const auto &part : d.masses)
    if (!part.material_name.empty()) {
      if (std::find(materials.begin(), materials.end(), part.material_name) !=
          materials.end())
        throw std::runtime_error("duplicate material name");
      materials.push_back(part.material_name);
      bool carrier =
          std::any_of(d.masses.begin(), d.masses.end(), [&](const MassPart &p) {
            return p.node == part.node && p.material_name.empty();
          });
      if (!carrier)
        throw std::runtime_error(
            "removable material needs a non-removable node carrier");
      if (part.inertia.x > part.inertia.y + part.inertia.z ||
          part.inertia.y > part.inertia.x + part.inertia.z ||
          part.inertia.z > part.inertia.x + part.inertia.y)
        throw std::runtime_error(
            "material inertia violates triangle inequality");
    }
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
  prepare_propeller(d.engine);
  return d;
}
Simulation::Simulation(Definition d, double timestep)
    : def(std::move(d)), dt(timestep) {
  if (!std::isfinite(dt) || dt <= 0 || dt > 1. / 60)
    throw std::runtime_error("unsupported timestep");
  std::vector<int> nodes(def.nodes.size());
  std::iota(nodes.begin(), nodes.end(), 0);
  mass = properties(def, nodes);
  for (auto &axis : ownership)
    axis.release_elapsed = def.handoff;
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
  Vec aim = unit(p.aim_world);
  double desired_heading = std::atan2(aim.y, aim.x);
  if (p.mouse && instructor_mode == InstructorMode::FlightPath &&
      guidance.aim_rate_feedforward && std::hypot(aim.x, aim.y) > 1e-6) {
    if (instructor.aim_history_valid) {
      double delta = std::atan2(
          std::sin(desired_heading - instructor.previous_aim_heading),
          std::cos(desired_heading - instructor.previous_aim_heading));
      // A filtered difference follows moving aim without requiring a permanent
      // heading error. Filter before limiting: render-rate sample/hold input
      // otherwise loses most of its rate at each short simulation tick.
      constexpr double filter_time = .15;
      double decay = std::exp(-dt / filter_time);
      double ceiling = gravity / speed *
                       std::tan(clamp(guidance.max_bank, 0., rad(75)));
      instructor.aim_heading_rate = clamp(
          decay * instructor.aim_heading_rate + (1 - decay) * delta / dt,
          -ceiling, ceiling);
    } else
      instructor.aim_heading_rate = 0;
    instructor.previous_aim_heading = desired_heading;
    instructor.aim_history_valid = true;
  } else {
    instructor.aim_history_valid = false;
    instructor.aim_heading_rate = 0;
  }
  if (instructor_mode == InstructorMode::Legacy) {
    instructor.desired_bank = clamp(1.2 * heading_error, -rad(65), rad(65));
    instructor.desired_rates = {
        clamp(2. * (instructor.desired_bank - bank), -1.4, 1.4),
        clamp(1.3 * (pitch_error + alpha) +
                  gravity / speed * (1. / std::max(std::cos(bank), .4) - 1),
              -.6, .6),
        clamp(.4 * heading_error - 1.5 * beta, -.4, .4)};
  } else if (instructor_mode == InstructorMode::FlightPath) {
    // Aim is an air-relative flight-path direction in world coordinates, not a
    // nose attitude or altitude target.
    Vec air_world = body.velocity - wind_world;
    double heading = std::atan2(air_world.y, air_world.x);
    double path =
        std::atan2(-air_world.z, std::hypot(air_world.x, air_world.y));
    double desired_path = std::atan2(-aim.z, std::hypot(aim.x, aim.y));
    double error = std::atan2(std::sin(desired_heading - heading),
                              std::cos(desired_heading - heading));
    instructor.flight_path_error = desired_path - path;
    double heading_rate = instructor.aim_heading_rate + guidance.heading_gain * error;
    instructor.desired_bank = clamp(std::atan(speed * heading_rate / gravity),
                                    -guidance.max_bank, guidance.max_bank);
    // The sensor remains aero body-Z/weight. Convert the path-normal request
    // into that same quantity, accounting for attitude and the other force
    // components. A force-balanced pitched aircraft does not measure 1 on
    // body Z, and propeller normal force must not be requested twice.
    Vec normal_world{-std::sin(path) * std::cos(heading),
                     -std::sin(path) * std::sin(heading), -std::cos(path)};
    Vec normal_body = body.attitude.conjugate().rotate(normal_world);
    Vec force = aero.force + propulsion.force;
    double weight = mass.mass * gravity;
    double other_support = (force.x * normal_body.x + force.y * normal_body.y) / weight;
    double bank_support = std::max(-normal_body.z, .3);
    instructor.desired_normal_g =
        clamp((std::cos(path) + speed * guidance.path_gain *
                                    instructor.flight_path_error / gravity - other_support) /
                  bank_support + propulsion.force.z / weight,
              -.5, 5.);
    // A normal-load loop requests body rate; the actuator and damaged
    // aerodynamics must supply it.
    double theta_rate =
        guidance.normal_gain * (instructor.desired_normal_g - load_factor);
    Vec forward = body.attitude.rotate({1, 0, 0});
    double theta = std::atan2(-forward.z, std::hypot(forward.x, forward.y));
    double turn_rate =
        gravity / speed * std::tan(clamp(bank, -rad(75), rad(75)));
    double bank_error = std::atan2(std::sin(instructor.desired_bank - bank),
                                   std::cos(instructor.desired_bank - bank));
    instructor.desired_rates = {
        clamp(2 * bank_error - turn_rate * std::sin(theta), -1.4, 1.4),
        clamp(theta_rate * std::cos(bank) +
                  turn_rate * std::sin(bank) * std::cos(theta),
              -.6, .6),
        clamp(-theta_rate * std::sin(bank) +
                  turn_rate * std::cos(bank) * std::cos(theta) + 1.2 * beta,
              -.4, .4)};
  } else {
    instructor.desired_rates = {};
    instructor.desired_bank = 0;
    instructor.desired_normal_g = 0;
    instructor.flight_path_error = 0;
  }
  if (alpha > rad(13))
    instructor.desired_rates.y =
        std::min(instructor.desired_rates.y, -2 * (alpha - rad(13)));
  std::array<double, 3> gains{.9, 1.4, 1.0};
  for (int axis = 0; axis < 3; ++axis) {
    double error = instructor.desired_rates[axis] - body.omega[axis];
    bool available = false;
    for (std::size_t id = 0; id < def.chains.size(); ++id) {
      const auto &chain = def.chains[id];
      if (chain.axis == axis && chain.connected && !chain.jammed &&
          chain.range > 0 && chain.hinge > 0) {
        for (const auto &cell : def.cells)
          if (cell.chain == int(id) && cell.attached && cell.area_fraction > 0)
            available = true;
      }
    }
    double raw = gains[axis] * error + instructor.rate_bias[axis];
    bool pause = !guidance.rate_integral || !p.mouse || p.active[axis] ||
                 ownership[axis].direct_weight > .01 || !available ||
                 (raw >= 1 && error > 0) || (raw <= -1 && error < 0) ||
                 (axis == Pitch && alpha > rad(13) && error > 0);
    instructor.integral_paused[axis] = pause;
    if (instructor_mode == InstructorMode::FlightPath &&
        guidance.rate_integral) {
      constexpr std::array<double, 3> integral_gain{.8, 1.2, .5};
      if (!pause) {
        double increment = integral_gain[axis] * error * dt;
        // Limit integration to the remaining command headroom within this tick.
        // A whole-step gate alone stores an unreachable, dt-dependent bias when
        // the command crosses saturation between two controller samples.
        if (increment > 0)
          increment = std::min(increment, std::max(1 - raw, 0.));
        else
          increment = std::max(increment, std::min(-1 - raw, 0.));
        instructor.rate_bias[axis] =
            clamp(instructor.rate_bias[axis] + increment, -.9, .9);
      }
    } else
      instructor.rate_bias[axis] = 0;
    raw = p.mouse && instructor_mode != InstructorMode::Off
              ? gains[axis] * error + instructor.rate_bias[axis]
              : 0;
    instructor.saturated[axis] = std::abs(raw) >= 1;
    double ic = clamp(raw, -1, 1);
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
    double command = trimmed_command(ownership[c.axis].command, p.trim[c.axis],
                                     def.trim_enabled[c.axis]);
    sample_surface(c, command, q, dt);
  }
}
void advance_propulsion(Engine &e, double density, double altitude, double v,
                        double throttle, double dt) {
  if (e.model == PropellerModel::BladeElement) {
    advance_propeller_controls(e, throttle, dt);
    double old = e.rpm * 2 * pi / 60, next = old;
    // Implicit scalar midpoint, same signed loads used in free flight.
    for (int it = 0; it < 12; ++it) {
      double mid = (old + next) * .5;
      auto loads = propeller_loads(e, density, e.axis * v, {}, mid);
      double q = engine_shaft_torque(e, altitude, mid);
      double residual =
          next - old - dt * (q - loads.shaft_load) / e.rotor_inertia;
      if (std::abs(residual) < 1e-10) {
        e.torque = q;
        e.aero_torque = loads.shaft_load;
        e.thrust = dot(loads.at_mount.force, e.axis);
        e.shaft_power = q * mid;
        e.induced_velocity = loads.induced;
        e.profile_loss = loads.profile_loss;
        e.induced_loss = loads.induced_loss;
        e.induction_residual = loads.momentum_residual;
        e.induction_converged = loads.converged;
        break;
      }
      double h = 1e-4;
      auto shifted = propeller_loads(e, density, e.axis * v, {}, mid + h);
      double qshift = engine_shaft_torque(e, altitude, mid + h);
      double derivative =
          1 - .5 * dt *
                  ((qshift - shifted.shaft_load) - (q - loads.shaft_load)) /
                  (h * e.rotor_inertia);
      next -= residual / derivative;
      if (it == 11)
        throw std::runtime_error("rotor midpoint did not converge");
    }
    e.acceleration = (next - old) / dt;
    e.rpm = next * 60 / (2 * pi);
    return;
  }
  e.throttle_state += (clamp(throttle, 0, 1) - e.throttle_state) *
                      (1 - std::exp(-dt / e.response));
  double omega = std::max(e.rpm * 2 * pi / 60, 0.);
  double rpm_factor = clamp(e.rpm / e.rated_rpm, 0, 1.15);
  e.shaft_power = e.running
                      ? e.max_power * e.throttle_state * e.power_fraction *
                            std::exp(-altitude / e.altitude_scale) * rpm_factor
                      : 0;
  double n = e.rpm / 60, diameter = 2 * e.prop_radius,
         J = v / std::max(n * diameter, 1.);
  double prop_q =
      e.cq * density * n * n * std::pow(diameter, 5) * e.prop_fraction;
  double engine_q = e.shaft_power / std::max(omega, 20.);
  e.aero_torque = prop_q;
  e.acceleration = (engine_q - prop_q) / e.rotor_inertia;
  omega = std::max(0., omega + e.acceleration * dt);
  e.rpm = omega * 60 / (2 * pi);
  n = e.rpm / 60;
  J = v / std::max(n * diameter, 1.);
  e.thrust = e.ct * density * n * n * std::pow(diameter, 4) *
             (1 - e.advance_loss * J) * e.prop_fraction;
  // Apply the same shaft torque to rotor and mount; post-step RPM must not
  // change the reaction pair.
  e.torque = engine_q;
}
void Simulation::update_engine(const PilotIntent &p) {
  double v = std::max(
      body.attitude.conjugate().rotate(body.velocity - wind_world).x, 0.);
  density = 1.225 * std::exp(body.position.z / 8500.);
  if (def.engine.model == PropellerModel::Legacy)
    advance_propulsion(def.engine, density, -body.position.z, v, p.throttle,
                       dt);
  else {
    def.engine.target_rpm =
        def.engine.rated_rpm * clamp(p.prop_rpm_ratio, .5, 1.1);
    advance_propeller_controls(def.engine, p.throttle, dt);
    def.engine.torque = engine_shaft_torque(def.engine, -body.position.z,
                                            def.engine.rpm * 2 * pi / 60);
  }
}
Wrench Simulation::evaluate(const Body &b, bool store) {
  return evaluate_forces(b, store, false);
}
Wrench Simulation::evaluate_forces(const Body &b, bool store,
                                   bool midpoint_controls) {
  double rho = 1.225 * std::exp(b.position.z / 8500.);
  Vec vb = b.attitude.conjugate().rotate(b.velocity - wind_world);
  PropellerLoads blades;
  Wrench prop;
  if (def.engine.model == PropellerModel::BladeElement) {
    blades = propeller_loads(def.engine, rho,
                             vb + cross(b.omega, def.engine.position - mass.cg),
                             b.omega, def.engine.rpm * 2 * pi / 60);
    prop.add(blades.at_mount.force, def.engine.position - mass.cg,
             blades.at_mount.moment);
  }
  double wake = blades.induced;
  if (def.engine.model == PropellerModel::Legacy) {
    double disk = pi * def.engine.prop_radius * def.engine.prop_radius;
    wake = .5 * (std::sqrt(vb.x * vb.x +
                           2 * std::max(def.engine.thrust, 0.) / (rho * disk)) -
                 std::abs(vb.x));
  }
  Wrench a = evaluate_airframe(b, wake, store, midpoint_controls);
  if (def.engine.model == PropellerModel::Legacy) {
    prop.add({def.engine.thrust, 0, 0}, def.engine.position - mass.cg,
             {-def.engine.torque, 0, 0});
    prop.moment.z -= def.engine.thrust * .15 * def.engine.prop_radius *
                     std::sin(std::atan2(vb.z, vb.x));
  } else if (store) {
    auto &e = def.engine;
    e.thrust = dot(blades.at_mount.force, e.axis);
    e.aero_torque = blades.shaft_load;
    e.induced_velocity = blades.induced;
    e.profile_loss = blades.profile_loss;
    e.induced_loss = blades.induced_loss;
    e.induction_residual = blades.momentum_residual;
    e.induction_converged = blades.converged;
    e.tip_mach = blades.tip_mach;
  }
  if (store) {
    propulsion = prop;
  }
  a.force += prop.force;
  a.moment += prop.moment;
  return a;
}
Wrench Simulation::evaluate_airframe(const Body &b, double wake, bool store,
                                     bool midpoint_controls) {
  double rho = 1.225 * std::exp(b.position.z / 8500.);
  Vec vb = b.attitude.conjugate().rotate(b.velocity - wind_world);
  Wrench a;
  for (auto &c : def.cells) {
    if (!c.attached)
      continue;
    Vec r = c.position - mass.cg, local = vb + cross(b.omega, r);
    Vec from_prop = c.position - def.engine.position;
    double along = dot(from_prop, def.engine.axis),
           side = norm(from_prop - def.engine.axis * along);
    double flow_sign = (dot(vb, def.engine.axis) + wake) >= 0 ? 1 : -1;
    if (along * flow_sign < 0 && side < def.engine.prop_radius)
      local += def.engine.axis * wake;
    double deflection = 0;
    if (c.chain >= 0) {
      const auto &chain = def.chains[c.chain];
      deflection =
          (midpoint_controls ? chain.midpoint_deflection : chain.deflection) *
          chain.hinge * c.control_sign;
    }
    auto cell =
        cell_load(def, c, local, r, rho, cell_frame(def, c), deflection, store);
    a.force += cell.force;
    a.moment += cell.moment;
  }
  a.add(-unit(vb) * (.5 * rho * dot(vb, vb) * def.body_drag_area), -mass.cg);
  if (store) {
    aero = a;
    load_factor = -a.force.z / (mass.mass * gravity);
  }
  return a;
}
void integrate_body(Body &b, const MassProperties &m, const Wrench &force,
                    double elapsed) {
  // Implicit midpoint Euler equation preserves torque-free quadratic
  // invariants.
  Vec next = b.omega;
  for (int iteration = 0; iteration < 12; ++iteration) {
    Vec midpoint = (b.omega + next) * .5, L = m.inertia * midpoint;
    Vec residual = next - b.omega -
                   m.inertia.solve(force.moment - cross(midpoint, L)) * elapsed;
    if (norm(residual) < 1e-12)
      break;
    Mat jacobian;
    for (int j = 0; j < 3; ++j) {
      Vec basis;
      basis[j] = 1;
      Vec column = basis + m.inertia.solve(cross(basis, L) +
                                           cross(midpoint, m.inertia * basis)) *
                               (.5 * elapsed);
      for (int i = 0; i < 3; ++i)
        jacobian.a[i][j] = column[i];
    }
    next = next - jacobian.solve(residual);
    if (iteration == 11)
      throw std::runtime_error("angular midpoint did not converge");
  }
  Vec mid_omega = (b.omega + next) * .5;
  // The matching Cayley increment keeps world angular momentum consistent with
  // Euler's equation.
  double angle = 2 * std::atan(.5 * norm(mid_omega) * elapsed);
  Quat delta = Quat::axis(mid_omega, angle);
  Quat mid = (b.attitude * Quat::axis(mid_omega, angle * .5)).normalized();
  Vec accel = mid.rotate(force.force) / m.mass + Vec{0, 0, gravity};
  b.position += b.velocity * elapsed + accel * (.5 * elapsed * elapsed);
  b.velocity += accel * elapsed;
  b.attitude = (b.attitude * delta).normalized();
  b.omega = next;
}
void integrate_body(Body &b, const MassProperties &m, const BodyLoad &loads,
                    double dt) {
  const Body initial = b;
  std::vector<double> x{b.omega.x,    b.omega.y,    b.omega.z,
                        b.velocity.x, b.velocity.y, b.velocity.z};
  Body midpoint;
  auto residual = [&](const std::vector<double> &v) {
    Vec rates{v[0], v[1], v[2]}, velocity{v[3], v[4], v[5]};
    midpoint = initial;
    midpoint.omega = (initial.omega + rates) * .5;
    midpoint.velocity = (initial.velocity + velocity) * .5;
    midpoint.position += midpoint.velocity * (.5 * dt);
    double angle = 2 * std::atan(.5 * norm(midpoint.omega) * dt);
    midpoint.attitude =
        (initial.attitude * Quat::axis(midpoint.omega, angle * .5))
            .normalized();
    auto force = loads(midpoint);
    Vec angular =
        rates - initial.omega -
        m.inertia.solve(force.moment -
                        cross(midpoint.omega, m.inertia * midpoint.omega)) *
            dt;
    Vec linear =
        velocity - initial.velocity -
        (midpoint.attitude.rotate(force.force) / m.mass + Vec{0, 0, gravity}) *
            dt;
    return std::vector<double>{angular.x, angular.y, angular.z,
                               linear.x,  linear.y,  linear.z};
  };
  for (int it = 0; it < 12; ++it) {
    auto f = residual(x);
    double error = 0;
    for (double v : f)
      error += v * v;
    if (std::sqrt(error) < 1e-10)
      break;
    std::vector<std::vector<double>> J(6, std::vector<double>(6));
    for (int j = 0; j < 6; ++j) {
      auto shifted = x;
      double h = 1e-5 * std::max(1., std::abs(x[j]));
      shifted[j] += h;
      auto next = residual(shifted);
      for (int i = 0; i < 6; ++i)
        J[i][j] = (next[i] - f[i]) / h;
    }
    auto delta = linear_solve(J, f);
    for (int j = 0; j < 6; ++j)
      x[j] -= delta[j];
    if (it == 11)
      throw std::runtime_error("aerodynamic midpoint did not converge");
  }
  residual(x); // Leave callback diagnostics at the converged midpoint.
  b.omega = {x[0], x[1], x[2]};
  b.velocity = {x[3], x[4], x[5]};
  b.position += midpoint.velocity * dt;
  double angle = 2 * std::atan(.5 * norm(midpoint.omega) * dt);
  b.attitude =
      (initial.attitude * Quat::axis(midpoint.omega, angle)).normalized();
}
RotorStep integrate_rotor_body(Body &b, const MassProperties &m, Engine &e,
                               const Wrench &aero, Vec wind, double rho,
                               double dt, const RotorAirframeLoad &airframe) {
  const Body initial = b;
  double spin0 = e.rpm * 2 * pi / 60;
  std::vector<double> x{b.omega.x,    b.omega.y,    b.omega.z,   spin0,
                        b.velocity.x, b.velocity.y, b.velocity.z};
  PropellerLoads final_loads;
  Wrench final_aero;
  Quat final_mid;
  Vec final_rates{}, final_velocity{};
  double final_spin = 0, final_q = 0;
  auto residual = [&](const std::vector<double> &v, bool record = false) {
    Vec rates{v[0], v[1], v[2]}, velocity{v[4], v[5], v[6]};
    double spin = v[3];
    Vec mid_rates = (initial.omega + rates) * .5,
        mid_velocity = (initial.velocity + velocity) * .5;
    double mid_spin = (spin0 + spin) * .5;
    double angle = 2 * std::atan(.5 * norm(mid_rates) * dt);
    Quat qmid =
        (initial.attitude * Quat::axis(mid_rates, angle * .5)).normalized();
    Vec flow = qmid.conjugate().rotate(mid_velocity - wind) +
               cross(mid_rates, e.position - m.cg);
    auto loads = propeller_loads(e, rho, flow, mid_rates, mid_spin);
    double motor = engine_shaft_torque(e, -initial.position.z, mid_spin);
    Body midpoint = initial;
    midpoint.position += mid_velocity * (.5 * dt);
    midpoint.velocity = mid_velocity;
    midpoint.omega = mid_rates;
    midpoint.attitude = qmid;
    Wrench current_aero = airframe ? airframe(midpoint, loads) : aero;
    Wrench external = current_aero;
    external.add(loads.at_mount.force, e.position - m.cg,
                 loads.at_mount.moment);
    // Enforce the physical world-frame angular impulse using the same midpoint
    // orientation as the forces. This is a coordinate-consistent discrete Euler
    // equation, not an extra applied torque.
    Quat qend = (initial.attitude * Quat::axis(mid_rates, angle)).normalized();
    Vec old_H = initial.attitude.rotate(m.inertia * initial.omega +
                                        e.axis * (e.rotor_inertia * spin0));
    Vec new_H =
        qend.rotate(m.inertia * rates + e.axis * (e.rotor_inertia * spin));
    Vec angular = m.inertia.solve(qmid.conjugate().rotate(new_H - old_H) -
                                  external.moment * dt);
    double shaft = spin - spin0 + dot(rates - initial.omega, e.axis) -
                   (motor - loads.shaft_load) * dt / e.rotor_inertia;
    Vec linear =
        velocity - initial.velocity -
        (qmid.rotate(external.force) / m.mass + Vec{0, 0, gravity}) * dt;
    if (record) {
      final_loads = loads;
      final_aero = current_aero;
      final_mid = qmid;
      final_rates = mid_rates;
      final_velocity = mid_velocity;
      final_spin = mid_spin;
      final_q = motor;
    }
    return std::vector<double>{angular.x, angular.y, angular.z, shaft,
                               linear.x,  linear.y,  linear.z};
  };
  for (int it = 0; it < 12; ++it) {
    auto f = residual(x);
    double error = 0;
    for (double v : f)
      error += v * v;
    if (std::sqrt(error) < 1e-10)
      break;
    std::vector<std::vector<double>> J(7, std::vector<double>(7));
    for (int j = 0; j < 7; ++j) {
      auto shifted = x;
      double h = 1e-5 * std::max(1., std::abs(x[j]));
      shifted[j] += h;
      auto next = residual(shifted);
      for (int i = 0; i < 7; ++i)
        J[i][j] = (next[i] - f[i]) / h;
    }
    auto delta = linear_solve(J, f);
    for (int j = 0; j < 7; ++j)
      x[j] -= delta[j];
    if (it == 11)
      throw std::runtime_error("coupled rotor/body midpoint did not converge");
  }
  residual(x, true);
  b.omega = {x[0], x[1], x[2]};
  b.velocity = {x[4], x[5], x[6]};
  b.position += (initial.velocity + b.velocity) * (.5 * dt);
  double angle = 2 * std::atan(.5 * norm(final_rates) * dt);
  b.attitude = (initial.attitude * Quat::axis(final_rates, angle)).normalized();
  e.rpm = x[3] * 60 / (2 * pi);
  e.acceleration = (x[3] - spin0) / dt;
  e.torque = final_q;
  e.shaft_power = final_q * final_spin;
  e.aero_torque = final_loads.shaft_load;
  e.thrust = dot(final_loads.at_mount.force, e.axis);
  e.induced_velocity = final_loads.induced;
  e.profile_loss = final_loads.profile_loss;
  e.induced_loss = final_loads.induced_loss;
  e.induction_converged = final_loads.converged;
  e.induction_residual = final_loads.momentum_residual;
  e.tip_mach = final_loads.tip_mach;
  Wrench external = final_aero;
  external.add(final_loads.at_mount.force, e.position - m.cg,
               final_loads.at_mount.moment);
  RotorStep result;
  result.engine_work = final_q * final_spin * dt;
  result.external_work =
      (dot(final_mid.rotate(external.force), final_velocity) +
       dot(external.moment, final_rates) -
       final_loads.shaft_load * final_spin) *
      dt;
  result.prop_loss = (final_loads.profile_loss + final_loads.induced_loss) * dt;
  result.impulse_world = final_mid.rotate(external.moment) * dt;
  return result;
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
    std::vector<int> source_chains;
    for (int id : group) {
      def.nodes[id].attached = false;
      newly.erase(std::remove(newly.begin(), newly.end(), id), newly.end());
    }
    for (int i = 0; i < int(def.cells.size()); ++i)
      if (std::find(group.begin(), group.end(), def.cells[i].node) !=
          group.end()) {
        AeroCell local = def.cells[i];
        if (local.chain >= 0) {
          auto found = std::find(source_chains.begin(), source_chains.end(),
                                 local.chain);
          if (found == source_chains.end()) {
            source_chains.push_back(local.chain);
            auto surface = def.chains[local.chain];
            surface.connected = false;
            piece.chains.push_back(surface);
            local.chain = int(piece.chains.size()) - 1;
          } else
            local.chain = int(found - source_chains.begin());
        }
        piece.aero_cells.push_back({local, cell_frame(def, def.cells[i])});
        def.cells[i].attached = false;
        def.cells[i].force = {};
        def.cells[i].moment = {};
        def.cells[i].q = 0;
        def.cells[i].stall = 0;
        piece.cells.push_back(i);
      }
    evaluate_detached(
        piece); // Initial diagnostics only; no extra physical tick.
    debris.push_back(piece);
  }
  // Only chains with surviving cells retain a pilot path. Shared chains may
  // have an attached branch plus an independently unpowered branch on a piece.
  for (std::size_t id = 0; id < def.chains.size(); ++id) {
    bool has_surface = std::any_of(
        def.cells.begin(), def.cells.end(), [&](const AeroCell &cell) {
          return cell.attached && cell.chain == int(id);
        });
    if (!has_surface)
      def.chains[id].connected = false;
  }
  mass = properties(def, still);
  Vec offset = mass.cg - old_cg;
  body.position += body.attitude.rotate(offset);
  body.velocity += body.attitude.rotate(cross(body.omega, offset));
  // Every piece inherits the pre-separation rigid velocity field; aggregate
  // linear/angular momentum is conserved at the split.
}
Wrench Simulation::evaluate_detached(DetachedBody &piece, bool store) {
  return detached_load(piece, piece.state, store);
}
Wrench Simulation::detached_load(DetachedBody &piece, const Body &state,
                                 bool store, bool midpoint_controls) {
  Vec vb = state.attitude.conjugate().rotate(state.velocity - wind_world);
  double rho = 1.225 * std::exp(state.position.z / 8500.);
  Wrench result;
  for (auto &element : piece.aero_cells) {
    auto &cell = element.cell;
    Vec r = cell.position - piece.mass.cg;
    Vec local = vb + cross(state.omega, r);
    double angle = 0;
    if (cell.chain >= 0) {
      const auto &chain = piece.chains[cell.chain];
      angle =
          (midpoint_controls ? chain.midpoint_deflection : chain.deflection) *
          chain.hinge * cell.control_sign;
    }
    auto load =
        cell_load(def, cell, local, r, rho, element.frame, angle, store);
    result.force += load.force;
    result.moment += load.moment;
  }
  if (store)
    piece.aero = result;
  return result;
}
void Simulation::assemble_structural_loads(const Wrench &forces, Vec rates,
                                           double spin) {
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
  Vec angular_accel;
  if (def.engine.model == PropellerModel::BladeElement) {
    const auto &e = def.engine;
    Mat effective = mass.inertia;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        effective.a[i][j] -= e.rotor_inertia * e.axis[i] * e.axis[j];
    angular_accel = effective.solve(
        forces.moment - e.axis * (e.torque - e.aero_torque) -
        cross(rates, mass.inertia * rates + e.axis * (e.rotor_inertia * spin)));
  } else
    angular_accel =
        mass.inertia.solve(forces.moment - cross(rates, mass.inertia * rates));
  for (const auto &part : def.masses)
    if (def.nodes[part.node].attached) {
      Vec r = part.position - mass.cg;
      Vec relief = -(forces.force / mass.mass + cross(angular_accel, r) +
                     cross(rates, cross(rates, r))) *
                   (part.mass * part.remaining);
      Vec moment = cross(part.position, relief);
      node_bending[part.node] += moment.x;
      node_torsion[part.node] += moment.y;
    }
}
void Simulation::step(const PilotIntent &p) {
  update_engine(p);
  update_controls(p);
  // Existing pieces advance once from their OWN velocity/CG/rates and cells.
  // New pieces inherit the end-of-tick rigid state and start next tick.
  for (auto &piece : debris) {
    Vec v = piece.state.velocity - wind_world;
    double rho = 1.225 * std::exp(piece.state.position.z / 8500.);
    double q = .5 * rho * dot(v, v);
    for (auto &chain : piece.chains)
      sample_surface(chain, 0, q,
                     dt); // Unpowered local branch or mechanical jam.
    Body midpoint;
    BodyLoad loads = [&](const Body &trial) {
      midpoint = trial;
      return detached_load(piece, trial, false, true);
    };
    integrate_body(piece.state, piece.mass, loads, dt);
    detached_load(piece, midpoint, true, true);
  }
  Body midpoint;
  double spin0 = def.engine.rpm * 2 * pi / 60;
  if (def.engine.model == PropellerModel::Legacy) {
    BodyLoad loads = [&](const Body &trial) {
      midpoint = trial;
      return evaluate_forces(trial, false, true);
    };
    integrate_body(body, mass, loads, dt);
    evaluate_forces(midpoint, true, true);
  } else {
    PropellerLoads blades;
    RotorAirframeLoad loads = [&](const Body &trial,
                                  const PropellerLoads &prop) {
      midpoint = trial;
      blades = prop;
      return evaluate_airframe(trial, prop.induced, false, true);
    };
    rotor_step = integrate_rotor_body(body, mass, def.engine, {}, wind_world,
                                      density, dt, loads);
    evaluate_airframe(midpoint, blades.induced, true, true);
    propulsion = {};
    propulsion.add(blades.at_mount.force, def.engine.position - mass.cg,
                   blades.at_mount.moment);
  }
  Wrench forces{aero.force + propulsion.force, aero.moment + propulsion.moment};
  assemble_structural_loads(forces, midpoint.omega,
                            .5 * (spin0 + def.engine.rpm * 2 * pi / 60));
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
  if (!c->attached)
    for (auto &piece : debris)
      for (auto &element : piece.aero_cells)
        if (element.cell.id == id) {
          element.cell.area_fraction = c->area_fraction;
          element.cell.roughness = c->roughness;
        }
}
void Simulation::remove_material(const std::string &name, double remaining) {
  auto it = std::find_if(def.masses.begin(), def.masses.end(),
                         [&](const MassPart &p) {
                           return !name.empty() && p.material_name == name;
                         });
  if (it == def.masses.end() || !std::isfinite(remaining) || remaining < 0 ||
      remaining > it->remaining)
    throw std::runtime_error("invalid material removal (no restoration)");
  if (remaining == it->remaining)
    return;
  std::size_t id = it - def.masses.begin();
  int owner = -1;
  std::vector<int> nodes;
  if (def.nodes[it->node].attached) {
    for (int n = 0; n < int(def.nodes.size()); ++n)
      if (def.nodes[n].attached)
        nodes.push_back(n);
  } else {
    for (std::size_t n = 0; n < debris.size(); ++n)
      if (std::find(debris[n].nodes.begin(), debris[n].nodes.end(), it->node) !=
          debris[n].nodes.end()) {
        owner = int(n);
        nodes = debris[n].nodes;
        break;
      }
    if (owner < 0)
      throw std::runtime_error("material has no rigid owner");
  }
  // Compute first: invalid requests never partially change live
  // inventory/state.
  Definition updated = def;
  updated.masses[id].remaining = remaining;
  auto next_mass = properties(updated, nodes);
  const Body old_body = owner < 0 ? body : debris[owner].state;
  const MassProperties old_mass = owner < 0 ? mass : debris[owner].mass;
  double removed_fraction = it->remaining - remaining;
  DetachedBody parcel;
  parcel.material_name = name;
  parcel.mass = {it->mass * removed_fraction, it->position,
                 Mat::diagonal(it->inertia * removed_fraction)};
  parcel.state = old_body;
  Vec removed_offset = parcel.mass.cg - old_mass.cg;
  parcel.state.position += old_body.attitude.rotate(removed_offset);
  parcel.state.velocity +=
      old_body.attitude.rotate(cross(old_body.omega, removed_offset));
  Body next_body = old_body;
  Vec offset = next_mass.cg - old_mass.cg;
  next_body.position += old_body.attitude.rotate(offset);
  next_body.velocity += old_body.attitude.rotate(cross(old_body.omega, offset));
  def.masses[id].remaining = remaining;
  if (owner < 0) {
    body = next_body;
    mass = next_mass;
    // Preserve the fixed-tick sensor sampling schedule. Re-express cached
    // loads about the new CG instead of evaluating guidance sensors off-tick.
    aero.moment = aero.moment - cross(offset, aero.force);
    propulsion.moment = propulsion.moment - cross(offset, propulsion.force);
    load_factor = -aero.force.z / (mass.mass * gravity);
  } else {
    debris[owner].state = next_body;
    debris[owner].mass = next_mass;
    debris[owner].aero.moment =
        debris[owner].aero.moment - cross(offset, debris[owner].aero.force);
  }
  debris.push_back(parcel);
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
  for (auto &c : def.chains) {
    double command =
        def.trim_enabled[c.axis] ? clamp(t.intent.trim[c.axis], -1, 1) : 0;
    c.filtered = c.connected ? command * c.limit * clamp(c.range, 0, 1) : 0;
    double q = .5 * 1.225 * std::exp(-altitude / 8500) * speed * speed;
    c.deflection = c.filtered / (1 + q / c.q_limit);
    if (c.jammed)
      c.deflection = c.filtered = clamp(c.jam_position, -c.limit, c.limit);
  }
  def.engine.throttle_state = t.intent.throttle;
  auto &e = def.engine;
  if (e.model == PropellerModel::BladeElement) {
    double rho = 1.225 * std::exp(-altitude / 8500);
    Vec flow = body.attitude.conjugate().rotate(body.velocity - wind_world);
    double preferred = e.pitch;
    auto balance = [&](double value, bool pitch) {
      if (pitch)
        e.pitch = value;
      double spin = pitch ? e.target_rpm * 2 * pi / 60 : value;
      return propeller_loads(e, rho, flow, {}, spin).shaft_load -
             engine_shaft_torque(e, altitude, spin);
    };
    bool pitch = e.governor && !e.pitch_jammed && e.pitch_rate_fraction > 0;
    double lower = pitch ? e.pitch_min : 0,
           upper =
               pitch ? e.pitch_min + (e.pitch_max - e.pitch_min) * e.pitch_range
                     : e.rated_rpm * 2 * pi / 60 * 4;
    double selected = pitch ? preferred : e.rpm * 2 * pi / 60, best = 1e100,
           lo = lower, hi = upper;
    bool bracket = false;
    double last = lower, last_f = balance(lower, pitch);
    for (int j = 1; j <= 20; ++j) {
      double value = lower + (upper - lower) * j / 20.,
             f = balance(value, pitch);
      if (std::abs(f) < best) {
        best = std::abs(f);
        selected = value;
      }
      if (f * last_f <= 0) {
        lo = last;
        hi = value;
        bracket = true;
        break;
      }
      last = value;
      last_f = f;
    }
    if (bracket) {
      double f_lo = balance(lo, pitch);
      for (int j = 0; j < 32; ++j) {
        double mid = (lo + hi) * .5, f = balance(mid, pitch);
        if (f * f_lo <= 0)
          hi = mid;
        else {
          lo = mid;
          f_lo = f;
        }
      }
      selected = (lo + hi) * .5;
    }
    if (pitch) {
      e.pitch = selected;
      e.rpm = e.target_rpm;
    } else
      e.rpm = selected * 60 / (2 * pi);
    auto loads = propeller_loads(e, rho, flow, {}, e.rpm * 2 * pi / 60);
    e.torque = engine_shaft_torque(e, altitude, e.rpm * 2 * pi / 60);
    e.aero_torque = loads.shaft_load;
    e.acceleration = (e.torque - e.aero_torque) / e.rotor_inertia;
    e.thrust = dot(loads.at_mount.force, e.axis);
    e.induced_velocity = loads.induced;
    e.shaft_power = e.torque * e.rpm * 2 * pi / 60;
  } else {
    // Solve shaft/prop torque equilibrium for the trim's throttle.
    double rho = 1.225 * std::exp(-altitude / 8500), D = 2 * e.prop_radius;
    double power = e.max_power * t.intent.throttle *
                   std::exp(-altitude / e.altitude_scale) * e.power_fraction *
                   (e.running ? 1 : 0);
    e.rpm = e.prop_fraction > 0
                ? std::sqrt(std::max(power, 0.) * 60 /
                            (2 * pi * e.rated_rpm * e.cq * rho *
                             std::pow(D, 5) * e.prop_fraction)) *
                      60
                : e.rated_rpm;
    e.rpm = std::min(e.rpm, e.rated_rpm * 1.15);
    update_engine(t.intent);
  }
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
      assemble_structural_loads(trial, body.omega,
                                def.engine.rpm * 2 * pi / 60);
      solve_structure(node_bending, node_torsion, 0);
    }
    auto w = evaluate(body);
    Vec gravity_body =
        body.attitude.conjugate().rotate({0, 0, mass.mass * gravity});
    Vec f = w.force + gravity_body;
    if (def.engine.model == PropellerModel::BladeElement)
      w.moment = w.moment -
                 def.engine.axis * (def.engine.torque - def.engine.aero_torque);
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
    // Regularized least squares tolerates unavailable trim axes without
    // inventing authority.
    std::vector<std::vector<double>> normal(6, std::vector<double>(6));
    std::vector<double> rhs(6);
    for (int j = 0; j < 6; ++j) {
      for (int i = 0; i < 6; ++i) {
        rhs[j] -= J[i][j] * f[i];
        for (int k = 0; k < 6; ++k)
          normal[j][k] += J[i][j] * J[i][k];
      }
      normal[j][j] += 1e-9;
    }
    auto dx = linear_solve(normal, rhs);
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
  auto actual = evaluate(body, true);
  out.force_residual_N = actual.force + body.attitude.conjugate().rotate(
                                            {0, 0, mass.mass * gravity});
  out.moment_residual_Nm = actual.moment;
  if (def.engine.model == PropellerModel::BladeElement)
    out.moment_residual_Nm =
        out.moment_residual_Nm -
        def.engine.axis * (def.engine.torque - def.engine.aero_torque);
  out.rotor_acceleration = def.engine.acceleration;
  out.converged =
      out.residual < 1e-6 && std::abs(out.rotor_acceleration) < 1e-6;
  return out;
}
} // namespace flight
