#include "flight/core.hpp"
#include <stdexcept>
namespace flight {
namespace {
CurvePoint polar(const Engine &e, double alpha) {
  alpha = std::atan2(std::sin(alpha), std::cos(alpha));
  const auto &t = e.polar;
  auto hi = std::upper_bound(
      t.begin(), t.end(), alpha,
      [](double x, const CurvePoint &p) { return x < p.alpha; });
  if (hi == t.begin())
    return t.front();
  if (hi == t.end())
    return t.back();
  auto lo = hi - 1;
  double f = (alpha - lo->alpha) / (hi->alpha - lo->alpha);
  return {alpha, lo->cl + f * (hi->cl - lo->cl), lo->cd + f * (hi->cd - lo->cd),
          0};
}
PropellerLoads strips(const Engine &e, double rho, Vec flow, Vec rates,
                      double spin, double induced) {
  PropellerLoads p;
  p.induced = induced;
  for (const auto &strip : e.strips) {
    Vec tangent = cross(e.axis, strip.radial);
    Vec local = flow + cross(rates, strip.offset) +
                cross(e.axis, strip.offset) * spin + e.axis * induced;
    double axial = dot(local, e.axis), rotating = dot(local, tangent),
           speed = norm(local);
    double angle = e.pitch + strip.twist - std::atan2(axial, rotating);
    auto c = polar(e, angle);
    double q = .5 * rho * speed * speed, area = strip.area * e.prop_fraction;
    Vec lift_dir = unit(cross(strip.radial, local));
    Vec force = lift_dir * (q * area * c.cl) - unit(local) * (q * area * c.cd);
    p.at_mount.add(force, strip.offset);
    p.profile_loss += q * area * c.cd * speed;
    p.tip_mach = std::max(p.tip_mach, speed / 340.);
  }
  // Sample the actual geometric tip, not only the midpoint of the last annulus.
  for (int j = 0; j < e.azimuth_samples; ++j) {
    const auto &tip = e.strips[e.strips.size() - e.azimuth_samples + j];
    Vec r = tip.radial * e.prop_radius;
    Vec v = flow + cross(rates, r) + cross(e.axis, r) * spin + e.axis * induced;
    p.tip_mach = std::max(p.tip_mach, norm(v) / 340.);
  }
  p.shaft_load = -dot(p.at_mount.moment, e.axis);
  p.induced_loss = dot(p.at_mount.force, e.axis) * induced;
  return p;
}
} // namespace
void prepare_propeller(Engine &e) {
  if (e.model == PropellerModel::Legacy)
    return;
  if (e.stations.size() < 2 || e.polar.size() < 3 || e.blades < 1 ||
      e.radial_samples < 2 || e.azimuth_samples < 4 || e.prop_radius <= 0 ||
      norm(e.axis) < 1e-8 || e.pitch_min >= e.pitch_max || e.pitch_slew <= 0)
    throw std::runtime_error("invalid blade-element definition");
  e.axis = unit(e.axis);
  e.strips.clear();
  for (std::size_t i = 0; i < e.stations.size(); ++i) {
    auto &s = e.stations[i];
    if (s.radius_fraction <= 0 || s.radius_fraction > 1 || s.chord <= 0 ||
        (i && s.radius_fraction <= e.stations[i - 1].radius_fraction))
      throw std::runtime_error("invalid blade geometry");
  }
  for (std::size_t i = 0; i < e.polar.size(); ++i)
    if (e.polar[i].cd < 0 || (i && e.polar[i].alpha <= e.polar[i - 1].alpha))
      throw std::runtime_error("invalid propeller polar");
  if (e.polar.front().alpha > -pi || e.polar.back().alpha < pi)
    throw std::runtime_error("propeller polar must cover full incidence");
  Vec radial0 = unit(
          cross(e.axis, std::abs(e.axis.z) < .9 ? Vec{0, 0, 1} : Vec{0, 1, 0})),
      radial90 = cross(e.axis, radial0);
  double root = e.stations.front().radius_fraction,
         tip = e.stations.back().radius_fraction;
  double dr = (tip - root) * e.prop_radius / e.radial_samples;
  for (int i = 0; i < e.radial_samples; ++i) {
    double fraction = root + (i + .5) * (tip - root) / e.radial_samples;
    auto hi = std::upper_bound(
        e.stations.begin(), e.stations.end(), fraction,
        [](double f, const BladeStation &s) { return f < s.radius_fraction; });
    auto lo = hi - 1;
    double t = (fraction - lo->radius_fraction) /
               (hi->radius_fraction - lo->radius_fraction),
           chord = lo->chord + t * (hi->chord - lo->chord),
           twist = lo->twist + t * (hi->twist - lo->twist);
    for (int j = 0; j < e.azimuth_samples; ++j) {
      double angle = 2 * pi * (j + .5) / e.azimuth_samples;
      Vec radial = radial0 * std::cos(angle) + radial90 * std::sin(angle);
      e.strips.push_back({radial * (fraction * e.prop_radius), radial,
                          chord * dr * e.blades / e.azimuth_samples, twist});
    }
  }
  if (e.target_rpm <= 0)
    e.target_rpm = e.rated_rpm;
}
PropellerLoads propeller_loads(const Engine &e, double rho, Vec flow, Vec rates,
                               double spin) {
  if (e.prop_fraction == 0) {
    PropellerLoads zero;
    zero.converged = true;
    return zero;
  }
  if (e.strips.empty() || rho <= 0)
    throw std::runtime_error("unprepared propeller");
  double disc = pi * e.prop_radius * e.prop_radius;
  auto residual = [&](double v, PropellerLoads *result = nullptr) {
    auto p = strips(e, rho, flow, rates, spin, v);
    double value = 2 * rho * disc * v * norm(flow + e.axis * v) -
                   dot(p.at_mount.force, e.axis);
    if (result)
      *result = p;
    return value;
  };
  // Solve the signed, uniform-induction momentum closure. No division by RPM or
  // J. Large-brake/reversed-flow roots are a mathematical continuation, not
  // calibrated vortex-ring physics.
  double bound = std::max(20., norm(flow) + std::abs(spin) * e.prop_radius),
         lo = -bound, hi = bound;
  double f_lo = residual(lo), f_hi = residual(hi);
  for (int i = 0; i < 8 && (f_lo > 0 || f_hi < 0); ++i) {
    bound *= 2;
    lo = -bound;
    hi = bound;
    f_lo = residual(lo);
    f_hi = residual(hi);
  }
  if (f_lo > 0 || f_hi < 0)
    throw std::runtime_error("induced flow could not be bracketed");
  double v = clamp(e.induced_velocity, lo, hi);
  PropellerLoads p;
  for (int i = 0; i < 45; ++i) {
    double f = residual(v, &p);
    if (std::abs(f) < 1e-7) {
      p.momentum_residual = f;
      p.converged = true;
      return p;
    }
    if (f > 0) {
      hi = v;
      f_hi = f;
    } else {
      lo = v;
      f_lo = f;
    }
    double h = 1e-3, derivative = (residual(v + h) - f) / h;
    double next = derivative > 0 ? v - f / derivative : (lo + hi) * .5;
    if (!std::isfinite(next) || next <= lo || next >= hi)
      next = (lo + hi) * .5;
    v = next;
  }
  p.momentum_residual = residual(v, &p);
  p.converged = std::abs(p.momentum_residual) < 1e-4;
  return p;
}
double engine_shaft_torque(const Engine &e, double altitude, double spin) {
  double drag = -e.engine_friction * spin;
  if (!e.running || e.max_power == 0)
    return drag;
  double power = e.max_power * e.throttle_state * e.power_fraction *
                 std::exp(-altitude / e.altitude_scale) *
                 clamp(spin / (e.rated_rpm * 2 * pi / 60), 0, 1.15);
  return power / std::max(std::abs(spin), 20.) + drag;
}
void advance_propeller_controls(Engine &e, double throttle, double dt) {
  e.throttle_state += (clamp(throttle, 0, 1) - e.throttle_state) *
                      (1 - std::exp(-dt / e.response));
  if (!e.governor) {
    e.governor_saturated = false;
    return;
  }
  if (e.pitch_jammed) {
    e.governor_saturated = true;
    return;
  }
  double error = (e.rpm - e.target_rpm) / e.rated_rpm;
  double rate = e.governor_kp * error + e.governor_ki * e.governor_integral;
  double lower = e.pitch_min,
         upper = lower + (e.pitch_max - lower) * clamp(e.pitch_range, 0, 1);
  double limit = e.pitch_slew * e.pitch_rate_fraction,
         pump = clamp(std::abs(e.rpm) / e.pump_rpm, 0, 1);
  bool at_limit =
      (e.pitch <= lower && rate < 0) || (e.pitch >= upper && rate > 0);
  bool windup = (e.pitch <= lower && error < 0) ||
                (e.pitch >= upper && error > 0) || e.pitch_rate_fraction <= 0 ||
                pump < .1 || (rate > limit && error > 0) ||
                (rate < -limit && error < 0);
  e.governor_saturated =
      at_limit || std::abs(rate) > limit || e.pitch_rate_fraction <= 0;
  // Freeze only error that would wind up; opposite error must unwind the stored
  // bias.
  if (!windup)
    e.governor_integral = clamp(e.governor_integral + error * dt, -1, 1);
  rate = clamp(e.governor_kp * error + e.governor_ki * e.governor_integral,
               -limit, limit) *
         pump;
  double request = clamp(e.pitch + rate * dt, lower, upper);
  e.pitch += clamp(request - e.pitch, -limit * pump * dt, limit * pump * dt);
}
double mechanical_energy(const Body &b, const MassProperties &m,
                         const Engine &e) {
  double spin = e.rpm * 2 * pi / 60;
  return .5 * m.mass * dot(b.velocity, b.velocity) -
         m.mass * gravity * b.position.z +
         .5 * dot(b.omega, m.inertia * b.omega) +
         e.rotor_inertia * (spin * dot(b.omega, e.axis) + .5 * spin * spin);
}
Vec total_angular_momentum(const Body &b, const MassProperties &m,
                           const Engine &e) {
  return b.attitude.rotate(m.inertia * b.omega +
                           e.axis * (e.rotor_inertia * e.rpm * 2 * pi / 60));
}
} // namespace flight
