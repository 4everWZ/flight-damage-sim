#include "flight/core.hpp"
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;
// Only the non-advancing controller bench receives recorded Body sensors.
// The prescribed-surface and clock-locked flights never assign a Body state;
// their actual motion always comes from Simulation::step forces/moments.
using Axis3 = std::array<double, 3>;
// The same experiment can be built against the earlier core for a genuine
// before/after run without rewriting frozen source histories.
template <class Chain> double applied_sample(const Chain &chain) {
  if constexpr (requires { chain.midpoint_deflection; })
    return chain.midpoint_deflection;
  else
    return chain.deflection;
}
struct Sample {
  Body body;
  double load = 0, rho = 0;
  PilotIntent p;
  Axis3 command{}, bias{}, weight{};
  std::array<bool, 3> paused{};
  std::vector<double> surfaces, applied;
};
struct History {
  int hz = 0;
  Definition initial;
  std::vector<Sample> samples;
  Body final;
};
History record(const Definition &d, int hz, bool removed) {
  Simulation s(d, 1. / hz);
  auto trim = s.trim(70, 1000);
  if (!trim.converged)
    throw std::runtime_error("trim failed");
  History h;
  h.hz = hz;
  h.initial = s.def;
  auto p = trim.intent;
  p.mouse = true;
  p.aim_world = {1, .3, 0};
  for (int i = 0; i < 5 * hz; ++i) {
    if (removed && i == hz)
      s.remove_material("left_outer_forward", .2);
    p.active[Roll] = i >= hz && i < 2 * hz;
    p.direct[Roll] = 1;
    Sample a;
    a.body = s.body;
    a.load = s.load_factor;
    a.rho = 1.225 * std::exp(s.body.position.z / 8500.);
    a.p = p;
    s.step(p);
    for (int j = 0; j < 3; ++j) {
      a.command[j] = s.ownership[j].command;
      a.bias[j] = s.instructor.rate_bias[j];
      a.weight[j] = s.ownership[j].direct_weight;
      a.paused[j] = s.instructor.integral_paused[j];
    }
    for (auto &c : s.def.chains) {
      a.surfaces.push_back(c.deflection);
      a.applied.push_back(applied_sample(c));
    }
    h.samples.push_back(a);
    for (auto &n : s.def.nodes)
      if (!n.attached)
        throw std::runtime_error(
            "sensor bench requires unchanged available-control topology");
  }
  h.final = s.body;
  return h;
}
struct Controls {
  int hz = 0;
  std::vector<Sample> samples;
};
Controls control(const History &h, int sensor_hz, int controller_hz) {
  Simulation s(h.initial, 1. / controller_hz);
  Controls out;
  out.hz = controller_hz;
  for (int i = 0; i < 5 * controller_hz; ++i) {
    int tick = i * h.hz / controller_hz,
        held = (tick / (h.hz / sensor_hz)) * (h.hz / sensor_hz);
    auto src = h.samples[held];
    s.body = src.body;
    s.load_factor = src.load;
    s.density = src.rho;
    auto p = h.samples[tick].p;
    s.update_controls(p);
    Sample a;
    for (int j = 0; j < 3; ++j) {
      a.command[j] = s.ownership[j].command;
      a.bias[j] = s.instructor.rate_bias[j];
      a.weight[j] = s.ownership[j].direct_weight;
      a.paused[j] = s.instructor.integral_paused[j];
    }
    for (auto &c : s.def.chains) {
      a.surfaces.push_back(c.deflection);
      a.applied.push_back(applied_sample(c));
    }
    out.samples.push_back(a);
  }
  return out;
}
struct Actuators {
  int hz = 0;
  std::vector<std::vector<double>> end, applied;
  double area_error = -1, endpoint_area_error = -1;
};
Actuators actuators(const History &h, const Controls &requests, int request_hz,
                    int q_hz, int hz) {
  Simulation s(h.initial, 1. / hz);
  Actuators a;
  a.hz = hz;
  for (int i = 0; i < 5 * hz; ++i) {
    int ht = i * h.hz / hz;
    auto sensor = h.samples[(ht / (h.hz / q_hz)) * (h.hz / q_hz)];
    s.body = sensor.body;
    s.density = sensor.rho;
    int rt = i * requests.hz / hz;
    rt = (rt / (requests.hz / request_hz)) * (requests.hz / request_hz);
    auto p = h.samples[ht].p;
    p.mouse = false;
    p.active = {true, true, true};
    p.direct = requests.samples[rt].command;
    s.update_controls(p);
    std::vector<double> end, middle;
    for (auto &c : s.def.chains) {
      end.push_back(c.deflection);
      middle.push_back(applied_sample(c));
    }
    a.end.push_back(end);
    a.applied.push_back(middle);
  }
  if (request_hz == 120 && q_hz == 120) {
    // Independent exact area of a rate-limited ramp toward a held target.
    // The fixture has zero linkage lag. This is not an analytic claim for the
    // general combined lag/slew model.
    a.area_error = a.endpoint_area_error = 0;
    for (int k = 0; k < 5 * 120; ++k) {
      const auto &sensor = h.samples[k * h.hz / 120];
      const auto &r = requests.samples[k * requests.hz / 120];
      double q =
          .5 * sensor.rho * dot(sensor.body.velocity, sensor.body.velocity);
      double interval_error = 0, old_error = 0;
      for (size_t j = 0; j < h.initial.chains.size(); ++j) {
        const auto &c = h.initial.chains[j];
        if (c.lag || c.jammed || !c.connected)
          throw std::runtime_error("analytic actuator fixture changed");
        double command =
            r.command[c.axis] +
            (h.initial.trim_enabled[c.axis]
                 ? sensor.p.trim[c.axis] * (1 - std::abs(r.command[c.axis]))
                 : 0);
        command = clamp(command, -1, 1);
        double effort = 1 / (1 + q / c.q_limit);
        double target = command * c.limit * clamp(c.range, 0, 1) * effort;
        double start = k ? a.end[k * hz / 120 - 1][j] : c.deflection;
        double rate = c.slew * effort / (1 + std::max(c.friction, 0.));
        double duration = 1. / 120,
               ramp = std::min(duration, std::abs(target - start) / rate);
        double slope = target >= start ? rate : -rate;
        double exact = start * ramp + .5 * slope * ramp * ramp +
                       target * (duration - ramp);
        double area = 0, old_area = 0;
        for (int n = k * hz / 120; n < (k + 1) * hz / 120; ++n) {
          area += a.applied[n][j] / hz;
          old_area += a.end[n][j] / hz;
        }
        interval_error = std::max(interval_error, std::abs(area - exact));
        old_error = std::max(old_error, std::abs(old_area - exact));
      }
      a.area_error += interval_error;
      a.endpoint_area_error += old_error;
    }
  }
  return a;
}
double maxdiff(Axis3 a, Axis3 b) {
  double e = 0;
  for (int j = 0; j < 3; ++j)
    e = std::max(e, std::abs(a[j] - b[j]));
  return e;
}
void compare(const std::string &mode, int case_id, const Controls &a,
             const Controls &ref, int output_hz, std::ofstream &trace) {
  double bias = 0, cmd = 0, weight = 0;
  int pause_mismatch = 0;
  for (int k = 0; k < 5 * 120; ++k) {
    int ia = (k + 1) * a.hz / 120 - 1, ir = (k + 1) * ref.hz / 120 - 1;
    bias = std::max(bias, maxdiff(a.samples[ia].bias, ref.samples[ir].bias));
    ia = k * a.hz / 120;
    ir = k * ref.hz / 120;
    weight =
        std::max(weight, maxdiff(a.samples[ia].weight, ref.samples[ir].weight));
    for (int j = 0; j < 3; ++j)
      pause_mismatch += a.samples[ia].paused[j] != ref.samples[ir].paused[j];
    if (k >= 265 && k <= 273)
      trace << mode << ',' << case_id << ',' << a.hz << ',' << k / 120. << ','
            << a.samples[ia].weight[0] << ',' << a.samples[ia].paused[0] << ','
            << a.samples[ia].bias[0] << ',' << ref.samples[ir].paused[0] << ','
            << ref.samples[ir].bias[0] << '\n';
  }
  for (int i = 0; i < 5 * 960; ++i) {
    int out = (i / (960 / output_hz)) * (960 / output_hz);
    int ia = out * a.hz / 960, ir = out * ref.hz / 960;
    cmd += maxdiff(a.samples[ia].command, ref.samples[ir].command) / 960.;
  }
  std::cout << mode << ' ' << case_id << ' ' << a.hz << " endpoint_bias_max "
            << bias << " held_request_L1 " << cmd << " aligned_weight_max "
            << weight << " pause_mismatches " << pause_mismatch << '\n';
}
void compare_act(const std::string &mode, int case_id, const Actuators &a,
                 const Actuators &ref) {
  double endpoint = 0, applied = 0;
  for (int k = 0; k < 5 * 120; ++k) {
    auto &av = a.end[(k + 1) * a.hz / 120 - 1];
    auto &rv = ref.end[(k + 1) * ref.hz / 120 - 1];
    for (size_t j = 0; j < av.size(); ++j)
      endpoint = std::max(endpoint, std::abs(av[j] - rv[j]));
  }
  for (int i = 0; i < 5 * 960; ++i) {
    auto &av = a.applied[i * a.hz / 960];
    auto &rv = ref.applied[i * ref.hz / 960];
    double e = 0;
    for (size_t j = 0; j < av.size(); ++j)
      e = std::max(e, std::abs(av[j] - rv[j]));
    applied += e / 960.;
  }
  std::cout << mode << ' ' << case_id << ' ' << a.hz << " endpoint_surface_max "
            << endpoint << " applied_surface_L1 " << applied;
  if (a.area_error >= 0)
    std::cout << " local_area_error " << a.area_error
              << " endpoint_local_area_error " << a.endpoint_area_error;
  std::cout << '\n';
}
// Offline stage boundary: hold recorded ACTUAL applied positions using the
// existing mechanical-jam fixture. No controller/actuator response is claimed
// here; body, rotor, aero and structural evolution still use Simulation::step.
Body plant(const Definition &d, const History &h, int hz, bool removed) {
  Simulation s(d, 1. / hz);
  auto tr = s.trim(70, 1000);
  for (int i = 0; i < 5 * hz; ++i) {
    if (removed && i == hz)
      s.remove_material("left_outer_forward", .2);
    auto p = tr.intent;
    p.mouse = false;
    p.active = {true, true, true};
    const auto &sample = h.samples[i * h.hz / hz];
    for (size_t j = 0; j < s.def.chains.size(); ++j) {
      auto &c = s.def.chains[j];
      c.jammed = true;
      c.jam_position = sample.applied[j];
    }
    s.step(p);
  }
  return s.body;
}

Body fresh_native(const Definition &d, bool removed) {
  Simulation s(d);
  auto tr = s.trim(70, 1000);
  auto p = tr.intent;
  p.mouse = true;
  p.aim_world = {1, .3, 0};
  for (int i = 0; i < 600; ++i) {
    if (removed && i == 120)
      s.remove_material("left_outer_forward", .2);
    p.active[Roll] = i >= 120 && i < 240;
    p.direct[Roll] = 1;
    s.evaluate(s.body, true);
    s.step(p);
  }
  return s.body;
}
Body locked_controller(const Definition &d, int plant_hz, bool removed) {
  Simulation plant(d, 1. / plant_hz), controller(d, 1. / 120);
  auto tr = plant.trim(70, 1000);
  controller.trim(70, 1000);
  auto p = tr.intent;
  p.mouse = true;
  p.aim_world = {1, .3, 0};
  Axis3 request{};
  for (int i = 0; i < 5 * plant_hz; ++i) {
    if (removed && i == plant_hz)
      plant.remove_material("left_outer_forward", .2);
    if (i % (plant_hz / 120) == 0) {
      // Fixed physical sensor instant, not a plant-dt-dependent old force.
      // Read-only evaluation observes actual ENDPOINT controls at this instant.
      plant.evaluate(plant.body, true);
      controller.body = plant.body;
      controller.load_factor = plant.load_factor;
      controller.density = 1.225 * std::exp(plant.body.position.z / 8500.);
      p.active[Roll] = i >= plant_hz && i < 2 * plant_hz;
      p.direct[Roll] = 1;
      controller.update_controls(p);
      for (int a = 0; a < 3; ++a)
        request[a] = controller.ownership[a].command;
    }
    auto held = tr.intent;
    held.active = {true, true, true};
    held.direct = request;
    plant.step(
        held); // Real physical control chains, never prescribed positions.
  }
  return plant.body;
}
// A raw body-normal aerodynamic force sample, not a previously normalized G.
// Each run uses integer ticks of 1/(2*plant_hz): completed force midpoints are
// odd ticks; the fixed 1/240 s age is plant_hz/120 ticks. Linear reconstruction
// uses only force values already available before the controller wakes.
struct ForceSample {
  long long tick;
  double fz;
};
struct LoadRead {
  double fz;
  long long left, right;
  bool prehistory = false;
};
LoadRead delayed(const std::vector<ForceSample> &samples, long long target,
                 double initial) {
  if (target < 0)
    return {initial, target, target, true};
  auto it = std::lower_bound(samples.begin(), samples.end(), target,
                             [](auto &a, long long t) { return a.tick < t; });
  if (it == samples.end())
    throw std::runtime_error("future/unbracketed force requested");
  if (it->tick == target)
    return {it->fz, it->tick, it->tick, false};
  if (it == samples.begin())
    throw std::runtime_error("missing older force bracket");
  auto lo = it - 1;
  double w = double(target - lo->tick) / (it->tick - lo->tick);
  return {lo->fz * (1 - w) + it->fz * w, lo->tick, it->tick, false};
}
struct AgedRun {
  Body body;
  double pos = 0, vel = 0, rates = 0, att = 0, g = 0, command = 0, bias = 0,
         weight = 0, end = 0, applied = 0;
  int pauses = 0;
  double max_span = 0, event_mass_before = 0, event_mass_after = 0,
         event_force_change = 0, event_g_error = 0;
};
void compare_body(const Body &a, const Body &b, AgedRun &r) {
  r.pos = std::max(r.pos, norm(a.position - b.position));
  r.vel = std::max(r.vel, norm(a.velocity - b.velocity));
  r.rates = std::max(r.rates, norm(a.omega - b.omega));
  r.att = std::max({r.att, std::abs(a.attitude.w - b.attitude.w),
                    std::abs(a.attitude.x - b.attitude.x),
                    std::abs(a.attitude.y - b.attitude.y),
                    std::abs(a.attitude.z - b.attitude.z)});
}
AgedRun aged_loop(const Definition &d, int hz, int removed,
                  const History *reference, std::ofstream &trace) {
  Simulation plant(d, 1. / hz), controller(d, 1. / 120);
  auto tr = plant.trim(70, 1000);
  controller.trim(70, 1000);
  auto p = tr.intent;
  p.mouse = true;
  p.aim_world = {1, .3, 0};
  double initial = plant.aero.force.z;
  std::vector<ForceSample> samples;
  Axis3 request{};
  AgedRun r;
  for (int i = 0; i < 5 * hz; ++i) {
    if (removed && i == hz) {
      r.event_mass_before = plant.mass.mass;
      double fz = plant.aero.force.z;
      plant.remove_material("left_outer_forward", .2);
      r.event_mass_after = plant.mass.mass;
      r.event_force_change = std::abs(plant.aero.force.z - fz);
      r.event_g_error =
          std::abs(plant.load_factor + fz / (plant.mass.mass * gravity));
    }
    if (i % (hz / 120) == 0) {
      long long target = 2LL * i - hz / 120;
      auto read = delayed(samples, target, initial);
      if (read.right >= 2LL * i)
        throw std::runtime_error("noncausal read");
      double normal = -read.fz / (plant.mass.mass * gravity);
      r.max_span = std::max(r.max_span, (read.right - read.left) / (2. * hz));
      controller.body = plant.body;
      controller.load_factor = normal;
      controller.density = 1.225 * std::exp(plant.body.position.z / 8500.);
      p.active[Roll] = i >= hz && i < 2 * hz;
      p.direct[Roll] = 1;
      controller.update_controls(p);
      for (int a = 0; a < 3; ++a)
        request[a] = controller.ownership[a].command;
      if (reference) {
        const auto &s = reference->samples[i];
        compare_body(plant.body, s.body, r);
        r.g = std::max(r.g, std::abs(normal - s.load));
        r.command = std::max(r.command, maxdiff(request, s.command));
        for (int a = 0; a < 3; ++a) {
          r.bias = std::max(
              r.bias, std::abs(controller.instructor.rate_bias[a] - s.bias[a]));
          r.weight = std::max(
              r.weight,
              std::abs(controller.ownership[a].direct_weight - s.weight[a]));
          r.pauses += controller.instructor.integral_paused[a] != s.paused[a];
        }
      }
      trace << removed << ',' << hz << ',' << i / double(hz) << ','
            << target / (2. * hz) << ',' << read.left / (2. * hz) << ','
            << read.right / (2. * hz) << ',' << read.fz << ','
            << plant.mass.mass << ',' << normal << ',' << plant.load_factor
            << ',' << read.prehistory << '\n';
    }
    auto held = tr.intent;
    held.active = {true, true, true};
    held.direct = request;
    plant.step(held);
    samples.push_back({2LL * i + 1, plant.aero.force.z});
    for (const auto &node : plant.def.nodes)
      if (!node.attached)
        throw std::runtime_error("aged-loop fixture changed control topology");
    if (reference) {
      const auto &s = reference->samples[i];
      for (size_t j = 0; j < plant.def.chains.size(); ++j) {
        r.end = std::max(
            r.end, std::abs(plant.def.chains[j].deflection - s.surfaces[j]));
        r.applied =
            std::max(r.applied, std::abs(applied_sample(plant.def.chains[j]) -
                                         s.applied[j]));
      }
    }
  }
  r.body = plant.body;
  if (reference) {
    compare_body(r.body, reference->final, r);
    if (std::max({r.pos, r.vel, r.rates, r.att, r.g, r.command, r.bias,
                  r.weight, r.end, r.applied}) > 1e-10 ||
        r.pauses)
      throw std::runtime_error("fixed-age loop lost native 120 Hz identity");
  }
  return r;
}
void verify_delayed_samples() {
  const std::vector<ForceSample> samples{{1, -10}, {3, -30}, {5, -50}};
  for (auto pair : {std::pair<long long, double>{1, -10},
                    {2, -20},
                    {3, -30},
                    {4, -40},
                    {5, -50}}) {
    if (delayed(samples, pair.first, 7).fz != pair.second)
      throw std::runtime_error("exact/ramp force reconstruction failed");
  }
  if (!delayed({}, -1, 7).prehistory || delayed({}, -1, 7).fz != 7)
    throw std::runtime_error("steady-trim initial cache extension failed");
  for (long long target : {0LL, 6LL}) {
    bool rejected = false;
    try {
      delayed(samples, target, 7);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    if (!rejected)
      throw std::runtime_error("missing force bracket must not extrapolate");
  }
}
void load_age_experiment(const Definition &d, const std::string &prefix) {
  verify_delayed_samples();
  std::ofstream trace(prefix + "-trace.csv");
  if (!trace)
    throw std::runtime_error("cannot write load sample trace");
  trace << std::setprecision(17)
        << "case,hz,control_time,force_time,left_time,right_time,raw_fz,mass,"
           "normal_g,plant_cache_g,prehistory\n";
  auto clean = record(d, 120, 0), damaged = record(d, 120, 1);
  for (int hz : {120, 240, 480, 960})
    for (int removed : {0, 1}) {
      const History *native =
          hz == 120 ? (removed ? &damaged : &clean) : nullptr;
      auto r = aged_loop(d, hz, removed, native, trace);
      std::cout << "aged " << removed << ' ' << hz << " x " << r.body.position.x
                << " y " << r.body.position.y << " z " << r.body.position.z
                << " force_age_s " << 1. / 240 << " reconstruction_span_s "
                << r.max_span << " identity_checked " << (native ? 1 : 0);
      if (native)
        std::cout << " position_error " << r.pos << " velocity_error " << r.vel
                  << " rates_error " << r.rates << " attitude_component_error "
                  << r.att << " load_g_error " << r.g << " request_error "
                  << r.command << " bias_error " << r.bias
                  << " ownership_error " << r.weight
                  << " surface_endpoint_error " << r.end
                  << " surface_force_error " << r.applied
                  << " pause_mismatches " << r.pauses;
      if (removed)
        std::cout << " event_mass_before " << r.event_mass_before
                  << " event_mass_after " << r.event_mass_after
                  << " event_cached_force_change " << r.event_force_change
                  << " event_cached_g_error " << r.event_g_error;
      std::cout << '\n';
    }
}
void write_body(std::ostream &o, const Body &b) {
  for (Vec v : {b.position, b.velocity, b.omega})
    o << v.x << ' ' << v.y << ' ' << v.z << ' ';
  o << b.attitude.w << ' ' << b.attitude.x << ' ' << b.attitude.y << ' '
    << b.attitude.z << ' ';
}
void read_body(std::istream &in, Body &b) {
  for (Vec *v : {&b.position, &b.velocity, &b.omega})
    in >> v->x >> v->y >> v->z;
  in >> b.attitude.w >> b.attitude.x >> b.attitude.y >> b.attitude.z;
}
void write_history(const History &h, const std::string &path) {
  std::ofstream o(path);
  if (!o)
    throw std::runtime_error("cannot write history");
  o << "surface_samples_v2 " << std::setprecision(17) << h.hz << ' '
    << h.samples.size() << ' ' << h.initial.chains.size() << '\n';
  for (auto &c : h.initial.chains)
    o << c.deflection << ' ' << c.filtered << '\n';
  for (const auto &a : h.samples) {
    write_body(o, a.body);
    o << a.load << ' ' << a.rho << ' ';
    for (double v : a.p.trim)
      o << v << ' ';
    o << a.p.throttle << ' ' << a.p.prop_rpm_ratio << ' ';
    for (const auto *v : {&a.command, &a.bias, &a.weight})
      for (double x : *v)
        o << x << ' ';
    for (bool v : a.paused)
      o << v << ' ';
    for (double v : a.surfaces)
      o << v << ' ';
    for (double v : a.applied)
      o << v << ' ';
    o << '\n';
  }
  write_body(o, h.final);
  o << '\n';
}
History read_history(const Definition &d, const std::string &path) {
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error("history missing: " + path);
  History h;
  size_t count = 0, chains = 0;
  std::string version;
  in >> version;
  bool midpoint_format = version == "surface_samples_v2";
  if (midpoint_format)
    in >> h.hz;
  else
    h.hz = std::stoi(version); // Earlier frozen baseline applied endpoints.
  in >> count >> chains;
  if ((h.hz != 120 && h.hz != 960) || count != size_t(5 * h.hz) ||
      chains != d.chains.size())
    throw std::runtime_error("invalid sensor fixture header");
  Simulation initial(d, 1. / h.hz);
  auto tr = initial.trim(70, 1000);
  if (!tr.converged)
    throw std::runtime_error("history entry trim failed");
  h.initial = initial.def;
  for (auto &c : h.initial.chains)
    in >> c.deflection >> c.filtered;
  for (size_t i = 0; i < count; ++i) {
    Sample a;
    read_body(in, a.body);
    in >> a.load >> a.rho;
    for (double &v : a.p.trim)
      in >> v;
    in >> a.p.throttle >> a.p.prop_rpm_ratio;
    a.p.mouse = true;
    a.p.aim_world = {1, .3, 0};
    a.p.active[Roll] = i >= size_t(h.hz) && i < size_t(2 * h.hz);
    a.p.direct[Roll] = 1;
    for (auto *v : {&a.command, &a.bias, &a.weight})
      for (double &x : *v)
        in >> x;
    for (auto &v : a.paused) {
      int n;
      in >> n;
      v = n;
    }
    for (size_t j = 0; j < chains; ++j) {
      double v;
      in >> v;
      a.surfaces.push_back(v);
    }
    if (midpoint_format) {
      for (size_t j = 0; j < chains; ++j) {
        double v;
        in >> v;
        a.applied.push_back(v);
      }
    } else
      a.applied = a.surfaces;
    h.samples.push_back(a);
  }
  read_body(in, h.final);
  if (!in)
    throw std::runtime_error("incomplete sensor fixture");
  return h;
}
int main(int argc, char **argv) {
  try {
    if (argc != 4)
      throw std::runtime_error(
          "airframe output-prefix record|replay|age required");
    auto d = Definition::load(argv[1]);
    std::string prefix = argv[2], mode = argv[3];
    auto name = [&](int hz, int c) {
      return prefix + "-" + std::to_string(hz) + "-" + std::to_string(c) +
             ".txt";
    };
    std::cout << std::setprecision(15);
    if (mode == "age") {
      std::cout << std::setprecision(17);
      load_age_experiment(d, prefix);
      return 0;
    }
    if (mode == "record") {
      for (int hz : {120, 960})
        for (int c : {0, 1}) {
          auto h = record(d, hz, c);
          write_history(h, name(hz, c));
          std::cout << "recorded " << hz << ' ' << c << ' ' << h.samples.size()
                    << '\n';
        }
      return 0;
    }
    if (mode != "replay")
      throw std::runtime_error("unknown probe mode");
    std::ofstream trace(prefix + "-release.csv");
    trace << std::setprecision(15)
          << "mode,case,hz,time,weight,pause,bias,ref_pause,ref_bias\n";
    auto c120 = read_history(d, name(120, 0)),
         d120 = read_history(d, name(120, 1));
    for (int removed : {0, 1}) {
      auto &coarse = removed ? d120 : c120;
      auto ident = control(coarse, 120, 120);
      double e = 0, chain_error = 0;
      for (size_t i = 0; i < ident.samples.size(); ++i) {
        e = std::max(
            e, maxdiff(ident.samples[i].command, coarse.samples[i].command));
        for (size_t j = 0; j < coarse.initial.chains.size(); ++j)
          chain_error =
              std::max(chain_error, std::abs(ident.samples[i].surfaces[j] -
                                             coarse.samples[i].surfaces[j]));
      }
      if (e > 1e-13 || chain_error > 1e-13)
        throw std::runtime_error(
            "sensor replay lost native request/actuator identity");
      std::cout << "identity " << removed << ' ' << e << ' ' << chain_error
                << '\n';
      auto fine = read_history(d, name(960, removed));
      auto pi_ref = control(fine, 120, 960);
      auto full_ref = control(fine, 960, 960);
      for (int hz : {120, 240, 480, 960}) {
        compare("internal", removed, control(fine, 120, hz), pi_ref, 120,
                trace);
        compare("sensor_" + std::to_string(hz), removed, control(fine, hz, 960),
                full_ref, 960, trace);
      }
      // Freeze baseline requests as well as sensors for the actuator-only
      // bench.
      Controls requests;
      requests.hz = fine.hz;
      requests.samples = fine.samples;
      auto act_ref = actuators(fine, requests, 120, 120, 960);
      auto hold_ref = actuators(fine, requests, 960, 120, 960);
      for (int hz : {120, 240, 480, 960}) {
        compare_act("actuator", removed,
                    actuators(fine, requests, 120, 120, hz), act_ref);
        compare_act("request_hold_" + std::to_string(hz), removed,
                    actuators(fine, requests, hz, 120, 960), hold_ref);
      }
    }
    for (int hz : {120, 240, 480, 960}) {
      auto a = plant(d, c120, hz, false), b = plant(d, d120, hz, true);
      auto e = b.position - a.position;
      double identity = hz == 120 ? norm(a.position - c120.final.position) +
                                        norm(b.position - d120.final.position)
                                  : 0;
      if (identity > 1e-8)
        throw std::runtime_error(
            "prescribed-surface replay lost native 120 Hz identity");
      std::cout << "plant " << hz << ' ' << e.x << ' ' << e.y << ' ' << e.z
                << " norm " << norm(e) << " identity " << identity << '\n';
    }
    auto clean_fresh = fresh_native(d, false),
         damaged_fresh = fresh_native(d, true);
    for (int hz : {120, 240, 480, 960}) {
      auto a = locked_controller(d, hz, false),
           b = locked_controller(d, hz, true);
      auto e = b.position - a.position;
      double identity = hz == 120
                            ? norm(a.position - clean_fresh.position) +
                                  norm(b.position - damaged_fresh.position)
                            : 0;
      if (identity > 1e-8)
        throw std::runtime_error("locked loop lost fresh-native identity");
      std::cout << "locked " << hz << ' ' << e.x << ' ' << e.y << ' ' << e.z
                << " norm " << norm(e) << " identity " << identity << '\n';
    }

  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
