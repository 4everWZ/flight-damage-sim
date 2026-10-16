#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;

// Static physical-capability probe, not a replacement flight model. Preserve
// the loaded wing polar and real cell evaluator; remove propulsion/body/tail
// loads to isolate the lifting surface. No trim, time integration or controller.
Definition wing_only(Definition d, double scale, int damage, bool refine) {
  d.engine.prop_fraction = 0;
  d.engine.max_power = 0;
  d.engine.rpm = 0;
  d.body_drag_area = 0;
  std::vector<AeroCell> cells;
  for (auto c : d.cells) {
    if (c.table != "wing")
      continue;
    c.chain = -1;
    c.incidence = 0;
    c.position.x /= scale;
    c.position.y *= scale;
    c.span *= scale;
    c.chord /= scale; // Constant planform area, changed aspect ratio.
    if (damage == 1 && std::abs(c.position.y) > 3.75 * scale)
      c.area_fraction = 0; // Outer quarter of each half-wing: no roughness.
    if (damage == 2)
      c.area_fraction = .75; // Same lost area, original full-span footprint.
    if (refine) {
      c.area *= .5;
      c.id *= 2;
      cells.push_back(c);
      ++c.id;
    }
    cells.push_back(c);
  }
  d.cells = cells;
  return d;
}

void sample(const Definition &d, const char *name, double scale, double alpha,
            double hz) {
  Simulation s(d, 1 / hz);
  s.body.position = {0, 0, -1000};
  Vec direction{std::cos(alpha), 0, std::sin(alpha)};
  s.body.velocity = direction * 70;
  s.body.omega = {};
  auto w = s.evaluate(s.body, true);
  double area = 0, span = 0, inner_fz = 0, inner_q_area = 0, tip = 0;
  double q = .5 * 1.225 * std::exp(-1000. / 8500) * 70 * 70;
  for (const auto &c : s.def.cells) {
    area += c.area * c.area_fraction;
    if (c.area_fraction > 0)
      span = std::max(span, 2 * std::abs(c.position.y) + c.span);
    if (std::abs(c.position.y) < 3 * scale) {
      inner_fz += c.force.z;
      inner_q_area += c.q * c.area * c.area_fraction;
    }
    if (std::abs(c.position.y) > 3.75 * scale)
      tip += c.area * c.area_fraction;
  }
  Vec lift_direction{std::sin(alpha), 0, -std::cos(alpha)};
  std::cout << name << ',' << scale << ',' << alpha << ',' << hz << ','
            << s.def.cells.size() << ',' << area << ',' << span << ','
            << span * span / area << ',' << dot(w.force, lift_direction)
            << ',' << -dot(w.force, direction) << ','
            << dot(w.force, lift_direction) / (q * area) << ','
            << -dot(w.force, direction) / (q * area) << ','
            << inner_fz / inner_q_area << ',' << tip << '\n';
}

int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("airframe required");
    auto d = Definition::load(argv[1]);
    std::cout << std::setprecision(17)
              << "case,span_scale,alpha_rad,hz,cells,area_m2,span_m,aspect_ratio,"
                 "lift_N,drag_N,CL,CD,inner_Fz_over_qA,tip_area_m2\n";
    for (double hz : {120., 240.})
      for (double a : {0., 5., 10.}) {
        for (double scale : {std::sqrt(.5), 1., std::sqrt(2.)})
          sample(wing_only(d, scale, 0, false), "geometry", scale, rad(a), hz);
        sample(wing_only(d, 1, 0, true), "refined", 1, rad(a), hz);
        sample(wing_only(d, 1, 1, false), "tip_loss", 1, rad(a), hz);
        sample(wing_only(d, 1, 2, false), "distributed_loss", 1, rad(a), hz);
      }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
