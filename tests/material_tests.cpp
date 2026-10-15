#include "flight/core.hpp"
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace flight;
void require(bool ok, const char *m) {
  if (!ok)
    throw std::runtime_error(m);
}
struct Ledger {
  double mass = 0, energy = 0;
  Vec p{}, h{};
};
Ledger ledger(const Simulation &s, Vec origin) {
  Ledger out;
  auto add = [&](const Body &b, const MassProperties &m, const Engine &e) {
    out.mass += m.mass;
    out.energy += mechanical_energy(b, m, e);
    Vec p = b.velocity * m.mass;
    out.p += p;
    out.h += total_angular_momentum(b, m, e) + cross(b.position - origin, p);
  };
  add(s.body, s.mass, s.def.engine);
  Engine passive;
  for (auto &piece : s.debris)
    add(piece.state, piece.mass, passive);
  return out;
}
void sever(Simulation &s) {
  for (auto &e : s.def.edges)
    if (e.a == 0 && e.b == 1)
      e.failed = true;
  std::vector<double> zero(s.def.nodes.size());
  s.solve_structure(zero, zero, 0);
}
int main(int argc, char **argv) {
  try {
    if (argc != 3)
      throw std::runtime_error("material and unsplit BEM definitions required");
    auto def = Definition::load(argv[1]);
    Simulation s(def), reference(Definition::load(argv[2]));
    double initial_error = std::abs(s.mass.mass - reference.mass.mass) +
                           norm(s.mass.cg - reference.mass.cg);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        initial_error +=
            std::abs(s.mass.inertia.a[i][j] - reference.mass.inertia.a[i][j]);
    require(initial_error < 1e-9,
            "material discretization preserves initial mass/CG/full inertia");
    s.body.position = {120, -25, -1000};
    s.body.velocity = {65, 10, -5};
    s.body.omega = {.3, -.2, .1};
    s.body.attitude = (Quat::axis({0, 0, 1}, .7) * Quat::axis({0, 1, 0}, .3) *
                       Quat::axis({1, 0, 0}, .4))
                          .normalized();
    s.def.engine.rpm = 1234;
    s.def.engine.prop_fraction = 0;
    Vec origin = s.body.position;
    auto original = ledger(s, origin);
    auto mass = s.mass;
    auto body = s.body;
    Vec datum = body.position - body.attitude.rotate(mass.cg);
    s.evaluate(s.body, true);
    auto cells = s.def.cells;
    s.remove_material("left_outer_forward", .2);
    auto after = ledger(s, origin);
    double p_error = norm(after.p - original.p),
           h_error = norm(after.h - original.h),
           e_error = std::abs(after.energy - original.energy);
    require(std::abs(after.mass - original.mass) < 1e-10 && p_error < 1e-8 &&
                h_error < 1e-7 && e_error < 1e-6,
            "root removal conserves total mass/P/H/energy including spinning "
            "rotor and removed intrinsic inertia");
    require(
        s.mass.mass == mass.mass - 8 && s.debris.size() == 1 &&
            s.debris[0].material_name == "left_outer_forward",
        "8 kg leaves inventory once and is owned by a separate material body");
    require(s.tick == 0 && norm(s.body.omega - body.omega) == 0 &&
                s.body.attitude.w == body.attitude.w,
            "event supplies no ejection impulse, attitude write or extra "
            "physical tick");
    require(norm(s.body.position - s.body.attitude.rotate(s.mass.cg) - datum) <
                1e-10,
            "CG reference update leaves surviving geometry continuous");
    require(s.mass.cg.y > mass.cg.y && std::abs(s.mass.inertia.a[0][1]) > 1e-3,
            "local left-side loss shifts CG and full off-diagonal inertia");
    s.evaluate(s.body, true);
    for (std::size_t i = 0; i < cells.size(); ++i) {
      require(norm(s.def.cells[i].flow - cells[i].flow) < 1e-9,
              "surviving cell velocity field is continuous");
      require(s.def.cells[i].area_fraction == cells[i].area_fraction,
              "material event cannot invent aero area loss");
    }
    require(std::all_of(s.def.nodes.begin(), s.def.nodes.end(),
                        [](auto &n) { return n.attached; }) &&
                std::none_of(s.def.edges.begin(), s.def.edges.end(),
                             [](auto &e) {
                               return e.failed || e.remaining_section != 1;
                             }),
            "mass removal neither weakens nor severs structural connections "
            "implicitly");
    auto fixed = s;
    auto count = s.debris.size();
    s.remove_material("left_outer_forward", .2);
    require(s.debris.size() == count && s.mass.mass == fixed.mass.mass,
            "repeated remaining quantity is idempotent");
    for (double invalid :
         {.3, -.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
      bool rejected = false;
      try {
        s.remove_material("left_outer_forward", invalid);
      } catch (const std::exception &) {
        rejected = true;
      }
      require(rejected && s.mass.mass == fixed.mass.mass &&
                  s.debris.size() == count,
              "invalid/restoring event is atomic");
    }
    // Repeated partial removal plus removal after graph separation use the
    // current owner.
    s.remove_material("left_outer_forward", 0);
    auto before_split = ledger(s, origin);
    sever(s);
    auto split = ledger(s, origin);
    require(norm(before_split.p - split.p) < 1e-8 &&
                norm(before_split.h - split.h) < 1e-7 &&
                std::abs(before_split.energy - split.energy) < 1e-6,
            "later structural split cannot duplicate ejected mass");
    double parent_mass = s.mass.mass;
    auto before_piece = ledger(s, origin);
    s.remove_material("left_outer_aft", 0);
    auto after_piece = ledger(s, origin);
    require(s.mass.mass == parent_mass &&
                norm(before_piece.p - after_piece.p) < 1e-8 &&
                norm(before_piece.h - after_piece.h) < 1e-7 &&
                std::abs(before_piece.energy - after_piece.energy) < 1e-6,
            "removal from a detached wing changes that owner, not the parent");
    double event_mass_error = std::abs(after_piece.mass - original.mass);
    require(event_mass_error < 1e-10, "inventory plus structural bodies plus "
                                      "material bodies retain total mass");
    // Area loss alone is still an aerodynamic event with no implicit mass loss.
    Simulation aero(def);
    double whole = aero.mass.mass;
    aero.damage_cell(0, 0, 1);
    require(aero.mass.mass == whole && aero.debris.empty(),
            "area loss is not silently reinterpreted as material loss");
    // Free flight after ejection: gravity only, with rotor/body gyroscopic
    // exchange.
    Simulation free(def);
    free.def.body_drag_area = 0;
    free.def.engine.prop_fraction = 0;
    free.def.engine.max_power = 0;
    free.def.engine.engine_friction = 0;
    for (auto &curve : free.def.curves)
      for (auto &c : curve.second)
        c.cl = c.cd = c.cm = 0;
    free.body.omega = {.1, .05, -.02};
    free.remove_material("left_outer_forward", 0);
    auto free0 = ledger(free, {});
    PilotIntent neutral;
    for (int i = 0; i < 60; ++i)
      free.step(neutral);
    auto free1 = ledger(free, {});
    double free_energy = std::abs(free1.energy - free0.energy);
    require(free_energy < 1e-5, "physical integration of parent and removed "
                                "parcel retains free-flight energy");
    // Same real input and airframe, mass-only event: force/moment integration
    // must use changed mass/CG/inertia while keyboard ownership stays exact.
    std::vector<Body> flown;
    for (int scenario = 0; scenario < 4; ++scenario) {
      double hz = scenario >= 2 ? 240. : 120.;
      Simulation plane(def, 1 / hz);
      auto trim = plane.trim(70, 1000);
      require(trim.converged, "material fixture trims before removal");
      auto command = trim.intent;
      command.mouse = true;
      command.aim_world = {1, .3, 0};
      for (int i = 0; i < int(5 * hz); ++i) {
        double time = i / hz;
        if ((scenario == 1 || scenario == 2) && i == int(hz))
          plane.remove_material("left_outer_forward", .2);
        command.active[Roll] = time >= 1 && time < 2;
        command.direct[Roll] = 1;
        plane.step(command);
        if (command.active[Roll])
          require(plane.ownership[Roll].direct_weight == 1 &&
                      plane.ownership[Roll].command == 1,
                  "material loss cannot bypass or weaken per-axis keyboard "
                  "authority");
      }
      require(std::all_of(plane.def.nodes.begin(), plane.def.nodes.end(),
                          [](auto &n) { return n.attached; }),
              "material-only flight retains structural connectivity");
      flown.push_back(plane.body);
    }
    double flight_change = norm(flown[1].position - flown[0].position),
           flight_convergence = norm(flown[2].position - flown[1].position);
    double fine_effect = norm(flown[2].position - flown[3].position);
    double effect_disagreement = norm((flown[1].position - flown[0].position) -
                                      (flown[2].position - flown[3].position));
    bool full_roll_effect_resolved =
        fine_effect > 1e-4 && effect_disagreement < flight_change * .1;
    require(
        flight_change > 1e-4 && flight_convergence < 1,
        "mass-only flight changes physically and bulk state refines at 240 Hz");
    // Isolate mass properties from hybrid guidance decisions with matched,
    // fixed-trim open-loop flight. Keep the full-roll counterexample reported.
    std::vector<Body> open;
    for (int scenario = 0; scenario < 4; ++scenario) {
      double hz = scenario >= 2 ? 240. : 120.;
      Simulation plane(def, 1 / hz);
      auto trim = plane.trim(70, 1000);
      for (int i = 0; i < int(5 * hz); ++i) {
        if ((scenario == 1 || scenario == 2) && i == int(hz))
          plane.remove_material("left_outer_forward", .2);
        plane.step(trim.intent);
      }
      open.push_back(plane.body);
    }
    double open_effect = norm(open[1].position - open[0].position);
    double open_fine_effect = norm(open[2].position - open[3].position);
    double open_disagreement = norm((open[1].position - open[0].position) -
                                    (open[2].position - open[3].position));
    require(open_effect > 1e-3 && open_disagreement < open_effect * .1,
            "fixed-trim mass-only response agrees under refinement");
    // An arbitrarily small positive removal must remain integrable, not hit a
    // mass floor.
    Simulation tiny(def);
    tiny.remove_material("left_outer_forward", 1 - 1e-12);
    tiny.step(neutral);
    require(std::isfinite(tiny.debris[0].state.omega.x),
            "small material parcels are physically integrable");
    std::cout << std::setprecision(13)
              << "{\"checks_passed\":true,\"initial_discretization_error\":"
              << initial_error << ",\"mass_error_kg\":" << event_mass_error
              << ",\"linear_momentum_error\":" << p_error
              << ",\"angular_momentum_error\":" << h_error
              << ",\"instant_energy_error_J\":" << e_error
              << ",\"cg_y_shift_m\":" << fixed.mass.cg.y - mass.cg.y
              << ",\"Ixy_after_kgm2\":" << fixed.mass.inertia.a[0][1]
              << ",\"free_energy_error_J\":" << free_energy
              << ",\"flight_position_effect_m\":" << flight_change
              << ",\"flight_position_120_240_difference_m\":"
              << flight_convergence
              << ",\"flight_effect_240hz_m\":" << fine_effect
              << ",\"flight_effect_disagreement_m\":" << effect_disagreement
              << ",\"full_roll_effect_agreement_10pct\":"
              << (full_roll_effect_resolved ? "true" : "false")
              << ",\"open_loop_effect_m\":" << open_effect
              << ",\"open_loop_effect_240hz_m\":" << open_fine_effect
              << ",\"open_loop_effect_disagreement_m\":" << open_disagreement
              << ",\"tiny_parcel_mass_kg\":" << tiny.debris[0].mass.mass
              << "}\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
