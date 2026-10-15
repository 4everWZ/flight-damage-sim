#include "flight/core.hpp"
#include <iostream>
#include <stdexcept>
using namespace flight;
void require(bool okay, const char *message) {
  if (!okay)
    throw std::runtime_error(message);
}
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("BEM airframe required");
    auto def = Definition::load(argv[1]);
    auto e = def.engine;
    e.governor = false;
    double rho = 1.1;
    auto stopped = propeller_loads(e, rho, {80, 0, 0}, {}, 0);
    require(stopped.at_mount.force.x < -100 && stopped.shaft_load < -10,
            "stopped intact blades must drag and drive windmilling");
    auto tilted = e;
    tilted.axis = unit({1, .3, -.2});
    prepare_propeller(tilted);
    auto tilted_load = propeller_loads(tilted, rho, tilted.axis * 80, {}, 0);
    require(norm(tilted_load.at_mount.force -
                 tilted.axis * stopped.at_mount.force.x) < 1e-8 &&
                std::abs(tilted_load.shaft_load - stopped.shaft_load) < 1e-8,
            "prop force and shaft torque follow the installed axis");
    auto axial_step = e, tilted_step = tilted;
    advance_propulsion(axial_step, rho, 1000, 80, .7, 1. / 120);
    advance_propulsion(tilted_step, rho, 1000, 80, .7, 1. / 120);
    require(std::abs(axial_step.rpm - tilted_step.rpm) < 1e-9,
            "axial component bench uses the configured prop axis");
    auto removed = e;
    removed.prop_fraction = 0;
    auto missing = propeller_loads(removed, rho, {80, 0, 0}, {}, 0);
    require(norm(missing.at_mount.force) == 0 && missing.shaft_load == 0,
            "no blade area: no prop aerodynamic force");
    auto static_prop = propeller_loads(e, rho, {}, {}, 250);
    double thrust = static_prop.at_mount.force.x,
           ideal = std::sqrt(thrust * thrust * thrust /
                             (2 * rho * pi * e.prop_radius * e.prop_radius));
    require(thrust > 0 && static_prop.shaft_load * 250 >= ideal,
            "static thrust requires induced power");
    double worst = 0, minloss = 1e100;
    for (double spin : {-250., 0., 120., 250.})
      for (double speed : {-100., 0., 40., 100., 160.}) {
        Vec flow{speed, 5, -3}, rates{.1, .2, -.1};
        auto p = propeller_loads(e, rho, flow, rates, spin);
        double work = dot(p.at_mount.force, flow) +
                      dot(p.at_mount.moment, rates) - p.shaft_load * spin;
        double losses = p.profile_loss + p.induced_loss;
        worst = std::max(worst, std::abs(work + losses) / std::max(1., losses));
        minloss = std::min(minloss, losses);
        require(p.converged && std::isfinite(work) && losses >= -1e-6,
                "signed induction must converge with nonnegative aerodynamic "
                "losses");
      }
    require(worst < 1e-9, "blade lift/drag and induced-flow power balance");
    auto a = propeller_loads(e, rho, {40, 0, 0}, {}, 200),
         b = propeller_loads(e, rho, {100, 0, 0}, {}, 200);
    require(std::abs(a.shaft_load - b.shaft_load) > 100,
            "fixed pitch torque changes with advance");
    auto coarse = e;
    coarse.radial_samples = 6;
    coarse.azimuth_samples = 8;
    prepare_propeller(coarse);
    auto fine = e;
    fine.radial_samples = 24;
    fine.azimuth_samples = 16;
    prepare_propeller(fine);
    auto c = propeller_loads(coarse, rho, {70, 3, 4}, {}, 250),
         f = propeller_loads(fine, rho, {70, 3, 4}, {}, 250);
    require(norm(c.at_mount.force - f.at_mount.force) /
                    std::max(norm(f.at_mount.force), 1.) <
                .08,
            "blade geometry refinement converges");
    // Actual coupled free body: internal motor work changes energy but cannot
    // create net angular momentum.
    Engine rotor;
    rotor.model = PropellerModel::BladeElement;
    rotor.axis = unit({1, .2, .1});
    rotor.rotor_inertia = 3;
    rotor.prop_fraction = 0;
    rotor.rpm = 1800;
    rotor.engine_friction = 0;
    rotor.rated_rpm = 2400;
    rotor.max_power = 5000;
    rotor.altitude_scale = 12000;
    rotor.throttle_state = 1;
    MassProperties m;
    m.mass = 100;
    m.inertia = Mat::diagonal({20, 30, 40});
    m.inertia.a[0][2] = m.inertia.a[2][0] = 2;
    Body body;
    body.position = {0, 0, -100};
    body.velocity = {20, 2, 0};
    body.omega = {.5, .2, -.1};
    body.attitude =
        (Quat::axis({0, 0, 1}, .7) * Quat::axis({0, 1, 0}, .4)).normalized();
    double energy = mechanical_energy(body, m, rotor), work = 0;
    Vec angular = total_angular_momentum(body, m, rotor);
    for (int i = 0; i < 120; ++i) {
      auto step =
          integrate_rotor_body(body, m, rotor, Wrench{}, {}, rho, 1. / 120);
      work += step.engine_work;
    }
    double energy_error =
        std::abs(mechanical_energy(body, m, rotor) - energy - work) /
        std::max(work, 1.);
    double angular_error =
        norm(total_angular_momentum(body, m, rotor) - angular) / norm(angular);
    require(energy_error < 1e-9 && angular_error < 1e-9,
            "free-flight rotor/body energy and angular momentum accounting");
    // Power off is the conservation counterexample; relative RPM is allowed to
    // change as the body precesses.
    rotor.running = false;
    energy = mechanical_energy(body, m, rotor);
    angular = total_angular_momentum(body, m, rotor);
    for (int i = 0; i < 120; ++i)
      integrate_rotor_body(body, m, rotor, Wrench{}, {}, rho, 1. / 120);
    require(std::abs(mechanical_energy(body, m, rotor) - energy) < 1e-6 &&
                norm(total_angular_momentum(body, m, rotor) - angular) < 1e-7,
            "spinning free body conservative without power/air loads");
    std::cout << "PASS: signed/stopped/removed loads, static power, full power "
                 "balance, advance/refinement, coupled energy/H\n";
    std::cout << "stopped_drag_N=" << stopped.at_mount.force.x
              << " wind_torque_Nm=" << stopped.shaft_load
              << " power_balance_relative=" << worst
              << " min_loss_W=" << minloss
              << " free_energy_relative=" << energy_error
              << " free_H_relative=" << angular_error << '\n';
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
