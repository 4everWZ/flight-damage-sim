#pragma once
#include "math.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
namespace flight {
// SI units. Body FRD (forward/right/down), flat local world NED.
// Geometry is at a fixed datum; Body position/velocity are at the current CG.
// Quaternions map body vectors to world; omega is a body-frame angular rate.
// Input order is roll/pitch/yaw, independent of device bindings.
enum Axis { Roll = 0, Pitch = 1, Yaw = 2 };
struct PilotIntent {
  Vec aim_world{1, 0, 0};
  bool mouse = false;
  std::array<double, 3> direct{}, trim{};
  std::array<bool, 3> active{};
  double throttle = 0, prop_rpm_ratio = 1;
};
struct Body {
  Vec position{0, 0, -1000}, velocity{70, 0, 0}, omega{};
  Quat attitude{};
};
struct MassPart {
  int node = 0;
  double mass = 0;
  Vec position{}, inertia{};
  std::string material_name; // Empty for non-removable carrier mass.
  double remaining =
      1; // Fraction of this explicitly defined material inventory.
};
struct MassProperties {
  double mass = 0;
  Vec cg{};
  Mat inertia{};
};
struct CurvePoint {
  double alpha = 0, cl = 0, cd = 0, cm = 0;
};
struct ControlPolarSlice {
  double delta = 0;
  std::vector<CurvePoint> points;
};
struct ControlPolar {
  std::string name;
  std::vector<ControlPolarSlice> slices; // Rectangular delta/flow-alpha grid.
};
struct AeroCell {
  int id = 0, node = 0, chain = -1, plane = 0; // plane 0 horizontal, 1 vertical
  Vec position{};
  double area = 0, chord = 1, span = 1, incidence = 0, control_sign = 0,
         control_gain = 0;
  std::string table;
  double area_fraction = 1, lift_efficiency = 1, roughness = 0;
  bool attached = true;
  Vec force{}, moment{}, flow{};
  double alpha = 0, beta = 0, q = 0, stall = 0, deflection = 0;
  // Additional unit rotation of the legacy plane-specific basis in body axes.
  // Directions use elastic(body axes) * reference_frame * incidence.
  // Position/flow/force/moment remain body-axis vectors; default is identity.
  Quat reference_frame{};
  // Empty preserves the legacy effective-alpha lookup. An explicit binding
  // samples CL/CD/CM(flow_alpha, signed chain endpoint * hinge).
  std::string control_polar;
  // alpha/stall retain the legacy effective-alpha/15-degree diagnostic proxy,
  // including when this optional lookup is selected; neither is separation data.
  double flow_alpha = 0;
  bool control_polar_clamped = false;
};
struct ControlChain {
  std::string name;
  Axis axis = Roll;
  double limit = rad(20), slew = rad(90), q_limit = 18000;
  double range = 1, friction = 0, lag = 0, hinge = 1, deflection = 0,
         filtered = 0;
  // Force sample for the last advanced interval; deflection remains its
  // endpoint.
  double midpoint_deflection = 0;
  bool connected = true, jammed = false;
  double jam_position = 0;
};
struct AxisOwnership {
  double direct_weight = 0, release_from = 0, release_elapsed = .25,
         last_direct = 0, command = 0;
  bool was_active = false;
};
enum class InstructorMode { Off, Legacy, FlightPath };
struct GuidanceParameters {
  bool rate_integral = true;
  bool aim_rate_feedforward = true;
  double max_bank = rad(40), heading_gain = .35, path_gain = .6,
         normal_gain = .5;
};
struct InstructorState {
  Vec aim_body{}, desired_rates{};
  double desired_bank = 0, desired_normal_g = 1, flight_path_error = 0;
  // World-heading history, independent of aircraft rotation. Invalidated when
  // mouse/FlightPath guidance is inactive or the aim has no horizontal heading.
  double previous_aim_heading = 0, aim_heading_rate = 0;
  bool aim_history_valid = false;
  std::array<bool, 3> saturated{};
  std::array<double, 3> rate_bias{};
  std::array<bool, 3> integral_paused{};
  std::array<double, 3> command{};
};
struct StructuralNode {
  std::string name;
  bool attached = true;
  double bend = 0, twist = 0;
};
struct StructuralEdge {
  std::string name;
  int a = 0, b = 0;
  double bend_k = 0, twist_k = 0, section_modulus = 0, torsion_modulus = 0,
         yield_stress = 0, ultimate_stress = 0;
  double remaining_section = 1, plastic_strain = 0, failure_strain = .04,
         plastic_rate = .02;
  double bend_load = 0, twist_load = 0, utilization = 0;
  bool failed = false;
  double stiffness() const {
    return std::pow(remaining_section, 2) * std::exp(-12 * plastic_strain);
  }
};
enum class PropellerModel { Legacy, BladeElement };
struct BladeStation {
  double radius_fraction = 0, chord = 0, twist = 0;
};
struct BladeStrip {
  Vec offset{}, radial{};
  double area = 0, twist = 0;
};
struct PropellerLoads {
  Wrench at_mount{};
  double shaft_load = 0, induced = 0, profile_loss = 0, induced_loss = 0;
  double momentum_residual = 0, tip_mach = 0;
  bool converged = false;
};
struct Engine {
  Vec position{}, axis{1, 0, 0};
  PropellerModel model = PropellerModel::Legacy;
  std::vector<BladeStation> stations;
  std::vector<CurvePoint> polar;
  std::vector<BladeStrip> strips;
  int blades = 2, radial_samples = 12, azimuth_samples = 8;
  double pitch = rad(22), pitch_min = rad(8), pitch_max = rad(55),
         pitch_slew = rad(10);
  double governor_kp = rad(18), governor_ki = rad(8), governor_integral = 0,
         target_rpm = 0;
  double pitch_range = 1, pitch_rate_fraction = 1, pump_rpm = 400,
         engine_friction = .15;
  bool governor = false, pitch_jammed = false, governor_saturated = false;
  double induced_velocity = 0, profile_loss = 0, induced_loss = 0, tip_mach = 0;
  double induction_residual = 0;
  bool induction_converged = false;
  double max_power = 0, rated_rpm = 0, prop_radius = 0, rotor_inertia = 0,
         ct = 0, cq = 0, advance_loss = 0, altitude_scale = 0, response = 0;
  double acceleration = 0, aero_torque = 0;
  double rpm = 0, throttle_state = 0, power_fraction = 1, prop_fraction = 1,
         thrust = 0, torque = 0, shaft_power = 0;
  bool running = true; // Output-shaft map; no combustion/starter model.
};
// Authoritative component step. SI: density kg/m^3, altitude/airspeed m and
// m/s.
void advance_propulsion(Engine &, double density, double altitude,
                        double axial_airspeed, double throttle, double dt);
void prepare_propeller(Engine &);
PropellerLoads propeller_loads(const Engine &, double density,
                               Vec velocity_at_mount, Vec body_rates,
                               double relative_omega);
double engine_shaft_torque(const Engine &, double altitude,
                           double relative_omega);
void advance_propeller_controls(Engine &, double throttle, double dt);
struct Definition {
  std::vector<AeroCell> cells;
  std::vector<ControlChain> chains;
  std::vector<MassPart> masses;
  std::vector<StructuralNode> nodes;
  std::vector<StructuralEdge> edges;
  std::vector<std::pair<std::string, std::vector<CurvePoint>>> curves;
  std::vector<ControlPolar> control_polars;
  Engine engine;
  double body_drag_area = 0, handoff = .25;
  std::array<bool, 3> trim_enabled{true, true, true};
  static Definition load(const std::string &path);
};
void integrate_body(Body &, const MassProperties &, const Wrench &,
                    double elapsed);
// State-dependent loads are solved with the rigid state, at the same midpoint.
using BodyLoad = std::function<Wrench(const Body &)>;
void integrate_body(Body &, const MassProperties &, const BodyLoad &,
                    double elapsed);
using RotorAirframeLoad =
    std::function<Wrench(const Body &, const PropellerLoads &)>;
struct RotorStep {
  double engine_work = 0, external_work = 0, prop_loss = 0;
  Vec impulse_world{};
};
RotorStep integrate_rotor_body(Body &, const MassProperties &, Engine &,
                               const Wrench &aero, Vec wind, double density,
                               double dt, const RotorAirframeLoad & = {});
double mechanical_energy(const Body &, const MassProperties &, const Engine &);
Vec total_angular_momentum(const Body &, const MassProperties &,
                           const Engine &);
struct DetachedCell {
  AeroCell cell;
  Quat frame; // Frozen elastic/incidence basis at separation, in piece body
              // axes.
};
struct DetachedBody {
  std::string material_name; // Nonempty for removed material: no aero geometry.
  Body state;
  MassProperties mass;
  std::vector<int> cells, nodes; // Original definition indices, for provenance.
  std::vector<DetachedCell> aero_cells;
  std::vector<ControlChain>
      chains; // Local unpowered/jammed surfaces; no pilot link.
  Wrench aero{};
};
enum class TrimMode { Level, FixedThrottle };
struct TrimResult {
  PilotIntent intent;
  double alpha = 0, bank = 0, residual = 0;
  Vec force_residual_N{}, moment_residual_Nm{};
  double rotor_acceleration = 0;
  bool converged = false;
  int iterations = 0;
  // Positive climb angle in the air mass. Level retains its legacy entry
  // construction; FixedThrottle prescribes TAS and adds constant world wind.
  double flight_path = 0;
  TrimMode mode = TrimMode::Level;
  double propeller_pitch_rate = 0;
};
class Simulation {
public:
  Definition def;
  Body body;
  MassProperties mass;
  std::vector<DetachedBody> debris;
  std::array<AxisOwnership, 3> ownership;
  InstructorState instructor;
  InstructorMode instructor_mode = InstructorMode::FlightPath;
  GuidanceParameters guidance;
  Vec wind_world{};
  std::uint64_t tick = 0;
  double dt = 1. / 120;
  Wrench aero{}, propulsion{};
  double density = 1.1, load_factor = 0;
  RotorStep rotor_step;
  explicit Simulation(Definition d, double timestep = 1. / 120);
  void step(const PilotIntent &input);
  Wrench evaluate(const Body &b, bool store = false);
  void update_controls(const PilotIntent &input);
  Wrench evaluate_detached(DetachedBody &, bool store = true);
  void solve_structure(const std::vector<double> &bending,
                       const std::vector<double> &torsion, double elapsed);
  void damage_cell(int id, double area_fraction, double roughness);
  void weaken(const std::string &edge, double remaining_section);
  void remove_material(const std::string &name, double remaining_fraction);
  void disconnect(const std::string &chain);
  TrimResult trim(double speed, double altitude);
  TrimResult trim_fixed_throttle(double true_airspeed, double altitude,
                                 double throttle, double prop_rpm_ratio = 1);
  void reset_trim(const TrimResult &trim, double speed, double altitude);
  std::vector<double> node_bending, node_torsion;

private:
  TrimResult trim_condition(double speed, double altitude, bool fixed_throttle,
                            double throttle, double prop_rpm_ratio);
  Wrench evaluate_forces(const Body &, bool store, bool midpoint_controls);
  Wrench evaluate_airframe(const Body &, double wake, bool store,
                           bool midpoint_controls = false);
  Wrench detached_load(DetachedBody &, const Body &, bool store,
                       bool midpoint_controls = false);
  void update_engine(const PilotIntent &);
  void assemble_structural_loads(const Wrench &, Vec rates, double spin);
  void update_mass_and_detach();
};
} // namespace flight
