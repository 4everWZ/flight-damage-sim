#pragma once
#include "math.hpp"
#include <array>
#include <cstdint>
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
  double throttle = 0;
};
struct Body {
  Vec position{0, 0, -1000}, velocity{70, 0, 0}, omega{};
  Quat attitude{};
};
struct MassPart {
  int node = 0;
  double mass = 0;
  Vec position{}, inertia{};
};
struct MassProperties {
  double mass = 0;
  Vec cg{};
  Mat inertia{};
};
struct CurvePoint {
  double alpha = 0, cl = 0, cd = 0, cm = 0;
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
};
struct ControlChain {
  std::string name;
  Axis axis = Roll;
  double limit = rad(20), slew = rad(90), q_limit = 18000;
  double range = 1, friction = 0, lag = 0, hinge = 1, deflection = 0,
         filtered = 0;
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
  double max_bank = rad(40), heading_gain = .35, path_gain = .6,
         normal_gain = .5;
};
struct InstructorState {
  Vec aim_body{}, desired_rates{};
  double desired_bank = 0, desired_normal_g = 1, flight_path_error = 0;
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
struct Engine {
  Vec position{};
  double max_power = 0, rated_rpm = 0, prop_radius = 0, rotor_inertia = 0,
         ct = 0, cq = 0, advance_loss = 0, altitude_scale = 0, response = 0;
  double acceleration = 0;
  double rpm = 0, throttle_state = 0, power_fraction = 1, prop_fraction = 1,
         thrust = 0, torque = 0, shaft_power = 0;
  bool running = true; // initial implementation: fixed-pitch propeller
};
struct Definition {
  std::vector<AeroCell> cells;
  std::vector<ControlChain> chains;
  std::vector<MassPart> masses;
  std::vector<StructuralNode> nodes;
  std::vector<StructuralEdge> edges;
  std::vector<std::pair<std::string, std::vector<CurvePoint>>> curves;
  Engine engine;
  double body_drag_area = 0, handoff = .25;
  std::array<bool, 3> trim_enabled{true, true, true};
  static Definition load(const std::string &path);
};
void integrate_body(Body &, const MassProperties &, const Wrench &,
                    double elapsed);
struct DetachedBody {
  Body state;
  MassProperties mass;
  std::vector<int> cells, nodes;
};
struct TrimResult {
  PilotIntent intent;
  double alpha = 0, bank = 0, residual = 0;
  Vec force_residual_N{}, moment_residual_Nm{};
  double rotor_acceleration = 0;
  bool converged = false;
  int iterations = 0;
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
  explicit Simulation(Definition d, double timestep = 1. / 120);
  void step(const PilotIntent &input);
  Wrench evaluate(const Body &b, bool store = false);
  void update_controls(const PilotIntent &input);
  void solve_structure(const std::vector<double> &bending,
                       const std::vector<double> &torsion, double elapsed);
  void damage_cell(int id, double area_fraction, double roughness);
  void weaken(const std::string &edge, double remaining_section);
  void disconnect(const std::string &chain);
  TrimResult trim(double speed, double altitude);
  void reset_trim(const TrimResult &trim, double speed, double altitude);
  std::vector<double> node_bending, node_torsion;

private:
  void update_engine(const PilotIntent &);
  void assemble_structural_loads(const Wrench &);
  void update_mass_and_detach();
};
} // namespace flight
