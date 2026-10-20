#include "flight/core.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
using namespace flight;
namespace {
void require(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
double analytic_gamma_error=0, analytic_alpha_error=0, wind_error=0;
Definition analytic(Definition d) {
  Simulation inventory(d);
  auto c=d.cells.front(); c.node=0; c.chain=-1; c.plane=0; c.incidence=0;
  c.position=inventory.mass.cg; c.area=15; c.reference_frame={};
  d.cells={c}; d.body_drag_area=0;
  d.engine.prop_fraction=0; d.engine.running=false; d.engine.engine_friction=0;
  d.engine.governor=false;
  for(auto &table:d.curves) for(auto &p:table.second) { p.cl=5*p.alpha; p.cd=.04; p.cm=0; }
  return d;
}
void exact_glide(const Definition &base) {
  auto d=analytic(base); Simulation s(d);
  auto t=s.trim_fixed_throttle(70,1000,0);
  require(t.converged && t.intent.throttle==0,"analytic zero-power glide converges");
  double q=.5*1.225*std::exp(-1000./8500)*70*70, W=s.mass.mass*gravity;
  double drag=q*15*.04, gamma=-std::asin(drag/W);
  double pg=1/std::sqrt(1-70.*70/(340*340));
  double alpha=W*std::cos(gamma)/(q*15*5*pg);
  analytic_gamma_error=std::abs(t.flight_path-gamma);
  analytic_alpha_error=std::abs(t.alpha-alpha);
  require(analytic_gamma_error<1e-7 && analytic_alpha_error<1e-7,"independent lift/drag/gravity solution");
  require(std::abs(-dot(s.aero.force,s.body.attitude.conjugate().rotate(s.body.velocity))/70+
                   W*std::sin(t.flight_path))<.003,"glide energy slope from real drag");
  Simulation wind(d); wind.wind_world={9,-3,0};
  auto w=wind.trim_fixed_throttle(70,1000,0);
  require(w.converged,"uniform-wind TAS trim");
  require(std::abs(norm(wind.body.velocity-wind.wind_world)-70)<1e-12,"speed is air-relative");
  require(std::abs(w.flight_path-t.flight_path)<1e-7,"air-relative path invariant under wind");
  wind.reset_trim(w,70,1000);
  for(int i=0;i<120;++i) { s.step(t.intent); wind.step(w.intent); }
  wind_error=norm((wind.body.position-s.body.position)-wind.wind_world);
  require(wind_error<1e-5 && norm(wind.body.velocity-s.body.velocity-wind.wind_world)<1e-5,
          "one-second wind translation through actual 6DoF");
}
int invalid_inputs=0;
void inputs(const Definition &base) {
  for(auto values:std::vector<std::array<double,4>>{
      {10,1000,0,1},{70,-1,0,1},{70,1000,-.1,1},{70,1000,1.1,1},
      {70,1000,0,.4},{70,1000,0,1.2},{std::numeric_limits<double>::quiet_NaN(),1000,0,1},
      {70,std::numeric_limits<double>::infinity(),0,1}}) {
    Simulation s(base); bool rejected=false;
    try { s.trim_fixed_throttle(values[0],values[1],values[2],values[3]); }
    catch(const std::exception &) { rejected=true; }
    require(rejected,"invalid trim condition rejected"); ++invalid_inputs;
  }
}
void mechanics(const Definition &base) {
  for(double throttle:{0.,.2,1.}) {
    Simulation s(base); auto t=s.trim_fixed_throttle(70,1000,throttle);
    require(t.converged && t.intent.throttle==throttle && s.def.engine.throttle_state==throttle,
            "fixed throttle must not become another optimization variable");
    require(std::abs(t.rotor_acceleration)<1e-6 && std::abs(t.propeller_pitch_rate)<1e-8,
            "rotor and governor actually stationary");
    require(throttle==1?t.flight_path>0:t.flight_path<0,"path follows power balance, not prescribed descent");
  }
  Simulation requested(base);
  auto r=requested.trim_fixed_throttle(70,1000,0,.8);
  require(r.converged && std::abs(requested.def.engine.rpm-base.engine.rated_rpm*.8)<1e-8,
          "requested output-shaft RPM is consistent with the live input");
  requested.step(r.intent);
  require(std::abs(requested.def.engine.target_rpm-base.engine.rated_rpm*.8)<1e-8,
          "first tick retains requested shaft setting");
  Simulation impossible(base); impossible.def.engine.pitch_range=0;
  auto bad=impossible.trim_fixed_throttle(70,1000,0);
  require(!bad.converged && std::abs(bad.rotor_acceleration)>1e-6,"unbalanced prescribed-RPM state cannot be called trim");
  Simulation biased(base); biased.def.engine.governor_integral=.4;
  auto b=biased.trim_fixed_throttle(70,1000,0);
  require(!b.converged && std::abs(b.propeller_pitch_rate)>.01,"moving governor is not a steady propeller");
  Simulation jam(base); jam.def.engine.pitch_jammed=true; jam.def.engine.pitch=rad(30);
  auto j=jam.trim_fixed_throttle(70,1000,0);
  require(jam.def.engine.pitch==rad(30) && j.intent.throttle==0,"jam is not bypassed by trim");
  Simulation damaged(base); damaged.weaken("left_root_front",.3);
  auto &chain=damaged.def.chains[0]; chain.range=.6; chain.hinge=.7; chain.lag=.15; chain.friction=4;
  damaged.disconnect("right_aileron");
  int removed=0; for(const auto &c:damaged.def.cells) if(c.node==2 && removed++<8) damaged.damage_cell(c.id,0,1);
  auto before=damaged.def; auto tr=damaged.trim_fixed_throttle(70,1000,0);
  require(std::isfinite(tr.residual),"damaged attempt reports finite residual");
  for(std::size_t i=0;i<before.edges.size();++i)
    require(before.edges[i].remaining_section==damaged.def.edges[i].remaining_section &&
            before.edges[i].plastic_strain==damaged.def.edges[i].plastic_strain &&
            before.edges[i].yield_stress==damaged.def.edges[i].yield_stress,
            "trial equilibria must retain section/material/plastic history");
  require(!damaged.def.chains[1].connected && damaged.def.chains[0].range==.6 &&
          damaged.def.chains[0].lag==.15 && damaged.def.chains[0].friction==4 && damaged.def.chains[0].hinge==.7,
          "trim cannot restore a damaged control chain");
  for(std::size_t i=0;i<before.cells.size();++i)
    require(before.cells[i].area_fraction==damaged.def.cells[i].area_fraction,"trim cannot restore skin");
}
}
int main(int argc,char **argv) {
  try {
    if(argc!=3) throw std::runtime_error("material airframe and summary.json required");
    auto base=Definition::load(argv[1]); exact_glide(base); inputs(base); mechanics(base);
    std::filesystem::create_directories(std::filesystem::path(argv[2]).parent_path());
    std::ofstream out(argv[2]); require(bool(out),"cannot write summary");
    out<<std::setprecision(17)<<"{\n  \"analytic_gamma_error_rad\": "<<analytic_gamma_error
       <<",\n  \"analytic_alpha_error_rad\": "<<analytic_alpha_error
       <<",\n  \"one_second_wind_position_error_m\": "<<wind_error
       <<",\n  \"invalid_inputs_rejected\": "<<invalid_inputs<<"\n}\n";
    std::cout<<"glide trim: analytic force balance, wind, rotor/control and damage preservation pass\n";
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
