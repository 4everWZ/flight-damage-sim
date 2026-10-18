#include "flight/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;

// Capability measurements of the EXISTING synthetic fixture. No historical
// airframe is generated. Static component benches do not advance the body,
// rotor or structure, and are not claimed to be trimmed stall/roll trials.
int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::runtime_error("airframe and output prefix required");
    auto d = Definition::load(argv[1]);
    std::string prefix = argv[2];
    Simulation inventory(d);
    double area=0, span=0, chord_area=0, inc_lo=1e10, inc_hi=-1e10;
    for(const auto &c:d.cells) if(c.table=="wing") {
      area+=c.area; span=std::max(span,2*std::abs(c.position.y)+c.span);
      chord_area+=c.chord*c.area;
      inc_lo=std::min(inc_lo,c.incidence); inc_hi=std::max(inc_hi,c.incidence);
    }
    if(area<=0) throw std::runtime_error("this diagnostic needs a named wing table");
    std::ofstream inv(prefix+"-inventory.csv"), controls(prefix+"-controls.csv"),
                  stall(prefix+"-static.csv"), basis(prefix+"-basis.csv");
    if(!inv || !controls || !stall || !basis) throw std::runtime_error("cannot write evidence");
    inv<<std::setprecision(17)
       <<"mass_kg,cg_x_m,wing_area_m2,span_m,area_weighted_chord_m,incidence_min_deg,incidence_max_deg,prop_blades,prop_radius_m,output_shaft_rated_rpm,max_power_W\n"
       <<inventory.mass.mass<<','<<inventory.mass.cg.x<<','<<area<<','<<span<<','
       <<chord_area/area<<','<<inc_lo*180/pi<<','<<inc_hi*180/pi<<','
       <<d.engine.blades<<','<<d.engine.prop_radius<<','<<d.engine.rated_rpm<<','<<d.engine.max_power<<'\n';
    constexpr double altitude=1000;
    double rho=1.225*std::exp(-altitude/8500);
    controls<<std::setprecision(17)
            <<"tas_mps,altitude_m,rho_kg_m3,q_Pa,normalized_request,left_chain_rad,right_chain_rad,left_limit_rad,left_q_limit_Pa\n";
    for(double speed:{40.,70.,100.,140.}) for(double command:{-1.,-.5,.5,1.}) {
      Simulation s(d);
      s.instructor_mode=InstructorMode::Off;
      s.body.position={0,0,-altitude}; s.body.velocity={speed,0,0};
      s.density=rho; // Same density as step(), held for a component bench.
      PilotIntent p; p.active={true,true,true}; p.direct={command,0,0};
      for(int i=0;i<240;++i) s.update_controls(p);
      controls<<speed<<','<<altitude<<','<<rho<<','<<.5*rho*speed*speed<<','
              <<command<<','<<s.def.chains[0].deflection<<','<<s.def.chains[1].deflection<<','
              <<s.def.chains[0].limit<<','<<s.def.chains[0].q_limit<<'\n';
    }
    auto isolated=d;
    isolated.engine.prop_fraction=0; // Explicit prop-absent ablation, NOT throttle-closed gliding.
    isolated.engine.running=false; isolated.engine.rpm=0;
    Simulation s(isolated);
    s.body.position={0,0,-altitude};
    stall<<std::setprecision(17)
         <<"body_alpha_deg,tas_mps,rho_kg_m3,q_Pa,airframe_CL,wing_CL,inner_stalled,outer_stalled,wing_effective_alpha_min_deg,wing_effective_alpha_max_deg,prop_present,structure_solved\n";
    constexpr double speed=70;
    double q=.5*rho*speed*speed;
    for(int i=-16;i<=80;++i) {
      double a=rad(i*.25);
      Vec flow{std::cos(a),0,std::sin(a)}, lift{std::sin(a),0,-std::cos(a)};
      s.body.velocity=flow*speed;
      s.evaluate(s.body,true);
      double wing_lift=0, lo=1e10, hi=-1e10; int inner=0,outer=0;
      for(const auto &c:s.def.cells) if(c.table=="wing") {
        wing_lift+=dot(c.force,lift); lo=std::min(lo,c.alpha); hi=std::max(hi,c.alpha);
        if(c.stall>0) (std::abs(c.position.y)<2.5?inner:outer)++;
      }
      stall<<i*.25<<','<<speed<<','<<rho<<','<<q<<','
           <<dot(s.aero.force,lift)/(q*area)<<','<<wing_lift/(q*area)<<','<<inner<<','<<outer<<','
           <<lo*180/pi<<','<<hi*180/pi<<",0,0\n";
    }
    // Independent geometry counterexample: two prescribed flat panels at 6 deg
    // dihedral. Positions alone can be imported, but no definition field can
    // express their span/chord frames. Zero incidence isolates that missing DOF.
    // This is a capability witness, NOT a reconstruction of the NACA wing.
    auto panels=isolated;
    panels.cells.clear();
    for(int side:{-1,1}) {
      auto found=std::find_if(d.cells.begin(),d.cells.end(),[&](const auto &c){
        return c.table=="wing" && side*c.position.y>3.;
      });
      if(found==d.cells.end()) throw std::runtime_error("two-panel fixture unavailable");
      auto c=*found;
      c.incidence=0; c.chain=-1; c.position.z=-std::abs(c.position.y)*std::tan(rad(6));
      panels.cells.push_back(c);
    }
    Simulation b(panels); b.body.velocity={70,5,0}; b.body.position.z=-altitude;
    b.evaluate(b.body,true);
    basis<<std::setprecision(17)<<"side,y_m,z_m,prescribed_dihedral_deg,expected_geometric_alpha_rad,core_alpha_rad\n";
    for(const auto &c:b.def.cells) {
      int side=c.position.y>0?1:-1;
      Quat expected=Quat::axis({1,0,0},-side*rad(6));
      Vec normal=expected.rotate({0,0,1}), chord=expected.rotate({1,0,0});
      double alpha=std::atan2(dot(b.body.velocity,normal),dot(b.body.velocity,chord));
      basis<<side<<','<<c.position.y<<','<<c.position.z<<",6,"<<alpha<<','<<c.alpha<<'\n';
    }
  } catch(const std::exception &e) {
    std::cerr<<e.what()<<'\n'; return 1;
  }
}
