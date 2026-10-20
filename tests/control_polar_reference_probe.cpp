#include "flight/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace flight;
int main(int argc,char **argv) {
  try {
    if(argc!=5) throw std::runtime_error("airframe mach queries.csv output.csv required");
    auto d=Definition::load(argv[1]); double mach=std::stod(argv[2]);
    if(!std::isfinite(mach) || mach<=0) throw std::runtime_error("positive finite Mach required");
    auto found=std::find_if(d.cells.begin(),d.cells.end(),[](const auto &c){return c.id==34;});
    if(found==d.cells.end()) throw std::runtime_error("normalized replay needs fixture cell 34");
    auto cell=*found;
    Simulation inventory(d);
    // Normalized section force-kernel replay, not a vehicle or wind-tunnel
    // geometry model: apply the section load at its declared reference point.
    cell.node=0; cell.chain=0; cell.plane=0; cell.position=inventory.mass.cg;
    cell.area=cell.chord=cell.span=1; cell.incidence=0; cell.reference_frame={};
    cell.control_sign=1; cell.control_gain=0;
    d.cells={cell}; d.body_drag_area=0; d.engine.prop_fraction=0;
    d.engine.running=false; d.engine.engine_friction=0;
    std::ifstream query(argv[3]); std::ofstream out(argv[4]);
    if(!query || !out) throw std::runtime_error("reference file input/output");
    std::string line; std::getline(query,line);
    out<<std::setprecision(17)<<"alpha_deg,delta_deg,flow_alpha_rad,delta_eff_rad,command,cl,cd,cm,clamped\n";
    while(std::getline(query,line)) {
      std::replace(line.begin(),line.end(),',',' '); std::istringstream record(line); double a,delta;
      if(!(record>>a>>delta) || !std::isfinite(a) || !std::isfinite(delta)) throw std::runtime_error("invalid reference query");
      Simulation s(d); double speed=mach*340;
      s.body.velocity={speed*std::cos(rad(a)),0,speed*std::sin(rad(a))};
      s.density=1.225*std::exp(s.body.position.z/8500);
      double q=.5*s.density*speed*speed;
      double command=rad(delta)*(1+q/s.def.chains[0].q_limit)/s.def.chains[0].limit;
      if(std::abs(command)>1) throw std::runtime_error("query exceeds actual control travel");
      PilotIntent p; p.active[Roll]=true; p.direct[Roll]=command;
      for(int i=0;i<240;++i) s.update_controls(p);
      s.evaluate(s.body,true); const auto &c=s.def.cells[0];
      if(std::abs(c.deflection-rad(delta))>1e-12) throw std::runtime_error("actual angle not reached");
      Vec lift=unit(cross(Vec{0,1,0},c.flow)),drag=-unit(c.flow);
      out<<a<<','<<delta<<','<<c.flow_alpha<<','<<c.deflection<<','<<command<<','
         <<dot(c.force,lift)/c.q<<','<<dot(c.force,drag)/c.q<<','<<c.moment.y/c.q<<','<<c.control_polar_clamped<<'\n';
    }
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
