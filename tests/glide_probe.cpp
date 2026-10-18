#include "flight/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace flight;
namespace {
void require(bool ok,const char *m) { if(!ok) throw std::runtime_error(m); }
struct Region { int stalled=0,count=0; double alpha=-1e9,lift=0; };
std::array<Region,2> regions(const Simulation &s) {
  std::array<Region,2> result;
  for(const auto &c:s.def.cells) if(c.table=="wing" && c.attached && c.area_fraction>0) {
    auto &r=result[std::abs(c.position.y)<2.5?0:1];
    r.stalled+=c.stall>0; ++r.count; r.alpha=std::max(r.alpha,c.alpha); r.lift-=c.force.z;
  }
  return result;
}
double balance(const Simulation &s,const std::vector<StructuralEdge> &before) {
  std::vector<double> bend(s.def.nodes.size()),twist(bend.size());
  for(std::size_t i=0;i<before.size();++i) if(!before[i].failed) {
    const auto &e=s.def.edges[i];
    bend[e.b]+=e.bend_load; bend[e.a]-=e.bend_load;
    twist[e.b]+=e.twist_load; twist[e.a]-=e.twist_load;
  }
  double error=0;
  for(std::size_t i=1;i<bend.size();++i)
    error=std::max({error,std::abs(bend[i]-s.node_bending[i]),std::abs(twist[i]-s.node_torsion[i])});
  return error;
}
struct ControlSample { double moment=0,left=0,right=0; };
ControlSample response(const Simulation &state,PilotIntent input,double command,bool disconnected) {
  Simulation s=state;
  if(disconnected) for(auto &c:s.def.chains) if(c.axis==Roll) c.connected=false;
  s.density=1.225*std::exp(s.body.position.z/8500);
  input.mouse=false; input.active[Roll]=true; input.direct[Roll]=command;
  for(int i=0;i<int(2/s.dt+.5);++i) s.update_controls(input);
  auto loads=s.evaluate(s.body,true);
  return {loads.moment.x,s.def.chains[0].deflection,s.def.chains[1].deflection};
}
void authority(std::ostream &out,const Simulation &state,PilotIntent input,double speed,
               int mode,double hz,double time,const char *label) {
  auto zero=response(state,input,0,false);
  auto lostp=response(state,input,1,true),lostm=response(state,input,-1,true);
  require(lostp.moment==lostm.moment,"disconnected roll inputs cannot create authority");
  auto r=regions(state);
  for(double command:{.2,1.}) {
    auto plus=response(state,input,command,false),minus=response(state,input,-command,false);
    out<<speed<<','<<mode<<','<<hz<<','<<time<<','<<label<<','<<command<<','
       <<norm(state.body.velocity-state.wind_world)<<','<<r[0].stalled<<','<<r[1].stalled<<','
       <<zero.moment<<','<<plus.moment<<','<<minus.moment<<','<<(plus.moment-minus.moment)/(2*command)<<','
       <<plus.left<<','<<plus.right<<','<<minus.left<<','<<minus.right<<','<<lostp.moment-lostm.moment<<'\n';
  }
}
}
int main(int argc,char **argv) {
  try {
    if(argc!=3 && argc!=4) throw std::runtime_error("airframe output-prefix [grid-only] required");
    auto definition=Definition::load(argv[1]); std::string prefix=argv[2];
    std::cout<<std::setprecision(17)
      <<"tas_mps,throttle,converged,path_rad,alpha_parameter_rad,roll_trim,pitch_trim,yaw_trim,force_residual_N,moment_residual_Nm,rotor_accel_rad_s2,prop_pitch_rate_rad_s,prop_rpm,prop_pitch_rad,shaft_power_W\n";
    for(double speed:{40.,50.,55.,60.,70.,90.,140.}) for(double throttle:{0.,.2,1.}) {
      Simulation s(definition); auto t=s.trim_fixed_throttle(speed,1000,throttle);
      require(t.intent.throttle==throttle,"fixed-throttle grid");
      std::cout<<speed<<','<<throttle<<','<<t.converged<<','<<t.flight_path<<','<<t.alpha<<','
        <<t.intent.trim[0]<<','<<t.intent.trim[1]<<','<<t.intent.trim[2]<<','
        <<norm(t.force_residual_N)<<','<<norm(t.moment_residual_Nm)<<','<<t.rotor_acceleration<<','
        <<t.propeller_pitch_rate<<','<<s.def.engine.rpm<<','<<s.def.engine.pitch<<','<<s.def.engine.shaft_power<<'\n';
    }
    bool sustained=argc==4 && std::string(argv[3])=="sustained";
    if(argc==4 && !sustained) return 0;
    std::ofstream trace(prefix+"-traces.csv"),control(prefix+"-authority.csv");
    require(bool(trace)&&bool(control),"cannot write glide evidence");
    trace<<std::setprecision(17)<<"entry_tas,case,hz,time_s,altitude_m,tas_mps,air_path_rad,body_alpha_rad,throttle,prop_rpm,prop_pitch_rad,shaft_power_W,pitch_request,pitch_weight,elevator_rad,aero_body_z_g,root_stalled,outer_stalled,root_effective_alpha_max_rad,outer_effective_alpha_max_rad,root_lift_body_z_N,outer_lift_body_z_N,front_bend_Nm,rear_bend_Nm,skin_bend_Nm,front_strain,rear_strain,failed_edges,debris,balance_error_Nm\n";
    control<<std::setprecision(17)<<"entry_tas,case,hz,time_s,snapshot,request_amplitude,tas_mps,root_stalled,outer_stalled,zero_roll_moment_Nm,plus_roll_moment_Nm,minus_roll_moment_Nm,secant_roll_gain_Nm,plus_left_chain_rad,plus_right_chain_rad,minus_left_chain_rad,minus_right_chain_rad,disconnected_moment_difference_Nm\n";
    if(!sustained) {
      std::ofstream sweep(prefix+"-static.csv"); require(bool(sweep),"static sweep output");
      sweep<<std::setprecision(17)<<"tas_mps,body_flow_alpha_deg,root_stalled,outer_stalled,root_alpha_max_rad,outer_alpha_max_rad,root_lift_N,outer_lift_N,roll_secant_gain_Nm,plus_left_rad,plus_right_rad,disconnected_difference_Nm\n";
      for(double speed:{60.,140.}) {
        Simulation entry(definition); auto trim=entry.trim_fixed_throttle(speed,1000,0);
        require(trim.converged,"static sweep starts with real shaft/trim state");
        for(double angle=0;angle<=24;angle+=.5) {
          Simulation sample=entry;
          // Prescribed FLOW on an isolated copy, not a flight maneuver or
          // attitude controller. Rotor/elastic state is held at entry values.
          sample.body.velocity=sample.wind_world+sample.body.attitude.rotate(
              {speed*std::cos(rad(angle)),0,speed*std::sin(rad(angle))});
          sample.evaluate(sample.body,true); auto r=regions(sample);
          auto plus=response(sample,trim.intent,1,false),minus=response(sample,trim.intent,-1,false);
          auto dp=response(sample,trim.intent,1,true),dm=response(sample,trim.intent,-1,true);
          require(dp.moment==dm.moment,"static disconnected authority");
          sweep<<speed<<','<<angle<<','<<r[0].stalled<<','<<r[1].stalled<<','<<r[0].alpha<<','<<r[1].alpha<<','
            <<r[0].lift<<','<<r[1].lift<<','<<(plus.moment-minus.moment)/2<<','<<plus.left<<','<<plus.right<<','<<dp.moment-dm.moment<<'\n';
        }
      }
    }
    // case 0 coast; 1 pull; 2 pull + linkage/aero loss; 3 adds weak front-root section.
    auto speeds=sustained?std::vector<double>{60,140}:std::vector<double>{70,140};
    auto modes=sustained?std::vector<int>{1,3}:std::vector<int>{0,1,2,3};
    double duration=sustained?16:6, recovery=sustained?12:4;
    for(double speed:speeds) for(int mode:modes) for(double hz:{120.,240.}) {
      Simulation s(definition,1/hz); auto entry=s.trim_fixed_throttle(speed,1000,0);
      require(entry.converged,"flight requires a real fixed-power entry equilibrium");
      PilotIntent p=entry.intent; bool onset=false;
      for(int i=0;i<int(duration*hz);++i) {
        if(mode>=2 && i==int(hz)) {
          auto &c=s.def.chains[0]; c.range=.6; c.hinge=.7; c.lag=.15; c.friction=4;
          int n=0; for(const auto &cell:s.def.cells) if(cell.node==2 && n++<8) s.damage_cell(cell.id,0,1);
          if(mode==3) s.weaken("left_root_front",.3);
        }
        double before=i/hz;
        p.active[Pitch]=mode>0 && before>=2 && before<duration-.5;
        p.direct[Pitch]=before<recovery?clamp(before-2,0,1):-1;
        auto old_edges=s.def.edges; s.step(p); double time=(i+1)/hz;
        require(p.throttle==0 && s.def.engine.throttle_state==0,"glide power stays fixed");
        if(p.active[Pitch]) require(s.ownership[Pitch].command==p.direct[Pitch] && s.ownership[Pitch].direct_weight==1,"real direct pitch authority");
        double err=balance(s,old_edges); require(err<1e-6,"actual glide structural equilibrium");
        // Fresh endpoint diagnostics on a COPY: leave actual midpoint cache,
        // rotor seed and force-feedback timing untouched in the flying core.
        Simulation sample=s; sample.evaluate(sample.body,true); auto r=regions(sample);
        Vec air=s.body.velocity-s.wind_world,body=s.body.attitude.conjugate().rotate(air);
        int failed=0; for(const auto &e:s.def.edges) failed+=e.failed;
        auto carried=[&](int id){ return old_edges[id].failed?0:s.def.edges[id].bend_load; };
        trace<<speed<<','<<mode<<','<<hz<<','<<time<<','<<-s.body.position.z<<','<<norm(air)<<','
          <<std::atan2(-air.z,std::hypot(air.x,air.y))<<','<<std::atan2(body.z,body.x)<<','<<p.throttle<<','
          <<s.def.engine.rpm<<','<<s.def.engine.pitch<<','<<s.def.engine.shaft_power<<','
          <<s.ownership[Pitch].command<<','<<s.ownership[Pitch].direct_weight<<','<<s.def.chains[2].deflection<<','
          <<s.load_factor<<','<<r[0].stalled<<','<<r[1].stalled<<','<<r[0].alpha<<','<<r[1].alpha<<','
          <<r[0].lift<<','<<r[1].lift<<','<<carried(0)<<','<<carried(1)<<','<<carried(2)<<','
          <<s.def.edges[0].plastic_strain<<','<<s.def.edges[1].plastic_strain<<','<<failed<<','<<s.debris.size()<<','<<err<<'\n';
        if(i+1==int(1.9*hz+.5)) authority(control,sample,p,speed,mode,hz,time,"before_pull");
        if(r[0].stalled && !onset) { onset=true; authority(control,sample,p,speed,mode,hz,time,"inner_first_flag"); }
        if(i+1==int((duration-.2)*hz+.5)) authority(control,sample,p,speed,mode,hz,time,"late");
        if(!s.debris.empty()) break;
      }
    }
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
