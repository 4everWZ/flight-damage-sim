#include "flight/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace flight;
namespace {
void require(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
std::ofstream output(const std::string &prefix,const char *suffix,const char *header) {
  std::ofstream out(prefix+suffix); require(bool(out),"diagnostic output");
  out<<std::setprecision(17)<<header<<'\n'; return out;
}
Definition angles(Definition d) {
  for(auto &c:d.cells) if(c.table=="wing") {
    c.incidence=rad(2.1*(1-std::abs(c.position.y)/5));
    c.reference_frame=Quat::axis({1,0,0},rad(c.position.y>0?-6:6));
  }
  return d;
}
void no_prop(Definition &d) {
  d.engine.prop_fraction=0; d.engine.running=false; d.engine.engine_friction=0;
}
Simulation response(Simulation s,PilotIntent p,Axis axis,double command) {
  p.mouse=false; p.active[axis]=true; p.direct[axis]=command;
  s.density=1.225*std::exp(s.body.position.z/8500);
  for(int i=0;i<int(2/s.dt+.5);++i) s.update_controls(p);
  require(s.ownership[axis].direct_weight==1 && s.ownership[axis].command==command,"direct ownership");
  s.evaluate(s.body,true); return s;
}
void flow(Simulation &s,double speed,double alpha) {
  s.body.velocity=s.wind_world+s.body.attitude.rotate({speed*std::cos(alpha),0,speed*std::sin(alpha)});
}
void roll(const Definition &definition,const std::string &prefix) {
  auto out=output(prefix,"-roll.csv","tas,alpha_deg,mode,amplitude,gain_Nm,left_plus_rad,right_plus_rad,left_minus_rad,right_minus_rad,left_delta_Nm,right_delta_Nm,other_delta_Nm");
  for(double speed:{60.,140.}) {
    Simulation entry(angles(definition)); auto trim=entry.trim_fixed_throttle(speed,1000,0);
    require(trim.converged,"roll entry trim");
    for(double angle:{5.,14.,20.,24.}) for(const std::string mode:{"native","rigid_no_prop","no_wing_drag","linear_wing_CL","disconnect","damaged"}) {
      Simulation state=entry; flow(state,speed,rad(angle));
      if(mode=="rigid_no_prop") {
        no_prop(state.def); for(auto &n:state.def.nodes) n.bend=n.twist=0;
      }
      if(mode=="no_wing_drag" || mode=="linear_wing_CL")
        for(auto &table:state.def.curves) if(table.first=="wing") for(auto &p:table.second) {
          if(mode=="no_wing_drag") p.cd=0;
          else p.cl=5.2*p.alpha; // Counterfactual diagnostic only, never installed.
        }
      if(mode=="disconnect") { state.disconnect("left_aileron"); state.disconnect("right_aileron"); }
      if(mode=="damaged") {
        auto &c=state.def.chains[0]; c.range=.6; c.hinge=.7; c.lag=.15; c.friction=4;
        int n=0; for(const auto &c:state.def.cells) if(c.node==2 && n++<8) state.damage_cell(c.id,0,1);
      }
      for(double amplitude:{.001,.05,.2,1.}) {
        auto plus=response(state,trim.intent,Roll,amplitude),minus=response(state,trim.intent,Roll,-amplitude);
        double left=0,right=0,other=0;
        for(std::size_t i=0;i<plus.def.cells.size();++i) {
          auto &p=plus.def.cells[i]; auto &m=minus.def.cells[i];
          double delta=(cross(p.position-plus.mass.cg,p.force)+p.moment-
                        cross(m.position-minus.mass.cg,m.force)-m.moment).x;
          if(p.chain==0) left+=delta; else if(p.chain==1) right+=delta; else other+=delta;
        }
        double difference=plus.aero.moment.x-minus.aero.moment.x;
        require(std::abs(difference-left-right-other)<1e-7,"roll cell reduction");
        if(mode=="disconnect") require(difference==0,"disconnect cannot produce gain");
        out<<speed<<','<<angle<<','<<mode<<','<<amplitude<<','<<difference/(2*amplitude)<<','
          <<plus.def.chains[0].deflection<<','<<plus.def.chains[1].deflection<<','
          <<minus.def.chains[0].deflection<<','<<minus.def.chains[1].deflection<<','<<left<<','<<right<<','<<other<<'\n';
      }
    }
  }
}
void independent_fixtures(Definition definition,const std::string &prefix) {
  no_prop(definition); definition.body_drag_area=0;
  Simulation inventory(definition); auto c=definition.cells.front(); c.node=0; c.incidence=0;
  c.reference_frame={}; c.area=c.chord=1; c.control_gain=.8; c.control_sign=1;
  c.chain=0; c.position=inventory.mass.cg+Vec{0,-4,0};
  auto right=c; right.id=1; right.chain=1; right.position.y+=8; right.control_sign=-1;
  definition.cells={c,right};
  auto pair=output(prefix,"-pair.csv","tas,alpha_deg,amplitude,left_plus_rad,right_plus_rad,plus_moment_Nm,minus_moment_Nm");
  for(double speed:{60.,140.}) for(double angle:{5.,14.,17.,20.,24.}) for(double amplitude:{.001,.05,.2,1.}) {
    Simulation s(definition); s.body.position.z=-1000; flow(s,speed,rad(angle)); PilotIntent p;
    auto plus=response(s,p,Roll,amplitude),minus=response(s,p,Roll,-amplitude);
    pair<<speed<<','<<angle<<','<<amplitude<<','<<plus.def.chains[0].deflection<<','
      <<plus.def.chains[1].deflection<<','<<plus.aero.moment.x<<','<<minus.aero.moment.x<<'\n';
  }
  auto lookup=output(prefix,"-lookup.csv","table,effective_alpha_rad,inferred_CL,inferred_CD,inferred_CM");
  for(const auto &table:definition.curves) {
    c.table=table.first; c.position=inventory.mass.cg; c.control_sign=1;
    definition.cells={c}; std::vector<double> samples;
    for(std::size_t i=0;i<table.second.size();++i) {
      samples.push_back(table.second[i].alpha);
      if(i) samples.push_back((table.second[i-1].alpha+table.second[i].alpha)*.5);
    }
    for(double angle:samples) {
      Simulation s(definition); s.body.position.z=-1000; flow(s,70,angle);
      // Lookup boundary samples include actual control-shifted angles outside
      // the table domain, with the same chain mechanics as ordinary samples.
      for(double command:{-1.,0.,1.}) {
        auto sample=response(s,PilotIntent{},Roll,command); const auto &cell=sample.def.cells[0];
        Vec drag=-unit(cell.flow),lift=unit(cross(Vec{0,1,0},cell.flow));
        double pg=1/std::sqrt(1-70.*70/(340*340));
        lookup<<table.first<<','<<cell.alpha<<','<<dot(cell.force,lift)/(cell.q*pg)<<','
          <<dot(cell.force,drag)/cell.q<<','<<cell.moment.y/cell.q<<'\n';
      }
    }
  }
}
void pitch(const Definition &definition,const std::string &prefix) {
  auto grid=output(prefix,"-trim.csv","tas,converged,alpha_deg,path_deg,pitch_trim,elevator_rad,force_residual_N,moment_residual_Nm");
  auto envelope=output(prefix,"-pitch.csv","tas,alpha_deg,command,elevator_rad,moment_y_Nm,wing_moment_y_Nm,fixed_tail_moment_y_Nm,elevator_moment_y_Nm,max_wing_alpha_deg,min_elevator_alpha_deg,max_elevator_alpha_deg");
  auto flight=output(prefix,"-flights.csv","tas,mode,hz,time_s,actual_tas,body_alpha_deg,elevator_rad,command,trim,headroom_rad,max_wing_alpha_deg,stalled,failed");
  for(double speed:{50.,55.,56.,57.,58.,59.,60.,70.}) {
    Simulation entry(angles(definition)); auto trim=entry.trim_fixed_throttle(speed,1000,0);
    grid<<speed<<','<<trim.converged<<','<<trim.alpha*180/pi<<','<<trim.flight_path*180/pi<<','
      <<trim.intent.trim[Pitch]<<','<<entry.def.chains[2].deflection<<','<<norm(trim.force_residual_N)<<','<<norm(trim.moment_residual_Nm)<<'\n';
    for(double angle=0;angle<=16;angle+=.25) for(double command:{-1.,0.,1.}) {
      Simulation state=entry; flow(state,speed,rad(angle)); auto p=trim.intent; p.trim[Pitch]=0;
      auto sample=response(state,p,Pitch,command);
      double wing=0,fixed=0,elevator=0,maxwing=-1e9,minimum=1e9,maximum=-1e9;
      for(const auto &c:sample.def.cells) {
        double m=(cross(c.position-sample.mass.cg,c.force)+c.moment).y;
        if(c.table=="wing") { wing+=m; maxwing=std::max(maxwing,c.alpha); }
        if(c.table=="tail" && c.chain<0) fixed+=m;
        if(c.chain==2) { elevator+=m; minimum=std::min(minimum,c.alpha); maximum=std::max(maximum,c.alpha); }
      }
      envelope<<speed<<','<<angle<<','<<command<<','<<sample.def.chains[2].deflection<<','<<sample.aero.moment.y<<','
        <<wing<<','<<fixed<<','<<elevator<<','<<maxwing*180/pi<<','<<minimum*180/pi<<','<<maximum*180/pi<<'\n';
    }
    if(speed!=60 && speed!=70) continue;
    require(trim.converged,"dynamic simulated flight entry");
    for(const std::string mode:{"intact","elevator_range_loss","captured_jam","disconnect"}) for(double hz:{120.,240.}) {
      Simulation s(angles(definition),1/hz); auto t=s.trim_fixed_throttle(speed,1000,0); auto p=t.intent;
      for(int i=0;i<int(6*hz);++i) {
        if(i==int(hz)) {
          auto &c=s.def.chains[2];
          if(mode=="elevator_range_loss") { c.range=.6; c.hinge=.7; c.lag=.15; c.friction=4; }
          if(mode=="captured_jam") { c.jammed=true; c.jam_position=c.deflection; }
          if(mode=="disconnect") s.disconnect("elevator");
        }
        p.active[Pitch]=i>=int(2*hz); p.direct[Pitch]=clamp(i/hz-2,0,1);
        s.step(p); Simulation sample=s; sample.evaluate(sample.body,true);
        if(p.active[Pitch]) require(s.ownership[Pitch].direct_weight==1 &&
            s.ownership[Pitch].command==p.direct[Pitch],"dynamic direct pitch ownership");
        Vec vb=s.body.attitude.conjugate().rotate(s.body.velocity);
        double maxwing=-1e9; int stalled=0,failed=0;
        for(auto &c:sample.def.cells) if(c.table=="wing") { maxwing=std::max(maxwing,c.alpha); stalled+=c.stall>0; }
        for(auto &e:s.def.edges) failed+=e.failed;
        const auto &c=s.def.chains[2]; double q=.5*s.density*dot(s.body.velocity,s.body.velocity);
        double limit=c.limit*c.range/(1+q/c.q_limit);
        flight<<speed<<','<<mode<<','<<hz<<','<<(i+1)/hz<<','<<norm(s.body.velocity)<<','<<std::atan2(vb.z,vb.x)*180/pi<<','
          <<c.deflection<<','<<s.ownership[Pitch].command<<','<<p.trim[Pitch]<<','<<limit-c.deflection<<','
          <<maxwing*180/pi<<','<<stalled<<','<<failed<<'\n';
      }
    }
  }
}
}
int main(int argc,char **argv) {
  try {
    if(argc!=3) throw std::runtime_error("airframe output-prefix required");
    auto definition=Definition::load(argv[1]);
    independent_fixtures(definition,argv[2]); roll(definition,argv[2]); pitch(definition,argv[2]);
    std::cout<<"controlled table/roll/pitch diagnostics complete\n";
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
