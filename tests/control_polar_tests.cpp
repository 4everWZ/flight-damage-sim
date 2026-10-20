#include "flight/core.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace flight;
namespace {
void require(bool ok,const char *m) { if(!ok) throw std::runtime_error(m); }
CurvePoint law(double a,double d) {
  return {a,.2+2*a+.7*d+.3*a*d,.06+.02*a+.01*d+.02*a*d,.03-.4*a+.25*d+.1*a*d};
}
ControlPolar grid() {
  ControlPolar p; p.name="synthetic";
  for(double d:{-.4,0.,.4}) {
    ControlPolarSlice row; row.delta=d;
    for(double a:{-.5,0.,.5}) row.points.push_back(law(a,d));
    p.slices.push_back(row);
  }
  return p;
}
Definition fixture(Definition d,bool bind=true) {
  d.engine.prop_fraction=0; d.engine.running=false; d.engine.engine_friction=0;
  d.body_drag_area=0; d.control_polars={grid()};
  if(bind) for(auto &c:d.cells) if(c.chain>=0) c.control_polar="synthetic";
  return d;
}
double force_error=0,moment_error=0,split_force=0,split_moment=0,work_error=0;
int analytic_count=0,rejected_count=0;
void analytic(const Definition &base) {
  for(int plane:{0,1}) for(double side:{-1.,1.}) for(bool damaged:{false,true}) {
    auto d=fixture(base); auto c=d.cells[0]; c.node=0; c.chain=0; c.plane=plane;
    c.control_polar="synthetic"; c.control_sign=-1; c.control_gain=83;
    c.position={.5,-2.3,.4}; c.area=.7; c.chord=.9; c.incidence=.04;
    c.reference_frame=Quat::axis({0,0,1},side*.13)*Quat::axis({1,0,0},.08);
    if(damaged) { c.area_fraction=.35; c.lift_efficiency=.6; c.roughness=.2; }
    d.cells={c}; Simulation s(d); s.def.chains[0].deflection=side*.19; s.def.chains[0].hinge=.7;
    s.body.omega={.07,-.06,.03}; s.wind_world={3,-2,.5};
    s.body.attitude=Quat::axis({0,1,0},.3); Vec vb{70,2,3};
    s.body.velocity=s.body.attitude.rotate(vb)+s.wind_world;
    // Independent elementary incidence/reference matrices, not cell_frame.
    auto rx=[](Vec v,double a){return Vec{v.x,std::cos(a)*v.y-std::sin(a)*v.z,std::sin(a)*v.y+std::cos(a)*v.z};};
    auto ry=[](Vec v,double a){return Vec{std::cos(a)*v.x+std::sin(a)*v.z,v.y,-std::sin(a)*v.x+std::cos(a)*v.z};};
    auto rz=[](Vec v,double a){return Vec{std::cos(a)*v.x-std::sin(a)*v.y,std::sin(a)*v.x+std::cos(a)*v.y,v.z};};
    auto basis=[&](Vec v){ return rz(rx(plane==0?ry(v,.04):rz(v,-.04),.08),side*.13); };
    Vec chord=basis({1,0,0}),span=basis(plane==0?Vec{0,1,0}:Vec{0,0,-1}),
        normal=basis(plane==0?Vec{0,0,1}:Vec{0,1,0}),r=c.position-s.mass.cg;
    Vec local=vb+cross(s.body.omega,r); double a=std::atan2(dot(local,normal),dot(local,chord));
    double delta=-side*.19*.7,q=.5*1.225*std::exp(-1000./8500)*dot(local,local);
    auto coeff=law(a,delta); double pg=1/std::sqrt(1-dot(local,local)/(340*340));
    Vec f=unit(cross(span,local))*(q*c.area*c.area_fraction*c.lift_efficiency*coeff.cl*pg)
          -unit(local)*(q*c.area*(c.area_fraction*coeff.cd+.08*c.roughness));
    Vec couple=span*(q*c.area*c.area_fraction*c.chord*coeff.cm*(plane==0?1:-1));
    Vec m=cross(r,f)+couple; auto w=s.evaluate(s.body,true);
    force_error=std::max(force_error,norm(w.force-f)); moment_error=std::max(moment_error,norm(w.moment-m));
    const auto &sample=s.def.cells[0];
    require(std::abs(sample.flow_alpha-a)<1e-12 && std::abs(sample.deflection-delta)<1e-12,"flow and actual signed/hinged angle");
    require(!sample.control_polar_clamped,"inside declared polar domain");
    double drag_work=q*c.area*(c.area_fraction*coeff.cd+.08*c.roughness)*norm(local);
    work_error=std::max(work_error,std::abs(dot(w.force,vb)+dot(w.moment,s.body.omega)-dot(couple,s.body.omega)+drag_work));
    s.def.cells[0].control_gain=-177; auto unchanged=s.evaluate(s.body);
    require(norm(unchanged.force-w.force)==0 && norm(unchanged.moment-w.moment)==0,"legacy gain is not applied twice");
    ++analytic_count;
  }
  require(force_error<1e-8 && moment_error<1e-7 && work_error<1e-7,"independent bilinear wrench/work");
}
Definition single(const Definition &base) {
  auto d=fixture(base); Simulation inventory(d); auto c=d.cells[0];
  c.node=0; c.chain=0; c.control_polar="synthetic"; c.control_sign=1; c.control_gain=.8;
  c.incidence=0; c.reference_frame={}; c.area=c.chord=1; c.position=inventory.mass.cg;
  d.cells={c}; return d;
}
CurvePoint infer(Simulation &s,double a,double delta) {
  s.body.velocity={70*std::cos(a),0,70*std::sin(a)}; s.def.chains[0].deflection=delta;
  s.evaluate(s.body,true); auto &c=s.def.cells[0]; double pg=1/std::sqrt(1-70.*70/(340*340));
  return {c.flow_alpha,dot(c.force,unit(cross(Vec{0,1,0},c.flow)))/(c.q*pg),
                      dot(c.force,-unit(c.flow))/c.q,c.moment.y/c.q};
}
double equal_proxy_cl_difference=0;
void distinguish(const Definition &base) {
  Simulation first(single(base)),second(single(base));
  auto a=infer(first,.1,.2),b=infer(second,.26,0);
  require(std::abs(first.def.cells[0].alpha-second.def.cells[0].alpha)<1e-12,"equal legacy proxy");
  equal_proxy_cl_difference=std::abs(a.cl-b.cl);
  require(equal_proxy_cl_difference>.1 && std::abs(a.cm-b.cm)>.05,"independent delta behavior cannot be a scalar alpha shift");
  for(double alpha:{-.8,-.5,0.,.5,.8}) for(double delta:{-.6,-.4,0.,.4,.6}) {
    auto p=infer(first,alpha,delta),expected=law(clamp(alpha,-.5,.5),clamp(delta,-.4,.4));
    require(std::abs(p.cl-expected.cl)<1e-12 && std::abs(p.cd-expected.cd)<1e-12 && std::abs(p.cm-expected.cm)<1e-12,"knots/interiors and endpoint clamping");
    bool outside=std::abs(alpha)>.5 || std::abs(delta)>.4;
    require(first.def.cells[0].control_polar_clamped==outside,"domain flag reports any clamp");
  }
  auto no_tables=fixture(base,false); no_tables.control_polars.clear();
  Simulation native(no_tables),unbound(fixture(base,false)); native.body.omega=unbound.body.omega={.1,-.04,.03};
  auto x=native.evaluate(native.body),y=unbound.evaluate(unbound.body);
  require(norm(x.force-y.force)==0 && norm(x.moment-y.moment)==0,"unbound tables do not enable the model");
}
void mechanics(const Definition &base) {
  auto d=single(base); d.chains[0].range=.6; d.chains[0].hinge=.7; d.chains[0].lag=.15; d.chains[0].friction=4;
  Simulation s(d); s.density=1.225*std::exp(-1000./8500); PilotIntent p; p.active[Roll]=true; p.direct[Roll]=1;
  for(int i=0;i<240;++i) s.update_controls(p);
  s.evaluate(s.body,true); double angle=s.def.chains[0].deflection;
  require(angle>0 && angle<d.chains[0].limit*.6 && std::abs(s.def.cells[0].deflection-angle*.7)<1e-12,"damaged actual surface reaches polar");
  s.def.chains[0].jammed=true; s.def.chains[0].jam_position=angle;
  auto plus=s,minus=s; p.direct[Roll]=1; plus.update_controls(p); p.direct[Roll]=-1; minus.update_controls(p);
  auto wp=plus.evaluate(plus.body),wm=minus.evaluate(minus.body);
  require(norm(wp.force-wm.force)==0 && norm(wp.moment-wm.moment)==0,"captured jam ignores command sign");
  s.def.chains[0].jammed=false; s.def.chains[0].connected=false;
  plus=s; minus=s;
  for(int i=0;i<480;++i) { p.direct[Roll]=1; plus.update_controls(p); p.direct[Roll]=-1; minus.update_controls(p); }
  wp=plus.evaluate(plus.body); wm=minus.evaluate(minus.body);
  require(norm(wp.force-wm.force)==0 && norm(wp.moment-wm.moment)==0,"disconnect cannot invent authority");
  Simulation flight(fixture(base)); p={}; p.mouse=true; p.aim_world={1,.1,0};
  for(int i=0;i<120;++i) {
    p.active[Roll]=i>=20 && i<60; p.direct[Roll]=-.7;
    double previous=flight.ownership[Roll].command; flight.step(p);
    if(p.active[Roll]) require(flight.ownership[Roll].command==-.7 && flight.ownership[Roll].direct_weight==1,"full per-axis direct ownership");
    if(i==60) require(flight.ownership[Roll].command==previous,"smooth initial handback");
    for(const auto &c:flight.def.cells) if(c.chain>=0)
      require(std::abs(c.deflection-flight.def.chains[c.chain].midpoint_deflection*flight.def.chains[c.chain].hinge*c.control_sign)<1e-12,"polar consumes actual force-stage angle");
    std::vector<double> loads(flight.def.nodes.size());
    for(const auto &e:flight.def.edges) if(!e.failed) { loads[e.b]+=e.bend_load; loads[e.a]-=e.bend_load; }
    for(std::size_t j=1;j<loads.size();++j) require(std::abs(loads[j]-flight.node_bending[j])<1e-6,"new cell forces reach structural graph");
  }
}
void detach(const Definition &base) {
  Simulation s(fixture(base)); s.body.velocity={70,3,2}; s.body.omega={.1,-.06,.08};
  for(auto &c:s.def.cells) c.reference_frame=Quat::axis({1,0,0},c.position.y>0?-.1:.1);
  s.def.chains[0].jammed=true; s.def.chains[0].jam_position=s.def.chains[0].deflection=.12;
  s.def.nodes[1].bend=.03; s.def.nodes[2].twist=-.02;
  auto before=s.evaluate(s.body,true); Vec origin=s.body.position;
  for(auto &e:s.def.edges) if(e.a==0 && e.b==1) e.failed=true;
  std::vector<double> zero(s.def.nodes.size()); s.solve_structure(zero,zero,0);
  require(s.debris.size()==1,"graph-driven separation"); auto &piece=s.debris[0];
  auto parent=s.evaluate(s.body),child=s.evaluate_detached(piece,true);
  split_force=norm(parent.force+child.force-before.force);
  split_moment=norm(parent.moment+cross(s.body.position-origin,parent.force)+child.moment+cross(piece.state.position-origin,child.force)-before.moment);
  require(split_force<1e-7 && split_moment<1e-6,"same 2D force path before and after split");
  for(auto &e:piece.aero_cells) if(e.cell.chain>=0) require(e.cell.control_polar=="synthetic","binding survives detachment");
  auto plus=s,minus=s; PilotIntent p; p.active[Roll]=true; p.direct[Roll]=1; plus.step(p); p.direct[Roll]=-1; minus.step(p);
  require(norm(plus.debris[0].state.velocity-minus.debris[0].state.velocity)==0 &&
          norm(plus.debris[0].state.omega-minus.debris[0].state.omega)==0,"detached surfaces do not regain pilot link");
}
void parser(const std::string &path) {
  std::ifstream file(path); std::string source((std::istreambuf_iterator<char>(file)),{});
  std::filesystem::create_directories("build/control-polar-inputs");
  std::string table="control_polar test -30 -25 -.4 .07 -.1\ncontrol_polar test 30 -25 .6 .08 .1\ncontrol_polar test -30 25 -.2 .09 -.05\ncontrol_polar test 30 25 .8 .1 .2\n";
  auto load=[&](const std::string &records) {
    std::string name="build/control-polar-inputs/probe.airframe";
    { std::ofstream out(name); out<<records<<source; } return Definition::load(name);
  };
  auto d=load("cell_control_polar 34 test\n"+table);
  require(d.cells[34].control_polar=="test" && d.control_polars.size()==1,"forward cell/table references");
  std::istringstream rows(table); std::vector<std::string> reordered; std::string line,shuffled;
  while(std::getline(rows,line)) reordered.push_back(line);
  for(auto p=reordered.rbegin();p!=reordered.rend();++p) shuffled+=*p+'\n';
  auto sorted=load(shuffled+"cell_control_polar 34 test\n");
  require(sorted.control_polars[0].slices.front().delta==rad(-25) &&
          sorted.control_polars[0].slices.front().points.front().alpha==rad(-30),"unordered records form the declared grid");
  // Keep one tiny runnable optional fixture for transport tests; all original
  // production data files remain unchanged.
  { std::ofstream out("build/control-polar-inputs/enabled.airframe"); out<<source<<table<<"cell_control_polar 34 test\n"; }
  for(const auto &bad:std::vector<std::string>{
    "cell_control_polar 99999 test\n"+table,"cell_control_polar 0 test\n"+table,
    "cell_control_polar 34 absent\n"+table,"cell_control_polar 34 test\ncell_control_polar 34 test\n"+table,
    table+"control_polar test 30 25 .8 .1 .2\n",
    "control_polar test -30 0 0 .1 0\ncontrol_polar test 30 0 1 .1 0\n",
    "control_polar test -30 -25 0 .1 0\ncontrol_polar test 30 -25 1 .1 0\ncontrol_polar test -30 25 0 .1 0\n",
    "control_polar test -30 -25 0 .1 0\ncontrol_polar test 30 -25 1 .1 0\ncontrol_polar test -30 25 0 .1 0\ncontrol_polar test 35 25 1 .1 0\n",
    "control_polar test 0 0 0 -1 0\n", "control_polar test 0 0 nan .1 0\n",
    "control_polar test 0 0 0 .1\n", "control_polar test 0 0 1e309 .1 0\n", "cell_control_polar 34\n"+table}) {
    bool rejected=false; try { (void)load(bad); } catch(const std::exception &) { rejected=true; }
    require(rejected,"invalid grid/binding rejected"); ++rejected_count;
  }
}
}
int main(int argc,char **argv) {
  try {
    if(argc!=3) throw std::runtime_error("airframe summary.json required");
    auto d=Definition::load(argv[1]); parser(argv[1]); analytic(d); distinguish(d); mechanics(d); detach(d);
    std::ofstream out(argv[2]); require(bool(out),"summary output");
    out<<std::setprecision(17)<<"{\n\"analytic_cases\":"<<analytic_count<<",\n\"rejected_inputs\":"<<rejected_count
       <<",\n\"force_error_N\":"<<force_error<<",\n\"moment_error_Nm\":"<<moment_error
       <<",\n\"work_error_W\":"<<work_error<<",\n\"same_legacy_alpha_CL_difference\":"<<equal_proxy_cl_difference
       <<",\n\"split_force_error_N\":"<<split_force<<",\n\"split_moment_error_Nm\":"<<split_moment<<"\n}\n";
    std::cout<<"2D polar: analytic interpolation/wrench, independent delta, actual mechanics and split pass\n";
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
