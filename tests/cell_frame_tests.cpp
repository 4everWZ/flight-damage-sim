#include "flight/core.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace flight;
namespace {
void require(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
// Independent elementary matrices, rather than the production frame helper.
Vec rx(Vec v,double a) { return {v.x,std::cos(a)*v.y-std::sin(a)*v.z,std::sin(a)*v.y+std::cos(a)*v.z}; }
Vec ry(Vec v,double a) { return {std::cos(a)*v.x+std::sin(a)*v.z,v.y,-std::sin(a)*v.x+std::cos(a)*v.z}; }
Vec rz(Vec v,double a) { return {std::cos(a)*v.x-std::sin(a)*v.y,std::sin(a)*v.x+std::cos(a)*v.y,v.z}; }
Definition isolated(Definition d,double cd=.07,double cm=.03) {
  d.engine.prop_fraction=0; d.engine.running=false; d.engine.rpm=0; d.body_drag_area=0;
  for(auto &table:d.curves) for(auto &p:table.second) { p.cl=2*p.alpha; p.cd=cd; p.cm=cm; }
  return d; // Explicit synthetic linear law, never historical polar data.
}
double alpha_error=0,force_error=0,moment_error=0,work_error=0;
int matrix_cases=0,invalid_cases=0;
void analytic_cases(const Definition &base) {
  for(int plane:{0,1}) for(double sign:{-1.,1.}) for(bool elastic:{false,true}) {
    auto d=isolated(base); auto c=d.cells.front();
    c.plane=plane; c.chain=0; c.position={.6,-2.1,.4};
    c.incidence=sign*rad(7); c.control_gain=.8; c.control_sign=-1;
    Vec ref{sign*rad(17),rad(11),-rad(13)};
    c.reference_frame=Quat::axis({1,0,0},ref.x)*Quat::axis({0,1,0},ref.y)*Quat::axis({0,0,1},ref.z);
    d.cells={c}; Simulation s(d);
    double bend=elastic?.08:0,twist=elastic?-.06:0;
    s.def.nodes[c.node].bend=bend; s.def.nodes[c.node].twist=twist;
    s.def.chains[0].deflection=.11; s.def.chains[0].hinge=.7;
    s.body.attitude=(Quat::axis({0,0,1},.4)*Quat::axis({0,1,0},-.2)).normalized();
    s.wind_world={4,-2,.3}; Vec vb{65,4,3};
    s.body.velocity=s.body.attitude.rotate(vb)+s.wind_world; s.body.omega={.1,-.06,.08};
    auto frame=[&](Vec v) {
      v=plane==0?ry(v,c.incidence):rz(v,-c.incidence);
      return rx(ry(rx(ry(rz(v,ref.z),ref.y),ref.x),twist),bend);
    };
    Vec chord=frame({1,0,0}),span=frame(plane==0?Vec{0,1,0}:Vec{0,0,-1}),
        normal=frame(plane==0?Vec{0,0,1}:Vec{0,1,0});
    Vec r=c.position-s.mass.cg,local=vb+cross(s.body.omega,r);
    double a=std::atan2(dot(local,normal),dot(local,chord))+.8*(-.11*.7);
    double q=.5*1.225*std::exp(s.body.position.z/8500)*dot(local,local);
    double compression=1/std::sqrt(std::max(1-dot(local,local)/(340*340),.35));
    Vec f=unit(cross(span,local))*(q*c.area*2*a*compression)-unit(local)*(q*c.area*.07);
    Vec m=cross(r,f)+span*(q*c.area*c.chord*.03*(plane==0?1:-1));
    auto w=s.evaluate(s.body,true);
    alpha_error=std::max(alpha_error,std::abs(s.def.cells[0].alpha-a));
    force_error=std::max(force_error,norm(w.force-f)); moment_error=std::max(moment_error,norm(w.moment-m));
    require(norm(s.def.cells[0].flow-local)<1e-12,"point velocity remains in body axes");
    auto before=w;
    s.body.attitude=Quat::axis({1,0,0},.7)*Quat::axis({0,0,1},-.9);
    s.wind_world={-7,3,2}; s.body.velocity=s.body.attitude.rotate(vb)+s.wind_world;
    w=s.evaluate(s.body,true);
    require(norm(w.force-before.force)<1e-8 && norm(w.moment-before.moment)<1e-7,"world covariance");
    for(double cd:{0.,.07}) {
      for(auto &table:s.def.curves) for(auto &p:table.second) { p.cd=cd; p.cm=0; }
      w=s.evaluate(s.body,true);
      work_error=std::max(work_error,std::abs(dot(w.force,vb)+dot(w.moment,s.body.omega)+q*c.area*cd*norm(local)));
    }
    ++matrix_cases;
  }
  require(alpha_error<1e-12 && force_error<1e-8 && moment_error<1e-7 && work_error<1e-7,"analytic geometry/wrench/work");
}
Definition panels(const Definition &base,double incidence=0) {
  auto d=isolated(base,0,0); auto prototype=d.cells.front(); d.cells.clear();
  for(int side:{-1,1}) {
    auto c=prototype; c.id=side+1; c.chain=-1; c.plane=0;
    c.position={0,side*3.,-3*std::tan(rad(6))}; c.incidence=incidence;
    c.reference_frame=Quat::axis({1,0,0},-side*rad(6)); d.cells.push_back(c);
  }
  return d;
}
double dihedral_alpha=0,root_alpha=0,tip_alpha=0;
void symmetry(const Definition &base) {
  Simulation positive(panels(base)),negative(panels(base));
  positive.body.velocity={70,5,0}; negative.body.velocity={70,-5,0};
  auto p=positive.evaluate(positive.body,true),n=negative.evaluate(negative.body,true);
  dihedral_alpha=std::atan2(5*std::sin(rad(6)),70);
  require(std::abs(positive.def.cells[0].alpha+dihedral_alpha)<1e-12 &&
          std::abs(positive.def.cells[1].alpha-dihedral_alpha)<1e-12,"dihedral signs");
  double mxp=p.moment.x-positive.mass.cg.z*p.force.y+positive.mass.cg.y*p.force.z;
  double mxn=n.moment.x-negative.mass.cg.z*n.force.y+negative.mass.cg.y*n.force.z;
  require(mxp<0 && std::abs(mxp+mxn)<1e-9 && std::abs(p.force.y+n.force.y)<1e-9,"mirrored sideslip");
  Simulation incidence(panels(base,rad(2.1))); incidence.body.velocity={70,0,0};
  incidence.evaluate(incidence.body,true);
  for(const auto &c:incidence.def.cells) require(std::abs(c.alpha-rad(2.1))<1e-12,"incidence after reference frame");
  // Source-informed endpoint angles only: no historical polar, planform or
  // stall calibration is inferred from these two independent station samples.
  auto endpoints=panels(base);
  endpoints.cells[0].incidence=rad(2.1); endpoints.cells[1].incidence=0;
  Simulation stations(endpoints);
  stations.body.velocity={70*std::cos(rad(14)),0,70*std::sin(rad(14))};
  stations.evaluate(stations.body,true);
  root_alpha=stations.def.cells[0].alpha; tip_alpha=stations.def.cells[1].alpha;
  double projected=std::atan2(std::sin(rad(14))*std::cos(rad(6)),std::cos(rad(14)));
  require(std::abs(root_alpha-projected-rad(2.1))<1e-12 && std::abs(tip_alpha-projected)<1e-12,
          "root/tip incidence and dihedral composition");
  require(stations.def.cells[0].stall==1 && stations.def.cells[1].stall==0,
          "declared fixed stall diagnostic uses oriented local angle");
}
void parser(const std::string &path) {
  std::ifstream file(path); std::string original((std::istreambuf_iterator<char>(file)),{});
  auto base=Definition::load(path); int id=base.cells.front().id;
  std::filesystem::create_directories("build/cell-basis-inputs");
  auto load=[&](const std::string &record) {
    std::string name="build/cell-basis-inputs/probe.airframe";
    { std::ofstream out(name); out<<record<<'\n'<<original; }
    return Definition::load(name);
  };
  for(const auto &values:{"1e300 1e300 0 0","1e-300 1e-300 0 0","-1 -1 0 0"}) {
    auto d=load("cell_basis "+std::to_string(id)+" "+values);
    require(norm(d.cells.front().reference_frame.rotate({0,1,0})-Vec{0,0,1})<1e-12,"forward reference/normalization");
  }
  for(const auto &record:std::vector<std::string>{"cell_basis "+std::to_string(id)+" 0 0 0 0",
                          "cell_basis 999999 1 0 0 0",
                          "cell_basis "+std::to_string(id)+" 1 0 0 0\ncell_basis "+std::to_string(id)+" 1 0 0 0",
                          "cell_basis "+std::to_string(id)+" nan 0 0 0",
                          "cell_basis "+std::to_string(id)+" 1e309 0 0 0",
                          "cell_basis "+std::to_string(id)+" 1 0"}) {
    bool rejected=false; try { (void)load(record); } catch(const std::exception &) { rejected=true; }
    require(rejected,"invalid cell_basis must fail"); ++invalid_cases;
  }
  Simulation old(base); old.body.omega={.1,.2,-.1}; auto ref=old.evaluate(old.body);
  for(const auto &values:{"1 0 0 0","-1 0 0 0"}) {
    Simulation s(load("cell_basis "+std::to_string(id)+" "+values)); s.body=old.body;
    auto w=s.evaluate(s.body);
    require(norm(w.force-ref.force)==0 && norm(w.moment-ref.moment)==0,"exact identity compatibility");
  }
}
double split_force_error=0,split_moment_error=0;
void detach(const Definition &base) {
  auto d=isolated(base,0,0);
  for(auto &c:d.cells) c.reference_frame=Quat::axis({1,0,0},c.position.y>0?-.1:.1)*Quat::axis({0,0,1},.04);
  Simulation s(d); s.body.velocity={70,3,2}; s.body.omega={.1,-.06,.08};
  s.def.nodes[1].bend=.03; s.def.nodes[2].twist=-.02; s.def.chains[0].deflection=.12;
  auto before=s.evaluate(s.body,true); Vec origin=s.body.position; auto old_cells=s.def.cells;
  for(auto &e:s.def.edges) if(e.a==0 && e.b==1) e.failed=true;
  std::vector<double> zero(d.nodes.size()); s.solve_structure(zero,zero,0);
  require(s.debris.size()==1,"oriented graph detachment"); auto &piece=s.debris[0];
  auto parent=s.evaluate(s.body,true),child=s.evaluate_detached(piece,true);
  Vec f=parent.force+child.force;
  Vec m=parent.moment+cross(s.body.position-origin,parent.force)+child.moment+cross(piece.state.position-origin,child.force);
  split_force_error=norm(f-before.force); split_moment_error=norm(m-before.moment);
  require(split_force_error<1e-7 && split_moment_error<1e-6,"oriented wrench partition");
  for(const auto &element:piece.aero_cells) {
    const auto &prior=*std::find_if(old_cells.begin(),old_cells.end(),[&](const auto &c){return c.id==element.cell.id;});
    require(std::abs(element.cell.alpha-prior.alpha)<1e-12,"detached angle at birth");
  }
  for(auto &node:s.def.nodes) { node.bend=.5; node.twist=.4; }
  for(auto &c:s.def.cells) c.reference_frame=Quat::axis({0,1,0},.8);
  auto frozen=s.evaluate_detached(piece,true);
  require(norm(frozen.force-child.force)==0 && norm(frozen.moment-child.moment)==0,"detached frame frozen once");
  Vec vb=piece.state.attitude.conjugate().rotate(piece.state.velocity);
  require(std::abs(dot(child.force,vb)+dot(child.moment,piece.state.omega))<1e-7,"detached lift work");
}
} // namespace
int main(int argc,char **argv) {
  try {
    if(argc!=3) throw std::runtime_error("airframe and summary.json required");
    auto base=Definition::load(argv[1]); parser(argv[1]); analytic_cases(base); symmetry(base); detach(base);
    std::filesystem::create_directories(std::filesystem::path(argv[2]).parent_path());
    std::ofstream out(argv[2]); require(bool(out),"cannot write summary");
    out<<std::setprecision(17)<<"{\n  \"matrix_cases\": "<<matrix_cases<<",\n  \"invalid_inputs_rejected\": "<<invalid_cases
       <<",\n  \"max_alpha_error_rad\": "<<alpha_error<<",\n  \"max_force_error_N\": "<<force_error
       <<",\n  \"max_moment_error_Nm\": "<<moment_error<<",\n  \"max_work_error_W\": "<<work_error
       <<",\n  \"dihedral_alpha_rad\": "<<dihedral_alpha<<",\n  \"split_force_error_N\": "<<split_force_error
       <<",\n  \"root_endpoint_alpha_rad\": "<<root_alpha<<",\n  \"tip_endpoint_alpha_rad\": "<<tip_alpha
       <<",\n  \"split_moment_error_Nm\": "<<split_moment_error<<"\n}\n";
    std::cout<<"cell-frame parser, geometry/wrench, symmetry, work and separation pass\n";
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
