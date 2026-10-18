#include "flight/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace flight;
namespace {
void require(bool value,const char *message) {
  if(!value) throw std::runtime_error(message);
}
struct Audit {
  double balance=0,constitutive=0,plastic=0;
  void check(const Simulation &s,const Definition &before) {
    std::vector<double> bending(s.def.nodes.size()),torsion(bending.size());
    for(std::size_t i=0;i<before.edges.size();++i) {
      const auto &old=before.edges[i]; const auto &e=s.def.edges[i];
      if(old.failed || !before.nodes[old.a].attached || !before.nodes[old.b].attached) continue;
      // Stored loads belong to the graph solve BEFORE this step's plastic
      // increment/failure. Keep that interval's active-edge/stiffness mask.
      bending[e.b]+=e.bend_load; bending[e.a]-=e.bend_load;
      torsion[e.b]+=e.twist_load; torsion[e.a]-=e.twist_load;
      constitutive=std::max({constitutive,
          std::abs(e.bend_load-old.bend_k*old.stiffness()*(s.def.nodes[e.b].bend-s.def.nodes[e.a].bend)),
          std::abs(e.twist_load-old.twist_k*old.stiffness()*(s.def.nodes[e.b].twist-s.def.nodes[e.a].twist))});
      double zscale=std::pow(old.remaining_section,1.5);
      double stress=std::hypot(e.bend_load/(old.section_modulus*zscale),
                               e.twist_load/(old.torsion_modulus*zscale));
      double increment=s.dt*old.plastic_rate*std::pow(std::max(stress/old.yield_stress-1,0.),2);
      plastic=std::max(plastic,std::abs(e.plastic_strain-old.plastic_strain-increment));
      require(e.failed==(e.plastic_strain>=e.failure_strain),
              "new failure must follow the existing strain law");
    }
    for(std::size_t i=1;i<bending.size();++i)
      if(before.nodes[i].attached)
        balance=std::max({balance,std::abs(bending[i]-s.node_bending[i]),
                          std::abs(torsion[i]-s.node_torsion[i])});
    std::vector<bool> connected(s.def.nodes.size()); connected[0]=true;
    for(std::size_t pass=0;pass<connected.size();++pass)
      for(const auto &e:s.def.edges)
        if(!e.failed && (connected[e.a] || connected[e.b])) connected[e.a]=connected[e.b]=true;
    for(std::size_t i=0;i<connected.size();++i)
      require(connected[i]==s.def.nodes[i].attached,
              "attachment must follow surviving structural connectivity");
    require(balance<1e-6 && constitutive<1e-6 && plastic<1e-12,
            "real flight graph must satisfy equilibrium and declared material equations");
  }
};
}
int main(int argc,char **argv) {
  try {
    if(argc!=3) throw std::runtime_error("airframe and trace.csv required");
    auto definition=Definition::load(argv[1]);
    std::ofstream trace(argv[2]);
    if(!trace) throw std::runtime_error("cannot write trace");
    trace<<std::setprecision(17)
         <<"entry_speed,maneuver,weakened,hz,time_end_s,air_speed,aero_body_z_g,fx,fy,fz,roll_command,roll_owner,pitch_command,pitch_owner,elevator_rad,stalled_cells,max_effective_alpha_rad,front_bend_Nm,rear_bend_Nm,skin_bend_Nm,front_twist_Nm,front_util,rear_util,front_strain,rear_strain,failed_edges,debris,new_failures\n";
    std::cout<<std::setprecision(15)
             <<"entry_speed,maneuver,weakened,hz,entry_converged,entry_force_residual_N,entry_moment_residual_Nm,end_s,peak_g_before_2s,peak_g_2_to_6s,peak_g_all,peak_stalled_cells,stall_s,full_pitch_s,elevator_at_stop_s,max_elevator_rad,root_front_share_2s,first_failure_s,first_failed_edge,separation_s,max_utilization,balance_error_Nm,constitutive_error_Nm,plastic_error,release_jump\n";
    for(double entry:{70.,140.})
      for(bool pull:{false,true})
        for(bool weak:{false,true})
          for(double hz:{120.,240.}) {
            Simulation s(definition,1/hz);
            auto tr=s.trim(entry,1000);
            if(entry==70) require(tr.converged,"low-energy entry requires actual equilibrium");
            auto p=tr.intent; p.mouse=true; p.aim_world={1,.25,0};
            Audit audit;
            double early=-1e100,loaded=std::numeric_limits<double>::quiet_NaN(),peak=-1e100,
                   stall_s=0,full_s=0,stop_s=0,elevator=0,share=std::numeric_limits<double>::quiet_NaN(),
                   failure=-1,separation=-1,utilization=0,jump=0,end=0;
            int max_stalled=0; std::string failed_name="none";
            for(int i=0;i<int(8*hz);++i) {
              if(i==int(hz)) {
                auto &left=s.def.chains[0];
                left.lag=.15; left.friction=4; left.range=.6; left.hinge=.7;
                int removed=0;
                for(const auto &cell:s.def.cells)
                  if(cell.node==2 && removed++<8) s.damage_cell(cell.id,0,1);
                if(weak) s.weaken("left_root_front",.3);
              }
              p.active[Roll]=i>=int(hz) && i<int(1.4*hz);
              p.direct[Roll]=i<int(1.2*hz)?1:-1;
              p.active[Pitch]=pull && i>=int(2*hz) && i<int(6*hz);
              p.direct[Pitch]=1;
              auto before=s.def;
              auto old_roll=s.ownership[Roll].command,old_pitch=s.ownership[Pitch].command;
              double rho=1.225*std::exp(s.body.position.z/8500);
              Vec air=s.body.velocity-s.wind_world;
              double q=.5*rho*dot(air,air);
              s.step(p); end=(i+1)/hz;
              audit.check(s,before);
              if(p.active[Roll]) require(s.ownership[Roll].command==p.direct[Roll],"direct roll authority");
              if(p.active[Pitch]) {
                require(s.ownership[Pitch].command==1 && s.ownership[Pitch].direct_weight==1 &&
                            s.ownership[Roll].direct_weight==0,
                        "full pitch request follows completed roll handoff");
                full_s+=s.dt;
                const auto &c=s.def.chains[2];
                double stop=c.limit*c.range/(1+q/c.q_limit);
                if(std::abs(c.deflection-stop)<1e-6) stop_s+=s.dt;
              }
              if(i==int(1.4*hz)) jump=std::max(jump,std::abs(s.ownership[Roll].command-old_roll));
              if(pull && i==int(6*hz)) jump=std::max(jump,std::abs(s.ownership[Pitch].command-old_pitch));
              for(std::size_t j=0;j<before.chains.size();++j) {
                const auto &c=before.chains[j];
                double rate=c.slew/(1+q/c.q_limit)/(1+std::max(c.friction,0.));
                require(std::abs(s.def.chains[j].deflection-c.deflection)<=rate/hz+1e-12,
                        "sustained maneuver must respect actual surface slew");
              }
              peak=std::max(peak,s.load_factor);
              if(i<int(2*hz)) early=std::max(early,s.load_factor);
              else if(i<int(6*hz)) loaded=std::isfinite(loaded)?std::max(loaded,s.load_factor):s.load_factor;
              int stalled=0,failed=0;
              double max_alpha=0;
              for(const auto &c:s.def.cells) if(c.attached && c.area_fraction>0) {
                stalled+=c.stall>0;
                max_alpha=std::max(max_alpha,std::abs(c.alpha));
              }
              if(stalled) stall_s+=s.dt;
              max_stalled=std::max(max_stalled,stalled);
              for(const auto &e:s.def.edges) { failed+=e.failed; utilization=std::max(utilization,e.utilization); }
              std::string events;
              for(std::size_t j=0;j<before.edges.size();++j)
                if(!before.edges[j].failed && s.def.edges[j].failed) {
                  if(!events.empty()) events+=';';
                  events+=s.def.edges[j].name;
                }
              if(failed && failure<0) {
                failure=end;
                for(const auto &e:s.def.edges) if(e.failed) { failed_name=e.name; break; }
              }
              elevator=std::max(elevator,std::abs(s.def.chains[2].deflection));
              const auto &front=s.def.edges[0],&rear=s.def.edges[1],&skin=s.def.edges[2];
              require(front.name=="left_root_front" && rear.name=="left_root_rear" && skin.name=="left_root_skin",
                      "fixture left-root parallel paths");
              if(i+1==int(2*hz)) {
                double total=front.bend_load+rear.bend_load+skin.bend_load;
                require(std::abs(total)>1e-9,"root-share sample needs nonzero bending");
                share=front.bend_load/total;
              }
              Wrench force{s.aero.force+s.propulsion.force,{}};
              trace<<entry<<','<<(pull?"pull":"mouse")<<','<<weak<<','<<hz<<','<<end<<','
                   <<norm(s.body.velocity-s.wind_world)<<','<<s.load_factor<<','
                   <<force.force.x<<','<<force.force.y<<','<<force.force.z<<','
                   <<s.ownership[Roll].command<<','<<s.ownership[Roll].direct_weight<<','
                   <<s.ownership[Pitch].command<<','<<s.ownership[Pitch].direct_weight<<','
                   <<s.def.chains[2].deflection<<','<<stalled<<','<<max_alpha<<','<<front.bend_load<<','<<rear.bend_load<<','
                   <<skin.bend_load<<','<<front.twist_load<<','<<front.utilization<<','<<rear.utilization<<','
                   <<front.plastic_strain<<','<<rear.plastic_strain<<','<<failed<<','<<s.debris.size()<<','<<events<<'\n';
              if(!s.debris.empty()) { separation=end; break; }
              require(std::isfinite(norm(s.body.velocity)),"finite pre-separation trajectory");
            }
            require(jump<1e-13,"handoff request continuity");
            std::cout<<entry<<','<<(pull?"pull":"mouse")<<','<<weak<<','<<hz<<','<<tr.converged<<','
                     <<norm(tr.force_residual_N)<<','<<norm(tr.moment_residual_Nm)<<','<<end<<','
                     <<early<<','<<loaded<<','<<peak<<','<<max_stalled<<','<<stall_s<<','<<full_s<<','
                     <<stop_s<<','<<elevator<<','<<share<<','<<failure<<','<<failed_name<<','<<separation<<','
                     <<utilization<<','<<audit.balance<<','<<audit.constitutive<<','<<audit.plastic<<','<<jump<<'\n';
            std::cout.flush();
          }
    if(!trace) throw std::runtime_error("trace write failed");
  } catch(const std::exception &e) {
    std::cerr<<"FAIL: "<<e.what()<<'\n';
    return 1;
  }
}
