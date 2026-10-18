#include "flight/core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace flight;
void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::runtime_error("airframe and trace.csv required");
    auto definition=Definition::load(argv[1]);
    const std::array<const char *,5> names{
        "intact","linkage","linkage_cell_loss","roll_disconnected","linkage_cell_spar"};
    std::ofstream trace(argv[2]);
    if (!trace) throw std::runtime_error("cannot write trace");
    trace << std::setprecision(17)
          << "condition,sign,hz,time_s,roll_command,roll_owner,left_angle_rad,right_angle_rad,roll_rate_rad_s,bank_rad,load_g,roll_pi_paused,failed_edges,debris\n";
    std::cout << std::setprecision(15)
              << "condition,sign,hz,release_command_jump,peak_rate_bound_excess_rad,ownership_error,bank_at_release_rad,bank_final_rad,peak_g,failed_edges,debris\n";
    std::array<std::vector<Body>,2> disconnected_reference;
    for (int condition=0;condition<5;++condition)
      for (double sign:{-1.,1.})
        for (double hz:{120.,240.}) {
          Simulation s(definition,1/hz);
          auto trim=s.trim(70,1000);
          require(trim.converged,"actual trimmed entry required");
          auto p=trim.intent;
          p.mouse=true; p.aim_world={1,.25,0};
          double jump=0,excess=0,owner_error=0,peak_g=0,release_bank=0;
          for(int i=0;i<int(3*hz);++i) {
            if(i==int(hz) && condition>0) {
              auto before=s.def.chains;
              auto &left=s.def.chains[0];
              left.lag=.15; left.friction=4; left.range=.6; left.hinge=.7;
              if(condition==2 || condition==4) {
                int removed=0;
                for(const auto &cell:s.def.cells)
                  if(cell.node==2 && removed++<8) s.damage_cell(cell.id,0,1);
              }
              if(condition==3) {
                s.disconnect("left_aileron"); s.disconnect("right_aileron");
              }
              if(condition==4) s.weaken("left_root_front",.3);
              for(std::size_t j=0;j<before.size();++j)
                require(s.def.chains[j].deflection==before[j].deflection,
                        "damage event cannot teleport a surface");
            }
            p.active[Roll]=i>=int(hz) && i<int(1.4*hz);
            p.direct[Roll]=sign*(i<int(1.2*hz)?1:-1);
            auto old=s.def.chains;
            double bias=s.instructor.rate_bias[Roll], command=s.ownership[Roll].command;
            double rho=1.225*std::exp(s.body.position.z/8500);
            Vec air=s.body.velocity-s.wind_world;
            double q=.5*rho*dot(air,air);
            s.step(p);
            if(condition==3) {
              auto &reference=disconnected_reference[hz==120 ? 0 : 1];
              if(sign<0) reference.push_back(s.body);
              else {
                const auto &b=reference.at(i);
                const auto &a=s.body;
                require(norm(a.position-b.position)==0 && norm(a.velocity-b.velocity)==0 &&
                            norm(a.omega-b.omega)==0 && a.attitude.w==b.attitude.w &&
                            a.attitude.x==b.attitude.x && a.attitude.y==b.attitude.y &&
                            a.attitude.z==b.attitude.z,
                        "opposite direct roll cannot bypass two disconnected links");
              }
            }
            if(p.active[Roll]) require(s.ownership[Roll].command==p.direct[Roll],
                                      "keyboard must retain full requested roll");
            require(s.ownership[Pitch].direct_weight==0 && s.ownership[Yaw].direct_weight==0,
                    "roll override must not own pitch or yaw");
            if(i==int(1.4*hz)) {
              jump=std::abs(s.ownership[Roll].command-command);
              Vec down=s.body.attitude.conjugate().rotate({0,0,1});
              release_bank=std::atan2(down.y,down.z);
            }
            if(i>=int(1.4*hz)) {
              double t=clamp((i-int(1.4*hz))/hz/definition.handoff,0,1);
              double expected=1-t*t*(3-2*t);
              owner_error=std::max(owner_error,std::abs(s.ownership[Roll].direct_weight-expected));
            }
            if(s.instructor.integral_paused[Roll])
              require(s.instructor.rate_bias[Roll]==bias,"paused PI bias cannot accumulate");
            for(std::size_t j=0;j<old.size();++j) {
              const auto &c=old[j];
              double rate=c.slew/(1+q/c.q_limit)/(1+std::max(c.friction,0.));
              excess=std::max(excess,std::abs(s.def.chains[j].deflection-c.deflection)-rate/hz);
            }
            peak_g=std::max(peak_g,s.load_factor);
            int failed=0; for(const auto &e:s.def.edges) failed+=e.failed;
            Vec down=s.body.attitude.conjugate().rotate({0,0,1});
            trace << names[condition] << ',' << sign << ',' << hz << ',' << (i+1)/hz
                  << ',' << s.ownership[Roll].command << ',' << s.ownership[Roll].direct_weight
                  << ',' << s.def.chains[0].deflection << ',' << s.def.chains[1].deflection
                  << ',' << s.body.omega.x << ',' << std::atan2(down.y,down.z)
                  << ',' << s.load_factor << ',' << s.instructor.integral_paused[Roll]
                  << ',' << failed << ',' << s.debris.size() << '\n';
          }
          require(jump<1e-14 && excess<1e-12 && owner_error<1e-12,
                  "damaged handoff must retain continuity, slew and ownership limits");
          int failed=0; for(const auto &e:s.def.edges) failed+=e.failed;
          Vec down=s.body.attitude.conjugate().rotate({0,0,1});
          std::cout << names[condition] << ',' << sign << ',' << hz << ',' << jump << ','
                    << excess << ',' << owner_error << ',' << release_bank << ','
                    << std::atan2(down.y,down.z) << ',' << peak_g << ',' << failed
                    << ',' << s.debris.size() << '\n';
        }
    if(!trace) throw std::runtime_error("trace write failed");
  } catch(const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
