// Simulation-only local pipe. Reuses the VR controller, model, Pinocchio,
// online Ruckig, soft-start and recovery. No sockets, SDK or joint publisher.
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/simulation_recovery.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace {
using namespace tianji_qp_ik;
template<class T> T get(const unsigned char* p) {T v;std::memcpy(&v,p,sizeof v);return v;}
template<class T> void put(unsigned char* p,T v) {std::memcpy(p,&v,sizeof v);}
void check(bool ok,const char* why) {if(!ok)throw std::runtime_error(why);}
constexpr std::array<ArmSide,2> sides{ArmSide::kLeft,ArmSide::kRight};
int run(const char* profile,const char* model,bool continuous_follow) {
  const std::uint32_t endian=1;
  check(*reinterpret_cast<const unsigned char*>(&endian)==1&&sizeof(double)==8,"unsupported binary host");
  auto cfg=loadConfig(profile);
  check(cfg.ik_algorithm==IkAlgorithm::kPicoEeFrankaDls&&cfg.controller.model_state_only,"DLS model-reference profile required");
  MujocoRobot robot(model);
  SimulationRecovery::Pair home;
  for(int i=0;i<2;++i) {
    home[i].q=configuredInitialPosture(cfg.controller,robot.mapping(sides[i]).limits,sides[i]);
    robot.setArmPosition(sides[i],home[i].q);
  }
  robot.forward();
  DualArmController controller(robot,cfg);
  SimulationRecovery recovery(cfg,{robot.mapping(sides[0]).limits,robot.mapping(sides[1]).limits},home,.005,
                              continuous_follow?.05:.3);
  auto state=[&]() {return SimulationRecovery::Pair{controller.referenceState(sides[0]),controller.referenceState(sides[1])};};
  std::cout<<"{\"schema_version\":1,\"kind\":\"pico2_dls_ready\",\"simulation_only\":true}\n"<<std::flush;
  std::uint64_t sequence=0,epoch=0;
  double last_now=-1,last_source=-1,last_live_received=-1;
  for(;;) {
    std::array<unsigned char,272> request{};
    std::cin.read(reinterpret_cast<char*>(request.data()),request.size());
    if(std::cin.gcount()==0&&std::cin.eof())return 0;
    check(std::cin.gcount()==272,"truncated request");
    check(std::memcmp(request.data(),"P2IQ",4)==0&&request[4]==1&&get<std::uint16_t>(&request[6])==272,"bad request header");
    const auto op=request[5];
    const auto seq=get<std::uint64_t>(&request[8]),ep=get<std::uint64_t>(&request[16]);
    check(op>=1&&op<=8&&seq>sequence,"bad operation/sequence");
    check(op==1?ep==epoch+1:ep==epoch,"bad epoch");
    const auto source=get<double>(&request[24]),received=get<double>(&request[32]),now=get<double>(&request[40]);
    check(std::isfinite(source)&&std::isfinite(received)&&std::isfinite(now)&&source>=0&&received>=0&&now>=received,"bad clock");
    if(op>=2&&op!=3)check(epoch>0&&now>=last_now,"unprepared or clock rollback");
    if(op==2)check(now>last_now&&source>=last_source,"solve clock rollback");
    std::array<Vec7,2> seeds;
    DualArmTargets targets;
    for(int i=0;i<2;++i) {
      for(int j=0;j<7;++j)seeds[i][j]=get<double>(&request[48+(i*7+j)*8]);
      const auto& lim=robot.mapping(sides[i]).limits;
      check(seeds[i].allFinite()&&(seeds[i].array()>=lim.lower_position.array()).all()&&
            (seeds[i].array()<=lim.upper_position.array()).all(),"bad seed/limit");
      if(op!=3)check((seeds[i]-controller.reference(sides[i])).cwiseAbs().maxCoeff()<1e-7,"owner reference mismatch");
      double p[7];for(int j=0;j<7;++j){p[j]=get<double>(&request[160+(i*7+j)*8]);check(std::isfinite(p[j]),"bad pose");}
      if(op==2) {
        Eigen::Quaterniond q(p[6],p[3],p[4],p[5]);
        check(std::isfinite(q.norm())&&q.norm()>1e-12,"bad quaternion");
        auto& t=i==0?targets.left:targets.right;
        t.position=Eigen::Vector3d(p[0],p[1],p[2]);t.rotation=q.normalized().toRotationMatrix();
      }
    }
    bool accepted=true;
    if(op==1) {
      check((recovery.phase()==SimulationRecovery::Phase::kWaiting||
             recovery.phase()==SimulationRecovery::Phase::kHold||
             recovery.phase()==SimulationRecovery::Phase::kHomeReached)&&
            SimulationRecovery::atRest(state()),"reset while moving");
      controller.resetSolvers();epoch=ep;last_source=-1;last_live_received=-1;
    } else if(op==4) {
      last_live_received=-1;
      accepted=recovery.start(now-received<=.045,state());
      // PICO2 simulation-only faster approach. All other callers retain the
      // original default caps; nominal limits and the 0.5 s ramp are unchanged.
      if(accepted) {controller.resetSolvers();check(controller.beginSimulationSoftStart({1.4,3.,12.}),"soft start rejected");}
    } else if(op==8) {
      accepted=continuous_follow&&recovery.phase()==SimulationRecovery::Phase::kHold&&
          last_live_received>=0&&now-last_live_received<=1.&&
          recovery.start(now-received<=.045,state());
      // Reset the IK seed at rest, but retain the existing limiter's soft-start
      // caps and ramp progress. Only an accepted live solve refreshes eligibility.
      if(accepted)controller.resetSolvers();
    } else if(op==5||op==6) {
      if(op==5)last_live_received=-1;
      accepted=recovery.stop(state(),op==5);
    } else if(op==2) {
      check(recovery.teleop(),"solve outside TELEOP");
      targets.left_stale=targets.right_stale=now-received>.045;
      const auto result=controller.step(targets,.005);
      accepted=result.accepted;
      // Preserve the accepted controller's bounded braking output on rejection.
      if(accepted)last_live_received=received;
      else {
        last_live_received=-1;
        check(recovery.stop(state(),false),"failed to enter hold");
      }
      last_source=source;
    } else if(op==7) {
      const auto next=recovery.update(state());
      for(int i=0;i<2;++i)check(controller.setReferenceState(sides[i],next[i]),"recovery reference rejected");
    }
    check(recovery.phase()!=SimulationRecovery::Phase::kFault,"recovery fault");
    std::array<unsigned char,432> response{};
    std::memcpy(response.data(),"P2IR",4);response[4]=1;response[5]=op;
    put<std::uint16_t>(&response[6],432);put(&response[8],seq);put(&response[16],epoch);
    for(int i=0;i<2;++i) {
      const Vec7 q=op==3?seeds[i]:controller.reference(sides[i]);
      const auto pose=robot.armKinematicsAt(sides[i],q).tcp_pose;
      const int offset=24+i*204;
      put<std::uint32_t>(&response[offset],4U|(accepted?1U:0U));
      for(int j=0;j<7;++j)put(&response[offset+4+j*8],q[j]);
      for(int j=0;j<3;++j)put(&response[offset+60+j*8],pose.position[j]);
      const Eigen::Quaterniond rotation(pose.rotation);
      for(int j=0;j<4;++j)put(&response[offset+84+j*8],rotation.coeffs()[j]);
      if(op==2) {
        const auto& target=i==0?targets.left:targets.right;
        put(&response[offset+116],(target.position-pose.position).norm());
        put(&response[offset+124],rotationDistance(target.rotation,pose.rotation));
      }
      std::strncpy(reinterpret_cast<char*>(&response[offset+140]),recovery.name(),63);
    }
    if(op!=3)last_now=now;
    sequence=seq;
    std::cout.write(reinterpret_cast<const char*>(response.data()),response.size());std::cout.flush();
    check(bool(std::cout),"response pipe failed");
  }
}
}
int main(int argc,char** argv) {
  try {
    if(argc!=3&&argc!=4)return 2;
    const bool continuous_follow=argc==4;
    if(continuous_follow&&std::strcmp(argv[3],"--continuous-follow")!=0)return 2;
    return run(argv[1],argv[2],continuous_follow);
  }
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
