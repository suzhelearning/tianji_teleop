// Private model computation -> destination project's existing loopback TJRC.
// No hardware SDK, enable, reset-alarm or Home-device operation exists here.
#include "bilateral_cycle.hpp"
#include "tianji_mapped_palm/joint_command.hpp"
#include "tianji_mapped_palm/pico_udp_receiver.hpp"
#include "tianji_mapped_palm/wuji_hand_udp_receiver.hpp"
#include "tianji_mapped_palm/pico_mapped_corrected_palm.hpp"
#include "calibration/height_calibration.hpp"
#include "target_observer.hpp"
#include "real_event_channel.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>

namespace {
using namespace tianji_mapped_palm;
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
unsigned port(const std::string& s) {
  std::size_t used{}; const auto value=std::stoul(s,&used);
  if(used!=s.size() || value<1 || value>65535) throw std::invalid_argument("invalid port");
  return static_cast<unsigned>(value);
}
}
int main(int argc,char** argv) {
  try {
    std::map<std::string,std::string> args;
    bool hands=false,calibrate=false,recovery=false,real_calibrate=false,bounded=false,dropout=false;
    for(int i=1;i<argc;++i) {
      std::string key=argv[i];
      if(key=="--help") {
        std::cout<<"mapped_palm_tjrc_controller --config YAML --model XML --joint-command-host 127.0.0.1 --joint-command-port PORT [--urdf URDF] [--pico-port PORT] [--hand-teleop --hand-port PORT]\n";
        return 0;
      }
      if(key=="--headless" || key=="--continuous" || key=="--model-state-only" || key=="--pico-teleop") continue;
      if(key=="--hand-teleop") { hands=true; continue; }
      if(key=="--mapped-palm-xz-calibration") { calibrate=true; continue; }
      if(key=="--simulation-recovery") { recovery=true; continue; }
      if(key=="--real-resync-bounded") { bounded=true; continue; }
      if(key=="--real-xz-calibration") { real_calibrate=true; continue; }
      if(key=="--real-input-hold-300ms") { dropout=true; continue; }
      if(key!="--config" && key!="--model" && key!="--urdf" && key!="--pico-bind" && key!="--pico-port" &&
         key!="--hand-bind" && key!="--hand-port" && key!="--joint-command-host" && key!="--joint-command-port" && key!="--target-overlay-port" && key!="--real-event-fd" && key!="--real-event-token")
        throw std::invalid_argument("unsupported option: "+key);
      if(++i>=argc || args.count(key)) throw std::invalid_argument("missing/duplicate option: "+key);
      args[key]=argv[i];
    }
    auto required=[&](const std::string& k) { if(!args.count(k)) throw std::invalid_argument("required: "+k); return args.at(k); };
    auto optional=[&](const std::string& k,const std::string& d) { return args.count(k)?args.at(k):d; };
    const auto model=required("--model");
    auto config=loadConfig(required("--config"));
    auto urdf=optional("--urdf",(std::filesystem::path(model).parent_path()/"marvin_m6_s_ccs_696_v4_local.urdf").string());
    const auto out_port=port(required("--joint-command-port"));
    const bool event_channel=args.count("--real-event-fd");
    if((bounded || real_calibrate || dropout) && !event_channel) throw std::invalid_argument("private event channel required");
    int event_fd=-1;
    std::string event_token;
    if(event_channel) {
      if(recovery || calibrate) throw std::invalid_argument("real event policy cannot use simulation recovery/calibration");
      event_token=required("--real-event-token");
      if(event_token.size()!=32 || event_token.find_first_not_of("0123456789abcdef")!=std::string::npos)
        throw std::invalid_argument("invalid event identity");
      std::size_t used=0; event_fd=std::stoi(args.at("--real-event-fd"),&used);
      int type=0; socklen_t n=sizeof(type);
      if(used!=args.at("--real-event-fd").size() || event_fd<0 ||
         getsockopt(event_fd,SOL_SOCKET,SO_TYPE,&type,&n)!=0 || type!=SOCK_SEQPACKET)
        throw std::invalid_argument("private seqpacket event endpoint required");
    } else if(args.count("--real-event-token")) throw std::invalid_argument("event fd required");
    PicoUdpReceiverOptions pico_options;
    pico_options.bind_address=optional("--pico-bind","127.0.0.1");
    pico_options.port=static_cast<std::uint16_t>(port(optional("--pico-port","15000")));
    pico_options.use_mapped_corrected_palm_target=true;
    WujiHandUdpReceiverOptions hand_options;
    hand_options.bind_address=optional("--hand-bind","127.0.0.1");
    hand_options.port=static_cast<std::uint16_t>(port(optional("--hand-port","16000")));
    if(pico_options.bind_address!="127.0.0.1" || hand_options.bind_address!="127.0.0.1" ||
       out_port==pico_options.port || (hands && (out_port==hand_options.port || pico_options.port==hand_options.port)))
      throw std::invalid_argument("distinct loopback ports required");
    // Construct/validate model BEFORE opening any socket.
    NativeMappedPalmCycle cycle(config,model,urdf,true);
    std::unique_ptr<TargetObserver> target_observer;
    if(args.count("--target-overlay-port")) {
      const auto target_port=port(args.at("--target-overlay-port"));
      if(target_port==out_port || target_port==pico_options.port || (hands && target_port==hand_options.port))
        throw std::invalid_argument("target observer requires a distinct port");
      target_observer=std::make_unique<TargetObserver>(target_port);
    }
    std::unique_ptr<tianji_control::HeightCalibration> calibration;
    if(calibrate || real_calibrate) {
      MujocoRobot reference(model);
      Vec7 q=Vec7::Zero(); q[1]=-std::acos(-1.)/2.;
      for(auto side:{ArmSide::kLeft,ArmSide::kRight}) reference.setArmState(side,q,Vec7::Zero());
      reference.forward();
      const auto left=reference.tcpPose(ArmSide::kLeft),right=reference.tcpPose(ArmSide::kRight);
      calibration=std::make_unique<tianji_control::HeightCalibration>(
          std::array<double,2>{left.position.z(),right.position.z()},
          std::array<double,2>{left.position.x(),right.position.x()});
      if(calibrate) {
        const int flags=fcntl(STDIN_FILENO,F_GETFL,0);
        if(flags<0 || fcntl(STDIN_FILENO,F_SETFL,flags|O_NONBLOCK)<0) throw std::runtime_error("calibration control pipe unavailable");
        std::cout<<"MAPPED: C = hold both arms forward/horizontal for 2 seconds; S = start after success. Recalibration requires a new session.\n"<<std::flush;
      }
    }
    if(recovery) {
      const int flags=fcntl(STDIN_FILENO,F_GETFL,0);
      if(flags<0 || fcntl(STDIN_FILENO,F_SETFL,flags|O_NONBLOCK)<0) throw std::runtime_error("simulation control pipe unavailable");
      std::cout<<"MAPPED SIM: P holds; R requires settled simulation feedback; S resumes after a new input. No hardware recovery authority.\n"<<std::flush;
    }
    JointCommandExporter exporter(required("--joint-command-host"),static_cast<std::uint16_t>(out_port));
    LatestSpscExchange<PicoTeleopFrame> pico_frames;
    LatestSpscExchange<WujiHandTeleopFrame> hand_frames;
    PicoUdpReceiver pico(pico_options,pico_frames);
    std::unique_ptr<WujiHandUdpReceiver> hand;
    if(hands) hand=std::make_unique<WujiHandUdpReceiver>(hand_options,hand_frames);
    std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
    pico.start(); if(hand) hand->start();
    JointCommandArmReadiness readiness;
    HandCommandFreshness left_fresh(ArmSide::kLeft),right_fresh(ArmSide::kRight);
    JointCommandFrame output;
    for(int i=0;i<7;++i) {
      output.position_rad[static_cast<std::size_t>(i)]=cycle.reference_state(ArmSide::kLeft).q[i];
      output.position_rad[static_cast<std::size_t>(i+7)]=cycle.reference_state(ArmSide::kRight).q[i];
    }
    std::uint64_t native_tick=0;
    std::uint64_t calibration_epoch=0,calibration_generation=0;
    bool active=!(calibrate || real_calibrate);
    bool calibration_locked=false;
    unsigned real_calibration_state=0;
    std::uint64_t calibration_revision=0;
    bool blocked=false,armed=false,offered=false;
    std::string recovery_line;
    std::int64_t last_rearm_stamp=0;
    std::uint64_t rearm_sequence=0;
    std::array<double,4> offsets{};
    std::optional<PicoTeleopFrame> latest;
    std::uint64_t observed_epoch=0,observed_generation=0;
    std::uint64_t last_applied_epoch=0;
    std::int64_t pending_since=0;
    bool bounded_fault=false;
    unsigned bounded_fault_reason=4;
    std::int64_t bridge_stamp=0,receive_stamp=0;
    std::int64_t input_valid_ns=0;
    std::uint64_t valid_sequence=0,valid_generation=0;
    bool valid_button=false,dropout_holding=false;
    PicoReceiverStats previous_input_stats;
    const auto period=std::chrono::nanoseconds(static_cast<std::int64_t>(1e9/config.controller.rate_hz));
    while(!stopped) {
      const auto start=std::chrono::steady_clock::now(); auto now=now_ns();
      PicoTeleopFrame input; std::optional<PicoTeleopFrame> sample;
      if(pico_frames.tryReadLatest(input)) { sample=input; latest=input; }
      now=now_ns();
      const auto input_event=pico.inputEvent();
      now=now_ns(); // Do not compare a newly received event against an older clock.
      const bool pending=bounded && input_event.state==2;
      if(pending && pending_since==0) pending_since=now;
      if(!pending) pending_since=0;
      if(real_calibrate) {
        if(latest && (real_calibration_state==2 || real_calibration_state==3) && latest->tracking_epoch!=calibration_epoch) {
          active=false; real_calibration_state=0; ++calibration_revision;
          std::cout<<"MAPPED REAL: epoch changed; calibration invalidated\n"<<std::flush;
        }
        unsigned char request[25]; auto count=recv(event_fd,request,sizeof(request),MSG_DONTWAIT);
        if(count==0) throw std::runtime_error("real calibration owner disconnected");
        if(count<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) throw std::runtime_error("real calibration command failed");
        if(count>0) {
          if(count!=24 || std::memcmp(request,"MRC1",4)!=0 || request[5] || request[6] || request[7])
            throw std::invalid_argument("invalid calibration command");
          auto get=[&](unsigned offset) { std::uint64_t v=0; for(unsigned i=0;i<8;++i) v|=std::uint64_t(request[offset+i])<<(8*i); return v; };
          const auto revision=get(8),epoch=get(16);
          if(calibration_locked) throw std::invalid_argument("calibration locked for physical execution");
          if(revision!=calibration_revision || !latest || epoch!=latest->tracking_epoch)
            throw std::invalid_argument("calibration command identity/epoch mismatch");
          if(request[4]==1) {
            cycle.reset_at_rest(config.controller.initial_left_q_rad,config.controller.initial_right_q_rad);
            native_tick=0; last_applied_epoch=0; readiness=JointCommandArmReadiness{};
            offered=false; bounded_fault=false; offsets={}; cycle.configure_xz(0,0,0,0);
            valid_sequence=0; dropout_holding=false;
            active=false; real_calibration_state=1; ++calibration_revision;
            now=now_ns(); sample.reset(); calibration->begin(now);
            std::cout<<"MAPPED REAL: sampling X/Z; no motor authority\n"<<std::flush;
          } else if(request[4]==2) {
            now=now_ns();
            if(real_calibration_state!=2 || !calibration->ready() || epoch!=calibration_epoch ||
               now<latest->receive_monotonic_ns || now-latest->receive_monotonic_ns>50000000 ||
               !jointCommandPicoBridgeFresh(latest->bridge_send_monotonic_ns,now))
              throw std::invalid_argument("cannot lock stale/unready calibration");
            calibration_locked=true; real_calibration_state=3;
            std::cout<<"MAPPED REAL: calibration locked; executor must recheck feedback before enable\n"<<std::flush;
          } else throw std::invalid_argument("unknown calibration command");
        }
      }
      if(calibrate || recovery) {
        char keys[4096]; const auto count=read(STDIN_FILENO,keys,sizeof(keys));
        if(count==0) throw std::runtime_error("calibration control pipe closed");
        if(count<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) throw std::runtime_error("calibration control pipe failed");
        for(ssize_t i=0;i<count;++i) {
          if(recovery && (keys[i]=='R' || !recovery_line.empty())) {
            recovery_line+=keys[i];
            if(recovery_line.size()>2048) throw std::runtime_error("oversized recovery request");
            if(keys[i]!='\n') continue;
            std::istringstream request(recovery_line); recovery_line.clear();
            char tag; std::int64_t stamp=0; Vec7 left,right; double speed=0; bool valid=true;
            request>>tag>>stamp;
            for(int j=0;j<14;++j) { double v=0; if(!(request>>v) || !std::isfinite(v)) valid=false; (j<7?left[j]:right[j-7])=v; }
            for(int j=0;j<14;++j) { double v=0; if(!(request>>v) || !std::isfinite(v)) valid=false; speed=std::max(speed,std::abs(v)); }
            std::string extra; if(request>>extra) valid=false;
            valid=valid && tag=='R' && (!active || blocked) && stamp>last_rearm_stamp && now>=stamp && now-stamp<=50000000 && speed<=.03 && latest &&
              now>=latest->receive_monotonic_ns && now-latest->receive_monotonic_ns<=50000000 &&
              jointCommandPicoBridgeFresh(latest->bridge_send_monotonic_ns,now) && selectMappedCorrectedPalm(*latest).valid;
            if(!valid) { std::cout<<"MAPPED: R rejected: hold first, fresh input and settled feedback required\n"<<std::flush; continue; }
            try { cycle.reset_at_rest(left,right); }
            catch(const std::invalid_argument& error) { std::cout<<"MAPPED: R rejected: "<<error.what()<<'\n'<<std::flush; continue; }
            cycle.configure_xz(offsets[0],offsets[1],offsets[2],offsets[3]); native_tick=0;
            last_applied_epoch=0;
            readiness=JointCommandArmReadiness{}; active=false; blocked=false; armed=true; offered=false;
            last_rearm_stamp=stamp; rearm_sequence=latest->sequence;
            calibration_epoch=latest->tracking_epoch; calibration_generation=latest->resynchronization_generation;
            left_fresh.reset(); right_fresh.reset();
            std::cout<<"MAPPED: R accepted at measured rest; wait for new input then S\n"<<std::flush;
            continue;
          }
          if(recovery && keys[i]=='p') {
            active=false; armed=false; blocked=true;
            readiness.update(false,true);
            std::cout<<"MAPPED: holding; settle then R and S\n"<<std::flush; continue;
          }
          if(recovery && keys[i]=='s' && !calibration) {
            if(armed && !blocked && latest && latest->sequence>rearm_sequence && latest->tracking_epoch==calibration_epoch &&
               latest->resynchronization_generation==calibration_generation && now>=latest->receive_monotonic_ns &&
               now-latest->receive_monotonic_ns<=50000000 && jointCommandPicoBridgeFresh(latest->bridge_send_monotonic_ns,now)) {
              active=true; armed=false; std::cout<<"MAPPED: resumed\n"<<std::flush;
            } else std::cout<<"MAPPED: S rejected: R and fresh same-epoch input required\n"<<std::flush;
            continue;
          }
          if(!calibration) continue;
          if(keys[i]=='c' && (blocked || native_tick!=0)) {
            std::cout<<"MAPPED: C rejected: hold and R first\n"<<std::flush; continue;
          }
          if(keys[i]=='c' && !active) { calibration->begin(now); std::cout<<"MAPPED: sampling X/Z\n"<<std::flush; }
          if(keys[i]=='s' && !active && !blocked && calibration->ready() && latest &&
             (!recovery || !armed || latest->sequence>rearm_sequence) &&
             latest->tracking_epoch==calibration_epoch && latest->resynchronization_generation==calibration_generation &&
             selectMappedCorrectedPalm(*latest).valid && now>=latest->receive_monotonic_ns &&
             now-latest->receive_monotonic_ns<=50000000) {
            active=true; armed=false; std::cout<<"MAPPED: simulation input started\n"<<std::flush;
          } else if(keys[i]=='s') {
            const char* reason=active?"already active":blocked?"holding: settle, R then S":
              !calibration->ready()?"C calibration required":!latest?"no PICO input":
              latest->tracking_epoch!=calibration_epoch?"tracking epoch changed: recalibrate C (or R at rest)":
              latest->resynchronization_generation!=calibration_generation?"input resynchronized: recalibrate C (or R at rest)":
              !selectMappedCorrectedPalm(*latest).valid?"invalid palm pose":
              (now<latest->receive_monotonic_ns || now-latest->receive_monotonic_ns>50000000)?"stale PICO input":
              "new input required after R";
            std::cout<<"MAPPED: S rejected: "<<reason<<'\n'<<std::flush;
          }
        }
      }
      now=now_ns(); // The receiver may publish while keys are being processed.
      if(calibration && !active) {
        const auto before=calibration->status().state;
        if(sample) {
          const auto palms=selectMappedCorrectedPalm(*sample);
          tianji_control::RawProgress p;
          p.accepted=true; p.epoch=sample->tracking_epoch; p.sequence=sample->sequence;
          p.generation=sample->resynchronization_generation;
          p.skeleton_valid=palms.valid; p.rotations_valid=sample->upper_limb_skeleton.rotations_valid;
          for(int axis=0;axis<3;++axis) {p.palms[0][axis]=palms.left.position[axis];p.palms[1][axis]=palms.right.position[axis];}
          calibration->add(p,sample->receive_monotonic_ns,now);
        }
        if(calibration->tick(now)) {
          const auto x=*calibration->candidate_x(),z=*calibration->candidate();
          cycle.configure_xz(x[0],x[1],z[0],z[1]); calibration->commit();
          offsets={x[0],x[1],z[0],z[1]};
          calibration_epoch=latest->tracking_epoch; calibration_generation=latest->resynchronization_generation;
          if(real_calibrate) { real_calibration_state=2; active=true; }
          std::cout<<"MAPPED: calibration SUCCESS X=["<<x[0]<<","<<x[1]<<"] Z=["<<z[0]<<","<<z[1]<<"]"<<(real_calibrate?"; Enter to align (still disabled)\n":"; press S\n")<<std::flush;
        } else if(before!="failed" && calibration->status().state=="failed") {
          if(real_calibrate) real_calibration_state=4;
          std::cout<<"MAPPED: calibration FAILED: "<<calibration->status().error<<"\n"<<std::flush;
        }
      }
      // Check the original valid-input deadline BEFORE consuming a newly
      // arrived sample. Neither cached frames nor late input may renew it.
      bool dropout_wait=false;
      unsigned dropout_fault=0;
      const auto input_stats=dropout?pico.stats():PicoReceiverStats{};
      if(dropout && active && offered && !bounded_fault) {
        const auto& stats=input_stats;
        if(input_valid_ns<=0 || now<input_valid_ns || now-input_valid_ns>300000000)
          dropout_fault=4;
        if(input_event.state==0 || stats.malformed>previous_input_stats.malformed ||
           stats.crc_failures>previous_input_stats.crc_failures ||
           stats.reordered>previous_input_stats.reordered)
          dropout_fault=4;
        if(input_event.state==2 && (!bounded || dropout_holding)) dropout_fault=4;
        if(input_event.epoch!=last_applied_epoch ||
           (dropout_holding && latest && latest->resynchronization_generation!=valid_generation))
          dropout_fault=6;
        if(input_event.button!=valid_button) dropout_fault=7;
        if(sample && (sample->bridge_send_monotonic_ns<=0 ||
           sample->receive_monotonic_ns>now || sample->bridge_send_monotonic_ns>now ||
           std::min(sample->receive_monotonic_ns,sample->bridge_send_monotonic_ns)<input_valid_ns))
          dropout_fault=4;
        const bool new_fresh_sample=sample && sample->sequence!=valid_sequence &&
          sample->receive_monotonic_ns>0 && sample->bridge_send_monotonic_ns>0 &&
          now>=sample->receive_monotonic_ns && now>=sample->bridge_send_monotonic_ns &&
          now-std::min(sample->receive_monotonic_ns,sample->bridge_send_monotonic_ns)<50000000;
        dropout_wait=!dropout_fault && input_event.state==1 &&
          now-input_valid_ns>=50000000 && !new_fresh_sample;
        if(dropout_wait && latest && latest->resynchronization_generation!=valid_generation) {
          dropout_fault=6; dropout_wait=false;
        }
        if(dropout_fault && (!real_calibrate || calibration_locked)) {
          bounded_fault=true; bounded_fault_reason=dropout_fault;
        }
      }
      if(dropout) {
        previous_input_stats=input_stats;
        // A fresh local receive cannot make a delayed bridge sample fresh.
        if(sample && (sample->receive_monotonic_ns<=0 || sample->bridge_send_monotonic_ns<=0 ||
           now<sample->receive_monotonic_ns || now<sample->bridge_send_monotonic_ns ||
           now-std::min(sample->receive_monotonic_ns,sample->bridge_send_monotonic_ns)>=50000000))
          sample.reset();
      }
      BilateralCycleResult r;
      const bool step_enabled=active && !pending && !bounded_fault && !dropout_wait && !dropout_fault;
      if(step_enabled) {
        if(native_tick==0 && !dropout) sample=latest;
        r=cycle.step(++native_tick,now,sample);
      }
      if(sample && r.applied_sequence==sample->sequence) {
        bridge_stamp=sample->bridge_send_monotonic_ns; receive_stamp=sample->receive_monotonic_ns;
      }
      ++output.sequence; output.source_timestamp_ns=now_ns(); output.pico_tracking_epoch=active?r.applied_epoch:(latest?latest->tracking_epoch:0);
      if(dropout && offered && input_valid_ns>0 && output.source_timestamp_ns-input_valid_ns>300000000) {
        dropout_fault=4; dropout_wait=false;
      }
      if(step_enabled) for(int i=0;i<7;++i) { output.position_rad[static_cast<std::size_t>(i)]=r.left.q[i]; output.position_rad[static_cast<std::size_t>(i+7)]=r.right.q[i]; }
      WujiHandTeleopFrame h;
      if(hand_frames.tryReadLatest(h)) {
        left_fresh.observe(h); right_fresh.observe(h);
        for(std::size_t i=0;i<20;++i) {
          if(h.left_valid) output.position_rad[i+14]=h.left[i];
          if(h.right_valid) output.position_rad[i+34]=h.right[i];
        }
      }
      // Use the same clock snapshot as dropout_wait and cycle.step: crossing
      // 50 ms while solving is not an IK failure. The next cycle enters hold;
      // the independent post-solve 300 ms deadline above remains authoritative.
      const auto freshness_now=dropout?now:output.source_timestamp_ns;
      const bool live=active && r.freshness.live && r.control_executed && r.control.left.accepted && r.control.right.accepted &&
          receive_stamp>0 && freshness_now>=receive_stamp &&
          freshness_now-receive_stamp<=static_cast<std::int64_t>(config.cartesian_servo.target_timeout_seconds*1e9) &&
          jointCommandPicoBridgeFresh(bridge_stamp,freshness_now);
      // The kernel uses epoch_reset for both a new tracking epoch and an
      // accepted same-epoch stream resynchronization. Only the latter may
      // continue in simulation, after rebuilding references in cycle.step().
      // Compare applied epochs, not latest received epochs (application can lag).
      const bool same_epoch_resync=(recovery || bounded) && r.epoch_reset && last_applied_epoch!=0 &&
          r.applied_epoch==last_applied_epoch;
      const bool reset_requires_hold=(r.epoch_reset && !same_epoch_resync) ||
          r.button_action!=PicoTeleopButtonAction::kNone;
      const bool arm_ready=readiness.update(live,reset_requires_hold);
      if(dropout && !dropout_fault && live && sample && sample->sequence!=valid_sequence &&
         r.applied_sequence==sample->sequence) {
        input_valid_ns=std::min(sample->receive_monotonic_ns,sample->bridge_send_monotonic_ns);
        valid_sequence=sample->sequence; valid_generation=sample->resynchronization_generation;
        valid_button=sample->user_button_pressed;
      }
      if(recovery && active && offered && (r.epoch_reset || r.button_action!=PicoTeleopButtonAction::kNone)) {
        if(reset_requires_hold) { active=false; blocked=true; armed=false; }
        if(latest) {
          const auto& f=*latest; const auto& d=f.reset_diagnostic;
          std::cout<<"MAPPED_RESET_DIAG: now_ns="<<now
            <<" epoch_before="<<observed_epoch<<" epoch_now="<<f.tracking_epoch
            <<" generation_before="<<observed_generation<<" generation_now="<<f.resynchronization_generation
            <<" button_action="<<static_cast<int>(r.button_action)<<" button_pressed="<<f.user_button_pressed
            <<" cycle_reset="<<r.epoch_reset<<" frame_sequence="<<f.sequence
            <<" hold_required="<<reset_requires_hold
            <<" event_sequence="<<d.sequence<<" anchor_sequence="<<d.previous_sequence
            <<" anchor_epoch="<<d.previous_epoch<<" event_receive_ns="<<d.receive_ns
            <<" accepted_gap_ms="<<d.accepted_gap_ns/1e6
            <<" left_jump_m="<<d.left_position_m<<" right_jump_m="<<d.right_position_m
            <<" left_jump_rad="<<d.left_orientation_rad<<" right_jump_rad="<<d.right_orientation_rad
            <<" input_age_ms="<<(now-f.receive_monotonic_ns)/1e6<<'\n'<<std::flush;
        }
        std::cout<<(reset_requires_hold?
            "MAPPED: tracking reset; holding. Settle then R and S\n":
            "MAPPED: same-epoch input resynchronized; following remains enabled\n")<<std::flush;
      }
      if(r.applied_sequence!=0) last_applied_epoch=r.applied_epoch;
      if(latest) { observed_epoch=latest->tracking_epoch; observed_generation=latest->resynchronization_generation; }
      offered=offered || arm_ready;
      output.flags=static_cast<std::uint8_t>(
          (arm_ready && (!recovery || active)?kJointCommandArmsReadyFlag:0U) |
          ((!recovery || active) && left_fresh.live(output.source_timestamp_ns)?kJointCommandLeftHandReadyFlag:0U) |
          ((!recovery || active) && right_fresh.live(output.source_timestamp_ns)?kJointCommandRightHandReadyFlag:0U));
      if(event_channel) {
        // No ready-bit fabrication: holds authorize only the executor's own
        // last command, with its independent feedback and timeout guards.
        const bool event_fresh=input_event.receive_ns>0 && now>=input_event.receive_ns &&
          now-input_event.receive_ns<=50000000 && jointCommandPicoBridgeFresh(input_event.bridge_ns,now);
        const bool hold_ok=pending && offered && latest && event_fresh &&
          input_event.epoch==last_applied_epoch && input_event.button==latest->user_button_pressed &&
          now-pending_since<=100000000;
        unsigned event_state=arm_ready?1:5; // IK/output not accepted
        if(hold_ok) event_state=2;
        else if(same_epoch_resync && arm_ready) event_state=3;
        if(dropout_wait) event_state=9;
        if((!event_fresh && !dropout_wait) || input_event.state==0) event_state=4; // source invalid/stale
        if(pending && now-pending_since>100000000) event_state=8;
        if((input_event.epoch!=last_applied_epoch && offered) || (r.epoch_reset && !same_epoch_resync && offered)) event_state=6;
        if(r.button_action!=PicoTeleopButtonAction::kNone ||
           (pending && latest && input_event.button!=latest->user_button_pressed)) event_state=7;
        if(dropout_fault) event_state=dropout_fault;
        if(offered && event_state>=4 && event_state!=9 && !bounded_fault && (!real_calibrate || calibration_locked)) { bounded_fault=true; bounded_fault_reason=event_state; }
        if(bounded_fault) event_state=bounded_fault_reason;
        if(event_state>=4) output.flags &= ~kJointCommandArmsReadyFlag;
        // Frozen cycles have no kernel result; retain the applied identity.
        if(pending || (dropout && active && offered && !step_enabled)) output.pico_tracking_epoch=last_applied_epoch;
        dropout_holding=event_state==9;
        sendRealEvent(event_fd,event_token,event_state,output.sequence,
                      latest?latest->resynchronization_generation:0,output,
                      real_calibrate,calibration_revision,calibration_epoch,real_calibration_state,offsets,
                      dropout,input_valid_ns);
      }
      exporter.send(output);
      if(target_observer && output.sequence%4==0) target_observer->publish(r,output.sequence,output.source_timestamp_ns,live);
      std::this_thread::sleep_until(start+period);
    }
    if(hand) hand->stop(); pico.stop();
    output.flags=0; ++output.sequence; output.source_timestamp_ns=now_ns(); exporter.send(output);
    return 0;
  } catch(const std::exception& e) { std::cerr<<"mapped-palm: "<<e.what()<<'\n'; return 1; }
}
