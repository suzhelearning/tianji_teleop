#pragma once
// Offline only; the caller validates the TJVT file with the existing reader.
inline void ceresReferenceProbe(const std::vector<tianji_qp_ik::PicoTeleopFrame>& frames,
    tianji_qp_ik::SharedRootOptions options, tianji_qp_ik::QpIkConfig config) {
  using namespace tianji_qp_ik;
  if (config.ik_algorithm != IkAlgorithm::kPicoEeFrankaCeresLm)
    throw std::invalid_argument("--ceres-reference-loop requires Ceres profile");
  options.enabled=true; // offline model reference only, never writes a profile
  MujocoRobot robot(options.mujoco_path);
  for(auto side:{ArmSide::kLeft,ArmSide::kRight})
    robot.setArmPosition(side,configuredInitialPosture(config.controller,robot.mapping(side).limits,side));
  robot.forward();
  DualArmController controller(robot,config);
  DualArmSparkGuidance guidance(robot,config,options.urdf_path,SparkPostureGuideMode::kRuckig,&options);
  std::size_t index=0, cycles=0, ready=0, accepted=0, rejected=0, ack=0;
  std::vector<double> ik[2],tracking[2],rotation[2];
  const auto first=frames.front().receive_monotonic_ns;
  const auto last=frames.back().receive_monotonic_ns;
  std::cout<<std::setprecision(17);
  for(auto now=first;now<=last+1000000000;now+=5000000) {
    while(index<frames.size() && frames[index].receive_monotonic_ns<=now) {
      guidance.updateSharedRootFrame(frames[index],now);++index;
    }
    const auto mapped=guidance.stepSharedRoot(controller.referenceState(ArmSide::kLeft),
        controller.referenceState(ArmSide::kRight),.005,now,true,true);
    if(mapped.reference_reset_required)controller.resetSolvers();
    auto targets=mapped.cartesian_targets;
    targets.left_stale=targets.right_stale=!mapped.target_valid;
    const auto result=controller.step(targets,.005);
    ++cycles;ready+=mapped.target_valid;accepted+=result.accepted;
    rejected+=mapped.target_valid&&!result.accepted;
    ack+=guidance.confirmSharedRootReference(mapped.shared_root_cycle,result.accepted);
    const double t=double(now-first)*1e-9;
    for(int side=0;side<2;++side) {
      const auto& arm=side?result.right:result.left;
      if(t>=5&&t<=47&&mapped.target_valid&&result.accepted) {
        ik[side].push_back(arm.dls_posture_final_position_error_m);
        tracking[side].push_back(arm.pose_error.head<3>().norm());
        rotation[side].push_back(arm.pose_error.tail<3>().norm());
      }
      std::cout<<"ceres_frame t="<<t<<" side="<<side<<" mapped="<<mapped.target_valid
        <<" accepted="<<result.accepted<<" state="<<static_cast<int>(mapped.shared_root_state)
        <<" ik_m="<<arm.dls_posture_final_position_error_m
        <<" tracking_m="<<arm.pose_error.head<3>().norm()
        <<" tracking_rad="<<arm.pose_error.tail<3>().norm()
        <<" pinocchio="<<arm.ee_pinocchio_kinematics
        <<" ruckig_invoked="<<arm.ee_ruckig_invoked
        <<" ik_wall_us="<<arm.ee_ik_wall_time_us
        <<" ruckig_wall_us="<<arm.ee_ruckig_wall_time_us
        <<" pipeline_wall_us="<<arm.ee_ik_to_ruckig_wall_time_us
        <<" detail="<<arm.ik.detail<<" smoother="<<arm.dls_posture_ruckig_detail<<'\n';
    }
  }
  const auto percentile=[](std::vector<double> v) {
    if(v.empty())throw std::runtime_error("no accepted action samples");
    std::sort(v.begin(),v.end());return v[static_cast<std::size_t>(std::ceil(.9*double(v.size())))-1];
  };
  std::cout<<"ceres_reference cycles="<<cycles<<" mapped_ready="<<ready<<" accepted="<<accepted
    <<" mapped_rejected="<<rejected<<" acknowledgements="<<ack<<" source_frames="<<index<<'\n';
  for(int side=0;side<2;++side)
    std::cout<<"ceres_summary side="<<side<<" samples="<<ik[side].size()
      <<" ik_p90_m="<<percentile(ik[side])<<" tracking_p90_m="<<percentile(tracking[side])
      <<" rotation_p90_rad="<<percentile(rotation[side])<<'\n';
  std::cout<<"ceres_stopped left_velocity="<<controller.previousVelocity(ArmSide::kLeft).norm()
    <<" right_velocity="<<controller.previousVelocity(ArmSide::kRight).norm()
    <<" scope=model_reference actual=unavailable motion_authorized=false phase_a_accepted=false\n";
}
