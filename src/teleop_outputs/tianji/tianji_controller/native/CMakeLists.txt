cmake_minimum_required(VERSION 3.24)

project(tj_arm_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Installed native processes must load the control ABI even when a default or
# policy Python launcher starts them. Both layouts remain checkout-relative.
set(CMAKE_BUILD_RPATH "$ENV{CONDA_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib;$ORIGIN/../../../.pixi/envs/control/lib")
add_link_options("-Wl,--disable-new-dtags")

# Models moved to the description package; the controller keeps only profiles.
set(TIANJI_DESCRIPTION_PACKAGE
  "${CMAKE_CURRENT_LIST_DIR}/../../tianji_description")
set(TIANJI_DESCRIPTION_MODELS "${TIANJI_DESCRIPTION_PACKAGE}/models")

# Python-driven ctest entries import the workspace's installed python packages
# (tianji_description, tianji_runtime). The control environment is deliberately
# ROS-free and does not source the overlay, so the site-packages path is passed
# explicitly; a missing build fails with an explicit message.

if(NOT EXISTS "${TIANJI_DESCRIPTION_MODELS}/marvin_m6_wuji2.xml")
  message(FATAL_ERROR "controller models not found at ${TIANJI_DESCRIPTION_MODELS}")
endif()


# Keep the validated viewer/physics API; MuJoCo 3.12 changes mjv_moveCamera.
find_package(mujoco 3.10.0 EXACT CONFIG REQUIRED)
find_package(Eigen3 3.4 CONFIG REQUIRED NO_MODULE)
find_package(pinocchio CONFIG REQUIRED)
find_package(osqp CONFIG REQUIRED)
find_package(qpOASES CONFIG QUIET)
if(NOT qpOASES_FOUND)
  find_package(qpOASES MODULE REQUIRED)
endif()
find_package(yaml-cpp CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(Threads REQUIRED)
include(cmake/OptionalCeres.cmake)

# Build the pinned, licensed offline trajectory generator from this checkout.
add_subdirectory(third_party/ruckig)

if(NOT TARGET qpOASES::qpOASES)
  add_library(qpOASES::qpOASES UNKNOWN IMPORTED)
  set_target_properties(qpOASES::qpOASES PROPERTIES
    IMPORTED_LOCATION "${qpOASES_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${qpOASES_INCLUDE_DIR}"
  )
endif()

function(tianji_enable_warnings target)
  target_compile_options(${target} PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wshadow>
  )
endfunction()

add_library(tianji_qp_ik STATIC
  src/pico_ee_franka_ceres_lm.cpp
  src/ceres_trajectory_limiter.cpp
  src/ceres_pinocchio_arm_kinematics.cpp
  src/controller_ceres_lm.cpp
  src/franka_iterative_pose_dls.cpp
  src/pico_ee_franka_dls.cpp
  src/acceleration_bounds.cpp
  src/arm_angle.cpp
  src/acceleration_controller.cpp
  src/acceleration_qp.cpp
  src/acceleration_qpoases_solver.cpp
  src/benchmark_dataset.cpp
  src/cartesian_acceleration_servo.cpp
  src/cartesian_frf.cpp
  src/cartesian_otg.cpp
  src/cartesian_servo.cpp
  src/config.cpp
  src/controller.cpp
  src/equality_constrained_qpoases_solver.cpp
  src/hierarchical_qp.cpp
  src/hierarchical_qp_ik.cpp
  src/hierarchical_qpoases_solver.cpp
  src/interactive_marker.cpp
  src/iterative_pose_dls.cpp
  src/joint_trajectory_limiter.cpp
  src/joint_kinematics_plot.cpp
  src/joint_command.cpp
  src/mujoco_marker_adapter.cpp
  src/mujoco_joint_plot.cpp
  src/mujoco_robot.cpp
  src/nullspace_dls.cpp
  src/osqp_solver.cpp
  src/pinocchio_arm_kinematics.cpp
  src/pico_teleop_protocol.cpp
  src/pico_skeleton_overlay.cpp
  src/pico_teleop_session.cpp
  src/pico_trace_recorder.cpp
  src/pico_udp_receiver.cpp
  src/qp_builder.cpp
  src/qpoases_solver.cpp
  src/safety.cpp
  src/spark_upper_retarget.cpp
  src/shared_root_input.cpp
  src/shared_root_morphology.cpp
  src/shared_root_target_builder.cpp
  src/shared_root_closure.cpp
  src/shared_root_continuity.cpp
  src/shared_root_options.cpp
  src/shared_root_sha256.cpp
  src/shared_root_pipeline.cpp
  src/shared_root_guidance.cpp
  src/spark_upper_qpoases_ik.cpp
  src/spark_posture_reference.cpp
  src/spark_feedforward_reference.cpp
  src/spark_constraint_headroom.cpp
  src/spark_palm_twist_estimator.cpp
  src/spark_guidance.cpp
  src/spark_qpoases_diagnostic.cpp
  src/so3.cpp
  src/target_manager.cpp
  src/telemetry.cpp
  src/upper_arm_outward.cpp
  src/velocity_ik.cpp
  src/wuji_hand_teleop_protocol.cpp
  src/wuji_hand_udp_receiver.cpp
)
target_include_directories(tianji_qp_ik PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(tianji_qp_ik PUBLIC
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  mujoco::mujoco
  osqp::osqp
  pinocchio::pinocchio
  qpOASES::qpOASES
  ruckig::ruckig
)
tianji_enable_warnings(tianji_qp_ik)
if(TIANJI_ENABLE_CERES)
  target_link_libraries(tianji_qp_ik PUBLIC Ceres::ceres)
  target_compile_definitions(tianji_qp_ik PUBLIC TIANJI_HAS_CERES=1)
endif()

# Pure stdin/stdout absolute-TCP worker; no hardware/network authority.
add_executable(pico2_dls_worker apps/pico2_dls_worker.cpp)
target_link_libraries(pico2_dls_worker PRIVATE tianji_qp_ik)
tianji_enable_warnings(pico2_dls_worker)

# The inference package remains under src/, outside the teleop_outputs group.
set(TIANJI_INFERENCE_NATIVE "${CMAKE_CURRENT_LIST_DIR}/../../../../inference/mocap_policy_runtime/mocap_policy_runtime/native")
if(NOT EXISTS "${TIANJI_INFERENCE_NATIVE}/tcp_worker.cpp")
  message(FATAL_ERROR "mocap worker source not found at ${TIANJI_INFERENCE_NATIVE}/tcp_worker.cpp")
endif()
add_executable(mocap_tcp_worker "${TIANJI_INFERENCE_NATIVE}/tcp_worker.cpp")
target_link_libraries(mocap_tcp_worker PRIVATE tianji_qp_ik)
tianji_enable_warnings(mocap_tcp_worker)

add_executable(tianji_qp_ik_benchmark apps/benchmark_solver.cpp)
target_link_libraries(tianji_qp_ik_benchmark PRIVATE tianji_qp_ik)
tianji_enable_warnings(tianji_qp_ik_benchmark)

add_executable(tianji_hierarchical_ik_benchmark apps/benchmark_hierarchical_ik.cpp)
target_link_libraries(tianji_hierarchical_ik_benchmark PRIVATE tianji_qp_ik)
tianji_enable_warnings(tianji_hierarchical_ik_benchmark)

add_executable(tianji_cartesian_otg_benchmark apps/benchmark_cartesian_otg.cpp)
target_link_libraries(tianji_cartesian_otg_benchmark PRIVATE tianji_qp_ik)
tianji_enable_warnings(tianji_cartesian_otg_benchmark)

add_executable(tianji_cartesian_frf_benchmark apps/benchmark_cartesian_frf.cpp)
target_link_libraries(tianji_cartesian_frf_benchmark PRIVATE tianji_qp_ik)
tianji_enable_warnings(tianji_cartesian_frf_benchmark)

add_executable(tianji_qp_ik_viewer apps/run_qp_ik_viewer.cpp)
target_link_libraries(tianji_qp_ik_viewer PRIVATE tianji_qp_ik glfw Threads::Threads)
tianji_enable_warnings(tianji_qp_ik_viewer)

add_executable(tianji_inspect_model apps/inspect_model.cpp)
add_executable(tianji_record_action_trace apps/record_action_trace.cpp)
target_link_libraries(tianji_record_action_trace PRIVATE tianji_qp_ik)
tianji_enable_warnings(tianji_record_action_trace)
add_executable(tianji_shared_root_trace_audit apps/shared_root_trace_audit.cpp)
target_link_libraries(tianji_shared_root_trace_audit PRIVATE tianji_qp_ik)
tianji_enable_warnings(tianji_shared_root_trace_audit)
target_link_libraries(tianji_inspect_model PRIVATE mujoco::mujoco)
tianji_enable_warnings(tianji_inspect_model)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest CONFIG REQUIRED)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)

  # Python-driven ctest entries import the workspace's installed python packages
  # (tianji_description, tianji_runtime). The control environment is deliberately
  # ROS-free and does not source the overlay, so the site-packages directories are
  # collected here and passed explicitly.
  # This project installs into install/control, so the ament overlay is derived
  # from the workspace root rather than from CMAKE_INSTALL_PREFIX.
  get_filename_component(TIANJI_WORKSPACE
    "${CMAKE_CURRENT_LIST_DIR}/../../../../.." ABSOLUTE)
  file(GLOB TIANJI_PYTHON_SITE_PACKAGES
    "${TIANJI_WORKSPACE}/install/default/*/lib/python*/site-packages"
    "${TIANJI_WORKSPACE}/install/default/lib/python*/site-packages")
  if(NOT TIANJI_PYTHON_SITE_PACKAGES)
    message(WARNING
      "no installed python packages found under ${TIANJI_WORKSPACE}/install/default; "
      "python-driven ctest entries will fail until 'pixi run build-workspace' installs them")
  endif()
  string(REPLACE ";" ":" TIANJI_PYTHON_TESTS_PATH "${TIANJI_PYTHON_SITE_PACKAGES}")

  enable_testing()

  function(tianji_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE tianji_qp_ik GTest::gtest_main)
    target_compile_definitions(${name} PRIVATE
      TIANJI_PROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
      TIANJI_PROJECT_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}"
      # Models and the package URDFs live in the description package now.
      TIANJI_MODEL_DIR="${TIANJI_DESCRIPTION_MODELS}"
      TIANJI_DESCRIPTION_DIR="${TIANJI_DESCRIPTION_PACKAGE}"
    )
    tianji_enable_warnings(${name})
    add_test(NAME ${name} COMMAND ${name})
  endfunction()

  tianji_add_test(test_so3 tests/test_so3.cpp)
  tianji_add_test(test_ceres_trajectory_limiter tests/test_ceres_trajectory_limiter.cpp)
  tianji_add_test(test_simulation_recovery tests/test_simulation_recovery.cpp)
  tianji_add_test(test_pico_ee_franka_dls tests/test_pico_ee_franka_dls.cpp)
  tianji_add_test(test_franka_dls_controller tests/test_franka_dls_controller.cpp)
  add_test(NAME test_franka_dls_viewer COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_franka_dls_viewer.py $<TARGET_FILE:tianji_qp_ik_viewer>)
  if(TIANJI_ENABLE_CERES)
    tianji_add_test(test_pico_ee_franka_ceres_lm tests/test_pico_ee_franka_ceres_lm.cpp)
    tianji_add_test(test_ceres_controller tests/test_ceres_controller.cpp)
    add_test(NAME test_ceres_viewer COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ceres_viewer.py $<TARGET_FILE:tianji_qp_ik_viewer>)
  else()
    add_test(NAME test_ceres_unavailable COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ceres_viewer.py $<TARGET_FILE:tianji_qp_ik_viewer> --unavailable)
  endif()
  tianji_add_test(test_shared_root_geometry tests/test_shared_root_geometry.cpp)
  tianji_add_test(test_shared_root_closure tests/test_shared_root_closure.cpp)
  tianji_add_test(test_shared_root_coverage tests/test_shared_root_coverage.cpp)
  tianji_add_test(test_shared_root_input tests/test_shared_root_input.cpp)
  tianji_add_test(test_shared_root_morphology tests/test_shared_root_morphology.cpp)
  tianji_add_test(test_shared_root_target_builder tests/test_shared_root_target_builder.cpp)
  tianji_add_test(test_shared_root_continuity tests/test_shared_root_continuity.cpp)
  tianji_add_test(test_shared_root_options tests/test_shared_root_options.cpp)
  tianji_add_test(test_shared_root_pipeline tests/test_shared_root_pipeline.cpp)
  tianji_add_test(test_shared_root_guidance tests/test_shared_root_guidance.cpp)
  tianji_add_test(test_cartesian_servo tests/test_cartesian_servo.cpp)
  tianji_add_test(test_cartesian_acceleration_servo tests/test_cartesian_acceleration_servo.cpp)
  tianji_add_test(test_cartesian_otg tests/test_cartesian_otg.cpp)
  tianji_add_test(test_cartesian_frf tests/test_cartesian_frf.cpp)
  tianji_add_test(test_config tests/test_config.cpp)
  tianji_add_test(test_spark_upper_retarget tests/test_spark_upper_retarget.cpp)
  tianji_add_test(test_velocity_ik tests/test_velocity_ik.cpp)
  tianji_add_test(test_acceleration_bounds tests/test_acceleration_bounds.cpp)
  tianji_add_test(test_iterative_pose_dls tests/test_iterative_pose_dls.cpp)
  tianji_add_test(test_joint_trajectory_limiter tests/test_joint_trajectory_limiter.cpp)
  tianji_add_test(test_joint_command tests/test_joint_command.cpp)
  foreach(command_mode IN ITEMS finite continuous)
    set(command_mode_args)
    if(command_mode STREQUAL "continuous")
      list(APPEND command_mode_args --continuous)
    endif()
    add_test(
      NAME joint_command_viewer_${command_mode}
      COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_joint_command_viewer.py
        --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
        --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
        --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_wuji2.xml
        ${command_mode_args}
    )
    set_tests_properties(joint_command_viewer_${command_mode} PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 25
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endforeach()
  tianji_add_test(test_nullspace_dls tests/test_nullspace_dls.cpp)
  tianji_add_test(test_mujoco_robot tests/test_mujoco_robot.cpp)
  target_link_libraries(test_mujoco_robot PRIVATE mujoco::mujoco)
  tianji_add_test(test_pinocchio_arm_kinematics
                  tests/test_pinocchio_arm_kinematics.cpp)
  target_link_libraries(test_pinocchio_arm_kinematics PRIVATE
                        pinocchio::pinocchio)
  tianji_add_test(test_jacobian tests/test_jacobian.cpp)
  target_link_libraries(test_jacobian PRIVATE mujoco::mujoco)
  tianji_add_test(test_jdot_qdot tests/test_jdot_qdot.cpp)
  target_link_libraries(test_jdot_qdot PRIVATE mujoco::mujoco)
  tianji_add_test(test_qp_builder tests/test_qp_builder.cpp)
  tianji_add_test(test_hierarchical_qp tests/test_hierarchical_qp.cpp)
  tianji_add_test(test_acceleration_qp tests/test_acceleration_qp.cpp)
  tianji_add_test(test_acceleration_qpoases_solver tests/test_acceleration_qpoases_solver.cpp)
  target_link_libraries(test_acceleration_qpoases_solver PRIVATE qpOASES::qpOASES)
  tianji_add_test(test_hierarchical_qpoases_solver tests/test_hierarchical_qpoases_solver.cpp)
  target_link_libraries(test_hierarchical_qpoases_solver PRIVATE qpOASES::qpOASES)
  tianji_add_test(test_safety tests/test_safety.cpp)
  tianji_add_test(test_osqp_solver tests/test_osqp_solver.cpp)
  target_link_libraries(test_osqp_solver PRIVATE osqp::osqp)
  tianji_add_test(test_qpoases_solver tests/test_qpoases_solver.cpp)
  target_link_libraries(test_qpoases_solver PRIVATE qpOASES::qpOASES)
  tianji_add_test(test_equality_constrained_qpoases_solver
                  tests/test_equality_constrained_qpoases_solver.cpp)
  target_link_libraries(test_equality_constrained_qpoases_solver
                        PRIVATE qpOASES::qpOASES)
  tianji_add_test(test_solver_crosscheck tests/test_solver_crosscheck.cpp)
  target_link_libraries(test_solver_crosscheck PRIVATE osqp::osqp qpOASES::qpOASES)
  tianji_add_test(test_target_manager tests/test_target_manager.cpp)
  tianji_add_test(test_controller tests/test_controller.cpp)
  tianji_add_test(test_hierarchical_controller tests/test_hierarchical_controller.cpp)
  tianji_add_test(test_otg_controller tests/test_otg_controller.cpp)
  tianji_add_test(test_acceleration_controller tests/test_acceleration_controller.cpp)
  tianji_add_test(test_cartesian_otg_benchmark tests/test_cartesian_otg_benchmark.cpp)
  add_dependencies(test_cartesian_otg_benchmark tianji_cartesian_otg_benchmark)
  tianji_add_test(test_trajectory_regression tests/test_trajectory_regression.cpp)
  tianji_add_test(test_benchmark_dataset tests/test_benchmark_dataset.cpp)
  tianji_add_test(test_snapshot_exchange tests/test_snapshot_exchange.cpp)
  tianji_add_test(test_joint_kinematics_plot tests/test_joint_kinematics_plot.cpp)
  tianji_add_test(test_mujoco_joint_plot tests/test_mujoco_joint_plot.cpp)
  target_link_libraries(test_mujoco_joint_plot PRIVATE mujoco::mujoco)
  tianji_add_test(test_viewer_shortcut_binding tests/test_viewer_shortcut_binding.cpp)
  target_link_libraries(test_viewer_shortcut_binding PRIVATE mujoco::mujoco)
  tianji_add_test(test_pico_teleop_protocol tests/test_pico_teleop_protocol.cpp)
  tianji_add_test(test_wuji_hand_teleop_protocol
                  tests/test_wuji_hand_teleop_protocol.cpp)
  tianji_add_test(test_wuji_hand_udp_receiver
                  tests/test_wuji_hand_udp_receiver.cpp)
  tianji_add_test(test_pico_skeleton_overlay tests/test_pico_skeleton_overlay.cpp)
  tianji_add_test(test_spark_upper_qpoases_ik
                  tests/test_spark_upper_qpoases_ik.cpp)
  tianji_add_test(test_spark_posture_reference
                  tests/test_spark_posture_reference.cpp)
  tianji_add_test(test_spark_feedforward_reference
                  tests/test_spark_feedforward_reference.cpp)
  tianji_add_test(test_spark_palm_twist_estimator
                  tests/test_spark_palm_twist_estimator.cpp)
  tianji_add_test(test_spark_constraint_headroom
                  tests/test_spark_constraint_headroom.cpp)
  tianji_add_test(test_spark_guidance tests/test_spark_guidance.cpp)
  tianji_add_test(test_spark_qpoases_diagnostic
                  tests/test_spark_qpoases_diagnostic.cpp)
  tianji_add_test(test_arm_angle tests/test_arm_angle.cpp)
  tianji_add_test(test_upper_arm_outward tests/test_upper_arm_outward.cpp)
  tianji_add_test(test_pico_udp_receiver tests/test_pico_udp_receiver.cpp)
  tianji_add_test(test_pico_trace_recorder tests/test_pico_trace_recorder.cpp)
  tianji_add_test(test_pico_teleop_session tests/test_pico_teleop_session.cpp)
  tianji_add_test(test_interactive_marker tests/test_interactive_marker.cpp)
  tianji_add_test(test_mujoco_marker_adapter tests/test_mujoco_marker_adapter.cpp)
  target_link_libraries(test_mujoco_marker_adapter PRIVATE mujoco::mujoco)
  add_test(
    NAME compare_spark_guided_velocity_qp_analysis
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_compare_spark_guided_velocity_qp.py
  )
  add_test(NAME cartesian_frf_analysis_unit
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE} -m unittest
      test_analyze_cartesian_frf test_run_cartesian_frf_benchmark -v)
  set_tests_properties(cartesian_frf_analysis_unit PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  add_test(NAME pico_trace_algorithm_benchmark_runner_unit
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE} -m unittest
      test_run_pico_trace_algorithm_benchmark -v)
  set_tests_properties(pico_trace_algorithm_benchmark_runner_unit PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  add_test(NAME pico_trace_algorithm_benchmark_analysis_unit
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE} -m unittest
      test_analyze_pico_trace_algorithm_benchmark -v)
  set_tests_properties(pico_trace_algorithm_benchmark_analysis_unit PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  add_test(NAME cartesian_frf_runner_integration
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cartesian_frf_runner.py
      --binary $<TARGET_FILE:tianji_cartesian_frf_benchmark>)
  set_tests_properties(cartesian_frf_runner_integration PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} TIMEOUT 30)
  add_test(
    NAME compare_spark_three_way_following_analysis
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_compare_spark_three_way_following.py
  )
  add_test(
    NAME wuji_hand_viewer_integration
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_wuji_hand_viewer.py
      --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
  )
  set_tests_properties(
    wuji_hand_viewer_integration
    PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 15
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  foreach(pico_control_level IN ITEMS velocity acceleration)
    add_test(
      NAME pico_viewer_integration_${pico_control_level}
      COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
        --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
        --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
        --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
        --control-level ${pico_control_level}
        --algorithm hierarchical_qp
    )
    set_tests_properties(
      pico_viewer_integration_${pico_control_level}
      PROPERTIES
        RUN_SERIAL TRUE
        TIMEOUT 15
    )
  endforeach()
  add_test(
    NAME pico_viewer_button_pause_resume
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
      --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
      --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      --control-level velocity
      --algorithm spark_upper_qpoases_headroom_feedforward_velocity_qp
      --model-state-only
      --button-state-test
  )
  set_tests_properties(
    pico_viewer_button_pause_resume
    PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 20
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_test(
    NAME pico_viewer_button_hand_pause
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
      --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
      --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_wuji2.xml
      --control-level velocity
      --algorithm hierarchical_qp
      --button-hand-pause-test
  )
  set_tests_properties(
    pico_viewer_button_hand_pause
    PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 15
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_test(
    NAME pico_viewer_native_trace_recording
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
      --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
      --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      --control-level velocity
      --algorithm hierarchical_qp
      --assert-recording
      --inject-resynchronization
  )
  set_tests_properties(
    pico_viewer_native_trace_recording
    PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 15
  )
  foreach(spark_qpoases_variant IN ITEMS direct velocity_qp
                                         cartesian_otg_velocity_qp
                                         feedforward_velocity_qp
                                         headroom_feedforward_velocity_qp)
    set(spark_test_extra_args)
    if(NOT spark_qpoases_variant STREQUAL "direct")
      set(spark_test_extra_args
          --viewer-duration 3.0
          --send-duration 2.4)
    endif()
    if(spark_qpoases_variant STREQUAL "cartesian_otg_velocity_qp" OR
       spark_qpoases_variant STREQUAL "feedforward_velocity_qp")
      # These tests require Cartesian motion after joint-space takeover, whose
      # Cartesian twist is deliberately zero. Allow the bounded alignment from
      # the initial posture (up to 3.62 rad at 0.8 rad/s, plus ramps), then a
      # full motion period before exercising stale-source braking.
      set(spark_test_extra_args --send-duration 8.0)
    endif()
    add_test(
      NAME pico_viewer_integration_spark_upper_qpoases_${spark_qpoases_variant}
      COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
        --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
        --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
        --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
        --control-level velocity
        --algorithm spark_upper_qpoases_${spark_qpoases_variant}
        --model-state-only
        ${spark_test_extra_args}
    )
    set_tests_properties(
      pico_viewer_integration_spark_upper_qpoases_${spark_qpoases_variant}
      PROPERTIES
        RUN_SERIAL TRUE
        TIMEOUT 20
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endforeach()
  add_test(
    NAME pico_viewer_integration_spark_guided_velocity_qp
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
      --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
      --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      --control-level velocity
      --algorithm spark_guided_velocity_qp
      --model-state-only
  )
  set_tests_properties(
    pico_viewer_integration_spark_guided_velocity_qp
    PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 20
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  foreach(spark_variant IN ITEMS direct pose)
    add_test(
      NAME pico_viewer_integration_spark_${spark_variant}_velocity_qp
      COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
        --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
        --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
        --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
        --control-level velocity
        --algorithm spark_${spark_variant}_velocity_qp
        --model-state-only
    )
    set_tests_properties(
      pico_viewer_integration_spark_${spark_variant}_velocity_qp
      PROPERTIES
        RUN_SERIAL TRUE
        TIMEOUT 20
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endforeach()
  add_test(
    NAME pico_viewer_integration_default_profile
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
      --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_cartesian_otg_acceleration.yaml
      --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      --control-level acceleration
  )
  set_tests_properties(
    pico_viewer_integration_default_profile
    PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 15
  )
  add_test(
    NAME pico_viewer_integration_resynchronization
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
      --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
      --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      --control-level acceleration
      --algorithm hierarchical_qp
      --inject-resynchronization
  )
  set_tests_properties(
    pico_viewer_integration_resynchronization
    PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 15
  )
  add_test(
    NAME pico_viewer_monitor_no_input
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
      --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
      --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      --control-level acceleration
      --algorithm hierarchical_qp
      --viewer-duration 0.4
      --no-send
  )
  set_tests_properties(
    pico_viewer_monitor_no_input
    PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 10
  )
  add_test(
    NAME pico_viewer_forces_outward_only
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_viewer_integration.py
      --viewer $<TARGET_FILE:tianji_qp_ik_viewer>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_pico_teleop.yaml
      --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      --control-level velocity
      --algorithm hierarchical_qp
      --viewer-duration 0.4
      --no-send
      --requested-arm-angle-mode pico_outward
  )
  set_tests_properties(
    pico_viewer_forces_outward_only
    PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT 10
  )
  add_test(
    NAME test_hierarchical_ik_benchmark
    COMMAND tianji_hierarchical_ik_benchmark
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_hierarchical.yaml
      --model ${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      --steps 600
      --output ${CMAKE_CURRENT_BINARY_DIR}/tianji_hierarchical_ik_benchmark_test.csv
  )
  add_test(
    NAME test_viewer_default_profile
    COMMAND ${CMAKE_COMMAND}
      -DVIEWER=$<TARGET_FILE:tianji_qp_ik_viewer>
      -DEXPECTED_CONTROL_LEVEL=velocity
      -DEXPECTED_ALGORITHM=spark_upper_qpoases_headroom_feedforward_velocity_qp
      -DEXPECTED_STATE_SOURCE=model_reference
      -DEXPECTED_PICO_BIND=127.0.0.1:15000
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/assert_viewer_config.cmake
  )
  set_tests_properties(test_viewer_default_profile PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESOURCE_LOCK pico_udp_15000
    RUN_SERIAL TRUE
    TIMEOUT 15
  )
  add_test(
    NAME test_viewer_pico_profile
    COMMAND ${CMAKE_COMMAND}
      -DVIEWER=$<TARGET_FILE:tianji_qp_ik_viewer>
      -DCONFIG=config/qp_ik_pico_teleop.yaml
      -DEXPECTED_CONTROL_LEVEL=velocity
      -DEXPECTED_ALGORITHM=spark_upper_qpoases_headroom_feedforward_velocity_qp
      -DEXPECTED_STATE_SOURCE=model_reference
      -DENABLE_PICO=ON
      -DMODEL=${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/assert_viewer_config.cmake
  )
  set_tests_properties(test_viewer_pico_profile PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESOURCE_LOCK pico_udp_15000
  )
  add_test(
    NAME test_viewer_joint_plot_stream
    COMMAND ${CMAKE_COMMAND}
      -DVIEWER=$<TARGET_FILE:tianji_qp_ik_viewer>
      -DCONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_hierarchical.yaml
      -DMODEL=${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      -DWORKING_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/assert_viewer_joint_plot.cmake
  )
  set_tests_properties(test_viewer_joint_plot_stream PROPERTIES
    RUN_SERIAL TRUE
    TIMEOUT 15
  )
  add_test(
    NAME test_viewer_joint_telemetry_stream
    COMMAND ${CMAKE_COMMAND}
      -DVIEWER=$<TARGET_FILE:tianji_qp_ik_viewer>
      -DCONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/qp_ik_hierarchical.yaml
      -DMODEL=${TIANJI_DESCRIPTION_MODELS}/marvin_m6_qp_test.xml
      -DWORKING_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR}
      -DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/viewer_joint_telemetry_test.csv
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/assert_viewer_joint_telemetry.cmake
  )
  set_tests_properties(test_viewer_joint_telemetry_stream PROPERTIES
    RUN_SERIAL TRUE
    TIMEOUT 15
  )
endif()

# ---------------------------------------------------------------------------
# Install rules.
#
# Every non-test executable is installed, not just the main viewer: the offline
# recording/audit tools and workers are real operator entry points, and
# `tianji_runtime.native_executable` resolves them from this prefix.
# ---------------------------------------------------------------------------
set(TIANJI_NATIVE_TARGETS
  tianji_qp_ik_viewer
  mocap_tcp_worker
  pico2_dls_worker
  tianji_inspect_model
  tianji_record_action_trace
  tianji_shared_root_trace_audit
  tianji_qp_ik_benchmark
  tianji_hierarchical_ik_benchmark
  tianji_cartesian_otg_benchmark
  tianji_cartesian_frf_benchmark
)
install(TARGETS ${TIANJI_NATIVE_TARGETS}
  RUNTIME DESTINATION bin
)

# Native controller profiles. Shared Home/model resources are installed by
# tianji_description and resolved through the workspace resource authority.
install(DIRECTORY config/
  DESTINATION share/tianji_controller/config
  FILES_MATCHING PATTERN "*.yaml"
)
