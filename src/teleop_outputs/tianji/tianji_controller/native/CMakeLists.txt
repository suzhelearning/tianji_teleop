cmake_minimum_required(VERSION 3.24)

project(tj_arm_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(TIANJI_BUILD_ROS "Build the isolated ROS arm transport executable" OFF)

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
find_package(yaml-cpp CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(Threads REQUIRED)

# Pinned offline trajectory generator; no external solver fetches.
add_subdirectory(third_party/ruckig)

function(tianji_enable_warnings target)
  target_compile_options(${target} PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wshadow>
  )
endfunction()

add_library(tianji_qp_ik STATIC
  src/ruckig_trajectory_limiter.cpp
  src/dls_pinocchio_arm_kinematics.cpp
  src/controller_dls.cpp
  src/franka_iterative_pose_dls.cpp
  src/pico_ee_franka_dls.cpp
  src/config.cpp
  src/episode_relative.cpp
  src/controller.cpp
  src/interactive_marker.cpp
  src/joint_kinematics_plot.cpp
  src/joint_command.cpp
  src/mujoco_marker_adapter.cpp
  src/mujoco_joint_plot.cpp
  src/mujoco_robot.cpp
  src/pinocchio_arm_kinematics.cpp
  src/pico_teleop_protocol.cpp
  src/pico_skeleton_overlay.cpp
  src/pico_teleop_session.cpp
  src/pico_trace_recorder.cpp
  src/pico_udp_receiver.cpp
  src/safety.cpp
  src/shared_root_retarget.cpp
  src/shared_root_input.cpp
  src/shared_root_morphology.cpp
  src/shared_root_target_builder.cpp
  src/shared_root_closure.cpp
  src/shared_root_continuity.cpp
  src/shared_root_options.cpp
  src/shared_root_sha256.cpp
  src/shared_root_pipeline.cpp
  src/shared_root_guidance.cpp
  src/so3.cpp
  src/target_manager.cpp
  src/telemetry.cpp
  src/wuji_hand_teleop_protocol.cpp
  src/wuji_hand_udp_receiver.cpp
)
target_include_directories(tianji_qp_ik PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(tianji_qp_ik PUBLIC
  Eigen3::Eigen yaml-cpp::yaml-cpp mujoco::mujoco pinocchio::pinocchio ruckig::ruckig
)
tianji_enable_warnings(tianji_qp_ik)

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

add_executable(tianji_qp_ik_viewer apps/run_qp_ik_viewer.cpp)
target_link_libraries(tianji_qp_ik_viewer PRIVATE tianji_qp_ik glfw Threads::Threads)
tianji_enable_warnings(tianji_qp_ik_viewer)

if(TIANJI_BUILD_ROS)
  get_filename_component(TIANJI_WORKSPACE
    "${CMAKE_CURRENT_LIST_DIR}/../../../../.." ABSOLUTE)
  set(TIANJI_ARM_ROS_PREFIX "${TIANJI_WORKSPACE}/.pixi/envs/arm-ros")
  set(TIANJI_ARM_ROS_INTERFACES "${TIANJI_WORKSPACE}/install/arm-ros/tianji_interfaces")
  # Do not accidentally combine ROS from default with control's solver ABI.
  file(REAL_PATH "$ENV{CONDA_PREFIX}" TIANJI_ACTIVE_PREFIX)
  file(REAL_PATH "${TIANJI_ARM_ROS_PREFIX}" TIANJI_REQUIRED_PREFIX)
  if(NOT TIANJI_ACTIVE_PREFIX STREQUAL TIANJI_REQUIRED_PREFIX)
    message(FATAL_ERROR "TIANJI_BUILD_ROS requires the isolated arm-ros Pixi environment")
  endif()
  file(REAL_PATH "${Eigen3_DIR}" TIANJI_EIGEN_CONFIG)
  string(FIND "${TIANJI_EIGEN_CONFIG}" "${TIANJI_REQUIRED_PREFIX}/" TIANJI_EIGEN_PREFIX)
  if(NOT TIANJI_EIGEN_PREFIX EQUAL 0 OR Eigen3_VERSION VERSION_GREATER_EQUAL 4)
    message(FATAL_ERROR "ROS arm core requires arm-ros Eigen >=3.4,<4, not another environment")
  endif()
  find_package(rclcpp REQUIRED)
  find_package(tianji_interfaces REQUIRED)
  file(REAL_PATH "${rclcpp_DIR}" TIANJI_RCLCPP_CONFIG)
  string(FIND "${TIANJI_RCLCPP_CONFIG}" "${TIANJI_REQUIRED_PREFIX}/" TIANJI_RCLCPP_PREFIX)
  file(REAL_PATH "${tianji_interfaces_DIR}" TIANJI_INTERFACES_CONFIG)
  file(REAL_PATH "${TIANJI_ARM_ROS_INTERFACES}" TIANJI_REQUIRED_INTERFACES)
  string(FIND "${TIANJI_INTERFACES_CONFIG}" "${TIANJI_REQUIRED_INTERFACES}/" TIANJI_INTERFACES_PREFIX)
  if(NOT TIANJI_RCLCPP_PREFIX EQUAL 0 OR NOT TIANJI_INTERFACES_PREFIX EQUAL 0)
    message(FATAL_ERROR "ROS arm core requires arm-ros rclcpp and install/arm-ros/tianji_interfaces")
  endif()
  add_executable(tianji_arm_ros apps/run_qp_ik_viewer.cpp src/arm_ros_transport.cpp)
  target_compile_definitions(tianji_arm_ros PRIVATE TIANJI_ROS_TRANSPORT=1)
  target_link_libraries(tianji_arm_ros PRIVATE tianji_qp_ik glfw Threads::Threads
    rclcpp::rclcpp tianji_interfaces::tianji_interfaces__rosidl_typesupport_cpp)
  set_target_properties(tianji_arm_ros PROPERTIES
    BUILD_RPATH "${TIANJI_ARM_ROS_PREFIX}/lib;${TIANJI_ARM_ROS_INTERFACES}/lib"
    INSTALL_RPATH "$ORIGIN/../../../.pixi/envs/arm-ros/lib;$ORIGIN/../../arm-ros/tianji_interfaces/lib"
    INSTALL_RPATH_USE_LINK_PATH FALSE)
  tianji_enable_warnings(tianji_arm_ros)
  install(TARGETS tianji_arm_ros RUNTIME DESTINATION bin COMPONENT arm_ros)
endif()

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
    set_tests_properties(${name} PROPERTIES ENVIRONMENT
      "TIANJI_WORKSPACE=${TIANJI_WORKSPACE};TIANJI_ENVIRONMENT=default")
  endfunction()

  tianji_add_test(test_so3 tests/test_so3.cpp)
  tianji_add_test(test_ruckig_trajectory_limiter tests/test_ruckig_trajectory_limiter.cpp)
  tianji_add_test(test_simulation_recovery tests/test_simulation_recovery.cpp)
  tianji_add_test(test_pico_ee_franka_dls tests/test_pico_ee_franka_dls.cpp)
  tianji_add_test(test_franka_dls_controller tests/test_franka_dls_controller.cpp)
  tianji_add_test(test_episode_relative tests/test_episode_relative.cpp)
  tianji_add_test(test_shared_root_geometry tests/test_shared_root_geometry.cpp)
  tianji_add_test(test_shared_root_closure tests/test_shared_root_closure.cpp)
  tianji_add_test(test_shared_root_input tests/test_shared_root_input.cpp)
  tianji_add_test(test_shared_root_morphology tests/test_shared_root_morphology.cpp)
  tianji_add_test(test_shared_root_target_builder tests/test_shared_root_target_builder.cpp)
  tianji_add_test(test_shared_root_continuity tests/test_shared_root_continuity.cpp)
  tianji_add_test(test_shared_root_options tests/test_shared_root_options.cpp)
  tianji_add_test(test_shared_root_pipeline tests/test_shared_root_pipeline.cpp)
  tianji_add_test(test_shared_root_guidance tests/test_shared_root_guidance.cpp)
  tianji_add_test(test_shared_root_retarget tests/test_shared_root_retarget.cpp)
  tianji_add_test(test_config tests/test_config.cpp)
  tianji_add_test(test_joint_command tests/test_joint_command.cpp)
  tianji_add_test(test_mujoco_robot tests/test_mujoco_robot.cpp)
  tianji_add_test(test_pinocchio_arm_kinematics tests/test_pinocchio_arm_kinematics.cpp)
  tianji_add_test(test_jacobian tests/test_jacobian.cpp)
  tianji_add_test(test_target_manager tests/test_target_manager.cpp)
  tianji_add_test(test_snapshot_exchange tests/test_snapshot_exchange.cpp)
  tianji_add_test(test_joint_kinematics_plot tests/test_joint_kinematics_plot.cpp)
  tianji_add_test(test_mujoco_joint_plot tests/test_mujoco_joint_plot.cpp)
  tianji_add_test(test_pico_teleop_protocol tests/test_pico_teleop_protocol.cpp)
  tianji_add_test(test_wuji_hand_teleop_protocol tests/test_wuji_hand_teleop_protocol.cpp)
  tianji_add_test(test_wuji_hand_udp_receiver tests/test_wuji_hand_udp_receiver.cpp)
  tianji_add_test(test_pico_skeleton_overlay tests/test_pico_skeleton_overlay.cpp)
  tianji_add_test(test_pico_udp_receiver tests/test_pico_udp_receiver.cpp)
  tianji_add_test(test_pico_trace_recorder tests/test_pico_trace_recorder.cpp)
  tianji_add_test(test_pico_teleop_session tests/test_pico_teleop_session.cpp)
  tianji_add_test(test_interactive_marker tests/test_interactive_marker.cpp)
  tianji_add_test(test_mujoco_marker_adapter tests/test_mujoco_marker_adapter.cpp)
  add_test(NAME test_franka_dls_viewer
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${TIANJI_PYTHON_TESTS_PATH}" ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_franka_dls_viewer.py $<TARGET_FILE:tianji_qp_ik_viewer>)
  set_tests_properties(test_franka_dls_viewer PROPERTIES RUN_SERIAL TRUE TIMEOUT 60
    ENVIRONMENT "TIANJI_WORKSPACE=${TIANJI_WORKSPACE};TIANJI_ENVIRONMENT=default")
endif()

install(TARGETS mocap_tcp_worker
  RUNTIME DESTINATION bin)
install(TARGETS pico2_dls_worker tianji_qp_ik_viewer
  RUNTIME DESTINATION bin COMPONENT spd)
install(DIRECTORY config/
  DESTINATION share/tianji_controller/config COMPONENT spd
  FILES_MATCHING PATTERN "*.yaml")
