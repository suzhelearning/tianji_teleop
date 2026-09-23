cmake_minimum_required(VERSION 3.24)

project(tj_arm_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Native workers retain the control ABI when launched from the SPD ROS environment.
set(CMAKE_BUILD_RPATH "$ENV{CONDA_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib;$ORIGIN/../../../.pixi/envs/control/lib")
add_link_options("-Wl,--disable-new-dtags")

# Models moved to the description package; the controller keeps only profiles.
set(TIANJI_DESCRIPTION_PACKAGE
  "${CMAKE_CURRENT_LIST_DIR}/../../tianji_description")
set(TIANJI_DESCRIPTION_MODELS "${TIANJI_DESCRIPTION_PACKAGE}/models")

# The SPD profile, its geometry artifacts and the palm TCP model are one frozen
# set: fail early rather than start a worker against a different model tree.
if(NOT EXISTS "${TIANJI_DESCRIPTION_MODELS}/marvin_m6_wuji2_shared_root_ceres.xml")
  message(FATAL_ERROR "shared-root models not found at ${TIANJI_DESCRIPTION_MODELS}")
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

# Shared-root DLS/Ruckig core: the worker and the display helper are its only
# consumers, so nothing here is a route other than the SPD simulation chain.
add_library(tianji_qp_ik STATIC
  src/ruckig_trajectory_limiter.cpp
  src/dls_pinocchio_arm_kinematics.cpp
  src/controller_dls.cpp
  src/franka_iterative_pose_dls.cpp
  src/pico_ee_franka_dls.cpp
  src/config.cpp
  src/controller.cpp
  src/joint_kinematics_plot.cpp
  src/mujoco_joint_plot.cpp
  src/mujoco_robot.cpp
  src/pinocchio_arm_kinematics.cpp
  src/pico_teleop_protocol.cpp
  src/safety.cpp
  src/shared_root_retarget.cpp
  src/shared_root_input.cpp
  src/shared_root_morphology.cpp
  src/shared_root_target_builder.cpp
  src/shared_root_closure.cpp
  src/shared_root_continuity.cpp
  src/shared_root_options.cpp
  src/shared_root_sha256.cpp
  src/so3.cpp
  src/target_manager.cpp
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

# Display-only helper: renders the parent's private stdin frames, no control.
add_executable(tianji_qp_ik_viewer apps/run_qp_ik_viewer.cpp)
target_link_libraries(tianji_qp_ik_viewer PRIVATE tianji_qp_ik glfw Threads::Threads)
tianji_enable_warnings(tianji_qp_ik_viewer)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest CONFIG REQUIRED)
  get_filename_component(TIANJI_WORKSPACE
    "${CMAKE_CURRENT_LIST_DIR}/../../../../.." ABSOLUTE)

  enable_testing()

  function(tianji_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE tianji_qp_ik GTest::gtest_main)
    target_compile_definitions(${name} PRIVATE
      TIANJI_PROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
      # Models and the package URDFs live in the description package now.
      TIANJI_MODEL_DIR="${TIANJI_DESCRIPTION_MODELS}"
    )
    tianji_enable_warnings(${name})
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES ENVIRONMENT
      "TIANJI_WORKSPACE=${TIANJI_WORKSPACE};TIANJI_ENVIRONMENT=spd")
  endfunction()

  tianji_add_test(test_so3 tests/test_so3.cpp)
  tianji_add_test(test_ruckig_trajectory_limiter tests/test_ruckig_trajectory_limiter.cpp)
  tianji_add_test(test_simulation_recovery tests/test_simulation_recovery.cpp)
  tianji_add_test(test_pico_ee_franka_dls tests/test_pico_ee_franka_dls.cpp)
  tianji_add_test(test_franka_dls_controller tests/test_franka_dls_controller.cpp)
  tianji_add_test(test_shared_root_geometry tests/test_shared_root_geometry.cpp)
  tianji_add_test(test_shared_root_closure tests/test_shared_root_closure.cpp)
  tianji_add_test(test_shared_root_input tests/test_shared_root_input.cpp)
  tianji_add_test(test_shared_root_morphology tests/test_shared_root_morphology.cpp)
  tianji_add_test(test_shared_root_target_builder tests/test_shared_root_target_builder.cpp)
  tianji_add_test(test_shared_root_retarget tests/test_shared_root_retarget.cpp)
  tianji_add_test(test_shared_root_continuity tests/test_shared_root_continuity.cpp)
  tianji_add_test(test_shared_root_options tests/test_shared_root_options.cpp)
  tianji_add_test(test_config tests/test_config.cpp)
  tianji_add_test(test_mujoco_robot tests/test_mujoco_robot.cpp)
  tianji_add_test(test_pinocchio_arm_kinematics tests/test_pinocchio_arm_kinematics.cpp)
  tianji_add_test(test_jacobian tests/test_jacobian.cpp)
  tianji_add_test(test_target_manager tests/test_target_manager.cpp)
  tianji_add_test(test_joint_kinematics_plot tests/test_joint_kinematics_plot.cpp)
  tianji_add_test(test_mujoco_joint_plot tests/test_mujoco_joint_plot.cpp)
  tianji_add_test(test_pico_teleop_protocol tests/test_pico_teleop_protocol.cpp)
endif()

install(TARGETS pico2_dls_worker tianji_qp_ik_viewer
  RUNTIME DESTINATION bin COMPONENT spd)
install(DIRECTORY config/
  DESTINATION share/tianji_controller/config COMPONENT spd
  FILES_MATCHING PATTERN "*.yaml")
