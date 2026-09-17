cmake_minimum_required(VERSION 3.24)
project(pico2_v131_reference LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 3.4 CONFIG REQUIRED)
find_package(mujoco 3.10.0 EXACT CONFIG REQUIRED)
find_package(pinocchio CONFIG REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../control/cmake")
find_package(qpOASES CONFIG QUIET)
if(NOT qpOASES_FOUND)
  find_package(qpOASES MODULE REQUIRED)
endif()
if(NOT TARGET qpOASES::qpOASES)
  add_library(qpOASES::qpOASES UNKNOWN IMPORTED)
  set_target_properties(qpOASES::qpOASES PROPERTIES
    IMPORTED_LOCATION "${qpOASES_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${qpOASES_INCLUDE_DIR}")
endif()
# Same pinned 0.19.4 as the source; no download, no changes to old builds.
add_subdirectory("../../control/third_party/ruckig" ruckig)
add_library(pico2_v131 STATIC
  src/hierarchical_qp.cpp src/hierarchical_qpoases_solver.cpp
  src/velocity_ik.cpp src/full_joint_temporal_envelope.cpp
  src/pico_ee_v131_singularity.cpp src/pico_ee_v131_jacobian_svd.cpp
  src/so3.cpp src/velocity_qp.cpp src/mujoco_robot.cpp
  src/cartesian_otg.cpp src/cartesian_servo.cpp src/target_manager.cpp
  src/dexhand_target_conditioner.cpp)
target_include_directories(pico2_v131 PUBLIC include)
target_link_libraries(pico2_v131 PUBLIC Eigen3::Eigen mujoco::mujoco qpOASES::qpOASES ruckig::ruckig)
add_library(pico2_v131_adapter STATIC arm_ik.cpp)
target_link_libraries(pico2_v131_adapter PUBLIC pico2_v131 pinocchio::pinocchio)
add_executable(pico2_v131_worker worker.cpp)
target_link_libraries(pico2_v131_worker PRIVATE pico2_v131_adapter)
include(CTest)
if(BUILD_TESTING)
  add_executable(pico2_v131_probe src/probe.cpp)
  target_link_libraries(pico2_v131_probe PRIVATE pico2_v131)
  add_test(NAME pico2_v131_probe COMMAND pico2_v131_probe)
  add_executable(pico2_v131_model_probe model_probe.cpp)
  target_link_libraries(pico2_v131_model_probe PRIVATE pico2_v131)
  add_test(NAME pico2_v131_model_probe COMMAND pico2_v131_model_probe
    "${CMAKE_CURRENT_SOURCE_DIR}/models/marvin_m6_qp_pico_fast_kinematics.xml")
  add_executable(pico2_v131_model_trace model_trace.cpp)
  target_link_libraries(pico2_v131_model_trace PRIVATE pico2_v131_adapter)
  add_test(NAME pico2_v131_model_trace COMMAND pico2_v131_model_trace
    "${CMAKE_CURRENT_SOURCE_DIR}/models/marvin_m6_s_ccs_696_v4.urdf"
    "${CMAKE_CURRENT_SOURCE_DIR}/models/marvin_m6_qp_pico_fast_kinematics.xml")
endif()
