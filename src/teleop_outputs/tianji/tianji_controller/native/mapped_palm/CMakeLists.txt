cmake_minimum_required(VERSION 3.24)
project(tianji_mapped_palm_native LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_SKIP_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_RPATH "$ENV{CONDA_PREFIX}/lib")
# install/control/lib/mapped_palm -> workspace's isolated control dependencies.
set(CMAKE_INSTALL_RPATH "$ORIGIN/..;$ORIGIN/../../../../.pixi/envs/control/lib")
add_link_options("-Wl,--disable-new-dtags")
find_package(Eigen3 3.4 REQUIRED CONFIG)
find_package(mujoco 3.10.0 EXACT REQUIRED CONFIG)
find_package(pinocchio 3.9.0 EXACT REQUIRED CONFIG)
find_package(osqp REQUIRED CONFIG)
find_package(yaml-cpp 0.8.0 EXACT REQUIRED CONFIG)
find_path(QPOASES_INCLUDE qpOASES.hpp REQUIRED)
find_library(QPOASES_LIBRARY qpOASES REQUIRED)
# Same pinned 0.19.4 sources already shipped by the destination project.
# Build offline and independently; do not change the existing SPARK library.
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../third_party/ruckig"
                 "${CMAKE_CURRENT_BINARY_DIR}/ruckig")
# Preserve the original source closure; exclude GUI-only translation units.
file(GLOB CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
foreach(name interactive_marker joint_kinematics_plot mujoco_marker_adapter mujoco_joint_plot)
  list(REMOVE_ITEM CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/${name}.cpp")
endforeach()
add_library(mapped_palm_core STATIC ${CORE_SOURCES} bilateral_cycle.cpp)
target_include_directories(mapped_palm_core PUBLIC include "${QPOASES_INCLUDE}")
target_link_libraries(mapped_palm_core PUBLIC Eigen3::Eigen mujoco::mujoco
  pinocchio::pinocchio osqp::osqp yaml-cpp::yaml-cpp "${QPOASES_LIBRARY}" ruckig::ruckig)
target_compile_options(mapped_palm_core PRIVATE -Wall -Wextra -Wpedantic)
add_executable(mapped_palm_native_worker native_worker.cpp)
target_link_libraries(mapped_palm_native_worker PRIVATE mapped_palm_core)
find_package(Threads REQUIRED)
add_executable(mapped_palm_tjrc_controller tjrc_controller.cpp)
target_link_libraries(mapped_palm_tjrc_controller PRIVATE mapped_palm_core Threads::Threads)
include(CTest)
if(BUILD_TESTING)
  # Deployment config and assets are description resources; the native worker
  # reads them from the same place the installed profiles do.
  set(TIANJI_DESCRIPTION "${CMAKE_CURRENT_LIST_DIR}/../../../tianji_description")
  if(NOT EXISTS "${TIANJI_DESCRIPTION}/config/bandwidth.yaml")
    message(FATAL_ERROR "tianji_description config not found at ${TIANJI_DESCRIPTION}")
  endif()
  add_test(NAME mapped_palm_model_startup COMMAND mapped_palm_native_worker
    "${TIANJI_DESCRIPTION}/config/bandwidth.yaml"
    "${TIANJI_DESCRIPTION}/mapped_palm/assets/mapped_palm/marvin_m6_wuji2.xml"
    "${TIANJI_DESCRIPTION}/mapped_palm/assets/mapped_palm/marvin_m6_s_ccs_696_v4_local.urdf"
    --startup-handshake)
  set_tests_properties(mapped_palm_model_startup PROPERTIES TIMEOUT 20
    PASS_REGULAR_EXPRESSION "mapped_palm_worker_ready")
endif()

# ---------------------------------------------------------------------------
# Install rules.
#
# These workers retain their lib/mapped_palm install location; the RPATH above
# selects only the control environment, never the default Python/ROS ABI.
# ---------------------------------------------------------------------------
install(TARGETS mapped_palm_tjrc_controller mapped_palm_native_worker
  RUNTIME DESTINATION lib/mapped_palm
)
