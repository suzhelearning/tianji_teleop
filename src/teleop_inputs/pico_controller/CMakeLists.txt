cmake_minimum_required(VERSION 3.8)
project(pico_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BleFrame.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

add_executable(pico_bridge_node src/pico_bridge_node.cpp)
add_library(pico_tracking_epoch_store STATIC src/tracking_epoch_store.cpp)
target_include_directories(pico_tracking_epoch_store PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_executable(pico_smpl_ground_node src/pico_smpl_ground_node.cpp)
set_target_properties(pico_smpl_ground_node PROPERTIES OUTPUT_NAME pico_smpl_ground)
target_include_directories(pico_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(pico_smpl_ground_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(pico_bridge_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  builtin_interfaces
)
ament_target_dependencies(pico_smpl_ground_node
  rclcpp std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(pico_bridge_node "${cpp_typesupport_target}")
target_link_libraries(pico_bridge_node pico_tracking_epoch_store)

install(TARGETS pico_bridge_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS pico_smpl_ground_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/smpl_mujoco_visualizer.py DESTINATION lib/${PROJECT_NAME} RENAME smpl_mujoco_visualizer)
install(PROGRAMS scripts/pico_palm_tcp_calibrator.py DESTINATION lib/${PROJECT_NAME} RENAME pico_palm_tcp_calibrator)
install(PROGRAMS scripts/pico_palm_wrist_calibrator.py DESTINATION lib/${PROJECT_NAME} RENAME pico_palm_wrist_calibrator)
install(PROGRAMS scripts/pico_palm_tcp_runtime.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_palm_orientation_core.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_palm_orientation_client.py DESTINATION lib/${PROJECT_NAME} RENAME pico_palm_orientation_client)
install(PROGRAMS scripts/pico_palm_tcp_publisher.py DESTINATION lib/${PROJECT_NAME} RENAME pico_palm_tcp_publisher)
install(PROGRAMS scripts/pico_palm_skeleton_filter_core.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_calibration_artifact.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/pico_symmetric_geometry.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/pico_simple_artifact.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_palm_skeleton_filter_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_palm_skeleton_filter_node.py DESTINATION lib/${PROJECT_NAME} RENAME pico_palm_skeleton_filter)
install(PROGRAMS scripts/pico_arm_geometry_core.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_left_arm_geometry_core.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_arm_geometry_calibrator.py
  DESTINATION lib/${PROJECT_NAME} RENAME pico_arm_geometry_calibrator)
install(PROGRAMS scripts/pico_left_arm_geometry_calibrator.py DESTINATION lib/${PROJECT_NAME} RENAME pico_left_arm_geometry_calibrator)
install(PROGRAMS scripts/pico_arm_geometry_runtime.sh DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/start_pico_m0_runtime.sh
  scripts/setup_pico.py
  scripts/calibrate_pico_simple.py
  scripts/ensure_pico_user.py
  scripts/check_pico_session_ready.py
  scripts/cleanup_tianji_pico_processes.py
  scripts/pico_symmetric_profile.py
  scripts/pico_m0_readiness.py
  scripts/pico_process_cleanup.sh
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_m0_comparison_report.py
  DESTINATION lib/${PROJECT_NAME} RENAME pico_m0_comparison_report)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES
  test/test_smpl_mujoco_geometry.py
  test/test_pico_palm_tcp_calibrator.py
  test/test_pico_palm_tcp_runtime.py
  test/test_pico_palm_orientation_core.py
  test/test_pico_palm_orientation_service.py
  test/test_pico_palm_wrist_calibrator.py
  test/test_pico_calibration_artifact.py
  test/test_pico_calibration_menu.py
  test/test_pico_palm_skeleton_filter_core.py
  test/test_pico_palm_skeleton_filter_node.py
  test/test_pico_left_arm_geometry_core.py
  test/test_pico_left_arm_geometry_calibrator.py
  DESTINATION share/${PROJECT_NAME}/test
)

if(BUILD_TESTING)
  include("${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/TianjiGTest.cmake")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_setup_pico test/test_setup_pico.py)
  ament_add_pytest_test(test_pico_simple_lifecycle test/test_pico_simple_lifecycle.py)
  ament_add_pytest_test(test_pico_simple_ros_loaders test/test_pico_simple_ros_loaders.py)
  ament_add_pytest_test(test_sim_user test/test_sim_user.py)
  ament_add_pytest_test(test_pico_startup_safety test/test_pico_startup_safety.py)
  tianji_add_gtest(test_pico_frame test/test_pico_frame.cpp)
  tianji_add_gtest(test_smpl_ground_alignment test/test_smpl_ground_alignment.cpp)
  tianji_add_gtest(test_tracking_epoch_store test/test_tracking_epoch_store.cpp)
  target_include_directories(test_tracking_epoch_store PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_tracking_epoch_store pico_tracking_epoch_store)
  target_include_directories(test_pico_frame PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_include_directories(test_smpl_ground_alignment PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_add_pytest_test(test_smpl_mujoco_geometry test/test_smpl_mujoco_geometry.py)
  ament_add_pytest_test(test_pico_palm_tcp_calibrator test/test_pico_palm_tcp_calibrator.py)
  ament_add_pytest_test(test_pico_palm_wrist_calibrator test/test_pico_palm_wrist_calibrator.py)
  ament_add_pytest_test(test_pico_calibration_artifact test/test_pico_calibration_artifact.py)
  ament_add_pytest_test(test_pico_calibration_menu test/test_pico_calibration_menu.py)
  ament_add_pytest_test(test_pico_palm_tcp_runtime test/test_pico_palm_tcp_runtime.py)
  ament_add_pytest_test(test_pico_palm_orientation_core test/test_pico_palm_orientation_core.py)
  ament_add_pytest_test(test_pico_palm_orientation_service test/test_pico_palm_orientation_service.py)
  ament_add_pytest_test(test_pico_palm_skeleton_filter_core test/test_pico_palm_skeleton_filter_core.py)
  ament_add_pytest_test(test_pico_palm_skeleton_filter_node test/test_pico_palm_skeleton_filter_node.py)
  ament_add_pytest_test(test_pico_arm_geometry_core test/test_pico_arm_geometry_core.py)
  ament_add_pytest_test(test_pico_left_arm_geometry_core test/test_pico_left_arm_geometry_core.py)
  ament_add_pytest_test(test_pico_arm_geometry_calibrator test/test_pico_arm_geometry_calibrator.py)
  ament_add_pytest_test(test_pico_left_arm_geometry_calibrator test/test_pico_left_arm_geometry_calibrator.py)
  ament_add_pytest_test(test_pico_m0_scripts test/test_pico_m0_scripts.py)
  ament_add_pytest_test(test_pico_m0_readiness test/test_pico_m0_readiness.py)
  ament_add_pytest_test(test_pico_m0_comparison_report test/test_pico_m0_comparison_report.py)
  ament_add_pytest_test(test_launch_integration test/test_launch_integration.py)
  ament_add_pytest_test(test_smpl_ground_integration test/test_smpl_ground_integration.py)
endif()

ament_package()
