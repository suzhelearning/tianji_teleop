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
add_executable(tianji_mujoco_teleop_bridge src/tianji_mujoco_teleop_bridge_node.cpp)
add_library(pico_tracking_epoch_store STATIC src/tracking_epoch_store.cpp)
add_library(tianji_teleop_geometry STATIC src/tianji_teleop_geometry.cpp)
target_include_directories(tianji_teleop_geometry PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(tianji_teleop_geometry PUBLIC Eigen3::Eigen)
add_library(tianji_teleop_protocol STATIC src/tianji_teleop_protocol.cpp)
target_include_directories(tianji_teleop_protocol PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(tianji_teleop_protocol PUBLIC
  Eigen3::Eigen yaml-cpp tianji_teleop_geometry
)
target_include_directories(pico_tracking_epoch_store PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_executable(pico_smpl_ground_node src/pico_smpl_ground_node.cpp)
set_target_properties(pico_smpl_ground_node PROPERTIES OUTPUT_NAME pico_smpl_ground)
add_executable(pico_foot_imu_fusion_node src/pico_foot_imu_fusion_node.cpp)
set_target_properties(pico_foot_imu_fusion_node PROPERTIES OUTPUT_NAME pico_foot_imu_fusion)
target_include_directories(pico_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(tianji_mujoco_teleop_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(pico_foot_imu_fusion_node PRIVATE
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
ament_target_dependencies(tianji_mujoco_teleop_bridge
  rclcpp std_msgs geometry_msgs
)
target_link_libraries(tianji_mujoco_teleop_bridge tianji_teleop_protocol)
ament_target_dependencies(pico_foot_imu_fusion_node
  rclcpp sensor_msgs geometry_msgs std_srvs
)
ament_target_dependencies(pico_smpl_ground_node
  rclcpp std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(pico_bridge_node "${cpp_typesupport_target}")
target_link_libraries(pico_bridge_node pico_tracking_epoch_store)

install(TARGETS pico_bridge_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS tianji_mujoco_teleop_bridge DESTINATION lib/${PROJECT_NAME})
install(TARGETS pico_smpl_ground_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS pico_foot_imu_fusion_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS tianji_teleop_geometry DESTINATION lib)
install(TARGETS tianji_teleop_protocol DESTINATION lib)
install(PROGRAMS scripts/smpl_mujoco_visualizer.py DESTINATION lib/${PROJECT_NAME} RENAME smpl_mujoco_visualizer)
install(PROGRAMS scripts/pico_palm_tcp_calibrator.py DESTINATION lib/${PROJECT_NAME} RENAME pico_palm_tcp_calibrator)
install(PROGRAMS scripts/pico_palm_wrist_calibrator.py DESTINATION lib/${PROJECT_NAME} RENAME pico_palm_wrist_calibrator)
install(PROGRAMS scripts/pico_palm_tcp_runtime.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_palm_orientation_core.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_palm_orientation_client.py DESTINATION lib/${PROJECT_NAME} RENAME pico_palm_orientation_client)
install(PROGRAMS scripts/pico_palm_tcp_publisher.py DESTINATION lib/${PROJECT_NAME} RENAME pico_palm_tcp_publisher)
install(PROGRAMS scripts/pico_palm_skeleton_filter_core.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_calibration_artifact.py DESTINATION lib/${PROJECT_NAME})
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
  scripts/pico_m0_readiness.py
  scripts/pico_process_cleanup.sh
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pico_m0_comparison_report.py
  DESTINATION lib/${PROJECT_NAME} RENAME pico_m0_comparison_report)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
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
  include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/TianjiGTest.cmake")
  find_package(ament_cmake_pytest REQUIRED)
  tianji_add_gtest(test_pico_frame test/test_pico_frame.cpp)
  tianji_add_gtest(test_foot_imu_fusion_math test/test_foot_imu_fusion_math.cpp)
  tianji_add_gtest(test_foot_imu_input_state test/test_foot_imu_input_state.cpp)
  tianji_add_gtest(test_auto_calibration_state test/test_auto_calibration_state.cpp)
  tianji_add_gtest(test_smpl_ground_alignment test/test_smpl_ground_alignment.cpp)
  tianji_add_gtest(test_tracking_epoch_store test/test_tracking_epoch_store.cpp)
  tianji_add_gtest(test_tianji_teleop_geometry test/test_tianji_teleop_geometry.cpp)
  target_include_directories(test_tianji_teleop_geometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_tianji_teleop_geometry Eigen3::Eigen)
  target_link_libraries(test_tianji_teleop_geometry tianji_teleop_geometry)
  tianji_add_gtest(test_tianji_teleop_protocol test/test_tianji_teleop_protocol.cpp)
  target_include_directories(test_tianji_teleop_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_tianji_teleop_protocol
    Eigen3::Eigen yaml-cpp tianji_teleop_geometry tianji_teleop_protocol
  )
  target_include_directories(test_tracking_epoch_store PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_tracking_epoch_store pico_tracking_epoch_store)
  target_include_directories(test_auto_calibration_state PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_include_directories(test_pico_frame PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_include_directories(test_foot_imu_fusion_math PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_include_directories(test_foot_imu_input_state PRIVATE
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
  ament_add_pytest_test(
    test_tianji_mujoco_teleop_bridge_runtime
    test/test_tianji_mujoco_teleop_bridge_runtime.py
  )
  ament_add_pytest_test(test_smpl_ground_integration test/test_smpl_ground_integration.py)
  ament_add_pytest_test(test_imu900_foot_integration test/test_imu900_foot_integration.py)
endif()

ament_package()
