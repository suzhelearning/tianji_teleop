cmake_minimum_required(VERSION 3.8)
project(pico_odin)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(pico_odin_core
  src/extrinsics_file.cpp
  src/extrinsic_solver.cpp
  src/host_time_synchronizer.cpp
  src/runtime_alignment.cpp
)
target_include_directories(pico_odin_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pico_odin_core PUBLIC Eigen3::Eigen yaml-cpp)
ament_target_dependencies(pico_odin_core PUBLIC geometry_msgs)

add_executable(odin_pelvis_calibrator src/odin_pelvis_calibrator_node.cpp)
target_link_libraries(odin_pelvis_calibrator pico_odin_core)
ament_target_dependencies(odin_pelvis_calibrator
  rclcpp geometry_msgs nav_msgs std_msgs
)

add_executable(odin_pelvis_runtime src/odin_pelvis_runtime_node.cpp)
target_link_libraries(odin_pelvis_runtime pico_odin_core)
ament_target_dependencies(odin_pelvis_runtime
  rclcpp geometry_msgs nav_msgs std_msgs
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS pico_odin_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS odin_pelvis_calibrator odin_pelvis_runtime
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3 geometry_msgs nav_msgs rclcpp std_msgs)

if(BUILD_TESTING)
  include("${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/TianjiGTest.cmake")
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_launch_integration
    test/test_launch_integration.py
  )
  ament_add_pytest_test(test_runtime_ros_integration
    test/test_runtime_ros_integration.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_process_startup
    test/test_process_startup.py
    TIMEOUT 15
  )
  ament_add_pytest_test(test_calibrator_ros_integration
    test/test_calibrator_ros_integration.py
    TIMEOUT 20
  )

  tianji_add_gtest(test_se3 test/test_se3.cpp)
  target_include_directories(test_se3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_se3 Eigen3::Eigen)
  ament_target_dependencies(test_se3 geometry_msgs)

  tianji_add_gtest(test_extrinsics_file test/test_extrinsics_file.cpp)
  target_include_directories(test_extrinsics_file PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_extrinsics_file pico_odin_core)
  ament_target_dependencies(test_extrinsics_file geometry_msgs)

  tianji_add_gtest(test_extrinsic_solver test/test_extrinsic_solver.cpp)
  target_include_directories(test_extrinsic_solver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_extrinsic_solver pico_odin_core)
  ament_target_dependencies(test_extrinsic_solver geometry_msgs)

  tianji_add_gtest(test_calibration_session test/test_calibration_session.cpp)
  target_include_directories(test_calibration_session PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_calibration_session pico_odin_core)
  ament_target_dependencies(test_calibration_session geometry_msgs)

  tianji_add_gtest(test_runtime_alignment test/test_runtime_alignment.cpp)
  target_include_directories(test_runtime_alignment PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_runtime_alignment pico_odin_core)
  ament_target_dependencies(test_runtime_alignment geometry_msgs)

  tianji_add_gtest(test_pose_stream_continuity test/test_pose_stream_continuity.cpp)
  target_include_directories(test_pose_stream_continuity PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_pose_stream_continuity pico_odin_core)
  ament_target_dependencies(test_pose_stream_continuity geometry_msgs)

  tianji_add_gtest(test_pose_stationarity test/test_pose_stationarity.cpp)
  target_include_directories(test_pose_stationarity PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_pose_stationarity pico_odin_core)
  ament_target_dependencies(test_pose_stationarity geometry_msgs)

  tianji_add_gtest(test_host_time_synchronizer test/test_host_time_synchronizer.cpp)
  target_include_directories(test_host_time_synchronizer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_host_time_synchronizer pico_odin_core)
  ament_target_dependencies(test_host_time_synchronizer geometry_msgs)
endif()

ament_package()
