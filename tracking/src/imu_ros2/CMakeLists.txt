cmake_minimum_required(VERSION 3.8)
project(imu_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(im948_protocol
  src/im948_protocol.cpp
  src/orientation_reference.cpp)
target_compile_features(im948_protocol PUBLIC cxx_std_17)
target_include_directories(im948_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(imu_node src/imu_node.cpp)
target_compile_features(imu_node PUBLIC cxx_std_17)
target_link_libraries(imu_node im948_protocol)
ament_target_dependencies(imu_node rclcpp sensor_msgs std_msgs std_srvs)

add_executable(imu_multi_node src/imu_multi_node.cpp)
target_compile_features(imu_multi_node PUBLIC cxx_std_17)
target_link_libraries(imu_multi_node im948_protocol)
ament_target_dependencies(imu_multi_node rclcpp sensor_msgs std_msgs std_srvs)

if(BUILD_TESTING)
  include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/TianjiGTest.cmake")
  tianji_add_gtest(test_im948_protocol test/test_im948_protocol.cpp)
  target_link_libraries(test_im948_protocol im948_protocol)
endif()

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS im948_protocol
  EXPORT export_im948_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS imu_node imu_multi_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_im948_protocol HAS_LIBRARY_TARGET)
ament_package()
