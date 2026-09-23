cmake_minimum_required(VERSION 3.16)
project(tianji_cmd_pub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tianji_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Shared gtest wrapper, kept at the workspace root because several packages use it.
include("${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake/TianjiGTest.cmake")

# Retargeting geometry and the TJVR v4 wire format. The static libraries keep
# their original target and namespace names so the algorithms are unchanged.
add_library(tianji_teleop_geometry STATIC src/tianji_teleop_geometry.cpp)
target_include_directories(tianji_teleop_geometry PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(tianji_teleop_geometry PUBLIC Eigen3::Eigen)

add_library(tianji_teleop_protocol STATIC src/tianji_teleop_protocol.cpp)
target_include_directories(tianji_teleop_protocol PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(tianji_teleop_protocol PUBLIC
  Eigen3::Eigen yaml-cpp tianji_teleop_geometry
)

add_executable(pico_arm_input src/pico_arm_input_node.cpp)
target_include_directories(pico_arm_input PRIVATE include)
ament_target_dependencies(pico_arm_input rclcpp std_msgs geometry_msgs tianji_interfaces)
target_link_libraries(pico_arm_input tianji_teleop_protocol)

install(TARGETS pico_arm_input
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS tianji_teleop_geometry tianji_teleop_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  tianji_add_gtest(test_tianji_teleop_geometry test/test_tianji_teleop_geometry.cpp)
  target_include_directories(test_tianji_teleop_geometry PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_link_libraries(test_tianji_teleop_geometry Eigen3::Eigen tianji_teleop_geometry)

  tianji_add_gtest(test_tianji_teleop_protocol test/test_tianji_teleop_protocol.cpp)
  target_include_directories(test_tianji_teleop_protocol PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_link_libraries(test_tianji_teleop_protocol
    Eigen3::Eigen yaml-cpp tianji_teleop_geometry tianji_teleop_protocol
  )

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(pico_arm_input_runtime
    "test/test_pico_arm_input_runtime.py"
    TIMEOUT 300
  )
endif()

ament_package()
