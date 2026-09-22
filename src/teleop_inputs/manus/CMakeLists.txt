cmake_minimum_required(VERSION 3.16)
project(manus_bridge LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tianji_interfaces REQUIRED)
find_package(Threads REQUIRED)

# One SDK bundle in the workspace. An alternate provisioned SDK root can be
# selected explicitly without modifying source or relying on the current cwd.
get_filename_component(_default_sdk_root
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/manus_sdk" ABSOLUTE)
set(MANUS_SDK_ROOT "${_default_sdk_root}" CACHE PATH "Provisioned MANUS Integrated SDK root")
foreach(_sdk_file include/ManusSDK.h include/ManusSDKTypes.h
    include/ManusSDKTypeInitializers.h lib/libManusSDK_Integrated.so)
  if(NOT EXISTS "${MANUS_SDK_ROOT}/${_sdk_file}")
    message(FATAL_ERROR "MANUS SDK missing ${MANUS_SDK_ROOT}/${_sdk_file}; provision vendor/manus_sdk or set MANUS_SDK_ROOT")
  endif()
endforeach()
# Fail at configure time for unhydrated Git LFS pointers, not at final link.
file(READ "${MANUS_SDK_ROOT}/lib/libManusSDK_Integrated.so" _sdk_magic LIMIT 4 HEX)
if(NOT _sdk_magic STREQUAL "7f454c46")
  message(FATAL_ERROR "MANUS Integrated SDK library is not an ELF shared library (possibly a Git LFS pointer)")
endif()
add_library(manus_sdk SHARED IMPORTED)
set_target_properties(manus_sdk PROPERTIES
  IMPORTED_LOCATION "${MANUS_SDK_ROOT}/lib/libManusSDK_Integrated.so"
  INTERFACE_INCLUDE_DIRECTORIES "${MANUS_SDK_ROOT}/include")

add_executable(manus_data_publisher src/manus_data_publisher.cpp)
target_compile_features(manus_data_publisher PRIVATE cxx_std_17)
target_link_libraries(manus_data_publisher manus_sdk Threads::Threads)
ament_target_dependencies(manus_data_publisher rclcpp geometry_msgs tianji_interfaces)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(manus_data_publisher PRIVATE -Wall -Wextra -Wpedantic)
endif()
set_target_properties(manus_data_publisher PROPERTIES INSTALL_RPATH "$ORIGIN")
install(TARGETS manus_data_publisher DESTINATION lib/${PROJECT_NAME})
# Install the already-provisioned runtime; never download or vendor a second SDK.
install(FILES "${MANUS_SDK_ROOT}/lib/libManusSDK_Integrated.so"
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/manus_adapter scripts/manus_hand2_retarget
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES 90-manus.rules DESTINATION share/${PROJECT_NAME}/udev)
install(FILES LICENSE.wuji DESTINATION share/${PROJECT_NAME})

ament_package()
