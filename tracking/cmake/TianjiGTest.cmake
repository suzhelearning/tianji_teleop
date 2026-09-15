# Use the installed GTest library, not Humble's old bundled source snapshot.
# Keep ament's XML/result registration so colcon test-result remains authoritative.
include_guard(GLOBAL)
find_package(GTest CONFIG REQUIRED)
find_package(ament_cmake_test REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

function(tianji_add_gtest target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} GTest::gtest_main)
  ament_add_gtest_test(${target})
endfunction()
