# Ruckig Community 0.19.4, https://github.com/pantor/ruckig/tree/v0.19.4
# Upstream sources are unchanged; see LICENSE. Only the local, offline C++
# trajectory generator is built: no cloud client, Python wrapper or downloads.
add_library(ruckig STATIC
  src/ruckig/brake.cpp
  src/ruckig/position_first_step1.cpp
  src/ruckig/position_first_step2.cpp
  src/ruckig/position_second_step1.cpp
  src/ruckig/position_second_step2.cpp
  src/ruckig/position_third_step1.cpp
  src/ruckig/position_third_step2.cpp
  src/ruckig/velocity_second_step1.cpp
  src/ruckig/velocity_second_step2.cpp
  src/ruckig/velocity_third_step1.cpp
  src/ruckig/velocity_third_step2.cpp
)
add_library(ruckig::ruckig ALIAS ruckig)
target_compile_features(ruckig PUBLIC cxx_std_20)
target_include_directories(ruckig SYSTEM PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
set_target_properties(ruckig PROPERTIES POSITION_INDEPENDENT_CODE ON)
