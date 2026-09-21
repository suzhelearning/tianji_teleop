cmake_minimum_required(VERSION 3.10)
project(odin_send_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT TARGET odin_sdk)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../sdk ${CMAKE_BINARY_DIR}/odin_sdk_build)
endif()

add_executable(odin_send_calibration main.cpp)

target_include_directories(odin_send_calibration
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/logger
        ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty)

find_package(Threads REQUIRED)
target_link_libraries(odin_send_calibration
    PRIVATE
        odin_sdk
        Threads::Threads)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(odin_send_calibration PRIVATE -g -O0)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(odin_send_calibration PRIVATE -O2 -DNDEBUG)
endif()

install(TARGETS odin_send_calibration RUNTIME DESTINATION bin/)
