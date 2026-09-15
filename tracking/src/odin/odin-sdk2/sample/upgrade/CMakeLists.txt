cmake_minimum_required(VERSION 3.10)
project(odin_OTA_tool LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

if (NOT TARGET odin_sdk)
    add_subdirectory(${SDK_ROOT}/sdk ${CMAKE_BINARY_DIR}/odin_sdk_build)
endif()

add_executable(odin_OTA_tool upgrade.cpp)

target_include_directories(odin_OTA_tool
    PRIVATE
        ${SDK_ROOT}/include
        ${SDK_ROOT}/sdk/logger
        ${SDK_ROOT}/sdk/src
        ${SDK_ROOT}/sdk/utils/http
        ${SDK_ROOT}/3rdparty)

target_compile_definitions(odin_OTA_tool PRIVATE
    MG_TLS=MG_TLS_OPENSSL)

find_package(Threads REQUIRED)
target_link_libraries(odin_OTA_tool PRIVATE odin_sdk Threads::Threads)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(odin_OTA_tool PRIVATE -g -O0 -funwind-tables -rdynamic -fno-inline -fno-omit-frame-pointer)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(odin_OTA_tool PRIVATE -O2 -DNDEBUG)
endif()

install(TARGETS odin_OTA_tool RUNTIME DESTINATION bin/)
