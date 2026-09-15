cmake_minimum_required(VERSION 3.10)
project(odin_sdk LANGUAGES C CXX)

include(GNUInstallDirs)

# Default to Release build with -O3 optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Enable comprehensive warnings
add_compile_options(
  -Wall
  -Wextra
  -Wno-unused-parameter
  -Wno-sign-compare
  -Wno-format-truncation
  -Wno-variadic-macros
  -Wno-unused-function
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build protocol OBJECT library (will be merged into odin_sdk)
add_subdirectory(protocol)

# Build transport OBJECT library (will be merged into odin_sdk)
add_subdirectory(transport)

# Build device OBJECT library (will be merged into odin_sdk)
add_subdirectory(device)

# Build https OBJECT library (will be merged into odin_sdk)
# TODO: Uncomment when mongoose is available in 3rdparty/mongoose/
# add_subdirectory(secure_transport)

# Build http_transfer library (HTTP/HTTPS client)
add_subdirectory(utils/http)

# Build file_transfer OBJECT library (IFileTransfer + implementations)
add_subdirectory(file_transfer)

# SDK sources
file(GLOB ODIN_SDK_SOURCES
    logger/logger.cpp
    src/odin_sdk.cpp
    src/OdinSdkImpl.cpp
    src/SlamOdomSynchronizer.cpp
    src/odin_command_channel.cpp
    hotplugListener/UdpHotplugListener/UdpHotplugListener.cpp
    hotplugListener/CompositeHotplugListener/CompositeHotplugListener.cpp
    hotplugListener/HotplugListenerFactory.cpp
    discovery/OdinUdpDiscovery/OdinUdpDiscovery.cpp
    utils/network_utils.cpp
    ../3rdparty/tinyxml2/tinyxml2.cpp
    ../3rdparty/cJson/cJSON.c)

# Merge all OBJECT libraries into odin_sdk - external only needs to link odin_sdk.a
# 将所有对象库合并到 odin_sdk - 外部只需链接 odin_sdk.a (+ OpenSSL)
add_library(odin_sdk STATIC ${ODIN_SDK_SOURCES} 
    $<TARGET_OBJECTS:odin_protocol_obj>
    $<TARGET_OBJECTS:odin_transport_obj>
    $<TARGET_OBJECTS:odin_device_obj>
    $<TARGET_OBJECTS:file_transfer_obj>
    $<TARGET_OBJECTS:http_transfer_obj>)

# Public headers live under include/
target_include_directories(odin_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/logger
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/device
        ${CMAKE_CURRENT_SOURCE_DIR}/device/IDevice
        ${CMAKE_CURRENT_SOURCE_DIR}/device/Odin2
        ${CMAKE_CURRENT_SOURCE_DIR}/command
        ${CMAKE_CURRENT_SOURCE_DIR}/command/odin2Command
        ${CMAKE_CURRENT_SOURCE_DIR}/capture
        ${CMAKE_CURRENT_SOURCE_DIR}/capture/odinCapture
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/odinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/transport/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/transport/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/hotplugListener
        ${CMAKE_CURRENT_SOURCE_DIR}/hotplugListener/IHotplugListener
        ${CMAKE_CURRENT_SOURCE_DIR}/hotplugListener/UdpHotplugListener
        ${CMAKE_CURRENT_SOURCE_DIR}/utils
        ${CMAKE_CURRENT_SOURCE_DIR}/file_transfer
        ${CMAKE_CURRENT_SOURCE_DIR}/file_transfer/IFileTransfer
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/http
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/tinyxml2
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/cJson)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
target_include_directories(odin_sdk PRIVATE ${EIGEN3_INCLUDE_DIRS})
# PUBLIC链接OpenSSL，这样外部只需链接odin_sdk即可
target_link_libraries(odin_sdk 
    PUBLIC OpenSSL::SSL OpenSSL::Crypto
    PRIVATE Threads::Threads Eigen3::Eigen)

# Install odin_sdk and http_transfer
install(TARGETS odin_sdk http_transfer
    EXPORT odin_sdkTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../sdk/logger
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT odin_sdkTargets
    FILE odin_sdkTargets.cmake
    NAMESPACE odin::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/odin_sdk)
    