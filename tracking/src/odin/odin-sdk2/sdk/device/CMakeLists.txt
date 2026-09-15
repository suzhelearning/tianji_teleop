# Device abstraction layer library
# 设备抽象层库

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEVICE_SOURCES
    Odin2/odin2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command/OdinCommand.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../capture/odinCapture/OdinCaptureBase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/odinProtocol/OdinFrameAssembler.cpp
)

set(DEVICE_HEADERS
    IDevice/IDevice.h
    Odin2/odin2.h
)

# Build as OBJECT library for merging into odin_sdk
add_library(odin_device_obj OBJECT ${DEVICE_SOURCES})

find_package(Eigen3 REQUIRED)

target_include_directories(odin_device_obj
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/IDevice
        ${CMAKE_CURRENT_SOURCE_DIR}/Odin2
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${EIGEN3_INCLUDE_DIRS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../command
        ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command
        ${CMAKE_CURRENT_SOURCE_DIR}/../capture
        ${CMAKE_CURRENT_SOURCE_DIR}/../capture/odinCapture
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/odinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../transport/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/../transport/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/../logger
        ${CMAKE_CURRENT_SOURCE_DIR}/../file_transfer
        ${CMAKE_CURRENT_SOURCE_DIR}/../file_transfer/IFileTransfer
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils/http
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/cJson
)

# Also build as static library for standalone use
add_library(odin_device STATIC ${DEVICE_SOURCES})

target_include_directories(odin_device
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/IDevice
        ${CMAKE_CURRENT_SOURCE_DIR}/Odin2
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${EIGEN3_INCLUDE_DIRS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../command
        ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command
        ${CMAKE_CURRENT_SOURCE_DIR}/../capture
        ${CMAKE_CURRENT_SOURCE_DIR}/../capture/odinCapture
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/odinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../transport/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/../transport/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/../logger
        ${CMAKE_CURRENT_SOURCE_DIR}/../file_transfer
        ${CMAKE_CURRENT_SOURCE_DIR}/../file_transfer/IFileTransfer
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils/http
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/cJson
)

target_link_libraries(odin_device
    PRIVATE
        odin_protocol
        odin_transport
        Eigen3::Eigen
)
