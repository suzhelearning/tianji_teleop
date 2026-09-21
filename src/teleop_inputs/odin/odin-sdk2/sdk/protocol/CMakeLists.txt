# Protocol static library + OBJECT library
# 协议静态库 - 同时生成 .a 文件供保留，以及 OBJECT 用于合并到 SDK

set(PROTOCOL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/OdinProtocol/OdinProtocol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/FastCRC/FastCRChw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/FastCRC/FastCRCsw.cpp
)

# OBJECT library: for merging into odin_sdk
add_library(odin_protocol_obj OBJECT ${PROTOCOL_SOURCES})

target_include_directories(odin_protocol_obj
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty
)

set_target_properties(odin_protocol_obj PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Static library: for internal use/archival (可保留的独立库)
add_library(odin_protocol STATIC ${PROTOCOL_SOURCES})

target_include_directories(odin_protocol
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty
)
