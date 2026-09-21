# Transport OBJECT library - will be merged into odin_sdk
# 传输层对象库 - 会被合并到 odin_sdk 中

set(TRANSPORT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Network/UdpTransport.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Network/TcpTransport.cpp
)

# OBJECT library: for merging into odin_sdk
add_library(odin_transport_obj OBJECT ${TRANSPORT_SOURCES})

target_include_directories(odin_transport_obj
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/Usb
        ${CMAKE_CURRENT_SOURCE_DIR}/../logger
)

set_target_properties(odin_transport_obj PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Static library: for internal use/archival (可保留的独立库)
add_library(odin_transport STATIC ${TRANSPORT_SOURCES})

target_include_directories(odin_transport
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/Usb
        ${CMAKE_CURRENT_SOURCE_DIR}/../logger
)
