cmake_minimum_required(VERSION 3.10)
project(http_transfer LANGUAGES C CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)

# Mongoose source (embedded into http_transfer)
set(3RDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty)

# HTTP/HTTPS module sources + mongoose
set(HTTP_TRANSFER_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/http_client.cpp
    ${3RDPARTY_DIR}/mongoose/mongoose.c
)

# Build as OBJECT library (for merging into odin_sdk)
add_library(http_transfer_obj OBJECT ${HTTP_TRANSFER_SRCS})

# Also build as static library (for standalone use like https_client sample)
add_library(http_transfer STATIC ${HTTP_TRANSFER_SRCS})

# Set library properties
set_target_properties(http_transfer PROPERTIES
    VERSION 1.0.0
    SOVERSION 1
    POSITION_INDEPENDENT_CODE ON
    OUTPUT_NAME "http_transfer"
)

# Compile definitions for mongoose (both targets)
foreach(tgt http_transfer_obj http_transfer)
    target_compile_definitions(${tgt}
        PRIVATE
            MG_TLS=MG_TLS_OPENSSL
            MG_ENABLE_LINES=1
            MG_ENABLE_DIRECTORY_LISTING=1
            MG_ENABLE_SSI=0
    )
    target_include_directories(${tgt}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/http_transfer>
        PRIVATE
            ${3RDPARTY_DIR}
    )
    set_target_properties(${tgt} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        POSITION_INDEPENDENT_CODE ON
    )
endforeach()

# Link libraries (only for static library target)
target_link_libraries(http_transfer
    PUBLIC
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
)
