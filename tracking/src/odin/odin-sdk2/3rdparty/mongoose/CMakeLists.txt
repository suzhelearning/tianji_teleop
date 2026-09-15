# Mongoose Web Server Library
# SPDX-License-Identifier: GPL-2.0-only

cmake_minimum_required(VERSION 3.10)
project(mongoose LANGUAGES C)

include(GNUInstallDirs)

# Static library
add_library(mongoose STATIC mongoose.c)

# Include directories (use generator expressions for build/install paths)
target_include_directories(mongoose
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/mongoose>
)

# Compile definitions
target_compile_definitions(mongoose
    PUBLIC
        MG_ENABLE_LINES=1
        MG_ENABLE_DIRECTORY_LISTING=1
        MG_ENABLE_SSI=0
)

# Thread support
find_package(Threads REQUIRED)
target_link_libraries(mongoose PUBLIC Threads::Threads)

# Optional: Enable TLS
option(MG_ENABLE_OPENSSL "Enable OpenSSL TLS support" ON)
if(MG_ENABLE_OPENSSL)
    find_package(OpenSSL REQUIRED)
    target_compile_definitions(mongoose PUBLIC MG_TLS=MG_TLS_OPENSSL)
    target_link_libraries(mongoose PUBLIC OpenSSL::SSL OpenSSL::Crypto)
endif()

# Install targets
install(TARGETS mongoose
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(FILES mongoose.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mongoose)
