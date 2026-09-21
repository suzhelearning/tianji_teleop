cmake_minimum_required(VERSION 3.10)
project(file_transfer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# File transfer sources
set(FILE_TRANSFER_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/https/HttpsFileTransfer.cpp
)

# Build as OBJECT library (will be merged into odin_sdk)
add_library(file_transfer_obj OBJECT ${FILE_TRANSFER_SRCS})

# Include directories
target_include_directories(file_transfer_obj
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils/http
)

# Link dependencies (http_transfer provides HTTP/HTTPS functionality)
target_link_libraries(file_transfer_obj
    PUBLIC
        http_transfer
)
