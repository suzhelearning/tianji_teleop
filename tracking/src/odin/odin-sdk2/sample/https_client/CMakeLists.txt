cmake_minimum_required(VERSION 3.10)
project(https_client_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add http_transfer library (for standalone build)
if(NOT TARGET http_transfer)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/utils/http
                     ${CMAKE_CURRENT_BINARY_DIR}/http_transfer)
endif()

add_executable(https_client_test main.cpp)

# Link static http_transfer (dependencies propagate via PUBLIC)
target_link_libraries(https_client_test PRIVATE
    http_transfer
)

install(TARGETS https_client_test RUNTIME DESTINATION bin)
