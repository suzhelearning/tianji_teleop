cmake_minimum_required(VERSION 3.16)
project(tianji_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installs the python codec only. The reviewed controller profiles under
# native/config are installed into share/tianji_controller/config by the
# standalone native CMake project, into the SPD install prefix.
ament_python_install_package(${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
