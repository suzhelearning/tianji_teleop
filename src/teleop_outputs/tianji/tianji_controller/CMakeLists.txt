cmake_minimum_required(VERSION 3.16)
project(tianji_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installs the python package only. The vendor SDKs it loads at runtime live at
# the workspace root under vendor/ and are addressed through
# `tianji_runtime.vendor_path`, so they are not duplicated into share/.
#
# The native controller is built by bash/build_native.sh into install/control
# and is deliberately NOT an ament target: it has its own pinned dependency set
# and must run from the `control` Pixi environment.
ament_python_install_package(${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(tianji_controller_python
    "tianji_controller/tests"
    APPEND_ENV PYTHONPATH="${PROJECT_SOURCE_DIR}"
    TIMEOUT 900
  )
endif()

ament_package()
