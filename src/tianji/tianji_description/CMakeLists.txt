cmake_minimum_required(VERSION 3.16)
project(tianji_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ---------------------------------------------------------------------------
# Model and mesh closure.
#
# `models/` and `marvin_m6_ccs/` are installed as whole trees: the MJCF files
# use relative `meshdir` values (`tianji_wuji2`, `./`) and the URDFs reference
# `../marvin_m6_ccs/meshes/*`, so flattening the layout would silently break
# mesh resolution. Nothing in this package is compiled.
# ---------------------------------------------------------------------------
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)
install(DIRECTORY
  marvin_m6_ccs/
  DESTINATION share/${PROJECT_NAME}/marvin_m6_ccs
)
install(DIRECTORY
  mapped_palm/
  DESTINATION share/${PROJECT_NAME}/mapped_palm
)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# The two helpers shared by the C++ controller tests, the simulator and the
# collector's metadata: installed as an importable python package so callers
# use `tianji_description.home_config` / `.model_assets` rather than a path.
ament_python_install_package(${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
