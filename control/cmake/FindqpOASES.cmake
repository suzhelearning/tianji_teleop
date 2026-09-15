# Compatibility finder for installations that do not export qpOASESConfig.cmake.
find_path(qpOASES_INCLUDE_DIR NAMES qpOASES.hpp)
find_library(qpOASES_LIBRARY NAMES qpOASES)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(qpOASES
  REQUIRED_VARS qpOASES_INCLUDE_DIR qpOASES_LIBRARY
)

if(qpOASES_FOUND AND NOT TARGET qpOASES::qpOASES)
  add_library(qpOASES::qpOASES UNKNOWN IMPORTED)
  set_target_properties(qpOASES::qpOASES PROPERTIES
    IMPORTED_LOCATION "${qpOASES_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${qpOASES_INCLUDE_DIR}"
  )
endif()
