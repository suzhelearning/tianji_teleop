include(FetchContent)
# Opt-in dependency: old builds do not find/download/link Ceres.
option(TIANJI_ENABLE_CERES "Build the independent Ceres LM EE IK route" OFF)
if(TIANJI_ENABLE_CERES AND NOT TARGET Ceres::ceres)
  # A previous FetchContent build caches this project variable. Ceres 2.1's
  # installed config treats it as an active in-tree build and skips importing
  # Ceres::ceres, so discard that stale build-tree identity before discovery.
  unset(Ceres_BINARY_DIR CACHE)
  unset(Ceres_BINARY_DIR)
  find_package(Ceres 2.1 QUIET CONFIG)
  if(NOT Ceres_FOUND)
    # Function scope keeps Ceres' build options out of our existing test setup.
    function(tianji_fetch_ceres)
      set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
      set(BUILD_TESTING OFF)
      set(BUILD_EXAMPLES OFF)
      set(BUILD_BENCHMARKS OFF)
      set(BUILD_DOCUMENTATION OFF)
      set(MINIGLOG ON)
      set(GFLAGS OFF)
      set(SUITESPARSE OFF)
      set(CXSPARSE OFF)
      set(LAPACK OFF)
      set(EIGENSPARSE OFF)
      set(SCHUR_SPECIALIZATIONS OFF)
      set(EXPORT_BUILD_DIR OFF)
      FetchContent_Declare(ceres
        URL https://codeload.github.com/ceres-solver/ceres-solver/tar.gz/refs/tags/2.1.0
        URL_HASH SHA256=ccbd716a93f65d4cb017e3090ae78809e02f5426dce16d0ee2b4f8a4ba2411a8
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
      FetchContent_MakeAvailable(ceres)
    endfunction()
    tianji_fetch_ceres()
  endif()
endif()
