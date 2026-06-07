cmake_minimum_required(VERSION 3.21)
project(plrs_sim_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# nanobind shipped by the build-system requirement in pyproject.toml.
find_package(Python 3.11 COMPONENTS Interpreter Development.Module REQUIRED)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# TinyEKF is header-only. Pinned to the same SHA platformio.ini resolves to;
# bump both together or parity with the firmware build is lost.
include(FetchContent)
FetchContent_Declare(
  tinyekf
  GIT_REPOSITORY https://github.com/simondlevy/TinyEKF.git
  GIT_TAG        ba0d2a90e22f2f4e4120a91cb05ca2ca6a8e8da3
  GIT_SHALLOW    FALSE)
FetchContent_MakeAvailable(tinyekf)

nanobind_add_module(_native STABLE_ABI src/ekf_bind.cpp)
target_include_directories(_native PRIVATE
  ${CMAKE_SOURCE_DIR}/../lib/fusion
  ${CMAKE_SOURCE_DIR}/../lib/geometry
  ${tinyekf_SOURCE_DIR}/src)

install(TARGETS _native LIBRARY DESTINATION plrs_sim)
