cmake_minimum_required(VERSION 3.20)
project(mars_rover VERSION 0.16.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(MARS_ROVER_BUILD_PYTHON "Build the Python extension" ON)
option(MARS_ROVER_FAST_MATH "Use fast floating-point math in Release builds" ON)
option(MARS_ROVER_OPENMP "Parallelize large environment batches" ON)

if(MSVC)
  set(MARS_ROVER_WARNINGS /W4 /permissive-)
else()
  set(MARS_ROVER_WARNINGS -Wall -Wextra -Wpedantic)
endif()

add_library(mars_rover_core
  src/batch_env.cpp
  src/episode.cpp
  src/env.cpp
  src/mechanics.cpp
  src/observation.cpp
  src/physics.cpp
  src/renderer.cpp
  src/reward.cpp
  src/rover_rig.cpp
  src/terrain.cpp
  src/world_latents.cpp
  src/world_layout.cpp
)

target_include_directories(mars_rover_core PUBLIC include)
target_compile_options(mars_rover_core PRIVATE ${MARS_ROVER_WARNINGS})
if(MARS_ROVER_FAST_MATH)
  if(MSVC)
    target_compile_options(mars_rover_core PRIVATE $<$<CONFIG:Release>:/fp:fast>)
  else()
    target_compile_options(mars_rover_core PRIVATE $<$<CONFIG:Release>:-ffast-math>)
  endif()
endif()
if(MARS_ROVER_OPENMP)
  find_package(OpenMP QUIET)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(mars_rover_core PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(mars_rover_core PUBLIC MARS_ROVER_HAS_OPENMP=1)
  endif()
endif()

if(MARS_ROVER_BUILD_PYTHON)
  find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
    OUTPUT_VARIABLE pybind11_ROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE pybind11_RESULT
  )
  if(NOT pybind11_RESULT EQUAL 0)
    message(FATAL_ERROR "pybind11 is missing. Install it with: ${Python_EXECUTABLE} -m pip install pybind11")
  endif()
  find_package(pybind11 CONFIG REQUIRED PATHS "${pybind11_ROOT}")
  pybind11_add_module(_mars_rover_cpp bindings/pybind_module.cpp)
  target_link_libraries(_mars_rover_cpp PRIVATE mars_rover_core)
  target_compile_options(_mars_rover_cpp PRIVATE ${MARS_ROVER_WARNINGS})
  if(MINGW)

    target_compile_options(_mars_rover_cpp PRIVATE -Wa,-mbig-obj)
  endif()
endif()
