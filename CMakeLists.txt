cmake_minimum_required(VERSION 3.10)
project(fast_gicp)

option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)

set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP COMPONENTS CXX)

add_library(fast_gicp SHARED
  src/fast_gicp/gicp/lsq_registration.cpp
  src/fast_gicp/gicp/fast_gicp.cpp
  src/fast_gicp/gicp/fast_gicp_st.cpp
  src/fast_gicp/gicp/fast_vgicp.cpp
  src/fast_gicp/gicp/fast_svgicp.cpp
)

target_include_directories(fast_gicp PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(fast_gicp PUBLIC
  ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(fast_gicp PUBLIC OpenMP::OpenMP_CXX)
endif()

if(BUILD_PYTHON_BINDINGS)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(pygicp
    src/python/main.cpp
  )

  target_include_directories(pygicp PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )

  target_link_libraries(pygicp PRIVATE
    fast_gicp
  )

  if(APPLE)
    set_target_properties(pygicp PROPERTIES
      BUILD_WITH_INSTALL_RPATH TRUE
      INSTALL_RPATH "@loader_path"
    )
  else()
    set_target_properties(pygicp PROPERTIES
      BUILD_WITH_INSTALL_RPATH TRUE
      INSTALL_RPATH "$ORIGIN"
    )
  endif()
endif()