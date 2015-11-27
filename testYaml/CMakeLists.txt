cmake_minimum_required(VERSION 2.8)

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++0x -pedantic")
endif()

enable_testing()

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/pkg-config.cmake)

set(PROJECT_NAME YamlTest)
set(PROJECT_DESCRIPTION YamlTest)
set(PROJECT_URL https://brossette@idh.lirmm.fr/././././testYaml.git)

set(CUSTOM_HEADER_DIR YamlTest)
set(HEADERS )

setup_project()
if(CMAKE_BUILD_TYPE MATCHES Rel)
  if(NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
  endif()
endif()

# Add main library to pkg-config file.
PKG_CONFIG_APPEND_LIBS(YamlTest)
#
#search_for_boost()

# Require Eigen >= 3.2.1 for Eigen::Ref
SET(Eigen_REQUIRED "eigen3 >= 3.2.1")
IF(PKG_CONFIG_FOUND)
  SEARCH_FOR_EIGEN()
ELSE()
  find_path(Eigen_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library
    PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    PATH_SUFFIXES eigen3 eigen
  )
  INCLUDE_DIRECTORIES(SYSTEM ${Eigen_INCLUDE_DIR})
ENDIF()

ADD_REQUIRED_DEPENDENCY("yaml-cpp >= 0.5")

include_directories(include)

FIND_PACKAGE(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# add_subdirectory(bin)
# add_subdirectory(share)
add_subdirectory(src)
add_subdirectory(tests)
# add_subdirectory(var)

setup_project_finalize()
