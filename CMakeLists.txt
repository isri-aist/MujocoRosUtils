cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME mujoco_ros_utils)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/MujocoRosUtils)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT FALSE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
set(DOXYGEN_HTML_OUTPUT html)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(${PROJECT_NAME} LANGUAGES CXX)

# MuJoCo
if(NOT DEFINED MUJOCO_ROOT_DIR)
  message(FATAL_ERROR "Set \"MUJOCO_ROOT_DIR\"")
elseif(NOT EXISTS ${MUJOCO_ROOT_DIR})
  message(FATAL_ERROR "\"MUJOCO_ROOT_DIR\" is invalid path: ${MUJOCO_ROOT_DIR}")
endif()
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/lib")
set(MUJOCO_BIN_DIR "${MUJOCO_ROOT_DIR}/bin")
file(GLOB LIB_MUJOCO ${MUJOCO_BIN_DIR}/libmujoco[0-9][0-9][0-9].so
     ${MUJOCO_LIB_DIR}/libmujoco.so.*)
if(EXISTS ${LIB_MUJOCO})
  message(STATUS "MuJoCo lib found: " ${LIB_MUJOCO})
else()
  message(FATAL_ERROR "MuJoCo lib not found.")
endif()

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             tf2_ros
             eigen_conversions
             std_msgs
             std_srvs
             geometry_msgs
             visualization_msgs
             sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf2_ros
  eigen_conversions
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  LIBRARIES
  MujocoRosUtilsPlugin)

add_subdirectory(plugin)

# if(INSTALL_DOCUMENTATION) add_subdirectory(doc) endif()
