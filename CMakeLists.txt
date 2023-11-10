cmake_minimum_required(VERSION 3.10)

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
set(MUJOCO_REQUIRED_VERSION 237)

set(MUJOCO_MAIN_HEADER "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
if(NOT EXISTS "${MUJOCO_MAIN_HEADER}")
  message(
    FATAL_ERROR
      "MuJoCo main header (${MUJOCO_MAIN_HEADER}) not found, maybe MUJOCO_ROOT_DIR is badly set"
  )
endif()
file(READ "${MUJOCO_MAIN_HEADER}" MUJOCO_MAIN_HEADER_CONTENT)
if(MUJOCO_MAIN_HEADER_CONTENT MATCHES ".*#define mjVERSION_HEADER ([0-9]+).*")
  set(MUJOCO_VERSION ${CMAKE_MATCH_1})
else()
  message(
    FATAL_ERROR
      "Could not extract MuJoCo version from the header, please contact this package developers"
  )
endif()
if(${MUJOCO_VERSION} LESS ${MUJOCO_REQUIRED_VERSION})
  message(
    FATAL_ERROR
      "This package requires at least MuJoCo version ${MUJOCO_REQUIRED_VERSION} but you have ${MUJOCO_VERSION}"
  )
endif()

file(GLOB LIB_MUJOCO ${MUJOCO_BIN_DIR}/libmujoco[0-9][0-9][0-9].so
     ${MUJOCO_LIB_DIR}/libmujoco.so.*)
if(EXISTS ${LIB_MUJOCO})
  message(STATUS "MuJoCo lib found: " ${LIB_MUJOCO})
else()
  message(FATAL_ERROR "MuJoCo lib not found.")
endif()

# GLFW
find_package(glfw3 3.3 REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             roscpp
             tf2_ros
             eigen_conversions
             std_msgs
             std_srvs
             geometry_msgs
             visualization_msgs
             sensor_msgs)

add_message_files(FILES ScalarStamped.msg ExternalForce.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  tf2_ros
  eigen_conversions
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  LIBRARIES
  MujocoRosUtils
  MujocoRosUtilsPlugin)

add_subdirectory(plugin)

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
