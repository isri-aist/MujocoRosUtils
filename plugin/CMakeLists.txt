add_library(MujocoRosUtils SHARED ClockPublisher.cpp PosePublisher.cpp)
target_include_directories(MujocoRosUtils PRIVATE ${MUJOCO_INCLUDE_DIR})
target_include_directories(MujocoRosUtils PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(MujocoRosUtils PUBLIC ${LIB_MUJOCO} ${catkin_LIBRARIES})

add_library(MujocoRosUtilsPlugin SHARED Plugin.cpp)
target_include_directories(MujocoRosUtilsPlugin PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(MujocoRosUtilsPlugin PUBLIC MujocoRosUtils)

set(MUJOCO_PLUGIN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set_target_properties(
  MujocoRosUtilsPlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                  ${MUJOCO_PLUGIN_OUTPUT_DIR})
catkin_add_env_hooks(99.mujoco_ros_utils SHELLS sh DIRECTORY
                     ${PROJECT_SOURCE_DIR}/env-hooks)
