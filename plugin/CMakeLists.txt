add_library(
  MujocoRosUtils SHARED
  ClockPublisher.cpp PosePublisher.cpp ImagePublisher.cpp ActuatorCommand.cpp
  ExternalForce.cpp SensorPublisher.cpp)

target_include_directories(MujocoRosUtils PRIVATE ${MUJOCO_INCLUDE_DIR})
target_include_directories(MujocoRosUtils PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(MujocoRosUtils PUBLIC ${LIB_MUJOCO} ${catkin_LIBRARIES}
                                            glfw)
add_dependencies(MujocoRosUtils ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(MujocoRosUtilsPlugin SHARED Plugin.cpp)
target_include_directories(MujocoRosUtilsPlugin PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(MujocoRosUtilsPlugin PUBLIC MujocoRosUtils)
