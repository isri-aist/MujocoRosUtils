add_library(MujocoRosUtils SHARED PosePublisher.cpp)
target_include_directories(MujocoRosUtils PRIVATE ${MUJOCO_INCLUDE_DIR})
target_include_directories(MujocoRosUtils PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(MujocoRosUtils PUBLIC ${LIB_MUJOCO} ${catkin_LIBRARIES})

add_library(MujocoRosUtilsPlugin SHARED Plugin.cpp)
target_include_directories(MujocoRosUtilsPlugin PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(MujocoRosUtilsPlugin PUBLIC MujocoRosUtils)
