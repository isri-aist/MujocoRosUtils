# [MujocoRosUtils](https://github.com/isri-aist/MujocoRosUtils)
ROS-based MuJoCo utilities

[![CI](https://github.com/isri-aist/MujocoRosUtils/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/MujocoRosUtils/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/MujocoRosUtils/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/MujocoRosUtils)](https://github.com/isri-aist/MujocoRosUtils/blob/master/LICENSE)

https://github.com/isri-aist/MujocoRosUtils/assets/6636600/6cc3bc6c-113d-4a1d-97f5-d75b1000fc8a

## Features
- You can retrieve body poses and camera images, send commands to actuators, and apply external forces to the body in MuJoCo via ROS interfaces.
- Since it is in plugin style, you can use it without rebuilding MuJoCo from the source.

## Install

### Requirements
- Compiler supporting C++17
- Tested with `Ubuntu 20.04 / ROS Noetic`

### Dependencies
- [MuJoCo](https://github.com/deepmind/mujoco) (>= 2.3.5)

### Installation procedure
```bash
# Setup catkin workspace.
$ mkdir -p ${HOME}/ros/ws_mujoco/src
$ cd ${HOME}/ros/ws_mujoco
$ wstool init src
$ wstool set -t src isri-aist/MujocoRosUtils git@github.com:isri-aist/MujocoRosUtils.git --git -y
$ wstool update -t src
# Install dependent packages.
$ source /opt/ros/${ROS_DISTRO}/setup.bash
$ rosdep install -y -r --from-paths src --ignore-src
# Build a package.
$ catkin init
$ catkin config --extend /opt/ros/${ROS_DISTRO}
$ catkin build mujoco_ros_utils -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=<absolute path to MuJoCo>
```
`<absolute path to MuJoCo>` is the path to the root directory of MuJoCo.
For example, `${HOME}/.mujoco/mujoco-2.3.5` if you installed MuJoCo from release, or `${HOME}/src/mujoco` if you installed it from source.

Add `source ${HOME}/ros/ws_mujoco/devel/setup.bash` to `${HOME}/.bashrc`.

## Examples
Assume that MuJoCo is installed in `${HOME}/.mujoco/mujoco-2.3.5` from release, and the path to the catkin workspace is `${HOME}/ros/ws_mujoco`.
```bash
# Terminal 1
$ cp ${HOME}/ros/ws_mujoco/devel/lib/libMujocoRosUtilsPlugin.so ${HOME}/.mujoco/mujoco-2.3.5/bin/mujoco_plugin
$ cd ${HOME}/.mujoco/mujoco-2.3.5/bin
$ ./simulate `rospack find mujoco_ros_utils`/xml/sample_mujoco_ros_utils.xml
# Terminal 2
$ roslaunch mujoco_ros_utils display.launch
```
To visualize a point cloud restored from a depth image, add the `points:=true` option to `display.launch`.
(`ros-${ROS_DISTRO}-depth-image-proc` must be installed.)

## Plugins
### MujocoRosUtils::ClockPublisher
Plugin to publish clock topic.

All of the following attributes are optional.
- `topic_name`: Topic name of clock. (Default is `/clock`)
- `publish_rate`: Publish rate. (Default is 100.0 [Hz])
- `use_sim_time`: Value of `use_sim_time` rosparam. (Default is `true`)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::ClockPublisher"/>
</extension>
<worldbody>
  <plugin plugin="MujocoRosUtils::ClockPublisher">
    <config key="topic_name" value="/clock"/>
    <config key="publish_rate" value="100"/>
    <config key="use_sim_time" value="true"/>
  </plugin>
</worldbody>
```
This plugin must be registered in worldbody.

### MujocoRosUtils::PosePublisher
Plugin to publish topics or broadcast TF of pose and velocity of the body.

All of the following attributes are optional.
- `frame_id`: Frame ID of topics header or TF parent. (Default is `map`)
- `pose_topic_name`: Topic name of pose. (Default is `mujoco/<body name>/pose`)
- `vel_topic_name`: Topic name of velocity. (Default is `mujoco/<body name>/vel`)
- `publish_rate`: Publish rate. (Default is 30.0 [Hz])
- `output_tf`: Whether to broadcast TF. (Default is `false`)
- `tf_child_frame_id`: Child frame ID for TF. Used only when `output_tf` is `true`. (Default is `<body name>`)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::PosePublisher"/>
</extension>
<sensor>
  <plugin name="pose_publisher" plugin="MujocoRosUtils::PosePublisher" objtype="xbody" objname="object">
    <config key="frame_id" value="map"/>
    <config key="pose_topic_name" value="/pose"/>
    <config key="vel_topic_name" value="/vel"/>
    <config key="publish_rate" value="30"/>
    <config key="output_tf" value="false"/>
    <config key="tf_child_frame_id" value="object"/>
  </plugin>
</sensor>
```
The `objtype` attribute must be `xbody`.

### MujocoRosUtils::ExternalForce
Plugin to apply external force to the body.

All of the following attributes are optional.
- `topic_name`: Topic name of external force. (Default is `/external_force`)
- `vis_scale`: Arrow length scale. (Default is 0.1)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::ExternalForce"/>
</extension>
<worldbody>
  <body name="object" pos="0 0 1">
    <freejoint/>
    <geom type="box" size="0.2 0.2 0.05" mass="0.1" rgba="0.5 0.5 0.5 0.3"/>
    <plugin plugin="MujocoRosUtils::ExternalForce">
      <config key="topic_name" value="/external_force"/>
      <config key="vis_scale" value="0.1"/>
    </plugin>
  </body>
</worldbody>
```

### MujocoRosUtils::ImagePublisher
Plugin to publish topics of color and depth images.

All of the following attributes are optional.
- `frame_id`: Frame ID of topics header or TF parent. (Default is `<camera name>`)
- `color_topic_name`: Topic name of color image. (Default is `mujoco/<camera name>/color`)
- `depth_topic_name`: Topic name of depth image. (Default is `mujoco/<camera name>/depth`)
- `info_topic_name`: Topic name of camera information. (Default is `mujoco/<camera name>/camera_info`)
- `height`: Image height. (Default is 240)
- `width`: Image width. (Default is 320)
- `publish_rate`: Publish rate. (Default is 30.0 [Hz])

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::ImagePublisher"/>
</extension>
<sensor>
  <plugin name="image_publisher" plugin="MujocoRosUtils::ImagePublisher" objtype="camera" objname="camera">
    <config key="frame_id" value="camera"/>
    <config key="color_topic_name" value="/image/color"/>
    <config key="depth_topic_name" value="/image/depth"/>
    <config key="info_topic_name" value="/image/camera_info"/>
    <config key="height" value="240"/>
    <config key="width" value="320"/>
    <config key="publish_rate" value="30"/>
  </plugin>
</sensor>
```
The `objtype` attribute must be `camera`.

### MujocoRosUtils::ActuatorCommand
Plugin to send a command to an actuator via ROS topic.

The following attributes are required.
- `actuator_name`: Actuator name to which the command is sent.
- `topic_name`: Topic name of actuator command. (Default is `mujoco/<actuator name>`)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::ActuatorCommand"/>
</extension>
<actuator>
  <position name="camera_pan" joint="camera_pan"/>
  <plugin plugin="MujocoRosUtils::ActuatorCommand" joint="camera_pan">
    <config key="actuator_name" value="camera_pan"/>
    <config key="topic_name" value="/camera_pan"/>
  </plugin>
</actuator>
```
In the `plugin` element, you need to specify the `joint`, `body`, etc. of the actuator to be controlled.
This information is not used in the plugin, but is necessary to avoid errors in MJCF parsing.

The plugin itself is also added to the list of actuators, but it is a dummy actuator. The unwanted increase in the number of actuators (which also increases the dimension of `d->ctrl`) is a problem that should be solved in the future.
