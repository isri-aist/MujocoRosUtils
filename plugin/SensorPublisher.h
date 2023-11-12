#pragma once

#include <ros/ros.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <string>

namespace MujocoRosUtils
{

/** \brief Plugin to publish sensor data. */
class SensorPublisher
{
public:
  /** \brief Type of ROS message. */
  typedef enum MessageType_
  {
    //! Scalar
    MsgScalar = 0,

    //! Point
    MsgPoint,

    //! 3D vector
    MsgVector3,

    //! Quaternion
    MsgQuaternion
  } MessageType;

public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static SensorPublisher * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  SensorPublisher(SensorPublisher &&) = default;

  /** \brief Reset.
      \param m model
      \param plugin_id plugin ID
   */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Compute.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  /** \brief Constructor.
      \param m model
      \param d data
      \param sensor_id sensor ID
      \param msg_type type of ROS message
      \param frame_id frame ID of message header
      \param topic_name topic name
      \param publish_rate publish rate
   */
  SensorPublisher(const mjModel * m,
                  mjData * d,
                  int sensor_id,
                  MessageType msg_type,
                  const std::string & frame_id,
                  const std::string & topic_name,
                  mjtNum publish_rate);

protected:
  //! Sensor ID
  int sensor_id_ = -1;

  //! Type of ROS message
  MessageType msg_type_;

  //! Frame ID of message header
  std::string frame_id_;

  //! Topic name
  std::string topic_name_;

  //! ROS node handle
  std::shared_ptr<ros::NodeHandle> nh_;

  //! ROS publisher
  ros::Publisher pub_;

  //! Iteration interval to skip ROS publish
  int publish_skip_ = 0;

  //! Iteration count of simulation
  int sim_cnt_ = 0;
};

} // namespace MujocoRosUtils
