#pragma once

#include <ros/ros.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <string>

namespace mujoco::plugin::sensor
{

/** \brief Plugin to publish topics and broadcast TF of pose and velocity of the body. */
class PosePublisher
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static PosePublisher * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  PosePublisher(PosePublisher &&) = default;

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
      \param frame_id frame ID
      \param pose_topic_name topic name of pose
      \param vel_topic_name topic name of velocity
      \param publish_rate publish rate
      \param output_tf whether to broadcast TF
  */
  PosePublisher(const mjModel * m,
                mjData * d,
                int sensor_id,
                const std::string & frame_id,
                const std::string & pose_topic_name,
                const std::string & vel_topic_name,
                mjtNum publish_rate,
                bool output_tf);

protected:
  //! Sensor ID
  int sensor_id_ = -1;

  //! Body ID
  int body_id_ = -1;

  //! ROS node handle
  std::shared_ptr<ros::NodeHandle> nh_;

  //! ROS publisher for pose
  ros::Publisher pose_pub_;

  //! ROS publisher for velocity
  ros::Publisher vel_pub_;

  //! Frame ID
  std::string frame_id_;

  //! Iteration interval to skip ROS publish
  int publish_skip_ = 0;

  //! Whether to broadcast TF
  bool output_tf_ = false;

  //! Iteration count of simulation
  int sim_cnt_ = 0;
};

} // namespace mujoco::plugin::sensor
