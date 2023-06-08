#pragma once

#include <ros/ros.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <string>

namespace MujocoRosUtils
{

/** \brief Plugin to publish clock topic. */
class ClockPublisher
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static ClockPublisher * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  ClockPublisher(ClockPublisher &&) = default;

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
      \param topic_name topic name of clock
      \param publish_rate publish rate
      \param use_sim_time value of `use_sim_time` rosparam
  */
  ClockPublisher(const mjModel * m, mjData * d, const std::string & topic_name, mjtNum publish_rate, bool use_sim_time);

protected:
  //! ROS node handle
  std::shared_ptr<ros::NodeHandle> nh_;

  //! ROS publisher for clock
  ros::Publisher pub_;

  //! Topic name of clock
  std::string topic_name_;

  //! Iteration interval to skip ROS publish
  int publish_skip_ = 0;

  //! Value of `use_sim_time` rosparam
  bool use_sim_time_ = false;

  //! Iteration count of simulation
  int sim_cnt_ = 0;
};

} // namespace MujocoRosUtils
