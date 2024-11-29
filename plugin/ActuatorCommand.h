#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <limits>
#include <string>

namespace MujocoRosUtils
{

/** \brief Plugin to send a command to an actuator via ROS topic. */
class ActuatorCommand
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static ActuatorCommand * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  ActuatorCommand(ActuatorCommand &&) = default;

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
      \param actuator_id actuator ID
      \param topic_name topic name
  */
  ActuatorCommand(const mjModel * m, mjData * d, int actuator_id, std::string topic_name);

  /** \brief Constructor.
      \param msg command message
  */
  void callback(const std_msgs::msg::Float64::SharedPtr msg);

protected:
  //! Actuator ID
  int actuator_id_ = -1;

  //! Actuator command (NaN for no command)
  mjtNum ctrl_ = std::numeric_limits<mjtNum>::quiet_NaN();

  //! ROS variables
  //! @{
  rclcpp::Node::SharedPtr nh_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  //! @}
};

} // namespace MujocoRosUtils
