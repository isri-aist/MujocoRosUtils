#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <mujoco_ros_utils/ExternalForce.h>
#include <string>

namespace MujocoRosUtils
{

/** \brief Plugin to apply external force. */
class ExternalForce
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static ExternalForce * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Copy constructor. */
  ExternalForce(ExternalForce &&) = default;

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

  /** \brief Visualize.
      \param m model
      \param d data
      \param opt visualization option
      \param scn rendering scene
      \param plugin_id plugin ID
   */
  void visualize(const mjModel * m, mjData * d, const mjvOption * opt, mjvScene * scn, int plugin_id);

protected:
  /** \brief Constructor.
      \param m model
      \param d data
      \param body_id body ID
      \param topic_name topic name of external force
      \param vis_scale arrow length scale
  */
  ExternalForce(const mjModel * m, mjData * d, int body_id, const std::string & topic_name, mjtNum vis_scale);

  /** \brief Constructor.
      \param msg external force message
  */
  void callback(const mujoco_ros_utils::ExternalForce::ConstPtr & msg);

protected:
  //! ROS node handle
  std::shared_ptr<ros::NodeHandle> nh_;

  //! ROS callback queue
  ros::CallbackQueue callbackQueue_;

  //! ROS publisher for external force
  ros::Subscriber sub_;

  //! Body ID
  int body_id_ = -1;

  //! Topic name of external force
  std::string topic_name_;

  //! External force message
  std::shared_ptr<mujoco_ros_utils::ExternalForce> msg_;

  //! End time to apply external force (-1 if no external force is applied)
  mjtNum end_time_ = -1;

  //! Arrow length scale (negative value for no visualization)
  mjtNum vis_scale_;
};

} // namespace MujocoRosUtils
