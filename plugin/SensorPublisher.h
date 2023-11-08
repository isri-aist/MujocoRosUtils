#pragma once

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <mujoco_ros_utils/ScalarStamped.h>
#include <sstream>
#include <string>
#include <vector>

namespace MujocoRosUtils
{

/** \brief Checks that a plugin config attribute exists. */
inline bool checkAttr(const std::string & input)
{
  char * end;
  std::string value = input;
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

/** \brief Converts a string into a numeric vector. */
template<typename T>
inline void readVector(std::vector<T> & output, const std::string & input)
{
  std::stringstream ss(input);
  std::string item;
  char delim = ' ';
  while(getline(ss, item, delim))
  {
    if(!checkAttr(item))
    {
      continue;
    }
    output.push_back(static_cast<T>(strtod(item.c_str(), nullptr)));
  }
}

/** \brief Tactile sensor.

    A tactile sensor is attached to a site and senses contact normal forces between the site's parent body and all other
   bodies.
 */
class SensorPublisher
{
public:
  /** \brief Sensor surface type. */
  typedef enum SurfaceType_
  {
    //! Plane surface
    SurfacePlane = 0,

    //! Cylindrical surface
    SurfaceCylinder
  } SurfaceType;

  /** \brief Sensor grid type. */
  typedef enum GridType_
  {
    //! Square grid
    GridSquare = 0,

    //! Hexagonal grid
    GridHex
  } GridType;

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

  /** \brief Destructor. */
  ~SensorPublisher();

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
      \param topic_name topic name
   */
  SensorPublisher(const mjModel * m, mjData * d, int sensor_id, std::string sensor_name, std::string topic_name);

  /** \brief initSensors.
    \param model model
    \param d data
    \param sensor_id sensor ID
  */
  void initSensors(const mjModel * model, std::string topic_name);

protected:
  //! Sensor ID
  int sensor_id_ = -1;

  //! Sensor name
  std::string sensor_name_ = "";

  //! Site ID
  int site_id_ = -1;

  //! List of sensor positions in the site frame
  mjtNum * sensor_pos_list_;

  //! List of sensor normal directions in the site frame
  mjtNum * sensor_normal_list_;

  //! Frame ID of topic
  std::string frame_id_ = "";

  //! ROS node handle
  std::shared_ptr<ros::NodeHandle> nh_;

  //! ROS publisher
  ros::Publisher pub_;

  //! ROS publisher and sensor name
  std::map<std::string, std::pair<ros::Publisher, std::string>> sensor_map_;

  //! Sensor name
  const std::map<int, std::string> SENSOR_STRING = {{mjSENS_TOUCH, "touch"},
                                                    {mjSENS_ACCELEROMETER, "accelerometer"},
                                                    {mjSENS_VELOCIMETER, "velocimeter"},
                                                    {mjSENS_GYRO, "gyro"},
                                                    {mjSENS_FORCE, "force"},
                                                    {mjSENS_TORQUE, "torque"},
                                                    {mjSENS_MAGNETOMETER, "magnetometer"},
                                                    {mjSENS_RANGEFINDER, "rangefinder"},
                                                    {mjSENS_JOINTPOS, "jointpos"},
                                                    {mjSENS_JOINTVEL, "jointvel"},
                                                    {mjSENS_TENDONPOS, "tendonpos"},
                                                    {mjSENS_TENDONVEL, "tendonvel"},
                                                    {mjSENS_ACTUATORPOS, "actuatorpos"},
                                                    {mjSENS_ACTUATORVEL, "actuatorvel"},
                                                    {mjSENS_ACTUATORFRC, "actuatorfrc"},
                                                    {mjSENS_JOINTACTFRC, "jointactfrc"},
                                                    {mjSENS_BALLQUAT, "ballquat"},
                                                    {mjSENS_BALLANGVEL, "ballangvel"},
                                                    {mjSENS_JOINTLIMITPOS, "jointlimitpos"},
                                                    {mjSENS_JOINTLIMITVEL, "jointlimitvel"},
                                                    {mjSENS_JOINTLIMITFRC, "jointlimitfrc"},
                                                    {mjSENS_TENDONLIMITPOS, "tendonlimitpos"},
                                                    {mjSENS_TENDONLIMITVEL, "tendonlimitvel"},
                                                    {mjSENS_TENDONLIMITFRC, "tendonlimitfrc"},
                                                    {mjSENS_FRAMEPOS, "framepos"},
                                                    {mjSENS_FRAMEQUAT, "framequat"},
                                                    {mjSENS_FRAMEXAXIS, "framexaxis"},
                                                    {mjSENS_FRAMEYAXIS, "frameyaxis"},
                                                    {mjSENS_FRAMEZAXIS, "framezaxis"},
                                                    {mjSENS_FRAMELINVEL, "framelinvel"},
                                                    {mjSENS_FRAMEANGVEL, "frameangvel"},
                                                    {mjSENS_FRAMELINACC, "framelinacc"},
                                                    {mjSENS_FRAMEANGACC, "frameangacc"},
                                                    {mjSENS_SUBTREECOM, "subtreecom"},
                                                    {mjSENS_SUBTREELINVEL, "subtreelinvel"},
                                                    {mjSENS_SUBTREEANGMOM, "subtreeangmom"}};
  // const std::map<std::string , int> SENSOR_STRING = {{"touch",mjSENS_TOUCH},
  //                                                   {"accelerometer",mjSENS_ACCELEROMETER},
  //                                                   {"velocimeter",mjSENS_VELOCIMETER},
  //                                                   {"gyro",mjSENS_GYRO},
  //                                                   {"force",mjSENS_FORCE},
  //                                                   {"torque",mjSENS_TORQUE},
  //                                                   {"magnetometer",mjSENS_MAGNETOMETER},
  //                                                   {"rangefinder",mjSENS_RANGEFINDER},
  //                                                   {"jointpos",mjSENS_JOINTPOS},
  //                                                   {"jointvel",mjSENS_JOINTVEL},
  //                                                   {"tendonpos",mjSENS_TENDONPOS},
  //                                                   {"tendonvel",mjSENS_TENDONVEL},
  //                                                   {"actuatorpos",mjSENS_ACTUATORPOS},
  //                                                   {"actuatorvel",mjSENS_ACTUATORVEL},
  //                                                   {"actuatorfrc",mjSENS_ACTUATORFRC},
  //                                                   {"jointactfrc",mjSENS_JOINTACTFRC},
  //                                                   {"ballquat",mjSENS_BALLQUAT},
  //                                                   { "ballangvel",mjSENS_BALLANGVEL},
  //                                                   {"jointlimitpos",mjSENS_JOINTLIMITPOS},
  //                                                   {"jointlimitvel",mjSENS_JOINTLIMITVEL},
  //                                                   {"jointlimitfrc",mjSENS_JOINTLIMITFRC},
  //                                                   {"tendonlimitpos",mjSENS_TENDONLIMITPOS},
  //                                                   {"tendonlimitvel",mjSENS_TENDONLIMITVEL},
  //                                                   {"tendonlimitfrc",mjSENS_TENDONLIMITFRC},
  //                                                   {"framepos",mjSENS_FRAMEPOS},
  //                                                   {"framequat",mjSENS_FRAMEQUAT},
  //                                                   {"framexaxis",mjSENS_FRAMEXAXIS},
  //                                                   {"frameyaxis",mjSENS_FRAMEYAXIS},
  //                                                   {"framezaxis",mjSENS_FRAMEZAXIS},
  //                                                   {"framelinvel",mjSENS_FRAMELINVEL},
  //                                                   {"frameangvel",mjSENS_FRAMEANGVEL},
  //                                                   {"framelinacc",mjSENS_FRAMELINACC},
  //                                                   {"frameangacc",mjSENS_FRAMEANGACC},
  //                                                   {"subtreecom",mjSENS_SUBTREECOM},
  //                                                   {"subtreelinvel",mjSENS_SUBTREELINVEL},
  //                                                   {"subtreeangmom",mjSENS_SUBTREEANGMOM}};
};

} // namespace MujocoRosUtils
