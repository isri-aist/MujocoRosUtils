#include "SensorPublisher.h"

#include <mujoco/mujoco.h>

#include <iostream>

namespace MujocoRosUtils
{
void SensorPublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::SensorPublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  const char * attributes[] = {"sensor_name", "topic_name", "publish_rate", "use_sim_time"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel * m, int plugin_id,
                           int // sensor_id
                        ) { return 0; };

  // Can only run after forces have been computed
  plugin.needstage = mjSTAGE_ACC;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id) {
    auto * plugin_instance = SensorPublisher::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id) {
    delete reinterpret_cast<SensorPublisher *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id) {
    auto * plugin_instance = reinterpret_cast<class SensorPublisher *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    ) {
    auto * plugin_instance = reinterpret_cast<class SensorPublisher *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

SensorPublisher * SensorPublisher::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // sensor_name
  const char * sensor_name_char = mj_getPluginConfig(m, plugin_id, "sensor_name");
  if(strlen(sensor_name_char) == 0)
  {
    mju_error("[SensorPublisher] `sensor_name` is missing.");
    return nullptr;
  }

  std::vector<std::string> sensor_name_list;
  for(int n = 0; n < m->nsensor; n++)
  {
    if(m->names[m->name_sensoradr[n]])
    {
      sensor_name_list.push_back(mj_id2name(m, mjOBJ_SENSOR, n));
    }
  }

  if((std::find(sensor_name_list.begin(), sensor_name_list.end(), std::string(sensor_name_char)))
     == sensor_name_list.end())
  {
    mju_error("[SensorPublisher] The sensor name is not found in sensors.");
    return nullptr;
  }

  int target_sensor_id = static_cast<int>(
      std::distance(sensor_name_list.begin(),
                    (std::find(sensor_name_list.begin(), sensor_name_list.end(), std::string(sensor_name_char)))));

  // Set sensor_id
  int sensor_id = 0;
  for(; sensor_id < m->nsensor; sensor_id++)
  {
    if(m->sensor_type[sensor_id] == mjSENS_PLUGIN && m->sensor_plugin[sensor_id] == plugin_id)
    {
      break;
    }
  }
  if(sensor_id == m->nsensor)
  {
    mju_error("[SensorPublisher] Plugin not found in sensors.");
    return nullptr;
  }

  // topic_name
  const char * topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  std::string topic_name = "";
  if(strlen(topic_name_char) > 0)
  {
    topic_name = std::string(topic_name_char);
  }

  // publish_rate
  const char * publish_rate_char = mj_getPluginConfig(m, plugin_id, "publish_rate");
  mjtNum publish_rate = 100.0;
  if(strlen(publish_rate_char) > 0)
  {
    publish_rate = strtod(publish_rate_char, nullptr);
  }
  if(publish_rate <= 0)
  {
    mju_error("[SensorPublisher] `publish_rate` must be positive.");
    return nullptr;
  }

  // use_sim_time
  const char * use_sim_time_char = mj_getPluginConfig(m, plugin_id, "use_sim_time");
  bool use_sim_time = true;
  if(strlen(use_sim_time_char) > 0)
  {
    if(!(strcmp(use_sim_time_char, "true") == 0 || strcmp(use_sim_time_char, "false") == 0))
    {
      mju_error("[SensorPublisher] `use_sim_time` must be `true` or `false`.");
      return nullptr;
    }
    use_sim_time = (strcmp(use_sim_time_char, "true") == 0);
  }

  std::cout << "[SensorPublisher] Create." << std::endl;

  return new SensorPublisher(m, d, sensor_id, target_sensor_id, sensor_name_char, topic_name_char, publish_rate,
                             use_sim_time);
}

SensorPublisher::SensorPublisher(const mjModel * m,
                                 mjData *, // d
                                 int sensor_id,
                                 int target_sensor_id,
                                 std::string sensor_name,
                                 std::string topic_name,
                                 mjtNum publish_rate,
                                 bool use_sim_time)
: sensor_name_(sensor_name), sensor_id_(sensor_id), target_sensor_id_(target_sensor_id),
  site_id_(m->sensor_objid[sensor_id]),
  publish_skip_(std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1)), use_sim_time_(use_sim_time)
{
  if(topic_name.empty())
  {
    std::string sensor_name = std::string(mj_id2name(m, mjOBJ_SENSOR, sensor_id));
    topic_name = "mujoco/" + sensor_name;
  }

  int argc = 0;
  char ** argv = nullptr;
  if(!ros::isInitialized())
  {
    ros::init(argc, argv, "mujoco_ros", ros::init_options::NoSigintHandler);
  }

  nh_ = std::make_shared<ros::NodeHandle>();
  nh_->setParam("/use_sim_time", use_sim_time_);
  initSensors(m, topic_name);
}

SensorPublisher::~SensorPublisher() {}

void SensorPublisher::initSensors(const mjModel * model, std::string topic_name)
{
  std::string sensor_name, site, frame_id;
  int site_id = model->sensor_objid[target_sensor_id_];
  int parent_id = model->site_bodyid[site_id];
  int type = model->sensor_type[target_sensor_id_];

  // load only candidate sensors
  if(SENSOR_STRING.find(type) != SENSOR_STRING.end())
  {
    site = mj_id2name(model, model->sensor_objtype[target_sensor_id_], site_id);

    if(model->names[model->name_sensoradr[target_sensor_id_]])
    {
      sensor_name = mj_id2name(model, mjOBJ_SENSOR, target_sensor_id_);
    }
    else
    {
      std::cerr << "Sensor name resolution error. Skipping sensor of type " << type << " on site " << site << std::endl;
      return;
    }

    // Global frame sensors
    bool global_frame = false;
    frame_id = "world";
    switch(type)
    {
      {
        case mjSENS_FRAMEXAXIS:
        case mjSENS_FRAMEYAXIS:
        case mjSENS_FRAMEZAXIS:
        case mjSENS_FRAMELINVEL:
        case mjSENS_FRAMELINACC:
        case mjSENS_FRAMEANGACC:
          int refid = model->sensor_refid[target_sensor_id_];
          if(refid != -1)
          {
            int reftype = model->sensor_reftype[target_sensor_id_];
            if(reftype == mjOBJ_SITE)
            {
              refid = model->site_bodyid[refid];
              reftype = mjOBJ_BODY;
            }
            frame_id = mj_id2name(model, reftype, refid);
            std::cerr << "Sensor has relative frame with id " << refid << " and type " << reftype << " and ref_frame "
                      << frame_id << std::endl;
          }
          sensor_map_[sensor_name] =
              std::pair(nh_->advertise<geometry_msgs::Vector3Stamped>(topic_name, 1, true), frame_id);
          break;
      }
      case mjSENS_SUBTREECOM:
      case mjSENS_SUBTREELINVEL:
      case mjSENS_SUBTREEANGMOM:
        sensor_map_[sensor_name] =
            std::pair(nh_->advertise<geometry_msgs::Vector3Stamped>(topic_name, 1, true), frame_id);
        global_frame = true;
        break;
        {
          case mjSENS_FRAMEPOS:
            int refid = model->sensor_refid[target_sensor_id_];
            if(refid != -1)
            {
              int reftype = model->sensor_reftype[target_sensor_id_];
              if(reftype == mjOBJ_SITE)
              {
                refid = model->site_bodyid[refid];
                reftype = mjOBJ_BODY;
              }
              frame_id = mj_id2name(model, reftype, refid);
              std::cerr << "Sensor has relative frame with id " << refid << " and type " << reftype << " and ref_frame "
                        << frame_id << std::endl;
            }
            sensor_map_[sensor_name] =
                std::pair(nh_->advertise<geometry_msgs::PointStamped>(topic_name, 1, true), frame_id);
            global_frame = true;
            break;
        }

      case mjSENS_BALLQUAT:
      case mjSENS_FRAMEQUAT:
        sensor_map_[sensor_name] =
            std::pair(nh_->advertise<geometry_msgs::QuaternionStamped>(topic_name, 1, true), frame_id);
        global_frame = true;
        break;
    }

    // Check if sensor is in global frame and already setup
    if(global_frame || frame_id != "world")
    {
      std::cerr << "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: " << frame_id
                << ") of type " << SENSOR_STRING.at(type) << std::endl;
      return;
    }

    frame_id = mj_id2name(model, mjOBJ_BODY, parent_id);
    if(SENSOR_STRING.count(type))
    {
      std::cerr << "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: " << frame_id
                << ") of type " << SENSOR_STRING.at(type) << std::endl;
    }

    switch(type)
    {
      case mjSENS_ACCELEROMETER:
      case mjSENS_VELOCIMETER:
      case mjSENS_GYRO:
      case mjSENS_FORCE:
      case mjSENS_TORQUE:
      case mjSENS_MAGNETOMETER:
      case mjSENS_BALLANGVEL:
        sensor_map_[sensor_name] =
            std::pair(nh_->advertise<geometry_msgs::Vector3Stamped>(topic_name, 1, true), frame_id);
        break;

      case mjSENS_TOUCH:
      case mjSENS_RANGEFINDER:
      case mjSENS_JOINTPOS:
      case mjSENS_JOINTVEL:
      case mjSENS_TENDONPOS:
      case mjSENS_TENDONVEL:
      case mjSENS_ACTUATORPOS:
      case mjSENS_ACTUATORVEL:
      case mjSENS_ACTUATORFRC:
      case mjSENS_JOINTLIMITPOS:
      case mjSENS_JOINTLIMITVEL:
      case mjSENS_JOINTLIMITFRC:
      case mjSENS_TENDONLIMITPOS:
      case mjSENS_TENDONLIMITVEL:
      case mjSENS_TENDONLIMITFRC:
        sensor_map_[sensor_name] =
            std::pair(nh_->advertise<mujoco_ros_utils::ScalarStamped>(topic_name, 1, true), frame_id);
        break;

      default:
        std::cerr << "Sensor of type '" << type << "' (" << sensor_name << ") is unknown! Cannot publish to ROS"
                  << std::endl;
        break;
    }
  }
}

void SensorPublisher::reset(const mjModel *, // m
                            int // plugin_id
)
{
}

void SensorPublisher::compute(const mjModel * m, mjData * d, int // plugin_id
)
{
  sim_cnt_++;
  if(sim_cnt_ % publish_skip_ != 0)
  {
    return;
  }

  ros::Publisher pub;
  std::string frame_id, sensor_name;

  int adr, type;
  mjtNum cutoff;

  adr = m->sensor_adr[target_sensor_id_];
  type = m->sensor_type[target_sensor_id_];
  cutoff = (m->sensor_cutoff[target_sensor_id_] > 0 ? m->sensor_cutoff[target_sensor_id_] : 1);

  if(m->names[m->name_sensoradr[target_sensor_id_]])
  {
    sensor_name = mj_id2name(m, mjOBJ_SENSOR, target_sensor_id_);
  }
  else
  {
    return;
  }

  if(sensor_map_.find(sensor_name) == sensor_map_.end()) return;

  std::tie(pub, frame_id) = sensor_map_[sensor_name];

  switch(type)
  {
    {
      case mjSENS_FRAMELINVEL:
      case mjSENS_FRAMELINACC:
      case mjSENS_FRAMEANGACC:
      case mjSENS_SUBTREECOM:
      case mjSENS_SUBTREELINVEL:
      case mjSENS_SUBTREEANGMOM:
      case mjSENS_ACCELEROMETER:
      case mjSENS_VELOCIMETER:
      case mjSENS_GYRO:
      case mjSENS_FORCE:
      case mjSENS_TORQUE:
      case mjSENS_MAGNETOMETER:
      case mjSENS_BALLANGVEL:
      case mjSENS_FRAMEXAXIS:
      case mjSENS_FRAMEYAXIS:
      case mjSENS_FRAMEZAXIS:
        geometry_msgs::Vector3Stamped msg;
        msg.header.frame_id = frame_id;
        msg.header.stamp = ros::Time::now();
        msg.vector.x = (float)(d->sensordata[adr] / cutoff);
        msg.vector.y = (float)(d->sensordata[adr + 1] / cutoff);
        msg.vector.z = (float)(d->sensordata[adr + 2] / cutoff);
        pub.publish(msg);
        break;
    }

    case mjSENS_FRAMEPOS:
    {
      geometry_msgs::PointStamped msg;
      msg.header.frame_id = frame_id;
      msg.header.stamp = ros::Time::now();
      msg.point.x = (float)(d->sensordata[adr] / cutoff);
      msg.point.y = (float)(d->sensordata[adr + 1] / cutoff);
      msg.point.z = (float)(d->sensordata[adr + 2] / cutoff);
      pub.publish(msg);
      break;
    }

      {
        case mjSENS_TOUCH:
        case mjSENS_RANGEFINDER:
        case mjSENS_JOINTPOS:
        case mjSENS_JOINTVEL:
        case mjSENS_TENDONPOS:
        case mjSENS_TENDONVEL:
        case mjSENS_ACTUATORPOS:
        case mjSENS_ACTUATORVEL:
        case mjSENS_ACTUATORFRC:
        case mjSENS_JOINTLIMITPOS:
        case mjSENS_JOINTLIMITVEL:
        case mjSENS_JOINTLIMITFRC:
        case mjSENS_TENDONLIMITPOS:
        case mjSENS_TENDONLIMITVEL:
        case mjSENS_TENDONLIMITFRC:
          mujoco_ros_utils::ScalarStamped msg;
          msg.header.frame_id = frame_id;
          msg.header.stamp = ros::Time::now();
          msg.value = (float)(d->sensordata[adr] / cutoff);
          pub.publish(msg);
          break;
      }

    case mjSENS_BALLQUAT:
    {
      case mjSENS_FRAMEQUAT:
        geometry_msgs::QuaternionStamped msg;
        msg.header.frame_id = frame_id;
        msg.header.stamp = ros::Time::now();
        msg.quaternion.w = (float)(d->sensordata[adr] / cutoff);
        msg.quaternion.x = (float)(d->sensordata[adr + 1] / cutoff);
        msg.quaternion.y = (float)(d->sensordata[adr + 2] / cutoff);
        msg.quaternion.z = (float)(d->sensordata[adr + 3] / cutoff);
        pub.publish(msg);
        break;
    }

    default:
      std::cerr << "Sensor publisher and frame_id defined but type can't be serialized. This shouldn't happen! ("
                << sensor_name << " of type " << type << ")" << std::endl;
      break;
  }
}

} // namespace MujocoRosUtils