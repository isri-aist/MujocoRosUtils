#include "ActuatorCommand.h"

#include <mujoco/mujoco.h>

#include <iostream>

namespace MujocoRosUtils
{

void ActuatorCommand::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::ActuatorCommand";
  // Allow plugins to be placed on either the body element or the actuator element
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char * attributes[] = {"actuator_name", "topic_name"};

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel *, // m
                      int // plugin_id
                   ) { return 0; };

  plugin.nsensordata = +[](const mjModel *, // m
                           int, // plugin_id
                           int // sensor_id
                        ) { return 0; };

  plugin.needstage = mjSTAGE_VEL;

  plugin.init = +[](const mjModel * m, mjData * d, int plugin_id)
  {
    auto * plugin_instance = ActuatorCommand::Create(m, d, plugin_id);
    if(!plugin_instance)
    {
      return -1;
    }
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(plugin_instance);
    return 0;
  };

  plugin.destroy = +[](mjData * d, int plugin_id)
  {
    delete reinterpret_cast<ActuatorCommand *>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel * m, double *, // plugin_state
                     void * plugin_data, int plugin_id)
  {
    auto * plugin_instance = reinterpret_cast<class ActuatorCommand *>(plugin_data);
    plugin_instance->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel * m, mjData * d, int plugin_id, int // capability_bit
                    )
  {
    auto * plugin_instance = reinterpret_cast<class ActuatorCommand *>(d->plugin_data[plugin_id]);
    plugin_instance->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

ActuatorCommand * ActuatorCommand::Create(const mjModel * m, mjData * d, int plugin_id)
{
  // actuator_name
  const char * actuator_name_char = mj_getPluginConfig(m, plugin_id, "actuator_name");
  if(strlen(actuator_name_char) == 0)
  {
    mju_error("[ActuatorCommand] `actuator_name` is missing.");
    return nullptr;
  }
  int actuator_id = 0;
  for(; actuator_id < m->nu; actuator_id++)
  {
    if(strcmp(actuator_name_char, mj_id2name(m, mjOBJ_ACTUATOR, actuator_id)) == 0)
    {
      break;
    }
  }
  if(actuator_id == m->nu)
  {
    mju_error("[ActuatorCommand] The actuator with the specified name not found.");
    return nullptr;
  }

  // topic_name
  const char * topic_name_char = mj_getPluginConfig(m, plugin_id, "topic_name");
  std::string topic_name = "";
  if(strlen(topic_name_char) > 0)
  {
    topic_name = std::string(topic_name_char);
  }

  std::cout << "[ActuatorCommand] Create." << std::endl;

  return new ActuatorCommand(m, d, actuator_id, topic_name);
}

ActuatorCommand::ActuatorCommand(const mjModel * m,
                                 mjData *, // d
                                 int actuator_id,
                                 std::string topic_name)
: actuator_id_(actuator_id)
{
  if(topic_name.empty())
  {
    std::string actuator_name = std::string(mj_id2name(m, mjOBJ_ACTUATOR, actuator_id));
    topic_name = "mujoco/" + actuator_name;
  }

  int argc = 0;
  char ** argv = nullptr;
  if (!rclcpp::ok()) 
  {
      rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions node_options;

  nh_ = rclcpp::Node::make_shared("actuator_command", node_options);
  sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
    topic_name, 1, std::bind(&ActuatorCommand::callback, this, std::placeholders::_1));
  // // Use a dedicated queue so as not to call callbacks of other modules
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(nh_);
}

void ActuatorCommand::reset(const mjModel *, // m
                            int // plugin_id
)
{
}

void ActuatorCommand::compute(const mjModel *, // m
                              mjData * d,
                              int // plugin_id
)
{
  // Call ROS callback
  executor_->spin_once(std::chrono::seconds(0));

  // Set actuator command
  if(!std::isnan(ctrl_))
  {
    d->ctrl[actuator_id_] = ctrl_;
    ctrl_ = std::numeric_limits<mjtNum>::quiet_NaN();
  }
}

void ActuatorCommand::callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  ctrl_ = msg->data;
}

} // namespace MujocoRosUtils
