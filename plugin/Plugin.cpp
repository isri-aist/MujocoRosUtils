#include <mujoco/mjplugin.h>

#include "ActuatorCommand.h"
#include "ClockPublisher.h"
#include "ExternalForce.h"
#include "ImagePublisher.h"
#include "PosePublisher.h"
#include "SensorPublisher.h"

mjPLUGIN_LIB_INIT
{
  MujocoRosUtils::ClockPublisher::RegisterPlugin();
  MujocoRosUtils::PosePublisher::RegisterPlugin();
  MujocoRosUtils::ImagePublisher::RegisterPlugin();
  MujocoRosUtils::ActuatorCommand::RegisterPlugin();
  MujocoRosUtils::ExternalForce::RegisterPlugin();
  MujocoRosUtils::SensorPublisher::RegisterPlugin();
}
