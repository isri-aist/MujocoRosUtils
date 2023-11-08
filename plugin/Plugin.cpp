#include <mujoco/mjplugin.h>

#include "ActuatorCommand.h"
#include "ClockPublisher.h"
#include "ExternalForce.h"
#include "ImagePublisher.h"
#include "PosePublisher.h"
#include "SensorPublisher.h"

namespace MujocoRosUtils
{

mjPLUGIN_LIB_INIT
{
  ClockPublisher::RegisterPlugin();
  PosePublisher::RegisterPlugin();
  ImagePublisher::RegisterPlugin();
  ActuatorCommand::RegisterPlugin();
  ExternalForce::RegisterPlugin();
  SensorPublisher::RegisterPlugin();
}

} // namespace MujocoRosUtils
