#include <mujoco/mjplugin.h>

#include "ClockPublisher.h"
#include "ExternalForce.h"
#include "PosePublisher.h"

namespace MujocoRosUtils
{

mjPLUGIN_LIB_INIT
{
  ClockPublisher::RegisterPlugin();
  PosePublisher::RegisterPlugin();
  ExternalForce::RegisterPlugin();
}

} // namespace MujocoRosUtils
