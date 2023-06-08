#include <mujoco/mjplugin.h>

#include "ClockPublisher.h"
#include "PosePublisher.h"

namespace MujocoRosUtils
{

mjPLUGIN_LIB_INIT
{
  ClockPublisher::RegisterPlugin();
  PosePublisher::RegisterPlugin();
}

} // namespace MujocoRosUtils
