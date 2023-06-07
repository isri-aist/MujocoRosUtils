#include <mujoco/mjplugin.h>

#include "ClockPublisher.h"
#include "PosePublisher.h"

namespace mujoco::plugin::sensor
{

mjPLUGIN_LIB_INIT
{
  ClockPublisher::RegisterPlugin();
  PosePublisher::RegisterPlugin();
}

} // namespace mujoco::plugin::sensor
