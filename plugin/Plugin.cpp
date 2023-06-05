#include <mujoco/mjplugin.h>

#include "PosePublisher.h"

namespace mujoco::plugin::sensor
{

mjPLUGIN_LIB_INIT
{
  PosePublisher::RegisterPlugin();
}

} // namespace mujoco::plugin::sensor
