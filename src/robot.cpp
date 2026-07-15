#include "robot/robot.hpp"

namespace robot {

const BuildInfo& buildInfo() noexcept {
  static const BuildInfo info{};
  return info;
}

}  // namespace robot
