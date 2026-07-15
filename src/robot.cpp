#include "robot/core/build_info.hpp"

namespace robot {

const BuildInfo& buildInfo() noexcept {
  static const BuildInfo info{};
  return info;
}

}  // namespace robot
