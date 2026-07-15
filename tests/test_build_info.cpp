#include "robot/core/build_info.hpp"
#include "test_framework.hpp"

#include <string_view>

ROBOT_TEST("build metadata is available to PC tests") {
  const auto& info = robot::buildInfo();
  (void)info;
  ROBOT_REQUIRE(robot::BuildInfo::kConfigSchemaVersion == 1);
  ROBOT_REQUIRE(std::string_view(robot::BuildInfo::kProjectName) == "74000M");
  ROBOT_REQUIRE(std::string_view(robot::BuildInfo::kLanguageStandard) ==
                "C++17");
}
