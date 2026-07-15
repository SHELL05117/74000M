#pragma once

#include <cstdint>

namespace robot {

struct BuildInfo {
  static constexpr std::uint32_t kConfigSchemaVersion = 1;
  static constexpr const char* kProjectName = "74000M";
  static constexpr const char* kLanguageStandard = "C++17";
};

const BuildInfo& buildInfo() noexcept;

}  // namespace robot
