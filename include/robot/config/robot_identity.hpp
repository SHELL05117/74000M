#pragma once

#include <cstddef>
#include <cstdint>

namespace robot {

enum class VerificationLevel : std::uint8_t {
  Unverified,
  Implemented,
  PCValidated,
  SimValidated,
  HILValidated,
  FieldValidated,
  CompetitionApproved,
};

constexpr bool atLeast(VerificationLevel actual,
                       VerificationLevel required) noexcept {
  return static_cast<std::uint8_t>(actual) >=
         static_cast<std::uint8_t>(required);
}

struct RobotIdentity {
  const char* team_number{};
  const char* robot_id{};
  const char* robot_name{};
  const char* software_version{};
  std::uint32_t hardware_revision{};
  std::uint32_t config_schema{};
  std::uint32_t calibration_revision{};
};

inline bool boundedNonEmptyString(const char* value,
                                  std::size_t capacity) noexcept {
  if (value == nullptr || capacity == 0) return false;
  for (std::size_t i = 0; i < capacity; ++i) {
    if (value[i] == '\0') return i != 0;
  }
  return false;
}

inline bool boundedStringEqual(const char* left, const char* right,
                               std::size_t capacity) noexcept {
  if (left == nullptr || right == nullptr) return false;
  for (std::size_t i = 0; i < capacity; ++i) {
    if (left[i] != right[i]) return false;
    if (left[i] == '\0') return i != 0;
  }
  return false;
}

}  // namespace robot
