#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "robot/core/frame.hpp"
#include "robot/core/quality.hpp"
#include "robot/health/fault_manager.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/state/pose2d.hpp"
#include "robot/ui/registry_ids.hpp"

namespace robot {

enum class HmiPage : std::uint8_t {
  Dashboard,
  AutonSelect,
  PoseDebug,
  Health,
  ParameterList,
  ParameterEdit,
  Logs,
  Maintenance,
  About,
};

enum class ControllerPage : std::uint8_t {
  CompetitionHud,
  AutoSelect,
  PoseDebug,
  HealthSummary,
  FaultDetail,
  TimingAndLog,
  ParameterEdit,
  UiLocked,
};

enum class HmiOrigin : std::uint8_t { Brain, Controller };

enum class HmiAction : std::uint8_t {
  Navigate,
  Back,
  BeginAutonEdit,
  SelectRoute,
  ConfirmRoute,
  CancelRouteEdit,
  BeginParameterEdit,
  SelectParameter,
  AdjustParameterFine,
  AdjustParameterCoarse,
  ApplyParameterTransaction,
  RollbackParameterTransaction,
  SaveParameterProfile,
  RequestPoseReset,
  RequestImuCalibration,
  AcknowledgeFault,
  RequestFaultClear,
  AddLogMarker,
  ExitUiMode,
};

struct HmiEvent {
  FrameHeader h{};
  HmiOrigin origin{HmiOrigin::Brain};
  HmiAction action{HmiAction::Navigate};
  std::int32_t item_id{};
  std::int32_t delta_steps{};
};

enum class EditOwner : std::uint8_t { None, Brain, Controller };

struct HmiModel {
  FrameHeader h{};
  CompetitionMode mode{CompetitionMode::Boot};
  bool enabled{};
  bool field_connected{};
  std::array<char, 8> team_number{};
  Pose2d pose{};
  BodyVelocity2d velocity{};
  Quality translation_quality{Quality::Invalid};
  Quality heading_quality{Quality::Invalid};
  double battery_V{};
  double max_motor_temperature_C{};
  double output_derate{};
  std::uint32_t active_fault_count{};
  FaultSeverity highest_fault_severity{FaultSeverity::Info};
  std::array<char, 20> highest_fault_short{};
  RouteId draft_route{RouteIds::kDoNothing};
  RouteId confirmed_route{RouteIds::kDoNothing};
  bool route_confirmed{};
  bool route_locked{};
  std::uint32_t route_revision{};
  HmiPage brain_page{HmiPage::Dashboard};
  ControllerPage controller_page{ControllerPage::CompetitionHud};
  EditOwner auton_edit_owner{EditOwner::None};
  EditOwner parameter_edit_owner{EditOwner::None};
  ParameterId selected_parameter{ParameterIds::kInvalid};
  double active_value{};
  double staged_value{};
  bool parameter_dirty{};
  bool parameter_apply_allowed{};
  bool parameter_saved{};
  bool sd_inserted{};
  bool log_sink_ok{};
  std::uint32_t log_dropped_total{};
  std::uint32_t parameter_revision{};
};

template <std::size_t N>
inline bool copyFixed(const char* source, std::array<char, N>& target) noexcept {
  static_assert(N > 0, "fixed string needs a terminator");
  target.fill('\0');
  if (source == nullptr) return false;
  std::size_t i{};
  for (; i + 1 < N && source[i] != '\0'; ++i) target[i] = source[i];
  return source[i] == '\0' && i > 0;
}

}  // namespace robot
