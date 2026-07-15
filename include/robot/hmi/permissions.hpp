#pragma once

#include "robot/hmi/types.hpp"

namespace robot {

struct HmiPermissionContext {
  ModeSnapshot mode{};
  bool bench_unlocked{};
  bool stationary{};
  bool model_fresh{};
};

inline bool disabledForHmi(const HmiPermissionContext& context) noexcept {
  return !context.mode.enabled &&
         context.mode.mode == CompetitionMode::Disabled;
}

inline bool hmiPermissionAllows(HmiAction action,
                                const HmiPermissionContext& context) noexcept {
  if (action == HmiAction::Navigate || action == HmiAction::Back ||
      action == HmiAction::AddLogMarker || action == HmiAction::ExitUiMode)
    return true;

  if (!disabledForHmi(context) || !context.model_fresh) return false;

  if (action == HmiAction::BeginAutonEdit ||
      action == HmiAction::SelectRoute || action == HmiAction::ConfirmRoute ||
      action == HmiAction::CancelRouteEdit ||
      action == HmiAction::AcknowledgeFault)
    return true;

  if (context.mode.field_connected || !context.bench_unlocked ||
      !context.stationary)
    return false;

  return action == HmiAction::BeginParameterEdit ||
         action == HmiAction::SelectParameter ||
         action == HmiAction::AdjustParameterFine ||
         action == HmiAction::AdjustParameterCoarse ||
         action == HmiAction::ApplyParameterTransaction ||
         action == HmiAction::RollbackParameterTransaction ||
         action == HmiAction::SaveParameterProfile ||
         action == HmiAction::RequestPoseReset ||
         action == HmiAction::RequestImuCalibration ||
         action == HmiAction::RequestFaultClear;
}

}  // namespace robot
