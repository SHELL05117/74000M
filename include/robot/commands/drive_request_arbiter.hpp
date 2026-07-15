#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "robot/commands/scheduler.hpp"
#include "robot/drive/drive_request.hpp"
#include "robot/runtime/mode.hpp"

namespace robot {

enum ArbitrationReject : std::uint32_t {
  kArbitrationAccepted = 0,
  kArbitrationModeDisabled = 1u << 0,
  kArbitrationWrongSource = 1u << 1,
  kArbitrationEpochMismatch = 1u << 2,
  kArbitrationFuture = 1u << 3,
  kArbitrationStale = 1u << 4,
  kArbitrationBadPayload = 1u << 5,
  kArbitrationUnsupported = 1u << 6,
  kArbitrationBadLease = 1u << 7,
  kArbitrationBadSafetyRequest = 1u << 8,
};

struct DriveRequestCandidate {
  DriveRequest request{};
  bool present{};
};

struct ArbitrationResult {
  DriveRequest selected{};
  std::uint32_t reject_bits{};
  std::uint32_t rejected_count{};
  bool has_selection{};
};

struct DriveRequestArbiterConfig {
  TimeUs max_request_ttl_us{};
};

class DriveRequestArbiter {
 public:
  explicit DriveRequestArbiter(DriveRequestArbiterConfig config) noexcept
      : config_(config) {}

  template <std::size_t N>
  ArbitrationResult select(
      const std::array<DriveRequestCandidate, N>& candidates,
      const ModeSnapshot& mode, TimeUs now_us,
      const DriveCapabilities& capabilities,
      const LeaseAuthority& authority) const noexcept {
    ArbitrationResult result{};
    const DriveRequest* best{};
    int best_priority = -1;
    for (const auto& candidate : candidates) {
      if (!candidate.present) continue;
      const std::uint32_t reject =
          validate(candidate.request, mode, now_us, capabilities, authority);
      if (reject != kArbitrationAccepted) {
        result.reject_bits |= reject;
        ++result.rejected_count;
        continue;
      }
      const int priority = sourcePriority(candidate.request.source);
      if (best == nullptr || priority > best_priority ||
          (priority == best_priority &&
           candidate.request.h.sequence > best->h.sequence)) {
        best = &candidate.request;
        best_priority = priority;
      }
    }
    if (best != nullptr) {
      result.selected = *best;
      result.has_selection = true;
    }
    return result;
  }

 private:
  std::uint32_t validate(const DriveRequest& request,
                         const ModeSnapshot& mode, TimeUs now_us,
                         const DriveCapabilities& capabilities,
                         const LeaseAuthority& authority) const noexcept {
    std::uint32_t reject{};
    if (!mode.enabled) reject |= kArbitrationModeDisabled;
    if (request.h.mode_epoch != mode.epoch ||
        request.owner.mode_epoch != mode.epoch)
      reject |= kArbitrationEpochMismatch;
    if (request.h.time_us > now_us) {
      reject |= kArbitrationFuture;
    } else if (request.ttl_us == 0 || config_.max_request_ttl_us == 0 ||
               request.ttl_us > config_.max_request_ttl_us ||
               now_us - request.h.time_us > request.ttl_us) {
      reject |= kArbitrationStale;
    }
    if (!finitePayload(request.payload)) reject |= kArbitrationBadPayload;

    if (request.source == RequestSource::Safety) {
      if (!std::holds_alternative<BrakePayload>(request.payload))
        reject |= kArbitrationBadSafetyRequest;
      return reject;
    }
    if (!sourceAllowed(request.source, mode.mode))
      reject |= kArbitrationWrongSource;
    if (!supportedPayload(request.payload, capabilities, request.source))
      reject |= kArbitrationUnsupported;
    if (!authority.owns(request.owner) ||
        (request.owner.requirements & Requirement::kDrivetrain) == 0)
      reject |= kArbitrationBadLease;
    return reject;
  }

  static bool sourceAllowed(RequestSource source,
                            CompetitionMode mode) noexcept {
    if (source == RequestSource::Driver)
      return mode == CompetitionMode::Driver;
    if (source == RequestSource::FutureAutonomy)
      return mode == CompetitionMode::AutonomousInterface;
    if (source == RequestSource::Test)
      return mode == CompetitionMode::Test;
    return false;
  }

  static int sourcePriority(RequestSource source) noexcept {
    switch (source) {
      case RequestSource::Safety:
        return 4;
      case RequestSource::Test:
        return 3;
      case RequestSource::FutureAutonomy:
        return 2;
      case RequestSource::Driver:
        return 1;
      case RequestSource::None:
        return 0;
    }
    return 0;
  }

  DriveRequestArbiterConfig config_{};
};

}  // namespace robot
