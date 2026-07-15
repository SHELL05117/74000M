#pragma once

#include "robot/commands/scheduler.hpp"
#include "robot/drive/drive_request.hpp"

namespace robot {

enum SinkReject : std::uint32_t {
  kSinkAccepted = 0,
  kSinkFrameMismatch = 1u << 0,
  kSinkBadLease = 1u << 1,
  kSinkBadRequest = 1u << 2,
  kSinkAlreadyPublished = 1u << 3,
};

class DriveRequestSink {
 public:
  void beginFrame(const FrameHeader& header) noexcept {
    header_ = header;
    request_ = {};
    reject_bits_ = 0;
    published_ = false;
  }

  bool publish(const DriveRequest& request,
               const LeaseAuthority& authority) noexcept {
    std::uint32_t reject{};
    if (request.h.time_us != header_.time_us ||
        request.h.sequence != header_.sequence ||
        request.h.mode_epoch != header_.mode_epoch)
      reject |= kSinkFrameMismatch;
    if (!authority.owns(request.owner)) reject |= kSinkBadLease;
    if (request.source == RequestSource::None || request.ttl_us == 0 ||
        !finitePayload(request.payload) ||
        (request.owner.requirements & Requirement::kDrivetrain) == 0)
      reject |= kSinkBadRequest;
    if (published_) reject |= kSinkAlreadyPublished;
    reject_bits_ |= reject;
    if (reject != kSinkAccepted) return false;
    request_ = request;
    published_ = true;
    return true;
  }

  bool read(DriveRequest& request) const noexcept {
    if (!published_) return false;
    request = request_;
    return true;
  }

  std::uint32_t rejectBits() const noexcept { return reject_bits_; }

 private:
  FrameHeader header_{};
  DriveRequest request_{};
  std::uint32_t reject_bits_{};
  bool published_{};
};

}  // namespace robot
