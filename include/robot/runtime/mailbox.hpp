#pragma once

#include "robot/core/snapshot_box.hpp"
#include "robot/drive/actuator_frame.hpp"
#include "robot/runtime/mode.hpp"

namespace robot {

class ModeStore {
 public:
  virtual void publish(const ModeSnapshot& mode) = 0;
  virtual ModeSnapshot read() const = 0;
  virtual ~ModeStore() = default;
};

class ActuatorStore {
 public:
  virtual void publish(const ActuatorFrame& frame) = 0;
  virtual bool readLatest(ActuatorFrame& frame) const = 0;
  virtual ~ActuatorStore() = default;
};

template <typename Mutex>
class LockedModeStore final : public ModeStore {
 public:
  void publish(const ModeSnapshot& mode) override { box_.publish(mode); }
  ModeSnapshot read() const override { return box_.read(); }

 private:
  SnapshotBox<ModeSnapshot, Mutex> box_;
};

template <typename Mutex>
class LockedActuatorStore final : public ActuatorStore {
 public:
  void publish(const ActuatorFrame& frame) override { box_.publish(frame); }

  bool readLatest(ActuatorFrame& frame) const override {
    if (!box_.initialized()) return false;
    frame = box_.read();
    return true;
  }

 private:
  SnapshotBox<ActuatorFrame, Mutex> box_;
};

}  // namespace robot
