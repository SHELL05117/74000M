#include "robot/hmi/edit_lease.hpp"
#include "robot/hmi/event_queue.hpp"
#include "robot/hmi/parameter_transaction.hpp"
#include "robot/hmi/persistence.hpp"
#include "robot/hmi/render_models.hpp"
#include "robot/hmi/selection_manager.hpp"
#include "test_framework.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace {

const robot::RouteRegistry<2>& routes() {
  static const std::array<robot::RouteDescriptor, 2> values{{
      {robot::RouteIds::kDoNothing, "DO-NOTHING", "Do Nothing",
       robot::kAllianceAny, robot::kStartAny, {},
       robot::RouteQualityRequirement::None, 0, true, true},
      {0x101, "R-NEAR", "Red Near", robot::kAllianceRed,
       robot::kStartNear, {1.0, 2.0, 0.5},
       robot::RouteQualityRequirement::FullPoseGood, 1, false, false},
  }};
  static const robot::RouteRegistry<2> registry(values, 4);
  return registry;
}

const robot::ParameterRegistry<2>& parameters() {
  static const std::array<robot::ParameterDescriptor, 2> values{{
      {1, 1, "TURN-KP", "Turn proportional gain", robot::ParameterUnit::None,
       0.0, 10.0, 0.1, 1.0, 1.0, robot::ParameterAccess::BenchEditable,
       robot::ApplyPolicy::ImmediateWhenDisabled,
       robot::PersistencePolicy::UserProfile, true},
      {2, 1, "DRIVE-KP", "Drive proportional gain",
       robot::ParameterUnit::None, 0.0, 20.0, 0.5, 2.0, 2.0,
       robot::ParameterAccess::BenchEditable,
       robot::ApplyPolicy::ResetControllerHistory,
       robot::PersistencePolicy::UserProfile, true},
  }};
  static const robot::ParameterRegistry<2> registry(values, 8);
  return registry;
}

class FakeParameterBackend final : public robot::ParameterBackend {
 public:
  bool read(robot::ParameterId id, double& value) const noexcept override {
    if (id < 1 || id > 2) return false;
    value = values_[id - 1];
    return true;
  }

  bool validate(robot::ParameterId id, double value) const noexcept override {
    return id >= 1 && id <= 2 && value >= 0.0 && value <= 20.0;
  }

  bool apply(const robot::ParameterValue* values, std::size_t count,
             robot::ApplyPolicy strongest_policy) noexcept override {
    if (reject_apply_) return false;
    for (std::size_t i = 0; i < count; ++i)
      if (!validate(values[i].id, values[i].value)) return false;
    auto next = values_;
    for (std::size_t i = 0; i < count; ++i)
      next[values[i].id - 1] = values[i].value;
    values_ = next;
    strongest_policy_ = strongest_policy;
    ++revision_;
    return true;
  }

  std::uint32_t revision() const noexcept override { return revision_; }
  double value(std::size_t index) const noexcept { return values_[index]; }
  robot::ApplyPolicy strongestPolicy() const noexcept {
    return strongest_policy_;
  }
  void advanceRevision() noexcept { ++revision_; }

 private:
  std::array<double, 2> values_{1.0, 2.0};
  robot::ApplyPolicy strongest_policy_{
      robot::ApplyPolicy::ImmediateWhenDisabled};
  std::uint32_t revision_{1};
  bool reject_apply_{};
};

class MemorySlot final : public robot::ParameterProfileSlot {
 public:
  bool read(std::uint8_t* destination, std::size_t capacity,
            std::size_t& size) const noexcept override {
    if (!written_ || size_ > capacity) return false;
    for (std::size_t i = 0; i < size_; ++i) destination[i] = bytes_[i];
    size = size_;
    return true;
  }

  bool write(const std::uint8_t* source, std::size_t size) noexcept override {
    if (size > bytes_.size()) return false;
    for (std::size_t i = 0; i < size; ++i) bytes_[i] = source[i];
    size_ = size;
    written_ = true;
    return true;
  }

  void corrupt(std::size_t index) noexcept {
    if (index < size_) bytes_[index] ^= 0x5Au;
  }

 private:
  std::array<std::uint8_t, 128> bytes_{};
  std::size_t size_{};
  bool written_{};
};

robot::ModeSnapshot disabledMode(bool field_connected = false) {
  return {robot::CompetitionMode::Disabled, false, field_connected, 7, 0, 0};
}

}  // namespace

ROBOT_TEST("HMI route selection distinguishes draft confirm lock and fallback") {
  ROBOT_REQUIRE(routes().valid());
  robot::SelectionManager<2> selection(routes());
  const auto disabled = disabledMode();
  ROBOT_REQUIRE(selection.stage(0x101, robot::Alliance::Red,
                                robot::StartSide::Near, disabled));
  ROBOT_REQUIRE(!selection.confirm(disabled));
  auto locked = selection.lockForEnable(disabled);
  ROBOT_REQUIRE(locked.valid);
  ROBOT_REQUIRE(locked.fallback_to_do_nothing);
  ROBOT_REQUIRE(locked.route_id == robot::RouteIds::kDoNothing);

  selection.onDisabled();
  ROBOT_REQUIRE(selection.stage(robot::RouteIds::kDoNothing,
                                robot::Alliance::Blue,
                                robot::StartSide::Far, disabled));
  ROBOT_REQUIRE(selection.confirm(disabled));
  locked = selection.lockForEnable(disabled);
  ROBOT_REQUIRE(locked.valid && !locked.fallback_to_do_nothing);
}

ROBOT_TEST("HMI event queue coalesces navigation and protects critical input") {
  robot::HmiEventQueue<2> queue;
  robot::HmiEvent navigation{{100, 1, 7}, robot::HmiOrigin::Brain,
                             robot::HmiAction::Navigate, 1, 0};
  ROBOT_REQUIRE(queue.push(navigation));
  navigation.item_id = 2;
  ROBOT_REQUIRE(queue.push(navigation));
  ROBOT_REQUIRE(queue.size() == 1);
  robot::HmiEvent ordinary{{101, 2, 7}, robot::HmiOrigin::Controller,
                           robot::HmiAction::AcknowledgeFault, 0, 0};
  ROBOT_REQUIRE(queue.push(ordinary));
  robot::HmiEvent critical{{102, 3, 7}, robot::HmiOrigin::Brain,
                           robot::HmiAction::RequestFaultClear, 0, 0};
  ROBOT_REQUIRE(queue.push(critical));
  ROBOT_REQUIRE(queue.size() == 2);
  ROBOT_REQUIRE(queue.stats().navigation_dropped == 1);
  robot::HmiEvent popped{};
  ROBOT_REQUIRE(queue.pop(popped));
  ROBOT_REQUIRE(popped.action == robot::HmiAction::AcknowledgeFault);
}

ROBOT_TEST("HMI edit leases reject split brain and expire by epoch") {
  robot::EditLeaseManager manager;
  robot::EditLease brain{};
  robot::EditLease controller{};
  ROBOT_REQUIRE(manager.acquire(robot::EditDomain::Parameters,
                                robot::HmiOrigin::Brain, 100, 1000, 7, brain));
  ROBOT_REQUIRE(!manager.acquire(robot::EditDomain::Parameters,
                                 robot::HmiOrigin::Controller, 100, 1000, 7,
                                 controller));
  ROBOT_REQUIRE(manager.owns(brain, 1100, 7));
  manager.expire(1101, 7);
  ROBOT_REQUIRE(!manager.owns(brain, 1101, 7));
  ROBOT_REQUIRE(manager.acquire(robot::EditDomain::Parameters,
                                robot::HmiOrigin::Controller, 1101, 1000, 8,
                                controller));
  manager.releaseOwner(robot::HmiOrigin::Controller);
  ROBOT_REQUIRE(!manager.owns(controller, 1101, 8));
}

ROBOT_TEST("parameter transaction applies atomically only under bench gates") {
  ROBOT_REQUIRE(parameters().valid());
  FakeParameterBackend backend;
  robot::EditLeaseManager leases;
  robot::EditLease lease{};
  ROBOT_REQUIRE(leases.acquire(robot::EditDomain::Parameters,
                               robot::HmiOrigin::Brain, 100, 5000, 7, lease));
  robot::ParameterTransaction<2, 2> transaction(parameters(), backend);
  ROBOT_REQUIRE(transaction.begin(lease, 100, 3000, 7));
  ROBOT_REQUIRE(transaction.stage(1, 2, false));
  ROBOT_REQUIRE(transaction.stage(2, 2, true));
  double staged{};
  ROBOT_REQUIRE(transaction.stagedValue(1, staged));
  ROBOT_REQUIRE_NEAR(staged, 1.2, 1e-12);

  robot::ParameterTransactionContext context{
      {disabledMode(true), true, true, true}, 200};
  auto rejected = transaction.apply(context, leases);
  ROBOT_REQUIRE(!rejected.applied);
  ROBOT_REQUIRE((rejected.reject_bits & robot::kParameterApplyPermission) != 0);
  ROBOT_REQUIRE_NEAR(backend.value(0), 1.0, 1e-12);

  context.permission.mode = disabledMode(false);
  const auto applied = transaction.apply(context, leases);
  ROBOT_REQUIRE(applied.applied);
  ROBOT_REQUIRE_NEAR(backend.value(0), 1.2, 1e-12);
  ROBOT_REQUIRE_NEAR(backend.value(1), 6.0, 1e-12);
  ROBOT_REQUIRE(backend.strongestPolicy() ==
                robot::ApplyPolicy::ResetControllerHistory);
  ROBOT_REQUIRE(!transaction.active());
}

ROBOT_TEST("parameter transaction rejects stale revision without partial write") {
  FakeParameterBackend backend;
  robot::EditLeaseManager leases;
  robot::EditLease lease{};
  ROBOT_REQUIRE(leases.acquire(robot::EditDomain::Parameters,
                               robot::HmiOrigin::Controller, 10, 1000, 7,
                               lease));
  robot::ParameterTransaction<2, 2> transaction(parameters(), backend);
  ROBOT_REQUIRE(transaction.begin(lease, 10, 900, 7));
  ROBOT_REQUIRE(transaction.stage(1, 5, false));
  backend.advanceRevision();
  const robot::ParameterTransactionContext context{
      {disabledMode(), true, true, true}, 20};
  const auto rejected = transaction.apply(context, leases);
  ROBOT_REQUIRE(!rejected.applied);
  ROBOT_REQUIRE((rejected.reject_bits & robot::kParameterApplyRevision) != 0);
  ROBOT_REQUIRE_NEAR(backend.value(0), 1.0, 1e-12);
}

ROBOT_TEST("dual slot profile persistence falls back after CRC corruption") {
  FakeParameterBackend backend;
  MemorySlot slot_a;
  MemorySlot slot_b;
  robot::DualSlotParameterStore<2, 2> store(parameters(), backend, slot_a,
                                            slot_b, 0x74000u);
  robot::ParameterProfile<2> profile{};
  profile.state = robot::ProfileState::Draft;
  profile.values[0] = {1, 1.5};
  profile.values[1] = {2, 3.0};
  profile.count = 2;
  ROBOT_REQUIRE(store.save(profile));
  profile.values[0].value = 2.5;
  ROBOT_REQUIRE(store.save(profile));
  robot::ParameterProfile<2> loaded{};
  ROBOT_REQUIRE(store.load(loaded));
  ROBOT_REQUIRE(loaded.generation == 2);
  ROBOT_REQUIRE_NEAR(loaded.values[0].value, 2.5, 1e-12);
  slot_b.corrupt(5);
  ROBOT_REQUIRE(store.load(loaded));
  ROBOT_REQUIRE(loaded.generation == 1);
  ROBOT_REQUIRE_NEAR(loaded.values[0].value, 1.5, 1e-12);

  robot::DualSlotParameterStore<2, 2> wrong_robot(
      parameters(), backend, slot_a, slot_b, 0x1234u);
  ROBOT_REQUIRE(!wrong_robot.load(loaded));
}

ROBOT_TEST("controller frames are fixed printable ASCII and brain targets fit") {
  robot::HmiModel model{};
  model.h.sequence = 42;
  model.mode = robot::CompetitionMode::Disabled;
  model.battery_V = 8.2;
  model.output_derate = 0.75;
  model.pose = {1.234, -2.345, 0.25};
  model.velocity.vx_mps = 0.5;
  robot::copyFixed("74000M", model.team_number);
  robot::copyFixed("IMU STALE", model.highest_fault_short);
  for (std::uint8_t page = 0; page <= 7; ++page) {
    model.controller_page = static_cast<robot::ControllerPage>(page);
    const auto frame = robot::formatControllerFrame(model);
    ROBOT_REQUIRE(frame.model_sequence == 42);
    for (const auto& line : frame.lines)
      ROBOT_REQUIRE(robot::printableAsciiLine(line));
  }
  ROBOT_REQUIRE(robot::validBrainLayout(robot::BrainLayout{}));
}
