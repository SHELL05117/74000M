#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/control/feedforward.hpp"
#include "robot/state/pose2d.hpp"

namespace robot {

enum class TravelDirection : std::int8_t { Forward = 1, Reverse = -1 };

inline double directionSign(TravelDirection direction) noexcept {
  return direction == TravelDirection::Forward ? 1.0 : -1.0;
}

struct PathWaypoint {
  Pose2d pose{};
  double tangent_scale_m{};  // zero selects chord length
  TravelDirection direction{TravelDirection::Forward};
};

struct TrajectoryConstraints {
  double max_velocity_mps{};
  double max_acceleration_mps2{};
  double max_deceleration_mps2{};
  double max_angular_velocity_radps{};
  double max_centripetal_acceleration_mps2{};
  double max_total_acceleration_mps2{};
  double max_voltage_V{};
  double track_width_m{};
  double spatial_step_m{};
  double initial_speed_mps{};
  double final_speed_mps{};
  DifferentialFeedforwardConfig feedforward{};
};

inline bool validTrajectoryConstraints(
    const TrajectoryConstraints& constraints) noexcept {
  return std::isfinite(constraints.max_velocity_mps) &&
         constraints.max_velocity_mps > 0.0 &&
         std::isfinite(constraints.max_acceleration_mps2) &&
         constraints.max_acceleration_mps2 > 0.0 &&
         std::isfinite(constraints.max_deceleration_mps2) &&
         constraints.max_deceleration_mps2 > 0.0 &&
         std::isfinite(constraints.max_angular_velocity_radps) &&
         constraints.max_angular_velocity_radps > 0.0 &&
         std::isfinite(constraints.max_centripetal_acceleration_mps2) &&
         constraints.max_centripetal_acceleration_mps2 > 0.0 &&
         std::isfinite(constraints.max_total_acceleration_mps2) &&
         constraints.max_total_acceleration_mps2 > 0.0 &&
         std::isfinite(constraints.max_voltage_V) &&
         constraints.max_voltage_V > 0.0 && constraints.max_voltage_V <= 12.0 &&
         std::isfinite(constraints.track_width_m) &&
         constraints.track_width_m > 0.0 &&
         std::isfinite(constraints.spatial_step_m) &&
         constraints.spatial_step_m > 0.0 &&
         std::isfinite(constraints.initial_speed_mps) &&
         constraints.initial_speed_mps >= 0.0 &&
         std::isfinite(constraints.final_speed_mps) &&
         constraints.final_speed_mps >= 0.0 &&
         constraints.initial_speed_mps <= constraints.max_velocity_mps &&
         constraints.final_speed_mps <= constraints.max_velocity_mps &&
         validFeedforwardGains(constraints.feedforward.left) &&
         validFeedforwardGains(constraints.feedforward.right);
}

struct TrajectorySample {
  double time_s{};
  double distance_m{};
  Pose2d pose{};
  double path_curvature_per_m{};
  double linear_velocity_mps{};
  double linear_acceleration_mps2{};
  double angular_velocity_radps{};
  double angular_acceleration_radps2{};
  double predicted_left_voltage_V{};
  double predicted_right_voltage_V{};
  TravelDirection direction{TravelDirection::Forward};
  bool valid{};
};

template <std::size_t Capacity>
class FixedTrajectory {
  static_assert(Capacity >= 2, "trajectory needs at least two samples");

 public:
  bool append(const TrajectorySample& sample) noexcept {
    if (size_ == Capacity || !sample.valid) return false;
    if (size_ > 0 &&
        (sample.time_s < samples_[size_ - 1].time_s ||
         sample.distance_m < samples_[size_ - 1].distance_m))
      return false;
    samples_[size_++] = sample;
    return true;
  }

  void clear() noexcept {
    samples_ = {};
    size_ = 0;
  }
  std::size_t size() const noexcept { return size_; }
  const TrajectorySample& operator[](std::size_t index) const noexcept {
    return samples_[index];
  }
  double totalTime() const noexcept {
    return size_ == 0 ? 0.0 : samples_[size_ - 1].time_s;
  }

  TrajectorySample sampleAt(double time_s) const noexcept {
    if (size_ == 0 || !std::isfinite(time_s)) return {};
    if (time_s <= samples_[0].time_s) return samples_[0];
    if (time_s >= samples_[size_ - 1].time_s) return samples_[size_ - 1];
    std::size_t high = 1;
    for (; high < size_; ++high)
      if (samples_[high].time_s >= time_s) break;
    const auto& left = samples_[high - 1];
    const auto& right = samples_[high];
    const double span = right.time_s - left.time_s;
    if (!(span > 0.0)) return {};
    const double ratio = (time_s - left.time_s) / span;
    const auto interpolate = [ratio](double a, double b) noexcept {
      return a + ratio * (b - a);
    };
    TrajectorySample sample{};
    sample.time_s = time_s;
    sample.distance_m = interpolate(left.distance_m, right.distance_m);
    sample.pose.x_m = interpolate(left.pose.x_m, right.pose.x_m);
    sample.pose.y_m = interpolate(left.pose.y_m, right.pose.y_m);
    sample.pose.theta_rad =
        left.pose.theta_rad +
        ratio * wrapPi(right.pose.theta_rad - left.pose.theta_rad);
    sample.path_curvature_per_m =
        interpolate(left.path_curvature_per_m,
                    right.path_curvature_per_m);
    sample.linear_velocity_mps =
        interpolate(left.linear_velocity_mps, right.linear_velocity_mps);
    sample.linear_acceleration_mps2 = interpolate(
        left.linear_acceleration_mps2, right.linear_acceleration_mps2);
    sample.angular_velocity_radps = interpolate(
        left.angular_velocity_radps, right.angular_velocity_radps);
    sample.angular_acceleration_radps2 = interpolate(
        left.angular_acceleration_radps2,
        right.angular_acceleration_radps2);
    sample.predicted_left_voltage_V = interpolate(
        left.predicted_left_voltage_V, right.predicted_left_voltage_V);
    sample.predicted_right_voltage_V = interpolate(
        left.predicted_right_voltage_V, right.predicted_right_voltage_V);
    sample.direction = left.direction;
    sample.valid = valid(sample.pose) &&
                   std::isfinite(sample.linear_velocity_mps) &&
                   std::isfinite(sample.angular_velocity_radps);
    return sample;
  }

 private:
  std::array<TrajectorySample, Capacity> samples_{};
  std::size_t size_{};
};

enum class TrajectoryGenerationStatus : std::uint8_t {
  Success,
  InvalidInput,
  CapacityExceeded,
  DirectionChangeUnsupported,
  Infeasible,
};

namespace detail {

struct GeometryNode {
  Pose2d pose{};
  double distance_m{};
  double curvature_per_m{};
  TravelDirection direction{TravelDirection::Forward};
};

inline bool finiteWaypoint(const PathWaypoint& waypoint) noexcept {
  return valid(waypoint.pose) && std::isfinite(waypoint.tangent_scale_m) &&
         waypoint.tangent_scale_m >= 0.0;
}

inline double longitudinalTractionLimit(double speed, double curvature,
                                        double total_limit) noexcept {
  const double centripetal = speed * speed * std::abs(curvature);
  const double remaining =
      total_limit * total_limit - centripetal * centripetal;
  return remaining > 0.0 ? std::sqrt(remaining) : 0.0;
}

}  // namespace detail

template <std::size_t WaypointCapacity, std::size_t SampleCapacity>
class FixedTrajectoryGenerator {
 public:
  TrajectoryGenerationStatus generate(
      const std::array<PathWaypoint, WaypointCapacity>& waypoints,
      std::size_t waypoint_count, const TrajectoryConstraints& constraints,
      FixedTrajectory<SampleCapacity>& trajectory) noexcept {
    trajectory.clear();
    clearWorking();
    if (waypoint_count < 2 || waypoint_count > WaypointCapacity ||
        !validTrajectoryConstraints(constraints))
      return TrajectoryGenerationStatus::InvalidInput;
    const TravelDirection direction = waypoints[0].direction;
    for (std::size_t i = 0; i < waypoint_count; ++i) {
      if (!detail::finiteWaypoint(waypoints[i]))
        return TrajectoryGenerationStatus::InvalidInput;
      if (waypoints[i].direction != direction)
        return TrajectoryGenerationStatus::DirectionChangeUnsupported;
    }
    const auto geometry_status =
        buildGeometry(waypoints, waypoint_count, constraints.spatial_step_m,
                      direction);
    if (geometry_status != TrajectoryGenerationStatus::Success)
      return geometry_status;
    if (!parameterize(constraints, direction, trajectory)) {
      trajectory.clear();
      return TrajectoryGenerationStatus::Infeasible;
    }
    return TrajectoryGenerationStatus::Success;
  }

 private:
  TrajectoryGenerationStatus buildGeometry(
      const std::array<PathWaypoint, WaypointCapacity>& waypoints,
      std::size_t waypoint_count, double spatial_step_m,
      TravelDirection direction) noexcept {
    for (std::size_t segment = 0; segment + 1 < waypoint_count; ++segment) {
      const auto& start = waypoints[segment];
      const auto& end = waypoints[segment + 1];
      const double chord_x = end.pose.x_m - start.pose.x_m;
      const double chord_y = end.pose.y_m - start.pose.y_m;
      const double chord = std::hypot(chord_x, chord_y);
      if (!(chord > 1e-9)) return TrajectoryGenerationStatus::InvalidInput;
      const std::size_t divisions = static_cast<std::size_t>(
          std::max(2.0, std::ceil(chord / spatial_step_m)));
      const double start_scale =
          start.tangent_scale_m > 0.0 ? start.tangent_scale_m : chord;
      const double end_scale =
          end.tangent_scale_m > 0.0 ? end.tangent_scale_m : chord;
      const double sign = directionSign(direction);
      const Vec2 m0{sign * start_scale * std::cos(start.pose.theta_rad),
                    sign * start_scale * std::sin(start.pose.theta_rad)};
      const Vec2 m1{sign * end_scale * std::cos(end.pose.theta_rad),
                    sign * end_scale * std::sin(end.pose.theta_rad)};
      const std::size_t first = segment == 0 ? 0 : 1;
      for (std::size_t division = first; division <= divisions; ++division) {
        if (geometry_count_ == SampleCapacity)
          return TrajectoryGenerationStatus::CapacityExceeded;
        const double u = static_cast<double>(division) /
                         static_cast<double>(divisions);
        const double u2 = u * u;
        const double u3 = u2 * u;
        const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
        const double h10 = u3 - 2.0 * u2 + u;
        const double h01 = -2.0 * u3 + 3.0 * u2;
        const double h11 = u3 - u2;
        const double x = h00 * start.pose.x_m + h10 * m0.x_m +
                         h01 * end.pose.x_m + h11 * m1.x_m;
        const double y = h00 * start.pose.y_m + h10 * m0.y_m +
                         h01 * end.pose.y_m + h11 * m1.y_m;
        const double dh00 = 6.0 * u2 - 6.0 * u;
        const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
        const double dh01 = -6.0 * u2 + 6.0 * u;
        const double dh11 = 3.0 * u2 - 2.0 * u;
        const double dx = dh00 * start.pose.x_m + dh10 * m0.x_m +
                          dh01 * end.pose.x_m + dh11 * m1.x_m;
        const double dy = dh00 * start.pose.y_m + dh10 * m0.y_m +
                          dh01 * end.pose.y_m + dh11 * m1.y_m;
        const double ddh00 = 12.0 * u - 6.0;
        const double ddh10 = 6.0 * u - 4.0;
        const double ddh01 = -12.0 * u + 6.0;
        const double ddh11 = 6.0 * u - 2.0;
        const double ddx = ddh00 * start.pose.x_m + ddh10 * m0.x_m +
                           ddh01 * end.pose.x_m + ddh11 * m1.x_m;
        const double ddy = ddh00 * start.pose.y_m + ddh10 * m0.y_m +
                           ddh01 * end.pose.y_m + ddh11 * m1.y_m;
        const double derivative_norm = std::hypot(dx, dy);
        if (!(derivative_norm > 1e-9))
          return TrajectoryGenerationStatus::Infeasible;
        const double curvature =
            (dx * ddy - dy * ddx) /
            (derivative_norm * derivative_norm * derivative_norm);
        double heading = std::atan2(dy, dx);
        if (direction == TravelDirection::Reverse)
          heading = wrapPi(heading + units::kPi);
        detail::GeometryNode node{{x, y, heading}, 0.0, curvature, direction};
        if (geometry_count_ > 0) {
          const auto& previous = geometry_[geometry_count_ - 1];
          const double step = std::hypot(x - previous.pose.x_m,
                                         y - previous.pose.y_m);
          if (!(step > 1e-12))
            return TrajectoryGenerationStatus::Infeasible;
          node.distance_m = previous.distance_m + step;
        }
        geometry_[geometry_count_++] = node;
      }
    }
    return geometry_count_ >= 2 ? TrajectoryGenerationStatus::Success
                                : TrajectoryGenerationStatus::Infeasible;
  }

  bool parameterize(const TrajectoryConstraints& constraints,
                    TravelDirection direction,
                    FixedTrajectory<SampleCapacity>& trajectory) noexcept {
    const double sign = directionSign(direction);
    for (std::size_t i = 0; i < geometry_count_; ++i) {
      const double curvature = std::abs(geometry_[i].curvature_per_m);
      double limit = constraints.max_velocity_mps;
      if (curvature > 1e-12) {
        limit = std::min(limit,
                         constraints.max_angular_velocity_radps / curvature);
        limit = std::min(
            limit,
            std::sqrt(constraints.max_centripetal_acceleration_mps2 /
                      curvature));
      }
      const double left_factor =
          std::abs(sign - 0.5 * constraints.track_width_m *
                              geometry_[i].curvature_per_m);
      const double right_factor =
          std::abs(sign + 0.5 * constraints.track_width_m *
                              geometry_[i].curvature_per_m);
      const double wheel_factor = std::max(left_factor, right_factor);
      const auto& left_gains = constraints.feedforward.left;
      const auto& right_gains = constraints.feedforward.right;
      const double static_voltage =
          direction == TravelDirection::Forward
              ? std::max(left_gains.kS_forward_V,
                         right_gains.kS_forward_V)
              : std::max(left_gains.kS_reverse_V,
                         right_gains.kS_reverse_V);
      const double velocity_gain =
          std::max(left_gains.kV_Vs_per_unit,
                   right_gains.kV_Vs_per_unit);
      if (velocity_gain > 0.0 && wheel_factor > 1e-12) {
        if (constraints.max_voltage_V <= static_voltage) return false;
        limit = std::min(limit, (constraints.max_voltage_V - static_voltage) /
                                    (velocity_gain * wheel_factor));
      }
      if (!std::isfinite(limit) || limit < 0.0) return false;
      speed_limit_[i] = limit;
      speed_[i] = limit;
    }
    if (constraints.initial_speed_mps > speed_limit_[0] + 1e-9 ||
        constraints.final_speed_mps >
            speed_limit_[geometry_count_ - 1] + 1e-9)
      return false;
    speed_[0] = constraints.initial_speed_mps;
    for (std::size_t i = 1; i < geometry_count_; ++i) {
      const double ds = geometry_[i].distance_m - geometry_[i - 1].distance_m;
      const double traction = detail::longitudinalTractionLimit(
          speed_[i - 1], geometry_[i - 1].curvature_per_m,
          constraints.max_total_acceleration_mps2);
      const double acceleration =
          std::min(constraints.max_acceleration_mps2, traction);
      const double reachable = std::sqrt(std::max(
          0.0, speed_[i - 1] * speed_[i - 1] + 2.0 * acceleration * ds));
      speed_[i] = std::min(speed_[i], reachable);
    }
    speed_[geometry_count_ - 1] = constraints.final_speed_mps;
    for (std::size_t i = geometry_count_ - 1; i > 0; --i) {
      const double ds = geometry_[i].distance_m - geometry_[i - 1].distance_m;
      const double traction = detail::longitudinalTractionLimit(
          speed_[i], geometry_[i].curvature_per_m,
          constraints.max_total_acceleration_mps2);
      const double deceleration =
          std::min(constraints.max_deceleration_mps2, traction);
      const double reachable = std::sqrt(std::max(
          0.0, speed_[i] * speed_[i] + 2.0 * deceleration * ds));
      speed_[i - 1] = std::min(speed_[i - 1], reachable);
    }
    if (std::abs(speed_[0] - constraints.initial_speed_mps) > 1e-8)
      return false;

    time_s_[0] = 0.0;
    for (std::size_t i = 0; i + 1 < geometry_count_; ++i) {
      const double ds =
          geometry_[i + 1].distance_m - geometry_[i].distance_m;
      const double sum = speed_[i] + speed_[i + 1];
      if (!(sum > 1e-12)) return false;
      segment_dt_s_[i] = 2.0 * ds / sum;
      time_s_[i + 1] = time_s_[i] + segment_dt_s_[i];
      segment_acceleration_[i] =
          sign * (speed_[i + 1] - speed_[i]) / segment_dt_s_[i];
      const double omega_start =
          geometry_[i].curvature_per_m * speed_[i];
      const double omega_end =
          geometry_[i + 1].curvature_per_m * speed_[i + 1];
      segment_angular_acceleration_[i] =
          (omega_end - omega_start) / segment_dt_s_[i];
    }

    for (std::size_t i = 0; i < geometry_count_; ++i) {
      const double velocity = sign * speed_[i];
      const double omega = geometry_[i].curvature_per_m * speed_[i];
      const double acceleration =
          i + 1 < geometry_count_ ? segment_acceleration_[i] : 0.0;
      const double angular_acceleration =
          i + 1 < geometry_count_ ? segment_angular_acceleration_[i] : 0.0;
      const double half_track = 0.5 * constraints.track_width_m;
      const double left_velocity = velocity - half_track * omega;
      const double right_velocity = velocity + half_track * omega;
      const double left_acceleration =
          acceleration - half_track * angular_acceleration;
      const double right_acceleration =
          acceleration + half_track * angular_acceleration;
      const auto predicted = calculateDifferentialFeedforward(
          constraints.feedforward, left_velocity, left_acceleration,
          right_velocity, right_acceleration);
      if (!predicted.valid ||
          std::abs(predicted.left_V) > constraints.max_voltage_V + 1e-8 ||
          std::abs(predicted.right_V) > constraints.max_voltage_V + 1e-8)
        return false;
      TrajectorySample sample{};
      sample.time_s = time_s_[i];
      sample.distance_m = geometry_[i].distance_m;
      sample.pose = geometry_[i].pose;
      sample.path_curvature_per_m = geometry_[i].curvature_per_m;
      sample.linear_velocity_mps = velocity;
      sample.linear_acceleration_mps2 = acceleration;
      sample.angular_velocity_radps = omega;
      sample.angular_acceleration_radps2 = angular_acceleration;
      sample.predicted_left_voltage_V = predicted.left_V;
      sample.predicted_right_voltage_V = predicted.right_V;
      sample.direction = direction;
      sample.valid = true;
      if (!trajectory.append(sample)) return false;
    }
    return true;
  }

  void clearWorking() noexcept {
    geometry_ = {};
    speed_limit_ = {};
    speed_ = {};
    time_s_ = {};
    segment_dt_s_ = {};
    segment_acceleration_ = {};
    segment_angular_acceleration_ = {};
    geometry_count_ = 0;
  }

  std::array<detail::GeometryNode, SampleCapacity> geometry_{};
  std::array<double, SampleCapacity> speed_limit_{};
  std::array<double, SampleCapacity> speed_{};
  std::array<double, SampleCapacity> time_s_{};
  std::array<double, SampleCapacity> segment_dt_s_{};
  std::array<double, SampleCapacity> segment_acceleration_{};
  std::array<double, SampleCapacity> segment_angular_acceleration_{};
  std::size_t geometry_count_{};
};

template <std::size_t SourceCapacity, std::size_t OutputCapacity>
inline bool resampleTrajectory(const FixedTrajectory<SourceCapacity>& source,
                               double period_s,
                               FixedTrajectory<OutputCapacity>& output) noexcept {
  output.clear();
  if (source.size() < 2 || !std::isfinite(period_s) || period_s <= 0.0)
    return false;
  const double total = source.totalTime();
  bool finished = false;
  for (std::size_t index = 0; !finished; ++index) {
    const double requested = static_cast<double>(index) * period_s;
    const double time = requested < total ? requested : total;
    const auto sample = source.sampleAt(time);
    if (!sample.valid || !output.append(sample)) {
      output.clear();
      return false;
    }
    finished = time >= total;
    if (finished) continue;
    if (index + 1 >= OutputCapacity) {
      output.clear();
      return false;
    }
  }
  return true;
}

}  // namespace robot
