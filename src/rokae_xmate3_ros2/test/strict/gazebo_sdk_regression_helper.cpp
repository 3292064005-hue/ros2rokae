#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <set>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "rokae/robot.h"
#include "rokae_xmate3_ros2/action/move_append.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"
#include "rokae_xmate3_ros2/srv/adjust_speed_online.hpp"
#include "rokae_xmate3_ros2/srv/connect.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_pos.hpp"
#include "rokae_xmate3_ros2/srv/get_posture.hpp"
#include "rokae_xmate3_ros2/srv/move_reset.hpp"
#include "rokae_xmate3_ros2/srv/move_start.hpp"
#include "rokae_xmate3_ros2/srv/set_default_speed.hpp"
#include "runtime/planner_core.hpp"
#include "runtime/pose_utils.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using Duration = std::chrono::duration<double>;
namespace rt = rokae_xmate3_ros2::runtime;
using MoveAppend = rokae_xmate3_ros2::action::MoveAppend;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

constexpr double kPlannerDtSec = 0.01;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kVelocityNearZeroRadPerSec = 0.01;
constexpr double kMotionStartVelocityRadPerSec = 0.05;
constexpr double kMotionStartPositionDeltaRad = 0.003;
constexpr double kPositionToleranceMm = 3.0;
constexpr double kOrientationToleranceDeg = 1.0;
constexpr double kJointToleranceRad = 0.03;
constexpr double kReadyPoseToleranceRad = 0.12;
constexpr double kDurationToleranceSec = 0.05;
constexpr double kDurationToleranceRatio = 0.08;
constexpr double kJunctionPlateauSec = 0.15;
constexpr double kZoneDurationGapSec = 0.10;
constexpr double kZoneDurationGapRatio = 0.10;
constexpr double kAdjustAfterStartSec = 0.35;
constexpr auto kInterfaceTimeout = std::chrono::seconds(45);
constexpr auto kMotionTimeout = std::chrono::seconds(90);

enum class RegressionMode {
  default_smoke,
  strict,
};

struct JointSample {
  double time_from_record_start_sec = 0.0;
  std::array<double, 6> position{};
  std::array<double, 6> velocity{};
};

struct CaseMetrics {
  std::string name;
  std::string execution_backend = "none";
  std::string control_owner = "none";
  double planned_duration_sec = 0.0;
  double actual_duration_sec = 0.0;
  double motion_start_offset_sec = 0.0;
  double finish_offset_sec = 0.0;
  double adjust_offset_sec = -1.0;
  double remaining_after_adjust_sec = -1.0;
  double planned_path_length_m = 0.0;
  int blend_segment_count = 0;
  double final_position_error_mm = 0.0;
  double final_orientation_error_deg = 0.0;
  double final_joint_error_rad = 0.0;
  double max_junction_plateau_sec = 0.0;
  double max_line_deviation_mm = 0.0;
  double max_arc_radial_deviation_mm = 0.0;
  double max_zone_path_deviation_mm = 0.0;
  bool success = true;
  std::vector<std::string> errors;
  std::vector<std::string> notes;
  std::vector<JointSample> samples;
};

struct Scenario {
  std::string name;
  double default_speed_mm_per_s = 100.0;
  int default_zone_mm = 0;
  std::vector<rt::MotionCommandSpec> specs;
  std::function<void(rokae::xMateRobot &, std::error_code &)> execute;
  std::optional<double> adjust_speed_scale;
  double adjust_after_start_sec = kAdjustAfterStartSec;
  bool verify_duration_against_plan = true;
};

struct RuntimeLogEntry {
  std::string request_id;
  std::string state;
  std::string backend;
  std::string owner;
  std::string message;
};

struct ServiceProbe {
  std::string name;
  rclcpp::ClientBase::SharedPtr client;
};

class JointStateRecorder {
 public:
  JointStateRecorder() {
    node_ = std::make_shared<rclcpp::Node>("rokae_gazebo_sdk_regression_probe");
    connect_client_ = node_->create_client<rokae_xmate3_ros2::srv::Connect>("/xmate3/cobot/connect");
    get_joint_pos_client_ =
        node_->create_client<rokae_xmate3_ros2::srv::GetJointPos>("/xmate3/cobot/get_joint_pos");
    get_posture_client_ =
        node_->create_client<rokae_xmate3_ros2::srv::GetPosture>("/xmate3/cobot/get_posture");
    move_reset_client_ =
        node_->create_client<rokae_xmate3_ros2::srv::MoveReset>("/xmate3/cobot/move_reset");
    move_start_client_ =
        node_->create_client<rokae_xmate3_ros2::srv::MoveStart>("/xmate3/cobot/move_start");
    set_default_speed_client_ =
        node_->create_client<rokae_xmate3_ros2::srv::SetDefaultSpeed>("/xmate3/cobot/set_default_speed");
    adjust_speed_online_client_ = node_->create_client<rokae_xmate3_ros2::srv::AdjustSpeedOnline>(
        "/xmate3/cobot/adjust_speed_online");
    move_append_action_client_ =
        rclcpp_action::create_client<MoveAppend>(node_, "/xmate3/cobot/move_append");
    trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        node_, "/joint_trajectory_controller/follow_joint_trajectory");
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/xmate3/joint_states",
        50,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { handleJointState(*msg); });
    ros2_control_joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        20,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { handleRos2ControlJointState(*msg); });
    operation_state_sub_ = node_->create_subscription<rokae_xmate3_ros2::msg::OperationState>(
        "/xmate3/cobot/operation_state",
        50,
        [this](const rokae_xmate3_ros2::msg::OperationState::SharedPtr msg) {
          handleOperationState(*msg);
        });
    rosout_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout",
        200,
        [this](const rcl_interfaces::msg::Log::SharedPtr msg) { handleRosout(*msg); });
  }

  ~JointStateRecorder() {
    stop();
  }

  void start() {
    if (running_) {
      return;
    }
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    running_ = true;
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void stop() {
    running_ = false;
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (executor_ && node_) {
      executor_->remove_node(node_);
    }
    executor_.reset();
  }

  bool waitForInterfaces(std::chrono::steady_clock::duration timeout) {
    const std::vector<ServiceProbe> service_probes{
        {"/xmate3/cobot/connect", connect_client_},
        {"/xmate3/cobot/get_joint_pos", get_joint_pos_client_},
        {"/xmate3/cobot/get_posture", get_posture_client_},
        {"/xmate3/cobot/move_reset", move_reset_client_},
        {"/xmate3/cobot/move_start", move_start_client_},
        {"/xmate3/cobot/set_default_speed", set_default_speed_client_},
        {"/xmate3/cobot/adjust_speed_online", adjust_speed_online_client_},
    };

    const auto deadline = Clock::now() + timeout;
    for (const auto &probe : service_probes) {
      bool ready = false;
      while (Clock::now() < deadline) {
        if (probe.client->wait_for_service(std::chrono::milliseconds(250))) {
          ready = true;
          break;
        }
      }
      if (!ready) {
        std::cerr << "Timed out waiting for service " << probe.name << std::endl;
        return false;
      }
    }

    bool action_ready = false;
    while (Clock::now() < deadline) {
      if (move_append_action_client_->wait_for_action_server(std::chrono::milliseconds(250))) {
        action_ready = true;
        break;
      }
    }
    if (!action_ready) {
      std::cerr << "Timed out waiting for action /xmate3/cobot/move_append" << std::endl;
      return false;
    }

    bool trajectory_action_ready = false;
    while (Clock::now() < deadline) {
      if (trajectory_action_client_->wait_for_action_server(std::chrono::milliseconds(250))) {
        trajectory_action_ready = true;
        break;
      }
    }
    if (!trajectory_action_ready) {
      std::cerr << "Timed out waiting for action /joint_trajectory_controller/follow_joint_trajectory"
                << std::endl;
      return false;
    }

    return waitForJointHeartbeat(deadline - Clock::now()) &&
           waitForOperationStateHeartbeat(deadline - Clock::now()) &&
           waitForRos2ControlJointHeartbeat(deadline - Clock::now());
  }

  bool waitForJointHeartbeat(std::chrono::steady_clock::duration timeout) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (heartbeat_count_ > 0) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cerr << "Timed out waiting for /xmate3/joint_states heartbeat" << std::endl;
    return false;
  }

  bool waitForOperationStateHeartbeat(std::chrono::steady_clock::duration timeout) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (operation_state_count_ > 0) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cerr << "Timed out waiting for /xmate3/cobot/operation_state heartbeat" << std::endl;
    return false;
  }

  bool waitForRos2ControlJointHeartbeat(std::chrono::steady_clock::duration timeout) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (ros2_control_heartbeat_count_ > 0) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cerr << "Timed out waiting for /joint_states heartbeat" << std::endl;
    return false;
  }

  void startRecording() {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
    record_start_ = Clock::now();
    record_start_position_ = latest_position_;
    recording_ = true;
  }

  std::vector<JointSample> stopRecording() {
    std::lock_guard<std::mutex> lock(mutex_);
    recording_ = false;
    return samples_;
  }

  double secondsSinceRecordingStart() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return Duration(Clock::now() - record_start_).count();
  }

  double secondsFromRecordingStart(const Clock::time_point &time_point) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return Duration(time_point - record_start_).count();
  }

  bool waitForMotionStart(std::chrono::steady_clock::duration timeout, Clock::time_point &start_time) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (samples_.size() >= 3) {
          const std::size_t last = samples_.size() - 1;
          const std::size_t first = last - 2;
          bool sustained_motion = true;
          for (std::size_t index = first; index <= last; ++index) {
            double max_position_delta = 0.0;
            double max_velocity = 0.0;
            for (std::size_t joint = 0; joint < samples_[index].position.size(); ++joint) {
              max_position_delta = std::max(
                  max_position_delta,
                  std::abs(samples_[index].position[joint] - record_start_position_[joint]));
              max_velocity = std::max(max_velocity, std::abs(samples_[index].velocity[joint]));
            }
            if (max_position_delta < kMotionStartPositionDeltaRad ||
                max_velocity < kMotionStartVelocityRadPerSec) {
              sustained_motion = false;
              break;
            }
          }
          if (sustained_motion) {
            start_time = record_start_ +
                         std::chrono::duration_cast<Clock::duration>(
                             Duration(samples_[first].time_from_record_start_sec));
            return true;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }

  bool waitForIdleAndStill(std::chrono::steady_clock::duration timeout, Clock::time_point &finish_time) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_latest_joint_state_ && has_operation_state_ &&
            latest_joint_state_time_ >= record_start_ &&
            latest_operation_state_ == rokae_xmate3_ros2::msg::OperationState::IDLE &&
            latestAbsVelocityLocked() < kVelocityNearZeroRadPerSec) {
          finish_time = latest_joint_state_time_;
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }

  bool isIdleAndStill() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_latest_joint_state_ && has_operation_state_ &&
           latest_operation_state_ == rokae_xmate3_ros2::msg::OperationState::IDLE &&
           latestAbsVelocityLocked() < kVelocityNearZeroRadPerSec;
  }

  std::string latestRuntimeRequestId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (runtime_logs_.empty()) {
      return {};
    }
    return runtime_logs_.back().entry.request_id;
  }

  bool waitForNewRuntimeRequest(const std::string &previous_request_id,
                                std::string &request_id,
                                std::chrono::steady_clock::duration timeout) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = runtime_logs_.rbegin(); it != runtime_logs_.rend(); ++it) {
          if (it->entry.request_id != previous_request_id) {
            request_id = it->entry.request_id;
            return true;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool waitForRuntimeRequestState(const std::string &request_id,
                                  const std::set<std::string> &target_states,
                                  Clock::time_point &observed_time,
                                  std::string &matched_state,
                                  std::chrono::steady_clock::duration timeout) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = runtime_logs_.rbegin(); it != runtime_logs_.rend(); ++it) {
          if (it->entry.request_id != request_id) {
            continue;
          }
          if (target_states.count(it->entry.state) > 0) {
            observed_time = it->observed_time;
            matched_state = it->entry.state;
            return true;
          }
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  std::string latestRuntimeBackendForRequest(const std::string &request_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = runtime_logs_.rbegin(); it != runtime_logs_.rend(); ++it) {
      if (it->entry.request_id == request_id && !it->entry.backend.empty()) {
        return it->entry.backend;
      }
    }
    return {};
  }

  std::string latestRuntimeOwnerForRequest(const std::string &request_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = runtime_logs_.rbegin(); it != runtime_logs_.rend(); ++it) {
      if (it->entry.request_id == request_id &&
          !it->entry.owner.empty() &&
          it->entry.owner != "none") {
        return it->entry.owner;
      }
    }
    return {};
  }

 private:
  void handleJointState(const sensor_msgs::msg::JointState &msg) {
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    ++heartbeat_count_;
    has_latest_joint_state_ = true;
    latest_joint_state_time_ = now;
    if (recording_) {
      JointSample sample;
      sample.time_from_record_start_sec = Duration(now - record_start_).count();
      for (size_t i = 0; i < 6; ++i) {
        sample.position[i] = i < msg.position.size() ? msg.position[i] : 0.0;
        sample.velocity[i] = i < msg.velocity.size() ? msg.velocity[i] : 0.0;
        latest_position_[i] = sample.position[i];
        latest_velocity_[i] = sample.velocity[i];
      }
      samples_.push_back(sample);
    } else {
      for (size_t i = 0; i < 6; ++i) {
        latest_position_[i] = i < msg.position.size() ? msg.position[i] : 0.0;
        latest_velocity_[i] = i < msg.velocity.size() ? msg.velocity[i] : 0.0;
      }
    }
  }

  void handleOperationState(const rokae_xmate3_ros2::msg::OperationState &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++operation_state_count_;
    has_operation_state_ = true;
    latest_operation_state_ = msg.state;
  }

  void handleRos2ControlJointState(const sensor_msgs::msg::JointState &) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++ros2_control_heartbeat_count_;
  }

  void handleRosout(const rcl_interfaces::msg::Log &msg) {
    if (msg.name != "xcore_gazebo_controller" || msg.msg.find("[runtime]") == std::string::npos) {
      return;
    }

    const auto parsed = parseRuntimeText(msg.msg);
    if (!parsed.has_value()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    runtime_logs_.push_back(RuntimeLogObservation{*parsed, Clock::now()});
    constexpr std::size_t kMaxRuntimeLogs = 256;
    if (runtime_logs_.size() > kMaxRuntimeLogs) {
      runtime_logs_.erase(runtime_logs_.begin(), runtime_logs_.begin() + (runtime_logs_.size() - kMaxRuntimeLogs));
    }
  }

  double latestAbsVelocityLocked() const {
    double max_value = 0.0;
    for (double value : latest_velocity_) {
      max_value = std::max(max_value, std::abs(value));
    }
    return max_value;
  }

  struct RuntimeLogObservation {
    RuntimeLogEntry entry;
    Clock::time_point observed_time;
  };

  static std::optional<RuntimeLogEntry> parseRuntimeText(const std::string &text) {
    const auto extract = [&](const std::string &key) -> std::optional<std::string> {
      const auto key_pos = text.find(key);
      if (key_pos == std::string::npos) {
        return std::nullopt;
      }
      const auto value_start = key_pos + key.size();
      auto value_end = text.find(' ', value_start);
      if (value_end == std::string::npos) {
        value_end = text.size();
      }
      return text.substr(value_start, value_end - value_start);
    };

    const auto request_id = extract("request=");
    const auto state = extract("state=");
    const auto backend = extract("backend=");
    const auto owner = extract("owner=");
    const auto message = extract("message=");
    if (!request_id.has_value() || !state.has_value()) {
      return std::nullopt;
    }

    RuntimeLogEntry entry;
    entry.request_id = *request_id;
    entry.state = *state;
    entry.backend = backend.value_or(std::string{});
    entry.owner = owner.value_or(std::string{});
    entry.message = message.value_or(std::string{});
    return entry;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  bool running_ = false;

  rclcpp::Client<rokae_xmate3_ros2::srv::Connect>::SharedPtr connect_client_;
  rclcpp::Client<rokae_xmate3_ros2::srv::GetJointPos>::SharedPtr get_joint_pos_client_;
  rclcpp::Client<rokae_xmate3_ros2::srv::GetPosture>::SharedPtr get_posture_client_;
  rclcpp::Client<rokae_xmate3_ros2::srv::MoveReset>::SharedPtr move_reset_client_;
  rclcpp::Client<rokae_xmate3_ros2::srv::MoveStart>::SharedPtr move_start_client_;
  rclcpp::Client<rokae_xmate3_ros2::srv::SetDefaultSpeed>::SharedPtr set_default_speed_client_;
  rclcpp::Client<rokae_xmate3_ros2::srv::AdjustSpeedOnline>::SharedPtr adjust_speed_online_client_;
  rclcpp_action::Client<MoveAppend>::SharedPtr move_append_action_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ros2_control_joint_state_sub_;
  rclcpp::Subscription<rokae_xmate3_ros2::msg::OperationState>::SharedPtr operation_state_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;

  mutable std::mutex mutex_;
  std::vector<JointSample> samples_;
  Clock::time_point record_start_{Clock::now()};
  bool recording_ = false;
  std::size_t heartbeat_count_ = 0;
  std::size_t ros2_control_heartbeat_count_ = 0;
  std::size_t operation_state_count_ = 0;
  bool has_latest_joint_state_ = false;
  bool has_operation_state_ = false;
  Clock::time_point latest_joint_state_time_{Clock::now()};
  std::array<double, 6> latest_position_{};
  std::array<double, 6> latest_velocity_{};
  std::array<double, 6> record_start_position_{};
  uint8_t latest_operation_state_ = rokae_xmate3_ros2::msg::OperationState::IDLE;
  std::vector<RuntimeLogObservation> runtime_logs_;
};

std::vector<double> toVector(const std::array<double, 6> &values) {
  return std::vector<double>(values.begin(), values.end());
}

rokae::CartesianPosition toCartesian(const std::vector<double> &pose) {
  return rokae::CartesianPosition(
      {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]});
}

double maxAbsVelocity(const std::array<double, 6> &velocity) {
  double max_value = 0.0;
  for (double component : velocity) {
    max_value = std::max(max_value, std::abs(component));
  }
  return max_value;
}

double maxJointError(const std::array<double, 6> &actual, const std::vector<double> &target) {
  double error = 0.0;
  for (size_t i = 0; i < 6 && i < target.size(); ++i) {
    error = std::max(error, std::abs(actual[i] - target[i]));
  }
  return error;
}

double positionErrorMm(const std::vector<double> &expected_pose, const std::vector<double> &actual_pose) {
  const double dx = actual_pose[0] - expected_pose[0];
  const double dy = actual_pose[1] - expected_pose[1];
  const double dz = actual_pose[2] - expected_pose[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz) * 1000.0;
}

double orientationErrorDeg(const std::vector<double> &expected_pose, const std::vector<double> &actual_pose) {
  return rt::pose_utils::angularDistance(expected_pose, actual_pose) * kRadToDeg;
}

double pointToSegmentDistanceMm(const Eigen::Vector3d &point,
                                const Eigen::Vector3d &start,
                                const Eigen::Vector3d &end) {
  const Eigen::Vector3d segment = end - start;
  const double length_sq = segment.squaredNorm();
  if (length_sq < 1e-12) {
    return (point - start).norm() * 1000.0;
  }
  const double alpha =
      std::clamp((point - start).dot(segment) / length_sq, 0.0, 1.0);
  const Eigen::Vector3d projection = start + alpha * segment;
  return (point - projection).norm() * 1000.0;
}

double maxLineDeviationMm(const std::vector<JointSample> &samples,
                          gazebo::xMate3Kinematics &kinematics,
                          const std::vector<double> &start_pose,
                          const std::vector<double> &end_pose,
                          double window_start_sec,
                          double window_end_sec) {
  const Eigen::Vector3d line_start(start_pose[0], start_pose[1], start_pose[2]);
  const Eigen::Vector3d line_end(end_pose[0], end_pose[1], end_pose[2]);
  double max_deviation = 0.0;
  for (const auto &sample : samples) {
    if (sample.time_from_record_start_sec < window_start_sec ||
        sample.time_from_record_start_sec > window_end_sec) {
      continue;
    }
    const auto pose = kinematics.forwardKinematicsRPY(toVector(sample.position));
    const Eigen::Vector3d point(pose[0], pose[1], pose[2]);
    max_deviation = std::max(max_deviation, pointToSegmentDistanceMm(point, line_start, line_end));
  }
  return max_deviation;
}

bool computeCircleForDeviation(const std::vector<double> &start_pose,
                               const std::vector<double> &aux_pose,
                               const std::vector<double> &end_pose,
                               Eigen::Vector3d &center,
                               double &radius) {
  const Eigen::Vector3d p1(start_pose[0], start_pose[1], start_pose[2]);
  const Eigen::Vector3d p2(aux_pose[0], aux_pose[1], aux_pose[2]);
  const Eigen::Vector3d p3(end_pose[0], end_pose[1], end_pose[2]);
  const Eigen::Vector3d u = p2 - p1;
  const Eigen::Vector3d v = p3 - p1;
  const Eigen::Vector3d w = u.cross(v);
  const double w_sq = w.squaredNorm();
  if (w_sq < 1e-12) {
    return false;
  }

  center = p1 + (u.squaredNorm() * v.cross(w) + v.squaredNorm() * w.cross(u)) / (2.0 * w_sq);
  radius = (p1 - center).norm();
  return std::isfinite(radius) && radius > 1e-9;
}

double maxArcRadialDeviationMm(const std::vector<JointSample> &samples,
                               gazebo::xMate3Kinematics &kinematics,
                               const std::vector<double> &start_pose,
                               const std::vector<double> &aux_pose,
                               const std::vector<double> &end_pose,
                               double window_start_sec,
                               double window_end_sec) {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.0;
  if (!computeCircleForDeviation(start_pose, aux_pose, end_pose, center, radius)) {
    return 0.0;
  }

  double max_deviation = 0.0;
  for (const auto &sample : samples) {
    if (sample.time_from_record_start_sec < window_start_sec ||
        sample.time_from_record_start_sec > window_end_sec) {
      continue;
    }
    const auto pose = kinematics.forwardKinematicsRPY(toVector(sample.position));
    const Eigen::Vector3d point(pose[0], pose[1], pose[2]);
    max_deviation = std::max(max_deviation, std::abs((point - center).norm() - radius) * 1000.0);
  }
  return max_deviation;
}

double pointToPolylineDeviationMm(const Eigen::Vector3d &point, const std::vector<Eigen::Vector3d> &polyline) {
  if (polyline.size() < 2) {
    return 0.0;
  }

  double deviation = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1 < polyline.size(); ++index) {
    deviation = std::min(deviation, pointToSegmentDistanceMm(point, polyline[index], polyline[index + 1]));
  }
  return std::isfinite(deviation) ? deviation : 0.0;
}

double maxPolylineDeviationMm(const std::vector<JointSample> &samples,
                              gazebo::xMate3Kinematics &kinematics,
                              const std::vector<Eigen::Vector3d> &polyline,
                              double window_start_sec,
                              double window_end_sec) {
  double max_deviation = 0.0;
  for (const auto &sample : samples) {
    if (sample.time_from_record_start_sec < window_start_sec ||
        sample.time_from_record_start_sec > window_end_sec) {
      continue;
    }
    const auto pose = kinematics.forwardKinematicsRPY(toVector(sample.position));
    const Eigen::Vector3d point(pose[0], pose[1], pose[2]);
    max_deviation = std::max(max_deviation, pointToPolylineDeviationMm(point, polyline));
  }
  return max_deviation;
}

double maxNearZeroPlateau(const std::vector<JointSample> &samples, double window_start_sec, double window_end_sec) {
  if (samples.empty() || window_end_sec <= window_start_sec) {
    return 0.0;
  }

  double longest_sec = 0.0;
  bool in_plateau = false;
  double plateau_start_sec = 0.0;
  double plateau_last_sec = 0.0;

  for (const auto &sample : samples) {
    if (sample.time_from_record_start_sec < window_start_sec ||
        sample.time_from_record_start_sec > window_end_sec) {
      continue;
    }
    const bool near_zero = maxAbsVelocity(sample.velocity) < kVelocityNearZeroRadPerSec;
    if (near_zero) {
      if (!in_plateau) {
        in_plateau = true;
        plateau_start_sec = sample.time_from_record_start_sec;
      }
      plateau_last_sec = sample.time_from_record_start_sec;
      longest_sec = std::max(longest_sec, plateau_last_sec - plateau_start_sec);
    } else {
      in_plateau = false;
    }
  }

  return longest_sec;
}

double maxJunctionPlateau(const rt::MotionPlan &plan,
                          double motion_start_offset_sec,
                          const std::vector<JointSample> &samples) {
  if (plan.segments.size() < 2 || samples.empty()) {
    return 0.0;
  }

  double cumulative_sec = motion_start_offset_sec;
  double longest_sec = 0.0;
  for (size_t index = 0; index + 1 < plan.segments.size(); ++index) {
    cumulative_sec += plan.segments[index].trajectory_total_time;
    longest_sec = std::max(
        longest_sec,
        maxNearZeroPlateau(samples, cumulative_sec - 0.35, cumulative_sec + 0.35));
  }
  return longest_sec;
}

void addFailure(CaseMetrics &metrics, const std::string &message) {
  metrics.success = false;
  metrics.errors.push_back(message);
}

template <typename Command>
bool startCommands(rokae::xMateRobot &robot, const std::vector<Command> &cmds, std::error_code &ec) {
  std::string cmd_id;
  robot.moveReset(ec);
  if (ec) {
    return false;
  }
  robot.moveAppend(cmds, cmd_id, ec);
  if (ec) {
    return false;
  }
  robot.moveStart(ec);
  return !ec;
}

bool configureRobot(rokae::xMateRobot &robot) {
  std::error_code ec;
  robot.connectToRobot(ec);
  if (ec) {
    std::cerr << "connectToRobot failed: " << ec.message() << std::endl;
    return false;
  }
  robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  if (ec) {
    std::cerr << "setMotionControlMode failed: " << ec.message() << std::endl;
    return false;
  }
  robot.setOperateMode(rokae::OperateMode::automatic, ec);
  if (ec) {
    std::cerr << "setOperateMode failed: " << ec.message() << std::endl;
    return false;
  }
  robot.setPowerState(true, ec);
  if (ec) {
    std::cerr << "setPowerState failed: " << ec.message() << std::endl;
    return false;
  }
  robot.setDefaultConfOpt(false, ec);
  if (ec) {
    std::cerr << "setDefaultConfOpt failed: " << ec.message() << std::endl;
    return false;
  }
  return true;
}

void bestEffortRecover(rokae::xMateRobot &robot) {
  std::error_code ec;
  robot.adjustSpeedOnline(1.0, ec);
  ec.clear();
  robot.stop(ec);
  ec.clear();
  robot.moveReset(ec);
}

bool moveToReadyPose(rokae::xMateRobot &robot,
                    JointStateRecorder &recorder,
                    const std::array<double, 6> &ready_pose,
                    bool enforce_tolerance) {
  std::error_code ec;
  robot.adjustSpeedOnline(1.0, ec);
  if (ec) {
    std::cerr << "adjustSpeedOnline(1.0) failed before ready pose: " << ec.message() << std::endl;
    return false;
  }
  robot.setDefaultSpeed(250, ec);
  if (ec) {
    std::cerr << "setDefaultSpeed(ready) failed: " << ec.message() << std::endl;
    return false;
  }
  robot.setDefaultZone(0, ec);
  if (ec) {
    std::cerr << "setDefaultZone(ready) failed: " << ec.message() << std::endl;
    return false;
  }

  const std::vector<rokae::MoveAbsJCommand> commands{
      rokae::MoveAbsJCommand(toVector(ready_pose), 250, 0)};
  const auto previous_request_id = recorder.latestRuntimeRequestId();
  recorder.startRecording();
  if (!startCommands(robot, commands, ec)) {
    recorder.stopRecording();
    std::cerr << "moveStart(ready pose) failed: " << ec.message() << std::endl;
    return false;
  }

  std::string request_id;
  if (!recorder.waitForNewRuntimeRequest(previous_request_id, request_id, std::chrono::seconds(10))) {
    recorder.stopRecording();
    std::cerr << "Waiting for ready-pose runtime request id timed out" << std::endl;
    return false;
  }

  Clock::time_point start_time{};
  std::string start_state;
  if (!recorder.waitForRuntimeRequestState(request_id,
                                           {"executing", "completed", "completed_relaxed", "failed", "stopped"},
                                           start_time,
                                           start_state,
                                           std::chrono::seconds(20))) {
    recorder.stopRecording();
    std::cerr << "Waiting for ready pose runtime start timed out" << std::endl;
    return false;
  }
  if (start_state == "failed" || start_state == "stopped") {
    recorder.stopRecording();
    std::cerr << "Ready pose runtime reached terminal state " << start_state << std::endl;
    return false;
  }

  Clock::time_point settled_time{};
  if (!recorder.waitForIdleAndStill(std::chrono::seconds(45), settled_time)) {
    recorder.stopRecording();
    std::cerr << "Waiting for ready pose idle-and-still settle timed out" << std::endl;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  recorder.stopRecording();

  const auto final_joints = robot.jointPos(ec);
  if (ec) {
    std::cerr << "jointPos(final ready pose) failed: " << ec.message() << std::endl;
    return false;
  }
  const auto final_joint_error = maxJointError(final_joints, toVector(ready_pose));
  if (enforce_tolerance && final_joint_error > kReadyPoseToleranceRad) {
    std::cerr << "ready pose joint error too large: " << final_joint_error << std::endl;
    return false;
  }
  return true;
}

rt::MotionRequest buildRequest(const std::string &request_id,
                               const std::vector<double> &start_joints,
                               double default_speed,
                               int default_zone,
                               const std::vector<rt::MotionCommandSpec> &commands) {
  rt::MotionRequest request;
  request.request_id = request_id;
  request.start_joints = start_joints;
  request.default_speed = default_speed;
  request.default_zone = default_zone;
  request.strict_conf = false;
  request.avoid_singularity = true;
  request.trajectory_dt = kPlannerDtSec;
  request.commands = commands;
  return request;
}

void writeCsv(const std::filesystem::path &path, const std::vector<JointSample> &samples) {
  std::ofstream out(path);
  out << "time_sec";
  for (int joint = 1; joint <= 6; ++joint) {
    out << ",q" << joint;
  }
  for (int joint = 1; joint <= 6; ++joint) {
    out << ",dq" << joint;
  }
  out << "\n";
  out << std::fixed << std::setprecision(6);
  for (const auto &sample : samples) {
    out << sample.time_from_record_start_sec;
    for (double value : sample.position) {
      out << "," << value;
    }
    for (double value : sample.velocity) {
      out << "," << value;
    }
    out << "\n";
  }
}

std::string quoteJson(const std::string &text) {
  std::ostringstream oss;
  oss << '"';
  for (char ch : text) {
    switch (ch) {
      case '\\':
        oss << "\\\\";
        break;
      case '"':
        oss << "\\\"";
        break;
      case '\n':
        oss << "\\n";
        break;
      default:
        oss << ch;
        break;
    }
  }
  oss << '"';
  return oss.str();
}

void writeJson(const std::filesystem::path &path, const CaseMetrics &metrics) {
  std::ofstream out(path);
  out << "{\n";
  out << "  \"name\": " << quoteJson(metrics.name) << ",\n";
  out << "  \"success\": " << (metrics.success ? "true" : "false") << ",\n";
  out << "  \"execution_backend\": " << quoteJson(metrics.execution_backend) << ",\n";
  out << "  \"control_owner\": " << quoteJson(metrics.control_owner) << ",\n";
  out << "  \"planned_duration_sec\": " << metrics.planned_duration_sec << ",\n";
  out << "  \"actual_duration_sec\": " << metrics.actual_duration_sec << ",\n";
  out << "  \"motion_start_offset_sec\": " << metrics.motion_start_offset_sec << ",\n";
  out << "  \"finish_offset_sec\": " << metrics.finish_offset_sec << ",\n";
  out << "  \"adjust_offset_sec\": " << metrics.adjust_offset_sec << ",\n";
  out << "  \"remaining_after_adjust_sec\": " << metrics.remaining_after_adjust_sec << ",\n";
  out << "  \"planned_path_length_m\": " << metrics.planned_path_length_m << ",\n";
  out << "  \"blend_segment_count\": " << metrics.blend_segment_count << ",\n";
  out << "  \"final_position_error_mm\": " << metrics.final_position_error_mm << ",\n";
  out << "  \"final_orientation_error_deg\": " << metrics.final_orientation_error_deg << ",\n";
  out << "  \"final_joint_error_rad\": " << metrics.final_joint_error_rad << ",\n";
  out << "  \"max_junction_plateau_sec\": " << metrics.max_junction_plateau_sec << ",\n";
  out << "  \"max_line_deviation_mm\": " << metrics.max_line_deviation_mm << ",\n";
  out << "  \"max_arc_radial_deviation_mm\": " << metrics.max_arc_radial_deviation_mm << ",\n";
  out << "  \"max_zone_path_deviation_mm\": " << metrics.max_zone_path_deviation_mm << ",\n";
  out << "  \"notes\": [";
  for (size_t i = 0; i < metrics.notes.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << quoteJson(metrics.notes[i]);
  }
  out << "],\n";
  out << "  \"errors\": [";
  for (size_t i = 0; i < metrics.errors.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << quoteJson(metrics.errors[i]);
  }
  out << "]\n";
  out << "}\n";
}

void writeSummaryJson(const std::filesystem::path &path, const std::vector<CaseMetrics> &metrics_list) {
  std::ofstream out(path);
  out << "{\n  \"cases\": [\n";
  for (size_t i = 0; i < metrics_list.size(); ++i) {
    const auto &metrics = metrics_list[i];
    out << "    {\n";
    out << "      \"name\": " << quoteJson(metrics.name) << ",\n";
    out << "      \"success\": " << (metrics.success ? "true" : "false") << ",\n";
    out << "      \"execution_backend\": " << quoteJson(metrics.execution_backend) << ",\n";
    out << "      \"control_owner\": " << quoteJson(metrics.control_owner) << ",\n";
    out << "      \"planned_duration_sec\": " << metrics.planned_duration_sec << ",\n";
    out << "      \"actual_duration_sec\": " << metrics.actual_duration_sec << ",\n";
    out << "      \"remaining_after_adjust_sec\": " << metrics.remaining_after_adjust_sec << ",\n";
    out << "      \"planned_path_length_m\": " << metrics.planned_path_length_m << ",\n";
    out << "      \"blend_segment_count\": " << metrics.blend_segment_count << "\n";
    out << "    }";
    if (i + 1 < metrics_list.size()) {
      out << ",";
    }
    out << "\n";
  }
  out << "  ]\n}\n";
}

CaseMetrics runScenario(const Scenario &scenario,
                        rokae::xMateRobot &robot,
                        JointStateRecorder &recorder,
                        gazebo::xMate3Kinematics &kinematics,
                        const std::array<double, 6> &ready_pose,
                        RegressionMode mode) {
  CaseMetrics metrics;
  metrics.name = scenario.name;

  if (!moveToReadyPose(robot, recorder, ready_pose, mode == RegressionMode::strict)) {
    addFailure(metrics, "failed to move robot to ready pose");
    return metrics;
  }

  std::error_code ec;
  robot.adjustSpeedOnline(1.0, ec);
  if (ec) {
    addFailure(metrics, "failed to restore online speed scale: " + ec.message());
    return metrics;
  }
  robot.setDefaultSpeed(static_cast<int>(std::lround(scenario.default_speed_mm_per_s)), ec);
  if (ec) {
    addFailure(metrics, "setDefaultSpeed failed: " + ec.message());
    return metrics;
  }
  robot.setDefaultZone(scenario.default_zone_mm, ec);
  if (ec) {
    addFailure(metrics, "setDefaultZone failed: " + ec.message());
    return metrics;
  }

  const auto start_joints = robot.jointPos(ec);
  if (ec) {
    addFailure(metrics, "jointPos(start) failed: " + ec.message());
    return metrics;
  }
  const auto start_pose = kinematics.forwardKinematicsRPY(toVector(start_joints));

  std::optional<rt::MotionPlan> plan;
  std::vector<double> expected_joints;
  std::vector<double> expected_pose;
  if (mode == RegressionMode::strict) {
    rt::MotionPlanner planner;
    auto planned = planner.plan(buildRequest(scenario.name, toVector(start_joints), scenario.default_speed_mm_per_s,
                                             scenario.default_zone_mm, scenario.specs));
    if (!planned.valid()) {
      addFailure(metrics, "offline MotionPlanner failed: " + planned.error_message);
      return metrics;
    }
    plan = std::move(planned);
    metrics.notes = plan->notes;
    for (const auto &segment : plan->segments) {
      metrics.planned_duration_sec += segment.trajectory_total_time;
      metrics.planned_path_length_m += segment.path_length_m;
      if (segment.blend_to_next) {
        ++metrics.blend_segment_count;
      }
    }
    if (!plan->segments.empty()) {
      expected_joints = plan->segments.back().target_joints;
      expected_pose = kinematics.forwardKinematicsRPY(expected_joints);
    }
  } else if (!scenario.specs.empty()) {
    const auto &last_spec = scenario.specs.back();
    expected_joints = last_spec.target_joints;
    expected_pose = last_spec.target_cartesian;
    metrics.notes.push_back("default smoke mode: offline planner skipped");
    if (expected_pose.empty() && !expected_joints.empty()) {
      expected_pose = kinematics.forwardKinematicsRPY(expected_joints);
    }
  }

  const auto previous_request_id = recorder.latestRuntimeRequestId();
  recorder.startRecording();
  scenario.execute(robot, ec);
  if (ec) {
    metrics.samples = recorder.stopRecording();
    addFailure(metrics, "executeCommand failed: " + ec.message());
    return metrics;
  }

  std::string request_id;
  if (!recorder.waitForNewRuntimeRequest(previous_request_id, request_id, std::chrono::seconds(10))) {
    metrics.samples = recorder.stopRecording();
    addFailure(metrics, "timed out waiting for runtime request id");
    return metrics;
  }

  Clock::time_point motion_start{};
  std::string start_state;
  if (!recorder.waitForRuntimeRequestState(request_id,
                                           {"executing"},
                                           motion_start,
                                           start_state,
                                           std::chrono::seconds(45))) {
    metrics.samples = recorder.stopRecording();
    addFailure(metrics, "timed out waiting for runtime executing state");
    return metrics;
  }
  metrics.motion_start_offset_sec = recorder.secondsFromRecordingStart(motion_start);

  if (scenario.adjust_speed_scale.has_value()) {
    const auto adjust_deadline = motion_start + std::chrono::seconds(30);
    while (Clock::now() < adjust_deadline) {
      if (Duration(Clock::now() - motion_start).count() >= scenario.adjust_after_start_sec) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    robot.adjustSpeedOnline(*scenario.adjust_speed_scale, ec);
    if (ec) {
      metrics.samples = recorder.stopRecording();
      addFailure(metrics, "adjustSpeedOnline failed: " + ec.message());
      return metrics;
    }
    metrics.adjust_offset_sec = recorder.secondsSinceRecordingStart();
  }

  Clock::time_point finish_time{};
  std::string terminal_state;
  if (mode == RegressionMode::default_smoke) {
    if (!recorder.waitForIdleAndStill(kMotionTimeout, finish_time)) {
      metrics.samples = recorder.stopRecording();
      addFailure(metrics, "waiting for default-smoke motion settle failed: timed out");
      return metrics;
    }
    terminal_state = "completed_smoke";
  } else {
    if (!recorder.waitForRuntimeRequestState(request_id,
                                             {"completed", "completed_relaxed", "failed", "stopped"},
                                             finish_time,
                                             terminal_state,
                                             kMotionTimeout)) {
      metrics.samples = recorder.stopRecording();
      addFailure(metrics, "waiting for motion completion failed: timed out");
      return metrics;
    }
    if (terminal_state == "failed" || terminal_state == "stopped") {
      addFailure(metrics, "runtime reached terminal state " + terminal_state);
    }
  }
  metrics.execution_backend = recorder.latestRuntimeBackendForRequest(request_id);
  if (metrics.execution_backend.empty()) {
    metrics.execution_backend = "none";
  }
  metrics.control_owner = recorder.latestRuntimeOwnerForRequest(request_id);
  if (metrics.control_owner.empty()) {
    metrics.control_owner = "none";
  }

  Clock::time_point settled_time{};
  if (mode != RegressionMode::default_smoke &&
      recorder.waitForIdleAndStill(std::chrono::seconds(2), settled_time)) {
    finish_time = settled_time;
  }

  metrics.samples = recorder.stopRecording();
  metrics.actual_duration_sec = Duration(finish_time - motion_start).count();
  metrics.finish_offset_sec = recorder.secondsFromRecordingStart(finish_time);
  if (metrics.adjust_offset_sec >= 0.0) {
    metrics.remaining_after_adjust_sec = metrics.finish_offset_sec - metrics.adjust_offset_sec;
  }

  const auto actual_joints = robot.jointPos(ec);
  if (ec) {
    addFailure(metrics, "jointPos(final) failed: " + ec.message());
    return metrics;
  }
  const auto actual_pose_vector = kinematics.forwardKinematicsRPY(toVector(actual_joints));
  if (!expected_joints.empty()) {
    metrics.final_joint_error_rad = maxJointError(actual_joints, expected_joints);
  }
  if (!expected_pose.empty()) {
    metrics.final_position_error_mm = positionErrorMm(expected_pose, actual_pose_vector);
    metrics.final_orientation_error_deg = orientationErrorDeg(expected_pose, actual_pose_vector);
  }
  metrics.max_junction_plateau_sec =
      plan.has_value() ? maxJunctionPlateau(*plan, metrics.motion_start_offset_sec, metrics.samples) : 0.0;

  if (mode == RegressionMode::strict && !scenario.specs.empty()) {
    const double window_start_sec = metrics.motion_start_offset_sec;
    const double window_end_sec = metrics.finish_offset_sec;
    if (scenario.specs.size() == 1 && scenario.specs.front().kind == rt::MotionKind::move_l) {
      metrics.max_line_deviation_mm =
          maxLineDeviationMm(metrics.samples,
                             kinematics,
                             start_pose,
                             scenario.specs.front().target_cartesian,
                             window_start_sec,
                             window_end_sec);
    }
    if (scenario.specs.size() == 1 && scenario.specs.front().kind == rt::MotionKind::move_c) {
      metrics.max_arc_radial_deviation_mm =
          maxArcRadialDeviationMm(metrics.samples,
                                 kinematics,
                                 start_pose,
                                 scenario.specs.front().aux_cartesian,
                                 scenario.specs.front().target_cartesian,
                                 window_start_sec,
                                 window_end_sec);
    }
    if (scenario.specs.size() >= 2) {
      std::vector<Eigen::Vector3d> polyline;
      polyline.reserve(scenario.specs.size() + 1);
      polyline.emplace_back(start_pose[0], start_pose[1], start_pose[2]);
      for (const auto &spec : scenario.specs) {
        if (spec.target_cartesian.size() >= 3) {
          polyline.emplace_back(spec.target_cartesian[0], spec.target_cartesian[1], spec.target_cartesian[2]);
        }
      }
      metrics.max_zone_path_deviation_mm =
          maxPolylineDeviationMm(metrics.samples, kinematics, polyline, window_start_sec, window_end_sec);
    }
  }

  if (mode == RegressionMode::strict && scenario.verify_duration_against_plan) {
    const double allowed = std::max(kDurationToleranceSec, kDurationToleranceRatio * metrics.planned_duration_sec);
    if (std::abs(metrics.actual_duration_sec - metrics.planned_duration_sec) > allowed) {
      std::ostringstream oss;
      oss << "duration mismatch: planned=" << metrics.planned_duration_sec
          << " actual=" << metrics.actual_duration_sec
          << " allowed=" << allowed;
      addFailure(metrics, oss.str());
    }
  }

  if (mode == RegressionMode::strict) {
    if (metrics.execution_backend != "jtc") {
      addFailure(metrics, "strict NRT regression expected JTC backend, observed " + metrics.execution_backend);
    }
    if (metrics.control_owner != "trajectory") {
      addFailure(metrics, "strict NRT regression expected trajectory owner, observed " + metrics.control_owner);
    }
    if (metrics.final_position_error_mm > kPositionToleranceMm) {
      std::ostringstream oss;
      oss << "final TCP position error too large: " << metrics.final_position_error_mm << " mm";
      addFailure(metrics, oss.str());
    }
    if (metrics.final_orientation_error_deg > kOrientationToleranceDeg) {
      std::ostringstream oss;
      oss << "final TCP orientation error too large: " << metrics.final_orientation_error_deg << " deg";
      addFailure(metrics, oss.str());
    }
    if (metrics.final_joint_error_rad > kJointToleranceRad) {
      std::ostringstream oss;
      oss << "final joint error too large: " << metrics.final_joint_error_rad << " rad";
      addFailure(metrics, oss.str());
    }
    if (scenario.name == "move_l_long" && metrics.max_line_deviation_mm > 5.0) {
      std::ostringstream oss;
      oss << "line path deviation too large: " << metrics.max_line_deviation_mm << " mm";
      addFailure(metrics, oss.str());
    }
    if (scenario.name == "move_c_single" && metrics.max_arc_radial_deviation_mm > 6.0) {
      std::ostringstream oss;
      oss << "arc radial deviation too large: " << metrics.max_arc_radial_deviation_mm << " mm";
      addFailure(metrics, oss.str());
    }
    if (scenario.name == "move_l_zone_blend" &&
        metrics.max_zone_path_deviation_mm >
            static_cast<double>(scenario.default_zone_mm) + 2.0) {
      std::ostringstream oss;
      oss << "zone path deviation too large: " << metrics.max_zone_path_deviation_mm << " mm";
      addFailure(metrics, oss.str());
    }
    if (scenario.name == "move_sp_single") {
      const bool has_spiral_segment =
          plan.has_value() &&
          std::any_of(plan->segments.begin(), plan->segments.end(), [](const rt::PlannedSegment &segment) {
            return segment.path_family == rt::PathFamily::cartesian_spiral && segment.path_length_m > 0.0;
          });
      if (!has_spiral_segment) {
        addFailure(metrics, "MoveSP strict reference plan did not retain cartesian_spiral path metadata");
      }
    }
  }

  return metrics;
}

void exportMetrics(const std::filesystem::path &results_dir, const CaseMetrics &metrics) {
  std::filesystem::create_directories(results_dir);
  writeCsv(results_dir / (metrics.name + ".csv"), metrics.samples);
  writeJson(results_dir / (metrics.name + ".json"), metrics);
}

void reportCase(const CaseMetrics &metrics) {
  std::cout << metrics.name << ": backend=" << metrics.execution_backend
            << " owner=" << metrics.control_owner
            << " planned=" << metrics.planned_duration_sec
            << "s actual=" << metrics.actual_duration_sec
            << "s path=" << metrics.planned_path_length_m
            << "m blends=" << metrics.blend_segment_count
            << "s pos_err=" << metrics.final_position_error_mm
            << "mm rot_err=" << metrics.final_orientation_error_deg
            << "deg joint_err=" << metrics.final_joint_error_rad
            << "rad plateau=" << metrics.max_junction_plateau_sec
            << "s line_dev=" << metrics.max_line_deviation_mm
            << "mm arc_dev=" << metrics.max_arc_radial_deviation_mm
            << "mm zone_dev=" << metrics.max_zone_path_deviation_mm << "mm";
  if (!metrics.success) {
    std::cout << " [FAILED]";
  }
  std::cout << std::endl;
  for (const auto &error : metrics.errors) {
    std::cout << "  - " << error << std::endl;
  }
}

std::string requireResultsDir(int argc, char **argv) {
  for (int index = 1; index + 1 < argc; ++index) {
    if (std::string(argv[index]) == "--results-dir") {
      return argv[index + 1];
    }
  }
  return {};
}

RegressionMode parseMode(int argc, char **argv) {
  for (int index = 1; index + 1 < argc; ++index) {
    if (std::string(argv[index]) == "--mode") {
      const std::string mode = argv[index + 1];
      if (mode == "strict") {
        return RegressionMode::strict;
      }
    }
  }
  return RegressionMode::default_smoke;
}

}  // namespace

int main(int argc, char **argv) {
  const std::string results_dir_arg = requireResultsDir(argc, argv);
  const auto mode = parseMode(argc, argv);
  if (results_dir_arg.empty()) {
    std::cerr << "Usage: gazebo_sdk_regression_helper --results-dir <path> [--mode default|strict]" << std::endl;
    return 2;
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  const std::filesystem::path results_dir(results_dir_arg);
  std::filesystem::create_directories(results_dir);

  JointStateRecorder recorder;
  recorder.start();
  if (!recorder.waitForInterfaces(kInterfaceTimeout)) {
    recorder.stop();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  rokae::xMateRobot robot;
  if (!configureRobot(robot)) {
    recorder.stop();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  gazebo::xMate3Kinematics kinematics;
  const std::array<double, 6> ready_pose{0.0, 0.45, 0.08, 0.0, 0.72, 0.0};
  const auto ready_pose_vector = toVector(ready_pose);
  const auto start_pose = kinematics.forwardKinematicsRPY(ready_pose_vector);

  auto make_line_pose = [&](double dx, double dy, double dz) {
    auto pose = start_pose;
    pose[0] += dx;
    pose[1] += dy;
    pose[2] += dz;
    return pose;
  };

  const auto long_line_target = make_line_pose(-0.18, 0.12, -0.02);
  const auto circle_aux = make_line_pose(-0.01, 0.06, -0.02);
  const auto circle_target = make_line_pose(-0.06, 0.12, -0.01);
  const auto blend_target_a = make_line_pose(-0.10, 0.05, -0.01);
  const auto blend_target_b = make_line_pose(-0.14, -0.05, -0.01);
  const auto blend_target_c = make_line_pose(-0.08, 0.00, -0.04);
  const auto speed_target = make_line_pose(-0.24, 0.14, -0.03);
  auto spiral_target = start_pose;
  spiral_target[2] += 0.035;
  spiral_target[3] -= 0.03;
  spiral_target[4] += 0.02;
  spiral_target[5] -= 0.03;
  const std::array<double, 6> move_absj_short_target = {0.10, 0.36, 0.12, 0.02, 0.66, -0.04};
  const auto smoke_line_target = make_line_pose(-0.015, 0.005, -0.005);

  const std::vector<rokae::MoveLCommand> long_line_commands{
      rokae::MoveLCommand(toCartesian(long_line_target), 220, 0)};
  const std::vector<rokae::MoveCCommand> circle_commands{
      rokae::MoveCCommand(toCartesian(circle_target), toCartesian(circle_aux), 180, 0)};
  const std::vector<rokae::MoveLCommand> zone_blend_commands{
      rokae::MoveLCommand(toCartesian(blend_target_a), 180, 20),
      rokae::MoveLCommand(toCartesian(blend_target_b), 180, 20),
      rokae::MoveLCommand(toCartesian(blend_target_c), 180, 0)};
  const std::vector<rokae::MoveLCommand> zone_stop_commands{
      rokae::MoveLCommand(toCartesian(blend_target_a), 180, 0),
      rokae::MoveLCommand(toCartesian(blend_target_b), 180, 0),
      rokae::MoveLCommand(toCartesian(blend_target_c), 180, 0)};
  const std::vector<rokae::MoveLCommand> speed_scale_commands{
      rokae::MoveLCommand(toCartesian(speed_target), 200, 0)};
  const std::vector<rokae::MoveSPCommand> spiral_commands{
      rokae::MoveSPCommand(toCartesian(spiral_target), 0.004, 0.0001, M_PI, true, 60, 0)};
  const std::vector<rokae::MoveAbsJCommand> move_absj_short_commands{
      rokae::MoveAbsJCommand(toVector(move_absj_short_target), 180, 0)};
  const std::vector<rokae::MoveLCommand> smoke_line_commands{
      rokae::MoveLCommand(toCartesian(smoke_line_target), 100, 0)};

  auto move_l_spec = [](const std::vector<double> &target, double speed, int zone) {
    rt::MotionCommandSpec spec;
    spec.kind = rt::MotionKind::move_l;
    spec.target_cartesian = target;
    spec.speed = speed;
    spec.zone = zone;
    return spec;
  };

  auto move_absj_spec = [](const std::array<double, 6> &target, double speed, int zone) {
    rt::MotionCommandSpec spec;
    spec.kind = rt::MotionKind::move_absj;
    spec.target_joints = toVector(target);
    spec.speed = speed;
    spec.zone = zone;
    return spec;
  };

  auto move_c_spec = [](const std::vector<double> &target,
                        const std::vector<double> &aux,
                        double speed,
                        int zone) {
    rt::MotionCommandSpec spec;
    spec.kind = rt::MotionKind::move_c;
    spec.target_cartesian = target;
    spec.aux_cartesian = aux;
    spec.speed = speed;
    spec.zone = zone;
    return spec;
  };

  auto move_sp_spec = [](const std::vector<double> &target,
                         double radius,
                         double radius_step,
                         double angle,
                         bool direction,
                         double speed,
                         int zone) {
    rt::MotionCommandSpec spec;
    spec.kind = rt::MotionKind::move_sp;
    spec.target_cartesian = target;
    spec.radius = radius;
    spec.radius_step = radius_step;
    spec.angle = angle;
    spec.direction = direction;
    spec.speed = speed;
    spec.zone = zone;
    return spec;
  };

  const std::vector<Scenario> scenarios =
      mode == RegressionMode::strict
          ? std::vector<Scenario>{
                Scenario{
                    "move_l_long",
                    220.0,
                    0,
                    {move_l_spec(long_line_target, 220.0, 0)},
                    [long_line_commands](rokae::xMateRobot &robot_ref, std::error_code &ec) {
                      startCommands(robot_ref, long_line_commands, ec);
                    },
                    std::nullopt,
                    kAdjustAfterStartSec,
                    true,
                },
                Scenario{
                    "move_c_single",
                    180.0,
                    0,
                    {move_c_spec(circle_target, circle_aux, 180.0, 0)},
                    [circle_commands](rokae::xMateRobot &robot_ref, std::error_code &ec) {
                      startCommands(robot_ref, circle_commands, ec);
                    },
                    std::nullopt,
                    kAdjustAfterStartSec,
                    true,
                },
                Scenario{
                    "move_l_zone_blend",
                    180.0,
                    20,
                    {
                        move_l_spec(blend_target_a, 180.0, 20),
                        move_l_spec(blend_target_b, 180.0, 20),
                        move_l_spec(blend_target_c, 180.0, 0),
                    },
                    [zone_blend_commands](rokae::xMateRobot &robot_ref, std::error_code &ec) {
                      startCommands(robot_ref, zone_blend_commands, ec);
                    },
                    std::nullopt,
                    kAdjustAfterStartSec,
                    true,
                },
                Scenario{
                    "move_l_zone_stop",
                    180.0,
                    0,
                    {
                        move_l_spec(blend_target_a, 180.0, 0),
                        move_l_spec(blend_target_b, 180.0, 0),
                        move_l_spec(blend_target_c, 180.0, 0),
                    },
                    [zone_stop_commands](rokae::xMateRobot &robot_ref, std::error_code &ec) {
                      startCommands(robot_ref, zone_stop_commands, ec);
                    },
                    std::nullopt,
                    kAdjustAfterStartSec,
                    true,
                },
                Scenario{
                    "move_sp_single",
                    60.0,
                    0,
                    {move_sp_spec(spiral_target, 0.004, 0.0001, M_PI, true, 60.0, 0)},
                    [spiral_commands](rokae::xMateRobot &robot_ref, std::error_code &ec) {
                      startCommands(robot_ref, spiral_commands, ec);
                    },
                    std::nullopt,
                    kAdjustAfterStartSec,
                    true,
                },
                Scenario{
                    "move_l_speed_fast",
                    200.0,
                    0,
                    {move_l_spec(speed_target, 200.0, 0)},
                    [speed_scale_commands](rokae::xMateRobot &robot_ref, std::error_code &ec) {
                      startCommands(robot_ref, speed_scale_commands, ec);
                    },
                    2.0,
                    kAdjustAfterStartSec,
                    false,
                },
                Scenario{
                    "move_l_speed_nominal",
                    200.0,
                    0,
                    {move_l_spec(speed_target, 200.0, 0)},
                    [speed_scale_commands](rokae::xMateRobot &robot_ref, std::error_code &ec) {
                      startCommands(robot_ref, speed_scale_commands, ec);
                    },
                    1.0,
                    kAdjustAfterStartSec,
                    false,
                },
                Scenario{
                    "move_l_speed_slow",
                    200.0,
                    0,
                    {move_l_spec(speed_target, 200.0, 0)},
                    [speed_scale_commands](rokae::xMateRobot &robot_ref, std::error_code &ec) {
                      startCommands(robot_ref, speed_scale_commands, ec);
                    },
                    0.25,
                    kAdjustAfterStartSec,
                    false,
                },
            }
          : std::vector<Scenario>{
                Scenario{
                    "move_absj_short",
                    180.0,
                    0,
                    {move_absj_spec(move_absj_short_target, 180.0, 0)},
                    [move_absj_short_commands](rokae::xMateRobot &robot_ref, std::error_code &ec) {
                      startCommands(robot_ref, move_absj_short_commands, ec);
                    },
                    std::nullopt,
                    kAdjustAfterStartSec,
                    false,
                },
            };

  std::vector<CaseMetrics> all_metrics;
  all_metrics.reserve(scenarios.size());

  for (const auto &scenario : scenarios) {
    auto metrics = runScenario(scenario, robot, recorder, kinematics, ready_pose, mode);
    if (mode == RegressionMode::strict && scenario.name == "move_l_zone_blend") {
      bool has_blend = false;
      rt::MotionPlanner planner;
      const auto plan = planner.plan(
          buildRequest(scenario.name + "_verify", ready_pose_vector, scenario.default_speed_mm_per_s,
                       scenario.default_zone_mm, scenario.specs));
      if (plan.valid()) {
        for (const auto &segment : plan.segments) {
          has_blend = has_blend || segment.blend_to_next;
        }
      }
      if (!has_blend) {
        addFailure(metrics, "zone>0 reference plan did not produce blend_to_next");
      }
      if (metrics.max_junction_plateau_sec >= kJunctionPlateauSec) {
        std::ostringstream oss;
        oss << "zone blend plateau too long: " << metrics.max_junction_plateau_sec << " s";
        addFailure(metrics, oss.str());
      }
    }
    if (mode == RegressionMode::strict &&
        scenario.name == "move_l_zone_stop" &&
        metrics.max_junction_plateau_sec < kJunctionPlateauSec) {
      std::ostringstream oss;
      oss << "zone=0 should stop at junction, observed plateau " << metrics.max_junction_plateau_sec << " s";
      addFailure(metrics, oss.str());
    }

    exportMetrics(results_dir, metrics);
    reportCase(metrics);
    all_metrics.push_back(metrics);
    bestEffortRecover(robot);
  }

  auto find_case = [&](const std::string &name) -> CaseMetrics * {
    for (auto &metrics : all_metrics) {
      if (metrics.name == name) {
        return &metrics;
      }
    }
    return nullptr;
  };

  if (auto *blend = find_case("move_l_zone_blend"), *stop = find_case("move_l_zone_stop");
      mode == RegressionMode::strict && blend != nullptr && stop != nullptr) {
    const double required_gap = std::max(kZoneDurationGapSec, kZoneDurationGapRatio * stop->actual_duration_sec);
    if (!(blend->actual_duration_sec + required_gap <= stop->actual_duration_sec)) {
      std::ostringstream oss;
      oss << "zone>0 duration should be shorter than zone=0 by at least " << required_gap
          << " s, observed blend=" << blend->actual_duration_sec
          << " stop=" << stop->actual_duration_sec;
      addFailure(*blend, oss.str());
      addFailure(*stop, oss.str());
      exportMetrics(results_dir, *blend);
      exportMetrics(results_dir, *stop);
    }
  }

  if (auto *fast = find_case("move_l_speed_fast"),
      *nominal = find_case("move_l_speed_nominal"),
      *slow = find_case("move_l_speed_slow");
      mode == RegressionMode::strict && fast != nullptr && nominal != nullptr && slow != nullptr) {
    if (!(fast->remaining_after_adjust_sec > 0.0 && nominal->remaining_after_adjust_sec > 0.0 &&
          slow->remaining_after_adjust_sec > 0.0)) {
      const std::string message = "missing remaining duration after speed adjustment";
      addFailure(*fast, message);
      addFailure(*nominal, message);
      addFailure(*slow, message);
    } else {
      if (!(fast->remaining_after_adjust_sec < nominal->remaining_after_adjust_sec &&
            nominal->remaining_after_adjust_sec < slow->remaining_after_adjust_sec)) {
        const std::string message = "expected fast < nominal < slow completion ordering after speed adjustment";
        addFailure(*fast, message);
        addFailure(*nominal, message);
        addFailure(*slow, message);
      }
      if (!(fast->remaining_after_adjust_sec <= 0.65 * nominal->remaining_after_adjust_sec)) {
        std::ostringstream oss;
        oss << "fast remaining duration too large: fast=" << fast->remaining_after_adjust_sec
            << " nominal=" << nominal->remaining_after_adjust_sec;
        addFailure(*fast, oss.str());
      }
      if (!(slow->remaining_after_adjust_sec >= 1.8 * nominal->remaining_after_adjust_sec)) {
        std::ostringstream oss;
        oss << "slow remaining duration too small: slow=" << slow->remaining_after_adjust_sec
            << " nominal=" << nominal->remaining_after_adjust_sec;
        addFailure(*slow, oss.str());
      }
    }
    exportMetrics(results_dir, *fast);
    exportMetrics(results_dir, *nominal);
    exportMetrics(results_dir, *slow);
  }

  writeSummaryJson(results_dir / "summary.json", all_metrics);

  std::error_code cleanup_ec;
  robot.setPowerState(false, cleanup_ec);
  cleanup_ec.clear();
  robot.disconnectFromRobot(cleanup_ec);

  recorder.stop();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  const auto failed_cases =
      static_cast<std::size_t>(std::count_if(all_metrics.begin(), all_metrics.end(), [](const CaseMetrics &metrics) {
        return !metrics.success;
      }));
  if (failed_cases > 0) {
    std::cout << "gazebo_sdk_regression mode="
              << (mode == RegressionMode::strict ? "strict" : "default")
              << " collected " << failed_cases
              << " failing case(s); see summary.json for details" << std::endl;
    return 1;
  }
  return 0;
}
