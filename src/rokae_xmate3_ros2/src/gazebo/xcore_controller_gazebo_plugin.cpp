/**
 * @file xcore_controller_gazebo_plugin.cpp
 * @brief xCore SDK Gazebo 仿真控制器插件
 *
 * 这个插件直接控制Gazebo关节来实现xCore SDK功能，
 * 同时配合ros2_control的joint_state_broadcaster提供标准ROS2接口。
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <atomic>
#include <thread>
#include <memory>
#include <map>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <limits>

#include <builtin_interfaces/msg/time.hpp>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"
#include "runtime/runtime_publish_bridge.hpp"
#include "runtime/runtime_context.hpp"
#include "runtime/ros_bindings.hpp"

// ROS2 接口头文件
#include "rokae_xmate3_ros2/msg/operation_state.hpp"



namespace gazebo {

namespace {

constexpr double kTrajectorySampleDt = 0.01;

}  // namespace

namespace runtime = rokae_xmate3_ros2::runtime;

class GazeboRuntimeBackend final : public runtime::BackendInterface {
 public:
  GazeboRuntimeBackend(std::vector<physics::JointPtr> *joints,
                       const std::array<std::pair<double, double>, 6> *original_joint_limits)
      : joints_(joints), original_joint_limits_(original_joint_limits) {}

  runtime::RobotSnapshot readSnapshot() const override {
    runtime::RobotSnapshot snapshot;
    if (!joints_) {
      return snapshot;
    }
    for (size_t i = 0; i < 6 && i < joints_->size(); ++i) {
      snapshot.joint_position[i] = (*joints_)[i]->Position(0);
      snapshot.joint_velocity[i] = (*joints_)[i]->GetVelocity(0);
      snapshot.joint_torque[i] = (*joints_)[i]->GetForce(0);
    }
    return snapshot;
  }

  void applyControl(const runtime::ControlCommand &command) override {
    if (!joints_ || !command.has_effort) {
      clearControl();
      return;
    }
    for (size_t i = 0; i < 6 && i < joints_->size(); ++i) {
      (*joints_)[i]->SetForce(0, command.effort[i]);
    }
  }

  void clearControl() override {
    if (!joints_) {
      return;
    }
    for (size_t i = 0; i < 6 && i < joints_->size(); ++i) {
      (*joints_)[i]->SetForce(0, 0.0);
    }
  }

  void setBrakeLock(const runtime::RobotSnapshot &snapshot, bool locked) override {
    if (!joints_ || !original_joint_limits_ || locked == brakes_locked_) {
      return;
    }

    if (locked) {
      clearControl();
      for (size_t i = 0; i < 6 && i < joints_->size(); ++i) {
        (*joints_)[i]->SetLowerLimit(0, snapshot.joint_position[i]);
        (*joints_)[i]->SetUpperLimit(0, snapshot.joint_position[i]);
        (*joints_)[i]->SetVelocity(0, 0.0);
      }
    } else {
      for (size_t i = 0; i < 6 && i < joints_->size(); ++i) {
        (*joints_)[i]->SetLowerLimit(0, (*original_joint_limits_)[i].first);
        (*joints_)[i]->SetUpperLimit(0, (*original_joint_limits_)[i].second);
      }
    }

    brakes_locked_ = locked;
  }

  [[nodiscard]] bool brakesLocked() const override { return brakes_locked_; }

 private:
  std::vector<physics::JointPtr> *joints_;
  const std::array<std::pair<double, double>, 6> *original_joint_limits_;
  bool brakes_locked_ = false;
};

class XCoreControllerPlugin : public ModelPlugin {
public:
    XCoreControllerPlugin() : ModelPlugin() {}
    ~XCoreControllerPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
    void InitROS2();
    void InitPublishers();
    void OnUpdate();
    void ExecuteMotion();
    [[nodiscard]] runtime::MotionRuntime *motionRuntime();
    [[nodiscard]] const runtime::MotionRuntime *motionRuntime() const;
    void RefreshJointStateCache();
    void GetCachedJointState(std::array<double, 6> &position,
                             std::array<double, 6> &velocity,
                             std::array<double, 6> &torque);
    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;
    std::vector<physics::JointPtr> joints_;
    std::vector<std::string> joint_names_;
    int joint_num_ = 0;
    event::ConnectionPtr update_conn_;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    std::unique_ptr<xMate3Kinematics> kinematics_;

    std::unique_ptr<runtime::RuntimeContext> runtime_context_;
    std::unique_ptr<runtime::RosBindings> ros_bindings_;
    std::unique_ptr<runtime::RuntimePublishBridge> publish_bridge_;
    bool joint_positions_initialized_ = false;

    bool initial_pose_set_ = false;
    std::array<std::pair<double, double>, 6> original_joint_limits_;
    const std::vector<double> default_initial_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double current_update_dt_ = kTrajectorySampleDt;
    double last_sim_time_ = -1.0;
    std::mutex joint_state_cache_mutex_;
    std::array<double, 6> cached_joint_position_{};
    std::array<double, 6> cached_joint_velocity_{};
    std::array<double, 6> cached_joint_torque_{};
    std::unique_ptr<GazeboRuntimeBackend> motion_backend_;
    std::atomic<uint64_t> next_request_id_{1};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<rokae_xmate3_ros2::msg::OperationState>::SharedPtr operation_state_pub_;
};

void XCoreControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    gzmsg << "[xCore Controller] Loading plugin for model: " << _model->GetName() << std::endl;

    model_ = _model;
    sdf_ = _sdf;
    // 获取关节 - 直接用于控制
    joint_names_ = {"xmate_joint_1", "xmate_joint_2", "xmate_joint_3",
                    "xmate_joint_4", "xmate_joint_5", "xmate_joint_6"};
    for (const auto& name : joint_names_) {
        auto joint = _model->GetJoint(name);
        if (joint) {
            joints_.push_back(joint);
        } else {
            gzerr << "[xCore] Joint not found: " << name << std::endl;
        }
    }
    joint_num_ = joints_.size();

    // 保存URDF原始关节限位
    const std::array<std::pair<double, double>, 6> default_limits = {
        std::make_pair(-3.0527, 3.0527), // joint1
        std::make_pair(-2.0933, 2.0933), // joint2
        std::make_pair(-2.0933, 2.0933), // joint3
        std::make_pair(-3.0527, 3.0527), // joint4
        std::make_pair(-2.0933, 2.0933), // joint5
        std::make_pair(-6.1082, 6.1082)  // joint6
    };
    for (int i = 0; i < 6 && i < joint_num_; ++i) {
        original_joint_limits_[i] = default_limits[i];
    }

    motion_backend_ = std::make_unique<GazeboRuntimeBackend>(&joints_, &original_joint_limits_);
    runtime_context_ = std::make_unique<runtime::RuntimeContext>();
    runtime_context_->attachBackend(motion_backend_.get());

    // ========== 删除原来这里的初始位置设置代码 ==========
    joint_positions_initialized_ = false;

    InitROS2();

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&XCoreControllerPlugin::OnUpdate, this));

    gzmsg << "[xCore Controller] Plugin loaded successfully, joints: " << joint_num_ << std::endl;
}

XCoreControllerPlugin::~XCoreControllerPlugin() {
    if (executor_) {
        executor_->cancel();
    }
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    if (executor_ && node_) {
        executor_->remove_node(node_);
    }
    executor_.reset();
    node_.reset();
}

void XCoreControllerPlugin::InitROS2() {
    if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("xcore_gazebo_controller");
    kinematics_ = std::make_unique<xMate3Kinematics>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    auto joint_state_fetcher = [this](std::array<double, 6> &position,
                                      std::array<double, 6> &velocity,
                                      std::array<double, 6> &torque) {
        GetCachedJointState(position, velocity, torque);
    };
    auto time_provider = [this]() { return node_->get_clock()->now(); };
    auto trajectory_dt_provider = [this]() {
        return current_update_dt_ > 1e-6 ? current_update_dt_ : kTrajectorySampleDt;
    };
    auto request_id_generator = [this](const std::string &prefix) {
        return prefix + std::to_string(next_request_id_.fetch_add(1));
    };

    InitPublishers();
    ros_bindings_ = std::make_unique<runtime::RosBindings>(
        node_,
        *runtime_context_,
        *kinematics_,
        joint_state_fetcher,
        time_provider,
        trajectory_dt_provider,
        request_id_generator);
    publish_bridge_ = std::make_unique<runtime::RuntimePublishBridge>(
        *runtime_context_,
        [this]() {
            return ros_bindings_ != nullptr && ros_bindings_->isRecordingPath();
        },
        [this](const std::array<double, 6> &joint_position) {
            if (ros_bindings_ != nullptr) {
                ros_bindings_->recordPathSample(joint_position);
            }
        });

    executor_thread_ = std::thread([this]() {
        try {
            executor_->spin();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "ROS executor stopped unexpectedly: %s", e.what());
        }
    });
}

void XCoreControllerPlugin::InitPublishers() {
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/xmate3/joint_states", 10);
    operation_state_pub_ = node_->create_publisher<rokae_xmate3_ros2::msg::OperationState>("/xmate3/cobot/operation_state", 10);
}

void XCoreControllerPlugin::RefreshJointStateCache() {
    std::array<double, 6> position{};
    std::array<double, 6> velocity{};
    std::array<double, 6> torque{};
    for (int i = 0; i < 6 && i < joint_num_; ++i) {
        position[i] = joints_[i]->Position(0);
        velocity[i] = joints_[i]->GetVelocity(0);
        torque[i] = joints_[i]->GetForce(0);
    }
    std::lock_guard<std::mutex> lock(joint_state_cache_mutex_);
    cached_joint_position_ = position;
    cached_joint_velocity_ = velocity;
    cached_joint_torque_ = torque;
}

void XCoreControllerPlugin::GetCachedJointState(std::array<double, 6> &position,
                                                std::array<double, 6> &velocity,
                                                std::array<double, 6> &torque) {
    std::lock_guard<std::mutex> lock(joint_state_cache_mutex_);
    position = cached_joint_position_;
    velocity = cached_joint_velocity_;
    torque = cached_joint_torque_;
}



runtime::MotionRuntime *XCoreControllerPlugin::motionRuntime() {
    return runtime_context_ ? &runtime_context_->motionRuntime() : nullptr;
}

const runtime::MotionRuntime *XCoreControllerPlugin::motionRuntime() const {
    return runtime_context_ ? &runtime_context_->motionRuntime() : nullptr;
}

void XCoreControllerPlugin::OnUpdate() {
    double sim_dt = kTrajectorySampleDt;
    if (model_ && model_->GetWorld()) {
        const double sim_time = model_->GetWorld()->SimTime().Double();
        if (last_sim_time_ >= 0.0) {
            const double candidate_dt = sim_time - last_sim_time_;
            if (candidate_dt > 1e-6 && candidate_dt < 0.1) {
                sim_dt = candidate_dt;
            }
        }
        last_sim_time_ = sim_time;
    }
    current_update_dt_ = sim_dt;

    // ========== 【最高优先级】强制设置初始姿态（仅执行一次） ==========
    if (!initial_pose_set_ && joint_num_ > 0) {
        gzmsg << "[xCore Controller] Setting initial joint pose..." << std::endl;
        for (int i = 0; i < joint_num_ && i < static_cast<int>(default_initial_pose_.size()); ++i) {
            // 第三个参数true：跳过物理约束，强制设置关节位置
            joints_[i]->SetPosition(0, default_initial_pose_[i], true);
            // 清空关节速度和力，避免残留值导致姿态跳变
            joints_[i]->SetVelocity(0, 0.0);
            joints_[i]->SetForce(0, 0.0);
        }
        // 标记初始化完成
        initial_pose_set_ = true;
        joint_positions_initialized_ = true;
        gzmsg << "[xCore Controller] Initial pose set successfully!" << std::endl;
    }

    // 运动控制/抱闸逻辑（必须在初始姿态设置完成后执行）
    if (initial_pose_set_) {
        ExecuteMotion();
    }

    RefreshJointStateCache();

    // 原有发布逻辑
    static auto last_pub_time = node_->now();
    auto now = node_->now();
    if ((now - last_pub_time).seconds() >= 0.001 && publish_bridge_ != nullptr) {
        std::array<double, 6> cached_pos{};
        std::array<double, 6> cached_vel{};
        std::array<double, 6> cached_torque{};
        GetCachedJointState(cached_pos, cached_vel, cached_torque);
        joint_state_pub_->publish(
            publish_bridge_->buildJointStateMessage(
                now, "base_link", joint_names_, cached_pos, cached_vel, cached_torque));
        operation_state_pub_->publish(publish_bridge_->buildOperationStateMessage());
        last_pub_time = now;
    }

    // 路径录制逻辑
    if (publish_bridge_ != nullptr) {
        std::array<double, 6> cached_pos{};
        std::array<double, 6> cached_vel{};
        std::array<double, 6> cached_torque{};
        GetCachedJointState(cached_pos, cached_vel, cached_torque);
        publish_bridge_->maybeRecordPathSample(cached_pos);
    }

}

void XCoreControllerPlugin::ExecuteMotion() {
    if (!motion_backend_) {
        return;
    }

    const auto snapshot = motion_backend_->readSnapshot();

    if (!runtime_context_->sessionState().powerOn() && initial_pose_set_) {
        if (motionRuntime() != nullptr) {
            motionRuntime()->stop("power off");
        }
        const bool was_locked = motion_backend_->brakesLocked();
        if (!was_locked) {
            gzmsg << "[xCore Controller] Engaging joint brakes..." << std::endl;
        }
        motion_backend_->setBrakeLock(snapshot, true);
        if (!was_locked && motion_backend_->brakesLocked()) {
            gzmsg << "[xCore Controller] Joint brakes locked at initial pose." << std::endl;
        }
        return;
    }

    if (motion_backend_->brakesLocked()) {
        gzmsg << "[xCore Controller] Releasing joint brakes..." << std::endl;
        motion_backend_->setBrakeLock(snapshot, false);
        gzmsg << "[xCore Controller] Joint brakes released." << std::endl;
    }

    if (runtime_context_->sessionState().dragMode()) {
        motion_backend_->clearControl();
        return;
    }

    if (motionRuntime() != nullptr) {
        const auto status = motionRuntime()->tick(*motion_backend_, current_update_dt_);
        if (publish_bridge_ != nullptr && node_ != nullptr) {
            publish_bridge_->emitRuntimeStatus(status, node_->now(), node_->get_logger());
        }
    } else {
        motion_backend_->clearControl();
    }
}

GZ_REGISTER_MODEL_PLUGIN(XCoreControllerPlugin)

} // namespace gazebo
