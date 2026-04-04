#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_SEMANTIC_TOPICS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_SEMANTIC_TOPICS_HPP

namespace rokae_xmate3_ros2::runtime::rt_topics {

inline constexpr const char *kControlJointPosition = "rt/control/joint_position";
inline constexpr const char *kControlCartesianPosition = "rt/control/cartesian_position";
inline constexpr const char *kControlTorque = "rt/control/torque";
inline constexpr const char *kControlStop = "rt/control/stop";
inline constexpr const char *kControlDispatchMode = "rt/control/dispatch_mode";
inline constexpr const char *kControlSurface = "rt/control/api_surface";
inline constexpr const char *kControlSequence = "rt/control/sequence";

inline constexpr const char *kConfigRtNetworkTolerance = "rt/config/network_tolerance";
inline constexpr const char *kConfigUseRciClient = "rt/config/use_rci_client";
inline constexpr const char *kConfigJointImpedance = "rt/config/joint_impedance";
inline constexpr const char *kConfigCartesianImpedance = "rt/config/cartesian_impedance";
inline constexpr const char *kConfigFilterFrequency = "rt/config/filter_frequency";
inline constexpr const char *kConfigFilterLimit = "rt/config/filter_limit";
inline constexpr const char *kConfigCartesianDesiredWrench = "rt/config/cartesian_desired_wrench";
inline constexpr const char *kConfigCollisionBehaviourThresholds = "rt/config/collision_behaviour_thresholds";
inline constexpr const char *kConfigTorqueCutoffFrequency = "rt/config/torque_cutoff_frequency";
inline constexpr const char *kConfigForceControlFrame = "rt/config/fc_frame";
inline constexpr const char *kConfigCartesianLimit = "rt/config/cartesian_limit";
inline constexpr const char *kConfigEndEffectorFrame = "rt/config/end_effector_frame";
inline constexpr const char *kConfigLoad = "rt/config/load";
inline constexpr const char *kCatalogProvenance = "runtime/catalog/provenance";

}  // namespace rokae_xmate3_ros2::runtime::rt_topics

#endif
