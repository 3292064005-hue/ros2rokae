#ifndef ROKAE_XMATE3_ROS2_RUNTIME_BACKEND_PROVIDER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_BACKEND_PROVIDER_HPP

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "runtime/backend_contract_catalog.hpp"
#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

/**
 * @brief Generic backend factory request issued by a resolved runtime provider.
 *
 * The request names a host-supported backend factory key instead of binding the public provider
 * host interface to a fixed family of factory methods. Adding a new backend family therefore only
 * requires a host implementation + provider to agree on a new factory key; the public abstract
 * interface remains unchanged.
 */
struct RuntimeBackendFactoryRequest {
  std::string factory_key;
  bool attach_trajectory_client = false;
};

/**
 * @brief Host-owned backend creation adapter consumed by runtime backend providers.
 *
 * Providers are resolved independently from the host/runtime entry point. The host then passes a
 * stable adapter surface that exposes only the runtime-facing services a provider may need:
 * shared ROS node access, optional joint-name catalog, advertised backend factory keys, and one
 * generic backend creation hook. This keeps provider interfaces free from Gazebo/headless concrete
 * context types and prevents the host from constructing provider-native context variants or baking
 * backend-family-specific factory methods into the public abstraction.
 */
class RuntimeBackendProviderHost {
 public:
  virtual ~RuntimeBackendProviderHost() = default;

  [[nodiscard]] virtual std::string hostKind() const = 0;
  [[nodiscard]] virtual rclcpp::Node::SharedPtr node() const = 0;
  [[nodiscard]] virtual const std::vector<std::string> *jointNames() const = 0;
  [[nodiscard]] virtual bool supportsFactory(const std::string &factory_key) const noexcept = 0;
  [[nodiscard]] virtual std::vector<std::string> advertisedFactoryKeys() const = 0;
  [[nodiscard]] virtual std::unique_ptr<BackendInterface> createBackend(
      const RuntimeBackendFactoryRequest &request) const = 0;
};

/**
 * @brief Provider boundary between runtime host assembly and concrete backend implementations.
 *
 * The runtime host resolves one provider, asks it for a concrete backend instance through the
 * stable RuntimeBackendProviderHost adapter, and consumes only the runtime::BackendInterface +
 * BackendContractDescriptor surface afterwards. Host/bootstrap code must not hard-code
 * backend-specific construction branches, capability flag lists, or controller ownership rules.
 */
class RuntimeBackendProvider {
 public:
  virtual ~RuntimeBackendProvider() = default;

  [[nodiscard]] virtual std::string key() const = 0;
  [[nodiscard]] virtual const BackendContractDescriptor &contract() const noexcept = 0;
  [[nodiscard]] virtual std::vector<std::string> capabilityFlags() const = 0;
  [[nodiscard]] virtual std::unique_ptr<BackendInterface> createBackend(
      const RuntimeBackendProviderHost &host) const = 0;
  [[nodiscard]] virtual bool emitsExternalTrajectoryOwnershipWarning() const noexcept { return false; }
};

/**
 * @brief Replaceable registry for runtime backend providers.
 *
 * Builtin providers are registered lazily the first time the registry is used. Additional providers
 * may be registered (or existing ones replaced) without editing the host/bootstrap code-paths.
 */
class RuntimeBackendProviderRegistry {
 public:
  void registerProvider(std::shared_ptr<const RuntimeBackendProvider> provider);
  [[nodiscard]] bool unregisterProvider(const std::string &backend_key);
  [[nodiscard]] std::shared_ptr<const RuntimeBackendProvider> resolve(const std::string &backend_key) const;
  [[nodiscard]] std::vector<std::string> list() const;

 private:
  void ensureBuiltinsRegisteredLocked() const;

  mutable bool builtins_registered_ = false;
  mutable std::unordered_map<std::string, std::shared_ptr<const RuntimeBackendProvider>> providers_;
  mutable std::mutex mutex_;
};

[[nodiscard]] RuntimeBackendProviderRegistry &runtimeBackendProviderRegistry();
void registerRuntimeBackendProvider(std::shared_ptr<const RuntimeBackendProvider> provider);
[[nodiscard]] bool unregisterRuntimeBackendProvider(const std::string &backend_key);
[[nodiscard]] std::shared_ptr<const RuntimeBackendProvider> resolveRuntimeBackendProvider(
    const std::string &backend_key);
[[nodiscard]] std::vector<std::string> listRuntimeBackendProviders();

}  // namespace rokae_xmate3_ros2::runtime

#endif
