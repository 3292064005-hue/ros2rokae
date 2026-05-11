#include "runtime/backend_provider.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr const char *kGazeboRuntimeFactoryKey = "gazebo_runtime";
constexpr const char *kHeadlessSimFactoryKey = "headless_sim";

class RequestedFactoryRuntimeBackendProvider final : public RuntimeBackendProvider {
 public:
  RequestedFactoryRuntimeBackendProvider(BackendContractDescriptor descriptor,
                                         std::vector<std::string> extra_flags,
                                         RuntimeBackendFactoryRequest request,
                                         bool warn_external_trajectory_owner)
      : contract_(std::move(descriptor)),
        extra_flags_(std::move(extra_flags)),
        request_(std::move(request)),
        warn_external_trajectory_owner_(warn_external_trajectory_owner) {}

  [[nodiscard]] std::string key() const override { return contract_.backend_mode; }

  [[nodiscard]] const BackendContractDescriptor &contract() const noexcept override { return contract_; }

  [[nodiscard]] std::vector<std::string> capabilityFlags() const override {
    return mergeBackendCapabilityFlags(contract_, extra_flags_);
  }

  [[nodiscard]] std::unique_ptr<BackendInterface> createBackend(
      const RuntimeBackendProviderHost &host) const override {
    if (!host.supportsFactory(request_.factory_key)) {
      throw std::runtime_error(
          "runtime backend provider '" + contract_.backend_mode +
          "' requires backend factory '" + request_.factory_key +
          "' which is unavailable from host kind '" + host.hostKind() + "'");
    }
    return host.createBackend(request_);
  }

  [[nodiscard]] bool emitsExternalTrajectoryOwnershipWarning() const noexcept override {
    return warn_external_trajectory_owner_;
  }

 private:
  BackendContractDescriptor contract_{};
  std::vector<std::string> extra_flags_{};
  RuntimeBackendFactoryRequest request_{};
  bool warn_external_trajectory_owner_ = false;
};

std::vector<std::shared_ptr<const RuntimeBackendProvider>> builtinProviders() {
  return {
      std::make_shared<RequestedFactoryRuntimeBackendProvider>(
          describeBackendMode("jtc"),
          std::vector<std::string>{"compat.alias.get_joint_torque", "compat.alias.get_end_torque"},
          RuntimeBackendFactoryRequest{kGazeboRuntimeFactoryKey, true, false},
          true),
      std::make_shared<RequestedFactoryRuntimeBackendProvider>(
          describeBackendMode("hybrid"),
          std::vector<std::string>{"compat.alias.get_joint_torque", "compat.alias.get_end_torque"},
          RuntimeBackendFactoryRequest{kGazeboRuntimeFactoryKey, true, true},
          false),
      std::make_shared<RequestedFactoryRuntimeBackendProvider>(
          describeBackendMode("effort"),
          std::vector<std::string>{"compat.alias.get_joint_torque", "compat.alias.get_end_torque"},
          RuntimeBackendFactoryRequest{kGazeboRuntimeFactoryKey, false, true},
          false),
      std::make_shared<RequestedFactoryRuntimeBackendProvider>(
          describeBackendMode("headless_sim"),
          std::vector<std::string>{},
          RuntimeBackendFactoryRequest{kHeadlessSimFactoryKey, false, true},
          false),
  };
}

std::string normalizeProviderKey(const std::string &backend_key) {
  return normalizeBackendModeKey(backend_key);
}

}  // namespace

void RuntimeBackendProviderRegistry::ensureBuiltinsRegisteredLocked() const {
  if (builtins_registered_) {
    return;
  }
  for (const auto &provider : builtinProviders()) {
    providers_[normalizeProviderKey(provider->key())] = provider;
  }
  builtins_registered_ = true;
}

void RuntimeBackendProviderRegistry::registerProvider(std::shared_ptr<const RuntimeBackendProvider> provider) {
  if (provider == nullptr) {
    throw std::invalid_argument("runtime backend provider must not be null");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  ensureBuiltinsRegisteredLocked();
  providers_[normalizeProviderKey(provider->key())] = std::move(provider);
}

bool RuntimeBackendProviderRegistry::unregisterProvider(const std::string &backend_key) {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureBuiltinsRegisteredLocked();
  return providers_.erase(normalizeProviderKey(backend_key)) > 0;
}

std::shared_ptr<const RuntimeBackendProvider> RuntimeBackendProviderRegistry::resolve(
    const std::string &backend_key) const {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureBuiltinsRegisteredLocked();
  const auto normalized = normalizeProviderKey(backend_key);
  const auto it = providers_.find(normalized);
  if (it != providers_.end()) {
    return it->second;
  }
  throw std::runtime_error("unsupported runtime backend provider: " + backend_key);
}

std::vector<std::string> RuntimeBackendProviderRegistry::list() const {
  std::lock_guard<std::mutex> lock(mutex_);
  ensureBuiltinsRegisteredLocked();
  std::vector<std::string> providers;
  providers.reserve(providers_.size());
  for (const auto &entry : providers_) {
    providers.push_back(entry.first);
  }
  std::sort(providers.begin(), providers.end());
  return providers;
}

RuntimeBackendProviderRegistry &runtimeBackendProviderRegistry() {
  static RuntimeBackendProviderRegistry registry;
  return registry;
}

void registerRuntimeBackendProvider(std::shared_ptr<const RuntimeBackendProvider> provider) {
  runtimeBackendProviderRegistry().registerProvider(std::move(provider));
}

bool unregisterRuntimeBackendProvider(const std::string &backend_key) {
  return runtimeBackendProviderRegistry().unregisterProvider(backend_key);
}

std::shared_ptr<const RuntimeBackendProvider> resolveRuntimeBackendProvider(const std::string &backend_key) {
  return runtimeBackendProviderRegistry().resolve(backend_key);
}

std::vector<std::string> listRuntimeBackendProviders() {
  return runtimeBackendProviderRegistry().list();
}

}  // namespace rokae_xmate3_ros2::runtime
