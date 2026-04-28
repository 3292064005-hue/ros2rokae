#include <mutex>
#include <utility>

#include "compat/internal/compat_shared.hpp"
#include "compat/internal/model_api_impl.hpp"

namespace rokae {

xMateModel<6>::xMateModel(std::shared_ptr<detail::CompatRobotHandle> handle)
    : xMateModel() {
  if (!handle) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(handle->mutex);
    impl_->load = handle->model_load_cache;
    impl_->f_t_ee = handle->model_f_t_ee;
    impl_->ee_t_k = handle->model_ee_t_k;
  }

  std::weak_ptr<detail::CompatRobotHandle> weak_handle = handle;
  impl_->current_seed_provider = [weak_handle]() {
    if (const auto locked = weak_handle.lock()) {
      if (locked->backend) {
        error_code ec;
        const auto joints = locked->backend->jointPos(ec);
        if (!ec) {
          return joints;
        }
      }
    }
    return std::array<double, 6>{};
  };
  impl_->sync_load = [weak_handle](const Load &load) {
    if (const auto locked = weak_handle.lock()) {
      std::lock_guard<std::mutex> lock(locked->mutex);
      locked->model_load_cache = load;
    }
  };
  impl_->sync_tcp = [weak_handle](const std::array<double, 16> &f_t_ee,
                                  const std::array<double, 16> &ee_t_k) {
    if (const auto locked = weak_handle.lock()) {
      std::lock_guard<std::mutex> lock(locked->mutex);
      locked->model_f_t_ee = f_t_ee;
      locked->model_ee_t_k = ee_t_k;
    }
  };
}

}  // namespace rokae
