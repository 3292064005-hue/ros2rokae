#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_FAST_SHM_RING_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_FAST_SHM_RING_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "rokae_xmate3_ros2/runtime/rt_fast_command.hpp"

namespace rokae_xmate3_ros2::runtime {

struct RtFastShmRingShared;

class RtFastShmRingWriter {
 public:
  explicit RtFastShmRingWriter(const std::string &name = "/rokae_xmate3_rt_fast_ring");
  ~RtFastShmRingWriter();

  RtFastShmRingWriter(const RtFastShmRingWriter &) = delete;
  RtFastShmRingWriter &operator=(const RtFastShmRingWriter &) = delete;

  [[nodiscard]] bool ready() const noexcept;
  bool write(const RtFastCommandFrame &frame, std::uint32_t *queue_depth = nullptr) noexcept;

 private:
  struct Mapping;

  std::unique_ptr<Mapping> mapping_;
  RtFastShmRingShared *ring_ = nullptr;
};

class RtFastShmRingReader {
 public:
  explicit RtFastShmRingReader(const std::string &name = "/rokae_xmate3_rt_fast_ring");
  ~RtFastShmRingReader();

  RtFastShmRingReader(const RtFastShmRingReader &) = delete;
  RtFastShmRingReader &operator=(const RtFastShmRingReader &) = delete;

  [[nodiscard]] bool ready() const noexcept;
  bool readLatest(RtFastCommandFrame &frame, std::uint32_t *queue_depth = nullptr) noexcept;

 private:
  struct Mapping;

  std::unique_ptr<Mapping> mapping_;
  RtFastShmRingShared *ring_ = nullptr;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
