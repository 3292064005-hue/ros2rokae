#include "rokae_xmate3_ros2/runtime/rt_fast_shm_ring.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace rokae_xmate3_ros2::runtime {

constexpr std::uint32_t kRtFastRingCapacity = 2048;

struct RtFastCommandPod {
  std::uint64_t sequence;
  int rt_mode;
  std::uint8_t kind;
  std::uint8_t finished;
  std::int64_t sent_time_ns;
  std::array<double, 6> values;
  char dispatch_mode[64];
};

struct alignas(64) RtFastShmRingShared {
  std::atomic<std::uint32_t> initialized{0};
  std::atomic<std::uint64_t> write_index{0};
  std::atomic<std::uint64_t> read_index{0};
  std::atomic<std::uint64_t> dropped_count{0};
  RtFastCommandPod slots[kRtFastRingCapacity];
};

namespace {

constexpr std::uint32_t kDispatchModeMax = 63;

inline std::int64_t toSteadyNs(const std::chrono::steady_clock::time_point tp) noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}

inline std::chrono::steady_clock::time_point fromSteadyNs(const std::int64_t ns) noexcept {
  return std::chrono::steady_clock::time_point{std::chrono::nanoseconds(ns)};
}

inline RtFastCommandPod toPod(const RtFastCommandFrame &frame) noexcept {
  RtFastCommandPod pod{};
  pod.sequence = frame.sequence;
  pod.rt_mode = frame.rt_mode;
  pod.kind = static_cast<std::uint8_t>(frame.kind);
  pod.finished = frame.finished ? 1U : 0U;
  pod.sent_time_ns = toSteadyNs(frame.sent_at);
  pod.values = frame.values;
  std::snprintf(pod.dispatch_mode, sizeof(pod.dispatch_mode), "%s", frame.dispatch_mode.c_str());
  return pod;
}

inline RtFastCommandFrame fromPod(const RtFastCommandPod &pod) noexcept {
  RtFastCommandFrame frame;
  frame.sequence = pod.sequence;
  frame.rt_mode = pod.rt_mode;
  frame.kind = static_cast<RtFastCommandKind>(pod.kind);
  frame.finished = pod.finished != 0U;
  frame.sent_at = fromSteadyNs(pod.sent_time_ns);
  frame.values = pod.values;
  frame.dispatch_mode = pod.dispatch_mode;
  return frame;
}

inline bool shouldCreateRing(const int fd) noexcept {
  struct stat st {};
  if (fstat(fd, &st) != 0) {
    return false;
  }
  return st.st_size == 0;
}

}  // namespace

struct RtFastShmRingWriter::Mapping {
  int fd = -1;
  void *addr = MAP_FAILED;
  std::size_t size = 0;
};

struct RtFastShmRingReader::Mapping {
  int fd = -1;
  void *addr = MAP_FAILED;
  std::size_t size = 0;
};

RtFastShmRingWriter::RtFastShmRingWriter(const std::string &name) {
  mapping_ = std::make_unique<Mapping>();
  mapping_->size = sizeof(RtFastShmRingShared);
  mapping_->fd = shm_open(name.c_str(), O_RDWR | O_CREAT, 0660);
  if (mapping_->fd < 0) {
    mapping_.reset();
    return;
  }

  if (shouldCreateRing(mapping_->fd) && ftruncate(mapping_->fd, static_cast<off_t>(mapping_->size)) != 0) {
    close(mapping_->fd);
    mapping_.reset();
    return;
  }

  mapping_->addr = mmap(nullptr, mapping_->size, PROT_READ | PROT_WRITE, MAP_SHARED, mapping_->fd, 0);
  if (mapping_->addr == MAP_FAILED) {
    close(mapping_->fd);
    mapping_.reset();
    return;
  }

  ring_ = static_cast<RtFastShmRingShared *>(mapping_->addr);
  std::uint32_t expected = 0;
  if (ring_->initialized.compare_exchange_strong(expected, 1U)) {
    ring_->write_index.store(0, std::memory_order_relaxed);
    ring_->read_index.store(0, std::memory_order_relaxed);
    ring_->dropped_count.store(0, std::memory_order_relaxed);
    std::memset(ring_->slots, 0, sizeof(ring_->slots));
  }
}

RtFastShmRingWriter::~RtFastShmRingWriter() {
  if (!mapping_) {
    return;
  }
  if (mapping_->addr != MAP_FAILED) {
    munmap(mapping_->addr, mapping_->size);
  }
  if (mapping_->fd >= 0) {
    close(mapping_->fd);
  }
}

bool RtFastShmRingWriter::ready() const noexcept {
  return ring_ != nullptr && mapping_ && mapping_->addr != MAP_FAILED;
}

bool RtFastShmRingWriter::write(const RtFastCommandFrame &frame, std::uint32_t *queue_depth) noexcept {
  if (!ready()) {
    return false;
  }

  const auto pod = toPod(frame);
  // SPSC contract: reserve slot from current write index, write payload, then publish index.
  const std::uint64_t write_index = ring_->write_index.load(std::memory_order_relaxed);
  const std::uint64_t read_index = ring_->read_index.load(std::memory_order_acquire);
  const std::uint64_t backlog = write_index >= read_index ? (write_index - read_index) : 0;
  if (backlog >= kRtFastRingCapacity) {
    ring_->read_index.store(write_index - kRtFastRingCapacity + 1, std::memory_order_release);
    ring_->dropped_count.fetch_add(1, std::memory_order_relaxed);
  }
  ring_->slots[write_index % kRtFastRingCapacity] = pod;
  ring_->write_index.store(write_index + 1, std::memory_order_release);
  if (queue_depth != nullptr) {
    const std::uint64_t post_read = ring_->read_index.load(std::memory_order_acquire);
    const std::uint64_t post_write = ring_->write_index.load(std::memory_order_acquire);
    const std::uint64_t depth = post_write >= post_read ? (post_write - post_read) : 0;
    *queue_depth = static_cast<std::uint32_t>(std::min<std::uint64_t>(depth, kRtFastRingCapacity));
  }
  return true;
}

RtFastShmRingReader::RtFastShmRingReader(const std::string &name) {
  mapping_ = std::make_unique<Mapping>();
  mapping_->size = sizeof(RtFastShmRingShared);
  mapping_->fd = shm_open(name.c_str(), O_RDWR, 0660);
  if (mapping_->fd < 0) {
    mapping_.reset();
    return;
  }

  mapping_->addr = mmap(nullptr, mapping_->size, PROT_READ | PROT_WRITE, MAP_SHARED, mapping_->fd, 0);
  if (mapping_->addr == MAP_FAILED) {
    close(mapping_->fd);
    mapping_.reset();
    return;
  }
  ring_ = static_cast<RtFastShmRingShared *>(mapping_->addr);
}

RtFastShmRingReader::~RtFastShmRingReader() {
  if (!mapping_) {
    return;
  }
  if (mapping_->addr != MAP_FAILED) {
    munmap(mapping_->addr, mapping_->size);
  }
  if (mapping_->fd >= 0) {
    close(mapping_->fd);
  }
}

bool RtFastShmRingReader::ready() const noexcept {
  return ring_ != nullptr && mapping_ && mapping_->addr != MAP_FAILED;
}

bool RtFastShmRingReader::readLatest(RtFastCommandFrame &frame, std::uint32_t *queue_depth) noexcept {
  if (!ready()) {
    return false;
  }
  const std::uint64_t write_index = ring_->write_index.load(std::memory_order_acquire);
  std::uint64_t read_index = ring_->read_index.load(std::memory_order_acquire);
  if (write_index <= read_index) {
    if (queue_depth != nullptr) {
      *queue_depth = 0;
    }
    return false;
  }

  const std::uint64_t latest = write_index - 1;
  const RtFastCommandPod pod = ring_->slots[latest % kRtFastRingCapacity];
  ring_->read_index.store(write_index, std::memory_order_release);
  frame = fromPod(pod);

  if (queue_depth != nullptr) {
    *queue_depth = static_cast<std::uint32_t>(std::min<std::uint64_t>(write_index - read_index, kRtFastRingCapacity));
  }
  return true;
}

}  // namespace rokae_xmate3_ros2::runtime
