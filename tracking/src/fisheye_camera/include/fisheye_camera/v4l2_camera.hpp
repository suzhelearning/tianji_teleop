#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace fisheye_camera {

static constexpr int kNumBuffers = 2;

struct FrameData {
  const uint8_t* ptr;
  size_t length;
};

class V4L2Camera {
public:
  V4L2Camera() = default;
  ~V4L2Camera();

  V4L2Camera(const V4L2Camera&) = delete;
  V4L2Camera& operator=(const V4L2Camera&) = delete;

  bool open(const std::string& device, int width, int height, int fps);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Returns false on timeout or error. On success, out is valid until release_frame().
  bool grab_frame(FrameData& out);
  void release_frame();

  void apply_controls(const std::unordered_map<std::string, int>& settings);

private:
  struct MmapBuffer {
    void* ptr = nullptr;
    size_t length = 0;
  };

  int fd_ = -1;
  std::array<MmapBuffer, kNumBuffers> buffers_{};
  int num_buffers_ = 0;
  uint32_t last_dequeued_ = 0;
  bool streaming_ = false;
};

}  // namespace fisheye_camera
