#include "fisheye_camera/v4l2_camera.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace fisheye_camera {

static int xioctl(int fd, unsigned long request, void* arg) {
  int r;
  do {
    r = ioctl(fd, request, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

V4L2Camera::~V4L2Camera() { close(); }

bool V4L2Camera::open(const std::string& device, int width, int height, int fps) {
  close();

  fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // Set format: MJPEG
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = static_cast<uint32_t>(width);
  fmt.fmt.pix.height = static_cast<uint32_t>(height);
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    close();
    return false;
  }

  // Set frame rate
  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps);
  xioctl(fd_, VIDIOC_S_PARM, &parm);

  // Request buffers
  v4l2_requestbuffers req{};
  req.count = kNumBuffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count < 1) {
    close();
    return false;
  }
  num_buffers_ = static_cast<int>(req.count);

  // Query and mmap each buffer, then queue it
  for (int i = 0; i < num_buffers_; ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = static_cast<uint32_t>(i);
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      close();
      return false;
    }

    buffers_[i].length = buf.length;
    buffers_[i].ptr = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[i].ptr == MAP_FAILED) {
      buffers_[i].ptr = nullptr;
      close();
      return false;
    }

    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      close();
      return false;
    }
  }

  // Start streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    close();
    return false;
  }
  streaming_ = true;

  return true;
}

void V4L2Camera::close() {
  if (streaming_ && fd_ >= 0) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
  }
  for (int i = 0; i < num_buffers_; ++i) {
    if (buffers_[i].ptr && buffers_[i].ptr != MAP_FAILED) {
      munmap(buffers_[i].ptr, buffers_[i].length);
      buffers_[i].ptr = nullptr;
      buffers_[i].length = 0;
    }
  }
  num_buffers_ = 0;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool V4L2Camera::grab_frame(FrameData& out) {
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  int ret = poll(&pfd, 1, 500);
  if (ret <= 0) {
    return false;
  }

  v4l2_buffer buf{};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
    return false;
  }

  last_dequeued_ = buf.index;
  out.ptr = static_cast<const uint8_t*>(buffers_[buf.index].ptr);
  out.length = buf.bytesused;
  return true;
}

void V4L2Camera::release_frame() {
  v4l2_buffer buf{};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = last_dequeued_;
  xioctl(fd_, VIDIOC_QBUF, &buf);
}

void V4L2Camera::apply_controls(const std::unordered_map<std::string, int>& settings) {
  if (fd_ < 0) return;

  auto set_ctrl = [this](uint32_t id, int value) {
    v4l2_control ctrl{};
    ctrl.id = id;
    ctrl.value = value;
    xioctl(fd_, VIDIOC_S_CTRL, &ctrl);
  };

  auto it = settings.find("auto_exposure");
  if (it != settings.end()) {
    // V4L2: 3 = auto, 1 = manual
    set_ctrl(V4L2_CID_EXPOSURE_AUTO, it->second ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL);
  }

  it = settings.find("exposure");
  if (it != settings.end()) {
    set_ctrl(V4L2_CID_EXPOSURE_ABSOLUTE, it->second);
  }

  it = settings.find("gain");
  if (it != settings.end()) {
    set_ctrl(V4L2_CID_GAIN, it->second);
  }

  it = settings.find("auto_wb");
  if (it != settings.end()) {
    set_ctrl(V4L2_CID_AUTO_WHITE_BALANCE, it->second);
  }

  it = settings.find("wb_temperature");
  if (it != settings.end()) {
    set_ctrl(V4L2_CID_WHITE_BALANCE_TEMPERATURE, it->second);
  }
}

}  // namespace fisheye_camera
