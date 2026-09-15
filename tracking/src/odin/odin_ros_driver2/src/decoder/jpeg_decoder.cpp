#include "jpeg_decoder.h"
#include <cstdlib>
#include <vector>
#include "utility/ros_compat.hpp"

#ifdef USE_JETSON_DECODER
#include <NvJpegDecoder.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>
#include <unistd.h>
#endif

namespace odin_decoder {

#ifdef USE_JETSON_DECODER
// Note: Hardware decoder is only enabled at compile-time for JetPack R36+
// R36+ uses libjpeg-8b which is ABI compatible with NvJpegDecoder
// R35 and below are blocked at CMake level due to libjpeg-turbo ABI conflict

class JetsonHardwareDecoder : public JpegDecoder {
 public:
  JetsonHardwareDecoder() {
    decoder_ = NvJPEGDecoder::createJPEGDecoder("jpegdec");
    if (!decoder_) {
      RC_LOG_ERROR("[JetsonDecoder] Failed to create NvJPEGDecoder");
      initialized_ = false;
      return;
    }
    initialized_ = true;
    RC_LOG_INFO("[JetsonDecoder] Hardware decoder initialized (JetPack R36+ verified at compile time)");
  }

  ~JetsonHardwareDecoder() override {
    if (decoder_) {
      delete decoder_;
    }
  }

  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override {
    if (!initialized_ || !decoder_ || !data || size < 100) return false;

    // Step 1: Allocate aligned memory (64-byte aligned, GPU friendly)
    void* aligned_data = nullptr;
    if (posix_memalign(&aligned_data, 64, size) != 0) {
        //RCLCPP_ERROR(rclcpp::get_logger("decoder"), "Failed to allocate aligned memory");
        return false;
    }
    std::unique_ptr<void, decltype(&free)> aligned_guard(aligned_data, &free);

    // Step 2: Copy data
    memcpy(aligned_data, data, size);

    // Step 3: Call decodeToFd (data is now aligned)
    int fd = -1;
    uint32_t width = 0, height = 0;
    uint32_t pixfmt = 0;

    int decode_ret = decoder_->decodeToFd(fd, static_cast<unsigned char*>(aligned_guard.get()), 
                                          static_cast<unsigned long>(size), pixfmt, width, height);

    if (decode_ret < 0 || fd < 0 || width == 0 || height == 0) {
      if (fd >= 0) close(fd);
      RC_LOG_ERROR("[JetsonDecoder] decodeToFd failed (ret=%d, fd=%d, w=%u, h=%u, pixfmt=%u)",
              decode_ret, fd, width, height, pixfmt);
      return false;
    }

    NvBufSurface* surf = nullptr;
    if (NvBufSurfaceFromFd(fd, reinterpret_cast<void**>(&surf)) != 0 || !surf) {
      close(fd);
      RC_LOG_ERROR("[JetsonDecoder] NvBufSurfaceFromFd failed");
      return false;
    }

    if (surf->numFilled == 0 || !surf->surfaceList) {
      close(fd);
      RC_LOG_ERROR("[JetsonDecoder] Invalid surface (numFilled=%d, surfaceList=%p)",
              surf->numFilled, (void*)surf->surfaceList);
      return false;
    }

    NvBufSurfaceParams& params = surf->surfaceList[0];
    
    // Validate surface parameters
    if (params.width == 0 || params.height == 0 || params.planeParams.num_planes == 0) {
      close(fd);
      RC_LOG_ERROR("[JetsonDecoder] Invalid params (w=%u, h=%u, planes=%u)",
              params.width, params.height, params.planeParams.num_planes);
      return false;
    }

    // YUV420/NV12 require even width/height
    if ((params.planeParams.num_planes >= 2) &&
        ((params.width & 1u) || (params.height & 1u))) {
      close(fd);
      RC_LOG_WARN("[JetsonDecoder] Reject odd dimension frame (w=%u, h=%u)",
              params.width, params.height);
      return false;
    }

    // Basic pitch check
    for (uint32_t i = 0; i < params.planeParams.num_planes; ++i) {
      if (params.planeParams.pitch[i] == 0) {
        close(fd);
        RC_LOG_ERROR("[JetsonDecoder] Invalid pitch at plane %u", i);
        return false;
      }
    }
    
    // Map all planes for YUV420 planar format
    uint32_t mapped_planes = 0;
    for (uint32_t i = 0; i < params.planeParams.num_planes; i++) {
      if (NvBufSurfaceMap(surf, 0, i, NVBUF_MAP_READ) != 0) {
        for (uint32_t j = 0; j < mapped_planes; j++) {
          NvBufSurfaceUnMap(surf, 0, j);
        }
        close(fd);
        return false;
      }
      mapped_planes++;
      // Sync each plane individually to ensure cache coherency
      NvBufSurfaceSyncForCpu(surf, 0, i);
    }

    // Validate mapped addresses
    for (uint32_t i = 0; i < params.planeParams.num_planes; i++) {
      if (!params.mappedAddr.addr[i]) {
        for (uint32_t j = 0; j < mapped_planes; j++) {
          NvBufSurfaceUnMap(surf, 0, j);
        }
        close(fd);
        return false;
      }
    }

    int h = static_cast<int>(params.height);
    int w = static_cast<int>(params.width);
    
    bool success = false;
    
    // Handle I420/YUV420 planar format (Y, U, V separate planes)
    // colorFormat=2 = NVBUF_COLOR_FORMAT_YUV420, pixfmt=842091865 = V4L2_PIX_FMT_YUV420
    if (params.planeParams.num_planes >= 3) {
      // Use cv::Mat to handle pitched planes directly
      cv::Mat y_plane(h, w, CV_8UC1, params.mappedAddr.addr[0], params.planeParams.pitch[0]);
      cv::Mat u_plane(h/2, w/2, CV_8UC1, params.mappedAddr.addr[1], params.planeParams.pitch[1]);
      cv::Mat v_plane(h/2, w/2, CV_8UC1, params.mappedAddr.addr[2], params.planeParams.pitch[2]);
      
      // I420 memory layout: Y (w*h) contiguous + U (w/2 * h/2) contiguous + V (w/2 * h/2) contiguous
      // Total size = w * h * 3 / 2, stored row by row
      std::vector<uint8_t> yuv_data(w * h * 3 / 2);
      
      // Copy Y plane (contiguous)
      cv::Mat y_dst(h, w, CV_8UC1, yuv_data.data());
      y_plane.copyTo(y_dst);
      
      // Copy U plane (contiguous, right after Y)
      cv::Mat u_dst(h/2, w/2, CV_8UC1, yuv_data.data() + w * h);
      u_plane.copyTo(u_dst);
      
      // Copy V plane (contiguous, right after U)
      cv::Mat v_dst(h/2, w/2, CV_8UC1, yuv_data.data() + w * h + w * h / 4);
      v_plane.copyTo(v_dst);
      
      cv::Mat yuv_i420(h * 3 / 2, w, CV_8UC1, yuv_data.data());
      cv::cvtColor(yuv_i420, output, cv::COLOR_YUV2BGR_I420);
      success = !output.empty();
    }
    // Handle NV21 format (Y plane + interleaved VU plane - Jetson outputs NV21)
    else if (params.planeParams.num_planes == 2) {
      cv::Mat nv21(h * 3 / 2, w, CV_8UC1);
      cv::Mat y_plane(h, w, CV_8UC1, params.mappedAddr.addr[0], params.planeParams.pitch[0]);
      cv::Mat vu_plane(h/2, w, CV_8UC1, params.mappedAddr.addr[1], params.planeParams.pitch[1]);
      y_plane.copyTo(nv21(cv::Rect(0, 0, w, h)));
      vu_plane.copyTo(nv21(cv::Rect(0, h, w, h/2)));
      cv::cvtColor(nv21, output, cv::COLOR_YUV2BGR_NV21);
      success = !output.empty();
    }
    else {
      cv::Mat raw(h, w, CV_8UC3, params.mappedAddr.addr[0], params.planeParams.pitch[0]);
      raw.copyTo(output);
      success = !output.empty();
    }

    for (uint32_t i = 0; i < mapped_planes; i++) {
      NvBufSurfaceUnMap(surf, 0, i);
    }
    // Note: Do NOT call NvBufSurfaceDestroy(surf)!
    // surf is mapped from fd, close(fd) will automatically release the underlying buffer
    // Calling Destroy corrupts NvJPEGDecoder internal state, causing subsequent decode failures
    close(fd);
    
    return success;
  }

  std::string GetName() const override { 
    return initialized_ ? "NvJpegDecoder Hardware" : "NvJpegDecoder (init failed)"; 
  }

  bool IsInitialized() const { return initialized_; }

 private:
  NvJPEGDecoder* decoder_ = nullptr;
  bool initialized_ = false;
};
#endif

class SoftwareDecoder : public JpegDecoder {
 public:
  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override {
    cv::Mat raw(1, static_cast<int>(size), CV_8UC1, const_cast<uint8_t*>(data));
    output = cv::imdecode(raw, cv::IMREAD_COLOR);
    return !output.empty();
  }

  std::string GetName() const override { return "OpenCV Software Decoder"; }
};

#ifdef USE_JETSON_DECODER
class HybridDecoder : public JpegDecoder {
 public:
  HybridDecoder() {
    hw_decoder_ = std::make_unique<JetsonHardwareDecoder>();
    sw_decoder_ = std::make_unique<SoftwareDecoder>();
    
    // JetPack R36+ verified at compile time - hardware decoder should work
    if (hw_decoder_->IsInitialized()) {
      use_hardware_ = true;
      RC_LOG_INFO("[HybridDecoder] Mode: Hardware decoder (JetPack R36+)");
    } else {
      use_hardware_ = false;
      RC_LOG_WARN("[HybridDecoder] Mode: Software decoder (HW init failed)");
    }
  }

  bool Decode(const uint8_t* data, size_t size, cv::Mat& output) override {
    // Validate JPEG structure - MUST have SOI (0xFFD8) at start and EOI (0xFFD9) at end
    if (!data || size < 4) {
      return false;  // Invalid data, skip frame
    }
    
    bool has_soi = (data[0] == 0xFF && data[1] == 0xD8);
    bool has_eoi = (data[size-2] == 0xFF && data[size-1] == 0xD9);
    
    if (!has_soi || !has_eoi) {
      // Incomplete/corrupt JPEG - skip this frame entirely to avoid crash
      invalid_jpeg_count_++;
      if (invalid_jpeg_count_ <= 10 || invalid_jpeg_count_ % 100 == 0) {
        RC_LOG_WARN("[HybridDecoder] Incomplete JPEG (no SOI/EOI), frame dropped (count: %d)", invalid_jpeg_count_);
      }
      return false;
    }
    
    // Use hardware decoder if available and compatible
    if (use_hardware_ && hw_decoder_) {
      if (hw_decoder_->Decode(data, size, output)) {
        return true;
      }
      // Hardware available but failed on this frame: drop it
      hw_fail_count_++;
      if (hw_fail_count_ <= 10 || hw_fail_count_ % 100 == 0) {
        RC_LOG_WARN("[HybridDecoder] HW decode failed on valid JPEG (fail count: %d), frame dropped",
                hw_fail_count_);
      }
      return false;
    }

    // Software decoder path (for libjpeg-turbo systems or HW unavailable)
    if (sw_decoder_ && sw_decoder_->Decode(data, size, output)) {
      return true;
    }

    return false;
  }

  std::string GetName() const override { 
    if (use_hardware_) {
      return "Hardware (Jetson NvJPEG + libjpeg-8b)";
    }
    return "Software (OpenCV + libjpeg-turbo)";
  }

 private:
  std::unique_ptr<JetsonHardwareDecoder> hw_decoder_;
  std::unique_ptr<SoftwareDecoder> sw_decoder_;
  bool use_hardware_ = false;
  int hw_fail_count_ = 0;
  int invalid_jpeg_count_ = 0;
};
#endif

std::unique_ptr<JpegDecoder> CreateDecoder() {
#ifdef USE_JETSON_DECODER
  auto decoder = std::make_unique<HybridDecoder>();
  RC_LOG_INFO("[JPEG Decoder] Using Hybrid Decoder (Hardware + Software fallback)");
  return decoder;
#else
  RC_LOG_INFO("[JPEG Decoder] Using OpenCV Software Decoder");
  return std::make_unique<SoftwareDecoder>();
#endif
}

}  // namespace odin_decoder