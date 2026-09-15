#pragma once
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

namespace odin_decoder {

class JpegDecoder {
 public:
  virtual ~JpegDecoder() = default;
  virtual bool Decode(const uint8_t* data, size_t size, cv::Mat& output) = 0;
  virtual std::string GetName() const = 0;
};

std::unique_ptr<JpegDecoder> CreateDecoder();

}  // namespace odin_decoder
