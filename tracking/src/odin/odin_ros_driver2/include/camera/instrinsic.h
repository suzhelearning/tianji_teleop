#pragma once
namespace MTSDK {
namespace MTCAMERA {
typedef struct {
  int width = 0;
  int height = 0;
  float fx = 0.0f;
  float fy = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;
  float skew = 0.0f;
  float k[9] = {0.0f};
  float p[3] = {0.0f};
} intrinsic;
}  // namespace MTCAMERA
}  // namespace MTSDK