// @file pointcloud2depth_node.hpp
// @brief Library-free point cloud -> depth map projection core.
//
// Implements the projection pipeline described in
//   sdk/docs (PointCloud -> Depth, plain-types SDK design):
//     (1) homogeneous lidar-to-camera transform
//     (2) perspective projection (u = fx*X/Z + cx, v = fy*Y/Z + cy)
//     (3) z-buffered write + optional 3x3 dilation
//
// This file deliberately avoids PCL/Eigen/OpenCV so it can be lifted into
// the device-side SDK as a pure C/C++ module without modification.
//
// Coordinate convention:
//   - Input xyz_in is in the lidar (or "cloud") frame, units in meters.
//   - Tcl is the 4x4 row-major transform from lidar frame to camera frame.
//   - Camera convention: +Z forward (depth), pinhole intrinsics A11/A22/u0/v0.
//   - Skew term A12 is intentionally dropped on projection to match the
//     existing PCL/Eigen reprojector behavior ("project drops skew,
//     un-project keeps skew").

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace odin {
namespace depth_core {

// Tunable parameters describing the projection.
struct DepthParams {
  // Camera intrinsics at full image resolution (pixel units).
  float A11;  // fx
  float A12;  // skew (ignored on projection, kept for symmetry with calibration file)
  float A22;  // fy
  float u0;   // cx
  float v0;   // cy

  // Lidar -> camera extrinsic, 4x4 row-major. Bottom row [0 0 0 1].
  float Tcl[16];

  // Output depth map size and downscale factor relative to full image.
  //   out_width  ~= image_width  / scale
  //   out_height ~= image_height / scale
  int   out_width;
  int   out_height;
  float scale;

  // Depth filtering (meters in camera frame).
  float z_min;
  float z_max;

  // 3x3 dilation radius. 0 disables; 1 fills the 8 neighbors.
  int   dilate_radius;

  // If true, use z-buffer semantics on conflicts (closer point wins) for
  // both the central pixel and dilated neighbors. If false (default per the
  // depth-pipeline-handoff spec), later writes overwrite the central pixel
  // unconditionally, and dilation only fills neighbors that are still 0.
  bool  use_z_buffer;
};

// Apply a 3x4 Kcl (stored in a 16-element row-major buffer; the last row is
// unused) to a homogeneous point (x, y, z, 1).
//   xp = M00*x + M01*y + M02*z + M03
//   yp = M10*x + M11*y + M12*z + M13
//   zp = M20*x + M21*y + M22*z + M23
static inline void mat4_apply_kcl(const float M[16], float x, float y, float z,
                                  float* xo, float* yo, float* zo) {
  *xo = M[0] * x + M[1] * y + M[2]  * z + M[3];
  *yo = M[4] * x + M[5] * y + M[6]  * z + M[7];
  *zo = M[8] * x + M[9] * y + M[10] * z + M[11];
}

// Compute Kcl = (K / scale) * Tcl, where K is the pinhole intrinsics matrix
//   [ A11   0   u0 ]
//   [  0   A22  v0 ]
//   [  0    0    1 ]
// A12 (skew) is intentionally dropped to mirror the existing reprojector.
//
// Result layout (row-major, only the top three rows carry data):
//   Kcl[0..3]   = fx*Tcl[0..3] + cx*Tcl[8..11]
//   Kcl[4..7]   = fy*Tcl[4..7] + cy*Tcl[8..11]
//   Kcl[8..11]  = Tcl[8..11]
//   Kcl[12..15] = (0, 0, 0, 1) placeholder
static inline void compute_kcl_with_scale(const DepthParams& p, float scale,
                                          float Kcl_out[16]) {
  const float fx = p.A11 / scale;
  const float fy = p.A22 / scale;
  const float cx = p.u0  / scale;
  const float cy = p.v0  / scale;

  for (int j = 0; j < 4; ++j) {
    const float t0 = p.Tcl[0 * 4 + j];
    const float t1 = p.Tcl[1 * 4 + j];
    const float t2 = p.Tcl[2 * 4 + j];
    Kcl_out[0 * 4 + j] = fx * t0 + cx * t2;
    Kcl_out[1 * 4 + j] = fy * t1 + cy * t2;
    Kcl_out[2 * 4 + j] = t2;
  }
  Kcl_out[12] = 0.0f;
  Kcl_out[13] = 0.0f;
  Kcl_out[14] = 0.0f;
  Kcl_out[15] = 1.0f;
}

// Convenience overload that reads scale from params.
static inline void compute_kcl(const DepthParams& p, float Kcl_out[16]) {
  compute_kcl_with_scale(p, p.scale, Kcl_out);
}

// Project a single 3D point in the lidar frame to image coordinates.
// Returns true if the point is within depth bounds and pixel bounds.
// Writes the resulting integer pixel and depth (meters) on success.
static inline bool project_point(const float Kcl[16],
                                 float x, float y, float z,
                                 int width, int height,
                                 float z_min, float z_max,
                                 int* u_out, int* v_out, float* depth_out) {
  float xp, yp, zp;
  mat4_apply_kcl(Kcl, x, y, z, &xp, &yp, &zp);
  if (zp < z_min || zp > z_max) return false;

  const int u = static_cast<int>(xp / zp + 0.5f);
  const int v = static_cast<int>(yp / zp + 0.5f);
  if (static_cast<unsigned>(u) >= static_cast<unsigned>(width))  return false;
  if (static_cast<unsigned>(v) >= static_cast<unsigned>(height)) return false;

  *u_out     = u;
  *v_out     = v;
  *depth_out = zp;
  return true;
}

// Hot path: project a packed xyz point cloud into a depth map.
// depth_out must be pre-allocated to out_w * out_h floats and zero-initialized
// by the caller. Returns the number of valid points written.
//
// Behaviour matches the depth-pipeline-handoff spec by default
// (use_z_buffer = false):
//   - central pixel: always overwritten by the latest point hitting it
//   - dilation:      only fills neighbors that are still 0 (no z compare)
//   - depth filter:  only zp > 0 (no z_min/z_max range)
//
// If use_z_buffer = true, the closer point wins on every conflict (both
// central and dilated), and zp is also bounded by [z_min, z_max].
static inline int pointcloud_to_depth(const float* xyz_in,
                                      uint32_t      n_points,
                                      const float   Kcl[16],
                                      int           out_w,
                                      int           out_h,
                                      int           dilate_radius,
                                      float         z_min,
                                      float         z_max,
                                      bool          use_z_buffer,
                                      float*        depth_out) {
  if (xyz_in == nullptr || depth_out == nullptr || out_w <= 0 || out_h <= 0) {
    return 0;
  }

  int written = 0;
  for (uint32_t i = 0; i < n_points; ++i) {
    const float x = xyz_in[3 * i + 0];
    const float y = xyz_in[3 * i + 1];
    const float z = xyz_in[3 * i + 2];

    // Skip non-finite and exact-origin points (common LiDAR invalid markers).
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    if (x == 0.0f && y == 0.0f && z == 0.0f) continue;

    float xp, yp, zp;
    mat4_apply_kcl(Kcl, x, y, z, &xp, &yp, &zp);

    if (use_z_buffer) {
      if (zp < z_min || zp > z_max) continue;
    } else {
      // Spec: only require zp > 0 (in front of camera).
      if (zp <= 0.0f) continue;
    }

    const int u = static_cast<int>(xp / zp + 0.5f);
    const int v = static_cast<int>(yp / zp + 0.5f);
    if (static_cast<unsigned>(u) >= static_cast<unsigned>(out_w))  continue;
    if (static_cast<unsigned>(v) >= static_cast<unsigned>(out_h)) continue;

    const int idx = v * out_w + u;
    if (use_z_buffer) {
      if (depth_out[idx] == 0.0f || zp < depth_out[idx]) {
        depth_out[idx] = zp;
      }
    } else {
      // Spec: later writes overwrite earlier (no z compare on central).
      depth_out[idx] = zp;
    }

    if (dilate_radius > 0) {
      for (int dv = -dilate_radius; dv <= dilate_radius; ++dv) {
        const int nv = v + dv;
        if (static_cast<unsigned>(nv) >= static_cast<unsigned>(out_h)) continue;
        for (int du = -dilate_radius; du <= dilate_radius; ++du) {
          if (du == 0 && dv == 0) continue;
          const int nu = u + du;
          if (static_cast<unsigned>(nu) >= static_cast<unsigned>(out_w)) continue;
          const int nidx = nv * out_w + nu;
          if (use_z_buffer) {
            if (depth_out[nidx] == 0.0f || zp < depth_out[nidx]) {
              depth_out[nidx] = zp;
            }
          } else {
            // Spec: dilation only fills empty pixels.
            if (depth_out[nidx] == 0.0f) depth_out[nidx] = zp;
          }
        }
      }
    }

    ++written;
  }
  return written;
}

}  // namespace depth_core
}  // namespace odin
