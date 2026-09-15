// @file traits.hpp
// @brief Primary template for camera projection model traits.
//        Each projection model (FisheyeProjection, PinholeProjection, ...)
//        specializes CameraModelTraits<Projection> to expose its parameter
//        layout and metadata.
//
// 摄像机投影模型 traits 主模板。各投影模型（FisheyeProjection 等）通过
// 特化 CameraModelTraits<Projection> 暴露其内/外参数布局与元信息。
#pragma once

namespace camera {

// Primary template. Intentionally empty: a specialization MUST be provided
// for every projection model that needs trait introspection. Using an
// unspecialized CameraModelTraits<T> is a hard compile error by design.
template <typename Projection>
struct CameraModelTraits;

}  // namespace camera
