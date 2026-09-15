# 驱动集成 ESS 立体校正模块设计方案

> 目标：在现有驱动 (`/home/m/o2/src/ros_driver`) 中集成立体校正功能，读取标定结果 (`8.yaml`)，发布 Isaac ROS ESS 所需的话题

---

## 1. 现状分析

### 1.1 当前驱动架构

```
┌────────────────────────────────────────────────────────────────┐
│                    odin_ros_driver_node                        │
│  发布: /odin/image/compressed, /odin/image2/compressed, ...    │
└────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────────┐
│                    post_process_node                           │
│  - colorUndistort: 单目去畸变（无立体校正）                      │
│  - rawCloudRender: 点云着色                                    │
│  发布: /odin/image_undistort, /odin/image2_undistort           │
└────────────────────────────────────────────────────────────────┘
```

### 1.2 当前 `colorUndistort` 的问题

```cpp
// colorUndistort.cpp - 当前实现
// 问题1: 只做单目去畸变，没有立体校正
// 问题2: 没有使用 T_cl_cr（左右相机变换）计算校正旋转
// 问题3: 输出图像没有极线对齐
```

**当前 `camera_calib.yaml` 结构：**
```yaml
Tcl_0: [...]           # 相机0到LiDAR变换 (有)
cam_0: {...}           # 左相机内参 (有)
cam_1: {...}           # 右相机内参 (有)
# 缺失: T_cl_cr        # 左相机到右相机变换 (需要从8.yaml获取)
```

### 1.3 ESS 需要什么

| 话题 | 当前状态 | ESS要求 |
|------|----------|---------|
| `/left/image_rect` | ❌ 无 | 极线校正后的左图像 |
| `/right/image_rect` | ❌ 无 | 极线校正后的右图像 |
| `/left/camera_info` | ❌ 无 | P矩阵含基线信息 |

---

## 2. 设计方案

### 2.1 整体架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                        标定结果 (8.yaml)                            │
│  - T_cl_cr: 左→右相机变换 (基线信息)                                 │
│  - cam_0, cam_1: PolyFisheye 内参                                   │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼ (启动时读取)
┌─────────────────────────────────────────────────────────────────────┐
│                    改进后的 post_process_node                        │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 新增: StereoRectifier 类                                     │   │
│  │   - 读取 8.yaml 获取 T_cl_cr                                 │   │
│  │   - 计算立体校正旋转 R1, R2                                   │   │
│  │   - 生成校正映射表 map1_l, map2_l, map1_r, map2_r            │   │
│  │   - 计算新内参 K_new 和 P 矩阵                                │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│              ┌───────────────┴───────────────┐                      │
│              ▼                               ▼                      │
│  ┌─────────────────────┐        ┌─────────────────────┐            │
│  │ 左图像校正           │        │ 右图像校正           │            │
│  │ cv::remap(map_l)    │        │ cv::remap(map_r)    │            │
│  └─────────────────────┘        └─────────────────────┘            │
│              │                               │                      │
│              ▼                               ▼                      │
│     /left/image_rect              /right/image_rect                │
│     /left/camera_info             /right/camera_info               │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        Isaac ROS ESS                                │
│                      (深度估计)                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 新增模块: `StereoRectifier`

**文件位置：**
- `include/utility/stereo_rectifier.hpp`
- `src/stereo_rectifier.cpp`

**核心功能：**

```cpp
class StereoRectifier {
public:
    // 从 8.yaml 初始化
    bool init(const std::string& calib_yaml_path);
    
    // 校正图像
    void rectify(const cv::Mat& left_raw, const cv::Mat& right_raw,
                 cv::Mat& left_rect, cv::Mat& right_rect);
    
    // 获取 CameraInfo (ESS 用)
    sensor_msgs::msg::CameraInfo getLeftCameraInfo() const;
    sensor_msgs::msg::CameraInfo getRightCameraInfo() const;
    
    // 获取基线 (米)
    double getBaseline() const { return baseline_; }

private:
    // 从 T_cl_cr 计算立体校正参数
    void computeRectification();
    
    // 生成 PolyFisheye 校正映射表
    void generateRectifyMaps();
    
    // 成员变量
    Eigen::Matrix4d T_cl_cr_;      // 左→右变换
    Eigen::Matrix3d R1_, R2_;      // 校正旋转
    double baseline_;               // 基线 (米)
    
    cv::Mat map1_l_, map2_l_;      // 左相机映射表
    cv::Mat map1_r_, map2_r_;      // 右相机映射表
    
    Eigen::Matrix3d K_new_;        // 校正后内参
    int out_width_, out_height_;   // 输出尺寸
};
```

### 2.3 关键算法

#### 2.3.1 从 `T_cl_cr` 提取基线

```cpp
void StereoRectifier::computeRectification() {
    // T_cl_cr 是左相机到右相机的变换
    // 其平移分量就是基线向量
    Eigen::Vector3d T = T_cl_cr_.block<3,1>(0,3);
    Eigen::Matrix3d R = T_cl_cr_.block<3,3>(0,0);
    
    baseline_ = T.norm();  // 基线长度 (米)
    
    // 构建立体校正坐标系
    Eigen::Vector3d e1 = T / baseline_;  // x轴: 沿基线方向
    
    Eigen::Vector3d e2 = Eigen::Vector3d(0,0,1).cross(e1);
    e2.normalize();
    
    Eigen::Vector3d e3 = e1.cross(e2);
    
    // 校正旋转矩阵
    Eigen::Matrix3d R_rect;
    R_rect.row(0) = e1.transpose();
    R_rect.row(1) = e2.transpose();
    R_rect.row(2) = e3.transpose();
    
    R1_ = R_rect;           // 左相机校正旋转
    R2_ = R_rect * R.transpose();  // 右相机校正旋转
}
```

#### 2.3.2 生成 PolyFisheye 校正映射表

```cpp
void StereoRectifier::generateRectifyMaps() {
    // 输出尺寸 (ESS 推荐)
    out_width_ = 960;
    out_height_ = 576;
    double fov_deg = 90.0;
    
    // 新的针孔内参
    double f_new = 0.5 * out_width_ / tan(fov_deg * M_PI / 360.0);
    K_new_ << f_new, 0, out_width_ / 2.0,
              0, f_new, out_height_ / 2.0,
              0, 0, 1;
    
    // 为每个输出像素计算对应的原始像素位置
    // (复用 refer_code 中 generate_stereo_rectify_maps.py 的逻辑)
    for (int v = 0; v < out_height_; ++v) {
        for (int u = 0; u < out_width_; ++u) {
            // 1. 输出像素 → 校正后归一化坐标
            Eigen::Vector3d p_rect = K_new_.inverse() * Eigen::Vector3d(u, v, 1);
            
            // 2. 校正坐标系 → 原相机坐标系
            Eigen::Vector3d p_cam = R1_.transpose() * p_rect;
            
            // 3. 计算入射角
            double r_xy = sqrt(p_cam.x()*p_cam.x() + p_cam.y()*p_cam.y());
            double theta = atan2(r_xy, p_cam.z());
            double phi = atan2(p_cam.y(), p_cam.x());
            
            // 4. PolyFisheye 模型
            double r_distorted = polyfisheye_r(theta, k_coeffs_);
            
            // 5. 计算原始像素坐标
            double x_d = r_distorted * cos(phi);
            double y_d = r_distorted * sin(phi);
            double u_in = fx_ * x_d + skew_ * y_d + cx_;
            double v_in = fy_ * y_d + cy_;
            
            map1_l_.at<float>(v, u) = u_in;
            map2_l_.at<float>(v, u) = v_in;
        }
    }
    // 同理生成 map1_r_, map2_r_ (使用 R2_)
}
```

#### 2.3.3 生成 ESS CameraInfo

```cpp
sensor_msgs::msg::CameraInfo StereoRectifier::getLeftCameraInfo() const {
    sensor_msgs::msg::CameraInfo info;
    info.width = out_width_;
    info.height = out_height_;
    info.distortion_model = "plumb_bob";
    
    // D: 全零 (已校正)
    info.d = {0, 0, 0, 0, 0};
    
    // K: 校正后内参
    info.k = {K_new_(0,0), 0, K_new_(0,2),
              0, K_new_(1,1), K_new_(1,2),
              0, 0, 1};
    
    // R: 校正旋转
    info.r = {R1_(0,0), R1_(0,1), R1_(0,2),
              R1_(1,0), R1_(1,1), R1_(1,2),
              R1_(2,0), R1_(2,1), R1_(2,2)};
    
    // P: 投影矩阵 (左相机 Tx=0)
    double fx = K_new_(0,0);
    info.p = {fx, 0, K_new_(0,2), 0,        // Tx = 0
              0, fx, K_new_(1,2), 0,
              0, 0, 1, 0};
    
    return info;
}

sensor_msgs::msg::CameraInfo StereoRectifier::getRightCameraInfo() const {
    // 同上，但 P[0,3] = -fx * baseline
    auto info = getLeftCameraInfo();
    double fx = K_new_(0,0);
    info.p[3] = -fx * baseline_;  // 关键: Tx = -fx * baseline
    // R 使用 R2_
    return info;
}
```

---

## 3. 修改现有代码

### 3.1 `post_process_ros2.cpp` 改动

```cpp
// 新增头文件
#include "stereo_rectifier.hpp"

class PostProcessNode : public rclcpp::Node {
public:
    PostProcessNode() {
        // ... 现有初始化 ...
        
        // 新增: 立体校正器
        declare_parameter<std::string>("stereo_calib_yaml", "");
        declare_parameter<bool>("enable_ess_output", false);
        
        std::string stereo_yaml = get_parameter("stereo_calib_yaml").as_string();
        enable_ess_ = get_parameter("enable_ess_output").as_bool();
        
        if (enable_ess_ && !stereo_yaml.empty()) {
            if (!stereo_rectifier_.init(stereo_yaml)) {
                RCLCPP_ERROR(get_logger(), "Failed to init StereoRectifier");
            } else {
                RCLCPP_INFO(get_logger(), "StereoRectifier initialized, baseline=%.2fmm",
                           stereo_rectifier_.getBaseline() * 1000);
            }
            
            // 新增发布者
            left_rect_pub_ = create_publisher<Image>("/left/image_rect", 10);
            right_rect_pub_ = create_publisher<Image>("/right/image_rect", 10);
            left_info_pub_ = create_publisher<CameraInfo>("/left/camera_info", 10);
            right_info_pub_ = create_publisher<CameraInfo>("/right/camera_info", 10);
        }
    }

private:
    // 在图像回调中添加立体校正
    void processStereoPair(const cv::Mat& left, const cv::Mat& right,
                           const std_msgs::msg::Header& header) {
        if (!enable_ess_) return;
        
        cv::Mat left_rect, right_rect;
        stereo_rectifier_.rectify(left, right, left_rect, right_rect);
        
        // 发布校正后图像
        auto left_msg = cv_bridge::CvImage(header, "rgb8", left_rect).toImageMsg();
        auto right_msg = cv_bridge::CvImage(header, "rgb8", right_rect).toImageMsg();
        left_rect_pub_->publish(*left_msg);
        right_rect_pub_->publish(*right_msg);
        
        // 发布 CameraInfo
        auto left_info = stereo_rectifier_.getLeftCameraInfo();
        auto right_info = stereo_rectifier_.getRightCameraInfo();
        left_info.header = header;
        right_info.header = header;
        left_info_pub_->publish(left_info);
        right_info_pub_->publish(right_info);
    }
    
    StereoRectifier stereo_rectifier_;
    bool enable_ess_ = false;
    
    // 新增发布者
    rclcpp::Publisher<Image>::SharedPtr left_rect_pub_;
    rclcpp::Publisher<Image>::SharedPtr right_rect_pub_;
    rclcpp::Publisher<CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<CameraInfo>::SharedPtr right_info_pub_;
};
```

### 3.2 `camera_calib.yaml` 更新

将 `8.yaml` 中的 `T_cl_cr` 添加到配置文件，或直接读取 `8.yaml`：

```yaml
# camera_calib.yaml 新增字段
T_cl_cr: [
  0.999965000, 0.004001000, 0.007315000, 0.051228000,
  -0.003980000, 0.999988000, -0.002838000, -0.000413000,
  -0.007326000, 0.002809000, 0.999969000, -0.001565000,
  0.000000000, 0.000000000, 0.000000000, 1.000000000
]
```

### 3.3 Launch 文件更新

```python
# driver.launch.py 新增参数
Node(
    package='odin_ros_driver_rev1',
    executable='post_process_node',
    parameters=[{
        'cam_calib_yaml': cam_calib_yaml,
        'stereo_calib_yaml': '/path/to/8.yaml',  # 新增
        'enable_ess_output': True,                # 新增
    }],
),
```

---

## 4. 文件结构

```
ros_driver/
├── include/utility/
│   ├── colorUndistort.hpp        # 保持不变
│   ├── stereo_rectifier.hpp      # 新增
│   └── ...
├── src/
│   ├── colorUndistort.cpp        # 保持不变
│   ├── stereo_rectifier.cpp      # 新增
│   ├── post_process_ros2.cpp     # 修改
│   └── ...
├── config/
│   ├── camera_calib.yaml         # 可添加 T_cl_cr
│   └── control_command.yaml      # 可添加 enable_ess_output
└── docs/
    └── ESS_STEREO_RECTIFY_INTEGRATION.md  # 本文档
```

---

## 5. 数据流对比

### 5.1 改动前

```
/odin/image/compressed  → decode → undistort(单目) → /odin/image_undistort
/odin/image2/compressed → decode → undistort(单目) → /odin/image2_undistort
```

### 5.2 改动后

```
/odin/image/compressed  ─┐
                         ├─→ decode → stereo_rectify → /left/image_rect
/odin/image2/compressed ─┘                           → /right/image_rect
                                                     → /left/camera_info
                                                     → /right/camera_info
                                                              │
                                                              ▼
                                                         Isaac ROS ESS
                                                              │
                                                              ▼
                                                      /disparity, /depth
```

---

## 6. 验证清单

- [ ] `T_cl_cr` 正确解析，基线 ≈ 51mm
- [ ] 校正后图像极线水平对齐
- [ ] `/left/camera_info` 的 P[0,3] = 0
- [ ] `/right/camera_info` 的 P[0,3] = -fx * baseline (负值)
- [ ] ESS 能正常输出深度图
- [ ] 深度值在合理范围内

---

## 7. 工作量估计

| 任务 | 文件 | 工作量 |
|------|------|--------|
| 新增 `StereoRectifier` 类 | stereo_rectifier.hpp/cpp | ~300行 |
| 修改 `post_process_ros2.cpp` | post_process_ros2.cpp | ~50行 |
| 更新 launch/config | driver.launch.py, yaml | ~10行 |
| 测试与调试 | - | 1-2小时 |

**总计**: 新增约 350 行代码，修改约 60 行

---

文档日期：2026-02-02
