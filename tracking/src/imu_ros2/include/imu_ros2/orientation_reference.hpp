#ifndef IMU_ROS2__ORIENTATION_REFERENCE_HPP_
#define IMU_ROS2__ORIENTATION_REFERENCE_HPP_

namespace imu_ros2
{

struct Quaternion
{
  double x{};
  double y{};
  double z{};
  double w{1.0};
};

bool is_valid_quaternion(const Quaternion & quaternion);

Quaternion normalize(const Quaternion & quaternion);

void align_hemisphere_in_place(Quaternion & current, const Quaternion & reference);

class OrientationReference
{
public:
  OrientationReference() = default;
  OrientationReference(bool force_positive_w, bool use_quaternion_continuity);

  void configure(bool force_positive_w, bool use_quaternion_continuity);

  bool apply(const Quaternion & raw_orientation, Quaternion & output_orientation);

private:
  bool force_positive_w_{false};
  bool use_quaternion_continuity_{true};
  bool has_last_orientation_{false};
  Quaternion last_orientation_{};
};

}  // namespace imu_ros2

#endif  // IMU_ROS2__ORIENTATION_REFERENCE_HPP_
