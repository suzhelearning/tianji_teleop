#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "ManusDataPublisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<ManusDataPublisher>();
    const auto initialized = node->Initialize();
    if (initialized != ClientReturnCode::ClientReturnCode_Success) {
      throw std::runtime_error("MANUS SDK initialization/connection failed: " +
        std::to_string(static_cast<int>(initialized)));
    }
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("manus_data_publisher"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
