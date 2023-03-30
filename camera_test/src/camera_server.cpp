#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include "camera_service/main_camera_node.hpp"

#define ENABLE_CAM_INFINITE 1

int main(int argc, char** argv)
{
#if ENABLE_CAM_INFINITE
  setenv("enableCamInfiniteTimeout", "1", 1);
#endif
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto my_node = std::make_shared<cyberdog::camera::CameraServerNode>();
  exec.add_node(my_node);

  exec.spin();
  rclcpp::shutdown();
}
