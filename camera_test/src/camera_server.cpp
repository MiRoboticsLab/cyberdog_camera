#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include "camera_service/main_camera_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto my_node = std::make_shared<cyberdog::camera::CameraServerNode>();
  exec.add_node(my_node);

  exec.spin();
  rclcpp::shutdown();
}
