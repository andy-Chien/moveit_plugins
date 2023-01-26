#include <rclcpp/rclcpp.hpp>
#include "scene_buffer/scene_buffer.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneBuffer>());
  rclcpp::shutdown();
  return 0;
}