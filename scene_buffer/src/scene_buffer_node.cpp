#include <rclcpp/rclcpp.hpp>
#include "scene_buffer/scene_buffer.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto scene_buffer_node = std::make_shared<
    SceneBuffer>("scene_buffer", node_options);
  scene_buffer_node->init();
  
  rclcpp::spin(scene_buffer_node);
  rclcpp::shutdown();
  return 0;
}