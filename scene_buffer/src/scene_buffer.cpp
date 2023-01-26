#include "scene_buffer/scene_buffer.hpp"

SceneBuffer::SceneBuffer()
: Node("scene_buffer_node")
{
  get_obstacle_service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
    "get_trajectory_obstacle", std::bind(
    &SceneBuffer::get_obstacle_cb, this, std::placeholders::_1, std::placeholders::_2));
  
  param_client_node_ = rclcpp::Node::make_shared("param_client_node");
  param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
    param_client_node_, "node_name");
}

void SceneBuffer::load_robot()
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("model_loader_node", node_options);
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  return;
}

void SceneBuffer::get_obstacle_cb(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> req,
  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> res)
{
  return;
}