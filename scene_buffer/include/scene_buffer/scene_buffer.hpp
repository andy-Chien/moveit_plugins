#include <rclcpp/rclcpp.hpp>

#include <example_interfaces/srv/add_two_ints.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


#include <memory>

class SceneBuffer : public rclcpp::Node
{
public:
  SceneBuffer();
private:
  void load_robot();
  void get_obstacle_cb(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> req,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> res);

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr get_obstacle_service_;
  rclcpp::SyncParametersClient::SharedPtr param_client_;
  std::shared_ptr<rclcpp::Node> param_client_node_;
};