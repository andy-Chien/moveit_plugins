#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "scene_buffer_parameters.hpp"
#include "mr_msgs/srv/compute_trajectory_length.hpp"


class ComputeTrajLength : public rclcpp::Node
{
public:
    ComputeTrajLength(const std::string& node_name, const rclcpp::NodeOptions& node_options)
    : Node(node_name, node_options)
    {

    }
    void init()
    {
        param_listener_ = std::make_shared<ParamListener>(shared_from_this());
        params_ = param_listener_->get_params();

        for(const auto& robot_name : params_.robot_names)
        {
            auto robot = std::make_shared<Robot>(shared_from_this(), robot_name);
            robots_.insert(std::pair<std::string, std::shared_ptr<Robot>>(
            robot_name, robot)
            );
        }
    }

    class Robot
    {
    public:
        using ComputeSrv = mr_msgs::srv::ComputeTrajectoryLength;
        Robot(std::shared_ptr<rclcpp::Node> node, std::string robot_name)
        : robot_name_(robot_name), node_(node)
        {
            load_robot(robot_name);
            cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            compute_trajectory_length_service_ = node_->create_service<ComputeSrv>(
                robot_name + "/compute_trajectory_length", std::bind(
                    &Robot::compute_trajectory_length_cb, 
                    this, std::placeholders::_1, std::placeholders::_2
                ),
                rmw_qos_profile_services_default, cb_group_
            );
        }

    private:
        void load_robot(const std::string& robot_name)
        {
            param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
                node_, robot_name + "/move_group");
            while(!param_client_->wait_for_service(std::chrono::milliseconds(500))){
                RCLCPP_INFO(node_->get_logger(), "Waiting for get param service.");
            }

            const std::vector<std::string> params_name(
                {"robot_description", "robot_description_semantic"});

            param_client_->get_parameters(params_name,
                [this](std::shared_future<std::vector<rclcpp::Parameter>> future) -> void {
                    const auto& params = future.get();
                    this->load_robot(params.at(0).as_string(), params.at(1).as_string());
                }
            );
        }
        void load_robot(const std::string& urdf, const std::string& srdf)
        {
            robot_model_loader::RobotModelLoader rml(
                node_, robot_model_loader::RobotModelLoader::Options(urdf, srdf));
            model_ = rml.getModel();
            state_ = std::make_shared<moveit::core::RobotState>(model_);
            state_->setToDefaultValues();
        }
        void compute_trajectory_length_cb(const std::shared_ptr<ComputeSrv::Request> req,
            std::shared_ptr<ComputeSrv::Response> res)
        {
            const size_t points_cnt = req->trajectory.points.size();
            const size_t joints_cnt = req->trajectory.joint_names.size();
            std::vector<Eigen::Vector3d> pos_list;
            std::vector<std::array<const double, 4>> quat_list;
            pos_list.reserve(points_cnt);
            quat_list.reserve(points_cnt);

            if(state_->getRobotModel()->hasLinkModel(req->target_link))
            {
                for(const auto& point : req->trajectory.points){
                    for(size_t i=0; i<joints_cnt; i++){
                        state_->setJointPositions(req->trajectory.joint_names[i], &point.positions[i]);
                    }
                    state_->update();
                    const auto& t = state_->getGlobalLinkTransform(req->target_link);
                    pos_list.emplace_back(t.translation());
                    const auto& q = Eigen::Quaterniond(t.linear());
                    quat_list.emplace_back(std::array<const double, 4>({q.w(), q.x(), q.y(), q.z()}));
                }
            }else{
                RCLCPP_ERROR(node_->get_logger(), "%s link is not included.", req->target_link.c_str());
            }
            const bool update_tool_traj_length = pos_list.size() == points_cnt;
            const bool update_quat_traj_length = quat_list.size() == points_cnt;
            res->joint_traj_length = 0;
            res->tool_traj_length = 0;
            res->quat_traj_length = 0;
            for(size_t i=0; i<points_cnt - 1; i++){
                const auto& p1 = req->trajectory.points[i];
                const auto& p2 = req->trajectory.points[i + 1];
                for(size_t j=0; j<joints_cnt; j++){
                    res->joint_traj_length += fabs(p2.positions[j] - p1.positions[j]);
                }
                if(update_tool_traj_length){
                    const auto& v = pos_list[i + 1] - pos_list[i];
                    res->tool_traj_length += std::hypot(v(0), v(1), v(2));
                }
                if(update_quat_traj_length){
                    const auto& a = quat_list[i + 1];
                    const auto& b = quat_list[i];
                    const std::array<double, 4> d = {a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]};
                    const double qd = sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2] + d[3]*d[3]);
                    res->quat_traj_length += (qd > 1) ? 2 - qd : qd;
                }
            }
        }
        moveit::core::RobotModelPtr model_;
        moveit::core::RobotStatePtr state_;

        std::string robot_name_;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::CallbackGroup::SharedPtr cb_group_;
        rclcpp::AsyncParametersClient::SharedPtr param_client_;
        rclcpp::Service<ComputeSrv>::SharedPtr compute_trajectory_length_service_;
    };

private:
    using Params = scene_buffer::Params;
    using ParamListener = scene_buffer::ParamListener;
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;
    std::map<std::string, std::shared_ptr<Robot>> robots_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto compute_traj_length_node = std::make_shared<
        ComputeTrajLength>("compute_traj_length", node_options);
    compute_traj_length_node->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(compute_traj_length_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}