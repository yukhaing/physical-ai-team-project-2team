#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

class OmxXyzController
{
public:
  explicit OmxXyzController(const rclcpp::Node::SharedPtr & node)
  : node_(node), move_group_(node_, "arm")
  {
    node_->declare_parameter("planning_time", 5.0);
    node_->declare_parameter("position_tolerance", 0.01);
    node_->declare_parameter("velocity_scale", 0.2);
    node_->declare_parameter("acceleration_scale", 0.2);

    move_group_.setPlanningTime(node_->get_parameter("planning_time").as_double());
    move_group_.setGoalPositionTolerance(
      node_->get_parameter("position_tolerance").as_double());
    move_group_.setMaxVelocityScalingFactor(
      node_->get_parameter("velocity_scale").as_double());
    move_group_.setMaxAccelerationScalingFactor(
      node_->get_parameter("acceleration_scale").as_double());

    command_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "target_position", rclcpp::QoS(1),
      std::bind(&OmxXyzController::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(), "Ready. Planning frame: %s; command topic: %s",
      move_group_.getPlanningFrame().c_str(), command_sub_->get_topic_name());
  }

private:
  void command_callback(const geometry_msgs::msg::PointStamped::SharedPtr command)
  {
    if (moving_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "Robot is moving; command ignored");
      return;
    }

    std::lock_guard<std::mutex> lock(move_mutex_);
    const auto & p = command->point;
    const std::string frame = command->header.frame_id.empty() ?
      move_group_.getPlanningFrame() : command->header.frame_id;

    RCLCPP_INFO(
      node_->get_logger(), "Target in %s: x=%.4f, y=%.4f, z=%.4f",
      frame.c_str(), p.x, p.y, p.z);

    move_group_.setPoseReferenceFrame(frame);
    move_group_.setStartStateToCurrentState();
    move_group_.setPositionTarget(p.x, p.y, p.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned =
      move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;

    if (!planned) {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed; robot was not moved");
      move_group_.clearPoseTargets();
      moving_ = false;
      return;
    }

    const bool executed =
      move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (executed) {
      RCLCPP_INFO(node_->get_logger(), "Motion completed");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Execution failed");
    }

    move_group_.clearPoseTargets();
    moving_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr command_sub_;
  std::atomic_bool moving_{false};
  std::mutex move_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "omx_xyz_controller",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto controller = std::make_unique<OmxXyzController>(node);
  spin_thread.join();
  controller.reset();
  rclcpp::shutdown();
  return 0;
}
