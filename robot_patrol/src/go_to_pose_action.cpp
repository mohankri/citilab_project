
#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "action_msgs/msg/goal_status.hpp"
#include "custom_interfaces/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/utils.h>

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = custom_interfaces::action::GoToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPoseAction>;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("GotoPose", options) {
    using namespace std::placeholders;

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    node_ = rclcpp_action::create_server<GoToPoseAction>(
        this, action_name_, std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

#if 0
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot_1/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", qos);
#endif

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);

    RCLCPP_INFO(get_logger(), "Service Server Ready");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);

    current_pos_.x = odom_msg->pose.pose.position.x;
    current_pos_.y = odom_msg->pose.pose.position.y;
    current_pos_.theta = tf2::getYaw(q);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {

    (void)uuid;

    desired_pos_ = goal->goal_pos;

    RCLCPP_INFO(get_logger(), "Goal Received");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    using namespace std::placeholders;

    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<GoToPoseAction::Feedback>();

    auto result = std::make_shared<GoToPoseAction::Result>();
    auto timer = create_wall_timer(std::chrono::milliseconds(1000), // 10 Hz
                                   [this, goal_handle, feedback]() {
                                     feedback->current_pos = this->current_pos_;
                                     goal_handle->publish_feedback(feedback);
                                   });

    send_navigation_goal(goal_handle);
    timer->cancel();
  }

  void
  send_navigation_goal(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    enum Cmd { MOVE_TO_POSITION, ALIGN_TO_TARGET };
    auto result = std::make_shared<GoToPoseAction::Result>();
    rclcpp::Rate loop_rate(10);

    Cmd move = MOVE_TO_POSITION;

    while (rclcpp::ok()) {

      if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);
        return;
      }

      // 1. Position error
      double dx = desired_pos_.x - current_pos_.x;
      double dy = desired_pos_.y - current_pos_.y;
      double positional_error = std::hypot(dx, dy);
      geometry_msgs::msg::Twist cmd;
      double goal_direction;
      double orientation_error;

      switch (move) {
      case MOVE_TO_POSITION:
        // Goal reached?
        if (positional_error < positional_tolerance_) {
          move = ALIGN_TO_TARGET;
          break;
        }

        // 2. Direction to goal
        goal_direction = std::atan2(dy, dx);

        // 4. Normalize to [-pi, pi]
        orientation_error =
            normalize_angle(goal_direction - current_pos_.theta);
        cmd.angular.z = std::clamp(orientation_error, -1.0, 1.0);
        cmd.linear.x = 0.1;
        publisher_->publish(cmd);
        break;
      case ALIGN_TO_TARGET:
        // 3. Angular error vs current heading
        orientation_error =
            normalize_angle(desired_pos_.theta - current_pos_.theta);

        // 4. Normalize to [-pi, pi]
        if (std::fabs(orientation_error) <= orientation_tolerance_) {
          cmd.angular.x = 0.0;
          publisher_->publish(cmd);
          result->status = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(get_logger(), "Goal Completed");

          return;
        }

        // Proportional control
        cmd.angular.z = std::clamp(orientation_error, -1.0, 1.0);
        publisher_->publish(cmd);
        break;
      }
      loop_rate.sleep();
    }
    return;
  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  const std::string action_name_ = "/go_to_pose";
  bool first_odom_ = true;
  const float positional_tolerance_ = 0.075f;
  const double orientation_tolerance_ = (10 * M_PI) / 180; // 10 deg

  rclcpp_action::Server<GoToPoseAction>::SharedPtr node_;
  geometry_msgs::msg::Pose2D desired_pos_, current_pos_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_publisher_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto server_ = std::make_shared<GoToPose>();

  rclcpp::spin(server_);
  rclcpp::shutdown();

  return 0;
}