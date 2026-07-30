
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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/node_options.hpp"
#include <tf2/utils.h>

class GoToPose : public rclcpp::Node {
public:
    using GoToPoseAction = custom_interfaces::action::GoToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPoseAction>;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;


    explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("GotoPose", options)
    {
        using namespace std::placeholders;

        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

        node_ = rclcpp_action::create_server<GoToPoseAction>(
            this, 
            action_name_,
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1));

        odom_sub_ = 
            create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&GoToPose::odom_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            qos);

        set_initial_pose(0.0, 0.0, 0.0);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        tf2::Quaternion q(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);

        current_pos_.x = current_x_ = odom_msg->pose.pose.position.x;
        current_pos_.y = current_y_ = odom_msg->pose.pose.position.y;
        current_pos_.theta = current_yaw_ = tf2::getYaw(q);

        if (first_odom_) {
            last_x_ = current_x_;
            last_y_ = current_y_;
            last_yaw_ = current_yaw_;
            first_odom_ = false;
            return;
        }

        double dx = current_x_ - last_x_;
        double dy = current_y_ - last_y_;
       // total_distance_traveled_ += std::sqrt(dx * dx + dy * dy);

        last_x_ = current_x_;
        last_y_ = current_y_;
        last_yaw_ = current_yaw_;
    }

    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
        
        (void)uuid;
        
        desired_pos_ = goal->goal_pos;

        RCLCPP_INFO(get_logger(),
                "Received goal request: "
                "x=%.2f, y=%.2f, theta=%.2f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        (void)goal_handle;

        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        using namespace std::placeholders;

        std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(get_logger(), "Executing goal.");
        const auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<GoToPoseAction::Feedback>();

        auto result = std::make_shared<GoToPoseAction::Result>();
#if 1
        auto timer = create_wall_timer(
            std::chrono::milliseconds(1000),  // 10 Hz
                [this, goal_handle, feedback]() {
                    //const auto goal = goal_handle->get_goal();
                    feedback->current_pos = this->current_pos_;
                    goal_handle->publish_feedback(feedback);
        });

        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        timer->cancel();
#endif
    }

    void set_initial_pose(double x, double y, double yaw) {
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;

        initial_pose.header.frame_id = "map";
        initial_pose.header.stamp = get_clock()->now();

        initial_pose.pose.pose.position.x = x;
        initial_pose.pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);

        initial_pose.pose.pose.orientation = tf2::toMsg(q);

        for (int i = 0; i < 10; ++i) {
            initial_pose.header.stamp = get_clock()->now();
           // initial_pose_publisher_->publish(initial_pose);

            RCLCPP_INFO(get_logger(), "Publishing initial pose (%d/10)", i + 1);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(get_logger(), "Initial pose set to x: %.2f, y: %.2f, yaw: %.2f",
            x, y, yaw);
    }

    const std::string action_name_ = "/go_to_pose";
    bool first_odom_ = true;
    rclcpp_action::Server<GoToPoseAction>::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    double last_x_ = 0.0, last_y_ = 0.0, last_yaw_ = 0.0;
    //rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr current_pos_, desired_pos_;
    geometry_msgs::msg::Pose2D desired_pos_, current_pos_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto server_ = std::make_shared<GoToPose>();

    rclcpp::spin(server_);
    rclcpp::shutdown();

    return 0;
}