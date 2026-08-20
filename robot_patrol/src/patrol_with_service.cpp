#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <custom_interfaces/srv/get_direction.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iterator>
#include <limits>
#include <map>
#include <string>

using namespace std::chrono_literals;

class patrol_with_service : public rclcpp::Node {
public:
  patrol_with_service() : Node("patrol_with_service") {

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //"/fastbot_1/scan", qos,
        "/scan", qos, // Real Robot
        std::bind(&patrol_with_service::laser_scan_callback, this,
                  std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        //"/fastbot_1/cmd_vel", qos);
        "/cmd_vel", qos); // Real Robot

    client_ = this->create_client<custom_interfaces::srv::GetDirection>(
        service_name_);

    // Wait for the service to be available (checks every second)
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
    }

    auto timer_period = std::chrono::milliseconds(100);

    timer_ = this->create_wall_timer(
        timer_period, std::bind(&patrol_with_service::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Client Ready");
  }

private:
  void timer_callback() { send_direction_service_request(); }

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_msg_ = msg;
  }

  void send_direction_service_request() {
    if (!last_scan_msg_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Request Sent");

    auto request =
        std::make_shared<custom_interfaces::srv::GetDirection::Request>();

    request->laser_data = *last_scan_msg_;

    // Callback fires when the response arrives — no blocking, no second
    // executor
    client_->async_send_request(
        request, std::bind(&patrol_with_service::direction_response_callback,
                           this, std::placeholders::_1));
  }

  void direction_response_callback(
      rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture
          future) {

    RCLCPP_INFO(this->get_logger(), "Response Received");

    auto response = future.get();

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.1f;

    if (response->direction == "forward") {
      cmd.angular.z = 0.0f;
    } else if (response->direction == "left") {
      cmd.angular.z = 0.5f;
    } else if (response->direction == "right") {
      cmd.angular.z = -0.5f;
    }
    publisher_->publish(cmd);
  }

  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
  rclcpp::TimerBase::SharedPtr timer_;

  const std::string service_name_ = "/direction_service";
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_ = std::make_shared<patrol_with_service>();

  rclcpp::spin(node_);

  rclcpp::shutdown();

  return 0;
}