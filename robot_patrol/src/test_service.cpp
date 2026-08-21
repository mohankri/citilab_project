
//#include "geometry_msgs/msg/detail/twist__struct.hpp"
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
#include <geometry_msgs/msg/twist.hpp>
#include <iterator>
#include <limits>
#include <map>
#include <robot_patrol/srv/get_direction.hpp>
#include <string>

using namespace std::chrono_literals;

class test_service : public rclcpp::Node {
public:
  test_service() : Node("test_service") {

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //"/fastbot_1/scan",
        "/scan", qos,
        std::bind(&test_service::laser_scan_callback, this,
                  std::placeholders::_1));

    client_ =
        this->create_client<robot_patrol::srv::GetDirection>(service_name_);

    // Wait for the service to be available (checks every second)
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      // RCLCPP_INFO(this->get_logger(), "Service %s not available, waiting
      // again...", service_name_.c_str());
    }

    auto timer_period = std::chrono::milliseconds(100);

    timer_ = this->create_wall_timer(
        timer_period, std::bind(&test_service::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Client Ready");
  }

private:
  void timer_callback() {

    send_direction_service_request();

    // RCLCPP_INFO(this->get_logger(), "Angular %.2f", cmd.angular.z);
    // publisher_->publish(cmd);
  }

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_msg_ = msg;
  }

  void send_direction_service_request() {
    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    if (last_scan_msg_ == nullptr) {
      RCLCPP_INFO(this->get_logger(), "Laser Scan data not available");
      return;
    }
    request->laser_data = *last_scan_msg_;

    // Callback fires when the response arrives — no blocking, no second
    // executor
    client_->async_send_request(
        request, std::bind(&test_service::direction_response_callback, this,
                           std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Request Sent");
  }

  void direction_response_callback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response Received %s",
                response->direction.c_str());
  }

  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
  rclcpp::TimerBase::SharedPtr timer_;

  const std::string service_name_ = "/direction_service";
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_ = std::make_shared<test_service>();

  rclcpp::spin(node_);

  rclcpp::shutdown();

  return 0;
}