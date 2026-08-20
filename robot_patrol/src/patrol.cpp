
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>
#include <iterator>
#include <limits>
#include <map>
#include <string>

class Patrol : public rclcpp::Node {
public:
  Patrol(const std::string &node_name = "patrol_node")
      : Node("patrol_node"), node_name_(node_name) {

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan",
        //"/scan",
        qos, std::bind(&Patrol::lcallback, this, std::placeholders::_1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel",
                                                          //"/cmd_vel",
                                                          qos);

    auto timer_period = std::chrono::milliseconds(100);

    timer_ = this->create_wall_timer(timer_period,
                                     std::bind(&Patrol::timer_callback, this));

    // RCLCPP_INFO(this->get_logger(), "%s Ready...", node_name_.c_str());
  }

private:
  std::string node_name_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  float direction_ = 0;
  bool obstacle_ahead_ = false;

  static constexpr float OBSTACLE_THRESHOLD = 0.30f; // metres
  // static constexpr float OBSTACLE_THRESHOLD = 0.55f; // metres

  // static constexpr float HALF_FOV  = M_PI / 18.0f; // 20
  // static constexpr float HALF_FOV = M_PI; /// 3.0f;  //30
  static constexpr float HALF_FOV = M_PI;

  void timer_callback() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.1;
    if (!obstacle_ahead_) {
      cmd.angular.z = 0.0;
    } else {
      cmd.angular.z = direction_ / 2;
    }
    // RCLCPP_INFO(this->get_logger(), "Angular %.2f", cmd.angular.z);
    publisher_->publish(cmd);
  }

  /*
      ros2 interface show sensor_msgs/msg/LaserScan
  */
  void lcallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    obstacle_ahead_ = false;

    float center_min = std::numeric_limits<float>::infinity();
    float center_max = 0.0f;

    for (int i = 0; i < static_cast<int>(msg->ranges.size()); i++) {
      float range = msg->ranges[i];

      if (std::isnan(range) || std::isinf(range)) {
        continue;
      }

      if (range < msg->range_min || range > msg->range_max) {
        continue;
      }

      float angle =
          msg->angle_min + static_cast<float>(i * msg->angle_increment);

      // angle = angle - msg.angle_min;  // always starts at 0
      if (angle < -M_PI_2 || angle > M_PI_2)
        continue;

      if (angle > -M_PI / 10.0 && angle < M_PI / 10.0) {
        center_min = std::min(center_min, range);
        center_max = std::max(center_max, range);
      }
    }

    if (center_min <= OBSTACLE_THRESHOLD) {
      obstacle_ahead_ = true;
      direction_ = -0.80f;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Patrol>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}