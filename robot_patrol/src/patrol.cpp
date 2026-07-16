
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/create_subscription.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iterator>
#include <limits>
#include <map>
#include <string>
#include <algorithm>

class Patrol : public rclcpp::Node {
public:
    Patrol(const std::string &node_name = "patrol_node")
        : Node("patrol_node"), node_name_(node_name) {
    
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            //"/fastbot_1/scan", 
            "/scan", 
            qos,
            std::bind(&Patrol::lcallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            //"/fastbot_1/cmd_vel", 
            "/cmd_vel", 
            qos);
        
        auto timer_period = std::chrono::milliseconds(100);

        timer_ = this->create_wall_timer(timer_period, 
            std::bind(&Patrol::timer_callback, this));

        //RCLCPP_INFO(this->get_logger(), "%s Ready...", node_name_.c_str());
    }

private:
    std::string node_name_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    float direction_ = 0;
    bool  obstacle_ahead_ = false;

    //static constexpr float OBSTACLE_THRESHOLD = 0.35f; // metres
    static constexpr float OBSTACLE_THRESHOLD = 0.70f; // metres

    static constexpr float HALF_FOV  = M_PI / 18.0f; // 20
    //static constexpr float HALF_FOV = M_PI; /// 3.0f;  //30
    //static constexpr float HALF_FOV = M_PI;

    
    // Compact struct to hold a filtered ray
    struct Ray {
        float angle;
        float range;
    };

    std::vector<Ray> front_ranges_; // front 180° rays, rebuilt each callback

    void timer_callback() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.1;
        if (!obstacle_ahead_) {
            cmd.angular.z = 0.0;
        } else {
            cmd.angular.z = direction_/2;
        }
        RCLCPP_INFO(this->get_logger(), "Angular %.2f", cmd.angular.z);
        publisher_->publish(cmd);
    }

    /*
        ros2 interface show sensor_msgs/msg/LaserScan
    */
    void lcallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        obstacle_ahead_ = false;
        front_ranges_.clear();

        // --- Capture only the front 180° rays (−π/2 to +π/2) ---
        for (int i = 0; i < static_cast<int>(msg->ranges.size()); i++) {
            float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;

            // Discard anything outside the front hemisphere
            if (angle < -M_PI_2 || angle > M_PI_2) 
                continue;

            front_ranges_.push_back({ angle, msg->ranges[i]});
        }

        // --- Step 1: check the ±15° forward cone for obstacles ---
        for (const auto &ray : front_ranges_) {
            if (!std::isfinite(ray.range)) {
                continue;
            }
            
            if (std::abs(ray.angle) > HALF_FOV)
                continue;

            if (ray.range < OBSTACLE_THRESHOLD) {
                obstacle_ahead_ = true;
                RCLCPP_INFO(this->get_logger(), "Obstacle Detected %.2f", ray.range);
                break;
            }
        }

        if (obstacle_ahead_) {
            float best_range = -1.0f;
            float best_angle =  0.0f;
            for (const auto &ray : front_ranges_) {
                if (!std::isfinite(ray.range)) 
                    continue;

                if (ray.range > best_range) {
                    best_range = ray.range;
                    best_angle = ray.angle;
                }
            }

            direction_ = best_angle;
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
