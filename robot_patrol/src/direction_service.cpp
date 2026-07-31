
#include "rclcpp/logging.hpp"
#include <algorithm>
#include <cmath>
#include <custom_interfaces/srv/get_direction.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

struct SectorRanges {
  float right_min; // 0-60
  float right_max;
  float center_min; // 60-120
  float center_max;
  float left_min; // 120-180
  float left_max;
};

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("DirectionService") {

    service_ = this->create_service<custom_interfaces::srv::GetDirection>(
        service_name_,
        std::bind(&DirectionService::handle_direction_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service Server Ready");
  }

private:
  const float sector_width = M_PI / 3.0f;

  SectorRanges getSectorRanges(const sensor_msgs::msg::LaserScan &msg) {

    SectorRanges sector = {std::numeric_limits<float>::infinity(), 0,
                           std::numeric_limits<float>::infinity(), 0,
                           std::numeric_limits<float>::infinity(), 0};

    const float sectors_width = M_PI / 3.0f;

#if 1
    for (int i = 0; i < static_cast<int>(msg.ranges.size()); i++) {
      float range = msg.ranges[i];

      if (std::isnan(range) || std::isinf(range)) {
        continue;
      }

      if (range < msg.range_min || range > msg.range_max) {
        continue;
      }

      float angle = msg.angle_min + static_cast<float>(i * msg.angle_increment);

      // angle = angle - msg.angle_min;  // always starts at 0
      if (angle < -M_PI_2 || angle > M_PI_2)
        continue;

      if (angle < -M_PI_2 + sectors_width) {
        sector.right_min = std::min(sector.right_min, range);
        sector.right_max = std::max(sector.right_max, range);
      } else if (angle < -M_PI_2 + (2.0f * sectors_width)) {
        sector.center_min = std::min(sector.center_min, range);
        sector.center_max = std::max(sector.center_max, range);
      } else {
        sector.left_min = std::min(sector.left_min, range);
        sector.left_max = std::max(sector.left_max, range);
      }
    }
#endif

#if 0
    const float sector_width = M_PI / 3.0f;

    for (std::size_t i = 0; i < msg.ranges.size(); ++i) {
      const float range = msg.ranges[i];

      if (!std::isfinite(range)) {
        continue;
      }

      if (range < msg.range_min || range > msg.range_max) {
        continue;
      }

      const float angle =
          msg.angle_min + static_cast<float>(i) * msg.angle_increment;

      // Use only the front 180 degrees.
      if (angle < -M_PI_2 || angle > M_PI_2) {
        continue;
      }

      if (angle < -M_PI_2 + sector_width) {
        sector.right = std::max(sector.right, range);
      } else if (angle < -M_PI_2 + 2.0f * sector_width) {
        sector.center = std::max(sector.center, range);
      } else {
        sector.left = std::max(sector.left, range);
      }
    }
#endif

    RCLCPP_INFO(this->get_logger(), "Min Right %f center %f Left %f",
                sector.right_min, sector.center_min, sector.left_min);
    RCLCPP_INFO(this->get_logger(), "Max Right %f Center %f Left %f",
                sector.right_max, sector.center_max, sector.left_max);

    return sector;
  }

  std::string getDirection(SectorRanges &sector) {
    if (sector.center_min > OBSTACLE_THRESHOLD) {
      return "forward";
      // return (sector.left_max > sector.right_max) ? "left" : "right";
    } else {
      return (sector.left_max > sector.right_max) ? "left" : "right";
    }
    /*  if (sector.center_min > OBSTACLE_THRESHOLD) {
        return "forward";
      } else if (sector.left_min > OBSTACLE_THRESHOLD) {
        return "left";
      } else if (sector.right_min > OBSTACLE_THRESHOLD) {
        return "right";
      } */
    return "stop";
  }

  void handle_direction_service(
      const std::shared_ptr<custom_interfaces::srv::GetDirection::Request>
          request,
      std::shared_ptr<custom_interfaces::srv::GetDirection::Response>
          response) {

    RCLCPP_INFO(this->get_logger(), "Request Received");

    const sensor_msgs::msg::LaserScan &msg = request->laser_data;

    SectorRanges sector = getSectorRanges(msg);

    RCLCPP_INFO(this->get_logger(), "Request Completed");

    response->direction = getDirection(sector);
  }

  static constexpr float OBSTACLE_THRESHOLD = 0.35f; // meters
  const std::string service_name_ = "/direction_service";
  rclcpp::Service<custom_interfaces::srv::GetDirection>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node_ = std::make_shared<DirectionService>();

  rclcpp::spin(node_);
  rclcpp::shutdown();
}