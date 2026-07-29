
#include "rclcpp/logging.hpp"
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <custom_interfaces/srv/get_direction.hpp>
#include <string>
#include <algorithm>

struct SectorRanges {
        float right; //0-60
        float center; //60-120 
        float left; //120-180
};

class DirectionService : public rclcpp::Node {
public:
    DirectionService() : Node("DirectionService") {

        service_ = this->create_service<custom_interfaces::srv::GetDirection>(
            service_name_,
            std::bind(&DirectionService::handle_direction_service,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Service Server Ready");

    }

private:

    SectorRanges getSectorRanges(const sensor_msgs::msg::LaserScan &msg) {
        SectorRanges sector = {
            std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity()
        };

        const float sectors_width = M_PI/3.0f;

        for (int i = 0; i < static_cast<int>(msg.ranges.size()); i++) {
            float range = msg.ranges[i];

            if (std::isnan(range) || std::isinf(range)) {
                continue;
            }

            if (range < msg.range_min || range > msg.range_max) {
                continue;
            }

            float angle = msg.angle_min + static_cast<float>(i * msg.angle_increment);
            
            angle = angle - msg.angle_min;  // always starts at 0
            if (angle < -M_PI_2 || angle > M_PI_2) 
                continue;


            if (angle < sectors_width) {
                sector.right = std::min(sector.right, range);
            } else if (angle < (2 * sectors_width)) {
                sector.center = std::min(sector.center, range);
            } else {
                sector.left = std::min(sector.left, range);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Right %f Center %f Left %f", sector.right, sector.center, sector.left);

        return sector;
    }

    std::string getDirection(SectorRanges &sector) {
        if (sector.center > OBSTACLE_THRESHOLD) {
            return "forward";
        } else if (sector.left > OBSTACLE_THRESHOLD) {
            return "left";
        } else if (sector.right > OBSTACLE_THRESHOLD) {
            return "right";
        }
        return "stop";
    }

    void handle_direction_service(const std::shared_ptr<custom_interfaces::srv::GetDirection::Request> request,
        std::shared_ptr<custom_interfaces::srv::GetDirection::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Request Received");

        const sensor_msgs::msg::LaserScan &msg = request->laser_data;
        
        SectorRanges sector = getSectorRanges(msg);
    
        RCLCPP_INFO(this->get_logger(), "Request Completed");

        response->direction = getDirection(sector);

    }
    
    static constexpr float OBSTACLE_THRESHOLD = 0.35f; //meters
    const std::string service_name_ = "/direction_service";
    rclcpp::Service<custom_interfaces::srv::GetDirection>::SharedPtr service_;

};

int
main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_ = std::make_shared<DirectionService>();

    rclcpp::spin(node_);
    rclcpp::shutdown();
}