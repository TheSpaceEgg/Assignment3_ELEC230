#include "Turtlebot3Driver.hpp"

Turtlebot3Driver::Turtlebot3Driver() : Node("turtlebot3_driver"), isObstacleDetected(false) {
    
    commandPub = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    
    auto qos_profile = rclcpp::SensorDataQoS();
    laserSub = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos_profile, std::bind(&Turtlebot3Driver::scanCallback, this, std::placeholders::_1));
}

void Turtlebot3Driver::moveForward() {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "base_link";
    
    msg->twist.linear.x = FORWARD_SPEED;
    commandPub->publish(std::move(msg));
}

void Turtlebot3Driver::stopMoving() {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    
    msg->header.stamp = this->now();
    msg->header.frame_id = "base_link";
    commandPub->publish(std::move(msg));
}

void Turtlebot3Driver::turn(double angular) {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    
    msg->header.stamp = this->now();
    msg->header.frame_id = "base_link";
    
    msg->twist.angular.z = angular;
    commandPub->publish(std::move(msg));
}

bool Turtlebot3Driver::obstacleDetected() const {
    return isObstacleDetected;
}

void Turtlebot3Driver::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    const double ANGLE_RANGE_DEG = 1;
    const double MIN_DIST_FROM_OBSTACLE = 0.5;

    const int n = static_cast<int>(scan->ranges.size());
    if (n <= 0 || scan->angle_increment <= 0.0) {
        isObstacleDetected = false;
        return;
    }

    const double a_min_deg = scan->angle_min * (180.0 / M_PI);
    const double a_inc_deg = scan->angle_increment * (180.0 / M_PI);

    int centre_idx = static_cast<int>(std::llround((0.0 - a_min_deg) / a_inc_deg));
    centre_idx = ((centre_idx % n) + n) % n;

    const int half_steps = std::max(0, static_cast<int>(std::floor((ANGLE_RANGE_DEG * 0.5) / a_inc_deg)));

    auto is_obstacle = [&](float r) -> bool {
        if (!std::isfinite(r)) return false;
        if (r <= 0.01f) return false;
        return r < static_cast<float>(MIN_DIST_FROM_OBSTACLE);
    };

    for (int offset = -half_steps; offset <= half_steps; ++offset) {
        const int idx = (centre_idx + offset + n) % n;

        if (is_obstacle(scan->ranges[idx])) {
            isObstacleDetected = true;
            return;
        }
    }

    isObstacleDetected = false;
}

