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

void Turtlebot3Driver::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    const double ANGLE_RANGE = 0.032;
    const double MIN_DIST_FROM_OBSTACLE = 0.5;

    double angle_min = scan->angle_min;
    double angle_increment = scan->angle_increment;
    
    int start_index = static_cast<int>((-ANGLE_RANGE / 2 - angle_min) / angle_increment);
    int end_index = static_cast<int>((ANGLE_RANGE / 2 - angle_min) / angle_increment);

    if (start_index < 0) start_index = 0;
    if (end_index >= (int)scan->ranges.size()) end_index = scan->ranges.size() - 1;

    for (int i = start_index; i <= end_index; ++i) {
        if (scan->ranges[i] < MIN_DIST_FROM_OBSTACLE && scan->ranges[i] > 0.01) {
            isObstacleDetected = true;
            return;
        }
    }
    isObstacleDetected = false;
}
