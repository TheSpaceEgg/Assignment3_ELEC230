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

    // if no scan data is detected - no obstacle!
    const int n = static_cast<int>(scan->ranges.size());
    if (n <= 0 || scan->angle_increment <= 0.0) {
        isObstacleDetected = false;
        return;
    }

    // don't worry too much about these - we are doing some quick calibration in case in the future we change what lidar we are using!
    const double a_min_deg = scan->angle_min * (180.0 / M_PI);
    const double a_inc_deg = scan->angle_increment * (180.0 / M_PI);
    int centre_idx = static_cast<int>(std::llround((0.0 - a_min_deg) / a_inc_deg));
    centre_idx = ((centre_idx % n) + n) % n;
    
    // half steps is looking at how many indexes of the scan lines in one direction given your total angle range you have selected. i.e. with a scan of 360 lines (1 per degree), and an angle_range of 6 degrees, you will see 3 degrees in each direction, resulting in 3 half steps, 6 scanlines in total
    const int half_steps = std::max(0, static_cast<int>(std::floor((ANGLE_RANGE_DEG * 0.5) / a_inc_deg)));

    auto is_obstacle = [&](float r) -> bool { // this is creating a simple helper "function" to check each scan line later - this stops us repeating code!
        if (!std::isfinite(r)) return false; // if a scan line is infinite that means it hasn't found any obstacle at any distance
        if (r <= 0.01f) return false; // if the reading is exceptionally low this also usually means something has disrupted the sensor like an insect, or rain on the sensor. We can ignore these.
        return r < static_cast<float>(MIN_DIST_FROM_OBSTACLE); // if the range seen on a given line is less than our set threshold, we return that threshold
    };

    //loop through our set of indexes, i.e. 357, 358, 359, 0, 1, 2, 3 (3 either side of the centre scan, achieved with a an angle range of 6-7.9 degrees)
    for (int offset = -half_steps; offset <= half_steps; ++offset) {
        const int idx = (centre_idx + offset + n) % n;
        // using our helper, we can say if we return any value that is true - so any given range above our threshold - that an obstacle is seen!
        if (is_obstacle(scan->ranges[idx])) {
            isObstacleDetected = true;
            return; // if we see an obstacle we don't need to continue with this loop or the remainder of the scan callback function
        }
    }
    // just to be sure, if we have reached this stage no obstacle has been seen. Let's just reassert it!
    isObstacleDetected = false;
}

