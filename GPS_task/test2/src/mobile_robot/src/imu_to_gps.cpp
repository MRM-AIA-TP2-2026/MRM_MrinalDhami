#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <iostream>

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)
#define EARTH_RADIUS 6371000.0 // meters

// Compute initial bearing using Haversine formula
double for_azimuth(double lat1, double lon1, double lat2, double lon2) {
    lat1 = DEG2RAD(lat1);
    lon1 = DEG2RAD(lon1);
    lat2 = DEG2RAD(lat2);
    lon2 = DEG2RAD(lon2);

    double dLon = lon2 - lon1;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    return atan2(y, x); // Bearing in radians
}

// Compute distance using Haversine formula
double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    lat1 = DEG2RAD(lat1);
    lon1 = DEG2RAD(lon1);
    lat2 = DEG2RAD(lat2);
    lon2 = DEG2RAD(lon2);

    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c; // Distance in meters
}

class AutonomousTraversal : public rclcpp::Node {
public:
    AutonomousTraversal(double target_lat, double target_lon)
        : Node("autonomous_traversal"), yaw_received_(false), rotation_complete_(false),
          gps_received_(false), initial_yaw_(0.0), target_yaw_(0.0),
          target_lat_(target_lat), target_lon_(target_lon), current_lat_(0.0), current_lon_(0.0) {

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_controller/out", 10, std::bind(&AutonomousTraversal::imu_callback, this, std::placeholders::_1));

        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps_plugin/out", 10, std::bind(&AutonomousTraversal::gps_callback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "⏳ Waiting for GPS data...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    bool yaw_received_;
    bool rotation_complete_;
    bool gps_received_;
    double initial_yaw_;
    double target_yaw_;
    double target_lat_;
    double target_lon_;
    double current_lat_;
    double current_lon_;

    double get_yaw_from_quaternion(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }


    // To normalize the angle between pi and - pi
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (gps_received_) return;

        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        gps_received_ = true;

        target_yaw_ = for_azimuth(current_lat_, current_lon_, target_lat_, target_lon_);
        RCLCPP_INFO(this->get_logger(), " GPS received! Current: (%.6f, %.6f) | Target: (%.6f, %.6f) | Bearing: %.4f rad",
                    current_lat_, current_lon_, target_lat_, target_lon_, target_yaw_);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (rotation_complete_ || !gps_received_) return;

        double current_yaw = get_yaw_from_quaternion(msg);

        if (!yaw_received_) {
            target_yaw_ = normalize_angle(target_yaw_);
            yaw_received_ = true;
        }

        double yaw_error = normalize_angle(target_yaw_ - current_yaw);

        RCLCPP_INFO(this->get_logger(), "Yaw: %.4f | Target: %.4f | Error: %.4f", 
                    current_yaw, target_yaw_, yaw_error);

        geometry_msgs::msg::Twist cmd_vel_msg;

        if (std::abs(yaw_error) < 0.05) {
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel_msg);
            RCLCPP_INFO(this->get_logger(), "✅ Rotation complete! Starting movement.");
            rotation_complete_ = true;
            return;
        }

        double turn_speed = std::clamp(std::abs(yaw_error) * 1.5, 0.1, 0.5);
        cmd_vel_msg.angular.z = (yaw_error > 0) ? -turn_speed : turn_speed;
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    double target_lat, target_lon;
    std::cout << "Enter target latitude: ";
    std::cin >> target_lat;
    std::cout << "Enter target longitude: ";
    std::cin >> target_lon;

    rclcpp::spin(std::make_shared<AutonomousTraversal>(target_lat, target_lon));
    rclcpp::shutdown();
    return 0;
}