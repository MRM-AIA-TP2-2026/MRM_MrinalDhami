#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

#define EARTH_RADIUS 6371000 // Earth radius in meters
#define METERS_TO_DEG_LAT 1.0 / 111320.0  // Approximate conversion for latitude

class IMUToGPS : public rclcpp::Node {
public:
    IMUToGPS() : Node("imu_to_gps") {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_controller/out", 10, std::bind(&IMUToGPS::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU to GPS Node Started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    double lat_ = 0.0, lon_ = 0.0; // Initial lat/lon (just to start and not get erros)
    bool imu_received_ = false;

   

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "lat and long has been initialized");


        // Extracting the quaternion from IMU messages here
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        RCLCPP_INFO(this->get_logger(), "quaternions were taken");

        // Converting quaternion values to roll, pitch, yaw
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Roll pitch yaw has been gathered");

        if (!imu_received_) {
            // Set initial GPS coordinates manually
            std::cout << "Enter current latitude: ";
            std::cin >> lat_;
            std::cout << "Enter current longitude: ";
            std::cin >> lon_;
            imu_received_ = true;
        }

        // Convert pitch & yaw changes into lat/lon movement
        double delta_lat = pitch * METERS_TO_DEG_LAT;
        double delta_lon = yaw * (1.0 / (111320.0 * std::cos(lat_ * M_PI / 180.0)));

        lat_ += delta_lat;
        lon_ += delta_lon;

         // Converting the latitude and longitude into radians to be used as coordinates in haversine
        double lat_rad = lat_ * M_PI / 180.0;
        double lon_rad = lon_ * M_PI / 180.0;

        RCLCPP_INFO(this->get_logger(), "Orientation -> Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "Estimated Position -> Lat: %.6f, Lon: %.6f", lat_, lon_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUToGPS>());
    rclcpp::shutdown();
    return 0;
}
