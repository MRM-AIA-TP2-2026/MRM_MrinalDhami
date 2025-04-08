#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <iostream>


// To make the code wait in few areas to debuggg better
#include <thread>
#include <chrono>

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

    RCLCPP_INFO(rclcpp::get_logger("imu_to_gps"), " bearing has beem calculated %f",atan2(y,x)); // Bearing in radians

    
    return  M_PI + M_PI_2 - atan2(y, x);
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

    // Here wheneevr spin() functions makes the Autonomous Traversal's use, it asks for the target target coordinates
    // and then continuously listens to imu and gps data being published and publishes the required data to /cmd_vel whenever necessary.

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

    bool goal_reached() const {
        return destinaton_;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    bool yaw_received_;
    bool rotation_complete_;
    bool gps_received_;
    bool goal_done_;
    bool moving_;
    double initial_yaw_;
    double target_yaw_;
    double target_lat_;
    double target_lon_;
    double current_lat_;
    double current_lon_;
    double destinaton_=false;

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
        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        // Always update the distance
        double distance = haversine_distance(current_lat_, current_lon_, target_lat_, target_lon_);

        if (!gps_received_) {
            gps_received_ = true;
            target_yaw_ = for_azimuth(current_lat_, current_lon_, target_lat_, target_lon_);
            RCLCPP_INFO(this->get_logger(), " GPS received! Target bearing: %.4f rad", target_yaw_);

            std::cout << "Waiting for 3 seconds...\n";
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
        else {
            RCLCPP_INFO(this->get_logger(), "The specifications are | latitude: %f | longitude: %f | distance: %f \n", current_lat_, current_lon_, distance);

        if (distance < 0.4) {
            RCLCPP_INFO(this->get_logger(),"The distance is: %f \n", distance);
            RCLCPP_INFO(this->get_logger(), " Destination reached! Stopping.");
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(stop_msg);
            moving_ = false;
            destinaton_ = true;
            return;
        }
        else if(rotation_complete_){
    
        // Move towards the target continuousl
            
            // double x = target_lon_ ;
            // double y = target_lat_ ;
        
            double speed;
            if (distance < 0.4) {
                speed = std::clamp(distance * 2.5, 0.05, 0.1);}
            else if (distance < 2.0) {
                speed = std::clamp(distance * 0.6, 0.2, 0.5);}
            else {
                speed = std::clamp(distance * 0.3, 0.5, 1.0);
            }
        
        
            // placing the condition that whenerver both cooridnates are of same sign, i make the linear sped to be -ve cause why not
            // if ((x > 0 && y > 0) || (x < 0 && y < 0)) {
            //     speed = -speed;
            // }
        
            geometry_msgs::msg::Twist move_msg;
            move_msg.linear.x = speed;
            
            cmd_vel_publisher_->publish(move_msg);
            moving_ = true;

            RCLCPP_INFO(this->get_logger(), "Speed is %f", speed);}}
    }

    //gps_call back stops and imu_callback starts in the down.



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

        if (std::abs(yaw_error) < 0.009) {
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel_msg);
            RCLCPP_INFO(this->get_logger(), "Rotation complete, the Yaw comes out to be: %f, ! Starting movement.", current_yaw);
            rotation_complete_ = true;
            return;
        }
        

        double turn_speed = std::clamp(std::abs(yaw_error) * 0.6, 0.03, 0.3);
        cmd_vel_msg.angular.z = (yaw_error > 0) ? -turn_speed : turn_speed;
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    while (rclcpp::ok()) {
        double target_lat, target_lon;
        std::cout << "Enter target latitude: ";
        std::cin >> target_lat;
        std::cout << "Enter target longitude: ";
        std::cin >> target_lon;

        auto node = std::make_shared<AutonomousTraversal>(target_lat, target_lon);
        // Auto keyword automaticvally detects the type of variable we are making, helps us to decrease the typing required.

        rclcpp::executors::SingleThreadedExecutor exec;
        // We make an executor of Single Thread type because, 
        // 1:- Executors handle the working of the node. So every action with a node is the reponsibility of the executor
        // 2:- Single Thread because we want 1 callback to be done before the next callback can be processed.

        exec.add_node(node);
        //Asking my exec to handle my node(Autonomous Traversal )
        // SO basically default spin(rclpp::spin() ) will make it run untill I stop it, but I can use a custom executor to stop the spin and start it again on my given conditions
        while (rclcpp::ok() && !node->goal_reached()) {
            exec.spin_some();           
            //rclcpp::sleep_for(100ms);   
        }

        exec.remove_node(node);
        std::cout << " Destination reached!\n\n";
    }

    rclcpp::shutdown();
    return 0;
}