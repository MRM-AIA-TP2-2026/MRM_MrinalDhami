#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <iostream>

class NavigatorNode : public rclcpp::Node {
public:
    NavigatorNode() : Node("simple_mover") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        imu_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/imu/data", 10, std::bind(&IMUHaversine::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Navigator Node Started. Waiting for user input...");

        // Start user input loop in a separate thread
        input_thread_ = std::thread(&NavigatorNode::get_user_input, this);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::thread input_thread_;

    double x_, y_, theta_; // Current position and orientation
    bool odom_received_ = false;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta_ = yaw;

        odom_received_ = true;
    }

    void get_user_input() {
        double target_x, target_y;
        while (rclcpp::ok()) {
            std::cout << "Enter target x: ";
            std::cin >> target_x;
            std::cout << "Enter target y: ";
            std::cin >> target_y;

            if (!odom_received_) {
                RCLCPP_WARN(this->get_logger(), "Odometry data not received yet!");
                continue;
            }

            double dx = target_x - x_;
            double dy = target_y - y_;
            double target_theta = atan2(dy, dx);

            rotate_to_target(target_theta);
            move_to_target(dx, dy);
        }
    }

    void rotate_to_target(double target_theta) {
        geometry_msgs::msg::Twist cmd;
        double error = target_theta - theta_;

        while (std::abs(error) > 0.05 && rclcpp::ok()) {
            cmd.angular.z = 0.5 * error; // Proportional control for rotation
            cmd_vel_pub_->publish(cmd);
            //rclcpp::sleep_for(std::chrono::milliseconds(75));
            error = target_theta - theta_;

            RCLCPP_INFO(this->get_logger(), "The current orientation error margin is: (%.2f)", error);
        }
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }

    void move_to_target(double dx, double dy) {
        geometry_msgs::msg::Twist cmd;
        double distance = std::sqrt(dx * dx + dy * dy);

        double moved = 0.0;
        rclcpp::Time start_time = this->now();

        while (moved < distance && rclcpp::ok()) {
            cmd.linear.x = 0.2;
            cmd_vel_pub_->publish(cmd);
            //rclcpp::sleep_for(std::chrono::milliseconds(75));

            double dt = (this->now() - start_time).seconds();
            moved = 0.2 * dt;

            RCLCPP_INFO(this->get_logger(), "The current distance moved is: (%.2f)", moved);
        }

        cmd.linear.x = 0.0;
        cmd_vel_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigatorNode>());
    rclcpp::shutdown();
    return 0;
}

