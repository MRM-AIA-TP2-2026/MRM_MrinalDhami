#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SimpleMover : public rclcpp::Node
{
public:
    SimpleMover() : Node("simple_mover"), target_x(0.0), target_y(0.0), current_x(0.0), current_y(0.0)
    {
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SimpleMover::odom_callback, this, std::placeholders::_1));
        target_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/target_cmd", 10, std::bind(&SimpleMover::target_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;
        move_to_target();
    }

    void target_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_x = current_x + msg->linear.x;
        target_y = current_y + msg->linear.y;
        RCLCPP_INFO(this->get_logger(), "Target set to: (%.2f, %.2f)", target_x, target_y);
    }

    void move_to_target()
    {
        geometry_msgs::msg::Twist cmd;
        
        if (std::abs(target_x - current_x) > 0.05)
        {
            cmd.linear.x = (target_x > current_x) ? 0.2 : -0.2;
        }
        else if (std::abs(target_y - current_y) > 0.05)
        {
            cmd.linear.y = (target_y > current_y) ? 0.2 : -0.2;
        }
        else
        {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
        }
        
        cmd_vel_pub->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_sub;
    
    double target_x, target_y;
    double current_x, current_y;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMover>());
    rclcpp::shutdown();
    return 0;
}

