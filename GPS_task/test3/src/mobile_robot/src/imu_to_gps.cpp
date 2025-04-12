#include "utils.hpp"

class AutonomousTraversal : public rclcpp::Node {
public:

    // Here wheneevr spin() functions makes the Autonomous Traversal's use, it asks for the target target coordinates
    // and then continuously listens to imu and gps data being published and publishes the required data to /cmd_vel whenever necessary.

    AutonomousTraversal(double target_lat, double target_lon)
        : Node("autonomous_traversal"), rotation_complete_(false),
          gps_received_(false), mover(0.0), bender(0.0), initial_yaw_(0.0), target_yaw_(0.0),
          target_lat_(target_lat), target_lon_(target_lon), current_lat_(0.0), current_lon_(0.0), destinaton_(false){

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_controller/out", 10, std::bind(&AutonomousTraversal::imu_callback, this, std::placeholders::_1));

        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps_plugin/out", 10, std::bind(&AutonomousTraversal::gps_callback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AutonomousTraversal::publisher_plus, this)
        );

        RCLCPP_INFO(this->get_logger(), "⏳ Waiting for GPS data...");
    }
    bool goal_reached() const {
        return destinaton_;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer;

    bool rotation_complete_;
    bool gps_received_;
    double mover;
    double bender;
    double initial_yaw_;
    double target_yaw_;
    double target_lat_;
    double target_lon_;
    double current_lat_;
    double current_lon_;
    double destinaton_;

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

    void publisher_plus() {
        if (destinaton_) return;

        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = mover;
        cmd_msg.angular.z = bender;
        cmd_vel_publisher_->publish(cmd_msg);
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {

        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        // Always update the distance
        double distance = haversine_distance(current_lat_, current_lon_, target_lat_, target_lon_);

        // This part to be deleted after it starts working
        if (!gps_received_) {
            gps_received_ = true;
            RCLCPP_INFO(this->get_logger(), " GPS received!: %f, %f rad", target_lat_, target_lon_ );
            std::cout << "Waiting for 3 seconds...\n";
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
        //
        else {
            target_yaw_ = for_azimuth(current_lat_, current_lon_, target_lat_, target_lon_);
            RCLCPP_INFO(this->get_logger(), "The specifications are | latitude: %f | longitude: %f | distance: %f \n", current_lat_, current_lon_, distance);

        if (distance < 0.4) {
            RCLCPP_INFO(this->get_logger(),"The distance is: %f \n", distance);
            RCLCPP_INFO(this->get_logger(), " Destination reached! Stopping.");
            geometry_msgs::msg::Twist stop_msg;
            mover = 0.0;
            bender=0.0;
            publisher_plus();
            destinaton_ = true;
            return;
        }
        else{
    
        // Move towards the target continuously
        
            double speed;
            if (distance < 0.5) {
                speed = std::clamp(distance * 2.5, 0.05, 0.1);}
            else if (distance < 2.0) {
                speed = std::clamp(distance * 0.6, 0.2, 0.5);}
            else {
                speed = std::clamp(distance * 0.3, 0.5, 1.0);
            }
            mover = speed;
            RCLCPP_INFO(this->get_logger(), "Speed is %f", speed);}}
    }

    //gps_call back stops and imu_callback starts in the down.



    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (destinaton_) return;

        double current_yaw = get_yaw_from_quaternion(msg);

        if (!destinaton_) {
            target_yaw_ = normalize_angle(target_yaw_);
        }

        double yaw_error = normalize_angle(target_yaw_ - current_yaw);
        RCLCPP_INFO(this->get_logger(), "Yaw: %.4f | Target: %.4f | Error: %.4f", 
                    current_yaw, target_yaw_, yaw_error);

        if (destinaton_) {
            bender = 0.0;
            RCLCPP_INFO(this->get_logger(), "I have reached the desired angle yay :D");
        } 
        else if(gps_received_){
            RCLCPP_INFO(this->get_logger(), "The rover is rotating right now, error is %f", yaw_error);
            double turn_speed = std::clamp(std::abs(yaw_error) * 0.6, 0.02, 0.3);
            bender = (yaw_error > 0) ? -turn_speed : turn_speed;}
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