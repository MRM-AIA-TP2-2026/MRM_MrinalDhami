#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ChatNode : public rclcpp::Node {
public:
  ChatNode(const std::string &username) : Node(username), username_(username) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chat_topic", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chat_topic", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::string text = msg->data;
        size_t start = text.find('[');
        size_t end = text.find(']');
        if (start != std::string::npos && end != std::string::npos && end > start) {
          std::string sender = text.substr(start+1, end - start - 1);
          if(sender != username_) {
            std::string message = text.substr(end+2);
            std::cout << sender << " said: " << message << std::endl;
          }
        }
      }
    );
    input_thread_ = std::thread(&ChatNode::input_loop, this);
  }
  
  ~ChatNode() {
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  void input_loop() {
    while(rclcpp::ok()){
      std::cout << "Enter your message: ";
      std::string input;
      std::getline(std::cin, input);
      std_msgs::msg::String msg;
      msg.data = "[" + username_ + "]: " + input;
      publisher_->publish(msg);
      std::cout << "You said: " << input << std::endl;
    }
  }
  
  std::string username_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::thread input_thread_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  if(argc < 2) {
    std::cerr << "Usage: ros2 run chatroom chat_node <username>" << std::endl;
    return 1;
  }
  auto node = std::make_shared<ChatNode>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
