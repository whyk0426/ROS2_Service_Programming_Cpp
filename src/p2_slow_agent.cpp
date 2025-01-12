#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class Slow_Agent : public rclcpp::Node
{
  public:
    Slow_Agent()
    : Node("slow_agent"), count_(0)
    {
      slow_publisher_ = this->create_publisher<std_msgs::msg::String>("slow_topic", 10);
      slow_timer_ = this->create_wall_timer(
      1000ms, std::bind(&Slow_Agent::slow_timer_callback, this));

      fast_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "fast_topic", 10, std::bind(&Slow_Agent::fast_topic_callback, this, _1));
    }

  private:
    void slow_timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Slow timer" + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "[Slow] Publishing: '%s'", message.data.c_str());
      slow_publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr slow_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slow_publisher_;
    size_t count_;  
    
    void fast_topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "[Slow] I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fast_subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slow_Agent>());
  rclcpp::shutdown();
  return 0;
}