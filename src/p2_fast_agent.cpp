#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class Fast_Agent : public rclcpp::Node
{
  public:
    Fast_Agent()
    : Node("fast_agent"), count_(0)
    {
      fast_publisher_ = this->create_publisher<std_msgs::msg::String>("fast_topic", 10);
      fast_timer_ = this->create_wall_timer(
      500ms, std::bind(&Fast_Agent::fast_timer_callback, this));

      slow_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "slow_topic", 10, std::bind(&Fast_Agent::slow_topic_callback, this, _1));
    }

  private:
    void fast_timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Fast timer" + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "[Fast] Publishing: '%s'", message.data.c_str());
      fast_publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fast_publisher_;
    size_t count_;  
    
    void slow_topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "[Fast] I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr slow_subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fast_Agent>());
  rclcpp::shutdown();
  return 0;
}