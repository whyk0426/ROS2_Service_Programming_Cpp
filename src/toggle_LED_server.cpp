#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <memory>

int a = 0;

void toggle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response>      response)
{ 
  (void)request;
  a = a + 1;
  a = a % 2;

  if (a==0)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LED if now OFF");
  else if (a==1)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LED if now ON");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("led_toggle_server");
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service =
    node->create_service<std_srvs::srv::Empty>("toggle_led", &toggle);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LED Toggle Service Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}