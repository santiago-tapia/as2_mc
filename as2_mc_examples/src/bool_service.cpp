
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <stdio.h>
#include <memory>

void do_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    static char buffer[256];
    static int i = 0;    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), request->data ? "Activate" : "Deactivate");
    response->success = true;
    sprintf(buffer, "Mensaje nº: %d", i);
    response->set__message(buffer);
    ++i;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("toggle_server");

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service =
    node->create_service<std_srvs::srv::SetBool>("toggle", &do_service);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to receive request.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}