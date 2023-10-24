
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "node_with_input.hpp"
#include "node_with_publisher.hpp"

int main(int argc, char * argv[])
{
  std::cout << "Process Input" << std::endl;
  rclcpp::init(argc, argv);
  
  rclcpp::shutdown();
  return 0;
}