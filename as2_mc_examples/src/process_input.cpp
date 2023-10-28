
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

  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr inputNode = std::make_shared<as2::mc::NodeWithInput>();
  rclcpp::Node::SharedPtr publishNode = std::make_shared<as2::mc::NodeWithPublisher>();

  executor.add_node(inputNode);
  executor.add_node(publishNode);
  executor.spin();    
  
  rclcpp::shutdown();
  return 0;
}