
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "node_with_input_by_topic.hpp"
#include "node_with_publisher.hpp"
#include "node_with_output_state.hpp"

int main(int argc, char * argv[])
{
  std::cout << "Process Input/Output" << std::endl;
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr inputNode = std::make_shared<as2::mc::NodeWithInputByTopic>();
  rclcpp::Node::SharedPtr publishNode = std::make_shared<as2::mc::NodeWithPublisher>();
  rclcpp::Node::SharedPtr outputNode = std::make_shared<node_with_output_state>();

  executor.add_node(inputNode);
  executor.add_node(publishNode);
  executor.add_node(outputNode);
  executor.spin();    
  
  rclcpp::shutdown();
  return 0;
}