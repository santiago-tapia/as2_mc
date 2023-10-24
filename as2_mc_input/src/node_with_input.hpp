
#include "as2_mc_input/mc_input_variable.hpp"

namespace as2 { namespace mc {

class NodeWithInput: public rclcpp::Node
{
public:
  NodeWithInput() : rclcpp::Node("node_with_input") 
  {
    pose.init(this);
  }

  struct Topic { static const std::string topic() { return std::string("/as2_mc/example/pose"); } };

  mc_InputVariable<Topic, geometry_msgs::msg::Pose2D> pose;

};

} } // namespace
