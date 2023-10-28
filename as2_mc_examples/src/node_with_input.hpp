#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "as2_mc_input/mc_input_variable.hpp"

using namespace std::chrono_literals;

namespace as2 { namespace mc {

class NodeWithInput: public rclcpp::Node
{
public:
  NodeWithInput() : rclcpp::Node("node_with_input") 
  {
    pose.init(this);
    timer_ = this->create_wall_timer(500ms, std::bind(&NodeWithInput::tick, this));
  }

  struct Topic { static const std::string topic() { return std::string("/as2_mc/example/pose"); } };

  mc_InputVariable<Topic, geometry_msgs::msg::Pose2D> pose;

  void tick()
  {
    RCLCPP_INFO(this->get_logger(), "Data: x = %f", pose->x);
  }
  rclcpp::TimerBase::SharedPtr timer_;
};

} } // namespace
