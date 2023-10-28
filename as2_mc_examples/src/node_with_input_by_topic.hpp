#include <chrono>
#include <functional>
#include <memory>

#include "as2_mc_timer_tick/mc_timer.hpp"
#include "as2_mc_timer_tick/i_timer_tick.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "as2_mc_input/mc_input_variable_by_topic.hpp"

using namespace std::chrono_literals;

namespace as2 { namespace mc {

class TheNaming 
{
public:
  static std::string name(size_t) 
  {
    return std::string("/as2_mc/example/pose");
  }
};

class NodeWithInputByTopic: public rclcpp::Node, public mc_Timer, public i_TimerTick
{
public:
  NodeWithInputByTopic() : rclcpp::Node("node_with_input") 
  {
    mc_Timer::init(this, this);
    pose.init(this);
  }

  mc_InputVariableByTopic<0, TheNaming, geometry_msgs::msg::Pose2D> pose;

  void timer_tick() override 
  {
    RCLCPP_INFO(this->get_logger(), "Data: x = %f", pose->x);
  }
};

} } // namespace
