
#include "as2_mc_output/mc_output_variable_by_topic.hpp"
#include "as2_mc_timer_tick/i_timer_tick.hpp"
#include "as2_mc_timer_tick/mc_timer.hpp"

namespace as2 { namespace mc {

class NodeWithPublisher : public rclcpp::Node, public mc_Timer, public i_TimerTick
{
public:
  NodeWithPublisher() : Node("as2_mc_input_example") {
    mc_Timer::init(this, this);
    out_var.init(this);
  }

  mc_OutputVariableByTopic<0, TheNaming, geometry_msgs::msg::Pose2D> out_var;
    
  void timer_tick() override 
  {
      static int i = 0;
      out_var->x = 2.0*i;
      out_var->y = 3.0*i;
      out_var->theta = -4.0*i;
      RCLCPP_INFO(this->get_logger(), "Publishing: %d", i);
      out_var.publish();
      ++i;
  }
};

} } // namespace