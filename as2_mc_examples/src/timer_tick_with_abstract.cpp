
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "as2_mc_timer_tick/mc_abstrac_timer.hpp"

using namespace as2::mc;

class TimerTickNode: public rclcpp::Node, public mc_Abstract_Timer
{
public:
  TimerTickNode() : rclcpp::Node("timer_tick")
  {
    this->declare_parameter<float>("cmd_freq", 1.0);
    mc_Abstract_Timer::init(this);
  }
  
  void timer_tick() override 
  {
    RCLCPP_INFO(this->get_logger(), "timer_tick");
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerTickNode>());
  rclcpp::shutdown();
  return 0;
}