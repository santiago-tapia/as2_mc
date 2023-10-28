
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "as2_mc_timer_tick/mc_timer.hpp"
#include "as2_mc_timer_tick/i_timer_tick.hpp"

using namespace as2::mc;

class TimerTickNode: public rclcpp::Node, public mc_Timer, public i_TimerTick
{
public:
  TimerTickNode() : rclcpp::Node("timer_tick")
  {
    mc_Timer::init(this, this);
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