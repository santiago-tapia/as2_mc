
#include "impl_mc_timer_afap.hpp"
#include "as2_mc_timer_tick/i_timer_tick.hpp"

using namespace as2::mc;

void impl_mc_Timer_Afap::timer_tick() 
{
    RCLCPP_INFO(this->node->get_logger(), "Canceling timer");
    timer_->cancel();
    i_timer_tick->timer_tick();
    RCLCPP_INFO(this->node->get_logger(), "Reset timer");
    timer_->reset();
}

