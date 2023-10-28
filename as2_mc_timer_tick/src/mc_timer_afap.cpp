
#include <chrono>
using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"

#include "as2_mc_timer_tick/i_timer_tick.hpp"
#include "as2_mc_timer_tick/mc_timer_afap.hpp"
#include "impl/impl_mc_timer_afap.hpp"

namespace as2 { namespace mc {

void mc_Timer_Afap::init(rclcpp::Node *node, i_TimerTick* itt) 
{
    RCLCPP_INFO(node->get_logger(), "mc_Timer_Afap::init");
    impl = std::make_shared<impl_mc_Timer_Afap>();
    impl->i_timer_tick = itt;
    impl->node = node;
    RCLCPP_INFO(node->get_logger(), "mc_Timer_Afap::init Creating timer");
    //TODO: get "sleep" period from node parameters
    // The sleeping time is the 10ms (hardcore) that is the period from Reset to Cancelling
    impl->timer_ = node->create_wall_timer(10ms, std::bind(&impl_mc_Timer_Afap::timer_tick, impl.get()));
}

} } // namespace
