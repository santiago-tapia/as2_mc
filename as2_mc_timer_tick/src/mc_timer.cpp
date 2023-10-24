
#include <chrono>
using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"

#include "as2_mc_timer_tick/i_timer_tick.hpp"
#include "as2_mc_timer_tick/mc_timer.hpp"
#include "impl/impl_mc_timer.hpp"

namespace as2 { namespace mc {

void mc_Timer::init(rclcpp::Node *node, i_TimerTick* itt) 
{
    impl = std::make_shared<impl_mc_Timer>();

    //TODO: get period from node parameters
    impl->timer_ = node->create_wall_timer(500ms, std::bind(&i_TimerTick::timer_tick, itt));
}

} } // namespace
