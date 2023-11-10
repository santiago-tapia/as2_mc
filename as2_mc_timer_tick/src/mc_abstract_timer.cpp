
#include "as2_mc_timer_tick/mc_abstrac_timer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2 { namespace mc {

void mc_Abstract_Timer::init(rclcpp::Node *node) 
{
    float cmd_freq_ = 100.0;
    try {
        node->get_parameter("cmd_freq", cmd_freq_); // Set if parameter is declared
    } catch (...) { }
    auto rate = std::chrono::duration<double>(1.0f / cmd_freq_);
    timer_ = node->create_wall_timer(rate, std::bind(&mc_Abstract_Timer::timer_tick, this));
}

} } // namespace
