
#ifndef AEROSTACK2_MODULAR_CLASS___IMPL_MC_TIMER_AFAP_HPP_53249857
#define AEROSTACK2_MODULAR_CLASS___IMPL_MC_TIMER_AFAP_HPP_53249857

#include "rclcpp/rclcpp.hpp"

namespace as2 { namespace mc {

class i_TimerTick;

class impl_mc_Timer_Afap {
public:
    rclcpp::Node *node;
    i_TimerTick* i_timer_tick;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_tick();
};

} } // namespace

#endif