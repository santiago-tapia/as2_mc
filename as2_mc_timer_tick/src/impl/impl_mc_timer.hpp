
#ifndef AEROSTACK2_MODULAR_CLASS___IMPL_MC_TIMER_HPP
#define AEROSTACK2_MODULAR_CLASS___IMPL_MC_TIMER_HPP

#include "rclcpp/rclcpp.hpp"

namespace as2 { namespace mc {

class impl_mc_Timer {
public:
    rclcpp::TimerBase::SharedPtr timer_;
};

} } // namespace

#endif