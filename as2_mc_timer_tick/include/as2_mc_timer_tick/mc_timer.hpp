
#ifndef AEROSTACK2_MODULAR_CLASS___MC_TIMER_HPP
#define AEROSTACK2_MODULAR_CLASS___MC_TIMER_HPP

#include <memory>
namespace rclcpp { class Node; }

namespace as2 { namespace mc {

class i_TimerTick;
class impl_mc_Timer;

class mc_Timer
{
public:
    void init(rclcpp::Node*, i_TimerTick*);

    std::shared_ptr<impl_mc_Timer> impl;
};

} } // namespace

#endif