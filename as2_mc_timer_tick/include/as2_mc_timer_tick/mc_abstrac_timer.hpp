
#ifndef AEROSTACK2_MODULAR_CLASS___MC_ABSTRACT_TIMER_HPP_981275319HQ341
#define AEROSTACK2_MODULAR_CLASS___MC_ABSTRACT_TIMER_HPP_981275319HQ341

#include <memory>
namespace rclcpp { 
    class Node; 
    class TimerBase;
}

#include "as2_mc_timer_tick/i_timer_tick.hpp"

namespace as2 { namespace mc {

class mc_Abstract_Timer : public i_TimerTick
{
public:
    void init(rclcpp::Node*);
protected:
    std::shared_ptr<rclcpp::TimerBase> timer_;
};

} } // namespace

#endif