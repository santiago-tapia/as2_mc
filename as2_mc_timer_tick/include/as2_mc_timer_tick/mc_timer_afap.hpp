
#ifndef AEROSTACK2_MODULAR_CLASS___MC_TIMER_AFAP_HPP_894321794
#define AEROSTACK2_MODULAR_CLASS___MC_TIMER_AFAP_HPP_894321794

#include <memory>
namespace rclcpp { class Node; }

namespace as2 { namespace mc {

class i_TimerTick;
class impl_mc_Timer_Afap;

class mc_Timer_Afap
{
public:
    void init(rclcpp::Node*, i_TimerTick*);

    std::shared_ptr<impl_mc_Timer_Afap> impl;
};

} } // namespace

#endif