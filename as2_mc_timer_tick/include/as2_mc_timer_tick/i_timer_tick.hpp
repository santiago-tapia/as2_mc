
#ifndef AEROSTACK2_MODULAR_CLASS_INTERFACE_TIMER_TICK_HPP
#define AEROSTACK2_MODULAR_CLASS_INTERFACE_TIMER_TICK_HPP

namespace as2 { namespace mc {

class i_TimerTick 
{
public:
    virtual ~i_TimerTick() {}
    virtual void timer_tick() = 0;
};

} } // namespace

#endif