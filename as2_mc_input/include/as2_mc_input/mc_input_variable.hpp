
#ifndef AEROSTACK2_MODULAR_CLASS___MC_TIMER_HPP
#define AEROSTACK2_MODULAR_CLASS___MC_TIMER_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

namespace as2 { namespace mc {

template <typename Id, typename Msg>
class mc_InputVariable
{
public:

    void init(rclcpp::Node* node) 
    {
        subscription_ = node->create_subscription<Msg>(Id::topic(), 10, std::bind(&mc_InputVariable::callback, this, _1));
    }


    Msg* operator->() const {
        return &state_variable;
    }

protected:
    Msg state_variable;
    rclcpp::Subscription<typename Msg>::SharedPtr subscription_;
    void callback(const Msg & msg) 
    {
        state_variable = msg; 
    }
};

} } // namespace

#endif