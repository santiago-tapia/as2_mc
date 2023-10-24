
#ifndef AEROSTACK2_MODULAR_CLASS___MC_INPUT_VARIABLE_HPP
#define AEROSTACK2_MODULAR_CLASS___MC_INPUT_VARIABLE_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

namespace as2 { namespace mc {

template <typename Id, typename Msg>
class mc_InputVariable
{
public:
    using SubscriptionSharedPtr = typename rclcpp::Subscription<Msg>::SharedPtr;

    void init(rclcpp::Node* node) 
    {
        auto f = std::bind(&mc_InputVariable::callback, this, _1);
        subscription_ = node->create_subscription<Msg>(Id::topic(), 10, f);
    }


    Msg* operator->() {
        return &state_variable;
    }

protected:
    Msg state_variable;
    SubscriptionSharedPtr subscription_;
    void callback(const Msg & msg) 
    {
        state_variable = msg; 
    }
};

} } // namespace

#endif