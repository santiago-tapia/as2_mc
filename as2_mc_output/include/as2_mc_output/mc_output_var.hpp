
#ifndef AEROSTACK2_MODULAR_CLASS___MC_OUTPUT_VAR_HPP_425345245
#define AEROSTACK2_MODULAR_CLASS___MC_OUTPUT_VAR_HPP_425345245

#include "rclcpp/rclcpp.hpp"

#include "as2_mc_output/mc_base.hpp"

namespace as2 { namespace mc {

template <typename Msg>
class mc_OutputVar : public mc_Base
{
public:
    virtual ~mc_OutputVar() {}

    using Msg_t = Msg;
    using PublisherSharedPtr = typename rclcpp::Publisher<Msg_t>::SharedPtr;

    void publish() const
    {
        publisher_->publish(state_variable);
    }

    Msg_t* operator->() 
    {
        return &state_variable;
    }

protected:
    Msg_t state_variable;
    PublisherSharedPtr publisher_;
    
};

} } // namespace

#endif