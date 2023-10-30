
#ifndef AEROSTACK2_MODULAR_CLASS___MC_OUTPUT_VARIABLE_BY_TOPIC_HPP_91874190479234HKAS
#define AEROSTACK2_MODULAR_CLASS___MC_OUTPUT_VARIABLE_BY_TOPIC_HPP_91874190479234HKAS

#include "rclcpp/rclcpp.hpp"

namespace as2 { namespace mc {

template <int Id, typename Naming, typename Msg>
class mc_OutputVariableByTopic
{
public:
    using PublisherSharedPtr = typename rclcpp::Publisher<Msg>::SharedPtr;

    void init(rclcpp::Node* node) 
    {
        publisher_ = node->create_publisher<Msg>(Naming::name(Id), 10);
    }

    void publish() const
    {
        publisher_->publish(state_variable);
    }

    Msg* operator->() 
    {
        return &state_variable;
    }

protected:
    Msg state_variable;
    PublisherSharedPtr publisher_;
    
};

} } // namespace

#endif