#ifndef AEROSTACK2_MODULAR_CLASS___MC_STD_MSGS_STRING_HPP_182309488798419823
#define AEROSTACK2_MODULAR_CLASS___MC_STD_MSGS_STRING_HPP_182309488798419823

#include "as2_mc_output/mc_output_var.hpp"
#include "std_msgs/msg/string.hpp"

namespace as2::mc {

class mc_Std_Msgs_String : public mc_OutputVar<std_msgs::msg::String>
{
public:
    const std::string& topic;
    mc_Std_Msgs_String(const std::string&);
    virtual void init(rclcpp::Node*) override;
};

} // namespace

#endif