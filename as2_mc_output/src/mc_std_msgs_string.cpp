
#include "as2_mc_output/mc_std_msgs_string.hpp"

namespace as2::mc {

mc_Std_Msgs_String::mc_Std_Msgs_String(const std::string& name) : topic(name)
{ 
}

void mc_Std_Msgs_String::init(rclcpp::Node* node)
{
    publisher_ = node->create_publisher<Msg_t>(topic, 10);
}

} // namespace