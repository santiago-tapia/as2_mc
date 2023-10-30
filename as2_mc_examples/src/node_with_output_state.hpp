
#include <memory>
#include <vector>

#include "as2_mc_output/mc_typed_names.hpp"

using namespace as2::mc;

class node_with_output_state : public rclcpp::Node
{
    using mc_BasePtr = std::shared_ptr<mc_Base>;
    using VarVector = std::vector<mc_BasePtr>;
    VarVector state;
    std::shared_ptr<mc_Std_Msgs_String> n1;
    std::shared_ptr<mc_Std_Msgs_String> n2;

public:
    node_with_output_state() : 
        rclcpp::Node("node_with_output_state"),
        n1(as2::mc::topics::global::TypedNotificationOne::create()), 
        n2(as2::mc::topics::global::TypedNotificationOne::create()),
        state{ n1, n2 }
    {
        for ( auto out : state ) out->init(this);
    }
};
