
#include <memory>
#include <vector>

#include "as2_mc_output/mc_typed_names.hpp"
#include "as2_mc_output/mc_std_msgs_string.hpp"
#include "as2_mc_timer_tick/mc_timer_afap.hpp"
#include "as2_mc_timer_tick/i_timer_tick.hpp"

using namespace as2::mc;

class node_with_output_state : public rclcpp::Node, public mc_Timer_Afap, public i_TimerTick
{
    using mc_BasePtr = std::shared_ptr<mc_OutputBase>;
    using VarVector = std::vector<mc_BasePtr>;

    std::shared_ptr<mc_Std_Msgs_String> n1;
    std::shared_ptr<mc_Std_Msgs_String> n2;
    VarVector state;

public:
    node_with_output_state() : 
        rclcpp::Node("node_with_output_state"),
        n1(as2::mc::topics::global::TypedNotificationOne::create()), 
        n2(as2::mc::topics::global::TypedNotificationTwo::create()),
        state{ n1, n2 }
    {
        for ( auto out : state ) out->init(this);
        mc_Timer_Afap::init(this, this);
    }

    void timer_tick() override 
    {
        RCLCPP_INFO(this->get_logger(), "node_with_output_state::timer_tick");
        static int i = 0;
        std::ostringstream ss;
        ss << "Counter in Notification 1: " << i;
        n1->value()->data = ss.str();
        n2->value()->data = "Notification two";
        for ( auto out : state ) out->publish();
        ++i;
    }
};
