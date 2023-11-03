
#ifndef AEROSTACK2_MODULAR_CLASS___MC_TYPED_NAMES_HPP_12384798237491Y4HIU2Q43
#define AEROSTACK2_MODULAR_CLASS___MC_TYPED_NAMES_HPP_12384798237491Y4HIU2Q43

#include "rclcpp/rclcpp.hpp"
#include "as2_mc_output/mc_std_msgs_string.hpp"

namespace as2 { namespace mc { 

template <typename MSG, typename Name, typename VarClass>
class mc_TypedNames
{
public:
    using MsgType = MSG;
    static std::shared_ptr<VarClass> create()
    {
        return std::make_shared<VarClass>(Name::name);
    }
};

namespace topics {
    namespace global 
    {
        const rclcpp::QoS qos         = rclcpp::QoS(10);
        const std::string alert_event = "alert_event";

        struct NotificationOne { inline static const std::string name = "notification_one"; };
        class TypedNotificationOne : public mc_TypedNames<std_msgs::msg::String, NotificationOne, mc_Std_Msgs_String> {};
        
        struct NotificationTwo { inline static const std::string name = "notification_two"; };
        class TypedNotificationTwo : public mc_TypedNames<std_msgs::msg::String, NotificationTwo, mc_Std_Msgs_String> {};

    }  // namespace global
    /* 
    namespace sensor_measurements 
    {
        const rclcpp::QoS qos     = rclcpp::SensorDataQoS();
        const std::string base    = "sensor_measurements:";
        const std::string imu     = base + "imu";
        const std::string lidar   = base + "lidar";
        const std::string gps     = base + "gps";
        const std::string camera  = base + "camera";
        const std::string battery = base + "battery";
        const std::string odom    = base + "odom";
    }  // namespace sensor_measurements
    */
} } }

#endif