#ifndef AEROSTACK2_MODULAR_CLASS___MC_BASE_CLASS_HPP_23058923H5I3UH3W4875Y
#define AEROSTACK2_MODULAR_CLASS___MC_BASE_CLASS_HPP_23058923H5I3UH3W4875Y

namespace rclcpp { class Node; }

namespace as2 { namespace mc {

class mc_Base 
{
public:
    virtual ~mc_Base() {}
    virtual void init(rclcpp::Node*) = 0;
};

} } // namespace

#endif