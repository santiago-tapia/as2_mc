
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

namespace as2 { namespace mc {

class NodeWithPublisher : public rclcpp::Node
{
public:
  NodeWithPublisher() : Node("as2_mc_input_example") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/as2_mc/example/pose", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&NodeWithPublisher::publish, this));
  }
    
  void publish()
  {
      static int i = 0;
      auto message = geometry_msgs::msg::Pose2D();
      message.x = 2.0*i;
      message.y = 3.0*i;
      message.theta = -4.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: %d", i);
      publisher_->publish(message);
      ++i;
  }
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} } // namespace