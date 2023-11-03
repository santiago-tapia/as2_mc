#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "as2_mc_timer_tick/mc_timer.hpp"
#include "as2_mc_timer_tick/i_timer_tick.hpp"

#include <chrono>
using namespace std::chrono_literals;

using namespace as2::mc;

class ToggleService: public rclcpp::Node, public mc_Timer, public i_TimerTick
{
public:
  ToggleService() : rclcpp::Node("toggle")
  {
    mc_Timer::init(this, this);
    client = this->create_client<std_srvs::srv::SetBool>("toggle");
    request = std::make_shared<std_srvs::srv::SetBool::Request>();
  }

  int wait_for_service() 
  {
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    return 1;
  }

  using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFutureWithRequest;

  void response_callback(ServiceResponseFuture future) 
  {
    auto request_response_pair = future.get();
    RCLCPP_INFO(this->get_logger(),"Result of %d is %d (%s)",
        (int)request_response_pair.first->data,
        (int)request_response_pair.second->success,
        request_response_pair.second->message.c_str());
  }
  
  void timer_tick() override 
  {
    static int i = 0;
    if ( i % 10 == 0 ) 
    {
        client->prune_pending_requests();
        request->data = !request->data;
        // auto callback = std::bind(&ToggleService::response_callback, this);
        auto callback = [this](ServiceResponseFuture future) {
            auto request_response_pair = future.get();
            RCLCPP_INFO(this->get_logger(),"Result of %d is %d (%s)",
                (int)request_response_pair.first->data,
                (int)request_response_pair.second->success,
                request_response_pair.second->message.c_str());
                };        
        auto future_id = client->async_send_request(request, callback);
    }
    ++i;
  }
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client;
  std::shared_ptr<std_srvs::srv::SetBool::Request> request;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToggleService>());
  rclcpp::shutdown();
  return 0;
}
