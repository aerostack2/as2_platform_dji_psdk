
#ifndef DETAILS__SERVICE_CLIENT_HPP_
#define DETAILS__SERVICE_CLIENT_HPP_

#include <memory>
#include <chrono>
using namespace std::chrono_literals;

namespace as2_platform_dji_psdk
{

template<typename ServiceDefinition>
class ServiceClient
{
  using Msg_t = typename ServiceDefinition::Msg_t;

public:
  void init(rclcpp::Node * node) {client = node->create_client<Msg_t>(ServiceDefinition::name);}

  bool wait_for_service()
  {
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "service `%s' is not available, waiting for 1s again...",
        ServiceDefinition::name.c_str());
    }
    return true;
  }

  using ServiceResponseFuture = typename rclcpp::Client<Msg_t>::SharedFutureWithRequest;

  void response_cb(ServiceResponseFuture future)
  {
    auto request_response_pair = future.get();
    RCLCPP_INFO(
      this->get_logger(), "Result of %d is %d (%s)",
      (int)request_response_pair.first->data, (int)request_response_pair.second->success,
      request_response_pair.second->message.c_str());
  }

  using Request_t = typename Msg_t::Request;
  void callAsyncServer(std::shared_ptr<Request_t> request)
  {
    client->prune_pending_requests();
    auto callback = [this](ServiceResponseFuture future) {
        auto request_response_pair = future.get();
        RCLCPP_INFO(
          this->get_logger(), "Result of %d is %d (%s)",
          (int)request_response_pair.first->data,
          (int)request_response_pair.second->success,
          request_response_pair.second->message.c_str());
      };
    auto future_id = client->async_send_request(request, callback);
  }
  using ClientPtr_t = typename rclcpp::Client<Msg_t>::SharedPtr;
  ClientPtr_t client;
};

}  // namespace as2_platform_dji_psdk

#endif  // DETAILS__SERVICE_CLIENT_HPP_
