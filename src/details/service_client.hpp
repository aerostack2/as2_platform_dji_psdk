// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef DETAILS__SERVICE_CLIENT_HPP_
#define DETAILS__SERVICE_CLIENT_HPP_

#include <memory>
#include <chrono>

namespace as2_platform_dji_psdk
{

template<typename ServiceDefinition>
class ServiceClient
{
  typedef typename ServiceDefinition::Msg_t Msg_t;

public:
  void init(rclcpp::Node * node) {client = node->create_client<Msg_t>(ServiceDefinition::name);}

  bool wait_for_service()
  {
    while (!client->wait_for_service(std::chrono::seconds(1))) {
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