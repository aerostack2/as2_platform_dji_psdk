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

#include "timer_with_rate.hpp"

#include "rclcpp/rclcpp.hpp"

namespace as2_platform_dji_psdk
{

void TimerWithRate::init(rclcpp::Node *node) {
  float cmd_freq_ = 100.0;
  try {
      node->get_parameter("cmd_freq", cmd_freq_); // Set if parameter is declared
  } catch (...) { }
  rate = std::chrono::duration<double>(1.0f / cmd_freq_);
  timer_ = node->create_wall_timer(rate, std::bind(&TimerWithRate::tick_rate, this));
}

void TimerWithRate::tick_rate() {
  // TODO(stapia): Review first call to TimerWithRate::tick_rate()
  this->timer_tick();  // Call the actual tick (overriden virtual)
  // Check if time constrains are fulfilled
  rclcpp::Time now = this->now();
  // Compute time to next tick
  // TODO(stapia): is this rclcpp::TimerBase::time_until_trigger() < 0?
  auto next_tick = last_ + period_;
  if ( now > next_tick ) {
    RCLCPP_CRITICAL(this->get_logger(), "Time requirement not fulfilled");
  }
}

}  // namespace as2_platform_dji_psdk

#endif  // DETAILS__TIMER_WITH_RATE_HPP_
