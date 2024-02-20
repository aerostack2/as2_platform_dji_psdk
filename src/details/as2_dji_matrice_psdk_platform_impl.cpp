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

#include "as2_dji_matrice_psdk_platform_impl.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2_platform_dji_psdk
{

DJIMatricePSDKPlatform_impl::DJIMatricePSDKPlatform_impl(as2::Node * node)
: turnOnMotorsService(TurnOnMotors::name, node),
  turnOffMotorsService(Takeoff::name, node),
  takeoffService(Takeoff::name, node),
  landService(Land::name, node),
  setLocalPositionService(SetLocalPositionService::name, node),
  obtainCtrlAuthorityService(ObtainCtrlAuthorityService::name, node),
  cameraSetupStreamingService(CameraSetupStreamingService::name, node),
  gimbalResetService(GimbalResetService::name, node),
  odom_sensor_("odom", node),
  node_(node)
{
  velocityCommand.msg() = sensor_msgs::msg::Joy();
  velocityCommand.msg().axes.resize(4);
  velocityCommand.msg().buttons.resize(2);
}

void DJIMatricePSDKPlatform_impl::init(rclcpp::Node * node)
{
  velocityCommand.init(node);
  gimbalCommand.init(node);
  gimbalStatus.init(node);

  // Subscribe to the odometry topic
  position_fused_sub_ = node->create_subscription<psdk_interfaces::msg::PositionFused>(
    "psdk_ros2/position_fused", 10,
    std::bind(&DJIMatricePSDKPlatform_impl::position_fused_callback, this, std::placeholders::_1));

  // Subscribe to the attitude topic
  attitude_sub_ = node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "psdk_ros2/attitude", 10,
    std::bind(&DJIMatricePSDKPlatform_impl::attitude_callback, this, std::placeholders::_1));

  velocity_sub_ = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "psdk_ros2/velocity_ground_fused", 10,
    std::bind(&DJIMatricePSDKPlatform_impl::velocity_callback, this, std::placeholders::_1));

  angular_velocity_sub_ = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "psdk_ros2/angular_rate_body_raw", 10,
    std::bind(
      &DJIMatricePSDKPlatform_impl::angular_velocity_callback, this, std::placeholders::_1));

  gimbal_control_sub_ = node->create_subscription<as2_msgs::msg::GimbalControl>(
    "platform/gimbal/gimbal_command", 10,
    std::bind(&DJIMatricePSDKPlatform_impl::gimbal_control_callback, this, std::placeholders::_1));

  // Initialize the camera streaming service
  node->declare_parameter<bool>("enbale_camera", false);
  bool enable_camera;
  node->get_parameter("enbale_camera", enable_camera);
  RCLCPP_INFO(
    node->get_logger(), "Camera streaming service: %s", enable_camera ? "enabled" : "disabled");
  if (enable_camera) {
    auto request = std::make_shared<psdk_interfaces::srv::CameraSetupStreaming::Request>();
    request->payload_index = 1;
    request->camera_source = 0;
    request->start_stop = 1;
    auto response = std::make_shared<psdk_interfaces::srv::CameraSetupStreaming::Response>();
    bool sr = cameraSetupStreamingService.sendRequest(request, response);
    bool success = sr && response->success;
    if (!success) {
      RCLCPP_INFO(node->get_logger(), "Camera streaming service failed");
    }
  }

  // Initialize the gimbal service
  node->declare_parameter<bool>("enable_gimbal", false);
  bool enable_gimbal;
  node->get_parameter("enable_gimbal", enable_gimbal);
  RCLCPP_INFO(
    node->get_logger(), "Gimbal reset service: %s", enable_gimbal ? "enabled" : "disabled");
  if (enable_gimbal) {
    auto request = std::make_shared<psdk_interfaces::srv::GimbalReset::Request>();
    request->payload_index = 1;
    request->reset_mode = 1;
    auto response = std::make_shared<psdk_interfaces::srv::GimbalReset::Response>();
    bool sr = gimbalResetService.sendRequest(request, response);
    bool success = sr && response->success;
    if (!success) {
      RCLCPP_INFO(node->get_logger(), "Gimbal reset service failed");
    }
  }

  RCLCPP_INFO(node->get_logger(), "DJIMatricePSDKPlatform_impl initialized");
}

void DJIMatricePSDKPlatform_impl::position_fused_callback(
  const psdk_interfaces::msg::PositionFused::SharedPtr msg)
{
  position_fused_msg_ = *msg.get();

  // Update the odometry sensor
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp = node_->now();
  odom_msg.header.frame_id = as2::tf::generateTfName(node_->get_namespace(), "odom");
  odom_msg.child_frame_id = as2::tf::generateTfName(node_->get_namespace(), "base_link");
  odom_msg.pose.pose.position.x = position_fused_msg_.position.x;
  odom_msg.pose.pose.position.y = position_fused_msg_.position.y;
  odom_msg.pose.pose.position.z = position_fused_msg_.position.z;
  odom_msg.pose.pose.orientation.x = attitude_msg_.quaternion.x;
  odom_msg.pose.pose.orientation.y = attitude_msg_.quaternion.y;
  odom_msg.pose.pose.orientation.z = attitude_msg_.quaternion.z;
  odom_msg.pose.pose.orientation.w = attitude_msg_.quaternion.w;

  // convert ENU to FLU
  Eigen::Vector3d vel_ENU =
    Eigen::Vector3d(velocity_msg_.vector.x, velocity_msg_.vector.y, velocity_msg_.vector.z);
  auto flu_speed = as2::frame::transform(odom_msg.pose.pose.orientation, vel_ENU);
  odom_msg.twist.twist.linear.x = flu_speed.x();
  odom_msg.twist.twist.linear.y = flu_speed.y();
  odom_msg.twist.twist.linear.z = flu_speed.z();

  odom_msg.twist.twist.angular.x = angular_velocity_msg_.vector.x;
  odom_msg.twist.twist.angular.y = angular_velocity_msg_.vector.y;
  odom_msg.twist.twist.angular.z = angular_velocity_msg_.vector.z;

  // RCLCPP_INFO(node_->get_logger(), "Position: %f, %f, %f", position_fused_msg_.position.x, position_fused_msg_.position.y, position_fused_msg_.position.z);
  odom_sensor_.updateData(odom_msg);
}

void DJIMatricePSDKPlatform_impl::attitude_callback(
  const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  attitude_msg_ = *msg.get();
}

void DJIMatricePSDKPlatform_impl::velocity_callback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  velocity_msg_ = *msg.get();
}

void DJIMatricePSDKPlatform_impl::angular_velocity_callback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  angular_velocity_msg_ = *msg.get();
}

void DJIMatricePSDKPlatform_impl::gimbal_control_callback(
  const as2_msgs::msg::GimbalControl::SharedPtr msg)
{
  // gimbal angle in the wrapper is at PSDKWrapper::gimbal_rotation_cb in gimbal.cpp
  // pitch and yaw are changed in their sign
  RCLCPP_INFO(
    node_->get_logger(), "Gimbal angle: %f %f %f", msg->control.vector.x, msg->control.vector.y,
    msg->control.vector.z);
  auto & out = gimbalCommand.msg();
  out.payload_index = 1;
  out.rotation_mode = 1;  // Incremental
  out.time = 0.5;         // In seconds, expected time to target rotation

  double desired_roll = msg->control.vector.x;
  double desired_pitch = msg->control.vector.y;
  double desired_yaw = msg->control.vector.z;

  double current_roll = gimbal_attitude_.x;
  double current_pitch = gimbal_attitude_.y;
  double current_yaw = gimbal_attitude_.z;

  if (
    desired_roll == current_roll && desired_pitch == current_pitch && desired_yaw == current_yaw) {
    return;
  }
  out.roll = desired_roll - current_roll;
  out.pitch = desired_pitch - current_pitch;
  out.yaw = desired_yaw - current_yaw;

  gimbalCommand.publish();

  // Update the gimbal attitude
  auto & gimbal_status = gimbalStatus.msg();
  gimbal_status.header.stamp = node_->now();
  gimbal_status.header.frame_id = as2::tf::generateTfName(node_->get_namespace(), "gimbal");
  geometry_msgs::msg::Quaternion q;
  as2::frame::eulerToQuaternion(desired_roll, desired_pitch, desired_yaw, q);
  gimbal_status.quaternion = q;
  gimbalStatus.publish();
}

}  // namespace as2_platform_dji_psdk
