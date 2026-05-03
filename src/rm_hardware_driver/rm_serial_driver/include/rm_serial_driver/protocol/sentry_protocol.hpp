// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SERIAL_DRIVER__SENTRY_PROTOCOL_HPP_
#define SERIAL_DRIVER__SENTRY_PROTOCOL_HPP_

// project
#include "rm_serial_driver/protocol.hpp"
#include "rm_interfaces/msg/blind.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace fyt::serial_driver::protocol {
// 步兵通信协议
class ProtocolSentry : public Protocol {
public:
  explicit ProtocolSentry(std::string_view port_name, bool enable_data_print);

  ~ProtocolSentry() = default;

  bool open() override { return packet_tool_->open(); }

  void close() override { packet_tool_->close(); }

  bool isOpen() override { return packet_tool_->isOpen(); }

  void send(const rm_interfaces::msg::GimbalCmd &data) override;
  void send(const geometry_msgs::msg::Twist &data);
  void send(const rm_interfaces::msg::Blind &data);
  //发送角度
  void send(const std_msgs::msg::Float64 &data);
  //发送爬坡标志位
  void send(const std_msgs::msg::Bool &data);
  //发送前哨站角度
  void send1(const std_msgs::msg::Float64 &data);
  //发送攻击前哨站标志位
  void send1(const std_msgs::msg::Bool &data);

  bool receive(
    rm_interfaces::msg::SerialReceiveData &data,
    int64_t *receipt_time_ns = nullptr) override;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(
    rclcpp::Node::SharedPtr node) override;

  std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node::SharedPtr node) const override;

  std::string getErrorMessage() override { return packet_tool_->getErrorMessage(); }

private:
  FixedPacketTool<64>::SharedPtr packet_tool_;
  FixedPacket<64> packet;
};
}  // namespace fyt::serial_driver::protocol

#endif  // SERIAL_DRIVER_SENTRY_PROTOCOL_HPP_
