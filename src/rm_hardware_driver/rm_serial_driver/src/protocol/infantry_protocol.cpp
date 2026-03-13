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

#include "rm_serial_driver/protocol/infantry_protocol.hpp"
#include <stdlib.h>

namespace fyt::serial_driver::protocol {
ProtocolInfantry::ProtocolInfantry(std::string_view port_name, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  packet_tool_ = std::make_shared<FixedPacketTool<64>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void ProtocolInfantry::send(const rm_interfaces::msg::GimbalCmd &data) {

  try{
    packet.loadData<unsigned char>(data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
    packet.loadData<float>(static_cast<float>(data.pitch), 2);
    packet.loadData<float>(static_cast<float>(data.yaw), 6);
    packet.loadData<float>(static_cast<float>(data.distance), 10);
    int target_id;
    if (data.target_id == "outpost")target_id = 8;
    else target_id = 4; // default target id
    packet.loadData<int>(target_id, 14);
  }
  catch(const std::invalid_argument &e){
    FYT_ERROR("serial_driver", "auto_aim_invalid_argument");
  }
  packet_tool_->sendPacket(packet);
}


void ProtocolInfantry::send(const rm_interfaces::msg::Blind &data) {
  if (data.is_left && data.yaw>0){
    try{
      packet.loadData<int>(static_cast<int>(std::stoi(data.number.substr(0,1))), 18);
      packet.loadData<float>(static_cast<float>(data.yaw),22);
    }
    catch(const std::invalid_argument &e){
      FYT_ERROR("serial_driver","left_blind_invalid_argument");
      packet.loadData<int>(static_cast<int>(-1), 18);
    }
  }
  else if (data.is_left){
    packet.loadData<int>(static_cast<int>(-1), 18);
  }
  if (!data.is_left && data.yaw<0)
  {
    try{
      packet.loadData<int>(static_cast<int>(std::stoi(data.number.substr(0,1))), 26);
      packet.loadData<float>(static_cast<float>(data.yaw),30);
    }
    catch(const std::invalid_argument &e){
      FYT_ERROR("serial_driver","right_blind_invalid_argument");
      packet.loadData<int>(static_cast<int>(-1), 26);
    }
  }
  else if (!data.is_left){
    packet.loadData<int>(static_cast<int>(-1), 26);
  }
  
  packet_tool_->sendPacket(packet);
}

void ProtocolInfantry::send(const geometry_msgs::msg::Twist &data) {
  // packet_.loadData<unsigned char>(0x00, 1);
  // is_spin
                  ///////////changed here ////////////
        // packet_.loadData<unsigned char>(data.is_spining ? 0x01 : 0x00, 2);
        // packet_.loadData<unsigned char>(data.is_navigating ? 0x01 : 0x00, 3);
                  ///////////changed here ////////////
  // gimbal control
  // packet_.loadData<float>(0, 4);
  // packet_.loadData<float>(0, 8);
  // packet_.loadData<float>(0, 12);
  // chassis control
  // linear x
  packet.loadData<float>(data.linear.x, 34);
  // linear y
  packet.loadData<float>(data.linear.y, 38);
  // angular z
  packet.loadData<float>(data.angular.z, 42);

  packet_tool_->sendPacket(packet);
}

void ProtocolInfantry::send(const std_msgs::msg::Float64 &data) {

  packet.loadData<float>(data.data, 46);

  packet_tool_->sendPacket(packet);
}

void ProtocolInfantry::send(const std_msgs::msg::Bool &data) {

  packet.loadData<unsigned char>(data.data, 50);
  packet_tool_->sendPacket(packet);
}


void ProtocolInfantry::send1(const std_msgs::msg::Float64 &data) {

  packet.loadData<float>(data.data, 51);

  packet_tool_->sendPacket(packet);
}

void ProtocolInfantry::send1(const std_msgs::msg::Bool &data) {

  packet.loadData<unsigned char>(data.data, 55);
  packet_tool_->sendPacket(packet);
}

bool ProtocolInfantry::receive(rm_interfaces::msg::SerialReceiveData &data) {
  FixedPacket<64> packet;
  if (packet_tool_->recvPacket(packet)) {
    packet.unloadData(data.mode, 1);
    packet.unloadData(data.roll, 2);
    packet.unloadData(data.pitch, 6);
    packet.unloadData(data.yaw, 10);
    packet.unloadData(data.game_status,14);
    packet.unloadData(data.remaining_time,15);
    packet.unloadData(data.blood,17);
    packet.unloadData(data.fire_count_enough,19); //由whether2occupy修改为读取发弹量
    packet.unloadData(data.whether2cruise,20);
    packet.unloadData(data.outpost_hp,21);

    //////////////////  added and change here //////////////////////
    /////navigation data
    // packet.unloadData(data.progress, 14);
    // packet.unloadData(data.outpostHp, 15);
    // packet.unloadData(data.targetX, 17);
    // packet.unloadData(data.targetY, 21);
    // packet.unloadData(data.fornothing, 32);  //占位符
    //////////////////  added and change here //////////////////////
    return true;
  } else {
    return false;
  }
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolInfantry::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  auto sub1 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  //////////////////  added and change here //////////////////////
  auto sub3 = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->send(*msg); });
  /*auto sub2 = node->create_subscription<rm_interfaces::msg::Blind>(
    "blind_detector/left/blind",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::Blind::SharedPtr msg){ this->send(*msg); });

  auto sub4 = node->create_subscription<rm_interfaces::msg::Blind>(
    "blind_detector/right/blind",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::Blind::SharedPtr msg){ this->send(*msg); });*/
    
  //自身与狗洞夹角
  auto sub5 = node->create_subscription<std_msgs::msg::Float64>(
    "/dogHole_angle_difference",
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr msg) { this->send(*msg); });
  //是否达到狗洞前
  auto sub6 = node->create_subscription<std_msgs::msg::Bool>(
    "/ifClimb",
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) { this->send(*msg); });
  //自身与前哨站夹角
  auto sub7 = node->create_subscription<std_msgs::msg::Float64>(
    "/outpost_angle_difference",
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr msg) { this->send1(*msg); });
  //是否到达攻击前哨站位置
  auto sub8 = node->create_subscription<std_msgs::msg::Bool>(
    "/if_attack_outpost",
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) { this->send1(*msg); });
  //return {sub1, sub2, sub3, sub4, sub5, sub6, sub7, sub8};
  return {sub1, sub3, sub5, sub6, sub7, sub8};
  //return {sub1, sub3, sub5, sub6, sub7, sub8};
  //////////////////  added and change here //////////////////////
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolInfantry::getClients(
  rclcpp::Node::SharedPtr node) const {
  auto client1 = node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client2 = node->create_client<rm_interfaces::srv::SetMode>("gimbal_pipeline/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client3 = node->create_client<rm_interfaces::srv::SetMode>("left/blind_detector/set_mode", rmw_qos_profile_services_default);  //补盲
  //auto client4 = node->create_client<rm_interfaces::srv::SetMode>("right/blind_detector/set_mode", rmw_qos_profile_services_default);  //补盲
  //return {client1, client2};
  return {client1, client2};  //补盲
}

}  // namespace fyt::serial_driver::protocol
