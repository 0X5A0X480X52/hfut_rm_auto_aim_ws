// Created by Chengfu Zou on 2023.7.1
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

#include "rm_serial_driver/serial_driver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <memory>
#include <thread>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "rm_serial_driver/uart_transporter.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::serial_driver {

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions &options)
: Node("serial_driver", options) {
  FYT_REGISTER_LOGGER("serial_driver", "~/fyt2024-log", INFO);
  listen_thread_ = std::make_unique<std::thread>(&SerialDriverNode::listenLoop, this);
}

void SerialDriverNode::init() {
  FYT_INFO("serial_driver", "Initializing SerialDriverNode!");

  target_frame_ = this->declare_parameter("target_frame", "odom");
  reconnect_interval_ms_ = this->declare_parameter("reconnect_interval_ms", 500);
  std::string port_name = this->declare_parameter("port_name", "/dev/ttyUSB0");
  std::string protocol_type = this->declare_parameter("protocol", "infantry");
  bool enable_data_print = this->declare_parameter("enable_data_print", false);

  protocol_ = ProtocolFactory::createProtocol(protocol_type, port_name, enable_data_print);
  if (protocol_ == nullptr) {
    FYT_FATAL("serial_driver", "Failed to create protocol with type: {}", protocol_type);
    rclcpp::shutdown();
    return;
  }
  FYT_INFO(
    "serial_driver", "Protocol has been created with type: {}, port: {}", protocol_type, port_name);

  subscriptions_ = protocol_->getSubscriptions(this->shared_from_this());
  for (auto sub : subscriptions_) {
    FYT_INFO("serial_driver", "Subscribe to topic: {}", sub->get_topic_name());
  }

  serial_receive_data_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
    "serial/receive", rclcpp::SensorDataQoS());
  wheel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("wheel_odom", 100);
  hp_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/current_hp", 100);
  bullet_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/bullet_remain", 100);
  time_remain_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/time_remain", 100);

  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  for (auto client : protocol_->getClients(this->shared_from_this())) {
    std::string name = client->get_service_name();
    set_mode_clients_.emplace(name, client);
    FYT_INFO("serial_driver", "Create client for service: {}", name);
  }

  heartbeat_ = HeartBeatPublisher::create(this);
  ensureConnection("startup");

  FYT_INFO("serial_driver", "SerialDriverNode has been initialized!");
}

SerialDriverNode::~SerialDriverNode() {
  FYT_INFO("serial_driver", "Destroy SerialDriverNode!");
  if (protocol_ != nullptr) {
    protocol_->close();
  }
  rclcpp::shutdown();
  if (listen_thread_ != nullptr) {
    listen_thread_->join();
  }
}

bool SerialDriverNode::ensureConnection(const std::string &reason) {
  if (protocol_ == nullptr) {
    return false;
  }
  if (protocol_->isOpen()) {
    return true;
  }

  while (rclcpp::ok()) {
    if (protocol_->open()) {
      FYT_INFO("serial_driver", "Serial port connected ({})", reason);
      return true;
    }

    auto error_message = protocol_->getErrorMessage();
    error_message = error_message.empty() ? "unknown" : error_message;
    FYT_WARN(
      "serial_driver",
      "Serial port unavailable ({}): {}. Retry in {} ms",
      reason,
      error_message,
      reconnect_interval_ms_);
    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms_));
  }

  return false;
}

void SerialDriverNode::listenLoop() {
  if (protocol_ == nullptr) {
    init();
  }
  if (protocol_ == nullptr || !ensureConnection("startup")) {
    return;
  }

  rm_interfaces::msg::SerialReceiveData receive_data;
  while (rclcpp::ok()) {
    int64_t packet_receipt_time_ns = 0;
    if (protocol_->receive(receive_data, &packet_receipt_time_ns)) {
      timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
      const auto packet_time_base = packet_receipt_time_ns > 0 ?
        rclcpp::Time(packet_receipt_time_ns, RCL_SYSTEM_TIME) :
        this->now();
      const auto packet_time =
        packet_time_base + rclcpp::Duration::from_seconds(timestamp_offset_);
      receive_data.header.stamp = packet_time;
      receive_data.header.frame_id = target_frame_;
      serial_receive_data_pub_->publish(receive_data);

      geometry_msgs::msg::TwistStamped twist;
      twist.header.stamp = packet_time;
      twist.header.frame_id = target_frame_;
      twist.twist.linear.x = receive_data.chassis_vx;
      twist.twist.linear.y = receive_data.chassis_vy;
      twist.twist.angular.z = receive_data.chassis_wz;

      std_msgs::msg::Int32 hp_msg;
      hp_msg.data = receive_data.blood;
      hp_publisher_->publish(hp_msg);

      std_msgs::msg::Int32 bullet_msg;
      bullet_msg.data = receive_data.blood;
      bullet_publisher_->publish(bullet_msg);

      wheel_pub_->publish(twist);

      if (time_publish_counter_ >= 1000) {
        std_msgs::msg::Int32 time_remain_msg;
        time_remain_msg.data = receive_data.remaining_time;
        time_remain_publisher_->publish(time_remain_msg);
        time_publish_counter_ = 0;
      } else {
        time_publish_counter_++;
      }

      for (auto &[service_name, client] : set_mode_clients_) {
        (void)service_name;
        if (client.mode.load() != receive_data.mode && !client.on_waiting.load()) {
          setMode(client, receive_data.mode);
        }
      }

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = packet_time;
      t.header.frame_id = target_frame_;
      t.child_frame_id = "gimbal_link";
      auto roll = receive_data.roll * M_PI / 180.0;
      auto pitch = -receive_data.pitch * M_PI / 180.0;
      auto yaw = receive_data.yaw * M_PI / 180.0;
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      t.transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(t);

      Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
      Eigen::Vector3d rpy = utils::getRPY(q_eigen.toRotationMatrix());
      q.setRPY(rpy[0], 0, 0);
      t.header.frame_id = target_frame_;
      t.child_frame_id = target_frame_ + "_rectify";
      tf_broadcaster_->sendTransform(t);
      continue;
    }

    if (!protocol_->isOpen()) {
      ensureConnection("runtime");
      continue;
    }

    auto error_message = protocol_->getErrorMessage();
    error_message = error_message.empty() ? "unknown" : error_message;
    FYT_WARN("serial_driver", "Failed to reveive packet! error message :{}", error_message);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void SerialDriverNode::setMode(SetModeClient &client, const uint8_t mode) {
  using namespace std::chrono_literals;

  std::string service_name = client.ptr->get_service_name();
  while (!client.ptr->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      FYT_ERROR(
        "serial_driver", "Interrupted while waiting for the service {}. Exiting.", service_name);
      return;
    }
    FYT_INFO("serial_driver", "Service {} not available, waiting again...", service_name);
  }
  if (!client.ptr->service_is_ready()) {
    FYT_WARN("serial_driver", "Service: {} is not available!", service_name);
    return;
  }

  auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
  req->mode = mode;

  client.on_waiting.store(true);
  auto result = client.ptr->async_send_request(
    req, [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
      client.on_waiting.store(false);
      if (result.get()->success) {
        client.mode.store(mode);
      }
    });
  (void)result;
}

}  // namespace fyt::serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::serial_driver::SerialDriverNode)
