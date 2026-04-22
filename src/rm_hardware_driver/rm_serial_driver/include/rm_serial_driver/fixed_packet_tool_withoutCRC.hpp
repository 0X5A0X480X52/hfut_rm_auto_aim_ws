// Copyright (C) 2021 RoboMaster-OSS
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
//
// Additional modifications and features by Chengfu Zou, 2023.
//
// Copyright (C) FYT Vision Group. All rights reserved.

#ifndef SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_
#define SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_

#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>

#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::serial_driver {

template <int capacity = 16>
class FixedPacketTool {
public:
  using SharedPtr = std::shared_ptr<FixedPacketTool>;
  FixedPacketTool() = delete;

  explicit FixedPacketTool(std::shared_ptr<TransporterInterface> transporter)
  : transporter_(std::move(transporter)) {
    if (!transporter_) {
      throw std::invalid_argument("transporter is nullptr");
    }
    FYT_REGISTER_LOGGER("serial_driver", "~/fyt2024-log", INFO);
  }

  ~FixedPacketTool() { enbaleRealtimeSend(false); }

  bool open() { return transporter_->open(); }
  void close() { transporter_->close(); }
  bool isOpen() { return transporter_->isOpen(); }
  void enbaleRealtimeSend(bool enable);
  void enbaleDataPrint(bool enable) { use_data_print_ = enable; }
  bool sendPacket(const FixedPacket<capacity> &packet);
  bool recvPacket(FixedPacket<capacity> &packet);

  std::string getErrorMessage() { return transporter_->errorMessage(); }

private:
  bool checkPacket(uint8_t *tmp_buffer, int recv_len);
  bool simpleSendPacket(const FixedPacket<capacity> &packet);

private:
  std::shared_ptr<TransporterInterface> transporter_;
  uint8_t tmp_buffer_[capacity];
  uint8_t recv_buffer_[capacity * 2];
  int recv_buf_len_{0};
  bool use_realtime_send_{false};
  bool use_data_print_{false};
  std::mutex realtime_send_mut_;
  std::unique_ptr<std::thread> realtime_send_thread_;
  std::queue<FixedPacket<capacity>> realtime_packets_;
};

template <int capacity>
bool FixedPacketTool<capacity>::checkPacket(uint8_t *buffer, int recv_len) {
  if (recv_len != capacity) {
    return false;
  }
  if ((buffer[0] != 0xff) || (buffer[capacity - 1] != 0x0d)) {
    return false;
  }
  return true;
}

template <int capacity>
bool FixedPacketTool<capacity>::simpleSendPacket(const FixedPacket<capacity> &packet) {
  if (transporter_->write(packet.buffer(), capacity) == capacity) {
    return true;
  }
  FYT_ERROR("serial_driver", "transporter_->write() failed");
  transporter_->close();
  return false;
}

template <int capacity>
void FixedPacketTool<capacity>::enbaleRealtimeSend(bool enable) {
  if (enable == use_realtime_send_) {
    return;
  }
  if (enable) {
    use_realtime_send_ = true;
    realtime_send_thread_ = std::make_unique<std::thread>([&]() {
      FixedPacket<capacity> packet;
      while (use_realtime_send_) {
        bool empty = true;
        {
          std::lock_guard<std::mutex> lock(realtime_send_mut_);
          empty = realtime_packets_.empty();
          if (!empty) {
            packet = realtime_packets_.front();
            realtime_packets_.pop();
          }
        }
        if (!empty) {
          simpleSendPacket(packet);
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
    });
  } else {
    use_realtime_send_ = false;
    realtime_send_thread_->join();
    realtime_send_thread_.reset();
  }
}

template <int capacity>
bool FixedPacketTool<capacity>::sendPacket(const FixedPacket<capacity> &packet) {
  if (use_realtime_send_) {
    std::lock_guard<std::mutex> lock(realtime_send_mut_);
    realtime_packets_.push(packet);
    return true;
  }
  return simpleSendPacket(packet);
}

template <int capacity>
bool FixedPacketTool<capacity>::recvPacket(FixedPacket<capacity> &packet) {
  int recv_len = transporter_->read(tmp_buffer_, capacity);
  if (recv_len > 0) {
    if (use_data_print_) {
      for (int i = 0; i < recv_len; i++) {
        std::cout << std::hex << static_cast<int>(tmp_buffer_[i]) << " ";
      }
      std::cout << "\n";
    }

    if (checkPacket(tmp_buffer_, recv_len)) {
      packet.copyFrom(tmp_buffer_);
      return true;
    }

    FYT_INFO("serial_driver", "checkPacket() failed, check if it is a broken frame");
    if (recv_buf_len_ + recv_len > capacity * 2) {
      recv_buf_len_ = 0;
    }
    memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer_, recv_len);
    recv_buf_len_ += recv_len;
    for (int i = 0; (i + capacity) <= recv_buf_len_; i++) {
      if (checkPacket(recv_buffer_ + i, capacity)) {
        packet.copyFrom(recv_buffer_ + i);
        int k = 0;
        for (int j = i + capacity; j < recv_buf_len_; j++, k++) {
          recv_buffer_[k] = recv_buffer_[j];
        }
        recv_buf_len_ = k;
        return true;
      }
    }
    FYT_WARN(
      "serial_driver",
      "checkPacket() failed with recv_len:{}, frame head:{}, frame end:{}",
      recv_len,
      tmp_buffer_[0],
      tmp_buffer_[recv_len - 1]);
    return false;
  }

  FYT_ERROR("serial_driver", "transporter_->read() failed");
  transporter_->close();
  return false;
}

using FixedPacketTool16 = FixedPacketTool<16>;
using FixedPacketTool32 = FixedPacketTool<32>;
using FixedPacketTool64 = FixedPacketTool<64>;

}  // namespace fyt::serial_driver

#endif  // SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_
