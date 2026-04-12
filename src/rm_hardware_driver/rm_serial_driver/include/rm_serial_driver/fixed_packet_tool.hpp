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

// std
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
// project
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include "rm_utils/logger/log.hpp"

#define CRC_START_8 0x00

namespace fyt::serial_driver {

template <int capacity = 16>
class FixedPacketTool {
public:
  using SharedPtr = std::shared_ptr<FixedPacketTool>;
  FixedPacketTool() = delete;
  explicit FixedPacketTool(std::shared_ptr<TransporterInterface> transporter)
  : transporter_(transporter) {
    if (!transporter) {
      throw std::invalid_argument("transporter is nullptr");
    }
    FYT_REGISTER_LOGGER("serial_driver", "~/fyt2024-log", INFO);
  }

  ~FixedPacketTool() { enbaleRealtimeSend(false); }

  bool isOpen() { return transporter_->isOpen(); }
  void enbaleRealtimeSend(bool enable);
  void enbaleDataPrint(bool enable) { use_data_print_ = enable; }
  bool sendPacket(const FixedPacket<capacity> &packet);
  bool recvPacket(FixedPacket<capacity> &packet);

  std::string getErrorMessage() { return transporter_->errorMessage(); }

private:
  bool checkPacket(uint8_t *tmp_buffer, int recv_len);
  uint8_t crc8_ccitt(uint8_t *data);
  bool simpleSendPacket(const FixedPacket<capacity> &packet);

private:
  std::shared_ptr<TransporterInterface> transporter_;
  // data
  uint8_t tmp_buffer_[capacity];       // NOLINT
  uint8_t recv_buffer_[capacity * 2];  // NOLINT
  int recv_buf_len_;
  // for realtime sending
  bool use_realtime_send_{false};
  bool use_data_print_{false};
  std::mutex realtime_send_mut_;
  std::unique_ptr<std::thread> realtime_send_thread_;
  std::queue<FixedPacket<capacity>> realtime_packets_;
};

//CRC校验计算
template <int capacity>
uint8_t FixedPacketTool<capacity>::crc8_ccitt(uint8_t *data) {
  static uint8_t sht75_crc_table[] =
    {
        0, 49, 98, 83, 196, 245, 166, 151, 185, 136, 219, 234, 125, 76, 31, 46,
        67, 114, 33, 16, 135, 182, 229, 212, 250, 203, 152, 169, 62, 15, 92, 109,
        134, 183, 228, 213, 66, 115, 32, 17, 63, 14, 93, 108, 251, 202, 153, 168,
        197, 244, 167, 150, 1, 48, 99, 82, 124, 77, 30, 47, 184, 137, 218, 235,
        61, 12, 95, 110, 249, 200, 155, 170, 132, 181, 230, 215, 64, 113, 34, 19,
        126, 79, 28, 45, 186, 139, 216, 233, 199, 246, 165, 148, 3, 50, 97, 80,
        187, 138, 217, 232, 127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149,
        248, 201, 154, 171, 60, 13, 94, 111, 65, 112, 35, 18, 133, 180, 231, 214,
        122, 75, 24, 41, 190, 143, 220, 237, 195, 242, 161, 144, 7, 54, 101, 84,
        57, 8, 91, 106, 253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23,
        252, 205, 158, 175, 56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210,
        191, 142, 221, 236, 123, 74, 25, 40, 6, 55, 100, 85, 194, 243, 160, 145,
        71, 118, 37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105,
        4, 53, 102, 87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42,
        193, 240, 163, 146, 5, 52, 103, 86, 120, 73, 26, 43, 188, 141, 222, 239,
        130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157, 172};
  uint16_t a;
  uint8_t crc;
  const uint8_t *ptr;

  crc = CRC_START_8;  // 下位机的初始值，确保一致
  ptr = data;
  // 按下位机协议，计算前 (capacity - 2) 个字节的 CRC
  for (a = 0; a < capacity - 2; a++) {
    crc = sht75_crc_table[(*ptr++) ^ crc];
  }
  return crc;
}

template <int capacity>
bool FixedPacketTool<capacity>::checkPacket(uint8_t *buffer, int recv_len) {
  // 检查长度
  if (recv_len != capacity) {
    FYT_WARN("serial_driver", "checkPacket Failed: recv_len != capacity");
    return false;
  }
  // 检查帧头，帧尾,
  if ((buffer[0] != 0xff) || (buffer[capacity - 1] != 0x0d)) {
    FYT_WARN("serial_driver", "checkPacket Failed: error head or tail");
    return false;
  }
  // TODO(gezp): 检查check_byte(buffer[capacity-2]),可采用异或校验(BCC)
  if (crc8_ccitt(buffer) != buffer[capacity-2]) {
    FYT_WARN("serial_driver", "checkPacket Failed: CRC failed");
    return false;
  }
  return true;
}

template <int capacity>
bool FixedPacketTool<capacity>::simpleSendPacket(const FixedPacket<capacity> &packet) {
  if (transporter_->write(packet.buffer(), capacity) == capacity) {
    return true;
  } else {
    // reconnect
    FYT_ERROR("serial_driver", "transporter_->write() failed");
    transporter_->close();
    transporter_->open();
    return false;
  }
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
  // 为避免对const对象进行修改，这里先复制出一个可修改的packet副本
  FixedPacket<capacity> mutable_packet = packet;
  // 设置校验位：根据前 capacity-2 个字节计算CRC，并将校验值设置到packet的校验位上（假设 setCheckByte 会将值写入 buffer_[capacity-2]）
  mutable_packet.setCheckByte(crc8_ccitt(mutable_packet.buffer()));

  if (use_realtime_send_) {
    std::lock_guard<std::mutex> lock(realtime_send_mut_);
    realtime_packets_.push(mutable_packet);
    return true;
  } else {
    return simpleSendPacket(mutable_packet);
  }
}

template <int capacity>
bool FixedPacketTool<capacity>::recvPacket(FixedPacket<capacity> &packet) {
  int recv_len = transporter_->read(tmp_buffer_, capacity);
  if (recv_len > 0) {
    // print data
    if (use_data_print_) {
      for (int i = 0; i < recv_len; i++) {
        std::cout << std::hex << static_cast<int>(tmp_buffer_[i]) << " ";
      }
      std::cout << "\n";
    }

    // check packet
    if (checkPacket(tmp_buffer_, recv_len)) {
      packet.copyFrom(tmp_buffer_);
      return true;
    } else {
      // 如果是断帧，拼接缓存，并遍历校验，获得合法数据
      FYT_INFO("serial_driver", "checkPacket() failed, check if it is a broken frame");
      if (recv_buf_len_ + recv_len > capacity * 2) {
        recv_buf_len_ = 0;
      }
      // 拼接缓存
      memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer_, recv_len);
      recv_buf_len_ = recv_buf_len_ + recv_len;
      // 遍历校验
      for (int i = 0; (i + capacity) <= recv_buf_len_; i++) {
        if (checkPacket(recv_buffer_ + i, capacity)) {
          packet.copyFrom(recv_buffer_ + i);
          // 读取一帧后，更新接收缓存
          int k = 0;
          for (int j = i + capacity; j < recv_buf_len_; j++, k++) {
            recv_buffer_[k] = recv_buffer_[j];
          }
          recv_buf_len_ = k;
          return true;
        }
      }
      // 表明断帧，或错误帧。
      FYT_WARN("serial_driver",
               "checkPacket() failed with recv_len:{}, frame head:{}, frame end:{}",
               recv_len,
               tmp_buffer_[0],
               tmp_buffer_[recv_len - 1]);
      return false;
    }
  } else {
    FYT_ERROR("serial_driver", "transporter_->read() failed");
    // reconnect
    transporter_->close();
    transporter_->open();
    // 串口错误
    return false;
  }
}

using FixedPacketTool16 = FixedPacketTool<16>;
using FixedPacketTool32 = FixedPacketTool<32>;
using FixedPacketTool64 = FixedPacketTool<64>;

}  // namespace fyt::serial_driver

#endif  // SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_
