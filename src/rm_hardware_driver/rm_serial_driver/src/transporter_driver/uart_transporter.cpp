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

#include "rm_serial_driver/uart_transporter.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace fyt::serial_driver {

bool UartTransporter::setParam(int speed, int flow_ctrl, int databits, int stopbits, int parity) {
  int speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200, 300};
  struct termios options;
  if (tcgetattr(fd_, &options) != 0) {
    error_message_ = "Setup Serial err";
    return false;
  }
  for (std::size_t i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    if (speed == name_arr[i]) {
      cfsetispeed(&options, speed_arr[i]);
      cfsetospeed(&options, speed_arr[i]);
    }
  }
  options.c_cflag |= CLOCAL;
  options.c_cflag |= CREAD;
  switch (flow_ctrl) {
    case 0:
      options.c_cflag &= ~CRTSCTS;
      break;
    case 1:
      options.c_cflag |= CRTSCTS;
      break;
    case 2:
      options.c_cflag |= IXON | IXOFF | IXANY;
      break;
  }
  options.c_cflag &= ~CSIZE;
  switch (databits) {
    case 5:
      options.c_cflag |= CS5;
      break;
    case 6:
      options.c_cflag |= CS6;
      break;
    case 7:
      options.c_cflag |= CS7;
      break;
    case 8:
      options.c_cflag |= CS8;
      break;
    default:
      error_message_ = "Unsupported data size";
      return false;
  }
  switch (parity) {
    case 'n':
    case 'N':
      options.c_cflag &= ~PARENB;
      options.c_iflag &= ~INPCK;
      break;
    case 'o':
    case 'O':
      options.c_cflag |= (PARODD | PARENB);
      options.c_iflag |= INPCK;
      break;
    case 'e':
    case 'E':
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_iflag |= INPCK;
      break;
    case 's':
    case 'S':
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      break;
    default:
      error_message_ = "Unsupported parity";
      return false;
  }
  switch (stopbits) {
    case 1:
      options.c_cflag &= ~CSTOPB;
      break;
    case 2:
      options.c_cflag |= CSTOPB;
      break;
    default:
      error_message_ = "Unsupported stop bits";
      return false;
  }

  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = 1;
  tcflush(fd_, TCIFLUSH);

  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    error_message_ = "com set error";
    return false;
  }
  return true;
}

bool UartTransporter::open() {
  if (is_open_) {
    return true;
  }

  fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) {
    error_message_ = "can't open uart device: " + device_path_ + ", errno: " + strerror(errno);
    return false;
  }

  if (fcntl(fd_, F_SETFL, 0) < 0) {
    error_message_ = "fcntl failed";
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  if (!setParam(speed_, flow_ctrl_, databits_, stopbits_, parity_)) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  error_message_.clear();
  is_open_ = true;
  return true;
}

void UartTransporter::close() {
  if (!is_open_) {
    return;
  }
  ::close(fd_);
  fd_ = -1;
  is_open_ = false;
}

bool UartTransporter::isOpen() { return is_open_; }

int UartTransporter::read(void *buffer, std::size_t len) {
  if (!is_open_ || fd_ < 0) {
    error_message_ = "uart device is not open";
    return -1;
  }

  int ret = ::read(fd_, buffer, len);
  if (ret < 0) {
    error_message_ = std::string("uart read failed: ") + strerror(errno);
  }
  return ret;
}

int UartTransporter::write(const void *buffer, std::size_t len) {
  if (!is_open_ || fd_ < 0) {
    error_message_ = "uart device is not open";
    return -1;
  }

  int ret = ::write(fd_, buffer, len);
  if (ret < 0) {
    error_message_ = std::string("uart write failed: ") + strerror(errno);
  }
  return ret;
}

}  // namespace fyt::serial_driver
