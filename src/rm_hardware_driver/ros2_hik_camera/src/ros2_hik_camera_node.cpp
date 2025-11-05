// Copyright (c) 2025
// Licensed under the MIT License.

#include "ros2_hik_camera/hik_camera_node.hpp"

namespace ros2_hik_camera
{

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
: Node("hik_camera", options), capturing_(false)
{
  RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

  // Initialize HIKVision SDK
  // Enumerate devices
  memset(&device_list_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  int status = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list_);
  
  RCLCPP_INFO(this->get_logger(), "Enumerate status = %d", status);
  RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list_.nDeviceNum);

  if (device_list_.nDeviceNum == 0) {
    RCLCPP_ERROR(this->get_logger(), "No camera found!");
    return;
  }

  // 打印所有检测到的相机信息
  for (unsigned int i = 0; i < device_list_.nDeviceNum; i++) {
    MV_CC_DEVICE_INFO* dev_info = device_list_.pDeviceInfo[i];
    std::string camera_name = "Unknown";
    std::string serial_number = "Unknown";
    
    if (dev_info->nTLayerType == MV_GIGE_DEVICE) {
      camera_name = std::string(reinterpret_cast<char*>(dev_info->SpecialInfo.stGigEInfo.chModelName));
      serial_number = std::string(reinterpret_cast<char*>(dev_info->SpecialInfo.stGigEInfo.chSerialNumber));
    } else if (dev_info->nTLayerType == MV_USB_DEVICE) {
      camera_name = std::string(reinterpret_cast<char*>(dev_info->SpecialInfo.stUsb3VInfo.chModelName));
      serial_number = std::string(reinterpret_cast<char*>(dev_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
    }
    
    RCLCPP_INFO(this->get_logger(), "Camera %d: %s (SN: %s)", i, camera_name.c_str(), serial_number.c_str());
  }

  // Initialize camera
  if (!initCamera()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera!");
    return;
  }

  // Create camera publisher
  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  // Load camera info
  camera_name_ = this->declare_parameter("camera_name", "hik_camera");
  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  auto camera_info_url =
    this->declare_parameter("camera_info_url", "package://ros2_hik_camera/config/camera_info.yaml");
  if (camera_info_manager_->validateURL(camera_info_url)) {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }

  // Declare and set parameters BEFORE starting grabbing
  declareParameters();

  // Add callback to the set parameter event
  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

  // Start grabbing
  startGrabbing();

  // Start capture thread
  capturing_ = true;
  capture_thread_ = std::thread([this]() -> void {
    RCLCPP_INFO(this->get_logger(), "Publishing image!");

    image_msg_.header.frame_id = "camera_optical_frame";
    image_msg_.encoding = "bgr8";

    // Allocate image data buffer
    MV_FRAME_OUT_INFO_EX frame_info;
    memset(&frame_info, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    
    // Get maximum image size
    MVCC_INTVALUE payload_size;
    MV_CC_GetIntValue(camera_handle_, "PayloadSize", &payload_size);
    frame_buffer_ = new unsigned char[payload_size.nCurValue];
    
    while (rclcpp::ok() && capturing_) {
      memset(&frame_info, 0, sizeof(MV_FRAME_OUT_INFO_EX));
      
      // Use MV_CC_GetOneFrameTimeout with appropriate timeout (2 seconds to accommodate frame rate)
      int status = MV_CC_GetOneFrameTimeout(camera_handle_, frame_buffer_, 
                                             payload_size.nCurValue, &frame_info, 2000);
      if (status == MV_OK) {
        fail_count_ = 0;  // Reset fail count on success
        if (frame_buffer_ != nullptr && frame_info.nFrameLen > 0) {
          // Log pixel type on first frame for debugging
          static bool logged_pixel_type = false;
          if (!logged_pixel_type) {
            RCLCPP_INFO(this->get_logger(), "Source pixel type: 0x%lx, Width: %d, Height: %d, FrameLen: %d", 
                       frame_info.enPixelType, frame_info.nWidth, frame_info.nHeight, frame_info.nFrameLen);
            logged_pixel_type = true;
          }
          
          // For Bayer formats, use OpenCV for debayering instead of HIK SDK
          if (frame_info.enPixelType == PixelType_Gvsp_BayerRG8 ||
              frame_info.enPixelType == PixelType_Gvsp_BayerGB8 ||
              frame_info.enPixelType == PixelType_Gvsp_BayerGR8 ||
              frame_info.enPixelType == PixelType_Gvsp_BayerBG8) {
            
            // Create OpenCV Mat from raw Bayer data
            cv::Mat bayer_image(frame_info.nHeight, frame_info.nWidth, CV_8UC1, frame_buffer_);
            cv::Mat bgr_image;
            
            // Determine the correct Bayer pattern
            int bayer_code = cv::COLOR_BayerRG2BGR; // Default to RG
            if (frame_info.enPixelType == PixelType_Gvsp_BayerGB8) {
              bayer_code = cv::COLOR_BayerGB2BGR;
            } else if (frame_info.enPixelType == PixelType_Gvsp_BayerGR8) {
              bayer_code = cv::COLOR_BayerGR2BGR;
            } else if (frame_info.enPixelType == PixelType_Gvsp_BayerBG8) {
              bayer_code = cv::COLOR_BayerBG2BGR;
            }
            
            // Convert Bayer to BGR using OpenCV
            cv::cvtColor(bayer_image, bgr_image, bayer_code);
            
            // Flip image if required
            if (flip_image_) {
              cv::flip(bgr_image, bgr_image, -1);  // Flip both horizontally and vertically
            }
            
            // Fill image message
            camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
            image_msg_.height = bgr_image.rows;
            image_msg_.width = bgr_image.cols;
            image_msg_.step = bgr_image.cols * 3;
            image_msg_.data.assign(bgr_image.data, bgr_image.data + bgr_image.total() * bgr_image.elemSize());
            
            // Publish
            camera_pub_.publish(image_msg_, camera_info_msg_);
          } else {
            // For non-Bayer formats, use HIK SDK conversion
            MV_CC_PIXEL_CONVERT_PARAM convert_param;
            memset(&convert_param, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
            
            convert_param.nWidth = frame_info.nWidth;
            convert_param.nHeight = frame_info.nHeight;
            convert_param.pSrcData = frame_buffer_;
            convert_param.nSrcDataLen = frame_info.nFrameLen;
            convert_param.enSrcPixelType = frame_info.enPixelType;
            convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            
            // Allocate buffer if needed
            unsigned int need_size = frame_info.nWidth * frame_info.nHeight * 3;
            if (bgr_buffer_size_ < need_size) {
              if (bgr_buffer_ != nullptr) {
                delete[] bgr_buffer_;
              }
              bgr_buffer_ = new unsigned char[need_size];
              bgr_buffer_size_ = need_size;
            }
            
            convert_param.pDstBuffer = bgr_buffer_;
            convert_param.nDstBufferSize = bgr_buffer_size_;
            
            // Verify camera handle before conversion
            if (camera_handle_ == nullptr) {
              RCLCPP_ERROR(this->get_logger(), "Camera handle is null during pixel conversion!");
              continue;
            }
            
            int convert_status = MV_CC_ConvertPixelType(camera_handle_, &convert_param);
            if (convert_status == MV_OK) {
              // Create OpenCV Mat
              cv::Mat image(frame_info.nHeight, frame_info.nWidth, CV_8UC3, bgr_buffer_);
              
              // Flip image if required
              if (flip_image_) {
                cv::flip(image, image, -1);  // Flip both horizontally and vertically
              }
              
              // Fill image message
              camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
              image_msg_.height = image.rows;
              image_msg_.width = image.cols;
              image_msg_.step = image.cols * 3;
              image_msg_.data.assign(image.data, image.data + image.total() * image.elemSize());
              
              // Publish
              camera_pub_.publish(image_msg_, camera_info_msg_);
            } else {
              RCLCPP_WARN(this->get_logger(), "Failed to convert pixel format, status = 0x%x", 
                         convert_status);
            }
          }
        }
      } else {
        // Provide more detailed error information based on HIKVision SDK docs
        if (static_cast<unsigned int>(status) == 0x80000007u) {
          // MV_E_NODATA: 可能原因：
          // 1. 相机帧率低，取流频率高
          // 2. 相机处于触发模式
          // 3. 相机停流
          if (fail_count_ == 0) {
            RCLCPP_WARN(this->get_logger(), 
                       "MV_E_NODATA (0x%x): No data available. Possible causes:\n"
                       "  1. Camera frame rate is too low for the acquisition frequency\n"
                       "  2. Camera is in trigger mode (should be continuous)\n"
                       "  3. Camera stopped streaming\n"
                       "  Suggestion: Check camera frame rate setting and trigger mode",
                       status);
          } else if (fail_count_ % 10 == 0) {
            // 只每10次打印一次，避免日志过多
            RCLCPP_WARN(this->get_logger(), 
                       "Still no data after %d attempts (MV_E_NODATA)", fail_count_);
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to get image buffer, status = 0x%x", status);
        }
        fail_count_++;
      }

      // 对于MV_E_NODATA，更宽容一些，因为这可能是正常现象（帧率不匹配）
      if (fail_count_ > 50 && static_cast<unsigned int>(status) == 0x80000007u) {
        RCLCPP_ERROR(this->get_logger(), 
                    "Persistent MV_E_NODATA error. Please check:\n"
                    "  1. Frame rate setting (current timeout: 1000ms)\n"
                    "  2. Trigger mode (should be OFF for continuous acquisition)\n"
                    "  3. Camera connection and power");
        fail_count_ = 0;  // 重置计数，继续尝试
      } else if (fail_count_ > 10 && static_cast<unsigned int>(status) != 0x80000007u) {
        // 对于其他错误，更快失败
        RCLCPP_FATAL(this->get_logger(), 
                    "Failed to get image buffer multiple times, exit! Last status = 0x%x", 
                    status);
        rclcpp::shutdown();
      }
    }
  });
}

HikCameraNode::~HikCameraNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down HikCameraNode!");
  
  // Stop capture thread
  capturing_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  // Stop grabbing
  stopGrabbing();

  // Close camera
  if (camera_handle_ != nullptr) {
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(camera_handle_);
  }

  // Free buffers
  if (bgr_buffer_ != nullptr) {
    delete[] bgr_buffer_;
  }
  if (frame_buffer_ != nullptr) {
    delete[] frame_buffer_;
  }

  RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
}

bool HikCameraNode::initCamera()
{
  // 获取序列号参数（如果指定）
  std::string camera_sn = this->declare_parameter("camera_sn", "");
  
  // 选择要初始化的相机
  MV_CC_DEVICE_INFO * selected_device = nullptr;
  
  if (camera_sn.empty()) {
    // 如果没有指定序列号，使用第一个相机
    selected_device = device_list_.pDeviceInfo[0];
    
    std::string sn = "Unknown";
    if (selected_device->nTLayerType == MV_GIGE_DEVICE) {
      sn = std::string(reinterpret_cast<char*>(selected_device->SpecialInfo.stGigEInfo.chSerialNumber));
    } else if (selected_device->nTLayerType == MV_USB_DEVICE) {
      sn = std::string(reinterpret_cast<char*>(selected_device->SpecialInfo.stUsb3VInfo.chSerialNumber));
    }
    
    RCLCPP_INFO(this->get_logger(), "No camera_sn specified, using first camera (SN: %s)", sn.c_str());
  } else {
    // 根据序列号查找相机
    for (unsigned int i = 0; i < device_list_.nDeviceNum; i++) {
      MV_CC_DEVICE_INFO* dev_info = device_list_.pDeviceInfo[i];
      std::string sn = "Unknown";
      
      if (dev_info->nTLayerType == MV_GIGE_DEVICE) {
        sn = std::string(reinterpret_cast<char*>(dev_info->SpecialInfo.stGigEInfo.chSerialNumber));
      } else if (dev_info->nTLayerType == MV_USB_DEVICE) {
        sn = std::string(reinterpret_cast<char*>(dev_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
      }
      
      if (sn == camera_sn) {
        selected_device = dev_info;
        RCLCPP_INFO(this->get_logger(), "Found camera with SN: %s", camera_sn.c_str());
        break;
      }
    }
    
    if (selected_device == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Camera with SN '%s' not found!", camera_sn.c_str());
      return false;
    }
  }
  
  // Create handle
  int status = MV_CC_CreateHandle(&camera_handle_, selected_device);
  if (status != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create handle, status = %d", status);
    return false;
  }

  // Open device
  status = MV_CC_OpenDevice(camera_handle_);
  if (status != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device, status = %d", status);
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    return false;
  }

  // Set trigger mode off (important: do this before setting other parameters)
  status = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
  if (status != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "Failed to set trigger mode, status = 0x%x", status);
  } else {
    RCLCPP_INFO(this->get_logger(), "Trigger mode set to OFF");
  }

  // Set acquisition mode to continuous
  status = MV_CC_SetEnumValueByString(camera_handle_, "AcquisitionMode", "Continuous");
  if (status != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "Failed to set acquisition mode, status = 0x%x", status);
  } else {
    RCLCPP_INFO(this->get_logger(), "Acquisition mode set to Continuous");
  }

  // Set pixel format to BayerRG8
  status = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
  if (status != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "Failed to set pixel format, status = 0x%x", status);
  } else {
    RCLCPP_INFO(this->get_logger(), "Pixel format set to BayerRG8");
  }

  // Enable frame rate control
  status = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
  if (status != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "Failed to enable frame rate control, status = 0x%x", status);
  } else {
    RCLCPP_INFO(this->get_logger(), "Frame rate control enabled");
  }

  // Set a conservative default frame rate (10fps to avoid MV_E_NODATA issues)
  // This will be overridden by parameters later if specified
  status = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", 10.0);
  if (status != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "Failed to set initial frame rate, status = 0x%x", status);
  } else {
    RCLCPP_INFO(this->get_logger(), "Initial frame rate set to 10.0 fps (conservative)");
  }

  // Set Bayer conversion quality (1 = bilinear interpolation)
  status = MV_CC_SetBayerCvtQuality(camera_handle_, 1);
  if (status != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "Failed to set Bayer conversion quality, status = 0x%x", status);
  } else {
    RCLCPP_INFO(this->get_logger(), "Bayer conversion quality set to bilinear");
  }

  RCLCPP_INFO(this->get_logger(), "Camera initialized successfully!");
  return true;
}

void HikCameraNode::startGrabbing()
{
  // Get payload size before starting grabbing
  MVCC_INTVALUE payload_size;
  int status = MV_CC_GetIntValue(camera_handle_, "PayloadSize", &payload_size);
  if (status == MV_OK) {
    RCLCPP_INFO(this->get_logger(), "Payload size: %u bytes", payload_size.nCurValue);
  }

  // Start grabbing
  status = MV_CC_StartGrabbing(camera_handle_);
  if (status != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start grabbing, status = 0x%x", status);
    return;
  } 
  
  RCLCPP_INFO(this->get_logger(), "Started grabbing!");
  
  // Send acquisition start command for continuous mode
  status = MV_CC_SetCommandValue(camera_handle_, "AcquisitionStart");
  if (status != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "Failed to send AcquisitionStart command, status = 0x%x. This may be normal.", status);
  } else {
    RCLCPP_INFO(this->get_logger(), "Sent AcquisitionStart command");
  }
  
  // Give camera more time to start acquiring
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void HikCameraNode::stopGrabbing()
{
  int status = MV_CC_StopGrabbing(camera_handle_);
  if (status != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "Failed to stop grabbing, status = %d", status);
  }
}

void HikCameraNode::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  int status;
  
  // Exposure time
  param_desc.description = "Exposure time in microseconds";
  double exposure_time = this->declare_parameter("exposure_time", 5000.0, param_desc);
  status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
  if (status == MV_OK) {
    RCLCPP_INFO(this->get_logger(), "Exposure time = %f us", exposure_time);
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to set exposure time, status = 0x%x", status);
  }

  // Gain
  param_desc.description = "Gain value";
  double gain = this->declare_parameter("gain", 8.0, param_desc);
  status = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
  if (status == MV_OK) {
    RCLCPP_INFO(this->get_logger(), "Gain = %f", gain);
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to set gain, status = 0x%x", status);
  }

  // Frame rate
  param_desc.description = "Frame rate (fps)";
  double frame_rate = this->declare_parameter("frame_rate", 30.0, param_desc);
  status = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
  if (status != MV_OK) {
    RCLCPP_WARN(this->get_logger(), "Failed to enable frame rate, status = 0x%x", status);
  }
  status = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate);
  if (status == MV_OK) {
    RCLCPP_INFO(this->get_logger(), "Frame rate = %f fps", frame_rate);
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to set frame rate, status = 0x%x", status);
  }

  // Flip image
  flip_image_ = this->declare_parameter("flip_image", false);
  RCLCPP_INFO(this->get_logger(), "Flip image = %s", flip_image_ ? "true" : "false");

  // Image resolution
  param_desc.description = "Image width (0 for camera default)";
  image_width_ = this->declare_parameter("image_width", 0, param_desc);
  param_desc.description = "Image height (0 for camera default)";
  image_height_ = this->declare_parameter("image_height", 0, param_desc);

  // Set resolution if both width and height are non-zero
  if (image_width_ > 0 && image_height_ > 0) {
    MVCC_INTVALUE stWidthMax = {0};
    MVCC_INTVALUE stHeightMax = {0};
    status = MV_CC_GetIntValue(camera_handle_, "WidthMax", &stWidthMax);
    if (status == MV_OK) {
      status = MV_CC_GetIntValue(camera_handle_, "HeightMax", &stHeightMax);
    }
    
    if (status == MV_OK) {
      if (image_width_ <= static_cast<int>(stWidthMax.nCurValue) && 
          image_height_ <= static_cast<int>(stHeightMax.nCurValue)) {
        // Set offset to 0 first to ensure ROI starts from top-left corner
        MV_CC_SetIntValue(camera_handle_, "OffsetX", 0);
        MV_CC_SetIntValue(camera_handle_, "OffsetY", 0);
        
        // Set width and height
        status = MV_CC_SetIntValue(camera_handle_, "Width", image_width_);
        if (status == MV_OK) {
          status = MV_CC_SetIntValue(camera_handle_, "Height", image_height_);
        }
        
        if (status == MV_OK) {
          RCLCPP_INFO(this->get_logger(), "Resolution set to %dx%d", image_width_, image_height_);
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to set resolution, status = 0x%x", status);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), 
          "Requested resolution %dx%d exceeds camera capability %dx%d, using camera default",
          image_width_, image_height_, 
          static_cast<int>(stWidthMax.nCurValue), static_cast<int>(stHeightMax.nCurValue));
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get camera resolution capability, status = 0x%x", status);
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Using camera default resolution");
  }
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto & param : parameters) {
    if (param.get_name() == "exposure_time") {
      int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
      if (status != MV_OK) {
        result.successful = false;
        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "gain") {
      int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      if (status != MV_OK) {
        result.successful = false;
        result.reason = "Failed to set gain, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "frame_rate") {
      int status = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", param.as_double());
      if (status != MV_OK) {
        result.successful = false;
        result.reason = "Failed to set frame rate, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "flip_image") {
      flip_image_ = param.as_bool();
    } else if (param.get_name() == "image_width") {
      image_width_ = param.as_int();
      // Apply resolution only if both width and height are non-zero
      if (image_width_ > 0 && image_height_ > 0) {
        // Set offset to 0 first
        MV_CC_SetIntValue(camera_handle_, "OffsetX", 0);
        MV_CC_SetIntValue(camera_handle_, "OffsetY", 0);
        
        int status = MV_CC_SetIntValue(camera_handle_, "Width", image_width_);
        if (status == MV_OK) {
          status = MV_CC_SetIntValue(camera_handle_, "Height", image_height_);
        }
        if (status != MV_OK) {
          result.successful = false;
          result.reason = "Failed to set resolution, status = " + std::to_string(status);
        }
      }
    } else if (param.get_name() == "image_height") {
      image_height_ = param.as_int();
      // Apply resolution only if both width and height are non-zero
      if (image_width_ > 0 && image_height_ > 0) {
        // Set offset to 0 first
        MV_CC_SetIntValue(camera_handle_, "OffsetX", 0);
        MV_CC_SetIntValue(camera_handle_, "OffsetY", 0);
        
        int status = MV_CC_SetIntValue(camera_handle_, "Width", image_width_);
        if (status == MV_OK) {
          status = MV_CC_SetIntValue(camera_handle_, "Height", image_height_);
        }
        if (status != MV_OK) {
          result.successful = false;
          result.reason = "Failed to set resolution, status = " + std::to_string(status);
        }
      }
    } else {
      result.successful = false;
      result.reason = "Unknown parameter: " + param.get_name();
    }
  }
  
  return result;
}

}  // namespace ros2_hik_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_hik_camera::HikCameraNode)
