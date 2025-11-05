// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// MindVision Camera SDK
#include <CameraApi.h>

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// C++ system
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// #include "rm_utils/heartbeat.hpp"

namespace mindvision_camera
{
class MVCameraNode : public rclcpp::Node
{
public:
  explicit MVCameraNode(const rclcpp::NodeOptions & options) : Node("mv_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting MVCameraNode!");

    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    int i_camera_counts = 10;  // 最多支持10个相机
    int i_status = -1;
    tSdkCameraDevInfo t_camera_enum_list[10];
    i_status = CameraEnumerateDevice(t_camera_enum_list, &i_camera_counts);
    RCLCPP_INFO(this->get_logger(), "Enumerate state = %d", i_status);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", i_camera_counts);

    // 没有连接设备
    if (i_camera_counts == 0) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      return;
    }

    // 获取序列号参数（如果指定）
    std::string camera_sn = this->declare_parameter("camera_sn", "");
    
    // 选择要初始化的相机
    tSdkCameraDevInfo* selected_camera = nullptr;
    if (camera_sn.empty()) {
      // 如果没有指定序列号，使用第一个相机
      selected_camera = &t_camera_enum_list[0];
      RCLCPP_INFO(this->get_logger(), "No camera_sn specified, using first camera: %s (SN: %s)", 
                  selected_camera->acFriendlyName, selected_camera->acSn);
    } else {
      // 根据序列号查找相机
      for (int i = 0; i < i_camera_counts; i++) {
        RCLCPP_INFO(this->get_logger(), "Camera %d: %s (SN: %s)", 
                    i, t_camera_enum_list[i].acFriendlyName, t_camera_enum_list[i].acSn);
        if (std::string(t_camera_enum_list[i].acSn) == camera_sn) {
          selected_camera = &t_camera_enum_list[i];
          RCLCPP_INFO(this->get_logger(), "Found camera with SN: %s", camera_sn.c_str());
          break;
        }
      }
      
      if (selected_camera == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Camera with SN '%s' not found!", camera_sn.c_str());
        return;
      }
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    i_status = CameraInit(selected_camera, -1, -1, &h_camera_);

    // 初始化失败
    RCLCPP_INFO(this->get_logger(), "Init state = %d", i_status);
    if (i_status != CAMERA_STATUS_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Init failed!");
      return;
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(h_camera_, &t_capability_);

    // 直接使用vector的内存作为相机输出buffer
    image_msg_.data.reserve(
      t_capability_.sResolutionRange.iHeightMax * t_capability_.sResolutionRange.iWidthMax * 3);

    // 设置手动曝光
    CameraSetAeState(h_camera_, false);

    // 设置连续采集模式 (0=连续采集, 1=软件触发, 2=硬件触发)
    CameraSetTriggerMode(h_camera_, 0);

    // 设置输出格式 (必须在 CameraPlay 之前设置)
    CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_RGB8);

    // Declare camera parameters (包括分辨率设置)
    declareParameters();

    // 让SDK进入工作模式，开始接收来自相机发送的图像
    // 数据。如果当前相机是触发模式，则需要接收到
    // 触发帧以后才会更新图像。
    CameraPlay(h_camera_);

    // Create camera publisher
    // rqt_image_view can't subscribe image msg with sensor_data QoS
    // https://github.com/ros-visualization/rqt/issues/187
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "mv_camera");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://rm_bringup/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    // Add callback to the set parameter event
    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MVCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        int status = CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000);
        if (status == CAMERA_STATUS_SUCCESS) {
          CameraImageProcess(h_camera_, pby_buffer_, image_msg_.data.data(), &s_frame_info_);
          if (flip_image_) {
            CameraFlipFrameBuffer(image_msg_.data.data(), &s_frame_info_, 3);
          }
          camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
          image_msg_.height = s_frame_info_.iHeight;
          image_msg_.width = s_frame_info_.iWidth;
          image_msg_.step = s_frame_info_.iWidth * 3;
          image_msg_.data.resize(s_frame_info_.iWidth * s_frame_info_.iHeight * 3);

          camera_pub_.publish(image_msg_, camera_info_msg_);

          // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
          // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，
          // 直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
          CameraReleaseImageBuffer(h_camera_, pby_buffer_);
          fail_conut_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to get image buffer, status = %d", status);
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Failed to get image buffer, exit!");
          rclcpp::shutdown();
        }
      }
    }};

    // heartbeat_ = HeartBeatPublisher::create(this);
  }

  ~MVCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }

    CameraUnInit(h_camera_);

    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
  }

private:
  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    // 对于CMOS传感器，其曝光的单位是按照行来计算的
    double exposure_line_time;
    CameraGetExposureLineTime(h_camera_, &exposure_line_time);
    param_desc.integer_range[0].from_value =
      t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time;
    param_desc.integer_range[0].to_value =
      t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time;
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    CameraSetExposureTime(h_camera_, exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time = %f", exposure_time);

    // Analog gain
    param_desc.description = "Analog gain";
    param_desc.integer_range[0].from_value = t_capability_.sExposeDesc.uiAnalogGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sExposeDesc.uiAnalogGainMax;
    int analog_gain;
    CameraGetAnalogGain(h_camera_, &analog_gain);
    analog_gain = this->declare_parameter("analog_gain", analog_gain, param_desc);
    CameraSetAnalogGain(h_camera_, analog_gain);
    RCLCPP_INFO(this->get_logger(), "Analog gain = %d", analog_gain);

    // RGB Gain
    // Get default value
    CameraGetGain(h_camera_, &r_gain_, &g_gain_, &b_gain_);
    // R Gain
    param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iRGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iRGainMax;
    r_gain_ = this->declare_parameter("rgb_gain.r", r_gain_, param_desc);
    // G Gain
    param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iGGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iGGainMax;
    g_gain_ = this->declare_parameter("rgb_gain.g", g_gain_, param_desc);
    // B Gain
    param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iBGainMin;
    param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iBGainMax;
    b_gain_ = this->declare_parameter("rgb_gain.b", b_gain_, param_desc);
    // Set gain
    CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: R = %d", r_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: G = %d", g_gain_);
    RCLCPP_INFO(this->get_logger(), "RGB Gain: B = %d", b_gain_);

    // Saturation
    param_desc.description = "Saturation";
    param_desc.integer_range[0].from_value = t_capability_.sSaturationRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sSaturationRange.iMax;
    int saturation;
    CameraGetSaturation(h_camera_, &saturation);
    saturation = this->declare_parameter("saturation", saturation, param_desc);
    CameraSetSaturation(h_camera_, saturation);
    RCLCPP_INFO(this->get_logger(), "Saturation = %d", saturation);

    // Gamma
    param_desc.integer_range[0].from_value = t_capability_.sGammaRange.iMin;
    param_desc.integer_range[0].to_value = t_capability_.sGammaRange.iMax;
    int gamma;
    CameraGetGamma(h_camera_, &gamma);
    gamma = this->declare_parameter("gamma", gamma, param_desc);
    CameraSetGamma(h_camera_, gamma);
    RCLCPP_INFO(this->get_logger(), "Gamma = %d", gamma);

    // Frame Rate (帧率控制)
    // 优先尝试使用 CameraSetFrameRate (网口相机支持)
    // 如果不支持，则回退到 CameraSetFrameSpeed (USB相机支持)
    param_desc.description = "Frame rate in Hz (<=0 for maximum, recommend 10-30 for dual cameras)";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 200;
    int frame_rate = this->declare_parameter("frame_rate", 10, param_desc);
    
    int status_fr = CameraSetFrameRate(h_camera_, frame_rate);
    if (status_fr == CAMERA_STATUS_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Frame rate = %d fps", frame_rate);
    } else if (status_fr == -4) {  // CAMERA_STATUS_NOT_SUPPORTED
      // 网口相机的帧率设置不支持，尝试使用帧速度模式（USB相机）
      RCLCPP_INFO(this->get_logger(), "CameraSetFrameRate not supported, using CameraSetFrameSpeed instead");
      
      // Frame Speed (帧速度模式): 0=Low, 1=Normal, 2=High, 3=Super
      // 对于双相机场景，建议使用 Low 模式以降低USB带宽
      param_desc.description = "Frame speed mode (0=Low, 1=Normal, 2=High, 3=Super)";
      param_desc.integer_range[0].from_value = 0;
      param_desc.integer_range[0].to_value = t_capability_.iFrameSpeedDesc - 1;
      int frame_speed = this->declare_parameter("frame_speed", 0, param_desc);  // 默认Low模式
      
      int status_fs = CameraSetFrameSpeed(h_camera_, frame_speed);
      if (status_fs == CAMERA_STATUS_SUCCESS) {
        const char* speed_names[] = {"Low", "Normal", "High", "Super"};
        RCLCPP_INFO(this->get_logger(), "Frame speed = %d (%s)", frame_speed,
                    frame_speed < 4 ? speed_names[frame_speed] : "Unknown");
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to set frame speed, status = %d", status_fs);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to set frame rate, status = %d", status_fr);
    }

    // Image Resolution (图像分辨率)
    // 0 表示使用相机默认分辨率，不进行设置
    param_desc.description = "Image width (0 for camera default)";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = t_capability_.sResolutionRange.iWidthMax;
    image_width_ = this->declare_parameter("image_width", 0, param_desc);
    
    param_desc.description = "Image height (0 for camera default)";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = t_capability_.sResolutionRange.iHeightMax;
    image_height_ = this->declare_parameter("image_height", 0, param_desc);
    
    // 如果用户指定了分辨率（非0），则设置分辨率
    if (image_width_ > 0 && image_height_ > 0) {
      tSdkImageResolution resolution;
      memset(&resolution, 0, sizeof(tSdkImageResolution));
      
      // 设置用户指定的宽度和高度
      resolution.iIndex = 0xFF;  // 自定义分辨率
      resolution.iWidth = image_width_;
      resolution.iHeight = image_height_;
      resolution.iWidthFOV = image_width_;
      resolution.iHeightFOV = image_height_;
      resolution.iHOffsetFOV = 0;
      resolution.iVOffsetFOV = 0;
      resolution.iWidthZoomSw = 0;
      resolution.iHeightZoomSw = 0;
      
      int status_res = CameraSetImageResolution(h_camera_, &resolution);
      if (status_res == CAMERA_STATUS_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Image resolution set to %dx%d", image_width_, image_height_);
        
        // 验证设置是否生效
        tSdkImageResolution current_res;
        CameraGetImageResolution(h_camera_, &current_res);
        RCLCPP_INFO(this->get_logger(), "Current resolution: %dx%d (FOV: %dx%d)", 
                    current_res.iWidth, current_res.iHeight,
                    current_res.iWidthFOV, current_res.iHeightFOV);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to set image resolution to %dx%d, status = %d. Using camera default.",
                    image_width_, image_height_, status_res);
      }
    } else if (image_width_ > 0 || image_height_ > 0) {
      RCLCPP_WARN(this->get_logger(), "Both image_width and image_height must be set (non-zero) to change resolution. Using camera default.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Using camera default resolution");
    }

    // Flip
    flip_image_ = this->declare_parameter("flip_image", false);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = CameraSetExposureTime(h_camera_, param.as_int());
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "analog_gain") {
        int status = CameraSetAnalogGain(h_camera_, param.as_int());
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failed to set analog gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rgb_gain.r") {
        r_gain_ = param.as_int();
        int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rgb_gain.g") {
        g_gain_ = param.as_int();
        int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rgb_gain.b") {
        b_gain_ = param.as_int();
        int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "saturation") {
        int status = CameraSetSaturation(h_camera_, param.as_int());
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failed to set saturation, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gamma") {
        int gamma = param.as_int();
        int status = CameraSetGamma(h_camera_, gamma);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failed to set Gamma, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "frame_rate") {
        int frame_rate = param.as_int();
        int status = CameraSetFrameRate(h_camera_, frame_rate);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failed to set frame rate, status = " + std::to_string(status);
        } else {
          RCLCPP_INFO(this->get_logger(), "Frame rate changed to %d fps", frame_rate);
        }
      } else if (param.get_name() == "frame_speed") {
        int frame_speed = param.as_int();
        int status = CameraSetFrameSpeed(h_camera_, frame_speed);
        if (status != CAMERA_STATUS_SUCCESS) {
          result.successful = false;
          result.reason = "Failed to set frame speed, status = " + std::to_string(status);
        } else {
          const char* speed_names[] = {"Low", "Normal", "High", "Super"};
          RCLCPP_INFO(this->get_logger(), "Frame speed changed to %d (%s)", frame_speed,
                      frame_speed < 4 ? speed_names[frame_speed] : "Unknown");
        }
      } else if (param.get_name() == "image_width") {
        image_width_ = param.as_int();
        // 只有当宽度和高度都非0时才设置分辨率
        if (image_width_ > 0 && image_height_ > 0) {
          tSdkImageResolution resolution;
          memset(&resolution, 0, sizeof(tSdkImageResolution));
          resolution.iIndex = 0xFF;  // 自定义分辨率
          resolution.iWidth = image_width_;
          resolution.iHeight = image_height_;
          resolution.iWidthFOV = image_width_;
          resolution.iHeightFOV = image_height_;
          resolution.iHOffsetFOV = 0;
          resolution.iVOffsetFOV = 0;
          resolution.iWidthZoomSw = 0;
          resolution.iHeightZoomSw = 0;
          
          int status = CameraSetImageResolution(h_camera_, &resolution);
          if (status != CAMERA_STATUS_SUCCESS) {
            result.successful = false;
            result.reason = "Failed to set image resolution, status = " + std::to_string(status);
          } else {
            RCLCPP_INFO(this->get_logger(), "Image resolution changed to %dx%d", image_width_, image_height_);
          }
        }
      } else if (param.get_name() == "image_height") {
        image_height_ = param.as_int();
        // 只有当宽度和高度都非0时才设置分辨率
        if (image_width_ > 0 && image_height_ > 0) {
          tSdkImageResolution resolution;
          memset(&resolution, 0, sizeof(tSdkImageResolution));
          resolution.iIndex = 0xFF;  // 自定义分辨率
          resolution.iWidth = image_width_;
          resolution.iHeight = image_height_;
          resolution.iWidthFOV = image_width_;
          resolution.iHeightFOV = image_height_;
          resolution.iHOffsetFOV = 0;
          resolution.iVOffsetFOV = 0;
          resolution.iWidthZoomSw = 0;
          resolution.iHeightZoomSw = 0;
          
          int status = CameraSetImageResolution(h_camera_, &resolution);
          if (status != CAMERA_STATUS_SUCCESS) {
            result.successful = false;
            result.reason = "Failed to set image resolution, status = " + std::to_string(status);
          } else {
            RCLCPP_INFO(this->get_logger(), "Image resolution changed to %dx%d", image_width_, image_height_);
          }
        }
      } else if (param.get_name() == "flip_image") {
        flip_image_ = param.as_bool();
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  int h_camera_;
  uint8_t * pby_buffer_;
  tSdkCameraCapbility t_capability_;  // 设备描述信息
  tSdkFrameHead s_frame_info_;        // 图像帧头信息

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  // Heartbeat
  // HeartBeatPublisher::SharedPtr heartbeat_;

  // RGB Gain
  int r_gain_, g_gain_, b_gain_;

  // Image Resolution
  int image_width_, image_height_;

  bool flip_image_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_conut_ = 0;
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

}  // namespace mindvision_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mindvision_camera::MVCameraNode)
