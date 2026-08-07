#pragma once
#include <chrono>
#include <string>

#include "opencv2/core/types.hpp"

namespace auto_buff {
enum class EnemyColor {
  Red = 0,
  Blue = 1,
};

enum class BuffBladeType {
  Inactivated = 0,
  Activated = 1,
};

struct RunePoints {
  RunePoints operator+(const RunePoints &other) const;
  RunePoints operator/(const float &other) const;
  RunePoints operator*(const float &other) const;

  std::vector<cv::Point2f> toVector2f() const {
    return {center, bottom_left, top_left, top_right, bottom_right};
  }
  std::vector<cv::Point> toVector2i() const {
    return {center, bottom_left, top_left, top_right, bottom_right};
  }

  cv::Point2f center;
  cv::Point2f bottom_right;
  cv::Point2f top_right;
  cv::Point2f top_left;
  cv::Point2f bottom_left;

  std::vector<RunePoints> children;
  std::vector<float> probs;
};

struct RuneObject {
  EnemyColor color;
  BuffBladeType type;
  RunePoints points;
  float prob;
  cv::Rect box;
  std::string frame_id;
  std::chrono::system_clock::time_point stamp;
  float area;
};

struct GridAndStride {
  int grid0;
  int grid1;
  int stride;
};

}  // namespace auto_buff
