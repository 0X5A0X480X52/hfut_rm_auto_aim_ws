#include "armor_fusion/visualization_utils.hpp"

#include <rclcpp/duration.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace fyt::auto_aim {

visualization_msgs::msg::MarkerArray buildVisualizationMarkers(
  const std::vector<std::vector<ArmorMeasurement>> & clusters,
  const std::vector<rm_interfaces::msg::Armor> & fused_armors,
  const std::string & target_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;

  int marker_id = 0;
  for (size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
    for (const auto & measurement : clusters[cluster_index]) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = target_frame;
      marker.header.stamp = stamp;
      marker.ns = "cluster_measurements";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = measurement.position.x();
      marker.pose.position.y = measurement.position.y();
      marker.pose.position.z = measurement.position.z();
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = 0.0F;
      marker.color.g = 1.0F;
      marker.color.b = 0.0F;
      marker.color.a = 0.5F;
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker_array.markers.push_back(marker);
    }
  }

  for (const auto & armor : fused_armors) {
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = target_frame;
    target_marker.header.stamp = stamp;
    target_marker.ns = "fused_targets";
    target_marker.id = marker_id++;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.pose = armor.pose;
    target_marker.scale.x = 0.15;
    target_marker.scale.y = 0.15;
    target_marker.scale.z = 0.15;
    target_marker.color.r = 1.0F;
    target_marker.color.g = 0.0F;
    target_marker.color.b = 0.0F;
    target_marker.color.a = 0.8F;
    target_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(target_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header = target_marker.header;
    text_marker.ns = "fused_target_ids";
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose = armor.pose;
    text_marker.pose.position.z += 0.2;
    text_marker.scale.z = 0.1;
    text_marker.color.r = 1.0F;
    text_marker.color.g = 1.0F;
    text_marker.color.b = 1.0F;
    text_marker.color.a = 1.0F;
    text_marker.text = armor.number;
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}

}  // namespace fyt::auto_aim
