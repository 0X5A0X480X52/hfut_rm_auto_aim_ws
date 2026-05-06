#include "armor_detector_nn/core/label_map.hpp"

#include <fstream>
#include <set>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

namespace {

std::string resolvePath(const std::string& raw_path) {
  if (raw_path.compare(0, 10, "package://") == 0) {
    auto slash = raw_path.find('/', 10);
    std::string pkg = raw_path.substr(10, slash - 10);
    std::string rel = raw_path.substr(slash + 1);
    return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
  }
  return raw_path;
}

EnemyColor parseColor(const std::string& s) {
  if (s == "red")   return fyt::EnemyColor::RED;
  if (s == "blue")  return fyt::EnemyColor::BLUE;
  if (s == "white") return fyt::EnemyColor::WHITE;
  throw std::runtime_error("LabelMap: unknown color '" + s + "'");
}

const std::set<std::string> kAllowedNumbers = {
  "1", "2", "3", "4", "5", "outpost", "sentry", "base", "negative"
};
const std::set<std::string> kAllowedTypes = {"small", "large", "invalid"};

}  // namespace

void LabelMap::load(const std::string& yaml_path) {
  std::string resolved = resolvePath(yaml_path);

  FYT_INFO("armor_detector_nn", "Loading label map from {}", resolved.c_str());

  YAML::Node root = YAML::LoadFile(resolved);
  auto model_labels = root["model_labels"];
  if (!model_labels || !model_labels.IsSequence()) {
    throw std::runtime_error("LabelMap: missing or invalid 'model_labels' in " + resolved);
  }

  entries_.clear();
  max_class_id_ = 0;

  for (const auto& item : model_labels) {
    LabelEntry entry;
    entry.class_id      = item["class_id"].as<int>();
    entry.model_label   = item["model_label"].as<std::string>();
    entry.color         = parseColor(item["color"].as<std::string>());
    entry.publish_number = item["publish_number"].as<std::string>();
    entry.publish_type  = item["publish_type"].as<std::string>();

    if (entries_.count(entry.class_id)) {
      throw std::runtime_error(
        "LabelMap: duplicate class_id " + std::to_string(entry.class_id));
    }
    entries_[entry.class_id] = entry;
    max_class_id_ = std::max(max_class_id_, entry.class_id);
  }

  auto ignore = root["ignore_labels"];
  ignored_ids_.clear();
  if (ignore && ignore.IsSequence()) {
    for (const auto& lbl : ignore) {
      std::string name = lbl.as<std::string>();
      bool found = false;
      for (const auto& [cid, entry] : entries_) {
        if (entry.model_label == name) {
          ignored_ids_.push_back(cid);
          found = true;
          break;
        }
      }
      if (!found) {
        FYT_ERROR("armor_detector_nn",
                  "ignore_labels entry '{}' not found in model_labels", name.c_str());
      }
    }
  }

  std::string err;
  if (!validate(err)) {
    throw std::runtime_error("LabelMap validation failed: " + err);
  }

  FYT_INFO("armor_detector_nn", "Label map loaded: {} entries, {} ignored",
           entries_.size(), ignored_ids_.size());
}

const LabelEntry* LabelMap::lookup(int class_id) const {
  auto it = entries_.find(class_id);
  if (it == entries_.end()) return nullptr;

  for (int ignored_id : ignored_ids_) {
    if (ignored_id == class_id) return nullptr;
  }
  return &it->second;
}

const LabelEntry* LabelMap::lookupByPublishedLabel(
    const std::string& publish_number,
    EnemyColor color) const {
  for (const auto& [cid, entry] : entries_) {
    if (entry.publish_number == publish_number && entry.color == color) {
      bool ignored = false;
      for (int ignored_id : ignored_ids_) {
        if (ignored_id == cid) {
          ignored = true;
          break;
        }
      }
      if (!ignored) {
        return &entry;
      }
    }
  }
  return nullptr;
}

std::vector<ArmorDetection>
LabelMap::filterByColor(const std::vector<ArmorDetection>& detections,
                         EnemyColor target_color) const {
  std::vector<ArmorDetection> result;
  result.reserve(detections.size());
  for (const auto& d : detections) {
    if (d.color == target_color) {
      result.push_back(d);
    }
  }
  return result;
}

bool LabelMap::validate(std::string& error) const {
  // Check all class IDs from 0..max_class_id are covered
  for (int i = 0; i <= max_class_id_; ++i) {
    if (!entries_.count(i)) {
      error = "missing class_id " + std::to_string(i);
      return false;
    }
  }

  for (const auto& [cid, entry] : entries_) {
    if (kAllowedNumbers.find(entry.publish_number) == kAllowedNumbers.end()) {
      error = "class_id " + std::to_string(cid) +
              ": publish_number '" + entry.publish_number + "' not in allowed set";
      return false;
    }
    if (kAllowedTypes.find(entry.publish_type) == kAllowedTypes.end()) {
      error = "class_id " + std::to_string(cid) +
              ": publish_type '" + entry.publish_type + "' not in allowed set";
      return false;
    }
  }

  return true;
}

}  // namespace fyt::auto_aim
