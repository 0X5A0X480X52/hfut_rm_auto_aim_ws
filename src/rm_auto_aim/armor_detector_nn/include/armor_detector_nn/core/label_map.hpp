#ifndef ARMOR_DETECTOR_NN_LABEL_MAP_HPP_
#define ARMOR_DETECTOR_NN_LABEL_MAP_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "armor_detector_nn/core/detection_types.hpp"

namespace fyt::auto_aim {

struct LabelEntry {
  int class_id{-1};
  std::string model_label;
  fyt::EnemyColor color{fyt::EnemyColor::RED};
  std::string publish_number;
  std::string publish_type;
};

class LabelMap {
public:
  LabelMap() = default;

  void load(const std::string& yaml_path);

  const LabelEntry* lookup(int class_id) const;
  const LabelEntry* lookupByPublishedLabel(
    const std::string& publish_number,
    fyt::EnemyColor color) const;

  std::vector<ArmorDetection>
  filterByColor(const std::vector<ArmorDetection>& detections,
                fyt::EnemyColor target_color) const;

  bool validate(std::string& error) const;

  const std::unordered_map<int, LabelEntry>& entries() const { return entries_; }
  const std::vector<int>& ignored() const { return ignored_ids_; }

private:
  std::unordered_map<int, LabelEntry> entries_;
  std::vector<int> ignored_ids_;
  int max_class_id_{0};
};

}  // namespace fyt::auto_aim

#endif
