// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_BACKEND_FACTORY_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_BACKEND_FACTORY_HPP_

#include <memory>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/process_models/composite.hpp"
#include "max_entropy_tracker/filters/process_models/rotation.hpp"
#include "max_entropy_tracker/filters/process_models/structural.hpp"
#include "max_entropy_tracker/filters/process_models/translation.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/interfaces/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/interfaces/norm4_measurement_noise.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/models/norm4_motion_model_bundle.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/backends/norm4_inekf_backend.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/backends/norm4_single_armor_imm_bundle.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/models/norm4_structure_provider.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/backends/norm4_ukf_backend_v1_adapter.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/backends/norm4_ukf_backend_v2.hpp"

namespace fyt::auto_aim::norm4_v3 {

enum class BackendType { UKF_V1, UKF_V2, INEKF };

inline BackendType backend_type_from_string(const std::string &s) {
  if (s == "ukf_v1") return BackendType::UKF_V1;
  if (s == "ukf_v2") return BackendType::UKF_V2;
  if (s == "inekf") return BackendType::INEKF;
  throw std::invalid_argument("Unknown backend type: " + s);
}

inline const Norm4V3UkfConfig &select_ukf_config(const UnifiedConfig &config,
                                                  BackendType type) {
  if (type == BackendType::UKF_V2 && config.norm4_v3.ukf_v2.enabled) {
    return config.norm4_v3.ukf_v2;
  }
  if (type == BackendType::INEKF && config.norm4_v3.inekf.enabled) {
    return config.norm4_v3.inekf;
  }
  return config.norm4_v3.ukf_v1;
}

inline bool is_profile_file(const std::string &profile) {
  return profile.find('/') != std::string::npos ||
         profile.find(".yaml") != std::string::npos ||
         profile.find(".yml") != std::string::npos;
}

inline std::string resolve_profile_path(const std::string &profile) {
  if (!is_profile_file(profile)) return profile;
  std::filesystem::path p(profile);
  if (p.is_absolute()) return p.string();
  std::filesystem::path cwd = std::filesystem::current_path();
  return (cwd / p).lexically_normal().string();
}

inline std::unique_ptr<IMotionModelBundle> create_motion_bundle_from_file(
    const UnifiedConfig &cfg, const std::string &profile_file) {
  YAML::Node root = YAML::LoadFile(resolve_profile_path(profile_file));
  auto motion = root["motion"];
  if (!motion) throw std::invalid_argument("motion profile missing motion node");
  std::string type = motion["type"] ? motion["type"].as<std::string>() : "native";

  if (type == "imm") {
    ImmBundleConfig icfg;
    auto imm = motion["imm"];
    if (!imm) throw std::invalid_argument("imm motion profile missing imm node");
    if (imm["enable_cv"]) icfg.enable_cv = imm["enable_cv"].as<bool>();
    if (imm["enable_ca"]) icfg.enable_ca = imm["enable_ca"].as<bool>();
    if (imm["enable_cs"]) icfg.enable_cs = imm["enable_cs"].as<bool>();
    if (imm["enable_ctrv"]) icfg.enable_ctrv = imm["enable_ctrv"].as<bool>();
    if (imm["q_cv"]) icfg.q_cv = imm["q_cv"].as<double>();
    if (imm["q_ca"]) icfg.q_ca = imm["q_ca"].as<double>();
    if (imm["q_z_vel"]) icfg.q_z_vel = imm["q_z_vel"].as<double>();
    if (imm["q_yaw_rate"]) icfg.q_yaw_rate = imm["q_yaw_rate"].as<double>();
    if (imm["cs_alpha"]) icfg.cs_alpha = imm["cs_alpha"].as<double>();
    if (imm["cs_a_max"]) icfg.cs_a_max = imm["cs_a_max"].as<double>();
    if (imm["p_stay"]) icfg.p_stay = imm["p_stay"].as<double>();
    if (imm["p_switch"]) icfg.p_switch = imm["p_switch"].as<double>();
    if (imm["z_model"]) icfg.z_model = imm["z_model"].as<std::string>();
    if (imm["yaw_model"]) icfg.yaw_model = imm["yaw_model"].as<std::string>();
    if (imm["q_r"]) icfg.q_r = imm["q_r"].as<double>();
    if (imm["q_dza"]) icfg.q_dza = imm["q_dza"].as<double>();
    if (imm["r_pos_base"]) icfg.r_pos_base = imm["r_pos_base"].as<double>();
    if (imm["r_yaw_base"]) icfg.r_yaw_base = imm["r_yaw_base"].as<double>();
    return std::make_unique<SingleArmorIMMBundle>(icfg);
  }

  TranslationConfig tc;
  tc.cv_process_noise_vel = cfg.motion.cv_process_noise_vel;
  tc.ca_process_noise_acc = cfg.motion.ca_process_noise_acc;
  tc.singer_alpha = cfg.motion.singer_alpha;
  tc.singer_sigma = cfg.motion.singer_sigma;
  RotationConfig rc;
  rc.cv_process_noise_rate = cfg.spin.spin_process_noise_delta_rate;
  rc.ca_process_noise_acc = cfg.spin.spin_process_noise_delta_acc;
  StructuralConfig sc;
  sc.process_noise_r = cfg.motion.process_noise_r;
  sc.process_noise_dz = cfg.motion.process_noise_dz;

  TranslationModel translation = cfg.motion.translation_model;
  RotationModel rotation = RotationModel::CV;

  auto native = motion["native"];
  if (native) {
    if (native["translation_model"]) {
      translation = translation_model_from_string(
          native["translation_model"].as<std::string>());
    }
    if (native["rotation_model"]) {
      rotation = rotation_model_from_string(
          native["rotation_model"].as<std::string>());
    }
    if (native["translation"]) {
      auto t = native["translation"];
      if (t["cv_process_noise_vel"])
        tc.cv_process_noise_vel = t["cv_process_noise_vel"].as<double>();
      if (t["ca_process_noise_acc"])
        tc.ca_process_noise_acc = t["ca_process_noise_acc"].as<double>();
      if (t["singer_alpha"]) tc.singer_alpha = t["singer_alpha"].as<double>();
      if (t["singer_sigma"]) tc.singer_sigma = t["singer_sigma"].as<double>();
    }
    if (native["rotation"]) {
      auto r = native["rotation"];
      if (r["cv_process_noise_rate"])
        rc.cv_process_noise_rate = r["cv_process_noise_rate"].as<double>();
      if (r["ca_process_noise_acc"])
        rc.ca_process_noise_acc = r["ca_process_noise_acc"].as<double>();
    }
    if (native["structural"]) {
      auto s = native["structural"];
      if (s["process_noise_r"]) sc.process_noise_r = s["process_noise_r"].as<double>();
      if (s["process_noise_dz"]) sc.process_noise_dz = s["process_noise_dz"].as<double>();
    }
  }
  auto proc_model = create_default_process_model(translation, rotation, tc, rc, sc, 3);
  return std::make_unique<NativeProcessModelBundle>(proc_model);
}

inline Norm4V3UkfConfig load_noise_profile_or_default(
    const Norm4V3UkfConfig &base, const std::string &noise_profile) {
  if (!is_profile_file(noise_profile) || noise_profile == "default") return base;
  YAML::Node root = YAML::LoadFile(resolve_profile_path(noise_profile));
  auto n = root["noise"];
  if (!n) throw std::invalid_argument("noise profile missing noise node");
  Norm4V3UkfConfig out = base;
  if (n["sigma_pos_xy"]) out.sigma_pos_xy = n["sigma_pos_xy"].as<double>();
  if (n["sigma_pos_z"]) out.sigma_pos_z = n["sigma_pos_z"].as<double>();
  if (n["sigma_yaw"]) out.sigma_yaw = n["sigma_yaw"].as<double>();
  if (n["dual_raw_R_scale"]) out.dual_raw_R_scale = n["dual_raw_R_scale"].as<double>();
  return out;
}

inline std::shared_ptr<CompositeProcessModel> create_v2_process_model_by_profile(
    const UnifiedConfig &cfg, const std::string &profile) {
  TranslationConfig tc;
  tc.cv_process_noise_vel = cfg.motion.cv_process_noise_vel;
  tc.ca_process_noise_acc = cfg.motion.ca_process_noise_acc;
  tc.singer_alpha = cfg.motion.singer_alpha;
  tc.singer_sigma = cfg.motion.singer_sigma;

  RotationConfig rc;
  rc.cv_process_noise_rate = cfg.spin.spin_process_noise_delta_rate;
  rc.ca_process_noise_acc = cfg.spin.spin_process_noise_delta_acc;

  StructuralConfig sc;
  sc.process_noise_r = cfg.motion.process_noise_r;
  sc.process_noise_dz = cfg.motion.process_noise_dz;

  TranslationModel translation = cfg.motion.translation_model;
  RotationModel rotation = RotationModel::CV;
  if (profile == "cv") {
    translation = TranslationModel::CV;
  } else if (profile == "ca") {
    translation = TranslationModel::CA;
  } else if (profile == "singer") {
    translation = TranslationModel::SINGER;
  } else if (profile == "yaw_ca") {
    rotation = RotationModel::CA;
  } else if (profile != "default") {
    throw std::invalid_argument("Unsupported motion_profile: " + profile);
  }
  return create_default_process_model(translation, rotation, tc, rc, sc, 3);
}

inline std::unique_ptr<IStructuredBackend> create_backend(
    BackendType type, const UnifiedConfig &config, double dt) {
  const auto &base_ukf_cfg = select_ukf_config(config, type);
  Norm4V3UkfConfig ukf_cfg = load_noise_profile_or_default(
      base_ukf_cfg, config.norm4_v3.backend_config.noise_profile);
  switch (type) {
    case BackendType::UKF_V1:
      return std::make_unique<UkfBackendV1Adapter>(config, dt);
    case BackendType::UKF_V2: {
      std::unique_ptr<IMotionModelBundle> motion;
      if (is_profile_file(config.norm4_v3.backend_config.motion_profile)) {
        motion = create_motion_bundle_from_file(
            config, config.norm4_v3.backend_config.motion_profile);
      } else {
        auto proc_model = create_v2_process_model_by_profile(
            config, config.norm4_v3.backend_config.motion_profile);
        motion = std::make_unique<NativeProcessModelBundle>(proc_model);
      }
      auto noise = std::make_unique<FixedCartesianNoiseModel>(ukf_cfg);
      return std::make_unique<UkfBackendV2>(std::move(motion),
                                            std::move(noise), ukf_cfg, config,
                                            dt);
    }
    case BackendType::INEKF: {
      std::unique_ptr<IMotionModelBundle> motion;
      if (is_profile_file(config.norm4_v3.backend_config.motion_profile)) {
        motion = create_motion_bundle_from_file(
            config, config.norm4_v3.backend_config.motion_profile);
      } else {
        auto proc_model = create_v2_process_model_by_profile(
            config, config.norm4_v3.backend_config.motion_profile);
        motion = std::make_unique<NativeProcessModelBundle>(proc_model);
      }
      auto noise = std::make_unique<FixedCartesianNoiseModel>(ukf_cfg);
      std::unique_ptr<IStructureProvider> structure;
      const auto &bc = config.norm4_v3.backend_config;
      if (!config.norm4_v3.slow_structure.enable ||
          bc.structure_profile == "snapshot") {
        structure = std::make_unique<UkfSnapshotStructureProvider>(*motion);
      } else {
        SlowStructureErrorUpdaterProvider::Config scfg;
        const auto &cfg = config.norm4_v3.slow_structure;
        scfg.q_theta_r1 = cfg.q_theta_r1;
        scfg.q_theta_r2 = cfg.q_theta_r2;
        scfg.q_theta_dza = cfg.q_theta_dza;
        scfg.prior_r1 = cfg.prior_r1;
        scfg.prior_r2 = cfg.prior_r2;
        scfg.prior_dza = cfg.prior_dza;
        scfg.prior_sigma_r = cfg.prior_sigma_r;
        scfg.prior_sigma_dza = cfg.prior_sigma_dza;
        scfg.alpha_r1_single = cfg.alpha_r1_single;
        scfg.alpha_r2_single = cfg.alpha_r2_single;
        scfg.alpha_dza_single = cfg.alpha_dza_single;
        scfg.alpha_r1_dual = cfg.alpha_r1_dual;
        scfg.alpha_r2_dual = cfg.alpha_r2_dual;
        scfg.alpha_dza_dual = cfg.alpha_dza_dual;
        scfg.prior_pull_gain = cfg.prior_pull_gain;
        scfg.min_r = cfg.min_r;
        scfg.max_r = cfg.max_r;
        scfg.min_dza = cfg.min_dza;
        scfg.max_dza = cfg.max_dza;
        structure =
            std::make_unique<SlowStructureErrorUpdaterProvider>(scfg, *motion);
      }
      return std::make_unique<InvariantPoseBackend>(std::move(motion),
                                                    std::move(noise),
                                                    std::move(structure),
                                                    ukf_cfg, config, dt);
    }
  }
  throw std::invalid_argument("Unknown backend type");
}

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_BACKEND_FACTORY_HPP_
