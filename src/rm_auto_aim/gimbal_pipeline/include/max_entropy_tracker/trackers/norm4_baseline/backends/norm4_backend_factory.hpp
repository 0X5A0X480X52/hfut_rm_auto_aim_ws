// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_BACKEND_FACTORY_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_BACKEND_FACTORY_HPP_

#include <memory>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

#include "rm_utils/url_resolver.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/process_models/composite.hpp"
#include "max_entropy_tracker/filters/process_models/rotation.hpp"
#include "max_entropy_tracker/filters/process_models/structural.hpp"
#include "max_entropy_tracker/filters/process_models/translation.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/interfaces/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/interfaces/norm4_ba_aware_ypd_noise.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/interfaces/norm4_measurement_noise.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/models/norm4_motion_model_bundle.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/backends/norm4_inekf_backend.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/backends/norm4_single_armor_imm_bundle.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/models/norm4_structure_provider.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/backends/norm4_ukf_backend_baseline_adapter.hpp"

namespace fyt::auto_aim::norm4_baseline {

enum class BackendType { UKF_BASELINE, INEKF };

inline BackendType backend_type_from_string(const std::string &s) {
  if (s == "ukf_baseline") return BackendType::UKF_BASELINE;
  if (s == "inekf") return BackendType::INEKF;
  throw std::invalid_argument("Unknown backend type: " + s);
}

inline const Norm4BaselineUkfConfig &select_ukf_config(const UnifiedConfig &config,
                                                  BackendType type) {
  if (type == BackendType::INEKF && config.norm4_baseline.inekf.enabled) {
    return config.norm4_baseline.inekf;
  }
  return config.norm4_baseline.ukf_baseline;
}

inline bool is_profile_file(const std::string &profile) {
  return profile.find('/') != std::string::npos ||
         profile.find(".yaml") != std::string::npos ||
         profile.find(".yml") != std::string::npos;
}

inline std::string resolve_profile_path(const std::string &profile) {
  const bool is_package_url = profile.rfind("package://", 0) == 0;
  const bool is_file_url = profile.rfind("file://", 0) == 0;
  if (is_package_url || is_file_url) {
    return fyt::utils::URLResolver::getResolvedPath(profile).string();
  }

  if (!is_profile_file(profile)) {
    if (profile == "default") return profile;
    std::filesystem::path alias =
        std::filesystem::current_path() /
        "src/rm_auto_aim/gimbal_pipeline/config/norm4_baseline/profiles/noise" /
        (profile + ".yaml");
    return alias.lexically_normal().string();
  }
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

inline Norm4BaselineUkfConfig load_noise_profile_or_default(
    const Norm4BaselineUkfConfig &base, const std::string &noise_profile) {
  if (noise_profile == "default") return base;
  YAML::Node root = YAML::LoadFile(resolve_profile_path(noise_profile));
  auto n = root["noise"];
  if (!n) throw std::invalid_argument("noise profile missing noise node");
  Norm4BaselineUkfConfig out = base;
  if (n["sigma_pos_xy"]) out.sigma_pos_xy = n["sigma_pos_xy"].as<double>();
  if (n["sigma_pos_z"]) out.sigma_pos_z = n["sigma_pos_z"].as<double>();
  if (n["sigma_yaw"]) out.sigma_yaw = n["sigma_yaw"].as<double>();
  if (n["dual_raw_R_scale"]) out.dual_raw_R_scale = n["dual_raw_R_scale"].as<double>();
  return out;
}

inline MeasurementNoiseConfig parse_measurement_noise_config(
    const std::string &profile_file) {
  MeasurementNoiseConfig out;
  YAML::Node root = YAML::LoadFile(resolve_profile_path(profile_file));
  auto n = root["noise"];
  if (!n) return out;

  if (n["type"]) out.type = n["type"].as<std::string>();

  // r_fixed
  if (auto rf = n["r_fixed"]) {
    if (rf["sigma_x"]) out.r_fixed.sigma_x = rf["sigma_x"].as<double>();
    if (rf["sigma_y"]) out.r_fixed.sigma_y = rf["sigma_y"].as<double>();
    if (rf["sigma_z"]) out.r_fixed.sigma_z = rf["sigma_z"].as<double>();
    if (rf["sigma_yaw"]) out.r_fixed.sigma_yaw = rf["sigma_yaw"].as<double>();
  }

  // r_floor
  if (auto rf = n["r_floor"]) {
    if (rf["sigma_x"]) out.r_floor.sigma_x = rf["sigma_x"].as<double>();
    if (rf["sigma_y"]) out.r_floor.sigma_y = rf["sigma_y"].as<double>();
    if (rf["sigma_z"]) out.r_floor.sigma_z = rf["sigma_z"].as<double>();
    if (rf["sigma_yaw"]) out.r_floor.sigma_yaw = rf["sigma_yaw"].as<double>();
  }

  // dynamic_blend
  if (auto db = n["dynamic_blend"]) {
    if (db["lambda"]) out.dynamic_blend.lambda = db["lambda"].as<double>();
  }

  // camera
  if (auto cam = n["camera"]) {
    if (cam["source"]) out.camera.source = cam["source"].as<std::string>();
    if (cam["fx"]) out.camera.fx = cam["fx"].as<double>();
    if (cam["fy"]) out.camera.fy = cam["fy"].as<double>();
    if (cam["cx"]) out.camera.cx = cam["cx"].as<double>();
    if (cam["cy"]) out.camera.cy = cam["cy"].as<double>();
    if (cam["image_width"]) out.camera.image_width = cam["image_width"].as<int>();
    if (cam["image_height"]) out.camera.image_height = cam["image_height"].as<int>();
  }

  // armor_geometry
  if (auto ag = n["armor_geometry"]) {
    if (auto s = ag["small"]) {
      if (s["width"]) out.armor_geometry.small_width = s["width"].as<double>();
      if (s["height"]) out.armor_geometry.small_height = s["height"].as<double>();
    }
    if (auto l = ag["large"]) {
      if (l["width"]) out.armor_geometry.large_width = l["width"].as<double>();
      if (l["height"]) out.armor_geometry.large_height = l["height"].as<double>();
    }
    if (auto o = ag["outpost"]) {
      if (o["width"]) out.armor_geometry.outpost_width = o["width"].as<double>();
      if (o["height"]) out.armor_geometry.outpost_height = o["height"].as<double>();
    }
  }

  // ypd_prior
  if (auto yp = n["ypd_prior"]) {
    if (yp["sigma_center_px"]) out.ypd_prior.sigma_center_px = yp["sigma_center_px"].as<double>();
    if (yp["sigma_size_px"]) out.ypd_prior.sigma_size_px = yp["sigma_size_px"].as<double>();
    if (yp["sigma_corner_px"]) out.ypd_prior.sigma_corner_px = yp["sigma_corner_px"].as<double>();
    if (yp["sigma_azi_min"]) out.ypd_prior.sigma_azi_min = yp["sigma_azi_min"].as<double>();
    if (yp["sigma_azi_max"]) out.ypd_prior.sigma_azi_max = yp["sigma_azi_max"].as<double>();
    if (yp["sigma_ele_min"]) out.ypd_prior.sigma_ele_min = yp["sigma_ele_min"].as<double>();
    if (yp["sigma_ele_max"]) out.ypd_prior.sigma_ele_max = yp["sigma_ele_max"].as<double>();
    if (yp["sigma_dist_min"]) out.ypd_prior.sigma_dist_min = yp["sigma_dist_min"].as<double>();
    if (yp["sigma_dist_max"]) out.ypd_prior.sigma_dist_max = yp["sigma_dist_max"].as<double>();
    if (yp["sigma_yaw_min"]) out.ypd_prior.sigma_yaw_min = yp["sigma_yaw_min"].as<double>();
    if (yp["sigma_yaw_max"]) out.ypd_prior.sigma_yaw_max = yp["sigma_yaw_max"].as<double>();
    if (yp["sigma_yaw_scale"]) out.ypd_prior.sigma_yaw_scale = yp["sigma_yaw_scale"].as<double>();
    if (yp["global_scale"]) out.ypd_prior.global_scale = yp["global_scale"].as<double>();
  }

  // quality_scale
  if (auto qs = n["quality_scale"]) {
    if (qs["enable"]) out.quality_scale.enable = qs["enable"].as<bool>();
    if (qs["confidence_floor"]) out.quality_scale.confidence_floor = qs["confidence_floor"].as<double>();
    if (qs["min_scale"]) out.quality_scale.min_scale = qs["min_scale"].as<double>();
    if (qs["max_scale"]) out.quality_scale.max_scale = qs["max_scale"].as<double>();
  }

  // ba_covariance
  if (auto ba = n["ba_covariance"]) {
    if (ba["enable"]) out.ba_covariance.enable = ba["enable"].as<bool>();
    if (ba["require_cov_valid"]) out.ba_covariance.require_cov_valid = ba["require_cov_valid"].as<bool>();
    if (ba["require_frame_aligned"]) out.ba_covariance.require_frame_aligned = ba["require_frame_aligned"].as<bool>();
    if (ba["min_confidence"]) out.ba_covariance.min_confidence = ba["min_confidence"].as<double>();
    if (ba["max_reproj_rms_px"]) out.ba_covariance.max_reproj_rms_px = ba["max_reproj_rms_px"].as<double>();
    if (ba["max_condition_number"]) out.ba_covariance.max_condition_number = ba["max_condition_number"].as<double>();
    if (ba["min_observations"]) out.ba_covariance.min_observations = ba["min_observations"].as<int>();
    if (ba["min_inlier_ratio"]) out.ba_covariance.min_inlier_ratio = ba["min_inlier_ratio"].as<double>();
    if (ba["max_weight"]) out.ba_covariance.max_weight = ba["max_weight"].as<double>();
    if (ba["weight_power"]) out.ba_covariance.weight_power = ba["weight_power"].as<double>();
    if (ba["scale"]) out.ba_covariance.scale = ba["scale"].as<double>();
    if (auto ec = ba["eigen_clamp"]) {
      if (ec["min"]) out.ba_covariance.eigen_clamp.min = ec["min"].as<double>();
      if (ec["max"]) out.ba_covariance.eigen_clamp.max = ec["max"].as<double>();
    }
    if (auto dc = ba["diag_clamp"]) {
      if (dc["x_min"]) out.ba_covariance.diag_clamp.x_min = dc["x_min"].as<double>();
      if (dc["y_min"]) out.ba_covariance.diag_clamp.y_min = dc["y_min"].as<double>();
      if (dc["z_min"]) out.ba_covariance.diag_clamp.z_min = dc["z_min"].as<double>();
      if (dc["yaw_min"]) out.ba_covariance.diag_clamp.yaw_min = dc["yaw_min"].as<double>();
      if (dc["x_max"]) out.ba_covariance.diag_clamp.x_max = dc["x_max"].as<double>();
      if (dc["y_max"]) out.ba_covariance.diag_clamp.y_max = dc["y_max"].as<double>();
      if (dc["z_max"]) out.ba_covariance.diag_clamp.z_max = dc["z_max"].as<double>();
      if (dc["yaw_max"]) out.ba_covariance.diag_clamp.yaw_max = dc["yaw_max"].as<double>();
    }
  }

  // debug
  if (auto dbg = n["debug"]) {
    if (dbg["enable_snapshot"]) out.debug.enable_snapshot = dbg["enable_snapshot"].as<bool>();
    if (dbg["log_throttle_ms"]) out.debug.log_throttle_ms = dbg["log_throttle_ms"].as<int>();
  }

  return out;
}

inline std::unique_ptr<IMeasurementNoiseModel> create_noise_model(
    const Norm4BaselineUkfConfig &ukf_cfg, const std::string &noise_profile) {
  if (noise_profile == "default") {
    return std::make_unique<FixedCartesianNoiseModel>(ukf_cfg);
  }

  auto noise_cfg = parse_measurement_noise_config(resolve_profile_path(noise_profile));
  if (noise_cfg.type == "ypd_ba") {
    return std::make_unique<BaAwareYpdNoiseModel>(noise_cfg, ukf_cfg);
  }
  return std::make_unique<FixedCartesianNoiseModel>(ukf_cfg);
}

inline UnifiedConfig apply_inekf_motion_overrides(const UnifiedConfig &config) {
  UnifiedConfig local = config;
  const auto &o = config.norm4_baseline.inekf_runtime;
  if (!o.translation_model.empty()) {
    local.motion.translation_model = translation_model_from_string(o.translation_model);
  }
  if (o.cv_process_noise_vel >= 0.0) local.motion.cv_process_noise_vel = o.cv_process_noise_vel;
  if (o.ca_process_noise_acc >= 0.0) local.motion.ca_process_noise_acc = o.ca_process_noise_acc;
  if (o.singer_alpha >= 0.0) local.motion.singer_alpha = o.singer_alpha;
  if (o.singer_sigma >= 0.0) local.motion.singer_sigma = o.singer_sigma;
  if (o.process_noise_r >= 0.0) local.motion.process_noise_r = o.process_noise_r;
  if (o.process_noise_dz >= 0.0) local.motion.process_noise_dz = o.process_noise_dz;
  if (o.spin_process_noise_delta_rate >= 0.0) {
    local.spin.spin_process_noise_delta_rate = o.spin_process_noise_delta_rate;
  }
  if (o.spin_process_noise_delta_acc >= 0.0) {
    local.spin.spin_process_noise_delta_acc = o.spin_process_noise_delta_acc;
  }
  return local;
}

inline std::unique_ptr<IStructuredBackend> create_backend(
    BackendType type, const UnifiedConfig &config, double dt) {
  const auto &base_ukf_cfg = select_ukf_config(config, type);
  switch (type) {
    case BackendType::UKF_BASELINE: {
      Norm4BaselineUkfConfig ukf_cfg = load_noise_profile_or_default(
          base_ukf_cfg, config.norm4_baseline.backend_config.noise_profile);
      (void)ukf_cfg;
      return std::make_unique<UkfBackendBaselineAdapter>(config, dt);
    }
    case BackendType::INEKF: {
      UnifiedConfig local_cfg = apply_inekf_motion_overrides(config);
      const std::string motion_profile =
          local_cfg.norm4_baseline.inekf_runtime.motion_profile.empty()
              ? local_cfg.norm4_baseline.backend_config.motion_profile
              : local_cfg.norm4_baseline.inekf_runtime.motion_profile;
      const std::string noise_profile =
          local_cfg.norm4_baseline.inekf_runtime.noise_profile.empty()
              ? local_cfg.norm4_baseline.backend_config.noise_profile
              : local_cfg.norm4_baseline.inekf_runtime.noise_profile;
      const std::string structure_profile =
          local_cfg.norm4_baseline.inekf_runtime.structure_profile.empty()
              ? local_cfg.norm4_baseline.backend_config.structure_profile
              : local_cfg.norm4_baseline.inekf_runtime.structure_profile;
      Norm4BaselineUkfConfig ukf_cfg =
          load_noise_profile_or_default(base_ukf_cfg, noise_profile);

      std::unique_ptr<IMotionModelBundle> motion;
      if (is_profile_file(motion_profile)) {
        motion = create_motion_bundle_from_file(local_cfg, motion_profile);
      } else {
        TranslationConfig tc;
        tc.cv_process_noise_vel = local_cfg.motion.cv_process_noise_vel;
        tc.ca_process_noise_acc = local_cfg.motion.ca_process_noise_acc;
        tc.singer_alpha = local_cfg.motion.singer_alpha;
        tc.singer_sigma = local_cfg.motion.singer_sigma;

        RotationConfig rc;
        rc.cv_process_noise_rate = local_cfg.spin.spin_process_noise_delta_rate;
        rc.ca_process_noise_acc = local_cfg.spin.spin_process_noise_delta_acc;

        StructuralConfig sc;
        sc.process_noise_r = local_cfg.motion.process_noise_r;
        sc.process_noise_dz = local_cfg.motion.process_noise_dz;

        TranslationModel translation = local_cfg.motion.translation_model;
        RotationModel rotation = RotationModel::CV;
        if (motion_profile == "cv") {
          translation = TranslationModel::CV;
        } else if (motion_profile == "ca") {
          translation = TranslationModel::CA;
        } else if (motion_profile == "singer") {
          translation = TranslationModel::SINGER;
        } else if (motion_profile == "yaw_ca") {
          rotation = RotationModel::CA;
        } else if (motion_profile != "default") {
          throw std::invalid_argument("Unsupported motion_profile: " +
                                      motion_profile);
        }
        // inekf_runtime.translation_model has the highest priority for InEKF.
        if (!local_cfg.norm4_baseline.inekf_runtime.translation_model.empty()) {
          translation = translation_model_from_string(
              local_cfg.norm4_baseline.inekf_runtime.translation_model);
        }
        auto proc_model =
            create_default_process_model(translation, rotation, tc, rc, sc, 3);
        motion = std::make_unique<NativeProcessModelBundle>(proc_model);
      }
      auto noise = create_noise_model(ukf_cfg, noise_profile);
      std::unique_ptr<IStructureProvider> structure;
      if (!local_cfg.norm4_baseline.slow_structure.enable ||
          structure_profile == "snapshot") {
        structure = std::make_unique<UkfSnapshotStructureProvider>(*motion);
      } else {
        SlowStructureErrorUpdaterProvider::Config scfg;
        const auto &cfg = local_cfg.norm4_baseline.slow_structure;
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
                                                    ukf_cfg, local_cfg, dt);
    }
  }
  throw std::invalid_argument("Unknown backend type");
}

}  // namespace fyt::auto_aim::norm4_baseline

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_BACKEND_FACTORY_HPP_
