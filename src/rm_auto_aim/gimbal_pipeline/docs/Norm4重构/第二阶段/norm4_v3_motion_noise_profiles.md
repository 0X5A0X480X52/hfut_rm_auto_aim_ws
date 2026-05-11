# Norm4 V3 Motion/Noise Profile 文件化配置

## 1) 完整示例（backend_config 直接指向文件）

```yaml
norm4_v3:
  backend_config:
    backend_type: "ukf_v2"   # ukf_v1 | ukf_v2 | inekf
    motion_profile: "src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/motion/default.yaml"
    noise_profile: "src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/noise/default.yaml"
    structure_profile: "slow" # slow | snapshot
```

## 2) motion_profile 可用默认文件

- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/motion/default.yaml`
  - native: `translation=singer`, `rotation=cv`
- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/motion/cv_cv.yaml`
  - native: `translation=cv`, `rotation=cv`
- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/motion/ca_ca.yaml`
  - native: `translation=ca`, `rotation=ca`
- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/motion/singer_cv.yaml`
  - native: `translation=singer`, `rotation=cv`
- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/motion/imm_xy_cv_ca_ctrv_cs__z_cv__yaw_cv.yaml`
  - IMM-xy: CV+CA+CTRV+CS, z: CV, yaw: CV
- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/motion/imm_xy_cv_ca_ctrv_cs__z_ca__yaw_ca.yaml`
  - IMM-xy: CV+CA+CTRV+CS, z: CA, yaw: CA
- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/motion/imm_xy_cv_ca_ctrv_cs__z_cs__yaw_cs.yaml`
  - IMM-xy: CV+CA+CTRV+CS, z: CS, yaw: CS

## 3) noise_profile 可用默认文件

- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/noise/default.yaml`
- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/noise/high_precision.yaml`
- `src/rm_auto_aim/gimbal_pipeline/config/norm4_v3/profiles/noise/high_robust.yaml`

## 4) 文件格式约定

motion 文件（native）:
```yaml
motion:
  type: native
  native:
    translation_model: cv|ca|singer
    rotation_model: cv|ca
    translation:
      cv_process_noise_vel: 0.5
      ca_process_noise_acc: 1.0
      singer_alpha: 0.5
      singer_sigma: 2.0
    rotation:
      cv_process_noise_rate: 0.5
      ca_process_noise_acc: 1.0
    structural:
      process_noise_r: 0.008
      process_noise_dz: 0.003
```

motion 文件（IMM）:
```yaml
motion:
  type: imm
  imm:
    enable_cv: true
    enable_ca: true
    enable_cs: true
    enable_ctrv: true
    q_cv: 0.5
    q_ca: 1.0
    q_z_vel: 0.8
    q_yaw_rate: 0.6
    cs_alpha: 0.5
    cs_a_max: 10.0
    p_stay: 0.82
    p_switch: 0.06
    z_model: cv|ca|cs
    yaw_model: cv|ca|cs
    q_r: 0.02
    q_dza: 0.005
    r_pos_base: 0.01
    r_yaw_base: 0.03
```

noise 文件:
```yaml
noise:
  sigma_pos_xy: 0.06
  sigma_pos_z: 0.08
  sigma_yaw: 0.12
  dual_raw_R_scale: 1.5
```
