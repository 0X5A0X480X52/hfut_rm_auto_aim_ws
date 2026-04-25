# TrackedRobot Domain Redesign in gimbal_pipeline

## 1. Goal

Unify all TrackedRobot related logic inside robot_description so that callers in controller/strategy/node layers do not need repeated manual conversion and scattered prediction code.

Design targets:
- One domain entry for state normalization, prediction, armor geometry, and representation conversion.
- Keep backward compatibility with existing TrackedRobot fields.
- Provide multi-output forms for computation and ROS communication:
  - Eigen::Vector3d and std::vector<Eigen::Vector3d>
  - geometry_msgs::msg::Point and std::vector<geometry_msgs::msg::Point>
  - rm_interfaces::msg::TrackedRobot

## 2. Current Pain Points (confirmed in code)

1) Domain logic is fragmented across multiple modules.
- robot_description has state sync + basic getters only.
- ArmorPositionCalculator owns geometry prediction.
- MpcReferenceGenerator owns second order state propagation.
- Strategies still compose center/velocity/yaw prediction by hand.

2) Conversion and prediction are duplicated in hot paths.
- current_position_strategy: rebuild center and velocity vectors manually.
- predicted_position_strategy: repeated center and velocity prediction.
- mpc_reference_generator: repeated center extraction and rebuild.

3) No single source of truth for motion model level.
- First order prediction and second order prediction are split in different classes.
- Caller decides where to apply dt and how to compose yaw/yaw rate.

## 3. Existing Capability Baseline

Current robot_description API already provides:
- normalizeState
- syncFullStateFromLegacy / syncLegacyStateFromFull
- centerPosition / linearVelocity / linearAcceleration
- yaw / yawVelocity / yawAcceleration
- resolveOffsets / generateArmorsOffsetFromProfile

This is a good base for building a full domain layer.

## 4. Proposed Architecture

Keep RobotDescriptionFacade as external entry, and extend TrackedRobotUsage into explicit domain sub-capabilities.

### 4.1 New capability groups under robot_description

A) State Views and Conversion
- Provide direct conversion helpers between Eigen and ROS message types.
- Provide explicit output forms so callers choose output type instead of rebuilding vectors.

B) Motion Model
- Provide first order and second order propagation on TrackedRobot.
- Keep full-state/legacy synchronization internal.

C) Armor Geometry
- Provide armor world position generation from robot state and dt.
- Offer both Eigen and geometry_msgs output forms.

### 4.2 API Sketch (to be added in robot_description headers)

State/Conversion:
- static Eigen::Vector3d toEigen(const geometry_msgs::msg::Point & p)
- static Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 & v)
- static geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & v)
- static geometry_msgs::msg::Vector3 toVector3(const Eigen::Vector3d & v)

Motion:
- enum class MotionModel { CONSTANT_VELOCITY, CONSTANT_ACCELERATION }
- static rm_interfaces::msg::TrackedRobot predict(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model)
- static Eigen::Vector3d predictCenterEigen(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model)
- static double predictYaw(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model)

Geometry:
- struct ArmorWorldPositions {
    std::vector<Eigen::Vector3d> eigen;
    std::vector<geometry_msgs::msg::Point> points;
  };
- static ArmorWorldPositions calculateArmorWorldPositions(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model,
    const OffsetFallbackGenerator & fallback_generator)
- static std::vector<Eigen::Vector3d> calculateArmorWorldPositionsEigen(...)
- static std::vector<geometry_msgs::msg::Point> calculateArmorWorldPositionsPoints(...)

## 5. Ownership Boundary After Redesign

Owned by robot_description:
- TrackedRobot state normalization/sync.
- State propagation with dt (first and second order).
- Armor offset resolution and world transform.
- Representation conversion helpers.

Owned by controller layer:
- Armor selection policy.
- Ballistic compensation.
- MPC optimization and constraints.
- Fire decision and control command construction.

Owned by MPC strategy only:
- Velocity clamp policy and all QP weight adaptation logic.

## 6. Migration Plan

Phase 1: Introduce domain APIs without changing callers.
- Add new headers/cpps under common/robot_description.
- Keep existing TrackedRobotUsage API intact.

Phase 2: Refactor ArmorPositionCalculator into thin adapter.
- calculate and calculatePredicted delegate to TrackedRobotUsage geometry API.
- Keep class signature unchanged.

Phase 3: Refactor MpcReferenceGenerator propagation path.
- Replace local propagateRobot with TrackedRobotUsage::predict(..., CONSTANT_ACCELERATION).
- Keep applyVelocityClamp local to MPC.

Phase 4: Refactor strategy files.
- Remove repeated manual center and velocity reconstruction.
- Replace by predictCenterEigen and calculateArmorWorldPositionsEigen.

Phase 5: Node and debug path cleanup.
- gimbal_pipeline_node uses conversion and accessor helpers directly.
- Ensure logger outputs remain unchanged.

Phase 6: Optional cleanup.
- Mark old helper names as compatibility wrappers.
- Add deprecation notes for direct legacy field access in strategy code.

## 7. Compatibility Rules

- Do not remove legacy fields from rm_interfaces::msg::TrackedRobot.
- Any API producing rm_interfaces::msg::TrackedRobot must call syncFullStateFromLegacy or syncLegacyStateFromFull to keep both views consistent.
- Existing public class signatures in gimbal_controller remain valid during migration.

## 8. Validation Matrix

Build checks:
- gimbal_pipeline library build
- test_mpc_core build

Behavior checks:
- Selection index continuity compared to baseline bag replay
- Reference trajectory continuity for MPC
- Fire advice stability in same replay

Data consistency checks:
- full_state_valid true after normalize/predict outputs
- legacy and full fields consistent for center/yaw/yaw_rate

## 9. Initial High Priority Refactor Targets

1) gimbal_controller/mpc/mpc_reference_generator.cpp
- Move propagateRobot responsibility to robot_description.

2) gimbal_controller/armor_position_calculator.cpp
- Move predicted geometry logic to robot_description.

3) gimbal_controller/strategies/current_position_strategy.cpp
- Replace manual predicted_center and target_velocity rebuild.

4) gimbal_controller/strategies/predicted_position_strategy.cpp
- Replace repeated predicted center and extra center rebuild.

## 10. Non-goals

- No behavior change in selector policy.
- No behavior change in fire advisor thresholds.
- No immediate change to message schema.
