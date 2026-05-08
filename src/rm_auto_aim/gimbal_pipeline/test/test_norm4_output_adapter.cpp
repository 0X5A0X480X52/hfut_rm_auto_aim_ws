#include <gtest/gtest.h>

#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_output_adapter.hpp"

namespace {

using fyt::auto_aim::UnifiedConfig;
using fyt::auto_aim::mode::TrackMode;
using fyt::auto_aim::norm4_v2::Norm4OutputAdapter;
using fyt::auto_aim::norm4_v2::Norm4RuntimeContext;

void rpyFromMessage(
    const geometry_msgs::msg::Quaternion &msg,
    double &roll,
    double &pitch,
    double &yaw) {
  tf2::Quaternion q;
  tf2::fromMsg(msg, q);
  q.normalize();
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

}  // namespace

TEST(Norm4OutputAdapter, StructuredModeBuildsRuntimeArmorOffsets) {
  UnifiedConfig cfg = UnifiedConfig::create_default();
  cfg.tracker.panel_angle_step = M_PI / 2.0;
  Norm4OutputAdapter adapter(cfg);

  Norm4RuntimeContext ctx;
  ctx.mode = TrackMode::STRUCTURED;
  ctx.r1 = 0.18;
  ctx.r2 = 0.27;
  ctx.dza = 0.045;

  const auto offsets = adapter.build_armors_offset_for_message(ctx);
  ASSERT_EQ(offsets.size(), 4u);

  EXPECT_NEAR(offsets[0].position.x, 0.18, 1e-9);
  EXPECT_NEAR(offsets[0].position.y, 0.0, 1e-9);
  EXPECT_NEAR(offsets[0].position.z, -0.045, 1e-9);

  EXPECT_NEAR(offsets[1].position.x, 0.0, 1e-9);
  EXPECT_NEAR(offsets[1].position.y, 0.27, 1e-9);
  EXPECT_NEAR(offsets[1].position.z, 0.045, 1e-9);

  EXPECT_NEAR(offsets[2].position.x, -0.18, 1e-9);
  EXPECT_NEAR(offsets[2].position.y, 0.0, 1e-9);
  EXPECT_NEAR(offsets[2].position.z, -0.045, 1e-9);

  EXPECT_NEAR(offsets[3].position.x, 0.0, 1e-9);
  EXPECT_NEAR(offsets[3].position.y, -0.27, 1e-9);
  EXPECT_NEAR(offsets[3].position.z, 0.045, 1e-9);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  rpyFromMessage(offsets[1].orientation, roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0, 1e-9);
  EXPECT_NEAR(pitch, 0.2618, 1e-6);
  EXPECT_NEAR(yaw, M_PI / 2.0, 1e-9);
}

TEST(Norm4OutputAdapter, AmbiguousModeUsesSingleZeroOffset) {
  UnifiedConfig cfg = UnifiedConfig::create_default();
  Norm4OutputAdapter adapter(cfg);

  Norm4RuntimeContext ctx;
  ctx.mode = TrackMode::AMBIGUOUS;
  ctx.r1 = 0.18;
  ctx.r2 = 0.27;
  ctx.dza = 0.045;

  const auto offsets = adapter.build_armors_offset_for_message(ctx);
  ASSERT_EQ(offsets.size(), 1u);
  EXPECT_NEAR(offsets[0].position.x, 0.0, 1e-12);
  EXPECT_NEAR(offsets[0].position.y, 0.0, 1e-12);
  EXPECT_NEAR(offsets[0].position.z, 0.0, 1e-12);
}
