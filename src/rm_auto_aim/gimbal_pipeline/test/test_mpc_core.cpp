// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "gimbal_controller/mpc/gimbal_dynamics_model.hpp"
#include "gimbal_controller/mpc/qp_solver.hpp"
#include "gimbal_controller/mpc/mpc_reference_generator.hpp"
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"

using gimbal_controller::mpc::GimbalDynamicsModel;
using gimbal_controller::mpc::QPSolver;
using gimbal_controller::mpc::QPResult;
using gimbal_controller::mpc::MpcReferenceGenerator;

// =====================================================================
//  TestGimbalDynamicsModel
// =====================================================================

class TestGimbalDynamicsModel : public ::testing::Test
{
protected:
  void SetUp() override
  {
    model_ = std::make_unique<GimbalDynamicsModel>(dt_);
  }

  double dt_{0.01};
  std::unique_ptr<GimbalDynamicsModel> model_;
};

TEST_F(TestGimbalDynamicsModel, MatrixDimensions)
{
  EXPECT_EQ(model_->A().rows(), 4);
  EXPECT_EQ(model_->A().cols(), 4);
  EXPECT_EQ(model_->B().rows(), 4);
  EXPECT_EQ(model_->B().cols(), 2);
}

TEST_F(TestGimbalDynamicsModel, MatrixValues)
{
  double dt = dt_;
  double dt2 = 0.5 * dt * dt;

  auto A = model_->A();
  // 对角线
  EXPECT_DOUBLE_EQ(A(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(A(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(A(2, 2), 1.0);
  EXPECT_DOUBLE_EQ(A(3, 3), 1.0);
  // yaw += yaw_dot * dt
  EXPECT_DOUBLE_EQ(A(0, 2), dt);
  // pitch += pitch_dot * dt
  EXPECT_DOUBLE_EQ(A(1, 3), dt);
  // 零元素
  EXPECT_DOUBLE_EQ(A(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(A(2, 3), 0.0);

  auto B = model_->B();
  EXPECT_DOUBLE_EQ(B(0, 0), dt2);
  EXPECT_DOUBLE_EQ(B(1, 1), dt2);
  EXPECT_DOUBLE_EQ(B(2, 0), dt);
  EXPECT_DOUBLE_EQ(B(3, 1), dt);
  EXPECT_DOUBLE_EQ(B(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(B(2, 1), 0.0);
}

TEST_F(TestGimbalDynamicsModel, ZeroInputStationaryState)
{
  // 静止状态 + 零控制输入 → 状态不变
  GimbalDynamicsModel::StateVector x0(0.5, -0.3, 0.0, 0.0);
  GimbalDynamicsModel::ControlVector u(0.0, 0.0);

  auto x1 = model_->predict(x0, u);
  EXPECT_NEAR(x1(0), 0.5, 1e-10);
  EXPECT_NEAR(x1(1), -0.3, 1e-10);
  EXPECT_NEAR(x1(2), 0.0, 1e-10);
  EXPECT_NEAR(x1(3), 0.0, 1e-10);
}

TEST_F(TestGimbalDynamicsModel, ConstantVelocityPropagation)
{
  // 初始有角速度，无控制输入 → 匀速运动
  double yaw0 = 0.0;
  double yaw_dot = 1.0;  // 1 rad/s
  GimbalDynamicsModel::StateVector x0(yaw0, 0.0, yaw_dot, 0.0);
  GimbalDynamicsModel::ControlVector u(0.0, 0.0);

  auto x1 = model_->predict(x0, u);
  EXPECT_NEAR(x1(0), yaw0 + yaw_dot * dt_, 1e-10);
  EXPECT_NEAR(x1(2), yaw_dot, 1e-10);
}

TEST_F(TestGimbalDynamicsModel, ConstantAccelerationTrajectory)
{
  // 从静止开始，恒定加速度 → 匀加速运动
  double a_yaw = 10.0;  // rad/s^2
  GimbalDynamicsModel::StateVector x0(0.0, 0.0, 0.0, 0.0);
  GimbalDynamicsModel::ControlVector u(a_yaw, 0.0);

  int steps = 100;
  auto x = x0;
  for (int i = 0; i < steps; ++i) {
    x = model_->predict(x, u);
  }

  double t = steps * dt_;
  double expected_yaw = 0.5 * a_yaw * t * t;
  double expected_yaw_dot = a_yaw * t;

  EXPECT_NEAR(x(0), expected_yaw, 1e-6);
  EXPECT_NEAR(x(2), expected_yaw_dot, 1e-6);
}

TEST_F(TestGimbalDynamicsModel, PredictionMatricesDimensions)
{
  int N = 10;
  Eigen::MatrixXd A_pred, B_pred;
  model_->buildPredictionMatrices(N, A_pred, B_pred);

  EXPECT_EQ(A_pred.rows(), 4 * N);
  EXPECT_EQ(A_pred.cols(), 4);
  EXPECT_EQ(B_pred.rows(), 4 * N);
  EXPECT_EQ(B_pred.cols(), 2 * N);
}

TEST_F(TestGimbalDynamicsModel, PredictionZeroInput)
{
  // 静止状态 + 零控制 → 所有预测状态不变
  int N = 5;
  GimbalDynamicsModel::StateVector x0(1.0, -0.5, 0.0, 0.0);
  Eigen::VectorXd U = Eigen::VectorXd::Zero(2 * N);

  Eigen::MatrixXd A_pred, B_pred;
  model_->buildPredictionMatrices(N, A_pred, B_pred);

  Eigen::VectorXd X_pred = A_pred * x0 + B_pred * U;

  for (int k = 0; k < N; ++k) {
    EXPECT_NEAR(X_pred(k * 4 + 0), 1.0, 1e-10);
    EXPECT_NEAR(X_pred(k * 4 + 1), -0.5, 1e-10);
    EXPECT_NEAR(X_pred(k * 4 + 2), 0.0, 1e-10);
    EXPECT_NEAR(X_pred(k * 4 + 3), 0.0, 1e-10);
  }
}

TEST_F(TestGimbalDynamicsModel, PredictionConstantAcceleration)
{
  // 恒定加速度控制输入 → 预测结果应与逐步 predict 一致
  int N = 10;
  GimbalDynamicsModel::StateVector x0(0.0, 0.0, 0.0, 0.0);
  double a_yaw = 5.0;
  Eigen::VectorXd U(2 * N);
  for (int k = 0; k < N; ++k) {
    U(2 * k) = a_yaw;
    U(2 * k + 1) = 0.0;
  }

  Eigen::MatrixXd A_pred, B_pred;
  model_->buildPredictionMatrices(N, A_pred, B_pred);
  Eigen::VectorXd X_pred = A_pred * x0 + B_pred * U;

  // 逐步预测做参考
  auto x = x0;
  for (int k = 0; k < N; ++k) {
    GimbalDynamicsModel::ControlVector u(a_yaw, 0.0);
    x = model_->predict(x, u);

    EXPECT_NEAR(X_pred(k * 4 + 0), x(0), 1e-8)
      << "yaw mismatch at step " << k;
    EXPECT_NEAR(X_pred(k * 4 + 2), x(2), 1e-8)
      << "yaw_dot mismatch at step " << k;
  }
}

TEST_F(TestGimbalDynamicsModel, DelayedBZeroDelay)
{
  // delay=0 时，B_d 应等于 B_pred
  int N = 8;
  Eigen::MatrixXd A_pred, B_pred;
  model_->buildPredictionMatrices(N, A_pred, B_pred);

  auto B_d = model_->buildDelayedB(N, 0.0);

  EXPECT_EQ(B_d.rows(), B_pred.rows());
  EXPECT_EQ(B_d.cols(), B_pred.cols());
  EXPECT_NEAR((B_d - B_pred).norm(), 0.0, 1e-10);
}

TEST_F(TestGimbalDynamicsModel, DelayedBFirstDStepsZero)
{
  // delay = 3 步 (0.03s @ dt=0.01) → u[0..2] 对前3步状态无影响
  int N = 10;
  int d = 3;
  double delay_s = d * dt_;

  auto B_d = model_->buildDelayedB(N, delay_s);

  // 检查: 对于行块 i < d，所有列块应为零
  for (int i = 0; i < d; ++i) {
    for (int j = 0; j < N; ++j) {
      auto block = B_d.block(i * 4, j * 2, 4, 2);
      EXPECT_NEAR(block.norm(), 0.0, 1e-10)
        << "Non-zero block at row-block " << i << ", col-block " << j;
    }
  }

  // 检查: 对于行块 i >= d，至少有一个非零列块
  for (int i = d; i < N; ++i) {
    double row_norm = 0;
    for (int j = 0; j <= i - d; ++j) {
      row_norm += B_d.block(i * 4, j * 2, 4, 2).norm();
    }
    EXPECT_GT(row_norm, 0.0)
      << "All-zero row-block " << i << " but should have non-zero entries";
  }
}

TEST_F(TestGimbalDynamicsModel, DifferenceMatrixDimensions)
{
  int N = 10;
  auto D = GimbalDynamicsModel::buildDifferenceMatrix(N);
  EXPECT_EQ(D.rows(), 2 * N);
  EXPECT_EQ(D.cols(), 2 * N);
}

TEST_F(TestGimbalDynamicsModel, DifferenceMatrixAction)
{
  int N = 4;
  auto D = GimbalDynamicsModel::buildDifferenceMatrix(N);

  // U = [1, 2, 3, 4, 5, 6, 7, 8] (4 steps, 2 controls each)
  Eigen::VectorXd U(8);
  U << 1, 2, 3, 4, 5, 6, 7, 8;

  Eigen::VectorXd dU = D * U;

  // 第一步: u[0] - 0 = [1, 2]
  EXPECT_DOUBLE_EQ(dU(0), 1.0);
  EXPECT_DOUBLE_EQ(dU(1), 2.0);
  // 第二步: u[1] - u[0] = [3-1, 4-2] = [2, 2]
  EXPECT_DOUBLE_EQ(dU(2), 2.0);
  EXPECT_DOUBLE_EQ(dU(3), 2.0);
  // 第三步: u[2] - u[1] = [5-3, 6-4] = [2, 2]
  EXPECT_DOUBLE_EQ(dU(4), 2.0);
  EXPECT_DOUBLE_EQ(dU(5), 2.0);
}

TEST_F(TestGimbalDynamicsModel, WeightMatrixDimensions)
{
  int N = 10;
  auto Q = GimbalDynamicsModel::buildWeightQ(N, 100, 100, 10, 10);
  auto R = GimbalDynamicsModel::buildWeightR(N, 0.01, 0.01);
  auto S = GimbalDynamicsModel::buildWeightS(N, 5.0, 5.0);

  EXPECT_EQ(Q.rows(), 4 * N);
  EXPECT_EQ(Q.cols(), 4 * N);
  EXPECT_EQ(R.rows(), 2 * N);
  EXPECT_EQ(R.cols(), 2 * N);
  EXPECT_EQ(S.rows(), 2 * N);
  EXPECT_EQ(S.cols(), 2 * N);
}

// =====================================================================
//  TestQPSolver
// =====================================================================

class TestQPSolver : public ::testing::Test
{
protected:
  void SetUp() override
  {
    solver_.init(200, 0.01);
  }

  QPSolver solver_;
};

TEST_F(TestQPSolver, Simple2VarUnconstrained)
{
  // min 0.5 * x^T H x + f^T x
  // H = [[2, 0], [0, 4]]
  // f = [-2, -4]
  // 解析解: x = H^{-1} * (-f) = [1, 1]
  Eigen::MatrixXd H(2, 2);
  H << 2, 0, 0, 4;
  Eigen::VectorXd f(2);
  f << -2, -4;

  // 宽松 box 约束
  Eigen::VectorXd lb(2), ub(2);
  lb << -100, -100;
  ub << 100, 100;

  auto result = solver_.solve(H, f, lb, ub);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.U(0), 1.0, 1e-6);
  EXPECT_NEAR(result.U(1), 1.0, 1e-6);
}

TEST_F(TestQPSolver, BoxConstrainedSolution)
{
  // min 0.5 * x^T I x + [-10, -10]^T x
  // 无约束解: x = [10, 10]
  // 带约束 0 <= x <= 5: 解应为 [5, 5]
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd f(2);
  f << -10, -10;

  Eigen::VectorXd lb(2), ub(2);
  lb << 0, 0;
  ub << 5, 5;

  auto result = solver_.solve(H, f, lb, ub);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.U(0), 5.0, 1e-6);
  EXPECT_NEAR(result.U(1), 5.0, 1e-6);
}

TEST_F(TestQPSolver, WithLinearConstraint)
{
  // min 0.5 * x^T I x + [0, 0]^T x
  // s.t. x1 + x2 <= 1, x >= -10, x <= 10
  // 解: x = [0, 0] (原点已满足约束且是最小值)
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd f = Eigen::VectorXd::Zero(2);

  Eigen::VectorXd lb(2), ub(2);
  lb << -10, -10;
  ub << 10, 10;

  Eigen::MatrixXd A_con(1, 2);
  A_con << 1, 1;
  Eigen::VectorXd lbA(1), ubA(1);
  lbA << -1e20;  // 无下界
  ubA << 1.0;    // x1 + x2 <= 1

  auto result = solver_.solve(H, f, lb, ub, A_con, lbA, ubA);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.U(0), 0.0, 1e-6);
  EXPECT_NEAR(result.U(1), 0.0, 1e-6);
}

TEST_F(TestQPSolver, ActiveLinearConstraint)
{
  // min 0.5 * x^T I x + [-3, -3]^T x
  // s.t. x1 + x2 <= 1, x >= -10, x <= 10
  // 无约束解: [3, 3]，但约束活跃
  // 解: x1 + x2 = 1, 对称 → x1 = x2 = 0.5
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd f(2);
  f << -3, -3;

  Eigen::VectorXd lb(2), ub(2);
  lb << -10, -10;
  ub << 10, 10;

  Eigen::MatrixXd A_con(1, 2);
  A_con << 1, 1;
  Eigen::VectorXd lbA(1), ubA(1);
  lbA << -1e20;
  ubA << 1.0;

  auto result = solver_.solve(H, f, lb, ub, A_con, lbA, ubA);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.U(0), 0.5, 1e-5);
  EXPECT_NEAR(result.U(1), 0.5, 1e-5);
}

TEST_F(TestQPSolver, HotstartConsistency)
{
  // 连续求解两次相同问题，验证结果一致
  Eigen::MatrixXd H(2, 2);
  H << 4, 1, 1, 2;
  Eigen::VectorXd f(2);
  f << -1, -1;
  Eigen::VectorXd lb(2), ub(2);
  lb << -10, -10;
  ub << 10, 10;

  auto r1 = solver_.solve(H, f, lb, ub);
  auto r2 = solver_.solve(H, f, lb, ub);  // hotstart

  ASSERT_TRUE(r1.success);
  ASSERT_TRUE(r2.success);
  EXPECT_NEAR(r1.U(0), r2.U(0), 1e-8);
  EXPECT_NEAR(r1.U(1), r2.U(1), 1e-8);
}

TEST_F(TestQPSolver, FourVariableMPC)
{
  // 模拟 1 步双轴 MPC: 4 变量 (2 步 × 2 轴)
  int n = 4;
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n, n) * 2.0;
  H(0, 2) = 0.1;
  H(2, 0) = 0.1;  // 保持对称
  Eigen::VectorXd f(n);
  f << -1, -2, -0.5, -1;

  Eigen::VectorXd lb = Eigen::VectorXd::Constant(n, -5.0);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(n, 5.0);

  auto result = solver_.solve(H, f, lb, ub);

  ASSERT_TRUE(result.success);
  // 验证所有解在约束范围内
  for (int i = 0; i < n; ++i) {
    EXPECT_GE(result.U(i), -5.0 - 1e-6);
    EXPECT_LE(result.U(i), 5.0 + 1e-6);
  }
}

// =====================================================================
//  TestMpcReferenceGenerator
// =====================================================================

class TestMpcReferenceGenerator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    position_calculator_ = std::make_shared<gimbal_controller::ArmorPositionCalculator>();
    armor_selector_ = std::make_shared<gimbal_controller::ArmorSelector>();
    local_compensator_ = std::make_shared<gimbal_controller::LocalTrajectoryCompensator>();
    local_compensator_->setParameters(20.0, 9.8, 0.001, 20);

    generator_.setComponents(position_calculator_, armor_selector_, local_compensator_);
  }

  rm_interfaces::msg::TrackedRobot makeStaticRobot(
    double x, double y, double z, int num_armors = 4)
  {
    rm_interfaces::msg::TrackedRobot robot;
    robot.center_position.x = x;
    robot.center_position.y = y;
    robot.center_position.z = z;
    robot.center_velocity.x = 0;
    robot.center_velocity.y = 0;
    robot.center_velocity.z = 0;
    robot.center_acceleration.x = 0;
    robot.center_acceleration.y = 0;
    robot.center_acceleration.z = 0;
    robot.yaw = 0.0;
    robot.yaw_velocity = 0.0;
    robot.yaw_acceleration = 0.0;
    robot.radius = 0.2;
    robot.radius_2 = 0.15;
    robot.d_za = 0.0;
    robot.d_zc = 0.0;
    robot.num_armors = num_armors;
    robot.robot_type = 1;  // STANDARD_4
    robot.track_state = 1;  // TRACKING

    // 生成默认装甲板偏移
    auto offsets = gimbal_controller::ArmorPositionCalculator::generateDefaultOffsets(
      robot.robot_type, num_armors, robot.radius, robot.radius_2,
      robot.d_za, robot.d_zc);

    robot.armors_offset.resize(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i) {
      robot.armors_offset[i].position.x = offsets[i].x();
      robot.armors_offset[i].position.y = offsets[i].y();
      robot.armors_offset[i].position.z = offsets[i].z();
    }

    return robot;
  }

  std::shared_ptr<gimbal_controller::ArmorPositionCalculator> position_calculator_;
  std::shared_ptr<gimbal_controller::ArmorSelector> armor_selector_;
  std::shared_ptr<gimbal_controller::LocalTrajectoryCompensator> local_compensator_;
  MpcReferenceGenerator generator_;
};

TEST_F(TestMpcReferenceGenerator, OutputDimension)
{
  int N = 10;
  double dt = 0.01;
  auto robot = makeStaticRobot(3.0, 0.0, 0.0);

  auto X_ref = generator_.generate(robot, 0.0, 0.0, N, dt);

  EXPECT_EQ(X_ref.size(), 4 * N);
}

TEST_F(TestMpcReferenceGenerator, StaticTargetConstantRef)
{
  int N = 10;
  double dt = 0.01;
  // 目标在正前方 3m 处
  auto robot = makeStaticRobot(3.0, 0.0, 0.0);

  auto X_ref = generator_.generate(robot, 0.0, 0.0, N, dt);

  // 静止目标 → 所有步的 yaw_ref/pitch_ref 应几乎相同
  double yaw0 = X_ref(0);
  double pitch0 = X_ref(1);

  for (int k = 1; k < N; ++k) {
    EXPECT_NEAR(X_ref(k * 4 + 0), yaw0, 1e-4)
      << "yaw_ref should be constant for static target at step " << k;
    EXPECT_NEAR(X_ref(k * 4 + 1), pitch0, 1e-4)
      << "pitch_ref should be constant for static target at step " << k;
  }
}

TEST_F(TestMpcReferenceGenerator, MovingTargetMonotonicity)
{
  int N = 20;
  double dt = 0.01;
  // 目标在右侧 3m，向右匀速运动 (y 增大)
  auto robot = makeStaticRobot(3.0, 1.0, 0.0);
  robot.center_velocity.y = 2.0;  // 2 m/s 向右

  auto X_ref = generator_.generate(robot, 0.0, 0.0, N, dt);

  // yaw_ref 应单调增大 (目标向右偏移)
  for (int k = 1; k < N; ++k) {
    EXPECT_GE(X_ref(k * 4 + 0), X_ref((k - 1) * 4 + 0) - 1e-6)
      << "yaw_ref should be monotonically increasing at step " << k;
  }
}

TEST_F(TestMpcReferenceGenerator, NoComponentsFallback)
{
  // 未注入组件 → 应返回 current_yaw/pitch 作为参考
  MpcReferenceGenerator gen_empty;
  int N = 5;
  double dt = 0.01;
  rm_interfaces::msg::TrackedRobot robot;

  auto X_ref = gen_empty.generate(robot, 0.5, -0.2, N, dt);

  EXPECT_EQ(X_ref.size(), 4 * N);
  for (int k = 0; k < N; ++k) {
    EXPECT_DOUBLE_EQ(X_ref(k * 4 + 0), 0.5);
    EXPECT_DOUBLE_EQ(X_ref(k * 4 + 1), -0.2);
  }
}

// =====================================================================
//  TestMpcEndToEnd
// =====================================================================

class TestMpcEndToEnd : public ::testing::Test
{
protected:
  void SetUp() override
  {
    model_ = std::make_unique<GimbalDynamicsModel>(dt_);
    solver_.init(200, 0.01);

    position_calculator_ = std::make_shared<gimbal_controller::ArmorPositionCalculator>();
    armor_selector_ = std::make_shared<gimbal_controller::ArmorSelector>();
    local_compensator_ = std::make_shared<gimbal_controller::LocalTrajectoryCompensator>();
    local_compensator_->setParameters(20.0, 9.8, 0.001, 20);

    ref_gen_.setComponents(position_calculator_, armor_selector_, local_compensator_);
  }

  rm_interfaces::msg::TrackedRobot makeStaticRobot(double x, double y, double z)
  {
    rm_interfaces::msg::TrackedRobot robot;
    robot.center_position.x = x;
    robot.center_position.y = y;
    robot.center_position.z = z;
    robot.center_velocity.x = 0;
    robot.center_velocity.y = 0;
    robot.center_velocity.z = 0;
    robot.center_acceleration.x = 0;
    robot.center_acceleration.y = 0;
    robot.center_acceleration.z = 0;
    robot.yaw = 0.0;
    robot.yaw_velocity = 0.0;
    robot.yaw_acceleration = 0.0;
    robot.radius = 0.2;
    robot.radius_2 = 0.15;
    robot.d_za = 0.0;
    robot.d_zc = 0.0;
    robot.num_armors = 4;
    robot.robot_type = 1;
    robot.track_state = 1;

    auto offsets = gimbal_controller::ArmorPositionCalculator::generateDefaultOffsets(
      robot.robot_type, 4, robot.radius, robot.radius_2, robot.d_za, robot.d_zc);
    robot.armors_offset.resize(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i) {
      robot.armors_offset[i].position.x = offsets[i].x();
      robot.armors_offset[i].position.y = offsets[i].y();
      robot.armors_offset[i].position.z = offsets[i].z();
    }
    return robot;
  }

  double dt_{0.01};
  int N_{18};
  std::unique_ptr<GimbalDynamicsModel> model_;
  QPSolver solver_;
  MpcReferenceGenerator ref_gen_;
  std::shared_ptr<gimbal_controller::ArmorPositionCalculator> position_calculator_;
  std::shared_ptr<gimbal_controller::ArmorSelector> armor_selector_;
  std::shared_ptr<gimbal_controller::LocalTrajectoryCompensator> local_compensator_;
};

TEST_F(TestMpcEndToEnd, ControlDirectionCorrectness)
{
  // 场景: 目标在右前方 (x=3, y=1), 云台初始朝正前方
  // 期望: yaw 加速度 > 0 (向右转)
  auto robot = makeStaticRobot(3.0, 1.0, 0.0);
  GimbalDynamicsModel::StateVector x0(0.0, 0.0, 0.0, 0.0);

  auto X_ref = ref_gen_.generate(robot, x0(0), x0(1), N_, dt_);

  Eigen::MatrixXd A_pred, B_pred;
  model_->buildPredictionMatrices(N_, A_pred, B_pred);
  auto D = GimbalDynamicsModel::buildDifferenceMatrix(N_);
  auto Q = GimbalDynamicsModel::buildWeightQ(N_, 100, 100, 10, 10);
  auto R = GimbalDynamicsModel::buildWeightR(N_, 0.01, 0.01);
  auto S = GimbalDynamicsModel::buildWeightS(N_, 5.0, 5.0);

  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  GimbalDynamicsModel::buildQP(A_pred, B_pred, D, Q, R, S, x0, X_ref, H, f);

  // 宽松 box 约束
  int n_vars = 2 * N_;
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_vars, -50.0);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_vars, 50.0);

  auto result = solver_.solve(H, f, lb, ub);

  ASSERT_TRUE(result.success) << "QP should converge";

  // 第一步 yaw 加速度应 > 0 (向右)
  EXPECT_GT(result.U(0), 0.0)
    << "yaw acceleration should be positive (turning right towards target)";
}

TEST_F(TestMpcEndToEnd, ControlWithinBounds)
{
  auto robot = makeStaticRobot(3.0, 1.0, 0.5);
  GimbalDynamicsModel::StateVector x0(0.0, 0.0, 0.0, 0.0);

  auto X_ref = ref_gen_.generate(robot, x0(0), x0(1), N_, dt_);

  Eigen::MatrixXd A_pred, B_pred;
  model_->buildPredictionMatrices(N_, A_pred, B_pred);
  auto D = GimbalDynamicsModel::buildDifferenceMatrix(N_);
  auto Q = GimbalDynamicsModel::buildWeightQ(N_, 100, 100, 10, 10);
  auto R = GimbalDynamicsModel::buildWeightR(N_, 0.01, 0.01);
  auto S = GimbalDynamicsModel::buildWeightS(N_, 5.0, 5.0);

  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  GimbalDynamicsModel::buildQP(A_pred, B_pred, D, Q, R, S, x0, X_ref, H, f);

  double max_acc = 20.0;  // rad/s^2
  int n_vars = 2 * N_;
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_vars, -max_acc);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_vars, max_acc);

  auto result = solver_.solve(H, f, lb, ub);

  ASSERT_TRUE(result.success);

  for (int i = 0; i < n_vars; ++i) {
    EXPECT_GE(result.U(i), -max_acc - 1e-6)
      << "Control U(" << i << ") below lower bound";
    EXPECT_LE(result.U(i), max_acc + 1e-6)
      << "Control U(" << i << ") above upper bound";
  }
}

TEST_F(TestMpcEndToEnd, RollingHorizonConvergence)
{
  // 模拟 30 步滚动 MPC，验证云台逐步趋近目标
  auto robot = makeStaticRobot(3.0, 0.5, 0.0);

  GimbalDynamicsModel::StateVector x = GimbalDynamicsModel::StateVector::Zero();
  // 初始偏差
  x(0) = 0.0;   // yaw 初始为 0
  x(1) = 0.0;   // pitch 初始为 0

  // 目标参考 yaw/pitch (大致计算)
  double target_yaw = std::atan2(0.5, 3.0);

  Eigen::MatrixXd A_pred, B_pred;
  model_->buildPredictionMatrices(N_, A_pred, B_pred);
  auto D = GimbalDynamicsModel::buildDifferenceMatrix(N_);
  auto Q = GimbalDynamicsModel::buildWeightQ(N_, 100, 100, 10, 10);
  auto R = GimbalDynamicsModel::buildWeightR(N_, 0.01, 0.01);
  auto S = GimbalDynamicsModel::buildWeightS(N_, 5.0, 5.0);

  double max_acc = 30.0;
  int n_vars = 2 * N_;
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_vars, -max_acc);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_vars, max_acc);

  double initial_error = std::abs(x(0) - target_yaw);

  int n_steps = 200;
  for (int step = 0; step < n_steps; ++step) {
    auto X_ref = ref_gen_.generate(robot, x(0), x(1), N_, dt_);

    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    GimbalDynamicsModel::buildQP(A_pred, B_pred, D, Q, R, S, x, X_ref, H, f);

    auto result = solver_.solve(H, f, lb, ub);
    ASSERT_TRUE(result.success) << "QP failed at step " << step;

    // 应用第一步控制
    GimbalDynamicsModel::ControlVector u(result.U(0), result.U(1));
    x = model_->predict(x, u);
  }

  double final_error = std::abs(x(0) - target_yaw);
  EXPECT_LT(final_error, initial_error * 0.25)
    << "After 200 MPC steps (2s), yaw error should be <25% of initial. "
    << "Initial: " << initial_error << ", Final: " << final_error;
}

TEST_F(TestMpcEndToEnd, DelayCompensation)
{
  // 验证延迟补偿: 有延迟时 MPC 仍能收敛
  auto robot = makeStaticRobot(3.0, 0.5, 0.0);

  GimbalDynamicsModel::StateVector x = GimbalDynamicsModel::StateVector::Zero();
  double delay_s = 0.03;  // 3 步延迟

  auto B_d = model_->buildDelayedB(N_, delay_s);

  Eigen::MatrixXd A_pred, B_pred_unused;
  model_->buildPredictionMatrices(N_, A_pred, B_pred_unused);
  auto D = GimbalDynamicsModel::buildDifferenceMatrix(N_);
  auto Q = GimbalDynamicsModel::buildWeightQ(N_, 100, 100, 10, 10);
  auto R = GimbalDynamicsModel::buildWeightR(N_, 0.01, 0.01);
  auto S = GimbalDynamicsModel::buildWeightS(N_, 5.0, 5.0);

  double max_acc = 30.0;
  int n_vars = 2 * N_;
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(n_vars, -max_acc);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(n_vars, max_acc);

  double target_yaw = std::atan2(0.5, 3.0);
  double initial_error = std::abs(x(0) - target_yaw);

  // 模拟延迟: 维护一个控制缓冲区
  int d = static_cast<int>(std::round(delay_s / dt_));
  std::vector<GimbalDynamicsModel::ControlVector> u_buffer(d,
    GimbalDynamicsModel::ControlVector::Zero());

  int n_steps = 300;
  for (int step = 0; step < n_steps; ++step) {
    auto X_ref = ref_gen_.generate(robot, x(0), x(1), N_, dt_);

    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    // 使用延迟矩阵 B_d
    GimbalDynamicsModel::buildQP(A_pred, B_d, D, Q, R, S, x, X_ref, H, f);

    auto result = solver_.solve(H, f, lb, ub);
    ASSERT_TRUE(result.success) << "QP failed at step " << step;

    // 新的控制输入进入缓冲区末端
    GimbalDynamicsModel::ControlVector u_new(result.U(0), result.U(1));

    // 从缓冲区头部取出实际执行的控制 (d 步前的输入)
    GimbalDynamicsModel::ControlVector u_exec = u_buffer.front();
    u_buffer.erase(u_buffer.begin());
    u_buffer.push_back(u_new);

    // 应用延迟后的控制
    x = model_->predict(x, u_exec);
  }

  double final_error = std::abs(x(0) - target_yaw);
  EXPECT_LT(final_error, initial_error * 0.20)
    << "With delay compensation (3s), yaw error should converge. "
    << "Initial: " << initial_error << ", Final: " << final_error;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
