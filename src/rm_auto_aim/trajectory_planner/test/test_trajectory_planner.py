#!/usr/bin/env python3
# Copyright (C) FYT Vision Group. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Unit tests for trajectory_planner modules
"""

import pytest
import numpy as np
import math

from trajectory_planner import (
    GimbalModel,
    GimbalConfig,
    MPCController,
    MPCConfig,
    TargetPredictor,
    TargetPredictorConfig,
    TargetManager,
    TargetManagerConfig
)


class TestGimbalModel:
    """Test GimbalModel class"""
    
    def test_init_default(self):
        """Test default initialization"""
        model = GimbalModel()
        assert model.config.dt == 0.01
        assert model.A.shape == (3, 3)
        assert model.B.shape == (3, 1)
    
    def test_init_custom_config(self):
        """Test custom configuration"""
        config = GimbalConfig(dt=0.02, omega_max=3.0)
        model = GimbalModel(config)
        assert model.config.dt == 0.02
        assert model.config.omega_max == 3.0
    
    def test_predict(self):
        """Test state prediction"""
        model = GimbalModel()
        state = np.array([0.0, 0.0, 0.0])
        control = 1.0
        
        next_state = model.predict(state, control)
        
        # State should change after applying control
        assert not np.allclose(next_state, state)
        # Alpha should increase with positive jerk
        assert next_state[2] > 0
    
    def test_constraints(self):
        """Test state constraints"""
        config = GimbalConfig(omega_max=1.0, alpha_max=5.0)
        model = GimbalModel(config)
        
        # State with excessive values
        state = np.array([0.0, 10.0, 100.0])  # omega and alpha too large
        
        constrained = model.apply_constraints(state)
        
        assert constrained[1] <= config.omega_max
        assert constrained[2] <= config.alpha_max
    
    def test_angle_wrap(self):
        """Test angle wrapping"""
        wrapped = GimbalModel._wrap_angle(4.0)
        assert -np.pi <= wrapped <= np.pi
        
        wrapped = GimbalModel._wrap_angle(-4.0)
        assert -np.pi <= wrapped <= np.pi
    
    def test_prediction_matrices(self):
        """Test prediction matrices construction"""
        model = GimbalModel()
        N = 10
        
        A_pred, B_pred = model.get_prediction_matrices(N)
        
        assert A_pred.shape == (N * 3, 3)
        assert B_pred.shape == (N * 3, N * 1)


class TestMPCController:
    """Test MPCController class"""
    
    def test_init_default(self):
        """Test default initialization"""
        controller = MPCController()
        assert controller.config.prediction_horizon == 18
    
    def test_solve_basic(self):
        """Test basic MPC solving"""
        config = MPCConfig(prediction_horizon=10, dt=0.01)
        controller = MPCController(mpc_config=config)
        
        current_state = np.array([0.0, 0.0, 0.0])
        target_yaw = np.deg2rad(10)
        target_trajectory = np.ones(10) * target_yaw
        
        u_opt, U_seq, solve_time = controller.solve(current_state, target_trajectory)
        
        assert controller.last_success
        assert solve_time > 0
        assert len(U_seq) == 10
    
    def test_solve_tracking(self):
        """Test tracking behavior"""
        config = MPCConfig(prediction_horizon=10, dt=0.01)
        controller = MPCController(mpc_config=config)
        
        current_state = np.array([0.0, 0.0, 0.0])
        target_yaw = np.deg2rad(30)
        target_trajectory = np.ones(10) * target_yaw
        
        # Simulate several steps
        state = current_state.copy()
        for _ in range(50):
            u_opt, _, _ = controller.solve(state, target_trajectory)
            state = controller.gimbal_model.predict(state, u_opt)
        
        # Should be closer to target
        assert abs(state[0] - target_yaw) < abs(current_state[0] - target_yaw)
    
    def test_predict_state(self):
        """Test state trajectory prediction"""
        config = MPCConfig(prediction_horizon=5, dt=0.01)
        controller = MPCController(mpc_config=config)
        
        current_state = np.array([0.0, 0.1, 0.0])
        control_sequence = np.zeros(5)
        
        trajectory = controller.predict_state(current_state, control_sequence)
        
        assert trajectory.shape == (6, 3)  # N+1 states
        assert np.allclose(trajectory[0], current_state)


class TestTargetPredictor:
    """Test TargetPredictor class"""
    
    def test_init_default(self):
        """Test default initialization"""
        predictor = TargetPredictor()
        assert predictor.config.min_confidence == 0.3
        assert predictor.config.selection_strategy == "nearest_yaw"
    
    def test_wrap_angle(self):
        """Test angle wrapping utility"""
        assert abs(TargetPredictor._wrap_angle(np.pi + 0.1) - (-np.pi + 0.1)) < 0.001
    
    def test_has_valid_target_empty(self):
        """Test valid target check with no data"""
        predictor = TargetPredictor()
        assert not predictor.has_valid_target()


class TestTargetManager:
    """Test TargetManager class"""
    
    def test_init_default(self):
        """Test default initialization"""
        manager = TargetManager()
        assert manager.config.target_timeout == 2.0
        assert not manager.is_tracking
    
    def test_set_target_robot(self):
        """Test setting target robot"""
        manager = TargetManager()
        
        success, message, previous = manager.set_target_robot("3")
        
        assert success
        assert manager.target_robot_id == "3"
        assert manager.is_tracking
        assert previous == ""
    
    def test_clear_target(self):
        """Test clearing target"""
        manager = TargetManager()
        manager.set_target_robot("3")
        
        manager.clear_target()
        
        assert manager.target_robot_id is None
        assert not manager.is_tracking
        assert manager.previous_robot_id == "3"
    
    def test_get_target_armor_ids_fallback(self):
        """Test armor ID retrieval fallback"""
        manager = TargetManager()
        manager.set_target_robot("3")
        
        # Without robot info, should return robot_id as fallback
        armor_ids = manager.get_target_armor_ids()
        
        assert "3" in armor_ids


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
