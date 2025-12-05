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
Trajectory Planner Package

A modular MPC-based trajectory planner for gimbal control in RoboMaster auto-aim system.

Modules:
- gimbal_model: Gimbal dynamics model
- mpc_controller: QP-based MPC controller
- target_predictor: Target trajectory extraction from armor_tracker
- target_manager: Target robot management from target_selector
- ballistic_client: Ballistic solver service client
- trajectory_planner_node: Main ROS2 node
"""

from trajectory_planner.gimbal_model import GimbalModel, GimbalConfig
from trajectory_planner.mpc_controller import MPCController, MPCConfig
from trajectory_planner.target_predictor import TargetPredictor, TargetPredictorConfig
from trajectory_planner.target_manager import TargetManager, TargetManagerConfig
from trajectory_planner.ballistic_client import BallisticClient, BallisticClientConfig

__all__ = [
    'GimbalModel',
    'GimbalConfig',
    'MPCController',
    'MPCConfig',
    'TargetPredictor',
    'TargetPredictorConfig',
    'TargetManager',
    'TargetManagerConfig',
    'BallisticClient',
    'BallisticClientConfig',
]
