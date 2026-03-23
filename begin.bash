#!/bin/bash
source /home/hfut-nuc/.bashrc
source /home/hfut-nuc/hfut_rm_auto_aim_ws/env.zsh
source /home/hfut-nuc/hfut_rm_auto_aim_ws/install/setup.bash

ros2 launch rm_bringup bringup_pipeline.launch.py
