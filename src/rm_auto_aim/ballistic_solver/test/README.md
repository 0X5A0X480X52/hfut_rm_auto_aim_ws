# ballistic_solver Tests

本目录包含针对 `ballistic_solver` 软件包的自动化测试，包括 launch 测试（launch_testing）和集成测试（pytest）。

## 目录结构
- `launch/test_ballistic_solver_launch.py`：使用 `launch_testing` 执行的 launch 测试，会启动 `ballistic_solver_node_exe` 并检查节点能否正常启动与退出。
- `test/test_ballistic_solver_service.py`：使用 pytest 的集成测试，启动（spawn）可执行文件并通过 rclpy 调用 `/ballistic_solver/solve` 服务。

## 测试类型说明
- Launch 测试 (launch_testing)
  - 验证节点可以被启动、接受参数并按预期退出。
  - 主要在 CI 中验证节点作为 ROS 组件或独立节点的可启动性。

- Pytest 集成测试
  - 以可执行文件的方式启动节点（`ballistic_solver_node_exe`），并使用 Python 客户端调用 `rm_interfaces/srv/SolveBallistic` 服务。
  - 测试覆盖静态目标、移动目标和异常（无效目标）场景。

## 运行准备
1. 构建 packages 并安装：

```bash
cd /home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws
colcon build --packages-select ballistic_solver
source install/setup.bash
```

2. 运行所有测试：

```bash
colcon test --packages-select ballistic_solver
```

或仅运行 launch 测试：

```bash
colcon test --packages-select ballistic_solver --ctest-args -R test_launch_test_ballistic_solver_launch.py
```

或仅运行 pytest 集成测试：

```bash
pytest src/rm_auto_aim/ballistic_solver/test/test_ballistic_solver_service.py
```

> 注意：运行 pytest 级别的测试时，请确保你已经 `source` 过 `install/setup.bash`，以便 `AMENT_PREFIX_PATH` 正确指向安装目录。

## 常见问题与调试建议
- FileNotFoundError: 无法找到 `ballistic_solver_node_exe`
  - 原因：测试脚本基于 `AMENT_PREFIX_PATH` 的第一项去定位 `share/ballistic_solver` 的安装目录，然后替换为 `lib/ballistic_solver` 去寻找 exe；如果 `AMENT_PREFIX_PATH` 非预期项（例如其他包路径位于前）则会导致路径指向错误的 install 前缀。
  - 解决办法：
    - 确保你已 `colcon build` 并执行 `source install/setup.bash`，或将脚本中的定位方式替换为 `ament_index_python.packages.get_package_share_directory('ballistic_solver')` 获取包路径（更稳健）。

- 运行测试时报 flake8 / cpplint 问题：
  - Python 相关问题通常是行过长（E501）、未使用的导入或模块级导入不在文件顶部（E402）。可以先修复 `launch/*.py` 和 `test/*.py` 中的风格问题以通过 lints。
  - C++ 的 cpplint 报错通常是 include 排序或 trailing spaces，须按 Google C++ 风格修正。

## 建议
- 为了提高测试稳定性，推荐把 `test/test_ballistic_solver_service.py` 中的 exe 定位方式替换为 `get_package_share_directory`，示例如下：

```python
from ament_index_python.packages import get_package_share_directory
pkg_dir = get_package_share_directory('ballistic_solver')
exe = os.path.join(pkg_dir.replace('/share/ballistic_solver', '/lib/ballistic_solver'), 'ballistic_solver_node_exe')
```

- 在 CI 环境中，通常已经正确设置 `AMENT_PREFIX_PATH`，不过对本地开发者来说明确 `source install/setup.bash` 仍是更稳妥的做法。

## 测试期望
- Launch 测试：节点可以启动并响应 SIGINT 退出（通过）。
- Pytest 服务测试：对静态、移动目标返回有效的 `success=True` 或 `False`（无效目标），并返回合理的 `flight_time`、`pitch` 等值。

---

如果你希望我也把 `test/test_ballistic_solver_service.py` 更新为 `get_package_share_directory` 的方式来稳健地定位 exe 并重新运行集成测试（以及同时修复 flake8/cpplint 报告的小问题），请回复确认，我会继续进行这些调整并重新运行测试。