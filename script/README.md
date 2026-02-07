# ROS2 Bag 使用说明（script 目录）

本说明文档介绍 `record_topics.sh` 和 `replay_bag.sh` 两个脚本的使用方法、参数说明与常见问题处理方法。

**前提条件**

- 已安装并配置 ROS2（请在运行脚本前执行 `source /opt/ros/<ros2-distro>/setup.bash` 或你的工作空间 `install`/`local_setup.bash`）。
- 已安装 `ros2` 命令并可在终端中使用。
- 脚本位于本目录：`record_topics.sh`、`replay_bag.sh`。

**文件权限**

脚本已设置为可执行：

```bash
chmod +x script/record_topics.sh script/replay_bag.sh
```

## `record_topics.sh` — 录制话题

用法概览：

```bash
./script/record_topics.sh [选项] <话题1> <话题2> ...
```

主要选项：

- `-o, --output DIR`：指定输出目录（默认 `~/ros2_bags`）。
- `-n, --name NAME`：指定 bag 名称（默认使用时间戳自动生成）。
- `-a, --all`：录制所有话题（等同 `ros2 bag record -a`）。
- `-d, --duration SEC`：录制固定秒数后自动停止。
- `-c, --compression`：启用压缩存储（使用 ros2 bag 的 file 压缩模式）。
- `-h, --help`：显示帮助。

示例：

```bash
# 录制指定话题
./script/record_topics.sh /camera/image_raw /lidar/points

# 录制所有话题 30 秒
./script/record_topics.sh -a -d 30

# 指定输出目录和名称
./script/record_topics.sh -o ./my_bags -n test_bag /camera/image_raw

# 使用压缩
./script/record_topics.sh -c /camera/image_raw
```

录制完成后，脚本会在输出目录下生成一个以 `BAG_NAME` 命名的文件夹（包含 `metadata.yaml` 等文件），并会尝试运行 `ros2 bag info` 来显示基本信息。

## `replay_bag.sh` — 重放 bag

用法概览：

```bash
./script/replay_bag.sh [选项] <bag_folder_path>
```

主要选项：

- `-r, --rate RATE`：播放速率，浮点数（默认 `1.0`）。
- `-l, --loop`：循环播放。
- `-s, --start SEC`：从指定秒数开始播放（起始偏移）。
- `-d, --duration SEC`：仅播放指定秒数。
- `-t, --topics TOPICS`：仅播放指定话题，多个话题用空格分隔（传入字符串）。
- `-i, --info`：仅显示 bag 信息，不播放。
- `-L, --list`：列出默认目录（`~/ros2_bags`）中的所有 bag。
- `-h, --help`：显示帮助。

示例：

```bash
# 正常播放
./script/replay_bag.sh ~/ros2_bags/my_bag

# 以 0.5 倍速播放
./script/replay_bag.sh -r 0.5 ~/ros2_bags/my_bag

# 循环播放
./script/replay_bag.sh -l ~/ros2_bags/my_bag

# 从第10秒开始播放30秒
./script/replay_bag.sh -s 10 -d 30 ~/ros2_bags/my_bag

# 仅播放某些话题
./script/replay_bag.sh -t '/camera/image_raw /lidar/points' ~/ros2_bags/my_bag

# 列出默认目录中的所有 bag
./script/replay_bag.sh -L

# 显示 bag 信息
./script/replay_bag.sh -i ~/ros2_bags/my_bag
```

注意：脚本期望 `bag` 的路径为 ros2 bag 录制生成的文件夹（包含 `metadata.yaml` 文件）。

## 常见问题与排查

- 如果出现 `ros2: command not found`：请确认已 source ROS2 环境，例如：

```bash
source /opt/ros/foxy/setup.bash    # 根据你的发行版调整
# 或者如果使用工作空间的 install
source install/local_setup.bash
```

- 如果 `ros2 bag info` 无法读取：检查指定路径下是否存在 `metadata.yaml`，且路径为录制时的输出文件夹而不是 `.bag` 单个文件。

- 若需长期保存大量数据：请确保磁盘空间充足，建议定期压缩或清理。

## 进阶建议

- 在录制高频率、占用大的话题（例如原始图像）时，推荐使用 `-c` 压缩以节省磁盘。
- 若需在回放时注入到真实系统中（例如带硬件控制），先在隔离环境里测试回放效果以避免意外命令执行。

---

如果你希望我把此说明追加到项目根 README 中的某个部分，或生成一个更短的快速上手指南，我可以继续修改。

## 配置文件（YAML 格式）

脚本支持从 YAML 配置文件读取默认参数，无需每次都指定命令行选项。项目包含两个配置文件：

### `record_config.yaml` — 录制配置

控制 `record_topics.sh` 的默认行为：

```yaml
output_dir: "~/ros2_bags"          # 输出目录
bag_name: ""                        # bag 名称（留空自动生成时间戳）
compression: false                  # 是否启用压缩
duration: 0                         # 录制时长（秒），0=不限制
record_all: false                   # 是否录制所有话题
topics:                             # 要录制的话题列表
  - /camera/image_raw
  - /camera/camera_info
  - /detector/armors
  - /tracker/target
```

**使用方式：**

- 编辑配置文件中的 `topics` 列表来指定要录制的话题
- 设置 `record_all: true` 可录制所有话题（忽略 topics 列表）
- 设置 `compression: true` 启用压缩存储
- 命令行参数优先级高于配置文件

**示例：**

```bash
# 使用配置文件中的话题列表录制
./script/record_topics.sh

# 覆盖配置，录制特定话题
./script/record_topics.sh /custom/topic

# 使用配置文件的话题，但指定输出目录
./script/record_topics.sh -o /tmp/bags
```

### `replay_config.yaml` — 重放配置

控制 `replay_bag.sh` 的默认行为：

```yaml
bag_path: ""                        # 默认 bag 路径（留空需命令行指定）
bag_dir: "~/ros2_bags"              # bag 搜索目录（用于 -L 列表功能）
rate: 1.0                           # 播放速率
loop: false                         # 是否循环播放
start_offset: 0                     # 起始偏移（秒）
duration: 0                         # 播放时长（秒），0=播放全部
topics:                             # 要重放的话题（留空重放全部）
  # - /camera/image_raw
  # - /detector/armors
```

**使用方式：**

- 设置 `bag_path` 可指定默认要重放的 bag
- 在 `topics` 中指定话题列表，仅重放这些话题
- 设置 `loop: true` 可循环重放
- 命令行参数优先级高于配置文件

**示例：**

```bash
# 使用配置文件中的设置重放（需要配置了 bag_path）
./script/replay_bag.sh

# 指定 bag 路径（覆盖配置文件）
./script/replay_bag.sh ~/ros2_bags/my_bag

# 使用配置的速率，但指定 bag
./script/replay_bag.sh -r 2.0 ~/ros2_bags/my_bag
```

### 配置优先级

命令行参数 > YAML 配置文件 > 脚本内置默认值

**注意事项：**

- 配置文件需要 Python3 和 PyYAML 库（ROS2 环境默认已安装）
- 路径支持 `~` 展开为用户主目录
- 脚本启动时会显示"✓ 已加载配置"提示
