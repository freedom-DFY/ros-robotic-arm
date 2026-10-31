# 五自由度机械臂：Ubuntu 24.04 / ROS 2 Jazzy

已将参考代码中的五舵机同步读写、Logitech 手柄控制、末端 XYZ 输入迁移为 ROS 2 C++ 节点，使用 `ament_cmake` 和 `colcon` 编译。
Ubuntu 24.04 是 [ROS 2 Jazzy 的官方支持平台](https://www.openrobotics.org/blog/2024/5/ros-jazzy-jalisco-released)。

## 工作空间

```text
src/
  DynamixelSDK-main/ros/
    dynamixel_sdk/                    # ROS 2 SDK；保留原有目录，方便 IDE 使用
    dynamixel_sdk_custom_interfaces/  # 增加五舵机同步消息/服务
    dynamixel_sdk_examples/           # 上游 ROS 2 单舵机示例
  five_dof_arm/
    src/sync_read_write_node.cpp      # 串口驱动 / dry_run
    src/logicool_controller.cpp       # 手柄、正逆运动学、XYZ 输入
    config/arm.yaml                   # ID、限位、初始姿态、机械臂尺寸
    launch/manipulator.launch.py
reference/                           # 仅本地保留，不提交到 GitHub
```

已删除解压参考代码中的 ROS 1 `ros/` 目录（catkin 包、ROS 1 launch、全向轮/视频等无关节点及旧编译产物），清理了工作空间旧的 `build/install/log` 后重新编译。
原始压缩包仍保存在原工作空间的 `reference/` 中，可用于查阅原代码；该目录不上传到 GitHub，克隆仓库不会包含这些资料。该目录有 `COLCON_IGNORE`，不参与编译。SDK 的 Protocol 1.0 支持是舵机通信协议，与 ROS 1 无关，予以保留。

## 编译

本机已有 Jazzy、编译工具和依赖，直接执行：

```bash
cd /home/duziteng/ros-robotic-arm
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

若在另一台已经安装 Jazzy 的 Ubuntu 24.04 上缺少依赖，可先安装：

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev python3-yaml ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python \
  ros-jazzy-ament-cmake-gtest ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
  ros-jazzy-rclcpp ros-jazzy-rclpy ros-jazzy-rosidl-default-generators \
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-joy ros-jazzy-launch-ros
```

也可在已完成 `rosdep init` 的机器上用 `rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy` 解析依赖。
无需启动 `roscore`，无需使用 `catkin_make`。

## 无硬件运行

```bash
source /opt/ros/jazzy/setup.bash
source /home/duziteng/ros-robotic-arm/install/setup.bash
ros2 launch five_dof_arm manipulator.launch.py dry_run:=true
```

`dry_run` 不打开串口，使用初始姿态作为状态，并立即回显收到的位置指令，适合验证 ROS 通信与逆解；它不模拟动力学、碰撞或实际运动过程。
另开终端，同样 source 后查看：

```bash
ros2 topic echo /joint_states
ros2 topic echo /end_effector_position
ros2 service call /sync_get_position dynamixel_sdk_custom_interfaces/srv/SyncGetPosition \
  '{id1: 1, id2: 2, id3: 3, id4: 4, id5: 5}'
```

## 实机运行

本驱动沿用原代码的 **Dynamixel X 系列 / MX(2.0)、Protocol 2.0、单圈位置模式**，地址为扭矩使能 64、目标位置 116、当前位置 132。其他型号需先核对控制表。
默认舵机 ID 为 1–5，波特率为 1000000，串口为 `/dev/ttyUSB0`。这五个 ID 对应五个机械关节。

```bash
ros2 launch five_dof_arm manipulator.launch.py \
  device_name:=/dev/ttyUSB0 baud_rate:=1000000
```

启动时先关闭扭矩，设置位置模式与速度/加速度，读取实际位置并写入同一目标，再使能扭矩；不会自动前往预设姿态。
默认退出时关闭扭矩，机械臂可能在重力下移动，启停时应支撑好机械臂；如需保留扭矩，可在 `arm.yaml` 中设置 `disable_torque_on_shutdown: false`。
默认限位 0–4095 仅为单圈编码范围，使用前请在配置中按实际关节行程收紧 `position_min/position_max`。
原代码的 profile 数值为加速度 500、速度 1000；可在配置中调低。

若串口报权限错误，执行 `sudo usermod -aG dialout "$USER"` 后注销并重新登录；用 `ls -l /dev/ttyUSB*` 确认设备名。

## 手柄操作

```bash
ros2 launch five_dof_arm manipulator.launch.py dry_run:=true use_joy:=true
```

实机手柄控制时去掉 `dry_run:=true`。默认启动 `joy/game_controller_node`，使用 SDL 标准映射，可通过 `joy_device_id:=0` 选择设备。
设备列表可用 `ros2 run joy joy_enumerate_devices` 查看。标准映射依据 [joy 官方说明](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md)。

| 按键 | 关节模式（默认） | 笛卡尔模式 |
| --- | --- | --- |
| BACK | 切换到笛卡尔模式 | 切换到关节模式 |
| LT / RT | 关节 1 −16 / +16 刻度 | RT：切割预设 |
| LB / RB | 关节 2 −8 / +8 刻度 | 初始 / 工作预设 |
| 十字键上 / 下 | 关节 3 +8 / −8 刻度 | X +10 / −10 mm |
| 十字键左 / 右 | 关节 4 +16 / −16 刻度 | Y +10 / −10 mm |
| X / B | 关节 5 +16 / −16 刻度 | — |
| Y / A | 初始 / 工作预设 | Z +10 / −10 mm |
| START | 切割预设 | — |

BACK 按下沿切换模式，长按不会重复切换。自动重发频率为 20 Hz，按住增量按键会持续发出位置指令。
初始、工作、切割预设分别为 `[2048,2160,1910,1360,2048]`、`[2048,2048,2048,2048,2048]`、`[2048,2240,1480,2910,2048]`。

若已有发布原始 Logitech DirectInput 布局（十字键为 axes[4:6]）的 ROS 2 Joy 节点，使用 `use_joy:=false joy_layout:=legacy`，由外部节点提供 `/joy`。
`legacy` 不适用于内置的 SDL `game_controller_node`。

## 位置指令和末端坐标

五舵机同步位置指令（以下用于已启动的 dry_run 或实机驱动）：

```bash
ros2 topic pub --once /sync_set_position dynamixel_sdk_custom_interfaces/msg/SyncSetPosition \
  '{id1: 1, id2: 2, id3: 3, id4: 4, id5: 5, position1: 2048, position2: 2160, position3: 1910, position4: 1360, position5: 2048}'
```

ID 必须是配置中的五个 ID，不能重复；允许按不同顺序给出 ID，位置与对应 ID 一一配对。超限时整条指令被拒绝。
`/sync_get_position` 返回 `success`、`message` 及五个位置字段，失败时应检查 `success`，不能把默认零值当作实际读数。

末端坐标输入保留原话题 `/auto_transform`，类型为 `geometry_msgs/msg/Point`，**单位为毫米**，坐标系沿用原始改进 DH 模型的基坐标系：

```bash
ros2 topic pub --once /auto_transform geometry_msgs/msg/Point '{x: 0.0, y: 250.0, z: 0.0}'
```

此命令格式可直接使用，但坐标是否可达取决于当前工具姿态及配置限位；可先读取 `/end_effector_position`，在其附近输入目标。
XYZ 指令保持工具相对径向平面的俯仰/滚转，并允许底座偏航跟随目标，不提供任意六维位姿控制。
逆解枚举底座/肘部解，校验完整正解，选择与当前关节位置最近且不超限的解；不可达或非有限数值输入会被拒绝。
没有新鲜的驱动状态（超过 1 秒未收到 `/joint_states`）时，控制器不发送手柄或 XYZ 指令。

机械参数沿用原资料：a2=246.06 mm、a3=242.25 mm、工具长=255.55 mm；原有 1.444 rad 安装偏置保留。
`/joint_states` 中 `joint_1`–`joint_5` 表示相对舵机 2048 刻度的角度，单位为弧度；`/end_effector_position` 为模型计算的毫米坐标。
当前没有新增 URDF、MoveIt、轨迹插补或碰撞检测；XYZ 是目标位置指令，TCP 路径不保证为直线。

可编辑 `src/five_dof_arm/config/arm.yaml`，或通过 `config:=/绝对路径/arm.yaml` 提供配置。
launch 参数 `dry_run/device_name/baud_rate/joy_layout` 始终覆盖 YAML 对应值；使用自定义串口、波特率时请显式传入这些参数。
直接 `ros2 run five_dof_arm sync_read_write_node --ros-args --params-file /绝对路径/arm.yaml` 则使用 YAML 的值。

## 验证

```bash
colcon test --packages-select five_dof_arm --event-handlers console_direct+
colcon test-result --verbose
python3 src/five_dof_arm/test/smoke_test.py
```

最后一条需已 source 工作空间，默认使用 ROS_DOMAIN_ID=73，自动启动并关闭 dry_run 节点。
已验证 4 个包编译通过；6 项运动学测试通过（含 100 个随机姿态往返校验）；dry_run 联调验证了话题、服务、ID 顺序、限位、手柄、模式切换、XYZ 逆解及非法目标拒绝。
尚未接入真实机械臂验证串口通信、舵机型号、机械零位及运动效果。

## Git 仓库

仓库只跟踪源码、配置和说明；`build/`、`install/`、`log/`、IDE 缓存、原始参考资料和本地凭据通过 `.gitignore` 排除。
DynamixelSDK 保留上游许可证；本仓库代码采用 Apache-2.0 许可证，见 `LICENSE`。
