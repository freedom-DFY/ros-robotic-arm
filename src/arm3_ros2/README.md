# 前三轴腕点控制 · Ubuntu 24.04 / ROS 2 Jazzy

由原三轴 ROS 1 代码迁移，使用 `rclpy`、`ament_python`、ROS 2 参数和 Python launch；旧 ROS 1 源码目录已删除。当前实机配置只使用 ID 1、2、3，不要求 ID 4、5 在线。不要同时运行占用同一串口的五轴节点、双电机测试节点或 DYNAMIXEL Wizard。

本包的控制器、关节目标、标定数组和正逆运动学均固定为三个关节。`motor_ids` 必须恰好包含三个不同 ID，仅增加 YAML 中的电机 ID 不能扩展为五轴。笛卡尔控制计算原 J4 安装点的 XYZ 位置，不控制 ID4/ID5 或末端姿态。工作空间另有独立的 `five_dof_arm` 五轴控制包，当前 `arm3.launch.py` 不会启动它。

腕点定义为原 J4 关节原点，即二臂末端原腕部接口；前三轴改进 D-H 几何、246.06 mm / 242.25 mm 臂长和安装偏置沿用原包。关节目标用弧度，腕点坐标用米。直线路径预先检查逆解、限位及分支连续性；没有碰撞检测，需要按实物校准零点、方向、臂长和机械限位。

## 编译和无硬件运行

本工作空间已有 Jazzy 和 DYNAMIXEL SDK 源码，执行：

```bash
cd /home/duziteng/ros-robotic-arm
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to arm3_ros2
source install/setup.bash
ros2 launch arm3_ros2 arm3.launch.py dry_run:=true
```

默认 `dry_run=true`，不打开串口；通过定时插值模拟位置反馈，不模拟动力学。也可执行 `ros2 run arm3_ros2 arm3_node`，内置默认参数与 YAML 相同。无需 `roscore`、`catkin_make` 或 Noetic 容器。

新机器已安装 ROS 2 Jazzy 后，缺少依赖时安装：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-serial python3-yaml \
  ros-jazzy-rclpy ros-jazzy-rcl-interfaces ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs ros-jazzy-std-msgs ros-jazzy-std-srvs \
  ros-jazzy-launch-ros ros-jazzy-ament-index-python
# 可选手柄驱动（保留原 Logitech DirectInput 布局）
sudo apt install ros-jazzy-joy-linux
```

SDK 随 `--packages-up-to arm3_ros2` 一起编译。若把本包单独复制到另一个工作空间，可安装 `sudo apt install ros-jazzy-dynamixel-sdk`。Ubuntu 24.04 使用 apt 提供的 SDK/serial，或编译本仓库 SDK，无需向系统 Python 执行 `pip install --user`。

另开终端，加载环境后查看反馈（每条 echo 命令单独运行，Ctrl+C 退出）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/duziteng/ros-robotic-arm/install/setup.bash
ros2 topic echo /arm3/encoder_ticks
ros2 topic echo /arm3/joint_states
ros2 topic echo /arm3/wrist_point
```

以下两条目标用于 **dry_run 验证**，按顺序执行并等待上一条完成：

```bash
ros2 topic pub --once /arm3/joint_target sensor_msgs/msg/JointState \
  '{name: [joint_1, joint_2, joint_3], position: [0.0, 0.18, -0.20]}'
ros2 topic pub --once /arm3/cartesian_target geometry_msgs/msg/PointStamped \
  '{header: {frame_id: base_link}, point: {x: 0.01, y: 0.30, z: 0.15}}'
ros2 service call /arm3/stop std_srvs/srv/Trigger '{}'
```

## 实机：只读连接、保持使能与运动

当前实机流程使用 [固定起点与 20 cm 直线配置](../../calibration/arm3_horizontal_up_20cm/README.md)，完整操作见 [执行20cm实机命令.txt](../../calibration/arm3_horizontal_up_20cm/执行20cm实机命令.txt)。该配置包含当前机械臂的参考零位、方向和已确认测试窗口；下面的 `config/arm3.yaml` 是未校准的通用模板。

重复测试使用 `ros2 service call /arm3/run_fixed_line std_srvs/srv/Trigger '{}'`：先按关节空间低速插值返回固定起点 `[3923,1061,2105]`，第一段不检查测试用软件窗口、不做笛卡尔逆解，允许从软件窗口外归位，但归位端点、插值指令和实时反馈仍须在独立机械运行边界及舵机内部位置范围内。每轴反馈在 10 刻度内且稳定 0.5 秒后，重新启用软件窗口的反馈检查，再执行 −Y 方向的 20 cm 直线。第二段完整路径在开始流程前预检查，每段受 120 秒限时。停止、卸力或故障取消整个流程；新目标不会替换活动流程。

当前实机限位已根据根目录关节限位文档更新：ID1 0～4095、ID2 740～2700、ID3 1047～3347 刻度。`mechanical_min_ticks/mechanical_max_ticks` 对所有主动运动生效；`position_min_ticks/position_max_ticks` 还可进一步缩小某次试验的范围。详见 [关节限位与修改说明](../../calibration/关节限位说明.md)。

单次相对运动命令 `ros2 run arm3_ros2 cartesian_step --dy -0.20` 仍可使用：从最新稳定反馈位置发送相对直线目标，最大位移 200 mm，发送前检查整段逆解和在线限位，不自动扩大范围。它不会先返回固定起点。

1. 固定底座并支撑大臂、二臂，在 Wizard 中关闭三轴扭矩后退出 Wizard。默认匹配 XM540-W270，型号编号 1120、ID 1/2/3、Protocol 2.0、1 Mbps；实际型号、ID 和串口需逐一核对。
2. 复制配置，保持 `calibration_confirmed: false`，启动真实串口的只读模式。节点此时只 ping 和读反馈，不配置模式、曲线、目标或扭矩。只读模式也不会主动关闭其他程序此前留下的扭矩。

```bash
cd /home/duziteng/ros-robotic-arm
cp src/arm3_ros2/config/arm3.yaml ~/arm3_hardware.yaml
ls -l /dev/serial/by-id/
ros2 launch arm3_ros2 arm3.launch.py dry_run:=false \
  device:=/dev/ttyUSB0 baud_rate:=1000000 config:="$HOME/arm3_hardware.yaml"
```

3. 扭矩关闭时，按实物测定每轴 `zero_ticks`、`joint_directions` 和 `position_min_ticks/position_max_ticks`。查看 `/arm3/encoder_ticks`，其 `data` 顺序与 `motor_ids` 一致，只读反馈显示单圈值（原始读数对 4096 取模）。`zero_ticks` 是机械参考姿态的读数，不能把任意拍摄姿态直接当作零位。配置数组都必须恰好三个整数。0–4095 是编码范围，不能直接作为装配后的机械限位。
4. 使能只把读到的当前位置写为保持目标，不检查应用层软件工作空间，也不要求 `calibration_confirmed=true`。即使当前位置在软件窗口外，仍可使能并保持，空闲保持和 stop 不会仅因该窗口而报错。舵机型号、通信、位置模式、零偏、硬件状态与内部寄存器限位检查仍有效，确保保持目标被原值接受。
5. 请求运动时才检查校准标记及软件限位与舵机内部限位的兼容性；启动时还检查软件范围位于机械运行边界内。普通关节/笛卡尔目标仍检查当前点、目标和路径，当前点超出窗口时会拒绝普通运动。固定起点流程的第一段允许从软件测试窗口外按关节空间归位，当前点和固定起点须在机械运行边界及舵机内部位置范围内；第二段仍完整检查软件限位和逆解。J1 可使用单圈全范围；J2/J3 仍用默认全范围或 `calibration_confirmed=false` 时不启动实际运动。校准完成后改为 `true` 并重启加载配置；运行时参数只读。
6. 在 Wizard 中预先核对 Position Control Mode=3、Homing Offset=0、Torque On by Goal Update 关闭，然后退出 Wizard。节点使能前逐台检查，保留 Drive Mode，不自动改写这些设置。时间型 Profile 按控制周期计算，默认 30 Hz 对应总时长 33 ms、加速 8 ms；速度型使用 YAML 的 `profile_velocity` / `profile_acceleration`。插值速度由 `joint_speed_rad_s` / `cartesian_speed_m_s` 控制。
7. 支撑机构并确认活动范围清空，在另一个已加载环境的终端手动使能：

```bash
ros2 service call /arm3/enable std_srvs/srv/SetBool '{data: true}'
```

仅当返回 `success: true` 才表示使能成功。以当时读到的实际位置作为初始保持目标。完成运动标定后重新查看 `/arm3/joint_states`，首次只把一个关节当前角度增减约 0.02 rad（约 1.15°），另外两个角度填当前值，再发送三元素 `joint_target`。不能直接照抄上面的 dry_run 绝对目标。单独调试时仍需 ID 1、2、3 全部在线。

停止并保持当前位置：

```bash
ros2 service call /arm3/stop std_srvs/srv/Trigger '{}'
```

支撑机构后关闭本节点使能的扭矩：

```bash
ros2 service call /arm3/enable std_srvs/srv/SetBool '{data: false}'
```

正常 Ctrl+C 退出也会尝试卸掉本节点使能的扭矩，关节可能因重力下落。通信或跟踪故障会清空轨迹并禁止后续运动；故障时电机可能仍保持最后目标，掉线时无法保证卸力成功。处理连接后先调用 `enable false`，再调用 `enable true`；USB I/O 故障通常需重新确认设备路径并重启节点。只在权限不足时执行 `sudo usermod -aG dialout "$USER"`，之后注销并重新登录。

## 接口与手柄

| 接口 | ROS 2 类型 | 单位/用途 |
| --- | --- | --- |
| `/arm3/joint_states` | `sensor_msgs/msg/JointState` | J1/J2/J3，rad |
| `/arm3/encoder_ticks` | `std_msgs/msg/Int32MultiArray` | `motor_ids` 顺序的单圈刻度 |
| `/arm3/wrist_point` | `geometry_msgs/msg/PointStamped` | 基座坐标系，m |
| `/arm3/joint_target` | `sensor_msgs/msg/JointState` | 三关节绝对角度，rad，可按 name 重排 |
| `/arm3/cartesian_target` | `geometry_msgs/msg/PointStamped` | 绝对 XYZ，m，frame_id 必须匹配 base_frame |
| `/arm3/auto_transform` | `geometry_msgs/msg/Point` | 原视觉兼容输入，绝对 XYZ，mm |
| `/arm3/enable` | `std_srvs/srv/SetBool` | 手动使能 / 卸力 |
| `/arm3/stop` | `std_srvs/srv/Trigger` | 清空路径并保持位置 |
| `/arm3/run_fixed_line` | `std_srvs/srv/Trigger` | 返回固定起点、稳定后执行固定直线 |
| `/arm3/motion_status` | `std_msgs/msg/String` | 当前运动阶段、完成、停止或故障状态 |

普通控制时新有效目标替换当前路径，无效目标保留现有路径。固定起点流程运行期间其他目标会被拒绝，需先调用 stop。旧视觉端也必须迁移为 ROS 2 才能直接发送消息。

```bash
ros2 launch arm3_ros2 arm3.launch.py dry_run:=true use_joy:=true joy_device:=/dev/input/js0
```

内置启动 `joy_linux/joy_linux_node`，保留原 DirectInput 映射：BACK 切换模式；关节模式下 LT/RT 调 J1，LB/RB 调 J2，十字键上下调 J3；笛卡尔模式下十字键上下调 X、左右调 Y，Y/A 调 Z。每次按下只走一步，需松开再按；首次 Joy 消息仅初始化按键状态。可用外部 ROS 2 节点发布 `/joy`，订阅支持 best-effort QoS。原映射不适用于 SDL `game_controller_node`，应先在 dry_run 验证按钮对应关系。当前环境未接手柄，未做实体按键验证。

launch 的 `dry_run/device/baud_rate` 始终覆盖 YAML 中相同参数；直接 `ros2 run ... --ros-args --params-file /绝对路径/配置.yaml` 则使用 YAML 值。ROS 1 的扁平 YAML 不能直接加载，需要 `/**: ros__parameters:` 层级。

## 验证与源码

```bash
source /opt/ros/jazzy/setup.bash
source /home/duziteng/ros-robotic-arm/install/setup.bash
colcon test --packages-select arm3_ros2 --event-handlers console_direct+
colcon test-result --test-result-base build/arm3_ros2 --verbose
python3 src/arm3_ros2/test/smoke_test.py
```

`arm3_ros2/node.py` 是 ROS 2 控制节点；`kinematics.py` / `dynamixel_bus.py` 沿用原三轴算法与 SDK 读写；`config/arm3.yaml` 为 ROS 2 参数；`launch/arm3.launch.py` 为启动入口。无硬件联调不能代替实机串口、型号、零位及运动验证。

本环境已通过 colcon 编译、pytest 及独立 ROS 2 launch 联调；覆盖参数覆盖顺序、关节名称重排、编码/角度/XYZ 反馈、无效目标拒绝、笛卡尔路径、旧 mm 输入、best-effort 手柄消息、停止、禁用/使能、固定起点两阶段流程及正常退出。固定起点流程的新代码仅完成模拟反馈验证，未自动发送实机运动；实机参考数据和此前测试情况见对应规划记录。

迁移参考：[ROS 2 官方 Python 迁移说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Migrating-from-ROS1/Migrating-Python-Packages.rst)、[Jazzy Ubuntu 24.04 安装说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)、[joy_linux 官方说明](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy_linux/README.md)。
