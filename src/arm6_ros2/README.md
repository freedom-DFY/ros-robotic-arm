# arm6_ros2：六轴机械臂 + 独立末端夹爪

唯一运行入口为 `arm6_node`，启动文件为 `arm6.launch.py`。本包连接实机串口，
控制六个旋转关节和一个独立夹爪，不包含仿真节点、URDF 模型或 `dry_run` 参数。
代码阅读说明见工作空间根目录的 [代码阅读.md](../../代码阅读.md)。

## 1. 电机与几何配置

| 项目 | 当前配置 |
| --- | --- |
| J1–J3 | XM540-W270，ID1/2/3，型号号 1120 |
| 前三轴零位/方向 | `[3819,989,2013]` / `[1,-1,-1]` |
| 前三轴运行限位 | `[0,740,1047]` 至 `[4095,2700,3347]` tick |
| 大臂/小臂 | 0.24606 / 0.24225 m |
| 前三轴速度上限 | 0.02 rad/s |
| J4–J6（假设） | XM430-W350，ID4/5/6，型号号 1020，零位 2048，方向 +1，限位 1024–3072 tick |
| 腕部（假设） | 三转轴相交的球腕 |
| TCP（假设） | 沿法兰局部 Z 轴偏移 0.12 m，姿态偏置为零 |
| 夹爪（假设） | XM430-W350，ID7；1800 tick 闭合、2600 tick 张开，完整开口 0–0.06 m |

夹爪不占手臂六个旋转自由度，系统共七个电机。
前三轴的标定依据整理在 [硬件说明](../../docs/hardware/README.md)。
控制表参考 ROBOTIS 官方手册：[XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)、
[XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)。

当前驱动只支持上述两个型号共用的 X 系列控制表和 4096 刻度单圈编码。
所有电机应连接兼容的 TTL 或 RS-485 电气总线，使用相同波特率、Protocol 2.0、
Position Control Mode 3、Homing Offset 0，并关闭 Torque On by Goal Update。
程序不自动修改运行模式或 EEPROM 参数。

标准 DH 约定为：

```text
T(i-1,i) = Rz(q_i + offset_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
T_base_tcp = T01 T12 T23 T34 T45 T56 T_flange_tcp
```

| 关节 | a (m) | d (m) | alpha (rad) | offset (rad) |
| --- | --- | --- | --- | --- |
| J1 | 0 | 0 | -π/2 | 0 |
| J2 | 0.24606 | 0 | π | 0 |
| J3 | 0.24225 | 0 | 0 | π/2 |
| J4 | 0 | 0 | -π/2 | 0 |
| J5 | 0 | 0 | π/2 | 0 |
| J6 | 0 | 0 | 0 | 0 |

前三行与原腕部点位置公式等价，回归测试独立检查其结果，不再依赖旧三轴源码。
编码换算保持 `q=(tick-zero)*direction*2π/4096`。
默认参考系名称保留 `arm3_reference`，只是沿用坐标名称，不依赖旧三轴包。

## 2. 安装与启动

从工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select arm6_ros2 --symlink-install
source install/setup.bash
ros2 launch arm6_ros2 arm6.launch.py device:=/dev/ttyUSB0 baud_rate:=1000000
```

`dynamixel_sdk` 由 ROS 系统依赖提供；缺失时执行 `sudo apt install ros-jazzy-dynamixel-sdk`。
启动会连接并检查全部七个电机型号，缺少电机或型号不符时退出，不使用虚拟反馈替代。
启动成功后仅只读反馈，手动使能才会原地保持：

```bash
ros2 service call /arm6/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /arm6/joint_states
ros2 topic echo /arm6/tcp_pose
ros2 topic echo /arm6/motion_status
```

使能先检查模式、偏置、内部限位和硬件错误，读取当前位置，在任何轴上扭矩前，
写入并回读全部保持目标，再逐轴使能和确认。中途失败会尝试撤销本次扭矩使能。
当前位置在应用运动边界外仍可原地保持；不会自动回零或自动运动。

## 3. 正式参数

配置文件为 [config/arm6.yaml](config/arm6.yaml)，也可指定外部配置：

```bash
ros2 launch arm6_ros2 arm6.launch.py \
  config:=/absolute/path/your_measured_arm6.yaml \
  device:=/dev/ttyUSB0 baud_rate:=1000000
```

`device`、`baud_rate` 以 launch 参数为准；其余参数在启动时从 YAML 读取，运行中只读。
不通过 launch、直接执行 `ros2 run arm6_ros2 arm6_node --ros-args --params-file ...` 时，
串口设置采用参数文件中的值。

| 参数 | 填写内容 |
| --- | --- |
| `motor_ids`、`expected_model_numbers` | 七项：六关节后接夹爪 |
| `zero_ticks`、`joint_directions` | 六项：每轴编码零位与方向 |
| `position_min_ticks`、`position_max_ticks` | 六项：统一用于普通运动、逆解和执行检查的运行边界 |
| `dh_a_m`、`dh_d_m`、`dh_alpha_rad`、`dh_offsets_rad` | 各六项：标准 DH |
| `tcp_xyz_m`、`tcp_rpy_rad` | 各三项：法兰到 TCP 的平移和固定轴 roll/pitch/yaw |
| `gripper_closed_ticks`、`gripper_open_ticks`、`gripper_max_width_m` | 夹爪端点及完整开口，支持编码方向相反 |
| `joint_velocity_limits_rad_s`、`joint_acceleration_limits_rad_s2` | 各六项：关节速度和加速度上限 |
| `gripper_speed_m_s`、`gripper_acceleration_m_s2` | 夹爪开口速度、加速度上限 |
| `calibration_confirmed` | 七电机标定、方向和运行边界验证后设 true，允许实机运动 |
| `geometry_confirmed` | DH 和 TCP 实测验证后设 true，额外允许末端位姿目标 |

后三轴与夹爪型号、零位、限位、DH 和 TCP 尚属假设，默认两个 confirmed 参数均为 false。
确认标志约束运动请求，不妨碍初始使能保持。运行边界必须位于电机内部位置限位中。

## 4. 关节、TCP 与夹爪命令

所有运动均从实测位置规划，运动中拒绝覆盖，另发目标前先等结束或停止。

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/arm6/joint_target` | `sensor_msgs/msg/JointState` | 六轴关节目标，单位 rad |
| `/arm6/pose_target` | `geometry_msgs/msg/PoseStamped` | TCP 位姿，位置 m，归一化四元数 x/y/z/w |
| `/arm6/gripper_target` | `std_msgs/msg/Float64` | 完整夹爪开口，单位 m |
| `/arm6/gripper/open`、`/arm6/gripper/close` | `std_srvs/srv/Trigger` | 张开/闭合请求 |
| `/arm6/enable` | `std_srvs/srv/SetBool` | 原地使能保持 / 断扭矩 |
| `/arm6/stop` | `std_srvs/srv/Trigger` | 清除轨迹并请求保持实测位置 |

关节命令需六个位置；可不提供名称，按 J1–J6 排列，也可将六个名称重排，但不得重复或缺项。
以下仅演示消息格式，具体目标应根据当前实物姿态选择：

```bash
ros2 topic pub --once /arm6/joint_target sensor_msgs/msg/JointState \
  '{name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], position: [0.15, -0.10, -0.14, 0.0, 0.38, 0.0]}'
```

TCP 请求的 `header.frame_id` 必须等于配置中的 `base_frame`。
节点先完成整段位置直线和最短姿态插值的离散逆解，检查限位与关节跳变后再执行。
点间是关节五次插值，所以 TCP 是采样近似直线，各段首尾速度为零。
无连续限位内逆解时拒绝，不跳到另一逆解分支。

夹爪命令：

```bash
ros2 service call /arm6/gripper/open std_srvs/srv/Trigger '{}'
ros2 service call /arm6/gripper/close std_srvs/srv/Trigger '{}'
ros2 topic pub --once /arm6/gripper_target std_msgs/msg/Float64 '{data: 0.03}'
```

夹爪运动保持手臂六轴目标不变。当前为位置开合，未实现抓取力控制、接触检测或 `GripperCommand` action。
服务返回 `Accepted` 只代表请求接收，完成情况以实测反馈及状态为准。

## 5. 关节轨迹 action

`/arm6/follow_joint_trajectory` 使用 `control_msgs/action/FollowJointTrajectory`。

```bash
ros2 action send_goal /arm6/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, -0.10, -0.14, 0.0, 0.38, 0.0], time_from_start: {sec: 10, nanosec: 0}}]}}' \
  --feedback
```

每点必须包含全部六关节，仅接受 positions；velocities、accelerations、effort 数组应为空。
header stamp 必须为零，时间严格递增；t=0 的点仅可等于当前实测位置。
每段使用首尾速度和加速度为零的五次插值，按解析峰值验证速度与加速度。
时间不足会拒绝，不自动改变请求时间。夹爪不包含在该六轴 action 内。

支持逐轴位置 `path_tolerance`、`goal_tolerance` 和 `goal_time_tolerance`。
零位置容差采用默认值，正值可收紧配置默认，负值及速度/加速度容差请求拒绝。
不支持 MultiDOF 或 component tolerance。action 持续返回期望值、实测值和误差，
实测到位且满足稳定时间后才成功。路径误差返回 -4，到位超时返回 -5。

## 6. 反馈、停止与故障

| 反馈 | 内容 |
| --- | --- |
| `/arm6/joint_states` | 前六项 `joint_1`–`joint_6` 为 rad；第七项 `gripper_joint` 为完整开口 m |
| `/arm6/encoder_ticks` | 七电机原始单圈刻度，顺序与 `motor_ids` 一致 |
| `/arm6/tcp_pose` | 当前 TCP 位置和姿态 |
| `/arm6/gripper_width` | 夹爪完整开口 m |
| `/arm6/motion_status` | `read_only`、`holding`、`moving`、`stopped`、`disabled`、`fault` |

```bash
ros2 service call /arm6/stop std_srvs/srv/Trigger '{}'
ros2 service call /arm6/enable std_srvs/srv/SetBool '{data: false}'
```

停止请求保持位置；断扭矩逐轴回读确认。软件停止不能替代物理急停。
通信故障时，保持或断扭矩可能无法确认，日志会报告原因。
故障清除后需要 `enable false` 再 `enable true`，不会自动续跑旧轨迹。
关机时也会尝试断扭矩并关闭串口。每个串口应由本驱动独占。

## 7. 测试与当前边界

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest src/arm6_ros2/test -q -p no:cacheprovider
```

仅保留数学、轨迹、节点状态与 SDK 通信回归测试。节点测试在构造前替换通信类，
SDK 测试使用假端口，不连接真实串口或向电机发送指令。这些测试不会安装为运行入口。

当前代码不包含环境/自碰撞规划、MoveIt、ros2_control 插件、动力学验证或六轴实机验收结论。
增加腕部和工具后，必须重新验证实际负载、TCP、限位和运动扫掠区域。
