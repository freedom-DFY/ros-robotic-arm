# arm6_ros2：六轴机械臂 + 独立末端夹爪

控制节点入口为 `arm6_node`，启动文件为 `arm6.launch.py`。本包连接实机串口，
控制六个旋转关节和一个独立夹爪，不包含仿真节点、URDF 模型或 `dry_run` 参数。
代码阅读说明见工作空间根目录的 [代码阅读.md](../../代码阅读.md)。

## 电脑控制与用户原点

连接控制节点后打开电脑操作面板：

```bash
ros2 run arm6_ros2 desktop_control
```

用户原点直接采用实物当前姿态，不要求 CAD 图姿态。当前保存的六轴原点为
`[3449,1227,1416,1117,1298,2525]` tick，对应 ID `[1,2,3,5,4,11]`。
面板相对角度为 `(实际tick−用户原点tick)×360/4096`，刻度增加方向计正；
这是操作参考，不修改末端模型的 `zero_ticks`、DH 偏置或几何确认状态。

面板提供六轴 ±20 tick 单轴点动、夹爪开合、回用户原点、实时刻度/相对角度、
使能保持、断扭矩和停止。键盘默认关闭，启用后数字 1–6 选轴，左右方向键点动，Esc 停止。
手动关节、夹爪和回用户原点不依赖 CAD 标定确认，仍执行实测编码范围、速度、状态及通信检查。
夹爪手动开合只写 ID12 的目标，六轴保持；关节点动只写所选电机，其他目标不变。

“设当前为本次原点”在保持状态下即时更新操作参考并保存一份记录；不会移动电机。
`home_ticks` 是启动时的原点默认值，运行中原点以 `/arm6/user_origin_ticks` 为准。
该按钮更新当前控制会话；再次启动控制器时使用配置中保存的 `home_ticks`。
关闭电脑面板不关闭独立控制节点，不自动断扭矩。原有标定面板独立保留。

| 手动接口 | 类型 | 内容 |
| --- | --- | --- |
| `/arm6/manual/joint_1/jog` 至 `joint_6/jog` | `SetBool` | true 加刻度，false 减刻度 |
| `/arm6/manual/gripper/open`、`close` | `Trigger` | 已测夹爪区间内慢速开合 |
| `/arm6/set_user_origin` | `Trigger` | 当前六轴实测位置设为本次用户原点 |
| `/arm6/home` | `Trigger` | 回用户原点，夹爪开口保持 |
| `/arm6/user_origin_ticks` | `Int32MultiArray` | 当前六轴用户原点 |

## F710 手柄输入编号采集

F710 使用 XInput（X）模式。首次可在 MODE 灯熄灭时采集；采集完成后必须保持当时的 MODE 状态，
不能切换后继续使用旧映射。先启动只读输入驱动：

```bash
ros2 run joy joy_enumerate_devices
ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=20.0 -p sticky_buttons:=false
```

另一个终端打开采集窗口：

```bash
ros2 run arm6_ros2 gamepad_monitor --output "$PWD/docs/hardware/f710_mapping.json"
```

松开控件后点击开始，依次完成左摇杆上/右、右摇杆上/右、十字键上/下、LB、A、B，
每次操作后松开，窗口自动保存轴或按钮的实际编号和方向。这里只订阅 `/joy`，
不发布机械臂命令，不需要关闭已有电脑控制面板。编号尚未采齐时不能视为手柄控制已就绪。
本机已识别 F710 XInput，设备 ID 0，`joy_node` 实际输出 8 axes、11 buttons。
其他电脑或驱动的编号仍需实测，不能直接套用 `game_controller_node` 的按钮表。
9 个动作完成后点击“确认操作名称无误”，才可将该映射用于手柄控制。

### F710 实机关节点动

保持机械臂控制器已使能并处于 `holding` / `stopped`，打开：

```bash
ros2 run arm6_ros2 gamepad_control --mapping "$PWD/docs/hardware/f710_mapping.json"
```

窗口默认关闭手柄输入。松开全部按键、摇杆回中后，点击“启用手柄输入”。
该按钮只启用手柄客户端，不自动使能电机、不回原点。

| 操作 | 功能 |
| --- | --- |
| 松开 LB，十字键上/下 | 循环选择 J1–J6 |
| 按住 LB，左摇杆上/下 | 所选关节加/减刻度，其他电机保持目标不重写 |
| 摇杆回中或松开 LB | 停止本次点动；再次操作前先松 LB 并回中 |
| 摇杆回中，按住 LB+A | 夹爪开合交替；每次按 A 切换方向，持续按住才连续小步运动 |
| B | 停止，并关闭手柄输入；重新操作需回中后再启用 |

关节每次最多 20 tick，沿用控制器低速限位。夹爪经 `/arm6/manual/gripper/jog` 每次最多 20 tick，
不把整段开合行程一次发送给电机。客户端只有在输入持续有效、LB 保持且上一小步完成后才请求下一步。
收不到 `/joy` 超过 0.3 s，或机械臂反馈超过 0.5 s，会停止本次手柄运动并关闭手柄输入；
断开恢复后不自动继续。窗口关闭同样请求停止本次手柄运动，独立控制器继续保持扭矩。
控制循环在独立 ROS 执行线程中运行，窗口拖动不会暂停输入超时检查。

本次用户已确认的实际映射为：左摇杆上/右 axes 7/6，右摇杆上/右 axes 4/3，
十字键上/下 axis 1 的正/负方向；LB/A/B 为 buttons 4/0/1。
该布局按实际采集使用，不根据默认按钮表强行重排，也不在控制中切换 MODE。
当前版本右摇杆仅用于确认回中，不绑定机械臂运动。电脑面板仍可作为备用停止入口。

参考：[F710 手册](https://www.logitech.com/assets/34885/2/f710620-002923003403gswamr.pdf)、
[ROS joy 驱动说明](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md)。

## 1. 电机与几何配置

| 项目 | 当前配置 |
| --- | --- |
| J1–J3 | XM540-W270，ID1/2/3，型号号 1120 |
| 前三轴零位/方向（旧标定，待 CAD 对齐） | `[3819,989,2013]` / `[1,-1,-1]` |
| 前三轴运行限位 | `[0,740,1043]` 至 `[4095,2700,3347]` tick；ID3 下限由当前保持姿态重测 |
| 几何来源 | 用户提供的 SolidWorks 总装标准 DH 表，已等价转换为下列 Craig 改进 DH 表 |
| 前三轴速度上限 | 0.02 rad/s |
| J4–J6 | XH430-W350，依次为 ID5/4/11，型号号 1000；ID5 以 2048 tick 为零、方向 +1；ID4/11 零位 2048、方向 +1 仍待 CAD 对齐 |
| 后三轴运行限位（用户提供） | J4 / ID5：−145°～45°，向内取整为 399–2560 tick；J5 / ID4、J6 / ID11：0–4095 tick |
| 腕部 | CAD 中 J4/J5/J6 交于同一腕心，保存姿态 J4/J6 共线 |
| 末端参考点 / TCP | J6 正向距腕心 73.5 mm 的夹爪转接件 CAD 原点；额外 TCP 平移和姿态偏置暂为零，抓取点待实测 |
| 夹爪 | XH430-W350，型号号 1000；ID12：3180 tick 闭合，沿 4095→0 至 280 tick 张开；完整开口 0.06 m 仍为待标定值 |

夹爪不占手臂六个旋转自由度，系统共七个电机。
前三轴的标定依据整理在 [硬件说明](../../docs/hardware/README.md)。
控制表参考 ROBOTIS 官方手册：[XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)、
[XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)、
[XH430-W350](https://emanual.robotis.com/docs/en/dxl/x/xh430-w350/)。

当前驱动支持上述三个型号在本程序中使用的 X 系列控制表和 4096 刻度单圈编码。
所有电机应连接兼容的 TTL 或 RS-485 电气总线，使用相同波特率、Protocol 2.0、
Homing Offset 0，并关闭 Torque On by Goal Update。六轴使用 Position Control Mode 3；
当前跨零点的夹爪使用 Extended Position Control Mode 4。
程序不自动修改运行模式或 EEPROM 参数。

夹爪的 `gripper_wraps_zero: true` 将低端点提升一圈，内部连续范围为 3180–4376，
其中 4376 的单圈读数为 280。驱动对齐实机计数所在分支，回读和目标都使用同一分支，
只允许在这段 1196 tick 的短弧内运动，不允许通过 281–3179 或累计旋转多圈。
使能后若反馈离开这段连续范围（包括计数复位），直接报故障，不取模掩盖异常。

ID11 按有限单圈关节处理，0 和 4095 是两个独立边界，不跨零点、不累计圈数。
例如从 4090 tick 到 10 tick，目标刻度沿 4090→…→10 递减，不能沿同方向越过 4095 绕回 0。
普通关节目标、轨迹 action 和逆解均受同一组限位约束，越限目标直接拒绝。

基座参考系为总装 CAD 坐标系 `arm6_cad`。以总装保存姿态为 `q_i=0`，
关节正方向按对应舵机局部 +Z 的右手定则。运行使用 Craig 改进 DH，关节 i 绕 z_i 转动：

```text
T(i-1,i) = Rx(alpha_(i-1)) Tx(a_(i-1)) Rz(q_i + offset_i) Tz(d_i)
T_base_tcp = T01 T12 T23 T34 T45 T56 T_adapter_tcp
```

| 关节 i | a_(i-1) (m) | d_i (m) | alpha_(i-1) (rad) | offset_i (rad) |
| --- | --- | --- | --- | --- |
| J1 | 0 | 0.0495 | 0 | 1.5256376777287952 |
| J2 | 0 | 0 | +π/2 | 1.0563936910253557 |
| J3 | 0.2489377 | -0.011977 | 0 | 0.20237092676874252 |
| J4 | 0.0000077 | 0.2905981 | +π/2 | -π/2 |
| J5 | 0 | 0 | +π/2 | 0 |
| J6 | 0 | 0.0735 | -π/2 | 0 |

配置仍沿用 `dh_*` 参数名，但 `dh_a_m`、`dh_alpha_rad` 每行现在填写上一索引的
`a_(i-1)`、`alpha_(i-1)`；`dh_d_m` 和 `dh_offsets_rad` 仍为本关节的 d_i 和角偏置。
外部 YAML 若保存了旧标准 DH 参数，也必须按上述表转换后使用。

转换方法：将原标准 DH 每行写作 `Z_i X_i`，改进 DH 每行写作 `X_(i-1) Z_i`，
其中 `X_i=Tx(a_i)Rx(alpha_i)`，`Z_i=Rz(q_i+offset_i)Tz(d_i)`。
本模型取 `X_0=I`，原标准表的 `a6=alpha6=0`，因此 `X_6=I`，
六轴连乘结果完全相同，无需附加基座或末端修正。

该表完整替换此前各 170 mm 的简化连杆模型；DH 长度按转轴几何定义。
保留 J3 的微小轴间距 `a3=0.0077 mm`，不在默认配置中近似为零。
`d6=0.0735 m` 已包含腕心到转接件原点的距离，不能在 TCP 中重复添加。
`tcp_xyz_m`、`tcp_rpy_rad` 定义从该原点到实际抓取点的附加变换，当前均为零。

全零关节角回算转接件位置约为 `(9.207413,469.062426,154.453681) mm`，
与提供的 CAD 参考 `(9.2073,469.0624,154.4533) mm` 各分量相差小于 0.001 mm。
测试同时检查球腕三轴相交、保存姿态 J4/J6 共线，并以原标准 DH 表为独立基准，
对零姿态、关节边界和随机关节角比较完整末端位置与姿态。

编码换算仍为 `q=(tick-zero)*direction*2π/4096`；`dh_offsets_rad` 是 CAD 的几何角偏置，
不替代编码器零位。当前 `zero_ticks`、`joint_directions` 和运行限位保留旧值，
必须在 CAD 保存姿态下记录六轴编码器值，并按各舵机局部 +Z 复核方向和限位。
`calibration_confirmed` 和 `geometry_confirmed` 继续为 false，完成对应实机验证后再开启。
这两个参数是人工确认状态，不是运行时自动标定检测结果。
`home_confirmed`/`home_arrival_tolerance_rad` 只描述是否已示教原点及距原点多远，
不能代替编码零位/方向与 CAD 的对应关系。程序分别报告原点偏差、人工标定确认及运动限位问题。
需要放宽“大致原点”时只调整 `home_arrival_tolerance_rad`；
`position_min_ticks`/`position_max_ticks` 始终使用电机实测允许运动区间。

用户提供的折叠待机照片与旧编码映射推算的接近伸直姿态不一致，当前映射尚不能作为
实物可达空间的依据。必须重新记录 CAD 姿态零位、核对方向，再重新规划先前的直线路径。

### 单轴点动记录 CAD 零位

控制器已使能保持、`calibration_confirmed=false` 时，可打开：

```bash
ros2 run arm6_ros2 encoder_calibration \
  --reference-image "$PWD/docs/hardware/最终机械臂图片.png" \
  --output "$PWD/docs/hardware/encoder_calibration_record.json"
```

每个按钮只点动一个关节的原始目标刻度（默认 ±20 tick，约 1.76°），
速度不超过 0.02 rad/s，加速度不超过 0.04 rad/s²，其他五轴及夹爪的保持目标不重写。
点动使用电机实测范围和反馈，不依赖未对齐的末端几何，也不更改 PID。
对应服务为 `/arm6/calibration/joint_1/jog` 至 `joint_6/jog`，类型 `SetBool`，
`true` 加刻度、`false` 减刻度；返回接收成功后等待状态恢复 `holding` 再下一步。
`/arm6/stop` 在点动时只停止并保持当前选中电机，不改写其他电机目标。

按参考图调整姿态，逐轴观察刻度增大是否与 CAD 本轴 +Z 右手正方向一致，
一致填 +1，相反填 −1。对齐后勾选 CAD 姿态并记录，面板保存稳定实测零位及人工核对的方向。
记录不会自动改写控制器参数、置确认标志或启动直线运动；应用记录前需核对内容。
标定会话应以 `home_on_enable=false` 运行，避免重新使能时自动离开正在调整的姿态。

## 2. 安装与启动

末端夹爪 ID 已由用户确认为 **12**，已同步填写到 `config/arm6.yaml` 和
`arm6_ros2/config.py`。总线波特率使用 **1000000（1 Mbps）**。
七项顺序为 **J1、J2、J3、J4、J5、J6、夹爪**，即 `[1,2,3,5,4,11,12]`，
必须互不重复。连接时仍需核对 ID5、ID11 和夹爪的型号号是否为 1000。

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
`baud_rate:=` 启动参数会覆盖 YAML 的波特率。
首次启动保持只读，先按上述 `motor_ids` 顺序核对七个反馈槽位：

```bash
ros2 topic echo /arm6/encoder_ticks
```

调用 `/arm6/enable` 会给全部七个电机上扭矩，先原地保持，再按当前配置自动回到
用户示教原点。该编码器原点由 `home_confirmed` 确认，独立于仍为 `false` 的
`calibration_confirmed` 和 `geometry_confirmed`：

```bash
ros2 service call /arm6/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /arm6/joint_states
ros2 topic echo /arm6/tcp_pose
ros2 topic echo /arm6/motion_status
```

使能先检查模式、偏置、内部限位和硬件错误，读取当前位置，在任何轴上扭矩前，
写入并回读全部保持目标，再逐轴使能和确认。中途失败会尝试撤销本次扭矩使能。
当前示教原点按 J1–J6 为 `[3449,1227,1416,1117,1298,2525]` tick，
电机 ID 顺序为 `[1,2,3,5,4,11]`。回程采用区间内的关节五次插值，沿用各轴速度和加速度上限，
ID11 不跨零点；夹爪保持使能时实测开口。服务成功表示开始回程，
等待 `/arm6/motion_status` 从 `moving` 变为 `holding` 才表示实测到位。
按用户要求，回原点单独允许各轴 3° 误差（`home_arrival_tolerance_rad`），
已在该范围内时直接保持当前位置。普通运动及直线测试的跟踪、到位误差不随之放宽。
结束前回读全部电机的最终目标，未接收时最多原值重发三次；回读成功仍不等于实测到位。
重复调用已使能节点不会重新启动回程；断扭矩后再次使能会重新回程。

自动回原点会在上扭矩前拒绝未确认原点、超出运动边界的当前位置或目标。
将 `home_on_enable` 设为 `false` 后，使能仅原地保持，允许在应用边界外保持实测位置。
示教原点不修改 `zero_ticks`、CAD 定义或电机 EEPROM，普通运动仍受原标定确认约束。
当前没有碰撞规划，需在实物环境中验证回程路径。
当前实机调试中已确认回程最终目标可被接收，但 ID3/ID4 的到位偏差仍需排查；
到位超时会中止轨迹并尝试保持实测位置，不自动放宽容差或反复重试。

## 3. 正式参数

默认配置为 [config/arm6.yaml](config/arm6.yaml)。
[config/arm6_calibration.yaml](config/arm6_calibration.yaml) 保存实机试验所用参数，基座为 `arm6_calibrated`，
使能时保持实测位置。需要加载该配置时，从工作空间根目录运行：

```bash
ros2 launch arm6_ros2 arm6.launch.py \
  config:="$PWD/src/arm6_ros2/config/arm6_calibration.yaml" \
  device:=/dev/ttyUSB0 baud_rate:=1000000
```

`device`、`baud_rate` 以 launch 参数为准；其余参数在启动时从 YAML 读取，运行中只读。
`config:=` 也可指定其他配置的绝对路径。两份配置的区别和实测依据见[硬件标定记录](../../docs/hardware/README.md#实机标定配置)。
不通过 launch、直接执行 `ros2 run arm6_ros2 arm6_node --ros-args --params-file ...` 时，
串口设置采用参数文件中的值。

| 参数 | 填写内容 |
| --- | --- |
| `motor_ids`、`expected_model_numbers` | 七项：六关节后接夹爪 |
| `home_on_enable` | true 时每次从未使能切换到使能后自动回示教原点；false 时只原地保持 |
| `home_confirmed`、`home_ticks` | 示教原点确认与六轴刻度，顺序 J1–J6；当前由用户手动摆放后读取确认 |
| `home_arrival_tolerance_rad` | 回原点专用容差，当前每轴 3°；不改变普通运动容差 |
| `zero_ticks`、`joint_directions` | 六项：每轴编码零位与方向 |
| `position_min_ticks`、`position_max_ticks` | 六项：统一用于普通运动、逆解和执行检查的运行边界 |
| `dh_a_m`、`dh_d_m`、`dh_alpha_rad`、`dh_offsets_rad` | 各六项：Craig 改进 DH；依次为每行的 a_(i-1)、d_i、alpha_(i-1)、角偏置 |
| `tcp_xyz_m`、`tcp_rpy_rad` | 各三项：转接件 CAD 原点到抓取点 TCP 的平移和固定轴 roll/pitch/yaw |
| `gripper_closed_ticks`、`gripper_open_ticks`、`gripper_max_width_m` | 夹爪端点及完整开口，支持编码方向相反 |
| `gripper_wraps_zero` | 夹爪范围是否跨编码器零点；true 时仅夹爪要求 Mode 4，使用有限连续区间 |
| `joint_velocity_limits_rad_s`、`joint_acceleration_limits_rad_s2` | 各六项：关节速度和加速度上限 |
| `gripper_speed_m_s`、`gripper_acceleration_m_s2` | 夹爪开口速度、加速度上限 |
| `calibration_confirmed` | 七电机标定、方向和运行边界验证后设 true，允许实机运动 |
| `geometry_confirmed` | DH 和 TCP 实测验证后设 true，额外允许末端位姿目标 |

ID5、ID11 和夹爪的型号仍待扫描核对；全部六轴编码零位、方向和限位需按 CAD 定义重新验证，
夹爪实际开口宽度、实际抓取点 TCP 及 CAD 几何与实物的一致性也待验证，
`calibration_confirmed`、`geometry_confirmed` 均保持 false，示教原点的 `home_confirmed` 为 true。
普通运动请求仍受标定确认约束，自动回示教原点单独受原点确认约束。
六轴运行边界必须位于电机内部位置限位中；Mode 4 夹爪由软件强制限制为测量短弧。

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
| `/arm6/encoder_ticks` | 六轴单圈刻度，夹爪为对齐后的连续刻度（3180–4376）；顺序与 `motor_ids` 一致 |
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

## 7. 实机水平向前 10 cm 测试

在已连接并保持的机械臂上执行：

```bash
ros2 run arm6_ros2 forward_line_test
```

此命令是真实运动客户端。默认以当前大致原点的末端位置为起点，
沿底座朝夹爪的水平径向向外移动 0.10 m，基座坐标 z 和夹爪姿态保持不变，夹爪开口不变。
可用 `--axis x` 或 `--axis y` 指定基座正 X/Y 方向，负方向用 `--axis=-x` 或 `--axis=-y`。
也可用 `--bearing-deg` 指定相对水平向外方向的转角：正值向左，负值向右。
默认 TCP 峰值速度上限 0.005 m/s，实际时长同时受关节速度和加速度约束。

2026-11-27 的旧映射在当时记录的折叠姿态附近，保持姿态水平向右后方 10 cm 的候选命令为：

```bash
ros2 run arm6_ros2 forward_line_test --bearing-deg=-105
```

2026-11-27 根据在线编码器 `[3371,987,1219,1059,1220,2487,3550]` 规划，
模型中 TCP 从约 `(235.1,285.0,469.9)` mm 到 `(293.1,203.5,469.9)` mm，
各轴距限位至少约 176 tick，预计时长约 87 s。这是该起点的几何可行候选；
每次运行仍从新鲜反馈重新规划，不把这组末端关节刻度当成可直接跳转的命令。

客户端读取控制器在线参数和实时编码器，检查处于 `holding` 且在原点 3° 范围内，
以及编码标定、几何确认状态，再对整个 100 mm 行程按不超过 1 mm 间距预检逆解和限位。
任何点不可达则不发送轨迹；不会用关节端点插值替代所要求的末端直线，也不会自动修改 PID。
整条轨迹通过现有 action 执行并等待实测结果，途中编码器推算的 TCP 偏离直线超过 5 mm
或反馈超时会取消本次轨迹并请求保持；Ctrl+C 同样取消本次轨迹，不退出控制节点。

输出的位移来自编码器和模型推算，实物 100 mm 距离可用直尺另行核对。
大致回原点不代表几何已标定；两个运动标定确认仍须按实测结果填写。
旧编码映射的逆解失败可以用[保存的读数和离线复现命令](../../docs/hardware/README.md#离线复现直线逆解失败)核对。

## 8. 软件回归与当前边界

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest src/arm6_ros2/test -q -p no:cacheprovider
```

仅保留数学、轨迹、节点状态与 SDK 通信回归测试。节点测试在构造前替换通信类，
SDK 测试使用假端口，不连接真实串口或向电机发送指令。这些测试不会安装为运行入口。

当前代码不包含环境/自碰撞规划、MoveIt、ros2_control 插件、动力学验证或六轴实机验收结论。
增加腕部和工具后，必须重新验证实际负载、TCP、限位和运动扫掠区域。
