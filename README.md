# 六轴机械臂与独立末端夹爪 · ROS 2 Jazzy

本仓库只维护 `arm6_ros2` 实机控制包：J1–J6 的电机 ID 依次为 `[1,2,3,5,4,11]`，另加一个独立夹爪电机。
夹爪 ID 为 12，闭合 3180 tick，沿 3180→4095→0→280 张开；实际开口宽度仍待标定。
几何由提供的 SolidWorks 总装标准 DH 表等价转换为 Craig 改进 DH，基座参考系为 `arm6_cad`，零姿态为 CAD 保存姿态。
末端参考点为夹爪转接件 CAD 原点；现有编码零位、方向与限位仍需与 CAD 对齐，抓取点 TCP 待实测。

- [安装、配置与操作](src/arm6_ros2/README.md)
- [代码阅读：每个文件的用途和调用关系](代码阅读.md)
- [前三轴标定依据与暂定硬件](docs/hardware/README.md)

## 安装与启动

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select arm6_ros2 --symlink-install
source install/setup.bash
ros2 launch arm6_ros2 arm6.launch.py device:=/dev/ttyUSB0 baud_rate:=1000000
```

DYNAMIXEL SDK 使用系统依赖 `ros-jazzy-dynamixel-sdk`，仓库不再包含上游多语言源码和示例。
当前机器若未安装 SDK，先运行上述 `rosdep install`；也可执行：

```bash
sudo apt install ros-jazzy-dynamixel-sdk
```

启动后只读七个电机的反馈，不自动上扭矩。确认电机状态后，在另一个已加载环境的终端手动使能：

```bash
ros2 service call /arm6/enable std_srvs/srv/SetBool '{data: true}'
```

使能先保持实测位置，再按关节速度限制回到用户示教原点
`[3449,1227,1416,1117,1298,2525]` tick（ID1/2/3/5/4/11），夹爪保持当前开口。
`home_on_enable: false` 可关闭自动回原点，恢复为仅原地保持。
回原点按用户要求允许每轴 3° 误差，已在范围内时保持当前位置。
该原点独立于 CAD 编码零位；普通运动仍需确认七电机标定，末端位姿运动还需确认 DH 和 TCP。
当前程序不包含碰撞规划，回程路径需在实物环境中验证。

电脑鼠标/键盘操作面板：

```bash
ros2 run arm6_ros2 desktop_control
```

面板以用户当前示教姿态为相对 0°，可做六轴单独点动、夹爪位置开合、回用户原点、停止和扭矩控制。
这些手动编码器操作不要求实物摆成 CAD 图；CAD 姿态仅用于另行建立末端几何映射。
键盘点动默认关闭，勾选后数字 1–6 选轴，左右方向键减/加刻度，Esc 停止并保持。

已采集 F710 映射后，可用 `ros2 run arm6_ros2 gamepad_control` 打开手柄控制窗口。
先回中并点击启用，再按住 LB 进行关节小步控制；松开停止，不依赖末端几何标定。
实际按键及启动驱动步骤见包内 F710 操作说明。

水平向前 10 cm 的实机测试入口：`ros2 run arm6_ros2 forward_line_test`。
它在在线原点、标定和完整直线路径预检通过后发送真实轨迹，详见包内操作说明。

## 目录

```text
src/arm6_ros2/       唯一 ROS 包：实机控制、运动学、轨迹、配置、启动与测试
代码阅读.md          按文件解释代码及阅读顺序
docs/hardware/      前三轴标定依据与本地原始硬件资料
```

旧三轴/五轴控制器、历史手柄入口、历史轨迹、模拟运行分支、URDF 示意模型、模拟联调脚本、
旧压缩包及构建产物已移除。保留的单元测试只注入假总线验证最终控制器，不提供模拟运行入口。
