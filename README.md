# 六轴机械臂与独立末端夹爪 · ROS 2 Jazzy

本仓库只维护 `arm6_ros2` 实机控制包：六个旋转关节（ID1–6）加一个独立夹爪电机（ID7）。
前三轴保留现有电机型号、编码零位、方向、限位与连杆几何；后三轴、DH 腕部结构及工具偏置暂用可替换假设。

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

使能会保持实测位置，不自动归零。真正运动前需在 `config/arm6.yaml` 中确认七电机标定；
末端位姿运动还需确认 DH 和 TCP。当前暂定参数尚未完成六轴实机验证，不包含碰撞规划。

## 目录

```text
src/arm6_ros2/       唯一 ROS 包：实机控制、运动学、轨迹、配置、启动与测试
代码阅读.md          按文件解释代码及阅读顺序
docs/hardware/      前三轴标定依据与本地原始硬件资料
```

旧三轴/五轴控制器、手柄入口、历史轨迹、模拟运行分支、URDF 示意模型、模拟联调脚本、
旧压缩包及构建产物已移除。保留的单元测试只注入假总线验证最终控制器，不提供模拟运行入口。
