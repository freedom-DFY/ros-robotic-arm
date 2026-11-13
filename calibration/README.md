# 三轴实机配置与路径记录

当前维护的流程为 **使能保持 → 返回固定起点 → 到位稳定 → 20 cm 水平直线**。

| 入口 | 内容 |
| --- | --- |
| [实机操作命令](arm3_horizontal_up_20cm/执行20cm实机命令.txt) | 编译、启动、使能、执行、查看状态及停止 |
| [配置与坐标说明](arm3_horizontal_up_20cm/README.md) | 固定起点、零位、速度、限位和到位条件 |
| [实机配置](arm3_horizontal_up_20cm/arm3_20cm_EXECUTE.yaml) | ROS 2 节点实际加载的参数 |
| [关节限位与修改说明](关节限位说明.md) | 文档中的机械范围、运行上限和归位边界 |
| [规划数据](arm3_horizontal_up_20cm/plan.json) | 路径和固定起点流程的参数记录 |
| [路径采样](arm3_horizontal_up_20cm/waypoints.csv) | 起点及 200 个笛卡尔采样点 |

通用控制代码与自动测试位于 [src/arm3_ros2](../src/arm3_ros2/README.md)。此目录保存当前机械臂的数据；零位、固定起点和软件允许窗口应分别核对，不能互相替代。
