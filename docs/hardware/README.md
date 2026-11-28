# 硬件参数与标定依据

当前程序为六个旋转关节加一个独立夹爪电机。几何由提供的 SolidWorks 总装标准 DH 表等价转换为 Craig 改进 DH，
电机 ID 按接线记录填写；保留的旧编码标定与限位需按 CAD 定义重新验证。

## 前三轴旧标定（保留待 CAD 对齐）

| 参数 | J1 / ID1 | J2 / ID2 | J3 / ID3 |
| --- | --- | --- | --- |
| 型号 | XM540-W270 | XM540-W270 | XM540-W270 |
| 零位 tick | 3819 | 989 | 2013 |
| 编码方向 | +1 | -1 | -1 |
| 运行下限 tick | 0 | 740 | 1043 |
| 运行上限 tick | 4095 | 2700 | 3347 |
| 速度上限 rad/s | 0.02 | 0.02 | 0.02 |

默认配置使用完整 CAD 模型，见 [改进 DH 参数表](../../src/arm6_ros2/README.md#1-电机与几何配置)。
该配置的基座为总装 CAD 坐标系，保存姿态为关节零位，关节正方向按舵机局部 +Z 右手定则。
上表编码器零位与方向仍为旧记录，不能视为已与该 CAD 姿态对齐。
编码映射为 `q=direction*(tick-zero)*2π/4096`。

限位依据：2026-11-13 根据原始《关节角度限制》记录整理。
ID1 没有记录机械挡块，当前仍为 Mode 3 单圈 0–4095，不采用跨零点或连续旋转逻辑。
ID2 原记录 740–3323，建议不超过 2700；小臂完全收缩时上限 2809，因此统一采用 2700。
ID3 原记录为 1047–3347；2026-11-27 按用户要求用保持姿态重新测量下限，
57 个反馈样本为 1043–1044 tick，取最小值 1043，现用 1043–3347。
编码增大与运动学角度增大的方向不必相同，不因“上抬”描述修改符号。

对应默认配置为 `src/arm6_ros2/config/arm6.yaml`；CAD 零姿态末端位置与球腕几何由
`test/test_arm6_model.py` 检查。历史三轴轨迹文件已删除，旧标定数值及依据保留在此。

## 暂定新增硬件

J4–J6 按关节顺序使用 ID5、ID4、ID11，型号填写 XH430-W350（1000）。
其中 ID4 的型号来自已有 Wizard 记录；ID5、ID11 和夹爪的型号需在扫描时一起核对。
夹爪 ID 已由用户确认为 12；闭合 3180 tick、跨零点到 280 tick 张开，实际开口宽度仍待实测。

2026-11-27 用户补充后三轴运行限位；前三轴继续使用上表旧限位。

| 关节 | 电机 ID | 运行限位 | 零位与方向 |
| --- | --- | --- | --- |
| J4 | 5 | −145°～45°；编码限位向内取整为 399–2560 tick | 用户确认 2048 tick 为 0°，方向 +1 |
| J5 | 4 | 0–4095 tick | 2048 tick、方向 +1 仍待 CAD 对齐 |
| J6 | 11 | 0–4095 tick，有限单圈 | 2048 tick、方向 +1 仍待 CAD 对齐 |
| 夹爪 | 12 | 3180–4095 与 0–280 tick | 闭合 3180，沿刻度递增跨零点至 280 张开 |

ID5 下限换算为 398.222… tick，取 399，避免实际角度低于 −145°。
ID11 不采用跨零点或连续旋转逻辑，目标沿区间内的刻度差插值，不允许累计同向多圈。
ID12 由用户于 2026-11-27 更正为跨零点的短弧范围，之前的 350–3150 记录已替换。
程序使用 Mode 4 的连续计数 3180–4376 表示该 1196 tick 区间；4376 对应单圈 280。
仅夹爪使用这一模式，六个关节（包括 ID11）仍为 Mode 3。
驱动在未使能时对齐计数分支，使能后固定分支，避免在原始计数 0 附近重启后多转一圈。
CAD 腕部三轴交于同一腕心，保存姿态 J4/J6 共线；J3 的轴间距 0.0077 mm 按原表保留。
夹爪转接件 CAD 原点位于腕心沿 J6 正方向 73.5 mm 处，已计入 `d6`。
额外 TCP 变换暂为零，实际抓取点需从该原点另行标定。
夹爪完整开口 0.06 m 仍为占位值，需测量实际最大开口宽度后替换。
实测后填写轴线、安装变换、零位、方向、限位和工具参数，再打开对应的 confirmed 开关。

## 用户示教原点

2026-11-27 用户手动摆放并确认后，读取六轴原点为
`[3372,992,1249,1063,1250,2487]` tick，对应 ID `[1,2,3,5,4,11]`。
该待机姿态由 `home_ticks` 保存，`home_confirmed`、`home_on_enable` 为 true。
使能先保持当前位置，再按关节限速回程，夹爪保持实测开口。
此示教位独立于 CAD 零姿态，不更改 `zero_ticks`；回程路径仍需在实物环境中验证。

2026-11-27 用户说明旧零位和方向尚未按 CAD 确认，并提供了折叠待机姿态照片。
该照片与旧映射推算的接近伸直状态不一致，先前模型下的可达性结论需在重新标定后重算。
照片对应的稳定反馈保存于 [标定待办参考数据](encoder_calibration_pending.json)，
只作为待机参考，不能直接代替 CAD 零姿态读数。

随后用户明确要求按当前实物姿态定义操作原点，不采用 CAD 图姿态。
最新保存的用户原点为 `[3449,1227,1416,1117,1298,2525]` tick，
对应 ID `[1,2,3,5,4,11]`，见 [用户原点记录](user_origin.json)。
电脑手动控制直接基于该编码参考，CAD 图及模型编码零位不再作为点动或回用户原点的前置条件。

## 实机标定配置

2026-11-28 的实机试验采用 [arm6_calibration.yaml](../../src/arm6_ros2/config/arm6_calibration.yaml)。
这是运行中控制器加载的临时参数文件的原样归档。它使用 `arm6_calibrated` 基座、实物参考姿态 A0 和实测关节方向。
DH 的 `a`、`d`、`alpha` 来自 CAD，参考角由实物姿态确定。示教回位沿用 `user_origin.json` 的六轴刻度。
模型输出的末端点为夹爪转接件中心，实机配置使能后原地保持。

[实机标定记录](arm6_calibration_2026-11-28.json) 保存 A0、腕部验证姿态、六轴参数与 30 mm 复测的起止读数。
复测通过逆解预检，执行结束时报到位超时。结束后的偏差用于后续排查。

## 离线复现直线逆解失败

用[折叠姿态原始读数](encoder_calibration_pending.json)和默认 `arm6.yaml`，沿旧模型的水平径向向外规划 100 mm，
保持末端朝向和开口。旧编码映射将该姿态算得接近伸展上限，逐毫米逆解在第 **3.0 mm** 无法继续。
这是旧映射的离线复现，与实机标定配置下的 30 mm 终点到位超时属于不同测试现象。

在仓库根目录运行以下代码，只加载模型并调用规划函数。

```bash
source /opt/ros/jazzy/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/src/arm6_ros2${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'PY'
import json
from pathlib import Path

from arm6_ros2.config import load_parameters
from arm6_ros2.forward_line_test import forward_plan
from arm6_ros2.model import ArmModel

record = json.loads(Path('docs/hardware/encoder_calibration_pending.json').read_text())
model = ArmModel(load_parameters('src/arm6_ros2/config/arm6.yaml'))
try:
    forward_plan(model, model.decode(record['measured_ticks']), 'radial', 0.005)
except ValueError as error:
    print(error)
    if 'unreachable at 3.0 mm of 100 mm' not in str(error):
        raise
else:
    raise SystemExit('Expected the reference path to fail at 3.0 mm')
PY
```

## 原始资料

本地原始文件 `关节角度限制.docx` 和 `机械臂图片.jpg` 已移入本目录，保留作后续机械测量参考，
不属于程序依赖，按 `.gitignore` 规则不提交 Git。硬件说明和关键标定数值随源码提交。
