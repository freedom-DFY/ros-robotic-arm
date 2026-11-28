"""Manual CAD-zero calibration panel; selected-motor jogs and measured capture."""

import argparse
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import messagebox, ttk
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import SetBool, Trigger


class CalibrationPanel:
    def __init__(self, node, output, reference_image):
        self.node, self.output = node, output
        self.root = tk.Tk()
        self.root.title('机械臂编码零位标定：单轴点动 / 其他轴保持')
        self.root.geometry('1230x780')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.state = None
        self.samples = deque(maxlen=60)
        self.last_feedback = 0.
        self.pending = None
        self.busy_until = 0.
        parameters = AsyncParameterClient(node, '/arm6')
        if not parameters.wait_for_services(timeout_sec=5.):
            raise RuntimeError('机械臂控制节点不在线')
        names = ['motor_ids', 'calibration_jog_step_ticks', 'calibration_confirmed']
        future = parameters.get_parameters(names)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or any(v.type == 0 for v in future.result().values):
            raise RuntimeError('请先启动包含单轴点动接口的新控制器')
        values = dict(zip(names, map(parameter_value_to_python, future.result().values)))
        if values['calibration_confirmed']:
            raise RuntimeError('当前标定已确认，点动标定入口已关闭')
        self.ids = values['motor_ids']
        self.step = values['calibration_jog_step_ticks']
        self.jog_clients = [node.create_client(SetBool, f'/arm6/calibration/joint_{i+1}/jog')
                            for i in range(6)]
        self.stop_client = node.create_client(Trigger, '/arm6/stop')
        self.tick_sub = node.create_subscription(Int32MultiArray, '/arm6/encoder_ticks', self.ticks, 10)
        self.state_sub = node.create_subscription(String, '/arm6/motion_status', self.status, 10)

        ttk.Label(self.root, text='只点动选中电机；其他关节保持目标不重写。夹爪不参与标定点动。',
                  font=('', 13)).pack(pady=8)
        body = ttk.Frame(self.root)
        body.pack(fill='both', expand=True, padx=12)
        controls = ttk.Frame(body)
        controls.pack(side='left', fill='both', padx=(0, 10))
        reference = ttk.Frame(body)
        reference.pack(side='right', fill='both', expand=True)
        for column, label in enumerate(('关节 / ID', '实测 tick', '减刻度', '加刻度', 'CAD 正方向')):
            ttk.Label(controls, text=label).grid(row=0, column=column, padx=5, pady=6)
        self.readouts, self.buttons, self.signs = [], [], []
        for i in range(6):
            ttk.Label(controls, text=f'J{i+1} / ID{self.ids[i]}').grid(row=i+1, column=0, padx=5, pady=10)
            value = tk.StringVar(value='等待反馈')
            ttk.Label(controls, textvariable=value, width=10).grid(row=i+1, column=1)
            self.readouts.append(value)
            for column, positive in ((2, False), (3, True)):
                button = ttk.Button(controls, text=f'{"+" if positive else "−"}{self.step}', width=7,
                                    command=lambda i=i, positive=positive: self.jog(i, positive))
                button.grid(row=i+1, column=column, padx=3)
                self.buttons.append(button)
            sign = tk.IntVar(value=0)
            self.signs.append(sign)
            box = ttk.Frame(controls)
            box.grid(row=i+1, column=4)
            ttk.Radiobutton(box, text='+1 同向', variable=sign, value=1).pack(anchor='w')
            ttk.Radiobutton(box, text='−1 反向', variable=sign, value=-1).pack(anchor='w')
        ttk.Label(controls, text=(f'每步 {self.step} tick ≈ {self.step*360/4096:.2f}°；'
                                 '速度不超过 0.02 rad/s。\n'
                                 '正方向判断：tick 增大时，若沿 CAD 本轴 +Z 的\n'
                                 '右手旋转方向运动，选 +1；反之选 −1。\n'
                                 '必须观察实物，不能直接沿用旧符号。'),
                  justify='left').grid(row=7, column=0, columnspan=5, pady=14)
        self.stop_button = ttk.Button(controls, text='停止当前单轴点动并保持', command=self.stop)
        self.stop_button.grid(row=8, column=0, columnspan=5, sticky='ew', pady=6)
        self.pose_confirmed = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls, text='实物已与右侧 CAD 保存姿态对齐',
                        variable=self.pose_confirmed).grid(row=9, column=0, columnspan=5, sticky='w', pady=6)
        ttk.Button(controls, text='记录当前 CAD 零位及已核对方向', command=self.capture).grid(
            row=10, column=0, columnspan=5, sticky='ew', pady=8)
        ttk.Label(controls, text='记录使用实际反馈，不使用点动目标。\n保存不会自动执行直线或改写控制参数。',
                  justify='left').grid(row=11, column=0, columnspan=5, sticky='w', pady=4)
        ttk.Label(reference, text='CAD 保存姿态参考（不是用户的折叠待机姿态）').pack()
        try:
            from PIL import Image, ImageTk
            source = Image.open(reference_image)
            source.thumbnail((650, 410))
            self.reference_photo = ImageTk.PhotoImage(source)
            ttk.Label(reference, image=self.reference_photo).pack(pady=8)
        except Exception:
            ttk.Label(reference, text=f'请打开参考图：\n{reference_image}', wraplength=550).pack(pady=10)
        ttk.Label(reference, text=('按单轴逐步接近参考姿态，避开桌面和线缆。\n'
                                  '电机保持扭矩期间，不要手动掰动关节。\n'
                                  '拍照姿态和 CAD 姿态不同，不能只把照片时的\n'
                                  '读数填成 CAD 零位。'), justify='left').pack(pady=12)
        self.message = tk.StringVar(value='等待实时反馈；尚未发送任何点动命令。')
        ttk.Label(self.root, textvariable=self.message, wraplength=1190).pack(fill='x', padx=12, pady=10)
        self.root.after(50, self.poll)

    def ticks(self, message):
        if len(message.data) == 7:
            self.samples.append((time.monotonic(), list(message.data)))
            self.last_feedback = time.monotonic()

    def status(self, message):
        self.state = message.data

    def jog(self, index, positive):
        if self.pending is not None or time.monotonic() < self.busy_until:
            return
        client = self.jog_clients[index]
        if not client.service_is_ready():
            self.message.set('单轴点动服务未就绪，请检查控制器版本。')
            return
        self.pending = client.call_async(SetBool.Request(data=positive))
        self.busy_until = time.monotonic()+0.5

    def stop(self):
        if self.state == 'calibrating' and self.stop_client.service_is_ready():
            self.pending = self.stop_client.call_async(Trigger.Request())

    def poll(self):
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0.)
        if self.pending is not None and self.pending.done():
            try:
                response = self.pending.result()
                self.message.set(('已接收：' if response.success else '拒绝：')+response.message)
            except Exception as exc:
                self.message.set(f'请求失败：{exc}')
            self.pending = None
        fresh = time.monotonic()-self.last_feedback < 0.3
        available = fresh and self.state in ('holding', 'stopped') and self.pending is None
        available = available and time.monotonic() >= self.busy_until
        for button in self.buttons:
            button.configure(state='normal' if available else 'disabled')
        self.stop_button.configure(state='normal' if self.state == 'calibrating' else 'disabled')
        if self.samples:
            for value, tick in zip(self.readouts, self.samples[-1][1][:6]):
                value.set(str(tick))
        if not fresh:
            self.message.set('实时反馈已断开；点动已禁用。')
        self.root.after(50, self.poll)

    def capture(self):
        if self.state not in ('holding', 'stopped') or time.monotonic()-self.last_feedback > 0.3:
            messagebox.showerror('不能记录', '请等待点动结束及稳定实时反馈。')
            return
        directions = [value.get() for value in self.signs]
        if not self.pose_confirmed.get() or any(v not in (-1, 1) for v in directions):
            messagebox.showerror('尚未确认', '请确认 CAD 姿态，并逐轴观察后选择六个方向。')
            return
        recent = [row for when, row in self.samples if time.monotonic()-when <= 0.8]
        if len(recent) < 10 or np.max(np.ptp(np.array(recent)[:, :6], axis=0)) > 4:
            messagebox.showerror('暂不记录', '姿态尚未稳定，请稍后记录。')
            return
        zeros = np.rint(np.median(np.array(recent)[:, :6], axis=0)).astype(int).tolist()
        record = dict(captured_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds'),
                      motor_ids=self.ids, zero_ticks=zeros, joint_directions=directions,
                      cad_pose_confirmed_by_user=True, directions_confirmed_by_user=True,
                      measured_ticks=recent[-1], sample_count=len(recent),
                      note='Measured CAD-pose zero and observed directions. Not yet applied to controller configuration.')
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
        self.message.set(f'已记录至 {self.output}；请告知助手读取并应用，当前控制参数尚未改变。')
        messagebox.showinfo('已记录', f'零位：{zeros}\n方向：{directions}\n已保存：{self.output}')

    def close(self):
        if self.state == 'calibrating' and self.stop_client.service_is_ready():
            future = self.stop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.)
        self.root.destroy()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path.cwd()/'encoder_calibration_record.json')
    parser.add_argument('--reference-image', type=Path,
                        default=Path.cwd()/'docs/hardware/最终机械臂图片.png')
    options = parser.parse_args(args)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('arm6_encoder_calibration_panel')
    panel = None
    try:
        panel = CalibrationPanel(node, options.output, options.reference_image)
        panel.root.mainloop()
    except KeyboardInterrupt:
        if panel is not None:
            panel.close()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
