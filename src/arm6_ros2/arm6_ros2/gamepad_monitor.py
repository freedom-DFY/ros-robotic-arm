"""Read-only F710 input mapper. This program has no robot command publishers."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import messagebox, ttk
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy

STEPS = [
    ('left_stick_up', '左摇杆向上推到底，然后松开回中', 'axis'),
    ('left_stick_right', '左摇杆向右推到底，然后松开回中', 'axis'),
    ('right_stick_up', '右摇杆向上推到底，然后松开回中', 'axis'),
    ('right_stick_right', '右摇杆向右推到底，然后松开回中', 'axis'),
    ('dpad_up', '十字键向上按，然后松开', 'any'),
    ('dpad_down', '十字键向下按，然后松开', 'any'),
    ('lb', '按下 LB，然后松开', 'button'),
    ('a', '按下 A，然后松开', 'button'),
    ('b', '按下 B，然后松开', 'button'),
]


class InputMapper:
    def __init__(self, node, output):
        self.node, self.output = node, output
        self.root = tk.Tk()
        self.root.title('F710 手柄编号采集（只读，不控制机械臂）')
        self.root.geometry('900x640')
        self.root.protocol('WM_DELETE_WINDOW', self.root.destroy)
        self.axes, self.buttons = [], []
        self.last_input = 0.
        self.baseline = None
        self.records = {}
        self.index = 0
        self.active = False
        self.confirmed = False
        self.gesture = None
        self.neutral_since = None
        self.prompt = tk.StringVar(value='松开所有摇杆和按键，再点击“开始采集”。')
        self.raw_axes = tk.StringVar(value='等待 /joy…')
        self.raw_buttons = tk.StringVar(value='等待 /joy…')
        self.connection = tk.StringVar(value='正在等待手柄反馈')
        ttk.Label(self.root, text='Logitech F710 · XInput · 采集完成后保持 MODE 状态不变', font=('', 15)).pack(pady=12)
        ttk.Label(self.root, text='本窗口只读取 /joy，没有任何机械臂运动接口。').pack()
        ttk.Label(self.root, textvariable=self.connection).pack(pady=8)
        ttk.Label(self.root, textvariable=self.prompt, font=('', 15), wraplength=850).pack(padx=15, pady=14)
        row = ttk.Frame(self.root)
        row.pack(pady=4)
        ttk.Button(row, text='开始 / 从头重新采集', command=self.start).pack(side='left', padx=12)
        ttk.Button(row, text='重录上一个动作', command=self.back).pack(side='left', padx=12)
        self.confirm_button = ttk.Button(row, text='确认操作名称无误', command=self.confirm, state='disabled')
        self.confirm_button.pack(side='left', padx=12)
        ttk.Label(self.root, textvariable=self.raw_axes, wraplength=860, justify='left').pack(fill='x', padx=20, pady=12)
        ttk.Label(self.root, textvariable=self.raw_buttons, wraplength=860, justify='left').pack(fill='x', padx=20, pady=8)
        self.log = tk.Text(self.root, height=14, state='disabled')
        self.log.pack(fill='both', expand=True, padx=18, pady=12)
        self.sub = node.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.root.after(40, self.poll)

    def append(self, text):
        self.log.configure(state='normal')
        self.log.insert('end', text+'\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def start(self):
        if time.monotonic()-self.last_input > 0.5 or not self.axes:
            messagebox.showerror('未连接', '还没有收到实时 /joy，请确认手柄及 joy_node。')
            return
        if any(self.buttons):
            messagebox.showerror('请松开按键', '请先松开所有手柄按键。')
            return
        self.baseline = self.axes.copy()
        self.records = {}
        self.confirmed = False
        self.confirm_button.configure(state='disabled')
        self.index = 0
        self.active = True
        self.gesture = None
        self.neutral_since = None
        self.append(f'开始采集；静止 axes={self.baseline}')
        self.show_step()
        self.save()

    def back(self):
        if self.baseline is None:
            return
        self.index = max(0, self.index-1)
        self.confirmed = False
        self.confirm_button.configure(state='disabled')
        for name, _, _ in STEPS[self.index:]:
            self.records.pop(name, None)
        self.active = True
        self.gesture = None
        self.neutral_since = None
        self.show_step()
        self.save()

    def show_step(self):
        if self.index < len(STEPS):
            self.prompt.set(f'{self.index+1}/{len(STEPS)}：{STEPS[self.index][1]}；其他控件保持松开。')
        else:
            self.prompt.set(f'9 个动作已记录。编号保存至：{self.output}')

    def save(self):
        record = dict(device='Logitech F710 Gamepad (XInput)', driver='joy_node', topic='/joy',
                      required_mode='XInput; keep the exact MODE state used during capture',
                      updated_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds'),
                      complete=self.index == len(STEPS), physical_controls_confirmed_by_user=self.confirmed,
                      axes_count=len(self.axes), buttons_count=len(self.buttons),
                      baseline_axes=self.baseline, mapping=self.records)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')

    def confirm(self):
        if self.index != len(STEPS):
            return
        sticks = [self.records[name]['index'] for name, _, _ in STEPS[:4]]
        buttons = [self.records[name]['index'] for name in ('lb', 'a', 'b')]
        if len(set(sticks)) != 4 or len(set(buttons)) != 3:
            messagebox.showerror('映射冲突', '摇杆轴或 LB/A/B 编号重复，请重录相应动作。')
            return
        self.confirmed = True
        self.save()
        self.prompt.set('映射已由用户确认；保持当前 X/D 与 MODE 状态，再启动手柄控制。')

    def on_joy(self, message):
        now = time.monotonic()
        self.axes, self.buttons = list(message.axes), list(message.buttons)
        self.last_input = now
        if not self.active or self.baseline is None:
            return
        if len(self.axes) != len(self.baseline):
            self.active = False
            self.prompt.set('设备轴数量变化，请松开控件后重新采集。')
            return
        deltas = [value-base for value, base in zip(self.axes, self.baseline)]
        pressed = [i for i, value in enumerate(self.buttons) if value]
        neutral = max((abs(value) for value in deltas), default=0.) < 0.15 and not pressed
        kind = STEPS[self.index][2]
        if not neutral:
            self.neutral_since = None
            if kind in ('button', 'any') and len(pressed) == 1:
                self.gesture = dict(kind='button', index=pressed[0], pressed_value=1)
            elif kind in ('axis', 'any') and not pressed:
                axis = max(range(len(deltas)), key=lambda i: abs(deltas[i]))
                if abs(deltas[axis]) >= 0.6:
                    if self.gesture is None or abs(deltas[axis]) > abs(self.gesture.get('delta', 0.)):
                        self.gesture = dict(kind='axis', index=axis, baseline=self.baseline[axis],
                                            active_value=self.axes[axis], delta=deltas[axis],
                                            sign=1 if deltas[axis] > 0 else -1)
        elif self.gesture is not None:
            if self.neutral_since is None:
                self.neutral_since = now
            if now-self.neutral_since >= 0.3:
                name, label, _ = STEPS[self.index]
                self.records[name] = self.gesture
                self.append(f'{name}：{self.gesture}')
                self.index += 1
                self.gesture = None
                self.neutral_since = None
                self.active = self.index < len(STEPS)
                self.show_step()
                self.save()
                if not self.active:
                    self.confirm_button.configure(state='normal')
                    print(f'F710 mapping captured: {self.output}', flush=True)

    def poll(self):
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0.)
        online = time.monotonic()-self.last_input < 0.5
        self.connection.set(f'已连接：{len(self.axes)} axes / {len(self.buttons)} buttons' if online else '尚未收到实时 /joy')
        self.raw_axes.set('axes：'+'  '.join(f'[{i}] {value:+.3f}' for i, value in enumerate(self.axes)))
        self.raw_buttons.set('buttons 按下编号：'+str([i for i, value in enumerate(self.buttons) if value]))
        if not online and self.active:
            self.active = False
            self.prompt.set('手柄反馈中断，采集暂停；恢复后请重新开始。')
        self.root.after(40, self.poll)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path.cwd()/'f710_mapping.json')
    options = parser.parse_args(args)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('arm6_gamepad_input_mapper')
    try:
        panel = InputMapper(node, options.output)
        panel.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
