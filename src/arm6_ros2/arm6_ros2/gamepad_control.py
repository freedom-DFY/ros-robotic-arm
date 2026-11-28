"""Measured-profile gamepad control: deadman, single-axis bounded jogs, no auto-enable."""

import argparse
import json
import math
from pathlib import Path
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import SetBool, Trigger


def load_profile(path):
    profile = json.loads(path.read_text())
    if not profile.get('complete') or not profile.get('physical_controls_confirmed_by_user'):
        raise ValueError('手柄映射尚未完整采集并由用户确认')
    mapping = profile['mapping']
    for name in ('left_stick_up', 'left_stick_right', 'right_stick_up', 'right_stick_right',
                 'dpad_up', 'dpad_down', 'lb', 'a', 'b'):
        entry = mapping[name]
        if entry['kind'] not in ('axis', 'button'):
            raise ValueError(f'Invalid input kind for {name}')
        size = profile['axes_count'] if entry['kind'] == 'axis' else profile['buttons_count']
        if type(entry['index']) is not int or not 0 <= entry['index'] < size:
            raise ValueError(f'Invalid measured index for {name}')
        if entry['kind'] == 'axis' and entry['sign'] not in (-1, 1):
            raise ValueError(f'Invalid measured direction for {name}')
    if any(mapping[name]['kind'] != 'button' for name in ('lb', 'a', 'b')):
        raise ValueError('LB/A/B must be measured button inputs')
    if len({mapping[name]['index'] for name in ('lb', 'a', 'b')}) != 3:
        raise ValueError('LB/A/B indexes must be distinct')
    sticks = [mapping[name]['index'] for name in ('left_stick_up', 'left_stick_right',
                                                'right_stick_up', 'right_stick_right')]
    if any(mapping[name]['kind'] != 'axis' for name in (
            'left_stick_up', 'left_stick_right', 'right_stick_up', 'right_stick_right')):
        raise ValueError('Stick movements must be measured axes')
    if len(set(sticks)) != 4:
        raise ValueError('The four measured stick axes must be distinct')
    up, down = mapping['dpad_up'], mapping['dpad_down']
    if up['kind'] != down['kind']:
        raise ValueError('D-pad up/down input kinds differ')
    if up['kind'] == 'axis' and (up['index'] != down['index'] or up['sign'] == down['sign']
                                 or up['index'] in sticks):
        raise ValueError('D-pad axis must have opposite signs and differ from stick axes')
    if up['kind'] == 'button' and up['index'] == down['index']:
        raise ValueError('D-pad button indexes must differ')
    return profile


class GamepadControl(Node):
    def __init__(self, profile):
        super().__init__('arm6_gamepad_control')
        self.lock = threading.RLock()
        self.profile, self.mapping = profile, profile['mapping']
        self.axes, self.buttons = [], []
        self.joy_time = self.state_time = self.tick_time = 0.
        self.state = None
        self.ticks = None
        self.ids = [1, 2, 3, 5, 4, 11, 12]
        self.selected = 0
        self.enabled = False
        self.notice = '待启用：松开所有控件，再点击“启用手柄输入”。'
        self.previous = dict(a=False, b=False, dpad_up=False, dpad_down=False)
        self.next_open = True
        self.grip_intent = None
        self.grip_blocked = False
        self.axis_blocked = False
        self.pending = None
        self.owned = None
        self.seen_busy = False
        self.sent_at = 0.
        self.stop_future = None
        self.must_stop = False
        self.neutral_gate = True
        self.jogs = [self.create_client(SetBool, f'/arm6/manual/joint_{i+1}/jog') for i in range(6)]
        self.gripper = self.create_client(SetBool, '/arm6/manual/gripper/jog')
        self.stop_client = self.create_client(Trigger, '/arm6/stop')
        self.status_pub = self.create_publisher(String, '/arm6/gamepad_status', 1)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.on_joy, 1)
        self.state_sub = self.create_subscription(String, '/arm6/motion_status', self.on_state, 1)
        self.tick_sub = self.create_subscription(Int32MultiArray, '/arm6/encoder_ticks', self.on_ticks, 1)
        self.timer = self.create_timer(0.05, self.update)

    def level(self, name):
        entry = self.mapping[name]
        if entry['kind'] == 'button':
            return float(self.buttons[entry['index']]) if self.buttons else 0.
        return (self.axes[entry['index']]-entry['baseline'])*entry['sign'] if self.axes else 0.

    def neutral(self):
        if not self.axes or any(self.buttons):
            return False
        return all(abs(self.level(name)) < 0.2 for name in (
            'left_stick_up', 'left_stick_right', 'right_stick_up', 'right_stick_right',
            'dpad_up', 'dpad_down'))

    def on_joy(self, message):
        with self.lock:
            if (len(message.axes) != self.profile['axes_count']
                    or len(message.buttons) != self.profile['buttons_count']
                    or any(not math.isfinite(v) for v in message.axes)):
                self.disable('手柄数据格式变化，请重新采集映射。')
                return
            self.axes, self.buttons = list(message.axes), list(message.buttons)
            self.joy_time = time.monotonic()
            current = {name: self.level(name) > 0.5 for name in self.previous}
            if (current['b'] and not self.previous['b']
                    and (self.enabled or self.owned is not None or self.pending is not None)):
                self.disable('B 停止：手柄输入已关闭。', force=True)
            if self.enabled:
                if not self.level('lb') and self.owned is None and self.pending is None:
                    if current['dpad_up'] and not self.previous['dpad_up']:
                        self.selected = (self.selected+1) % 6
                    elif current['dpad_down'] and not self.previous['dpad_down']:
                        self.selected = (self.selected-1) % 6
                if (current['a'] and not self.previous['a'] and self.level('lb')
                        and not self.neutral_gate and self.state in ('holding', 'stopped')
                        and abs(self.level('left_stick_up')) < 0.2
                        and self.owned is None and self.pending is None):
                    self.grip_intent = self.next_open
                    self.next_open = not self.next_open
                    self.grip_blocked = False
                if not current['a']:
                    self.grip_intent = None
                    self.grip_blocked = False
            if self.neutral():
                self.neutral_gate = False
                self.axis_blocked = False
            self.previous = current

    def on_state(self, message):
        with self.lock:
            self.state = message.data
            self.state_time = time.monotonic()
            if self.owned is not None and self.state == 'calibrating':
                self.seen_busy = True

    def on_ticks(self, message):
        with self.lock:
            if len(message.data) == 7:
                self.ticks = list(message.data)
                self.tick_time = time.monotonic()

    def enable_input(self):
        with self.lock:
            now = time.monotonic()
            if now-self.joy_time > 0.3 or not self.neutral():
                self.notice = '先松开 LB、所有按键，并让摇杆和十字键回中。'
                return
            if now-min(self.state_time, self.tick_time) > 0.5 or self.state not in ('holding', 'stopped'):
                self.notice = '机械臂未处于已使能保持状态，请先在电脑面板连接/使能。'
                return
            if self.pending is not None or self.owned is not None or self.stop_future is not None:
                self.notice = '先等待上一次请求及停止确认。'
                return
            self.enabled, self.neutral_gate = True, False
            self.notice = '已启用手柄输入；按住 LB 才允许运动。'

    def disable(self, text='手柄输入已关闭。', force=False):
        with self.lock:
            if self.owned is not None and self.seen_busy and self.state in ('holding', 'stopped'):
                self.owned = None
            self.enabled = False
            self.notice = text
            self.neutral_gate = True
            self.must_stop = self.owned is not None or self.pending is not None or force
            self.request_stop(force=force)

    def request_stop(self, force=False):
        if self.stop_future is not None:
            return
        # Wait for an outstanding jog reply so an unrelated rejected request
        # cannot cause a stop of another operator's motion. B is an explicit stop.
        if self.pending is not None and not force:
            return
        if not (self.owned is not None or self.must_stop or force):
            return
        if self.stop_client.service_is_ready():
            self.must_stop = True
            self.stop_future = self.stop_client.call_async(Trigger.Request())

    def desired(self):
        if not self.enabled or self.neutral_gate or not self.level('lb') or self.level('b'):
            return None
        if self.level('a'):
            return ('gripper', self.grip_intent) if self.grip_intent is not None and not self.grip_blocked else None
        value = self.level('left_stick_up')
        if abs(value) < 0.3 or self.axis_blocked:
            return None
        return ('joint', self.selected, value > 0)

    def command_finished(self, future):
        with self.lock:
            command = self.pending
            self.pending = None
            try:
                result = future.result()
                if not result.success:
                    self.notice = '请求拒绝：'+result.message
                    if 'limit' in result.message or 'endpoint' in result.message:
                        if command[0] == 'gripper':
                            self.grip_blocked = True
                        else:
                            self.axis_blocked = True
                    else:
                        self.enabled = False
                    if self.must_stop:
                        self.request_stop(force=True)
                    return
                self.owned = command
                self.sent_at = time.monotonic()
                self.seen_busy = False
                self.notice = result.message
                if self.must_stop or self.desired() != command:
                    self.must_stop = True
                    self.request_stop()
            except Exception as exc:
                self.enabled = False
                self.notice = f'手柄请求失败：{exc}'
                self.must_stop = True
                self.request_stop(force=True)

    def update(self):
        with self.lock:
            now = time.monotonic()
            if self.enabled and (now-self.joy_time > 0.3 or now-min(self.state_time, self.tick_time) > 0.5):
                self.disable('输入或机械臂反馈超时；手柄已关闭，等待重新启用。')
            if self.enabled and self.state in ('fault', 'disabled', 'read_only'):
                self.disable(f'机械臂状态为 {self.state}，手柄已关闭。')
            if self.stop_future is not None and self.stop_future.done():
                try:
                    result = self.stop_future.result()
                    if result.success:
                        self.owned = None
                        self.must_stop = False
                        self.neutral_gate = True
                    else:
                        self.enabled = False
                        self.notice = '停止未确认：'+result.message
                except Exception as exc:
                    self.enabled = False
                    self.notice = f'停止请求失败：{exc}'
                self.stop_future = None
            if self.must_stop:
                self.request_stop()
            desired = self.desired()
            if self.owned is not None and not self.must_stop:
                if self.seen_busy and self.state in ('holding', 'stopped'):
                    self.owned = None
                elif desired != self.owned:
                    self.must_stop = True
                    self.request_stop()
                elif not self.seen_busy and now-self.sent_at > 5.:
                    self.disable('未确认点动状态；已请求停止。')
            if (desired is not None and self.owned is None and self.pending is None
                    and self.stop_future is None and not self.must_stop
                    and self.state in ('holding', 'stopped')):
                if desired[0] == 'joint':
                    client, positive = self.jogs[desired[1]], desired[2]
                else:
                    client, positive = self.gripper, desired[1]
                if not client.service_is_ready():
                    self.enabled = False
                    self.notice = '手柄点动服务未就绪，请启动最新控制器。'
                else:
                    self.pending = desired
                    self.sent_at = now
                    future = client.call_async(SetBool.Request(data=positive))
                    future.add_done_callback(self.command_finished)
            self.status_pub.publish(String(data=(f'{"enabled" if self.enabled else "disabled"}; '
                f'J{self.selected+1}/ID{self.ids[self.selected]}; {self.notice}')))


class ControlWindow:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('F710 机械臂关节控制 · LB 按住运行')
        self.root.geometry('820x540')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.state = tk.StringVar()
        self.joint = tk.StringVar()
        self.notice = tk.StringVar()
        self.axes = tk.StringVar()
        ttk.Label(self.root, text='实测 F710 映射：保持采集时的 MODE 状态，不要切换', font=('', 14)).pack(pady=14)
        ttk.Label(self.root, textvariable=self.state).pack(pady=8)
        ttk.Label(self.root, textvariable=self.joint, font=('', 21)).pack(pady=10)
        buttons = ttk.Frame(self.root)
        buttons.pack(pady=10)
        ttk.Button(buttons, text='启用手柄输入（不使能电机）', command=node.enable_input).pack(side='left', padx=10)
        ttk.Button(buttons, text='关闭手柄输入并停止本次运动', command=node.disable).pack(side='left', padx=10)
        ttk.Label(self.root, text=('松开 LB 时：十字键 ↑ / ↓ 选择 J1–J6。\n'
                                  '按住 LB：左摇杆 ↑ 加刻度，↓ 减刻度；其他关节保持。\n'
                                  '摇杆回中或松开 LB：停止本次点动；再操作前先松 LB 回中。\n'
                                  '摇杆回中后，LB+A：夹爪开合交替，每次按住才持续运动。\n'
                                  'B：停止，并关闭手柄输入。每次请求最多 20 tick。'),
                  justify='left', font=('', 12)).pack(pady=12)
        ttk.Label(self.root, textvariable=self.notice, wraplength=780).pack(padx=16, pady=10)
        ttk.Label(self.root, textvariable=self.axes, wraplength=780).pack(padx=16, pady=8)
        self.root.after(50, self.refresh)

    def refresh(self):
        with self.node.lock:
            n = self.node
            pad = time.monotonic()-n.joy_time < 0.3
            arm = time.monotonic()-min(n.state_time, n.tick_time) < 0.5
            self.state.set(f'手柄 {"在线" if pad else "离线"} | 机械臂 {n.state if arm else "离线"} | '
                           f'手柄控制 {"已启用" if n.enabled else "未启用"}')
            self.joint.set(f'当前选中：J{n.selected+1} / ID{n.ids[n.selected]}')
            self.notice.set(n.notice+f'；下次 A 动作：{"张开" if n.next_open else "闭合"}')
            self.axes.set('实际 axes：'+' '.join(f'{i}:{v:+.2f}' for i, v in enumerate(n.axes)))
        self.root.after(50, self.refresh)

    def close(self):
        self.node.disable('窗口关闭，已请求停止本次手柄运动。')
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            with self.node.lock:
                waiting = self.node.owned is not None or self.node.pending is not None or self.node.must_stop
            if not waiting:
                break
            time.sleep(0.02)
        self.root.destroy()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mapping', type=Path, default=Path.cwd()/'docs/hardware/f710_mapping.json')
    options = parser.parse_args(args)
    profile = load_profile(options.mapping)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = GamepadControl(profile)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    window = None
    try:
        window = ControlWindow(node)
        window.root.mainloop()
    except KeyboardInterrupt:
        if window is not None:
            window.close()
    finally:
        node.disable('手柄控制退出。')
        executor.shutdown(timeout_sec=2.)
        node.destroy_node()
        rclpy.try_shutdown()
