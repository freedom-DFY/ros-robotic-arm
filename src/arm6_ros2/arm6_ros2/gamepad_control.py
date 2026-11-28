"""Measured-profile continuous velocity control with an LB deadman."""

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
from std_srvs.srv import Trigger

from arm6_ros2.jog import GRIPPER_AXIS, INTENT_SIZE, WRIST_AXES

WRIST_SELECTION = tuple(sorted(WRIST_AXES, key=WRIST_AXES.get))  # J4, J5, J6


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
    def __init__(self, profile, **options):
        super().__init__('arm6_gamepad_control', **options)
        self.lock = threading.RLock()
        self.profile, self.mapping = profile, profile['mapping']
        self.axes, self.buttons = [], []
        self.joy_time = self.state_time = self.tick_time = 0.
        self.state, self.ticks = None, None
        self.enabled, self.neutral_gate = False, True
        self.notice = '待启用：松开控件，点击“启用手柄输入”。'
        self.pause_reason = ''
        self.previous_b = False
        self.previous_a = self.wrist_select_pending = False
        self.wrist_axis = 3  # Preserve J6 as the initial D-pad selection.
        self.session = None
        self.start_future = self.stop_future = None
        self.start_sent = self.last_stream_feedback = self.retry_after = 0.
        self.end_requested, self.end_at = False, 0.
        self.force_stop, self.stop_retry_at = False, 0.
        self.start_client = self.create_client(Trigger, '/arm6/gamepad/start')
        self.stop_client = self.create_client(Trigger, '/arm6/stop')
        self.velocity_pub = self.create_publisher(Joy, '/arm6/gamepad/velocity', 1)
        self.status_pub = self.create_publisher(String, '/arm6/gamepad_status', 1)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.on_joy, 1)
        self.state_sub = self.create_subscription(String, '/arm6/motion_status', self.on_state, 1)
        self.tick_sub = self.create_subscription(Int32MultiArray, '/arm6/encoder_ticks', self.on_ticks, 1)
        self.stream_sub = self.create_subscription(String, '/arm6/gamepad/feedback', self.on_stream_feedback, 1)
        self.timer = self.create_timer(0.02, self.update)

    def level(self, name):
        entry = self.mapping[name]
        if entry['kind'] == 'button':
            return float(self.buttons[entry['index']]) if self.buttons else 0.
        return (self.axes[entry['index']]-entry['baseline'])*entry['sign'] if self.axes else 0.

    def motion_neutral(self):
        return bool(self.axes) and not self.level('b') and all(abs(self.level(name)) < 0.2 for name in (
            'left_stick_up', 'left_stick_right', 'right_stick_up', 'right_stick_right',
            'dpad_up', 'dpad_down'))

    def neutral(self):
        return not any(self.buttons) and self.motion_neutral()

    def feedback_fresh(self):
        now = time.monotonic()
        return now-self.joy_time <= 0.3 and now-min(self.state_time, self.tick_time) <= 0.5

    def on_joy(self, message):
        with self.lock:
            if (len(message.axes) != self.profile['axes_count']
                    or len(message.buttons) != self.profile['buttons_count']
                    or any(not math.isfinite(v) for v in message.axes)):
                self.disable('手柄数据格式变化，请重新采集映射。')
                return
            self.axes, self.buttons = list(message.axes), list(message.buttons)
            self.joy_time = time.monotonic()
            b = self.level('b') > 0.5
            if b and not self.previous_b and (self.enabled or self.session or self.start_future):
                self.disable('B 停止：手柄输入已关闭。', force=True)
            self.previous_b = b
            a = self.level('a') > 0.5
            can_select = self.enabled and not self.level('lb') and self.motion_neutral()
            if not can_select:
                self.wrist_select_pending = False
            elif a and not self.previous_a:
                self.wrist_select_pending = True
            elif not a and self.wrist_select_pending:
                index = (WRIST_SELECTION.index(self.wrist_axis)+1) % len(WRIST_SELECTION)
                self.wrist_axis = WRIST_SELECTION[index]
                self.wrist_select_pending = False
            self.previous_a = a
            if (self.motion_neutral() and self.feedback_fresh() and self.state in ('holding', 'stopped')
                    and not self.end_requested and self.start_future is None and self.stop_future is None):
                self.neutral_gate = False
                if self.pause_reason:
                    self.pause_reason = ''
                    self.notice = '输入已恢复；按住 LB 推杆可连续运动。'
            if self.session:
                self.send_velocity(end=self.end_requested)

    def on_state(self, message):
        with self.lock:
            self.state, self.state_time = message.data, time.monotonic()

    def on_ticks(self, message):
        with self.lock:
            if len(message.data) == 7:
                self.ticks, self.tick_time = list(message.data), time.monotonic()

    def on_stream_feedback(self, message):
        with self.lock:
            try:
                feedback = json.loads(message.data)
                if feedback['session'] != self.session:
                    return
                self.last_stream_feedback = time.monotonic()
                self.notice = feedback['message']
                if feedback['status'] == 'ended':
                    self.session = None
                    self.end_requested = False
                    if self.enabled:
                        self.neutral_gate = True
                        self.pause_reason = self.notice
            except (ValueError, KeyError, TypeError):
                return

    def enable_input(self):
        with self.lock:
            if not self.feedback_fresh() or not self.neutral():
                self.notice = '先松开所有按键、摇杆回中，并等待实时反馈。'
                return
            if self.state not in ('holding', 'stopped'):
                self.notice = '请先在电脑面板使能保持。'
                return
            if (self.end_requested or self.start_future is not None
                    or self.stop_future is not None or self.force_stop):
                self.notice = '等待上一会话及停止确认。'
                return
            self.enabled, self.neutral_gate = True, False
            self.pause_reason = ''
            self.notice = '已启用连续控制；按住 LB 推杆运行，回中即停。'

    def begin_end(self):
        if not self.end_requested:
            self.end_at = time.monotonic()
        self.end_requested = bool(self.session or self.start_future is not None)
        if self.session:
            self.send_velocity(end=True)

    def pause_input(self, text):
        self.neutral_gate = True
        self.pause_reason = text
        self.notice = text+'；恢复后回中即可继续。'
        self.begin_end()

    def disable(self, text='手柄输入已关闭。', force=False):
        with self.lock:
            self.enabled, self.neutral_gate = False, True
            self.pause_reason = ''
            self.notice = text
            self.begin_end()
            if force:
                self.force_stop = True
                self.request_emergency_stop()

    def request_emergency_stop(self):
        if (self.force_stop and self.stop_future is None and time.monotonic() >= self.stop_retry_at
                and self.stop_client.service_is_ready()):
            self.stop_retry_at = time.monotonic()+0.5
            try:
                self.stop_future = self.stop_client.call_async(Trigger.Request())
            except Exception as exc:
                self.notice = f'停止尚未确认，正在重试：{exc}'

    @staticmethod
    def stick_speed(value):
        if abs(value) < 0.3:
            return 0.
        return math.copysign(min(1., (abs(value)-0.2)/0.8), value)

    def desired(self):
        zeros = (0.,)*INTENT_SIZE
        if (not self.enabled or self.neutral_gate or not self.feedback_fresh()
                or not self.level('lb') or self.level('b')):
            return zeros
        axes = [0.]*INTENT_SIZE
        up, right = self.level('left_stick_up'), self.level('left_stick_right')
        if self.level('a'):
            axes[2] = self.stick_speed(up)
        else:
            index, value = (0, up) if abs(up) >= abs(right) else (1, right)
            axes[index] = self.stick_speed(value)
        axes[GRIPPER_AXIS] = self.stick_speed(self.level('right_stick_up'))
        rotate_up, rotate_down = self.level('dpad_up') > 0.5, self.level('dpad_down') > 0.5
        if rotate_up and rotate_down:
            return zeros
        axes[self.wrist_axis] = 1. if rotate_up else -1. if rotate_down else 0.
        return tuple(axes) if sum(v != 0 for v in axes) <= 1 else zeros

    def send_velocity(self, *, zero=False, end=False):
        if not self.session:
            return
        values = (0.,)*INTENT_SIZE if zero or end else self.desired()
        message = Joy(axes=list(values), buttons=[int(any(values)), int(end)])
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.session
        try:
            self.velocity_pub.publish(message)
        except Exception as exc:
            # No output can be guaranteed after a transport failure. Retire
            # the token locally and let the controller's watchdog stop it.
            self.session, self.end_requested = None, False
            self.neutral_gate = True
            self.pause_reason = f'速度发送失败：{exc}；回中后重连'
            self.notice = self.pause_reason

    def update(self):
        with self.lock:
            now = time.monotonic()
            if self.enabled and not self.feedback_fresh():
                self.pause_input('输入或机械臂反馈中断，已停止')
            if self.enabled and self.state in ('fault', 'disabled', 'read_only'):
                self.pause_input('请先在电脑面板恢复机械臂使能')
            if self.stop_future is not None and self.stop_future.done():
                try:
                    self.force_stop = not self.stop_future.result().success
                except Exception:
                    self.force_stop = True
                self.stop_future = None
            self.request_emergency_stop()
            if self.start_future is not None and self.start_future.done():
                future, self.start_future = self.start_future, None
                try:
                    result = future.result()
                    if not result.success:
                        raise ValueError(result.message)
                    if len(result.message) != 32 or any(c not in '0123456789abcdef' for c in result.message):
                        raise ValueError('控制器会话标识无效')
                    self.session = result.message
                    self.last_stream_feedback = now
                    if not self.enabled or self.end_requested or self.pause_reason:
                        self.end_requested = False
                        self.begin_end()
                    else:
                        self.send_velocity(zero=True)
                except Exception as exc:
                    self.end_requested = False
                    self.pause_input(f'连续控制未连接：{exc}')
                    self.retry_after = now+0.3
            if self.start_future is not None and now-self.start_sent > 3.:
                self.pause_input('连续控制连接应答超时')
            if self.session:
                if not self.end_requested and now-self.last_stream_feedback > 0.5:
                    self.pause_input('连续控制反馈中断，已停止')
                self.send_velocity(end=self.end_requested)
                if self.end_requested and now-self.end_at > 0.5:
                    # Even if the end acknowledgement was lost, the independent
                    # controller watchdog has expired the old session token.
                    self.session, self.end_requested = None, False
            elif (self.enabled and not self.neutral_gate and not self.pause_reason
                  and self.start_future is None and not self.end_requested and not self.force_stop
                  and self.stop_future is None and self.feedback_fresh()
                  and self.state in ('holding', 'stopped') and now >= self.retry_after):
                if self.start_client.service_is_ready():
                    self.start_sent = now
                    try:
                        self.start_future = self.start_client.call_async(Trigger.Request())
                    except Exception as exc:
                        self.pause_input(f'连续控制连接发送失败：{exc}')
                        self.retry_after = now+0.3
                else:
                    self.pause_input('连续控制服务未就绪，请加载最新控制器')
            mode = 'paused' if self.enabled and self.pause_reason else 'enabled' if self.enabled else 'disabled'
            self.status_pub.publish(String(data=f'{mode}; {"Z" if self.level("a") else "XY"}; {self.notice}'))


class ControlWindow:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('F710 腕部笛卡尔控制 · LB 按住运行')
        self.root.geometry('900x700')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.state = tk.StringVar()
        self.mode = tk.StringVar()
        self.notice = tk.StringVar()
        self.axes = tk.StringVar()
        ttk.Label(self.root, text='实测 F710 映射：保持采集时的 MODE 状态，不要切换', font=('', 14)).pack(pady=14)
        ttk.Label(self.root, textvariable=self.state).pack(pady=8)
        ttk.Label(self.root, textvariable=self.mode, font=('', 21)).pack(pady=10)
        buttons = ttk.Frame(self.root)
        buttons.pack(pady=10)
        ttk.Button(buttons, text='启用手柄输入（不使能电机）', command=node.enable_input).pack(side='left', padx=10)
        ttk.Button(buttons, text='关闭手柄输入并停止本次运动', command=node.disable).pack(side='left', padx=10)
        ttk.Label(self.root, text=('以下运动均需按住 LB：\n'
                                  '左摇杆 ↑/↓：基座 X+/X−；→/←：基座 Y+/Y−。\n'
                                  '按住 A + 左摇杆 ↑/↓：Z+/Z−；左右输入忽略。\n'
                                  '右摇杆 ↑ 张开 / ↓ 闭合；左右不绑定功能。\n'
                                  '松开 LB，短按 A 选择腕部 J4/J5/J6（默认 J6）。\n'
                                  '按住 LB，十字键 ↑/↓：单独转动所选腕关节。\n'
                                  'XY 平移只驱动 J1–J3，腕部关节保持角度。\n'
                                  'XYZ 相对基座固定；Z 沿底座竖直方向，XY 不随机械臂转向改变。\n'
                                  '斜推左杆取幅度较大的轴；平移、开合、旋转每次仅一种。\n'
                                  '摇杆回中或松开 LB：停止；保持启用，换向/切换动作可直接继续。\n'
                                  '短暂掉线：暂停，恢复且回中后自动继续；可以一直按住 LB。\n'
                                  '推杆保持多久就连续运动多久；推得越多越快，回中即停，到限位停止。\n'
                                  'B：停止并关闭手柄输入。'),
                  justify='left', font=('', 12)).pack(pady=12)
        ttk.Label(self.root, textvariable=self.notice, wraplength=780).pack(padx=16, pady=10)
        ttk.Label(self.root, textvariable=self.axes, wraplength=780).pack(padx=16, pady=8)
        self.root.after(50, self.refresh)

    def refresh(self):
        with self.node.lock:
            n = self.node
            pad = time.monotonic()-n.joy_time < 0.3
            arm = time.monotonic()-min(n.state_time, n.tick_time) < 0.5
            status = '已暂停·回中恢复' if n.enabled and n.pause_reason else '已启用' if n.enabled else '未启用'
            self.state.set(f'手柄 {"在线" if pad else "离线"} | 机械臂 {n.state if arm else "离线"} | '
                           f'手柄控制 {status}')
            plane = 'Z 高度' if n.level('a') and n.level('lb') else '基座 XY'
            self.mode.set(f'平移：{plane}   ·   十字键：J{WRIST_AXES[n.wrist_axis]+1}')
            self.notice.set(n.notice)
            self.axes.set('实际 axes：'+' '.join(f'{i}:{v:+.2f}' for i, v in enumerate(n.axes)))
        self.root.after(50, self.refresh)

    def close(self):
        self.node.disable('窗口关闭，已请求停止本次手柄运动。')
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            with self.node.lock:
                waiting = (self.node.session is not None or self.node.start_future is not None
                           or self.node.stop_future is not None)
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
