"""Desktop operator panel using a taught user origin, independent of CAD pose."""

import argparse
import json
from pathlib import Path
import subprocess
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import SetBool, Trigger


class DesktopControl:
    def __init__(self, node, origin_file):
        self.node, self.origin_file = node, origin_file
        self.root = tk.Tk()
        self.root.title('机械臂电脑控制 · 用户原点')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.state = None
        self.ticks = None
        self.origin = None
        self.ids = [1, 2, 3, 5, 4, 11, 12]
        self.last_ticks = self.last_state = 0.
        self.parameters = {}
        self.requests = []
        self.busy_until = 0.
        self.param_future = None
        self.last_param_request = 0.
        self.param_client = AsyncParameterClient(node, '/arm6')
        self.jogs = [node.create_client(SetBool, f'/arm6/manual/joint_{i+1}/jog') for i in range(6)]
        self.calibration_jogs = [node.create_client(SetBool, f'/arm6/calibration/joint_{i+1}/jog')
                                 for i in range(6)]
        self.services = {
            'enable': node.create_client(SetBool, '/arm6/enable'),
            'stop': node.create_client(Trigger, '/arm6/stop'),
            'home': node.create_client(Trigger, '/arm6/home'),
            'origin': node.create_client(Trigger, '/arm6/set_user_origin'),
            'open': node.create_client(Trigger, '/arm6/manual/gripper/open'),
            'close': node.create_client(Trigger, '/arm6/manual/gripper/close'),
        }
        self.subscriptions = [
            node.create_subscription(Int32MultiArray, '/arm6/encoder_ticks', self.on_ticks, 10),
            node.create_subscription(Int32MultiArray, '/arm6/user_origin_ticks', self.on_origin, 10),
            node.create_subscription(String, '/arm6/motion_status', self.on_state, 10),
        ]
        self.connection = tk.StringVar(value='正在连接已有机械臂控制节点…')
        ttk.Label(self.root, textvariable=self.connection, font=('', 14)).pack(anchor='w', padx=16, pady=12)
        ttk.Label(self.root, text='当前示教姿态作为用户原点；相对角度按编码器刻度增加方向计正，不要求 CAD 姿态。').pack(
            anchor='w', padx=16)
        top = ttk.Frame(self.root)
        top.pack(fill='x', padx=16, pady=10)
        self.enable_button = ttk.Button(top, text='使能保持', command=lambda: self.service('enable', True))
        self.enable_button.pack(side='left', padx=3)
        self.disable_button = ttk.Button(top, text='断扭矩', command=self.disable)
        self.disable_button.pack(side='left', padx=3)
        self.home_button = ttk.Button(top, text='回用户原点', command=lambda: self.service('home'))
        self.home_button.pack(side='left', padx=3)
        self.origin_button = ttk.Button(top, text='设当前为本次原点', command=lambda: self.service('origin'))
        self.origin_button.pack(side='left', padx=3)
        self.stop_button = tk.Button(top, text='停止并保持  [Esc]', bg='#b42318', fg='white',
                                     command=lambda: self.service('stop'), padx=15)
        self.stop_button.pack(side='right', padx=3)

        grid = ttk.LabelFrame(self.root, text='六轴单独控制：每步 ±20 tick，其他关节保持目标不变')
        grid.pack(fill='x', padx=16, pady=6)
        headers = ['关节 / 电机', '减少刻度', '增加刻度', '实测 tick', '用户原点 tick', '相对角度']
        for column, text in enumerate(headers):
            ttk.Label(grid, text=text).grid(row=0, column=column, padx=8, pady=4)
        for column in (3, 4, 5):
            grid.columnconfigure(column, weight=1)
        self.labels, self.jog_buttons = [], []
        for index in range(6):
            values = [tk.StringVar(value='—') for _ in range(4)]
            self.labels.append(values)
            for column, value in zip((0, 3, 4, 5), values):
                ttk.Label(grid, textvariable=value).grid(row=index+1, column=column, padx=8, pady=4)
            for column, positive in ((1, False), (2, True)):
                button = ttk.Button(grid, text='−20' if not positive else '+20', width=5,
                                    command=lambda index=index, positive=positive: self.jog(index, positive))
                button.grid(row=index+1, column=column, padx=6, pady=2)
                self.jog_buttons.append(button)
        grip = ttk.LabelFrame(self.root, text='夹爪 ID12：只改变夹爪，六轴保持不动')
        grip.pack(fill='x', padx=16, pady=8)
        self.gripper = tk.StringVar(value='等待反馈')
        ttk.Label(grip, textvariable=self.gripper).pack(side='left', padx=10, pady=10)
        self.open_button = ttk.Button(grip, text='张开至已测端点', command=lambda: self.service('open'))
        self.open_button.pack(side='left', padx=8)
        self.close_button = ttk.Button(grip, text='闭合至已测端点', command=lambda: self.service('close'))
        self.close_button.pack(side='left', padx=8)
        keyboard = ttk.Frame(self.root)
        keyboard.pack(fill='x', padx=16, pady=6)
        self.keyboard = tk.BooleanVar(value=False)
        ttk.Checkbutton(keyboard, text='启用键盘点动', variable=self.keyboard).pack(side='left')
        self.selected_joint = ttk.Combobox(keyboard, values=[f'J{i+1} / ID{mid}' for i, mid in enumerate(self.ids[:6])],
                                         state='readonly', width=18)
        self.selected_joint.current(0)
        self.selected_joint.pack(side='left', padx=10)
        ttk.Label(keyboard, text='数字 1–6 选轴；← 减刻度，→ 加刻度；Esc 停止').pack(side='left')
        ttk.Button(keyboard, text='打开保留的标定工具', command=self.open_calibration).pack(side='right')
        self.origin_text = tk.StringVar(value='用户原点：等待读取')
        ttk.Label(self.root, textvariable=self.origin_text).pack(anchor='w', padx=16, pady=4)
        self.log = tk.Text(self.root, height=5, state='disabled', wrap='word')
        self.log.pack(fill='both', expand=True, padx=16, pady=8)
        self.root.bind('<KeyPress>', self.key)
        self.note('面板已打开。启动界面不会发送运动、使能或断扭矩命令。')
        self.root.after(50, self.poll)

    def on_ticks(self, message):
        if len(message.data) == 7:
            self.ticks = list(message.data)
            self.last_ticks = time.monotonic()

    def on_origin(self, message):
        if len(message.data) == 6:
            self.origin = list(message.data)

    def on_state(self, message):
        self.state = message.data
        self.last_state = time.monotonic()

    def note(self, text):
        self.log.configure(state='normal')
        self.log.insert('end', f'{time.strftime("%H:%M:%S")}  {text}\n')
        if int(self.log.index('end-1c').split('.')[0]) > 200:
            self.log.delete('1.0', '20.0')
        self.log.see('end')
        self.log.configure(state='disabled')

    def submit(self, client, request, label, origin=False):
        if not client.service_is_ready():
            self.note(f'{label}：控制服务未就绪。')
            return
        if label != '停止并保持' and (self.requests or time.monotonic() < self.busy_until):
            return
        self.requests.append((client.call_async(request), label, origin, time.monotonic()))
        self.busy_until = time.monotonic()+0.5

    def service(self, name, value=None):
        labels = dict(enable='使能保持' if value else '断扭矩', stop='停止并保持', home='回用户原点',
                      origin='设置本次用户原点', open='夹爪张开', close='夹爪闭合')
        request = SetBool.Request(data=value) if name == 'enable' else Trigger.Request()
        self.submit(self.services[name], request, labels[name], origin=name == 'origin')

    def jog(self, index, positive):
        if not self.ready():
            return
        client = self.jogs[index]
        if not client.service_is_ready() and not self.parameters.get('calibration_confirmed', False):
            client = self.calibration_jogs[index]
        self.submit(client, SetBool.Request(data=positive),
                    f'J{index+1} / ID{self.ids[index]} {"+" if positive else "−"}20 tick')

    def disable(self):
        if messagebox.askyesno('断扭矩', '断扭矩后机械臂会失去支撑。请先托稳机械臂，再确认断扭矩。'):
            self.service('enable', False)

    def ready(self):
        return (time.monotonic()-min(self.last_ticks, self.last_state) < 0.5
                and self.state in ('holding', 'stopped') and not self.requests
                and time.monotonic() >= self.busy_until)

    def key(self, event):
        if event.keysym == 'Escape':
            self.service('stop')
            return 'break'
        editing = isinstance(event.widget, (tk.Entry, ttk.Entry, tk.Text)) and not isinstance(event.widget, ttk.Combobox)
        if not self.keyboard.get() or editing:
            return
        if event.char in ('1', '2', '3', '4', '5', '6'):
            self.selected_joint.current(int(event.char)-1)
            return 'break'
        if event.keysym in ('Left', 'Right'):
            self.jog(self.selected_joint.current(), event.keysym == 'Right')
            return 'break'

    def poll(self):
        # Drain ready callbacks so GUI refresh does not accumulate stale feedback.
        for _ in range(12):
            rclpy.spin_once(self.node, timeout_sec=0.)
        now = time.monotonic()
        if self.param_future is None and now-self.last_param_request > 3. and self.param_client.services_are_ready():
            names = ['motor_ids', 'home_ticks', 'home_on_enable', 'calibration_confirmed', 'geometry_confirmed']
            self.param_future = self.param_client.get_parameters(names)
            self.last_param_request = now
        if self.param_future is not None and self.param_future.done():
            try:
                response = self.param_future.result()
                names = ['motor_ids', 'home_ticks', 'home_on_enable', 'calibration_confirmed', 'geometry_confirmed']
                self.parameters = dict(zip(names, map(parameter_value_to_python, response.values)))
                self.ids = self.parameters['motor_ids']
                if self.origin is None:
                    self.origin = self.parameters['home_ticks']
            except Exception as exc:
                self.note(f'读取参数失败：{exc}')
            self.param_future = None
        remaining = []
        for future, label, save_origin, started in self.requests:
            if future.done():
                try:
                    result = future.result()
                    self.note(f'{label}：{"已接收" if result.success else "拒绝"}；{result.message}')
                    if result.success and save_origin:
                        self.origin = json.loads(result.message)
                        self.origin_file.parent.mkdir(parents=True, exist_ok=True)
                        self.origin_file.write_text(json.dumps(dict(origin_ticks=self.origin,
                            motor_ids=self.ids[:6],reference='User-taught pose, independent of CAD',
                            scope='Current controller session'), ensure_ascii=False, indent=2)+'\n')
                        self.note(f'本次原点记录已保存：{self.origin_file}')
                except Exception as exc:
                    self.note(f'{label}失败：{exc}')
            elif now-started > 8.:
                self.note(f'{label}响应超时；请查看实时状态，勿连续重复点击。')
            else:
                remaining.append((future, label, save_origin, started))
        self.requests = remaining
        online = now-min(self.last_ticks, self.last_state) < 0.5
        states = dict(holding='已使能保持', stopped='已停止并保持', calibrating='单轴手动运动中',
                      moving='轨迹运动中', fault='故障，保持/扭矩状态需核对', disabled='已断扭矩', read_only='只读连接')
        self.connection.set(('已连接  |  '+states.get(self.state, str(self.state))) if online else '反馈断开，点动禁用')
        available = self.ready()
        for button in self.jog_buttons:
            button.configure(state='normal' if available else 'disabled')
        for button, name in ((self.home_button, 'home'), (self.origin_button, 'origin'),
                             (self.open_button, 'open'), (self.close_button, 'close')):
            button.configure(state='normal' if available and self.services[name].service_is_ready() else 'disabled')
        self.enable_button.configure(state='normal' if online and self.state in ('read_only', 'disabled') else 'disabled')
        self.disable_button.configure(state='normal' if online and not self.requests else 'disabled')
        self.stop_button.configure(state='normal' if online else 'disabled')
        if self.ticks is not None:
            for index, values in enumerate(self.labels):
                values[0].set(f'J{index+1} / ID{self.ids[index]}')
                values[1].set(str(self.ticks[index]))
                values[2].set(str(self.origin[index]) if self.origin is not None else '—')
                values[3].set(f'{(self.ticks[index]-self.origin[index])*360/4096:+.2f}°'
                              if self.origin is not None else '—')
            tick = self.ticks[6]
            self.gripper.set(f'连续 tick {tick}；单圈 {tick%4096}；3180 闭合 / 4376（280）张开')
        if self.origin is not None:
            self.origin_text.set(f'当前用户原点：{self.origin}；重新设定只改变操作参考，不改变电机限位或 CAD 几何参数。')
        self.root.after(50, self.poll)

    def open_calibration(self):
        subprocess.Popen(['ros2', 'run', 'arm6_ros2', 'encoder_calibration',
                          '--reference-image', str(Path.cwd()/'docs/hardware/最终机械臂图片.png'),
                          '--output', str(Path.cwd()/'docs/hardware/encoder_calibration_record.json')])

    def close(self):
        # Closing a client does not disable the separate hardware controller.
        self.root.destroy()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--origin-file', type=Path, default=Path.cwd()/'user_origin.json')
    options = parser.parse_args(args)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('arm6_desktop_control')
    panel = None
    try:
        panel = DesktopControl(node, options.origin_file)
        panel.root.mainloop()
    except KeyboardInterrupt:
        if panel is not None:
            panel.close()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
