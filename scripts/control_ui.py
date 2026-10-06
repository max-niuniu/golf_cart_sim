#!/usr/bin/env python3
"""Manual command source; vehicle kinematics and actuation stay in the vehicle."""
import math
import signal
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class ManualCommand:
    """Hold-to-drive state. Disabled sources remain silent after a stop burst."""
    def __init__(self, max_speed=3.0, max_steer=0.5):
        self.max_speed, self.max_steer = max_speed, max_steer
        self.speed = min(1.0, max_speed)
        self.steer = 0.0
        self.enabled = False
        self.held = {}
        self.stop_until = 0.0

    def enable(self):
        self.held.clear()
        self.steer = 0.0
        self.enabled = True

    def stop(self, now):
        if self.enabled:
            self.stop_until = now + 0.30
        self.enabled = False
        self.held.clear()
        self.steer = 0.0

    def command(self, now):
        if not self.enabled:
            return (0.0, 0.0) if now < self.stop_until else None
        actions = set(self.held.values())
        direction = int('forward' in actions) - int('reverse' in actions)
        turn = int('left' in actions) - int('right' in actions)
        angle = turn * self.max_steer if actions & {'left', 'right'} else self.steer
        return direction * min(self.max_speed, max(0.0, self.speed)), max(-self.max_steer, min(self.max_steer, angle))


class ControlPanel:
    KEYS = {'w': 'forward', 'Up': 'forward', 's': 'reverse', 'Down': 'reverse',
            'a': 'left', 'Left': 'left', 'd': 'right', 'Right': 'right'}

    def __init__(self, root, node):
        self.root, self.node = root, node
        speed = float(node.declare_parameter('max_speed', 3.0).value)
        steer = float(node.declare_parameter('max_steer', 0.5).value)
        if not all(math.isfinite(x) and x > 0 for x in (speed, steer)) or steer >= math.pi / 2:
            raise ValueError('max_speed must be positive; max_steer must be between 0 and pi/2')
        self.command_topic = node.declare_parameter('command_topic', '/golf/drive').value
        odom_topic = node.declare_parameter('odom_topic', '/golf/odom').value
        self.pub = node.create_publisher(AckermannDriveStamped, self.command_topic, 1)
        self.sub = node.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.control = ManualCommand(speed, steer)
        self.pending_releases = {}
        self.last_odom = None
        self.odom_speed = 0.0
        self.closing = False
        self.close_at = None
        self.connected = False
        self.build_ui()
        root.bind('<KeyPress>', self.key_down)
        root.bind('<KeyRelease>', self.key_up)
        root.bind('<ButtonRelease-1>', self.mouse_up)
        root.bind('<FocusOut>', lambda event: root.after_idle(self.check_focus))
        root.bind('<Unmap>', lambda event: self.disarm() if event.widget == root else None)
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.tick()

    def build_ui(self):
        r = self.root
        r.title('Golf · Manual Control')
        r.geometry('470x580')
        r.minsize(450, 550)
        frame = ttk.Frame(r, padding=22)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='GOLF  /  MANUAL CONTROL', font=('', 17, 'bold')).pack(anchor='w')
        ttk.Label(frame, text='手动控制台 · Ackermann', foreground='#596579').pack(anchor='w', pady=(4, 16))
        self.status = tk.StringVar(value='等待车辆连接…')
        ttk.Label(frame, textvariable=self.status).pack(anchor='w')
        self.toggle = ttk.Button(frame, text='启用手动控制', command=self.toggle_manual)
        self.toggle.pack(fill='x', pady=(10, 14))
        self.speed_label = tk.StringVar()
        ttk.Label(frame, textvariable=self.speed_label).pack(anchor='w')
        self.speed_slider = ttk.Scale(frame, from_=0, to=self.control.max_speed, command=self.set_speed)
        self.speed_slider.set(self.control.speed)
        self.speed_slider.pack(fill='x', pady=(4, 12))
        self.steer_label = tk.StringVar()
        ttk.Label(frame, textvariable=self.steer_label).pack(anchor='w')
        # Positive virtual steering turns left; left end of the slider is positive.
        self.steer_slider = ttk.Scale(frame, from_=self.control.max_steer, to=-self.control.max_steer,
                                      command=self.set_steer)
        self.steer_slider.set(0)
        self.steer_slider.pack(fill='x', pady=(4, 8))
        ttk.Button(frame, text='转向回中', command=lambda: self.steer_slider.set(0)).pack()
        pad = ttk.Frame(frame)
        pad.pack(fill='x', pady=16)
        for i in range(3):
            pad.columnconfigure(i, weight=1)
        self.buttons = {}
        for action, label, row, col in [('forward', '前进  W / ↑', 0, 1), ('left', '左转  A / ←', 1, 0),
                                        ('reverse', '倒车  S / ↓', 1, 1), ('right', '右转  D / →', 1, 2)]:
            button = ttk.Button(pad, text=label, takefocus=False)
            button.grid(row=row, column=col, sticky='ew', padx=3, pady=4, ipady=8)
            button.bind('<ButtonPress-1>', lambda event, a=action: self.mouse_down(a))
            self.buttons[action] = button
        tk.Button(frame, text='停止并退出手动控制  ·  Space / Esc', command=self.disarm,
                  bg='#b32632', fg='white', activebackground='#94232c', activeforeground='white',
                  relief='flat', pady=11, takefocus=False).pack(fill='x')
        self.feedback = tk.StringVar(value='实际速度：等待里程计')
        ttk.Label(frame, textvariable=self.feedback).pack(anchor='w', pady=(14, 4))
        ttk.Label(frame, text='按住前进/倒车才行驶；松开即减速停车。\n可先拖动转向滑块，再用鼠标按住行驶按钮。\n窗口失焦会退出手动控制；自动导航时请保持关闭。',
                  foreground='#596579').pack(anchor='w')

    def set_speed(self, value):
        self.control.speed = float(value)
        self.speed_label.set(f'目标速度  {float(value):.2f} m/s')

    def set_steer(self, value):
        self.control.steer = float(value)
        self.steer_label.set(f'转向预设  {math.degrees(float(value)):+.1f}°  （正值向左）')

    def toggle_manual(self):
        if self.control.enabled:
            self.disarm()
        elif self.connected and not self.closing:
            self.control.enable()
            self.steer_slider.set(0)
            self.root.focus_set()

    def disarm(self):
        self.control.stop(time.monotonic())
        self.steer_slider.set(0)

    def mouse_down(self, action):
        if self.control.enabled:
            self.root.focus_set()
            self.control.held['mouse'] = action

    def mouse_up(self, event=None):
        self.control.held.pop('mouse', None)

    def key_down(self, event):
        key = event.keysym if len(event.keysym) > 1 else event.keysym.lower()
        if key in ('space', 'Escape'):
            self.disarm()
            return 'break'
        if key in self.KEYS:
            if key in self.pending_releases:
                self.root.after_cancel(self.pending_releases.pop(key))
            if self.control.enabled:
                self.control.held[key] = self.KEYS[key]
            return 'break'

    def key_up(self, event):
        key = event.keysym if len(event.keysym) > 1 else event.keysym.lower()
        if key not in self.KEYS:
            return
        # X11 repeat emits release/press pairs; a true release survives this delay.
        if key in self.pending_releases:
            self.root.after_cancel(self.pending_releases[key])
        def release():
            self.pending_releases.pop(key, None)
            self.control.held.pop(key, None)
        self.pending_releases[key] = self.root.after(25, release)
        return 'break'

    def check_focus(self):
        focus = self.root.focus_displayof()
        if not self.closing and (focus is None or focus.winfo_toplevel() != self.root):
            self.disarm()

    def on_odom(self, msg):
        self.last_odom = time.monotonic()
        self.odom_speed = msg.twist.twist.linear.x

    def close(self):
        if not self.closing:
            self.disarm()
            self.closing = True
            self.close_at = time.monotonic() + 0.35

    def tick(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        now = time.monotonic()
        self.connected = self.pub.get_subscription_count() > 0
        if not self.connected:
            self.disarm()
        cmd = self.control.command(now)
        if cmd is not None:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.drive.speed, msg.drive.steering_angle = cmd
            self.pub.publish(msg)
        self.toggle.configure(text='退出手动控制' if self.control.enabled else '启用手动控制',
                              state='normal' if self.connected and not self.closing else 'disabled')
        self.status.set('手动控制中 · 按住行驶' if self.control.enabled else
                        ('车辆已连接 · 手动控制关闭' if self.connected else '等待车辆指令订阅者…'))
        self.feedback.set(f'实际速度：{self.odom_speed:+.2f} m/s' if self.last_odom is not None and now-self.last_odom < 1.0
                          else '实际速度：无近期里程计数据')
        if self.closing and now >= self.close_at:
            self.root.destroy()
        else:
            self.root.after(50, self.tick)


def main():
    # Tk handles shutdown, allowing the final zero-command burst before ROS exits.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('golf_manual_control')
    root = None
    try:
        root = tk.Tk()
        panel = ControlPanel(root, node)
        signal.signal(signal.SIGINT, lambda *_: panel.close())
        signal.signal(signal.SIGTERM, lambda *_: panel.close())
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
