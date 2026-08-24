#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
五连杆并联结构 - 腿上下摆动测试节点
围绕固定中心正弦摆动（前后电机同相，腿上下），持续来回

电机映射（并联五连杆）:
  [0]=右前(RightFront, ID6), [1]=右后(RightBack, ID8)
  [2]=左前(LeftFront,  ID6), [3]=左后(LeftBack,  ID8)
  CAN1=右腿, CAN2=左腿

用法:
  source /opt/ros/humble/setup.bash
  source ~/wheel_leg_ws/install/setup.bash
  python3 ~/wheel_leg_ws/src/rl_policy/scripts/leg_swing_node.py

参数:
  freq:  摆动频率 Hz (默认 0.3)
  amp:   摆动幅度 rad (默认 0.15)
  kp:    位置刚度 (默认 20.0)
  kd:    阻尼 (默认 1.0)
  center: 摆动中心 rad, 4个值 (默认 [0.6, 0.0, 0.6, 0.0])
          (若收到joint_states则自动使用当前位置, 否则用此值)
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import JointState


class LegSwingNode(Node):
    def __init__(self):
        super().__init__('leg_swing_node')

        # 参数
        self.freq = self.declare_parameter('freq', 0.3).value       # Hz
        self.amp = self.declare_parameter('amp', 0.15).value        # rad
        self.kp = self.declare_parameter('kp', 20.0).value
        self.kd = self.declare_parameter('kd', 1.0).value
        center_p = self.declare_parameter('center', [0.6, 0.0, 0.6, 0.0]).value
        self.auto_enable = self.declare_parameter('auto_enable', True).value

        self.center = [float(x) for x in center_p]
        self.pos_valid = False

        self.pos_pub = self.create_publisher(Float32MultiArray, '/mcu/pos_cmd', 10)
        self.enable_pub = self.create_publisher(Bool, '/mcu/enable', 10)

        # 尝试读一次当前位置作为中心（3秒内收到即用）
        self.create_subscription(JointState, '/mcu/joint_states', self.js_cb, 10)
        self.center_timer = self.create_timer(1.0, self.center_timeout)
        self.center_wait = 0

        dt = 1.0 / 100.0
        self.timer = self.create_timer(dt, self.tick)
        self.t = 0.0

        if self.auto_enable:
            self.enable_motors(True)

        self.get_logger().info(
            f"摆动节点: freq={self.freq}Hz amp={self.amp}rad kp={self.kp} center={[round(c,3) for c in self.center]}")

    def enable_motors(self, en):
        msg = Bool()
        msg.data = en
        self.enable_pub.publish(msg)
        self.get_logger().info(f"电机 {'使能' if en else '失能'}")

    def js_cb(self, msg):
        if not self.pos_valid and len(msg.position) >= 4:
            self.center = list(msg.position[:4])
            self.pos_valid = True
            self.get_logger().info(f"已记录当前位置为摆动中心: {[round(c,3) for c in self.center]}")

    def center_timeout(self):
        self.center_wait += 1
        if self.center_wait >= 3 and not self.pos_valid:
            self.pos_valid = True
            self.get_logger().warn(f"未收到位置,使用参数center: {[round(c,3) for c in self.center]}")

    def tick(self):
        if not self.pos_valid:
            return
        self.t += 0.01
        phase = 2.0 * math.pi * self.freq * self.t
        delta = self.amp * math.sin(phase)

        # 前后电机同相,围绕中心摆动
        cmd = [self.center[i] + delta for i in range(4)]

        msg = Float32MultiArray()
        msg.data = cmd + [self.kp, self.kd]
        self.pos_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LegSwingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.enable_motors(False)   # 退出时失能，安全
        rclpy.shutdown()


if __name__ == '__main__':
    main()
