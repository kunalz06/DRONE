#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


HELP_TEXT = """
Manual Control Link (Keyboard)
==============================
Mode:
  m : enable MANUAL mode (takes control immediately)
  n : disable MANUAL mode (return to autonomous mission)

Velocity control (setpoint increments):
  w / s : forward / backward (x)
  a / d : left / right (y)
  i / k : up / down (z)
  j / l : yaw left / yaw right

Other:
  space : stop all velocities (x,y,z,yaw = 0)
  b     : return to base (auto hover, align to pad arrow, slow land)
  r     : print this help
  q     : quit (also returns to autonomous mode)
"""


class ManualControlLink(Node):
    def __init__(self):
        super().__init__("manual_control_link")
        self.pub_mode = self.create_publisher(Bool, "/manual_mode", 10)
        self.pub_cmd = self.create_publisher(Twist, "/manual_cmd_vel", 10)
        self.pub_rtb = self.create_publisher(Bool, "/manual_rtb", 10)

        self.manual_mode_enabled = False
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        self.xy_step = 0.20
        self.z_step = 0.15
        self.yaw_step = 0.25
        self.max_xy = 1.4
        self.max_z = 0.7
        self.max_yaw = 1.2

    @staticmethod
    def _clamp(val: float, low: float, high: float) -> float:
        return max(low, min(high, val))

    def _publish_mode(self):
        msg = Bool()
        msg.data = self.manual_mode_enabled
        self.pub_mode.publish(msg)

    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
        msg.angular.z = self.yaw_rate
        self.pub_cmd.publish(msg)

    def _publish_rtb(self):
        msg = Bool()
        msg.data = True
        self.pub_rtb.publish(msg)

    def publish(self):
        self._publish_mode()
        self._publish_cmd()

    def try_publish(self) -> bool:
        if not rclpy.ok():
            return False
        try:
            self.publish()
            return True
        except Exception:
            return False

    def _apply_limits(self):
        self.vx = self._clamp(self.vx, -self.max_xy, self.max_xy)
        self.vy = self._clamp(self.vy, -self.max_xy, self.max_xy)
        self.vz = self._clamp(self.vz, -self.max_z, self.max_z)
        self.yaw_rate = self._clamp(self.yaw_rate, -self.max_yaw, self.max_yaw)

    def stop(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

    def process_key(self, key: str) -> bool:
        if key == "m":
            self.manual_mode_enabled = True
            print("[MODE] MANUAL enabled")
        elif key == "n":
            self.manual_mode_enabled = False
            self.stop()
            print("[MODE] AUTONOMOUS enabled")
        elif key == "w":
            self.vx += self.xy_step
        elif key == "s":
            self.vx -= self.xy_step
        elif key == "a":
            self.vy += self.xy_step
        elif key == "d":
            self.vy -= self.xy_step
        elif key == "i":
            self.vz += self.z_step
        elif key == "k":
            self.vz -= self.z_step
        elif key == "j":
            self.yaw_rate += self.yaw_step
        elif key == "l":
            self.yaw_rate -= self.yaw_step
        elif key == " ":
            self.stop()
            print("[CMD] zero velocity")
        elif key == "b":
            self.manual_mode_enabled = False
            self.stop()
            self._publish_rtb()
            print("[CMD] RTB requested")
        elif key == "r":
            print(HELP_TEXT)
        elif key == "q":
            self.manual_mode_enabled = False
            self.stop()
            self.try_publish()
            print("[EXIT] manual link stopped")
            return False
        else:
            return True

        self._apply_limits()
        print(
            f"[CMD] mode={'MANUAL' if self.manual_mode_enabled else 'AUTO'} "
            f"vx={self.vx:.2f} vy={self.vy:.2f} vz={self.vz:.2f} yaw={self.yaw_rate:.2f}"
        )
        return True


def _read_key(timeout_s: float = 0.05) -> str:
    readable, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if readable:
        return sys.stdin.read(1)
    return ""


def main():
    if not sys.stdin.isatty():
        print("manual_control_link.py requires a TTY terminal.")
        sys.exit(1)

    old_settings = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())

    rclpy.init()
    node = ManualControlLink()
    print(HELP_TEXT)

    try:
        keep_running = True
        while rclpy.ok() and keep_running:
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key(0.05)
            if key:
                keep_running = node.process_key(key)
            if not node.try_publish():
                keep_running = False
    except KeyboardInterrupt:
        pass
    finally:
        node.manual_mode_enabled = False
        node.stop()
        if rclpy.ok():
            for _ in range(5):
                if not node.try_publish():
                    break
                try:
                    rclpy.spin_once(node, timeout_sec=0.0)
                except Exception:
                    break
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
