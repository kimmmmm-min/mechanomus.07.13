#!/usr/bin/env python3

import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import MotionCommand
import serial


class ControlNode(Node):
    def __init__(self):
        super().__init__("control")

        # Serial port
        self.ser = serial.Serial("/dev/ttyACM0", 115200)
        self.get_logger().info("Connected to Arduino on /dev/ttyACM0 @115200")

        # Mode: "ZERO", "MOTION", "MANUAL"
        self.mode = "ZERO"

        # Manual control variables
        self.manual_steering = 0
        self.manual_left_speed = 0
        self.manual_right_speed = 0

        # Limits for manual control
        self.max_speed = 255
        self.min_speed = -255
        self.max_steering = 10
        self.min_steering = -10
        self.speed_step = 10
        self.steering_step = 1

        # Subscription
        self.create_subscription(
            MotionCommand,
            "motion_command",
            self.motion_command_callback,
            10,
        )

        # Keyboard thread
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            "Space: change mode (ZERO → MOTION → MANUAL → ...), "
            "Manual mode: w/s (+/- speed), a/d (steer), z (reset)."
        )

    def motion_command_callback(self, msg: MotionCommand):
        """Handle MotionCommand messages depending on the current mode."""
        if self.mode == "MOTION":
            cmd = f"s{msg.steering}l{msg.right_speed}r{msg.left_speed}\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f"[MOTION] Sent: {cmd.strip()}")

        elif self.mode == "ZERO":
            cmd = "s0l0r0\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f"[ZERO] Sent: {cmd.strip()}")

        elif self.mode == "MANUAL":
            # Ignore topic messages in manual mode
            return

    def _keyboard_loop(self):
        """Read keyboard input: space for mode, WASD/Z for manual control."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)

                # Mode change with spacebar
                if ch == " ":
                    self._cycle_mode()
                    # Clear serial buffers when changing mode
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    continue

                # Manual control keys are only active in MANUAL mode
                if self.mode != "MANUAL":
                    continue

                updated = False

                if ch == "w":
                    # Increase left/right speed
                    self.manual_left_speed += self.speed_step
                    self.manual_right_speed += self.speed_step
                    updated = True

                elif ch == "s":
                    # Decrease left/right speed
                    self.manual_left_speed -= self.speed_step
                    self.manual_right_speed -= self.speed_step
                    updated = True

                elif ch == "a":
                    # Decrease steering
                    self.manual_steering -= self.steering_step
                    updated = True

                elif ch == "d":
                    # Increase steering
                    self.manual_steering += self.steering_step
                    updated = True

                elif ch == "f":
                    # Reset all to zero
                    self.manual_steering = 0
                    self.manual_left_speed = 0
                    self.manual_right_speed = 0
                    updated = True

                if updated:
                    # Clamp values
                    self.manual_left_speed = max(
                        self.min_speed, min(self.max_speed, self.manual_left_speed)
                    )
                    self.manual_right_speed = max(
                        self.min_speed, min(self.max_speed, self.manual_right_speed)
                    )
                    self.manual_steering = max(
                        self.min_steering, min(self.max_steering, self.manual_steering)
                    )

                    cmd = (
                        f"s{self.manual_steering}"
                        f"l{self.manual_left_speed}"
                        f"r{self.manual_right_speed}\n"
                    )
                    self.ser.write(cmd.encode())
                    self.get_logger().info(
                        f"Sent: {cmd.strip()}"
                    )

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _cycle_mode(self):
        """Cycle mode: ZERO -> MOTION -> MANUAL -> ZERO."""
        if self.mode == "ZERO":
            self.mode = "MOTION"
        elif self.mode == "MOTION":
            self.mode = "MANUAL"
            # Reset manual values when entering MANUAL mode
            self.manual_steering = 0
            self.manual_left_speed = 0
            self.manual_right_speed = 0
        else:
            self.mode = "MOTION"

        self.get_logger().info(f"Mode changed to: {self.mode}")

        # When entering ZERO mode, send immediate zero command
        if self.mode == "ZERO":
            cmd = "s0l0r0\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f"[ZERO] Sent: {cmd.strip()} (mode change)")

        # When entering MANUAL mode, send initial zero command too
        if self.mode == "MANUAL":
            cmd = "s0l0r0\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f"[MANUAL] Sent: {cmd.strip()} (mode change)")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
