#!/usr/bin/env python3
"""
WS2812B (NeoPixel over SPI) status light. Same wiring as led_lighting/led_test.py.

Subscribes:  led/command   std_msgs/String

    off | solid_red | blink_red_slow | blink_red | blink_red_fast

Red only; the state is told apart by blink rate. blink_red runs at blink_hz,
_slow at a quarter of it and _fast at four times it.

Own node, own process, for the same reason thermal_sensor is: blinking means
sleeping between writes, and nothing that sleeps may ever share a thread with
the offboard heartbeat. If the LED hardware is missing the node logs once and
keeps running as a no-op, so a loose SPI wire can never take the mission down.

Bench:
    ros2 run drone_testing led_status
    ros2 topic pub -1 /led/command std_msgs/String "{data: blink_red}"
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RED = (255, 0, 0)
RATE_SCALE = {'blink_red_slow': 0.25, 'blink_red': 1.0, 'blink_red_fast': 4.0}


class LedStatus(Node):

    def __init__(self):
        super().__init__('led_status')

        self.num_pixels = int(self.declare_parameter('num_pixels', 5).value)
        self.blink_hz = float(self.declare_parameter('blink_hz', 2.0).value)
        self.brightness = float(self.declare_parameter('brightness', 1.0).value)

        self.pixels = None
        try:
            import board
            import neopixel_spi as neopixel
            self.pixels = neopixel.NeoPixel_SPI(
                board.SPI(), self.num_pixels,
                pixel_order=neopixel.GRB, auto_write=False)
            self.get_logger().info(f"WS2812B up: {self.num_pixels} pixels on SPI.")
        except Exception as exc:
            self.get_logger().error(
                f"No LED hardware ({exc}). The node stays up and logs commands "
                "instead, so the mission is never blocked by the light.")

        self.mode = 'off'
        self.phase = False
        self.last_toggle = 0.0
        self._write((0, 0, 0))

        self.create_subscription(String, 'led/command', self.command_callback, 10)
        # Fixed fast tick; each blink mode toggles on its own half-period.
        self.create_timer(0.02, self.tick)

    def command_callback(self, msg):
        mode = msg.data.strip().lower()
        if mode == self.mode:
            return
        self.mode = mode
        self.phase = True
        self.last_toggle = time.monotonic()
        self.get_logger().info(f"LED -> {mode}")
        self._show()

    def tick(self):
        scale = RATE_SCALE.get(self.mode)
        if scale is None:
            return
        half_period = 0.5 / max(self.blink_hz * scale, 0.1)
        now = time.monotonic()
        if now - self.last_toggle >= half_period:
            self.last_toggle = now
            self.phase = not self.phase
            self._show()

    def _show(self):
        if self.mode == 'solid_red':
            self._write(RED)
        elif self.mode in RATE_SCALE:
            self._write(RED if self.phase else (0, 0, 0))
        else:
            self._write((0, 0, 0))

    def _write(self, color):
        if self.pixels is None:
            return
        try:
            self.pixels.fill(tuple(int(c * self.brightness) for c in color))
            self.pixels.show()
        except Exception as exc:
            self.get_logger().warning(f"LED write failed: {exc}",
                                      throttle_duration_sec=5.0)

    def destroy_node(self):
        self._write((0, 0, 0))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LedStatus()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
