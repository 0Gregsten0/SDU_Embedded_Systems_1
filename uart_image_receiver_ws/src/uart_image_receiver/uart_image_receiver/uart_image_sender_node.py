#!/usr/bin/env python3
import os
import struct
import zlib
from pathlib import Path

import rclpy
from rclpy.node import Node

try:
    import serial
except Exception:
    serial = None

MAGIC = b'IMJ1'

class UARTImageSenderNode(Node):
    def __init__(self):
        super().__init__('uart_image_sender_node')
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('dir', 'transferred_frames')
        self.declare_parameter('interval', 1.0)
        self.declare_parameter('resend_same', False)

        self.port = self.get_parameter('port').get_parameter_value().string_value or '/dev/ttyTHS1'
        self.baud = int(self.get_parameter('baud').value)
        self.dir = Path(self.get_parameter('dir').get_parameter_value().string_value or 'transferred_frames')
        self.interval = float(self.get_parameter('interval').value)
        self.resend_same = bool(self.get_parameter('resend_same').value)

        if serial is None:
            self.get_logger().error('pyserial not installed. Please: pip install pyserial')
            raise RuntimeError('Missing dependency: pyserial')

        self.dir.mkdir(parents=True, exist_ok=True)

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=2.0)
        except Exception as e:
            self.get_logger().fatal(f'Failed to open {self.port} @ {self.baud}: {e!r}')
            raise

        self.last_sent = None
        self.get_logger().info(f"Sending newest JPEG from {self.dir.resolve()} over {self.port} @ {self.baud}")
        self.create_timer(self.interval, self._tick)

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        return super().destroy_node()

    def _tick(self):
        try:
            newest = self._newest_jpeg()
            if newest is None:
                return
            if (not self.resend_same) and self.last_sent == newest:
                return

            data = newest.read_bytes()
            width, height = self._parse_wh(newest.name)

            ts = newest.stat().st_mtime
            sec = int(ts)
            nsec = int((ts - sec) * 1e9)

            enc = b'bgr8'
            enc_len = len(enc)
            header = struct.pack('>4sIIHHB', MAGIC, sec, nsec, width, height, enc_len) + enc
            jpg_len = struct.pack('>I', len(data))
            crc = zlib.crc32(data) & 0xffffffff
            packet = header + jpg_len + data + struct.pack('>I', crc)

            total = 0
            mv = memoryview(packet)
            while total < len(packet):
                w = self.ser.write(mv[total:total+4096])
                if w is None:
                    w = 0
                total += w

            self.last_sent = newest
            self.get_logger().info(f"Sent {newest.name} ({len(data)} bytes)")
        except Exception as e:
            self.get_logger().warn(f"Send failed: {e!r}")

    def _newest_jpeg(self):
        files = sorted(self.dir.glob('*.jpg'), key=lambda p: p.stat().st_mtime, reverse=True)
        return files[0] if files else None

    @staticmethod
    def _parse_wh(name: str):
        try:
            base = os.path.splitext(name)[0]
            wh = base.split('_')[-1]
            w, h = wh.split('x')
            return int(w), int(h)
        except Exception:
            return 0, 0

def main(args=None):
    rclpy.init(args=args)
    node = UARTImageSenderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
