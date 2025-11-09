#!/usr/bin/env python3
import os
import sys
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

# Optional deps; import lazily so the node can start even if not present, and we error clearly on first use.
try:
    import numpy as np
except Exception as e:
    np = None

try:
    import cv2
except Exception as e:
    cv2 = None

try:
    import serial  # pip install pyserial
    import serial.tools.list_ports as list_ports
except Exception as e:
    serial = None
    list_ports = None


MAGIC = b'IMJ1'  # 4-byte magic to identify a JPEG image frame packet v1


def _encoding_to_cv2(encoding: str):
    # Map common ROS encodings to cv2 color conversion codes
    # Returns (code or None), channels
    encoding = (encoding or '').lower()
    if encoding in ('bgr8', 'bgr8'):
        return None, 3
    if encoding in ('rgb8',):
        return cv2.COLOR_RGB2BGR, 3
    if encoding in ('mono8', '8uc1', 'grayscale8'):
        return None, 1
    # 16-bit mono downscale to 8-bit for transport
    if encoding in ('mono16', '16uc1', '16sc1'):
        return 'mono16', 1
    # Fallback: assume already BGR-packed
    return None, 3


class SerialImageSender(Node):
    def __init__(self):
        super().__init__('serial_image_sender')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('topic', '/image_raw')
        self.declare_parameter('frame_skip', 0)  # send every frame by default
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('chunk_size', 4096)
        self.declare_parameter('open_timeout', 3.0)
        self.declare_parameter('qos_depth', 1)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').get_parameter_value().integer_value or self.get_parameter('baud').value)
        topic = self.get_parameter('topic').get_parameter_value().string_value
        qdepth = int(self.get_parameter('qos_depth').get_parameter_value().integer_value or 1)

        qos = QoSProfile(
            depth=qdepth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )

        self._sub = self.create_subscription(Image, topic, self._on_image, qos)

        self._frame_skip = int(self.get_parameter('frame_skip').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._chunk_size = int(self.get_parameter('chunk_size').value)
        self._open_timeout = float(self.get_parameter('open_timeout').value)

        if serial is None:
            self.get_logger().error('pyserial is not installed. Please: pip install pyserial')
            raise RuntimeError('Missing dependency: pyserial')
        if np is None:
            self.get_logger().error('numpy is not installed. Please: pip install numpy')
            raise RuntimeError('Missing dependency: numpy')
        if cv2 is None:
            self.get_logger().error('opencv-python is not installed. Please: pip install opencv-python')
            raise RuntimeError('Missing dependency: opencv-python')

        # Open the serial port
        self._ser = None
        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0, write_timeout=self._open_timeout)
            self.get_logger().info(f'Opened serial port {port} @ {baud} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e!r}')
            raise

        # Stats
        self._frame_count = 0
        self._sent_count = 0
        self._last_warn = 0.0

        # Prebuild JPEG params
        self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)]

    def destroy_node(self):
        try:
            if self._ser:
                self.get_logger().info('Closing serial port')
                self._ser.close()
        except Exception:
            pass
        return super().destroy_node()

    def _on_image(self, msg: Image):
        self._frame_count += 1
        if self._frame_skip > 0 and (self._frame_count % (self._frame_skip + 1)) != 1:
            return  # skip

        try:
            img = np.frombuffer(msg.data, dtype=np.uint8)

            # Handle different encodings
            if msg.encoding.lower() in ('mono16', '16uc1', '16sc1'):
                img16 = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
                # Normalize to 8-bit
                max_val = np.max(img16) if img16.size else 1
                scale = 255.0 / max(1.0, float(max_val))
                img8 = (img16.astype(np.float32) * scale).astype(np.uint8)
                bgr = cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
            else:
                channels = 1 if 'mono' in msg.encoding.lower() or msg.encoding.lower() in ('8uc1',) else 3
                img = img.reshape((msg.height, msg.width, channels)) if channels > 1 else img.reshape((msg.height, msg.width))
                convert_code, ch = _encoding_to_cv2(msg.encoding)
                if convert_code == 'mono16':
                    # Already handled above
                    bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                elif convert_code is None:
                    # Either BGR already or MONO
                    if ch == 3:
                        bgr = img
                    else:
                        bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                else:
                    bgr = cv2.cvtColor(img, convert_code)

            # JPEG encode
            ok, jpg = cv2.imencode('.jpg', bgr, self._encode_params)
            if not ok:
                raise RuntimeError('cv2.imencode failed')
            jpg_bytes = jpg.tobytes()

            # Packet:
            # MAGIC(4) | sec(4) | nsec(4) | width(2) | height(2) | enc_len(1) | encoding(bytes) | jpg_len(4) | jpg_data | CRC32(4)
            enc = (msg.encoding or 'bgr8').encode('ascii', errors='ignore')[:31]
            enc_len = len(enc)

            sec = msg.header.stamp.sec if hasattr(msg, 'header') else 0
            nsec = msg.header.stamp.nanosec if hasattr(msg, 'header') else 0

            header = struct.pack('>4sIIHHB', MAGIC, sec, nsec, msg.width, msg.height, enc_len) + enc
            jpg_len = struct.pack('>I', len(jpg_bytes))

            import zlib
            crc = zlib.crc32(jpg_bytes) & 0xffffffff
            crc_bytes = struct.pack('>I', crc)

            packet = header + jpg_len + jpg_bytes + crc_bytes

            # Write in chunks to avoid blocking for too long
            total = 0
            mv = memoryview(packet)
            while total < len(packet):
                end = min(total + self._chunk_size, len(packet))
                wrote = self._ser.write(mv[total:end])
                if wrote is None:
                    wrote = 0
                total += wrote

            self._sent_count += 1
            if self._sent_count % 30 == 0:
                self.get_logger().info(f'Sent {self._sent_count} frames, last size={len(packet)} bytes')

        except Exception as e:
            # Rate-limit error logs
            now = time.time()
            if now - self._last_warn > 1.0:
                self._last_warn = now
                self.get_logger().warn(f'Failed to process/send image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialImageSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
