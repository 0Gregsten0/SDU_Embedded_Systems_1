#!/usr/bin/env python3
import os
import time
import struct
import zlib
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node

try:
    import serial
except Exception:
    serial = None

MAGIC = b'IMJ1'
HEADER_FIXED = 4 + 4 + 2 + 2 + 1  # sec,u32 + nsec,u32 + w,u16 + h,u16 + enc_len,u8

class UARTImageReceiver(Node):
    def __init__(self):
        super().__init__('uart_image_receiver')

        # Parameters
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('output_dir', 'uart_frames')
        self.declare_parameter('keep', 10)
        self.declare_parameter('print_every', 1)

        self.port = self.get_parameter('port').get_parameter_value().string_value or '/dev/ttyTHS1'
        self.baud = int(self.get_parameter('baud').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.output_dir = Path(self.get_parameter('output_dir').get_parameter_value().string_value or 'uart_frames')
        self.keep = int(self.get_parameter('keep').value)
        self.print_every = max(1, int(self.get_parameter('print_every').value))

        if serial is None:
            self.get_logger().error('pyserial not installed. Please: pip install pyserial')
            raise RuntimeError('Missing dependency: pyserial')

        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Open serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except Exception as e:
            self.get_logger().fatal(f'Failed to open {self.port} @ {self.baud}: {e!r}')
            raise

        self.get_logger().info(f'Listening on {self.port} @ {self.baud}, saving to: {self.output_dir.resolve()}')

        self._stop_evt = threading.Event()
        self._frames = 0
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        self.create_timer(5.0, self._heartbeat)

    def destroy_node(self):
        self.get_logger().info('Shutting down receiver...')
        self._stop_evt.set()
        try:
            if self._thread.is_alive():
                self._thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        return super().destroy_node()

    def _heartbeat(self):
        self.get_logger().info(f'Frames saved so far: {self._frames} (keeping last {self.keep})')

    def _reader_loop(self):
        buf = bytearray()
        while not self._stop_evt.is_set():
            try:
                chunk = self.ser.read(2048)
                if chunk:
                    buf += chunk
                else:
                    continue

                while True:
                    i = buf.find(MAGIC)
                    if i < 0:
                        if len(buf) > 8:
                            del buf[:-8]
                        break

                    if i > 0:
                        del buf[:i]

                    if len(buf) < 4 + HEADER_FIXED:
                        break

                    sec, nsec, width, height, enc_len = struct.unpack_from('>IIHHB', buf, 4)
                    header_total = 4 + HEADER_FIXED + enc_len + 4  # magic + fixed + enc + jpg_len
                    if len(buf) < header_total:
                        break

                    enc_start = 4 + HEADER_FIXED
                    enc = bytes(buf[enc_start:enc_start+enc_len]).decode('ascii', errors='ignore')

                    jpg_len = struct.unpack_from('>I', buf, enc_start+enc_len)[0]
                    packet_total = header_total + jpg_len + 4  # + jpg + crc
                    if len(buf) < packet_total:
                        break

                    jpg_start = header_total
                    jpg = bytes(buf[jpg_start:jpg_start+jpg_len])
                    crc_rx = struct.unpack_from('>I', buf, jpg_start+jpg_len)[0]
                    del buf[:packet_total]

                    crc = zlib.crc32(jpg) & 0xffffffff
                    if crc != crc_rx:
                        self.get_logger().warn('CRC mismatch, dropping frame')
                        continue

                    ts = f'{sec:010d}_{nsec:09d}'
                    fname = self.output_dir / f'{ts}_{width}x{height}.jpg'
                    try:
                        with open(fname, 'wb') as f:
                            f.write(jpg)
                    except Exception as e:
                        self.get_logger().error(f'Failed writing {fname.name}: {e!r}')
                        continue

                    self._frames += 1
                    if (self._frames % self.print_every) == 0:
                        self.get_logger().info(f'Saved frame #{self._frames} -> {fname.name} (enc={enc}, {len(jpg)} bytes)')

                    self._enforce_retention()
            except Exception as e:
                self.get_logger().warn(f'Error in reader loop: {e!r}')
                time.sleep(0.01)

    def _enforce_retention(self):
        try:
            files = sorted(self.output_dir.glob('*.jpg'))
            excess = max(0, len(files) - self.keep)
            for i in range(excess):
                try:
                    files[i].unlink(missing_ok=True)
                    self.get_logger().debug(f'Removed old frame {files[i].name}')
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().warn(f'Failed enforcing retention: {e!r}')


def main(args=None):
    rclpy.init(args=args)
    node = UARTImageReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
