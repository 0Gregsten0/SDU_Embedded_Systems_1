#!/usr/bin/env python3
import shutil
from pathlib import Path

import rclpy
from rclpy.node import Node


class FileTransferNode(Node):
    def __init__(self):
        super().__init__('file_transfer_node')
        self.declare_parameter('src_dir', 'uart_frames')
        self.declare_parameter('dst_dir', 'transferred_frames')
        self.declare_parameter('pattern', '*.jpg')
        self.declare_parameter('interval', 0.5)
        self.declare_parameter('keep_src', False)
        self.declare_parameter('max_per_cycle', 0)
        self.declare_parameter('keep', 10)  # <-- keep only the N newest files in dst

        self.src = Path(self.get_parameter('src_dir').get_parameter_value().string_value or 'uart_frames')
        self.dst = Path(self.get_parameter('dst_dir').get_parameter_value().string_value or 'transferred_frames')
        self.pattern = self.get_parameter('pattern').get_parameter_value().string_value or '*.jpg'
        self.interval = float(self.get_parameter('interval').value)
        self.keep_src = bool(self.get_parameter('keep_src').value)
        self.max_per_cycle = int(self.get_parameter('max_per_cycle').value)
        self.keep = max(0, int(self.get_parameter('keep').value))

        self.dst.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f"Transferring '{self.pattern}' from {self.src.resolve()} -> {self.dst.resolve()} "
            f"(keep_src={self.keep_src}, keep={self.keep})"
        )
        self.create_timer(self.interval, self._tick)

    def _tick(self):
        try:
            files = sorted(self.src.glob(self.pattern))
            moved = 0
            for p in files:
                if self.max_per_cycle and moved >= self.max_per_cycle:
                    break
                target = self.dst / p.name
                try:
                    if self.keep_src:
                        shutil.copy2(p, target)
                        action = "copied"
                    else:
                        shutil.move(str(p), str(target))
                        action = "moved"
                    moved += 1
                    self.get_logger().info(f"{action}: {p.name} -> {target}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to transfer {p.name}: {e!r}")

            # After each cycle, enforce retention in the destination
            self._enforce_retention()
        except Exception as e:
            self.get_logger().warn(f"Error during transfer tick: {e!r}")

    def _enforce_retention(self):
        """Keep only the newest N files in the destination directory."""
        if self.keep <= 0:
            return
        try:
            files = list(self.dst.glob(self.pattern))
            # Sort by modification time (newest first)
            files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
            # Delete everything beyond the first N
            for old in files[self.keep:]:
                try:
                    old.unlink(missing_ok=True)
                    self.get_logger().info(f"removed old file: {old.name}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to remove {old.name}: {e!r}")
        except Exception as e:
            self.get_logger().warn(f"Retention enforcement failed: {e!r}")


def main(args=None):
    rclpy.init(args=args)
    node = FileTransferNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
