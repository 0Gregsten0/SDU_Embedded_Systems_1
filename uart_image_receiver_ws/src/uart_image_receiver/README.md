# uart_image_receiver (ROS 2 Humble)

A small ROS 2 (rclpy) node that receives JPEG frames over a USB–UART using the IMJ1 packet format and keeps only the last N frames (default 10) on disk. It prints what it does in the terminal.

## Build
```bash
colcon build --packages-select uart_image_receiver
source install/setup.bash
```

## Run
```bash
ros2 launch uart_image_receiver uart_image_receiver.launch.py
# or:
ros2 run uart_image_receiver uart_image_receiver   --ros-args -p port:=/dev/ttyUSB0 -p baud:=921600 -p output_dir:=uart_frames -p keep:=10
```

## Parameters
- `port` (string): Serial device path. Default `/dev/ttyUSB0`
- `baud` (int): Baud rate. Default `921600`
- `timeout` (double): Serial read timeout seconds. Default `0.1`
- `output_dir` (string): Directory where frames are saved. Default `uart_frames`
- `keep` (int): How many most recent frames to keep on disk. Default `10`
- `print_every` (int): Log every N frames. Default `1`

## Packet Format
Must match the sender:
```
MAGIC='IMJ1' (4 bytes)
sec (u32), nsec (u32)
width (u16), height (u16)
enc_len (u8), encoding (ASCII, enc_len bytes)
jpg_len (u32), jpg_data (jpg_len bytes)
crc32 (u32) over jpg_data
```
