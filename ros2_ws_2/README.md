
# on every new terminal initialize ros2: 
source /opt/ros/jazzy/setup.bash

# Workflow:
# 1. Go to workspace
cd ros2_ws_2

# 1_2 deleate left over folders
rm -rf build/ install/ log/

# 2. Build
colcon build --packages-select my_robot_package

# 3. Source
source install/setup.bash

# 4. Select & Run using a launch file command
ros2 launch my_robot_package serial_image_sender.launch.py



# Overview Topics and nodes:
Image_Read
    input_topic: /camera/image_raw


## Serial Image Sender (UART)

This adds a ROS 2 node that subscribes to `/image_raw`, encodes each frame to JPEG, and streams it over a USB-UART (default `/dev/ttyUSB0`).

### Build
```bash
colcon build --packages-select my_robot_package
source install/setup.bash
```

### Run
```bash
ros2 launch my_robot_package serial_image_sender.launch.py
# or directly:
ros2 run my_robot_package serial_image_sender --ros-args -p port:=/dev/ttyUSB0 -p baud:=921600 -p topic:=/camera/image_raw -p jpeg_quality:=90
```

### Receiver (optional, non-ROS)
On another machine or the same one, you can receive and save frames via:
```bash
pip install pyserial
python src/my_robot_package/tools/receive_serial_images.py --port /dev/ttyUSB0 --baud 921600 --out frames
```

### Packet format
```
MAGIC='IMJ1' (4 bytes)
sec (u32), nsec (u32)
width (u16), height (u16)
enc_len (u8), encoding (ASCII, enc_len bytes)
jpg_len (u32), jpg_data (jpg_len bytes)
crc32 (u32) over jpg_data
```
