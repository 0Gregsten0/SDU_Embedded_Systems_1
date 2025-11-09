# Workspace for uart_image_receiver

# on every new terminal initialize ros2: 
source /opt/ros/humble/setup.bash

# Workflow:
# 1. Go to workspace
cd ~/Desktop/embedded_temp/uart_image_receiver_ws/src/uart_image_receiver/

# 1_2 deleate left over folders
rm -rf build/ install/ log/

# 2. Build
colcon build --packages-select uart_image_receiver

# 3. Source
source install/setup.bash

# 4. Select & Run using a launch file command
ros2 launch uart_image_receiver uart_image_receiver.launch.py


