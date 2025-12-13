#!/bin/bash

# 1. Source your ROS workspace
source install/setup.bash

echo "Starting Motor Driver in the background..."
# Start the driver and put it in the background using '&'
ros2 run my_boat_driver motor_driver_node &
DRIVER_PID=$!

# Wait a second for it to start
sleep 2

echo "Starting Teleop Keyboard..."
echo "Use keys to drive. Press CTRL+C to exit."

# 2. Run Teleop in the foreground so it gets your keystrokes
ros2 run my_boat_teleop teleop_keyboard_node

# 3. Cleanup: When you quit Teleop, kill the driver
kill $DRIVER_PID
