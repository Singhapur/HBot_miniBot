#!/bin/bash

# Launch the camera package
gnome-terminal -- bash -c "python3 src/robot_tfg_pkg/robot_tfg_pkg/camera_reader_yolo.py; exec bash"

sleep 2

# Launch RViz
gnome-terminal -- bash -c "ros2 run rviz2 rviz2; exec bash"

sleep 2

# Launch teleop_twist_keyboard
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"

sleep 2

# Launch RQt
gnome-terminal -- bash -c "rqt; exec bash"
