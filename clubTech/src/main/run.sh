#!/bin/bash


if [ -z "$1" ]; then
    ros2 launch main launcher.py
else
    cd ~/ros2_ws
    colcon build --packages-select $1 && source install/setup.bash && ros2 run $1 "${2:-$1}"
fi
