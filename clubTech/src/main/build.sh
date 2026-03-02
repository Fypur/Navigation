cd ~/ros2_ws
colcon build --packages-select "${1:-main}" && source install/setup.bash