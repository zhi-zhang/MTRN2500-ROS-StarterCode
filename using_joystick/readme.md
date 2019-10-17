# Joystick Example

## How to use:

In terminal 1:
1. Build using the command "colcon build"
2. Source using the command "source install/setup.bash"
3. Launch using the command "ros2 launch joystick_example joy_node.py"

In terminal 2:
1. Source using the command "source install/setup.bash"
2. Listern to the topic /joy using the command "ros2 topic echo joy"