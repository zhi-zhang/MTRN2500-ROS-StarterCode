# Joystick Example

This example will show how to use the joystick driver packaged with ROS2. We will launch a node that will read inputs from joystick and broadcast to the node /joy.

The driver we will use is called joy_node from the package joy. The launch file will set the node to read from /dev/input/js1.

Joy driver repository: https://github.com/ros2/joystick_drivers
Documentation for joy: http://wiki.ros.org/joystick_drivers


## How to use:

We will first need to build the our package. 

1. Open a terminal to mtrn2500_ws folder. 
2. Build our package using the command `colcon build`
3. Source the new package using the command `source install/setup.bash`
4. Launch using the command `ros2 launch joystick_example joy_node.py`

To see if the node have launched successfully we can in another terminal run the command `ros2 topic echo joy` to print message sent to the topic /joy to the terminal.
