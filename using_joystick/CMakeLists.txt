cmake_minimum_required(VERSION 3.10)
project(joystick_example)

find_package(ament_cmake REQUIRED)

# install launch files
install (DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
