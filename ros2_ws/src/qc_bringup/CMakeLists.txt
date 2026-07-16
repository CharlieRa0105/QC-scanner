cmake_minimum_required(VERSION 3.8)
project(qc_bringup)

# Launch-only package: no compiled code, just install the launch/ directory so
# `ros2 launch qc_bringup <file>` finds them.

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
