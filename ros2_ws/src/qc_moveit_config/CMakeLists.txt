cmake_minimum_required(VERSION 3.8)
project(qc_moveit_config)

# Launch + config only: the overhead-mount xacro and the move_group launch.
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
