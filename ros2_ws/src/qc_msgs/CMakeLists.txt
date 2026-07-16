cmake_minimum_required(VERSION 3.8)
project(qc_msgs)

# Interface-only package: define the QC Scanner's custom msgs/srvs/actions and
# let rosidl generate the C++/Python bindings. No node code lives here.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionState.msg"
  "msg/ScanWaypoint.msg"
  "msg/ScanPath.msg"
  "msg/ScanState.msg"
  "msg/InspectionResult.msg"
  "srv/StartMission.srv"
  "action/PlanPath.action"
  "action/ExecutePath.action"
  "action/Inspect.action"
  DEPENDENCIES geometry_msgs moveit_msgs
)

ament_package()
