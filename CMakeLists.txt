cmake_minimum_required(VERSION 3.5)
project(coppelia_sim_ros_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EEPoseGoals.msg"
  "msg/JointAngles.msg"
  "msg/ObjectInfo.msg"
  "srv/AddLine.srv"
  "srv/AddOrEditObject.srv"
  "srv/GripperControl.srv"
  "srv/RemoveObject.srv"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

# Install Python package
ament_python_install_package(coppelia_sim_ros_interface)

# Install examples
set(EXAMPLES_DIR coppelia_sim_ros_interface)
install(PROGRAMS
    ${EXAMPLES_DIR}/launch_entries/server_run.py
    ${EXAMPLES_DIR}/launch_entries/example_run.py
    DESTINATION lib/${PROJECT_NAME}
)

#ament_export_dependencies(rosidl_default_runtime)

# Setup the project
ament_package()
