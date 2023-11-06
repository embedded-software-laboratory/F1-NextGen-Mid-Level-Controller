# At least 3.10 so that IMPORTED targets can be used
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# Specify the project the used language and the version
project(lab_msgs VERSION 0.1)

# Configure compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
        msg/Header.msg
        msg/PathPoint.msg
        msg/Pose2D.msg
        msg/TimeStamp.msg
        msg/TrajectoryPoint.msg
        msg/VehicleCommandDirect.msg
        msg/VehicleCommandPathTracking.msg
        msg/VehicleCommandSpeedCurvature.msg
        msg/VehicleCommandTrajectory.msg
        msg/VehicleObservation.msg
        msg/VehicleState.msg
)
ament_package()
