cmake_minimum_required(VERSION 3.8)
project(mid_level_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lab_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(mid_level_controller
        src/Controller.cpp
        src/casadi_mpc_fn.c
        src/Localization.cpp
        src/MpcController.cpp
        src/PathInterpolation.cpp
        src/PathTrackingController.cpp
        src/SensorCalibration.cpp
        src/TrajectoryInterpolation.cpp
        src/VehicleModel.cpp
        src/bcm2835.c
        src/low_level_controller/spi.c
        src/low_level_controller/crc.c
        )

target_compile_features(mid_level_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(mid_level_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mid_level_controller PRIVATE "MID_LEVEL_CONTROLLER_BUILDING_LIBRARY")

install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS mid_level_controller
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_target_dependencies(mid_level_controller rclcpp lab_msgs nav_msgs tf2_ros)

add_executable(mid_level_controller_node src/mid_level_controller_node.cpp)
ament_target_dependencies(mid_level_controller_node rclcpp lab_msgs nav_msgs tf2_ros)
target_link_libraries(mid_level_controller_node mid_level_controller)

target_include_directories(mid_level_controller_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS
        mid_level_controller_node
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(
        include
)
ament_export_libraries(
        mid_level_controller
)
ament_export_targets(
        export_${PROJECT_NAME}
)

ament_package()
