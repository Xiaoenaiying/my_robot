cmake_minimum_required(VERSION 3.8)
project(my_robot_chassis)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
find_package(pluginlib REQUIRED)

# ═══ ros2_control 硬件插件库 ═══
add_library(origintank_hardware SHARED
    src/origintank_base_interface.cpp
    src/origintank_base.cpp
)
target_include_directories(origintank_hardware PRIVATE
    include
)
ament_target_dependencies(origintank_hardware
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface origintank_base_interface.xml)

install(TARGETS origintank_hardware
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# ═══ 安装资源 ═══
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY description/ DESTINATION share/${PROJECT_NAME}/description)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_export_libraries(origintank_hardware)

ament_package()
