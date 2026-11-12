cmake_minimum_required(VERSION 3.8)
project(my_robot_chassis)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

# 创建可执行目标
add_executable(origintank_node
    src/origintank_base.cpp
    src/serial_origintank.cpp
)

# 头文件目录
target_include_directories(origintank_node PRIVATE
    include
)

# ROS2 依赖
ament_target_dependencies(origintank_node
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
)

# 线程库
target_link_libraries(origintank_node
    Threads::Threads
)

# 安装可执行文件
install(TARGETS origintank_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()