cmake_minimum_required(VERSION 3.8)
project(my_robot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= 1. 查找依赖包 =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# 如果你的串口库在其他包里，也需要在这里 find_package(my_robot_usart REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ================= 2. 编译你的源文件 =================
# 🚨 注意：请把 src/serial_arm.cpp 换成你实际的 cpp 文件路径！
add_executable(serial_arm 
    src/serial_arm.cpp
    src/mobile_base_hardware_usart.cpp)

# 链接依赖
ament_target_dependencies(serial_arm
  rclcpp
  geometry_msgs
)

# ================= 3. 安装可执行文件 =================
# 🚨 只有加了这段，ros2 run 才能找到它！
install(TARGETS serial_arm
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()