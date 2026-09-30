cmake_minimum_required(VERSION 3.8)
project(my_robot_usart_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ===== 增添：文件 =====
add_executable(my_robot_usart_camera
  include/my_robot_usart_camera/mobile_base_hardware_usart.cpp
)

# 让编译器能找到 include/my_robot_usart_camera/xxx.hpp
target_include_directories(my_robot_usart_camera PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
