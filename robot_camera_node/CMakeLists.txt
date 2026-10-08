cmake_minimum_required(VERSION 3.8)
project(robot_camera_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 关键：包含头文件目录
include_directories(include)

# 编译节点（务必把相机底层驱动一起编译）
add_executable(camera_node
  src/camera_node.cpp
  src/mobile_base_hardware_camera.cpp
)

# 链接 ROS 2 依赖（推荐现代写法）
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

# 链接 OpenCV
target_link_libraries(camera_node
  ${OpenCV_LIBS}
)

# 安装可执行文件
install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()