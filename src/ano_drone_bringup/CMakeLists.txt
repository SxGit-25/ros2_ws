cmake_minimum_required(VERSION 3.8)
project(ano_drone_bringup)

# 默认使用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(serial REQUIRED)

# 添加自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AnoImuData.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# 包含目录
include_directories(
  include
  ${serial_INCLUDE_DIRS}
)

# 硬件接口节点
add_executable(ano_hardware_interface_node src/ano_hardware_interface_node.cpp)
target_include_directories(ano_hardware_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${serial_INCLUDE_DIRS}
)

ament_target_dependencies(ano_hardware_interface_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  builtin_interfaces
  std_msgs
  serial
  ${PROJECT_NAME}
)

# 将节点注册为组件
rclcpp_components_register_nodes(ano_hardware_interface_node "ano_drone_bringup::AnoHardwareInterfaceNode")

# 原有的ano_bridge_node（如果存在）
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/ano_bridge_node.cpp)
  add_executable(ano_bridge_node src/ano_bridge_node.cpp)
  target_include_directories(ano_bridge_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(ano_bridge_node
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )
  install(TARGETS ano_bridge_node
    DESTINATION lib/${PROJECT_NAME})
endif()

# 安装目标
install(TARGETS ano_hardware_interface_node
  DESTINATION lib/${PROJECT_NAME})

# 安装目录
install(DIRECTORY
  launch
  config
  rviz
  include
  DESTINATION share/${PROJECT_NAME}
)

# 安装消息定义
install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()