cmake_minimum_required(VERSION 3.8...3.10)
project(cvengers_challenge)

#设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pose_msg REQUIRED)
find_package(referee_pkg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(referee_pkg REQUIRED)#注意
find_package(armor_recognition_interfaces REQUIRED)#注意

include_directories(
  src/include
  ${OpenCV_INCLUDE_DIRS}
)

#vision_node
#创建可执行文件
add_executable(vision_node 
  src/vision_node.cpp
  src/src/image_processor.cpp
  src/src/armor_detector.cpp
)

#为目标添加依赖
ament_target_dependencies(vision_node 
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
  referee_pkg
  armor_recognition_interfaces
  pose_msg
)

# #shooter_node
# #创建可执行文件
# add_executable(shooter_node src/shooter_node.cpp)

# #为目标添加依赖
# ament_target_dependencies(shooter_node
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   cv_bridge
#   image_transport
#   OpenCV
#   referee_pkg
#   armor_recognition_interfaces
#   pose_msg
# )

# install
# 安装可执行文件
install(TARGETS
  vision_node
  # shooter_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装模型文件
# install(DIRECTORY models/
#   DESTINATION share/${PROJECT_NAME}/models
# )

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

ament_package()
