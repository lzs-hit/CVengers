cmake_minimum_required(VERSION 3.8...3.10)
project(cvengers_challenge)

#设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 手动设置Libtorch路径
set(LIBTORCH_HOME "/home/lzs/Vision_Arena_2025/libtorch")
set(Torch_DIR "${LIBTORCH_HOME}/share/cmake/Torch")

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
find_package(referee_pkg REQUIRED)
find_package(armor_recognition_interfaces REQUIRED)

# 查找 Libtorch 包
find_package(Torch REQUIRED PATHS "${LIBTORCH_HOME}" NO_DEFAULT_PATH)

if(Torch_FOUND)
  message(STATUS "LibTorch found at: ${LIBTORCH_HOME}")
else()
  message(FATAL_ERROR "LibTorch not found! Please set the correct LIBTORCH_HOME path")
endif()

include_directories(
  src/include
  ${OpenCV_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

#vision_node

#armor_node
#创建可执行文件
add_executable(armor_node 
  src/vision_node.cpp
  src/src/image_processor.cpp
  src/src/armor_detector.cpp
)

target_compile_definitions(armor_node PRIVATE ARMORNODE_MAIN)

#为目标添加依赖
ament_target_dependencies(armor_node
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

#train_digit_model_cpp

add_executable(train_digit_model_cpp 
    src/train_digit_model_libtorch.cpp 
)

# 为目标添加ROS2依赖（不包括Torch）
ament_target_dependencies(train_digit_model_cpp
  rclcpp
  # 移除Torch相关，因为Torch不是ROS2包
)

# 单独链接Torch库和其他非ROS2库
target_link_libraries(train_digit_model_cpp 
  ${TORCH_LIBRARIES}
  ${OpenCV_LIBS}
)

# 添加包含目录
target_include_directories(train_digit_model_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${TORCH_INCLUDE_DIRS}
)

# install
# 安装可执行文件
install(TARGETS
  armor_node
  train_digit_model_cpp  # 修正：这里应该是train_digit_model_cpp
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

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

ament_package()
