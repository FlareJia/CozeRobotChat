cmake_minimum_required(VERSION 3.0.2)
project(musicplayer_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## 添加服务定义
add_service_files(
  FILES
  playmusic.srv
)

## 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

## 声明 catkin 包信息
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs rospy
)

## 包含头文件目录（虽然这个包没用 C++，也保留）
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/client_play_music.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
