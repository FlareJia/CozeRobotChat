cmake_minimum_required(VERSION 3.0.2)
project(move_pkg)

## 找到必要的包（移除humanoid_plan_arm_trajectory）
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  trajectory_msgs
  message_generation
)

## 声明服务文件（保持不变）
add_service_files(
  FILES
  planArmTrajectoryCubicSpline.srv
  changeArmCtrlMode.srv
  ExecuteAction.srv
)

## 声明消息文件（保持不变）
add_message_files(
  FILES
  sensorsData.msg
  jointData.msg
  imuData.msg
  endEffectorData.msg
)

## 生成消息和服务（保持不变）
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  trajectory_msgs
)

## 配置catkin包（保持不变）
catkin_package(
  CATKIN_DEPENDS 
  rospy 
  std_msgs 
  sensor_msgs 
  trajectory_msgs 
  message_runtime
)

## 包含目录（保持不变）
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 安装Python脚本（保持不变）
catkin_install_python(PROGRAMS
  scripts/move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)