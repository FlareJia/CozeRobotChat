#!/bin/bash

# 设置环境变量
export USER=lab
export HOME=/home/lab
export XDG_RUNTIME_DIR=/run/user/1000
export PULSE_SERVER=unix:/run/user/1000/pulse/native

# ROS网络配置
export ROS_MASTER_URI=http://kuavo_master:11311
export ROS_HOSTNAME=kuavo_master
export ROS_IP=$(hostname -I | awk '{print $1}')

# 确保PulseAudio目录权限正确
chmod 755 /run/user/1000/pulse

# 等待PulseAudio socket存在
while [ ! -S /run/user/1000/pulse/native ]; do
    echo "等待PulseAudio初始化..."
    sleep 1
done

# 加载ROS环境
source /opt/ros/noetic/setup.bash
source /home/lab/szhr/CozeRobotChat/ros_ws/devel/setup.bash

# 启动主程序
echo "启动Kuavo音频播放器..."
roslaunch kuavo_audio_player play_music.launch

# 在这里添加其他启动命令
# 例如：
# echo "启动其他组件..."
# rosrun other_package other_node
