# ROS包编译说明

## 项目介绍
kuavo_audio_player用于启动一个roservice,向play_music2服务端推送音频文件和音量就能播放指定目录下的指定音频.
音频存放目录为/home/lab/.config/lejuconfig/music
快速调试指令:
rosservice call /play_music2 "music_number: 'reserved_1.mp3' volume: 80"


musicplayer为测试用包,roslaunch会像play_music2推送一条播放音乐指令
## 环境要求
- ROS版本：Noetic
- 依赖包：                                 
  - rospy
  - std_msgs
  - kuavo_audio_player
  - numpy
  - pyaudio
  - samplerate
  - scipy

## 编译步骤
cd ros_test
catkin_make
source devel/setup.bash

## 启动指令
roslaunch kuavo_audio_player play_music.launch