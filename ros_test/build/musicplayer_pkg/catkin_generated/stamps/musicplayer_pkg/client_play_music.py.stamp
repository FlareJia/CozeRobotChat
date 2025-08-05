#!/usr/bin/env python3
import rospy
from music_client.srv import playmusic


def call_play_music(music_number, volume):
    rospy.wait_for_service('play_music2')
    try:
        play_music = rospy.ServiceProxy('play_music2', playmusic)
        response = play_music(music_number, volume)
        if response.success_flag:
            rospy.loginfo(f"音乐 {music_number} 播放成功，音量：{volume}%")
        else:
            rospy.logwarn("音乐播放失败")
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

if __name__ == '__main__':
    rospy.init_node('play_music_client')
    music_number = rospy.get_param("~music_number", "3_点赞.wav")
    volume = rospy.get_param("~volume", 80)
    call_play_music(music_number, volume)
