#!/usr/bin/env python

import rospy
from kuavo_msgs.msg import robotHeadMotionData


class ControlHead:
    """双足机器人头部控制器SDK

    提供安全的头部运动控制接口，自动处理参数范围限制和初始化检查。
    """

    # 定义头部运动范围常量
    YAW_MIN = -30
    YAW_MAX = 30
    PITCH_MIN = -25
    PITCH_MAX = 25

    def __init__(self, auto_init=True):
        """初始化头部控制器

        :param auto_init: 是否自动初始化发布者，默认为True
        """
        self._head_pub = None
        self._initialized = False

        # 检查ROS节点是否已初始化
        if not rospy.core.is_initialized():
            rospy.logwarn("ROS node not initialized. Publisher will be created when init_head_controller is called.")

        if auto_init:
            self.init_head_controller()

    def init_head_controller(self):
        """初始化头部控制器

        :return: 是否成功初始化
        :rtype: bool
        """
        if self._initialized:
            return True

        try:
            self._head_pub = rospy.Publisher(
                '/robot_head_motion_data',
                robotHeadMotionData,
                queue_size=10,
                latch=True  # 对于状态类话题，latch=True可确保新订阅者收到最新消息
            )
            rospy.loginfo("Head controller initialized")

            # 等待发布者注册（但不阻塞）
            rospy.sleep(0.1)
            self._initialized = True
            return True

        except Exception as e:
            rospy.logerr(f"Failed to initialize head controller: {str(e)}")
            self._initialized = False
            return False

    def set_head_target(self, yaw, pitch, wait_for_feedback=False, feedback_timeout=2.0):
        """设置头部目标位置

        :param yaw: 头部偏航角，范围[-30, 30]度
        :param pitch: 头部俯仰角，范围[-25, 25]度
        :param wait_for_feedback: 是否等待执行反馈（当前SDK不支持，仅作预留）
        :param feedback_timeout: 等待反馈的超时时间（秒）
        :return: 是否成功发布指令
        :rtype: bool
        """
        # 检查初始化状态
        if not self._initialized:
            if not self.init_head_controller():
                rospy.logerr("Head controller failed to initialize. Cannot set head target.")
                return False

        # 参数类型验证
        try:
            yaw = float(yaw)
            pitch = float(pitch)
        except (TypeError, ValueError):
            rospy.logerr(f"Invalid yaw/pitch values: yaw={yaw}, pitch={pitch}. Must be numeric.")
            return False

        # 参数范围限制
        yaw = max(self.YAW_MIN, min(self.YAW_MAX, yaw))
        pitch = max(self.PITCH_MIN, min(self.PITCH_MAX, pitch))

        # 创建并发布消息
        try:
            head_target_msg = robotHeadMotionData()
            head_target_msg.joint_data = [yaw, pitch]
            self._head_pub.publish(head_target_msg)

            rospy.loginfo(f"Published head target: yaw={yaw:.1f}, pitch={pitch:.1f}")
            return True

        except Exception as e:
            rospy.logerr(f"Failed to publish head target: {str(e)}")
            return False

    def reset_head(self):
        """将头部复位到中立位置(0, 0)"""
        return self.set_head_target(0, 0)

    def is_initialized(self):
        """检查控制器是否已初始化"""
        return self._initialized