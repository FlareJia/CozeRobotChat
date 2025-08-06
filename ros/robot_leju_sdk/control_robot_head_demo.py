#!/usr/bin/env python

"""
机器人头部运动演示
执行一个简单的摇头动作序列
"""

import rospy
from control_robot_head import ControlHead


def main():
    """主函数：演示头部控制功能"""
    rospy.init_node('robot_head_demo', anonymous=True)

    # 创建头部控制器实例
    head_controller = ControlHead()

    # 检查初始化状态
    if not head_controller.is_initialized():
        rospy.logerr("Failed to initialize head controller. Exiting.")
        return

    rospy.loginfo("Starting head movement demo...")

    try:
        # 定义动作序列
        movements = [
            (0, 0),  # 中立位置
            (-30, 0),  # 左转极限
            (30, 0),  # 右转极限
            (0, 0),  # 回到中立
            (0, -25),  # 低头极限
            (0, 25),  # 抬头极限
            (0, 0)  # 回到中立
        ]

        # 执行动作序列
        for i, (yaw, pitch) in enumerate(movements):
            rospy.loginfo(f"Executing movement {i + 1}/{len(movements)}: ({yaw}, {pitch})")
            if head_controller.set_head_target(yaw, pitch):
                # 实际应用中应使用反馈机制而非固定等待
                # 这里仅为演示使用rospy.sleep
                rospy.sleep(3)
            else:
                rospy.logwarn(f"Movement {i + 1} failed")
                break

        rospy.loginfo("Head movement demo completed successfully")

    except Exception as e:
        rospy.logerr(f"Error during demo execution: {str(e)}")
    finally:
        # 确保头部回到安全位置
        head_controller.reset_head()
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Head demo node shutdown")