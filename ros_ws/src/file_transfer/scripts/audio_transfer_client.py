#!/usr/bin/env python3
"""
音频文件传输客户端脚本
供主程序通过命令行调用，上传音频文件到下位机
"""
import rospy
import argparse
from file_transfer_api import upload_to_lower  # 导入你已有的便捷函数

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='上传音频文件到下位机')
    parser.add_argument('--upper_source', required=True, 
                      help='上位机中音频文件的绝对路径')
    parser.add_argument('--lower_target', required=True, 
                      help='下位机中保存音频的目标路径（如：/home/robot/audio/output.wav）')
    args = parser.parse_args()

    # 初始化ROS节点（必须在调用服务前初始化）
    rospy.init_node('audio_transfer_client', anonymous=True)

    # 调用上传函数（使用你已有的FileTransferClient）
    success, msg, file_size = upload_to_lower(
        upper_source_path=args.upper_source,
        lower_target_path=args.lower_target
    )

    # 输出结果（主程序会捕获这些输出）
    if success:
        rospy.loginfo(f"上传成功: {msg}, 大小: {file_size}字节")
        exit(0)  # 成功退出码
    else:
        rospy.logerr(f"上传失败: {msg}")
        exit(1)  # 失败退出码

if __name__ == "__main__":
    main()
