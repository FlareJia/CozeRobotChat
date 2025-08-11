#!/usr/bin/env python3
import rospy
from file_transfer.scripts.file_transfer_api import FileTransferClient, download_from_lower, upload_to_lower

def main():
    rospy.init_node('my_main_application')
    
    try:
        '''
        # 方案1: 使用FileTransferClient类
        file_transfer = FileTransferClient()
        
        # 从下位机下载文件到上位机
        success, message, size = file_transfer.download_from_lower(
            lower_source_path="/home/lab/data/source.txt",  # 下位机路径
            upper_target_path="/home/leju_kuavo/received.txt"  # 上位机路径
        )
        
        if success:
            rospy.loginfo(f"Download successful! Size: {size} bytes")
        else:
            rospy.logerr(f"Download failed: {message}")
            
        # 从上位机上传文件到下位机
        success, message, size = file_transfer.upload_to_lower(
            upper_source_path="/home/leju_kuavo/source.txt",  # 上位机路径
            lower_target_path="/home/lab/received.txt"  # 下位机路径
        )
        
        if success:
            rospy.loginfo(f"Upload successful! Size: {size} bytes")
        else:
            rospy.logerr(f"Upload failed: {message}")
        '''
        '''
        # 方案2: 使用便捷函数
        success, message, size = download_from_lower(
            "/home/lab/data/another_file.txt",
            "/home/leju_kuavo/another_received.txt"
        )
        '''
        
        success, message, size = upload_to_lower(
            "/home/leju_kuavo/another_source.txt",
            "/home/lab/another_received.txt"
        )
            
    except rospy.ROSException as e:
        rospy.logerr(f"Failed to connect to file transfer service: {str(e)}")

if __name__ == "__main__":
    main()
