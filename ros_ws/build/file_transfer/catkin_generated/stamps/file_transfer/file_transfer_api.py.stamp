#!/usr/bin/env python3
"""
文件传输API封装 - 用于集成到其他项目
提供简洁的接口供其他ROS节点调用
"""
import rospy
import os
from file_transfer_pkg.srv import FileTransfer, FileTransferRequest

class FileTransferClient:
    """文件传输客户端API"""
    
    def __init__(self, service_name='file_transfer', timeout=10.0):
        """
        初始化文件传输客户端
        
        参数:
            service_name: ROS服务名称
            timeout: 等待服务的超时时间(秒)
        """
        self.service_name = service_name
        self.timeout = timeout
        
        try:
            rospy.wait_for_service(self.service_name, timeout=self.timeout)
            self.file_transfer = rospy.ServiceProxy(self.service_name, FileTransfer)
            rospy.loginfo(f"Connected to file transfer service at '{self.service_name}'")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to file transfer service: {str(e)}")
            raise
    
    def download_from_lower(self, lower_source_path, upper_target_path):
        """
        从下位机下载文件到上位机
        
        参数:
            lower_source_path: 下位机上的源文件路径
            upper_target_path: 上位机上的目标文件路径
            
        返回:
            (success, message, file_size)
        """
        try:
            request = FileTransferRequest()
            request.source_path = lower_source_path
            request.target_path = upper_target_path
            request.is_download = True
            
            response = self.file_transfer(request)
            
            if response.success:
                # 将接收到的文件数据写入目标路径
                target_dir = os.path.dirname(upper_target_path)
                if target_dir and not os.path.exists(target_dir):
                    os.makedirs(target_dir)
                
                file_data_bytes = bytes(response.file_data)
                with open(upper_target_path, 'wb') as f:
                    f.write(file_data_bytes)
                
                return True, response.message, response.file_size
            else:
                return False, response.message, 0
                
        except Exception as e:
            error_msg = f"Download failed: {str(e)}"
            rospy.logerr(error_msg)
            return False, error_msg, 0
    
    def upload_to_lower(self, upper_source_path, lower_target_path):
        """
        从上位机上传文件到下位机
        
        参数:
            upper_source_path: 上位机上的源文件路径
            lower_target_path: 下位机上的目标文件路径
            
        返回:
            (success, message, file_size)
        """
        try:
            request = FileTransferRequest()
            request.source_path = upper_source_path
            request.target_path = lower_target_path
            request.is_download = False
            
            # 读取上位机上的文件
            with open(upper_source_path, 'rb') as f:
                file_data = f.read()
            request.file_data = list(file_data)
            
            response = self.file_transfer(request)
            return response.success, response.message, response.file_size
            
        except Exception as e:
            error_msg = f"Upload failed: {str(e)}"
            rospy.logerr(error_msg)
            return False, error_msg, 0

# 便捷函数
def download_from_lower(lower_source_path, upper_target_path, service_name='file_transfer', timeout=10.0):
    """便捷函数: 从下位机下载文件到上位机"""
    client = FileTransferClient(service_name, timeout)
    return client.download_from_lower(lower_source_path, upper_target_path)

def upload_to_lower(upper_source_path, lower_target_path, service_name='file_transfer', timeout=10.0):
    """便捷函数: 从上位机上传文件到下位机"""
    client = FileTransferClient(service_name, timeout)
    return client.upload_to_lower(upper_source_path, lower_target_path)
