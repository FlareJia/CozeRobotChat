#!/usr/bin/env python3
"""
文件传输服务端 - 运行在上位机上
提供文件传输服务，供本地应用调用，同时也接收下位机的下载请求
"""
import rospy
import os
import logging
from file_transfer_pkg.srv import FileTransfer, FileTransferResponse

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('file_transfer_server')

def transfer_file(req):
    """处理文件传输请求"""
    logger.info(f"Received file transfer request: source={req.source_path}, target={req.target_path}, is_download={req.is_download}")
    
    try:
        # 处理下载请求 (下位机 -> 上位机)
        if req.is_download:
            if not os.path.exists(req.source_path):
                error_msg = f"Source file does not exist on lower computer: {req.source_path}"
                logger.error(error_msg)
                return FileTransferResponse(
                    success=False,
                    message=error_msg,
                    file_size=0
                )
            
            # 读取文件
            with open(req.source_path, 'rb') as f:
                file_data = f.read()
            
            # 准备响应
            response = FileTransferResponse()
            response.success = True
            response.message = f"File prepared for download from {req.source_path}"
            response.file_size = len(file_data)
            # 将bytes转换为list of int (uint8)
            response.file_data = list(file_data)
            
            logger.info(f"Prepared file for download: {req.source_path}, size: {len(file_data)} bytes")
            return response
            
        # 处理上传请求 (上位机 -> 下位机)
        else:
            # 确保目标目录存在
            target_dir = os.path.dirname(req.target_path)
            if target_dir and not os.path.exists(target_dir):
                os.makedirs(target_dir)
                logger.info(f"Created directory: {target_dir}")
            
            # 将请求中的file_data（list of int）转换为bytes并写入文件
            file_data_bytes = bytes(req.file_data)
            with open(req.target_path, 'wb') as f:
                f.write(file_data_bytes)
            
            logger.info(f"Uploaded file to: {req.target_path}, size: {len(file_data_bytes)} bytes")
            return FileTransferResponse(
                success=True,
                message=f"File uploaded successfully to {req.target_path}",
                file_size=len(file_data_bytes)
            )
            
    except Exception as e:
        error_msg = f"File transfer error: {str(e)}"
        logger.exception(error_msg)
        return FileTransferResponse(
            success=False,
            message=error_msg,
            file_size=0
        )

def file_transfer_server():
    """初始化并启动文件传输服务"""
    rospy.init_node('file_transfer_server')
    
    # 从参数服务器获取配置
    service_name = rospy.get_param('~service_name', 'file_transfer')
    
    # 创建服务
    service = rospy.Service(service_name, FileTransfer, transfer_file)
    
    # 获取服务的完整名称
    full_service_name = rospy.resolve_name(service_name)
    rospy.loginfo(f"File transfer server is ready at '{full_service_name}'")
    
    rospy.spin()

if __name__ == "__main__":
    file_transfer_server()
