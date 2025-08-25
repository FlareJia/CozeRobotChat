from typing import Optional
import os
import numpy as np
from datetime import datetime
from config import Config
import sys
import cv2
import pyrealsense2 as rs
from loguru import logger  # 确保已安装：pip install loguru
import shutil  # 补充导入（代码中用到 shutil.disk_usage）

# 从你的模块中导入 CameraService（假设类定义在同目录的 camera_service.py 中）
from services.camera_service import CameraService  # 替换为实际文件名


if __name__ == "__main__":
    # 1. 配置日志（可选，若需保存日志）
    logger.add("camera.log", level="INFO", format="{time} | {level} | {message}")
    
    # 2. 初始化相机服务（可指定输出目录，默认是 "images"）
    image_output_dir = os.path.join(Config.OUTPUT_DIR, Config.IMAGE_NAMES["images_dir"])  # 保存图片的目录
    camera = CameraService(output_dir=image_output_dir)
    #output_dir = "images"  # 保存图片的目录
    #camera = CameraService(output_dir=output_dir)
    
    # 3. 捕获稳定图像（target_frame 控制等待帧数，越大越稳定，默认50）
    target_frame = 50  # 可根据需求调整
    image_path = camera.capture_stable_image(target_frame=target_frame)
    
    # 4. 处理结果
    if image_path:
        logger.success(f"图像保存成功：{image_path}")
        # 额外验证：打印文件详细信息
        file_stats = os.stat(image_path)
        logger.info(f"文件大小：{file_stats.st_size / 1024:.2f}KB")
        logger.info(f"修改时间：{datetime.fromtimestamp(file_stats.st_mtime).strftime('%Y-%m-%d %H:%M:%S')}")
    else:
        logger.error("拍照或保存失败，请检查日志！")