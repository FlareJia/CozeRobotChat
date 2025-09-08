from typing import Optional
import os
import numpy as np
# 替换原有的 import datetime
from datetime import datetime  # 导入datetime类，而非整个模块
import sys
import cv2
import pyrealsense2 as rs  # 确保已安装：pip install pyrealsense2
from loguru import logger  # 或使用你的日志库

class CameraService:
    def __init__(self, output_dir: str = "images"):
        self.output_dir = output_dir
        # 确保输出目录存在，同时检查权限
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            logger.info(f"相机服务初始化，输出目录: {self.output_dir}")
        except PermissionError:
            logger.error(f"无权限创建目录: {self.output_dir}")
            raise  # 初始化失败时终止

    def capture_stable_image(self, target_frame: int = 50) -> Optional[str]:
        """增强版：添加详细的文件保存检查逻辑"""
        pipeline = rs.pipeline()
        config = rs.config()

        # 配置相机流（保持原有逻辑）
        try:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            logger.info("已配置相机流: 640x480, BGR8, 30fps")
        except Exception as e:
            logger.warning(f"默认配置失败，尝试兼容模式: {str(e)}")
            config.enable_stream(rs.stream.color)

        try:
            pipeline_profile = pipeline.start(config)
            logger.info("相机启动成功")
            color_sensor = pipeline_profile.get_device().first_color_sensor()
            logger.info(f"使用相机: {color_sensor.get_info(rs.camera_info.name)}")

            frame_count = 0
            # 1. 生成保存路径（强制转为绝对路径，避免相对路径 confusion）
            save_path = os.path.join(self.output_dir, "captured_image2.jpg")
            absolute_save_path = os.path.abspath(save_path)  # 关键：绝对路径
            logger.info(f"计划保存路径（绝对路径）: {absolute_save_path}")

            # 2. 提前检查输出目录是否存在且可写（防患于未然）
            if not os.path.exists(self.output_dir):
                logger.error(f"输出目录不存在: {self.output_dir}（绝对路径: {os.path.abspath(self.output_dir)}）")
                return None
            if not os.access(self.output_dir, os.W_OK):
                logger.error(f"输出目录无写入权限: {self.output_dir}（用户: {os.getlogin()}）")
                return None

            color_image = None
            while frame_count < target_frame:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                if not frames:
                    logger.error("超时未获取到帧")
                    return None
                color_frame = frames.get_color_frame()
                if not color_frame:
                    logger.warning("未获取到彩色帧，继续等待...")
                    continue
                frame_count += 1
                if frame_count % 10 == 0:
                    logger.info(f"等待稳定帧: {frame_count}/{target_frame}")
                if frame_count == target_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    logger.info(f"已获取目标帧，尺寸: {color_image.shape}")

            if color_image is None:
                logger.error("未获取到有效图像帧")
                return None

            # 3. 保存图像（添加底层文件操作日志）
            logger.info(f"开始写入文件: {absolute_save_path}")
            save_success = cv2.imwrite(absolute_save_path, color_image)
            
            # 4. 强制刷新文件系统（避免缓存导致的"假存在"）
            if 'linux' in sys.platform:
                # Linux系统：刷新目录元数据
                import subprocess
                subprocess.run(['sync', absolute_save_path], check=True)
                logger.info("已强制刷新文件系统缓存")

            # 5. 详细检查保存结果
            if not save_success:
                logger.error(f"cv2.imwrite 明确返回失败（底层API调用失败）")
                # 检查磁盘空间
                disk_stats = shutil.disk_usage(os.path.dirname(absolute_save_path))
                if disk_stats.free < 1024 * 1024:  # 小于1MB
                    logger.error(f"磁盘空间不足，剩余: {disk_stats.free / 1024 / 1024:.2f}MB")
                return None

            # 6. 检查文件是否真的存在（用绝对路径）
            if not os.path.exists(absolute_save_path):
                logger.error(f"保存后文件不存在（绝对路径验证失败）: {absolute_save_path}")
                # 检查是否被其他进程立即删除（如清理程序）
                logger.info(f"检查目录下其他文件: {os.listdir(os.path.dirname(absolute_save_path))}")
                return None

            # 7. 检查文件属性（大小、修改时间）
            file_stats = os.stat(absolute_save_path)
            logger.info(f"文件属性验证:")
            logger.info(f"  路径: {absolute_save_path}")
            logger.info(f"  大小: {file_stats.st_size / 1024:.2f}KB")
            
            # 正确调用：datetime类的fromtimestamp方法
            logger.info(f"  修改时间: {datetime.fromtimestamp(file_stats.st_mtime).strftime('%Y-%m-%d %H:%M:%S')}")
            if file_stats.st_size < 1024:
                logger.error(f"文件过小（可能损坏），大小: {file_stats.st_size}字节")
                os.remove(absolute_save_path)
                return None

            # 8. 验证文件内容是否可读取（最终确认）
            try:
                test_read = cv2.imread(absolute_save_path)
                if test_read is None:
                    logger.error(f"文件存在但无法被OpenCV读取（内容损坏）")
                    os.remove(absolute_save_path)
                    return None
                logger.info(f"文件内容验证通过，读取尺寸: {test_read.shape}")
            except Exception as e:
                logger.error(f"读取文件内容时出错: {str(e)}")
                return None

            logger.info(f"✅ 图像保存完全成功: {absolute_save_path}")
            return absolute_save_path


        except Exception as e:  # 捕获pyrealsense2的所有异常（兼容版本差异）
            logger.error(f"RealSense相机错误: {str(e)}")
            return None

        except Exception as e:
            logger.error(f"拍照过程异常: {str(e)}", exc_info=True)
            return None
        finally:
            pipeline.stop()
            logger.info("相机已停止")