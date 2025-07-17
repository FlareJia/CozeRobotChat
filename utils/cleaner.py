# utils/cleaner.py
import os
import time
import logging
import platform
from typing import List

try:
    import fcntl  # Unix系统专用
except ImportError:
    fcntl = None  # Windows系统不需要

logger = logging.getLogger(__name__)

class FileCleaner:
    """增强版文件清理工具类"""

    @staticmethod
    def clean_old_files(
        directories: List[str],
        max_age_seconds: int = 3600,
        extensions: List[str] = None,
        min_retain: int = 5  # 新增参数：至少保留最新N个文件
    ) -> int:
        """
        安全清理旧文件方法
        """
        deleted_count = 0
        current_time = time.time()
        safe_interval = 300  # 增加5分钟安全缓冲

        for dir_path in directories:
            if not os.path.exists(dir_path):
                logger.warning(f"目录不存在: {dir_path}")
                continue

            # 获取目录下所有文件并排序
            try:
                files = sorted(
                    [entry for entry in os.scandir(dir_path) if entry.is_file()],
                    key=lambda x: x.stat().st_mtime,
                    reverse=True
                )
            except Exception as e:
                logger.error(f"扫描目录失败 {dir_path}: {str(e)}")
                continue

            # 保留最新文件
            retain_list = files[:min_retain]

            for entry in files[min_retain:]:
                try:
                    # 跳过需要保留的文件
                    if entry in retain_list:
                        continue

                    # 扩展名过滤
                    if extensions and not entry.name.lower().endswith(tuple(ext.lower() for ext in extensions)):
                        continue

                    # 计算文件年龄（增加安全缓冲）
                    file_age = current_time - entry.stat().st_mtime
                    if file_age < (max_age_seconds + safe_interval):
                        continue

                    # 检查文件是否被占用
                    if FileCleaner._is_file_in_use(entry.path):
                        logger.debug(f"文件被占用，跳过清理: {entry.path}")
                        continue

                    # 执行删除
                    os.remove(entry.path)
                    deleted_count += 1
                    logger.info(f"安全删除旧文件: {entry.path} (年龄: {file_age//60}分钟)")

                except Exception as e:
                    logger.error(f"清理文件失败 {entry.path}: {str(e)}")

        logger.info(f"完成安全清理，共删除 {deleted_count} 个旧文件")
        return deleted_count

    @staticmethod
    def _is_file_in_use(file_path: str) -> bool:
        """跨平台文件占用状态检查"""
        if not os.path.exists(file_path):
            return False

        try:
            # Unix系统使用fcntl检查
            if fcntl:
                with open(file_path, 'a') as f:
                    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    fcntl.flock(f, fcntl.LOCK_UN)
                return False
            # Windows系统检查
            elif platform.system() == 'Windows':
                try:
                    os.rename(file_path, file_path)
                    return False
                except OSError:
                    return True
            # 其他系统
            else:
                return False
        except (BlockingIOError, PermissionError, OSError):
            return True
        except Exception as e:
            logger.warning(f"文件占用检查异常 {file_path}: {str(e)}")
            return False  # 未知错误时保守处理