import os
import logging

logger = logging.getLogger(__name__)


class PathManager:
    """安全的路径处理工具类"""

    @staticmethod
    def safe_join(base, *paths):
        """安全的路径拼接"""
        try:
            full_path = os.path.join(base, *paths)
            # 防止路径遍历攻击
            if os.path.abspath(full_path).startswith(os.path.abspath(base)):
                return full_path
            raise ValueError("Invalid path traversal attempt")
        except Exception as e:
            logger.error(f"Path join error: {str(e)}")
            return base

    @staticmethod
    def create_dir(path):
        """安全的目录创建"""
        try:
            os.makedirs(path, exist_ok=True)
            return True
        except OSError as e:
            logger.error(f"Directory creation failed: {str(e)}")
            return False