import os
import logging
from typing import List, Optional
from pathlib import Path
import datetime
from filelock import FileLock, Timeout
from utils.paths import PathManager
from config import Config

logger = logging.getLogger(__name__)


class AudioFileManager:
    """线程安全的音频文件管理器"""

    def __init__(self, output_dir: Optional[str] = None):
        self.config = Config()
        self.output_dir = output_dir or self.config.OUTPUT_DIR
        self._managed_files: List[str] = []
        self.lock = FileLock(os.path.join(self.output_dir, ".audio_manager.lock"))

        # 确保目录存在
        if not PathManager.create_dir(self.output_dir):
            raise RuntimeError(f"无法创建输出目录: {self.output_dir}")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()

    def register_file(self, file_path: str) -> bool:
        """线程安全的文件注册"""
        try:
            with self.lock.acquire(timeout=10):  # 10秒超时
                resolved_path = str(Path(file_path).resolve())
                if resolved_path not in self._managed_files:
                    self._managed_files.append(resolved_path)
                    logger.debug(f"注册文件: {resolved_path}")
                return True
        except Timeout:
            logger.error("获取文件锁超时，注册失败")
            return False

    def generate_filepath(self, prefix: str = "audio", extension: str = "mp3") -> str:
        """生成安全的音频文件路径"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{prefix}_{timestamp}.{extension}"
        return PathManager.safe_join(self.output_dir, filename)

    def cleanup(self) -> None:
        """线程安全的清理操作"""
        try:
            with self.lock.acquire(timeout=30):
                for file_path in self._managed_files.copy():
                    try:
                        if os.path.exists(file_path):
                            os.remove(file_path)
                            logger.info(f"安全删除: {file_path}")
                            self._managed_files.remove(file_path)
                    except Exception as e:
                        logger.error(f"删除失败 {file_path}: {str(e)}")
        except Timeout:
            logger.critical("清理操作获取锁超时，可能存在死锁")

    def save_audio(self, content: bytes, **naming_kwargs) -> Optional[str]:
        """保存音频内容到文件"""
        try:
            file_path = self.generate_filepath(**naming_kwargs)
            with open(file_path, "wb") as f:
                f.write(content)
            self.register_file(file_path)
            return file_path
        except IOError as e:
            logger.error(f"音频保存失败: {str(e)}")
            return None