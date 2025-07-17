# services/scheduler.py
import logging
from apscheduler.schedulers.background import BackgroundScheduler
from config import Config

logger = logging.getLogger(__name__)


class CleanupScheduler:
    """清理任务调度器"""

    def __init__(self):
        self.scheduler = BackgroundScheduler()
        self._configure()

    def _configure(self):
        """配置定时任务"""
        self.scheduler.add_job(
            self._cleanup_task,
            'interval',
            minutes=10,  # 每10分钟执行一次
            max_instances=1
        )

    def start(self):
        """启动调度器"""
        if not self.scheduler.running:
            self.scheduler.start()
            logger.info("文件清理调度器已启动")

    def _cleanup_task(self):
        """安全清理任务"""
        from utils.cleaner import FileCleaner

        FileCleaner.clean_old_files(
            directories=[Config.OUTPUT_DIR, Config.RECORD_DIR],
            extensions=['.wav', '.mp3'],
            max_age_seconds=Config.CLEANUP_CONFIG['retention_minutes'] * 60,
            min_retain=Config.CLEANUP_CONFIG['min_retain_files']
        )
