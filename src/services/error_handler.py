# services/error_handler.py
import logging
import psutil
import os
import time
from typing import Dict, Any, Callable, List
from enum import Enum
from dataclasses import dataclass
from .api_client import EnhancedCozeAPIClient
from .exceptions import AudioError, APIError
from src.hardware.audio_interface import RobotAudioInterface
from src.config import Config
from src.utils.paths import PathManager

logger = logging.getLogger(__name__)


class ErrorSeverity(Enum):
    """错误严重程度枚举"""
    INFO = 1
    WARNING = 2
    ERROR = 3
    CRITICAL = 4


class ErrorCategory(Enum):
    """错误类别枚举"""
    API = "api"
    AUDIO = "audio"
    SYSTEM = "system"
    NETWORK = "network"
    FILE = "file"
    UNKNOWN = "unknown"


@dataclass
class ErrorContext:
    """错误上下文数据类"""
    category: ErrorCategory
    severity: ErrorSeverity
    timestamp: float
    details: Dict[str, Any]
    original_error: Exception


class AdvancedErrorHandler:
    """增强版错误处理器"""

    ERROR_MAPPING = {
        400: ("参数错误，请检查输入内容", "client_error", ErrorSeverity.WARNING),
        401: ("身份验证失败", "auth_error", ErrorSeverity.ERROR),
        403: ("权限不足", "permission_error", ErrorSeverity.ERROR),
        404: ("请求资源不存在", "resource_error", ErrorSeverity.WARNING),
        500: ("服务暂时不可用，请稍后重试", "server_error", ErrorSeverity.ERROR),
        503: ("服务维护中", "maintenance_error", ErrorSeverity.ERROR),
    }

    def __init__(self):
        self.api_client = EnhancedCozeAPIClient(Config.BEARER_TOKEN)
        self.audio_interface = RobotAudioInterface()
        self.error_audio_dir = os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["error_dir"])
        self.error_history: List[ErrorContext] = []
        self.error_handlers: Dict[ErrorCategory, Callable] = {
            ErrorCategory.API: self.handle_api_error,
            ErrorCategory.AUDIO: self.handle_audio_error,
            ErrorCategory.SYSTEM: self.handle_system_error,
            ErrorCategory.NETWORK: self.handle_network_error,
            ErrorCategory.FILE: self.handle_file_error,
        }
        PathManager.create_dir(self.error_audio_dir)

    def handle_error(self, error: Exception, category: ErrorCategory = ErrorCategory.UNKNOWN) -> None:
        """
        统一的错误处理入口
        :param error: 异常对象
        :param category: 错误类别
        """
        try:
            # 创建错误上下文
            context = self._create_error_context(error, category)

            # 记录错误历史
            self.error_history.append(context)

            # 获取对应的错误处理器
            handler = self.error_handlers.get(category, self.handle_unknown_error)

            # 执行错误处理
            handler(context)

            # 检查是否需要紧急清理
            if context.severity == ErrorSeverity.CRITICAL:
                self._emergency_cleanup()

        except Exception as e:
            logger.critical(f"错误处理过程中发生异常: {str(e)}")
            self._emergency_cleanup()

    def _create_error_context(self, error: Exception, category: ErrorCategory) -> ErrorContext:
        """创建错误上下文"""
        severity = self._determine_error_severity(error)
        details = self._gather_error_details(error)

        return ErrorContext(
            category=category,
            severity=severity,
            timestamp=time.time(),
            details=details,
            original_error=error
        )

    def _determine_error_severity(self, error: Exception) -> ErrorSeverity:
        """确定错误严重程度"""
        if isinstance(error, APIError):
            _, _, severity = self.ERROR_MAPPING.get(error.status_code, ("", "", ErrorSeverity.ERROR))
            return severity
        elif isinstance(error, AudioError):
            return ErrorSeverity.ERROR
        elif isinstance(error, (OSError, IOError)):
            return ErrorSeverity.ERROR
        else:
            return ErrorSeverity.WARNING

    def _gather_error_details(self, error: Exception) -> Dict[str, Any]:
        """收集错误详细信息"""
        details = {
            "error_type": type(error).__name__,
            "error_message": str(error),
            "timestamp": time.time(),
            "system_info": {
                "cpu_percent": psutil.cpu_percent(),
                "memory_percent": psutil.virtual_memory().percent,
                "disk_usage": psutil.disk_usage('/').percent
            }
        }

        if isinstance(error, APIError):
            details["status_code"] = error.status_code
        elif isinstance(error, AudioError):
            details["device_info"] = self._get_audio_device_info()

        return details

    def _get_audio_device_info(self) -> Dict[str, Any]:
        """获取音频设备信息"""
        try:
            return {
                "device_count": self.audio_interface.audio.get_device_count(),
                "default_device": self.audio_interface.audio.get_default_device_info()
            }
        except Exception:
            return {"error": "无法获取音频设备信息"}

    def handle_api_error(self, context: ErrorContext) -> None:
        """处理API相关错误"""
        error = context.original_error
        if isinstance(error, APIError):
            status_code = error.status_code
            prompt, error_type, _ = self.ERROR_MAPPING.get(
                status_code,
                ("系统繁忙，请稍后再试", "unknown_error", ErrorSeverity.ERROR)
            )

            logger.error(f"API错误 [{status_code}]: {error_type} - {str(error)}")

            # 播放错误提示音频
            self._play_error_audio(prompt)

            # 记录详细日志
            self._log_error_details(context)

    def handle_audio_error(self, context: ErrorContext) -> None:
        """处理音频相关错误"""
        error = context.original_error
        logger.error(f"音频错误: {str(error)}")

        # 尝试恢复音频设备
        try:
            self.audio_interface.__del__()
            self.audio_interface = RobotAudioInterface()
        except Exception as e:
            logger.error(f"音频设备恢复失败: {str(e)}")

        # 播放错误提示
        self._play_error_audio("音频设备出现异常，请检查设备连接")

    def handle_system_error(self, context: ErrorContext) -> None:
        """处理系统相关错误"""
        error = context.original_error
        logger.error(f"系统错误: {str(error)}")

        # 检查系统资源
        self._check_system_resources()

        # 播放错误提示
        self._play_error_audio("系统出现异常，请稍后重试")

    def handle_network_error(self, context: ErrorContext) -> None:
        """处理网络相关错误"""
        error = context.original_error
        logger.error(f"网络错误: {str(error)}")

        # 检查网络连接
        self._check_network_connection()

        # 播放错误提示
        self._play_error_audio("网络连接异常，请检查网络设置")

    def handle_file_error(self, context: ErrorContext) -> None:
        """处理文件相关错误"""
        error = context.original_error
        logger.error(f"文件错误: {str(error)}")

        # 检查文件系统
        self._check_file_system()

        # 播放错误提示
        self._play_error_audio("文件系统出现异常，请检查存储空间")

    def handle_unknown_error(self, context: ErrorContext) -> None:
        """处理未知错误"""
        error = context.original_error
        logger.error(f"未知错误: {str(error)}")

        # 播放通用错误提示
        self._play_error_audio("系统出现异常，请稍后重试")

    def _check_system_resources(self) -> None:
        """检查系统资源使用情况"""
        try:
            cpu_percent = psutil.cpu_percent()
            memory_percent = psutil.virtual_memory().percent
            disk_usage = psutil.disk_usage('/').percent

            if cpu_percent > 90 or memory_percent > 90 or disk_usage > 90:
                logger.warning("系统资源使用率过高")
        except Exception as e:
            logger.error(f"系统资源检查失败: {str(e)}")

    def _check_network_connection(self) -> None:
        """检查网络连接状态"""
        try:
            # 尝试连接Coze API
            response = self.api_client._request(
                "GET",
                "health",
                timeout=5
            )
            if not response:
                logger.warning("网络连接检查失败：无法获取健康检查响应")
        except Exception as e:
            logger.error(f"网络连接检查失败: {str(e)}")
            # 可以在这里添加重试逻辑或其他网络恢复措施

    def _check_file_system(self) -> None:
        """检查文件系统状态"""
        try:
            # 检查输出目录
            if not os.path.exists(Config.OUTPUT_DIR):
                PathManager.create_dir(Config.OUTPUT_DIR)

            # 检查录音目录
            if not os.path.exists(Config.RECORD_DIR):
                PathManager.create_dir(Config.RECORD_DIR)
        except Exception as e:
            logger.error(f"文件系统检查失败: {str(e)}")

    def _play_error_audio(self, message: str) -> None:
        """播放错误提示音频"""
        try:
            error_audio_path = os.path.join(self.error_audio_dir, Config.AUDIO_NAMES["error_mp3"])
            if os.path.exists(error_audio_path):
                self.audio_interface.play_audio(error_audio_path)
            else:
                logger.warning("错误提示音频文件不存在")
        except Exception as e:
            logger.error(f"播放错误提示音频失败: {str(e)}")

    def _log_error_details(self, context: ErrorContext) -> None:
        """记录错误详细信息"""
        logger.error(f"错误详情: {context.details}")

    def _emergency_cleanup(self) -> None:
        """紧急资源释放"""
        logger.info("执行紧急清理...")
        try:
            # 释放音频接口资源
            self.audio_interface.__del__()

            # 清理临时文件
            self._cleanup_temp_files()

            # 重置错误历史
            self.error_history.clear()

            logger.info("紧急清理完成")
        except Exception as e:
            logger.critical(f"紧急清理过程中发生异常: {str(e)}")

    def _cleanup_temp_files(self) -> None:
        """清理临时文件"""
        try:
            temp_dirs = [Config.OUTPUT_DIR, Config.RECORD_DIR]
            for temp_dir in temp_dirs:
                if os.path.exists(temp_dir):
                    for file in os.listdir(temp_dir):
                        if file.endswith('.tmp'):
                            try:
                                os.remove(os.path.join(temp_dir, file))
                            except Exception as e:
                                logger.warning(f"清理临时文件失败: {str(e)}")
        except Exception as e:
            logger.error(f"清理临时文件目录失败: {str(e)}")

    def get_error_history(self) -> List[ErrorContext]:
        """获取错误历史"""
        return self.error_history.copy()

    def clear_error_history(self) -> None:
        """清除错误历史"""
        self.error_history.clear()