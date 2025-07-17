import logging
import threading
from typing import Dict, Any
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum
from datetime import datetime

from hardware.audio_interface import RobotAudioInterface
from services.api_client import EnhancedCozeAPIClient
from services.audio_manager import AudioFileManager
from services.error_handler import AdvancedErrorHandler
from config import Config


# 创建一个简单的日志处理器，避免在垃圾回收时的问题
class SafeLogger:
    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def info(self, msg):
        try:
            self._logger.info(msg)
        except Exception:
            pass  # 忽略日志记录错误

    def error(self, msg):
        try:
            self._logger.error(msg)
        except Exception:
            pass  # 忽略日志记录错误


logger = SafeLogger()


class ResourceType(Enum):
    """资源类型枚举"""
    AUDIO_DEVICE = "audio_device"
    API_CLIENT = "api_client"
    AUDIO_MANAGER = "audio_manager"
    ERROR_HANDLER = "error_handler"


@dataclass
class ResourceInfo:
    """资源信息数据类"""
    type: ResourceType
    instance: Any
    created_at: datetime
    last_used: datetime
    is_active: bool


class ResourceManager:
    """统一的资源管理器"""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
            return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):
            self.resources: Dict[ResourceType, ResourceInfo] = {}
            self.initialized = True

    @contextmanager
    def manage_resource(self, resource_type: ResourceType, **kwargs):
        """
        资源管理的上下文管理器
        :param resource_type: 资源类型
        :param kwargs: 资源初始化参数
        """
        try:
            resource = self._get_or_create_resource(resource_type, **kwargs)
            yield resource
        finally:
            self._cleanup_resource(resource_type)

    def _get_or_create_resource(self, resource_type: ResourceType, **kwargs) -> Any:
        """
        获取或创建资源
        :param resource_type: 资源类型
        :param kwargs: 资源初始化参数
        :return: 资源实例
        """
        if resource_type in self.resources:
            resource_info = self.resources[resource_type]
            if resource_info.is_active:
                resource_info.last_used = datetime.now()
                return resource_info.instance
            else:
                # 如果资源存在但未激活，重新初始化
                self._cleanup_resource(resource_type)

        # 创建新资源
        instance = self._create_resource(resource_type, **kwargs)
        self.resources[resource_type] = ResourceInfo(
            type=resource_type,
            instance=instance,
            created_at=datetime.now(),
            last_used=datetime.now(),
            is_active=True
        )
        return instance

    def _create_resource(self, resource_type: ResourceType, **kwargs) -> Any:
        """
        创建新资源
        :param resource_type: 资源类型
        :param kwargs: 资源初始化参数
        :return: 资源实例
        """
        if resource_type == ResourceType.AUDIO_DEVICE:
            return RobotAudioInterface()
        elif resource_type == ResourceType.API_CLIENT:
            return EnhancedCozeAPIClient(Config.BEARER_TOKEN)
        elif resource_type == ResourceType.AUDIO_MANAGER:
            return AudioFileManager(Config.OUTPUT_DIR)
        elif resource_type == ResourceType.ERROR_HANDLER:
            return AdvancedErrorHandler()
        else:
            raise ValueError(f"未知的资源类型: {resource_type}")

    def _cleanup_resource(self, resource_type: ResourceType) -> None:
        """
        清理资源
        :param resource_type: 资源类型
        """
        if resource_type in self.resources:
            resource_info = self.resources[resource_type]
            try:
                if hasattr(resource_info.instance, '__del__'):
                    resource_info.instance.__del__()
                resource_info.is_active = False
                logger.info(f"资源已清理: {resource_type.value}")
            except Exception as e:
                logger.error(f"清理资源失败 {resource_type.value}: {str(e)}")

    def cleanup_all(self) -> None:
        """清理所有资源"""
        try:
            for resource_type in list(self.resources.keys()):
                self._cleanup_resource(resource_type)
            self.resources.clear()
            logger.info("所有资源已清理")
        except Exception as e:
            logger.error(f"清理所有资源时发生错误: {str(e)}")

    def get_resource_status(self) -> Dict[str, Any]:
        """
        获取所有资源状态
        :return: 资源状态字典
        """
        return {
            resource_type.value: {
                "created_at": info.created_at.isoformat(),
                "last_used": info.last_used.isoformat(),
                "is_active": info.is_active
            }
            for resource_type, info in self.resources.items()
        }

    def __del__(self):
        """析构函数，确保清理所有资源"""
        try:
            self.cleanup_all()
        except Exception:
            pass  # 忽略清理过程中的错误