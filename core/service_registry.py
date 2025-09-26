# core/service_registry.py
import logging
import importlib
from typing import Type

from core.di_container import DIContainer
from config import DIConfig, Environment
from core.interfaces.unified_interfaces import (
    IAudioDevice, IAudioService, IAPIClient, IConversationManager,
    IWakeWordDetector, IChatProcessor, IStreamingProcessor, IErrorHandler
)
from services.api_client import EnhancedCozeAPIClient
from services.audio_service import AudioService
from services.error_handler import AdvancedErrorHandler
from services.chat_processor import ChatProcessor
from core.conversation_manager import ConversationManager
from core.streaming_processor import StreamingProcessor
from core.wake_word_detector import WakeWordDetector
from hardware.audio_interface import RobotAudioInterface
from utils.paths import PathManager
from config import Config
from managers.audio_manager import AudioFileManager
from utils.backoff import BackoffManager
from tests.mocks import (
    MockAudioDevice, MockAPIClient, MockAudioService, MockErrorHandler,
    MockChatProcessor, MockStreamingProcessor, MockWakeWordDetector, MockConversationManager
)

logger = logging.getLogger(__name__)


class ServiceRegistry:
    """服务注册器，负责向DI容器注册所有服务"""
    
    def __init__(self, container: DIContainer, config: DIConfig = None):
        self.container = container
        self.config = config or DIConfig()
        self.logger = logging.getLogger(__name__)
        
        # 服务实现映射
        self.service_implementations = {
            'EnhancedCozeAPIClient': EnhancedCozeAPIClient,
            'AudioService': AudioService,
            'AdvancedErrorHandler': AdvancedErrorHandler,
            'ChatProcessor': ChatProcessor,
            'ConversationManager': ConversationManager,
            'StreamingProcessor': StreamingProcessor,
            'WakeWordDetector': WakeWordDetector,
            'RobotAudioInterface': RobotAudioInterface,
            'PathManager': PathManager,
            'Config': Config,
            'AudioFileManager': AudioFileManager,
            'BackoffManager': BackoffManager,
            # Mock implementations for testing
            'MockAudioDevice': MockAudioDevice,
            'MockAPIClient': MockAPIClient,
            'MockAudioService': MockAudioService,
            'MockErrorHandler': MockErrorHandler,
            'MockChatProcessor': MockChatProcessor,
            'MockStreamingProcessor': MockStreamingProcessor,
            'MockWakeWordDetector': MockWakeWordDetector,
            'MockConversationManager': MockConversationManager
        }
    
    def register_all_services(self) -> None:
        """注册所有服务到DI容器"""
        try:
            if self.config.should_use_mock_services():
                self._register_mock_services()
            else:
                logger.info("开始注册所有服务...")
                
                # 注册基础服务（单例）
                self._register_infrastructure_services()
                
                # 注册业务服务
                self._register_business_services()
                
                # 注册核心服务
                self._register_core_services()
                
            self.logger.info("所有服务注册完成")
        except Exception as e:
            self.logger.error(f"服务注册失败: {e}")
            raise
    
    def _register_mock_services(self):
        """注册模拟服务（用于测试）"""
        self.logger.info("注册模拟服务")
        # 这里可以注册模拟实现
        # 暂时使用真实实现作为占位符
        self._register_infrastructure_services()
        self._register_business_services()
        self._register_core_services()
    
    def _get_service_implementation(self, interface_name: str):
        """根据配置获取服务实现"""
        impl_name = self.config.get_service_implementation(interface_name)
        logger.debug(f"获取服务实现: {interface_name} -> {impl_name}")
        
        if impl_name and impl_name in self.service_implementations:
            result = self.service_implementations[impl_name]
            logger.debug(f"从service_implementations找到: {result}")
            return result
        
        # 回退到默认实现
        default_implementations = {
            'IAPIClient': EnhancedCozeAPIClient,
            'IAudioService': AudioService,
            'IErrorHandler': AdvancedErrorHandler,
            'IAudioDevice': RobotAudioInterface,
            'IChatProcessor': ChatProcessor,
            'IStreamingProcessor': StreamingProcessor,
            'IWakeWordDetector': WakeWordDetector,
            'IConversationManager': ConversationManager
        }
        result = default_implementations.get(interface_name)
        logger.debug(f"使用默认实现: {interface_name} -> {result}")
        return result
    
    def _register_infrastructure_services(self) -> None:
        """注册基础设施服务"""
        logger.info("注册基础设施服务...")
        
        # 音频文件管理器（单例）- 使用工厂方法避免参数问题
        self.container.register_singleton(
            AudioFileManager,
            factory=lambda: AudioFileManager()
        )
        
        # 音频设备接口（单例）
        audio_device_impl = self._get_service_implementation('IAudioDevice')
        logger.debug(f"IAudioDevice实现类: {audio_device_impl}")
        if audio_device_impl:
            logger.info(f"注册IAudioDevice服务: {audio_device_impl}")
            self.container.register_singleton(
                IAudioDevice,
                audio_device_impl
            )
        else:
            logger.error("IAudioDevice实现类为None，跳过注册")
        
        # 为了向后兼容，也注册具体类型
        self.container.register_singleton(
            RobotAudioInterface,
            RobotAudioInterface
        )
        
        logger.info("基础设施服务注册完成")
    
    def _register_business_services(self) -> None:
        """注册业务服务"""
        logger.info("注册业务服务...")
        
        # API客户端（单例）- 使用工厂方法提供bearer_token
        from config import Config
        api_client_impl = self._get_service_implementation('IAPIClient')
        if api_client_impl:
            self.container.register_singleton(
                IAPIClient,
                factory=lambda: EnhancedCozeAPIClient(Config.BEARER_TOKEN)
            )
        
        # 为了向后兼容，也注册具体类型
        self.container.register_singleton(
            EnhancedCozeAPIClient,
            factory=lambda: EnhancedCozeAPIClient(Config.BEARER_TOKEN)
        )
        
        # 错误处理器（单例）
        error_handler_impl = self._get_service_implementation('IErrorHandler')
        if error_handler_impl:
            self.container.register_singleton(
                IErrorHandler,
                error_handler_impl
            )
        
        # 为了向后兼容，也注册具体类型
        self.container.register_singleton(
            AdvancedErrorHandler,
            AdvancedErrorHandler
        )
        
        # 音频服务（单例）
        audio_service_impl = self._get_service_implementation('IAudioService')
        if audio_service_impl:
            self.container.register_singleton(
                IAudioService,
                audio_service_impl
            )
        
        # 为了向后兼容，也注册具体类型
        self.container.register_singleton(
            AudioService,
            AudioService
        )
        
        logger.info("业务服务注册完成")
    
    def _register_core_services(self) -> None:
        """注册核心服务"""
        logger.info("注册核心服务...")
        
        # 聊天处理器（瞬态）
        chat_processor_impl = self._get_service_implementation('IChatProcessor')
        if chat_processor_impl:
            self.container.register_transient(
                IChatProcessor,
                chat_processor_impl
            )
        
        # 为了向后兼容，也注册具体类型
        self.container.register_transient(
            ChatProcessor,
            ChatProcessor
        )
        
        # 流式处理器（瞬态）
        streaming_processor_impl = self._get_service_implementation('IStreamingProcessor')
        if streaming_processor_impl:
            self.container.register_transient(
                IStreamingProcessor,
                streaming_processor_impl
            )
        
        # 为了向后兼容，也注册具体类型
        self.container.register_transient(
            StreamingProcessor,
            StreamingProcessor
        )
        
        # 唤醒词检测器（单例）
        wake_word_detector_impl = self._get_service_implementation('IWakeWordDetector')
        if wake_word_detector_impl:
            self.container.register_singleton(
                IWakeWordDetector,
                wake_word_detector_impl
            )
        
        # 为了向后兼容，也注册具体类型
        self.container.register_singleton(
            WakeWordDetector,
            WakeWordDetector
        )
        
        # 对话管理器（单例）
        conversation_manager_impl = self._get_service_implementation('IConversationManager')
        if conversation_manager_impl:
            self.container.register_singleton(
                IConversationManager,
                conversation_manager_impl
            )
        
        # 为了向后兼容，也注册具体类型
        self.container.register_singleton(
            ConversationManager,
            ConversationManager
        )
        
        logger.info("核心服务注册完成")
    
    def register_test_services(self) -> None:
        """注册测试服务（用于单元测试）"""
        logger.info("注册测试服务...")
        # 这里可以注册模拟服务用于测试
        pass
    
    def get_service_info(self) -> dict:
        """获取服务注册信息"""
        return self.container.get_registered_services()