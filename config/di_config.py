"""依赖注入配置模块"""

from typing import Dict, Any
from .environment import Environment


class DIConfig:
    """依赖注入配置类"""
    
    def __init__(self):
        self.environment = Environment.get_current()
    
    def get_service_implementations(self) -> Dict[str, str]:
        """获取服务实现映射"""
        if self.environment == Environment.TESTING:
            return {
                'IAudioDevice': 'MockAudioDevice',
                'IAudioService': 'MockAudioService',
                'IAPIClient': 'MockAPIClient',
                'IConversationManager': 'MockConversationManager',
                'IWakeWordDetector': 'MockWakeWordDetector',
                'IChatProcessor': 'MockChatProcessor',
                'IStreamingProcessor': 'MockStreamingProcessor',
                'IErrorHandler': 'MockErrorHandler'
            }
        else:
            return {
                'IAudioDevice': 'RobotAudioInterface',
                'IAudioService': 'AudioService',
                'IAPIClient': 'EnhancedCozeAPIClient',
                'IConversationManager': 'ConversationManager',
                'IWakeWordDetector': 'WakeWordDetector',
                'IChatProcessor': 'ChatProcessor',
                'IStreamingProcessor': 'StreamingProcessor',
                'IErrorHandler': 'ErrorHandler'
            }
    
    def get_service_implementation(self, interface_name: str) -> str:
        """获取单个服务实现"""
        implementations = self.get_service_implementations()
        return implementations.get(interface_name)
    
    def should_use_mock_services(self) -> bool:
        """是否应该使用模拟服务"""
        return self.environment == Environment.TESTING
    
    def should_enable_di_logging(self) -> bool:
        """是否启用依赖注入日志记录"""
        return self.environment == Environment.DEVELOPMENT
    
    def get_service_config(self, service_name: str) -> Dict[str, Any]:
        """获取服务配置"""
        configs = {
            'audio_service': {
                'buffer_size': 1024,
                'sample_rate': 16000,
                'channels': 1
            },
            'api_client': {
                'timeout': 30,
                'max_retries': 3,
                'base_url': 'https://api.coze.cn'
            },
            'conversation': {
                'max_history': 10,
                'context_window': 4000
            }
        }
        return configs.get(service_name, {})