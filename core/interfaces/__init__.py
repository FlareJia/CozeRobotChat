# core/interfaces/__init__.py
"""统一的接口导出"""

from .unified_interfaces import (
    # 音频相关接口
    IAudioDevice,
    IAudioService,
    
    # API相关接口
    IAPIClient,
    
    # 对话相关接口
    IConversationManager,
    IWakeWordDetector,
    IChatProcessor,
    IStreamingProcessor,
    
    # 错误处理接口
    IErrorHandler,
    ErrorCategory,
)

__all__ = [
    'IAudioDevice',
    'IAudioService',
    'IAPIClient',
    'IConversationManager',
    'IWakeWordDetector',
    'IChatProcessor',
    'IStreamingProcessor',
    'IErrorHandler',
    'ErrorCategory',
]
"""服务接口定义模块"""