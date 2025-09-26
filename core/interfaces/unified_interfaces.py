# core/interfaces/unified_interfaces.py
"""统一的服务接口定义"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
from enum import Enum


class ErrorCategory(Enum):
    """错误类别枚举"""
    API = "api"
    AUDIO = "audio"
    SYSTEM = "system"
    NETWORK = "network"
    FILE = "file"
    UNKNOWN = "unknown"


# ============= 音频相关接口 =============

class IAudioDevice(ABC):
    """音频设备接口"""
    
    @abstractmethod
    def detect_wake_word(self) -> bool:
        """检测唤醒词"""
        pass
    
    @abstractmethod
    def detect_bye_word(self) -> bool:
        """检测结束词"""
        pass
    
    @abstractmethod
    def record_audio(self, duration: int = None) -> str:
        """录制音频"""
        pass
    
    @abstractmethod
    def play_audio(self, audio_path: str) -> bool:
        """播放音频"""
        pass
    
    @abstractmethod
    def is_playing(self) -> bool:
        """检查是否正在播放"""
        pass
    
    @abstractmethod
    def stop_playing(self) -> None:
        """停止播放"""
        pass


class IAudioService(ABC):
    """音频服务接口"""
    
    @abstractmethod
    def start_conversation(self) -> None:
        """开始对话流程"""
        pass
    
    @abstractmethod
    def end_conversation(self) -> None:
        """结束对话流程"""
        pass
    
    @abstractmethod
    def is_in_conversation(self) -> bool:
        """检查是否在对话流程中"""
        pass
    
    @abstractmethod
    def is_playing(self) -> bool:
        """检查当前是否有音频正在播放"""
        pass
    
    @abstractmethod
    def play_wait_audio(self) -> None:
        """播放等待音频"""
        pass
    
    @abstractmethod
    def play_hello_audio(self) -> None:
        """播放问候音频"""
        pass
    
    @abstractmethod
    def play_bye_audio(self) -> None:
        """播放告别音频"""
        pass
    
    @abstractmethod
    def record_and_transcribe(self) -> Optional[str]:
        """录音并转录"""
        pass
    
    @abstractmethod
    def play_result_audio(self, audio_path: str) -> bool:
        """播放结果音频"""
        pass
    
    @abstractmethod
    def play_reserved_audio(self, audio_number: int) -> None:
        """播放预留音频"""
        pass
    
    @abstractmethod
    def stop_audio(self) -> None:
        """停止音频"""
        pass


# ============= API相关接口 =============

class IAPIClient(ABC):
    """API客户端接口"""
    
    @abstractmethod
    def send_chat_request(self, bot_id: str, user_id: str, query: str) -> Optional[Dict[str, Any]]:
        """发送聊天请求"""
        pass
    
    @abstractmethod
    def get_chat_status(self, conversation_id: str, chat_id: str) -> Optional[Dict[str, Any]]:
        """获取聊天状态"""
        pass
    
    @abstractmethod
    def get_chat_messages(self, conversation_id: str, chat_id: str) -> Optional[Dict[str, Any]]:
        """获取聊天消息"""
        pass
    
    @abstractmethod
    def text_to_speech(self, text: str, voice_id: str = None) -> Optional[str]:
        """文本转语音"""
        pass
    
    @abstractmethod
    def speech_to_text(self, audio_path: str) -> Optional[str]:
        """语音转文本"""
        pass
    
    @abstractmethod
    def upload_to_lower(self, file_path: str) -> Optional[str]:
        """上传文件"""
        pass
    
    @abstractmethod
    def create_streaming_chat(self, bot_id: str, user_id: str, query: str) -> Optional[Any]:
        """创建流式聊天"""
        pass
    
    @abstractmethod
    def get_streaming_response(self, stream_id: str) -> Optional[Any]:
        """获取流式响应"""
        pass


# ============= 对话相关接口 =============

class IConversationManager(ABC):
    """对话管理接口"""
    
    @abstractmethod
    def start_conversation(self) -> None:
        """开始对话"""
        pass


class IWakeWordDetector(ABC):
    """唤醒词检测接口"""
    
    @abstractmethod
    def detect_wake_word(self) -> bool:
        """检测唤醒词"""
        pass


class IChatProcessor(ABC):
    """聊天处理接口"""
    
    @abstractmethod
    def process_query(self, query: str) -> Optional[str]:
        """处理用户查询"""
        pass


class IStreamingProcessor(ABC):
    """流式处理接口"""
    
    @abstractmethod
    def process(self, transcript: str) -> bool:
        """处理流式对话"""
        pass


# ============= 错误处理接口 =============

class IErrorHandler(ABC):
    """错误处理接口"""
    
    @abstractmethod
    def handle_error(self, error: Exception, category: ErrorCategory = ErrorCategory.UNKNOWN) -> None:
        """处理错误"""
        pass
    
    @abstractmethod
    def handle_api_error(self, context: Any) -> None:
        """处理API错误"""
        pass
    
    @abstractmethod
    def handle_audio_error(self, context: Any) -> None:
        """处理音频错误"""
        pass
    
    @abstractmethod
    def handle_system_error(self, context: Any) -> None:
        """处理系统错误"""
        pass
    
    @abstractmethod
    def handle_network_error(self, context: Any) -> None:
        """处理网络错误"""
        pass
    
    @abstractmethod
    def handle_file_error(self, context: Any) -> None:
        """处理文件错误"""
        pass
    
    @abstractmethod
    def handle_unknown_error(self, context: Any) -> None:
        """处理未知错误"""
        pass
    
    @abstractmethod
    def get_error_history(self) -> List[Any]:
        """获取错误历史"""
        pass
    
    @abstractmethod
    def clear_error_history(self) -> None:
        """清除错误历史"""
        pass