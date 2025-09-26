# tests/mocks.py
"""测试用的Mock类定义"""

from unittest.mock import Mock
from typing import Any, Dict, Optional
from core.interfaces.unified_interfaces import (
    IAudioDevice, IAudioService, IAPIClient, IConversationManager,
    IWakeWordDetector, IChatProcessor, IStreamingProcessor, IErrorHandler
)


class MockAudioDevice(IAudioDevice):
    """模拟音频设备"""
    
    def __init__(self):
        self.is_recording = False
        self.is_playing_audio = False
        self.recorded_audio_data = b"mock_audio_data"
    
    def detect_wake_word(self) -> bool:
        """检测唤醒词"""
        return True
    
    def detect_bye_word(self) -> bool:
        """检测结束词"""
        return False
    
    def record_audio(self, duration: int = None) -> str:
        """录制音频"""
        self.is_recording = True
        return "mock_audio_path.wav"
    
    def play_audio(self, audio_path: str) -> bool:
        """播放音频"""
        self.is_playing_audio = True
        return True
    
    def is_playing(self) -> bool:
        """检查是否正在播放"""
        return self.is_playing_audio
    
    def stop_playing(self) -> None:
        """停止播放"""
        self.is_playing_audio = False


class MockAPIClient(IAPIClient):
    """模拟API客户端"""
    
    def __init__(self):
        self.connected = True
        self.response_data = {"message": "mock response"}
    
    def send_request(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """发送请求"""
        return self.response_data
    
    def get_response(self, request_id: str) -> Dict[str, Any]:
        """获取响应"""
        return {"request_id": request_id, "response": "mock response"}


class MockAudioService(IAudioService):
    """模拟音频服务"""
    
    def __init__(self):
        self.is_processing = False
        self.in_conversation = False
        self.playing = False
    
    def start_conversation(self) -> None:
        """开始对话"""
        self.in_conversation = True
    
    def end_conversation(self) -> None:
        """结束对话"""
        self.in_conversation = False
    
    def is_in_conversation(self) -> bool:
        """是否在对话中"""
        return self.in_conversation
    
    def is_playing(self) -> bool:
        """是否正在播放"""
        return self.playing
    
    def play_wait_audio(self) -> None:
        """播放等待音频"""
        self.playing = True
    
    def play_hello_audio(self) -> None:
        """播放问候音频"""
        self.playing = True
    
    def play_bye_audio(self) -> None:
        """播放再见音频"""
        self.playing = True
    
    def record_and_transcribe(self) -> Optional[str]:
        """录音并转录"""
        return "mock transcription"
    
    def play_result_audio(self, audio_path: str) -> bool:
        """播放结果音频"""
        self.playing = True
        return True
    
    def play_reserved_audio(self, audio_number: int) -> None:
        """播放预留音频"""
        self.playing = True
    
    def stop_audio(self) -> None:
        """停止音频"""
        self.playing = False


class MockErrorHandler(IErrorHandler):
    """模拟错误处理器"""
    
    def __init__(self):
        self.handled_errors = []
    
    def handle_error(self, error: Exception) -> None:
        """处理错误"""
        self.handled_errors.append(error)
    
    def log_error(self, message: str) -> None:
        """记录错误"""
        self.handled_errors.append(message)


class MockChatProcessor(IChatProcessor):
    """模拟聊天处理器"""
    
    def __init__(self):
        pass
    
    def process_message(self, message: str) -> str:
        """处理消息"""
        return f"Mock response to: {message}"


class MockStreamingProcessor(IStreamingProcessor):
    """模拟流处理器"""
    
    def __init__(self):
        self.is_streaming = False
    
    def start_streaming(self) -> None:
        """开始流处理"""
        self.is_streaming = True
    
    def stop_streaming(self) -> None:
        """停止流处理"""
        self.is_streaming = False


class MockWakeWordDetector(IWakeWordDetector):
    """模拟唤醒词检测器"""
    
    def __init__(self):
        self.is_listening = False
    
    def detect_wake_word(self, audio_data: bytes) -> bool:
        """检测唤醒词"""
        return True


class MockConversationManager(IConversationManager):
    """模拟对话管理器"""
    
    def __init__(self):
        self.conversations = []
    
    def start_conversation(self) -> None:
        """开始对话"""
        self.conversations.append("new_conversation")
    
    def end_conversation(self) -> None:
        """结束对话"""
        if self.conversations:
            self.conversations.pop()
    
    def process_input(self, input_data: str) -> str:
        """处理输入"""
        return f"Mock processed: {input_data}"