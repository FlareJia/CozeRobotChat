# core/conversation_manager.py
import logging
import os
import time
from typing import Optional
from contextlib import contextmanager

from config import Config
from core.streaming_processor import StreamingProcessor
from services.api_client import EnhancedCozeAPIClient
from services.audio_service import AudioService
from services.chat_processor import ChatProcessor
from services.error_handler import AdvancedErrorHandler, ErrorCategory
from services.exceptions import AudioError, APIError
from utils.string_utils import detect_bye_word

logger = logging.getLogger(__name__)


@contextmanager
def time_recorder(step_name):
    """
    计时上下文管理器
    :param step_name: 步骤名称
    """
    start_time = time.perf_counter()
    try:
        yield
    finally:
        elapsed = time.perf_counter() - start_time
        logger.info(f"[性能监控] {step_name}耗时: {elapsed:.3f}秒")


class ConversationManager:
    """
    对话管理器，负责处理整个对话流程
    """
    
    def __init__(self, api_client: EnhancedCozeAPIClient, audio_service: AudioService, error_handler: AdvancedErrorHandler):
        self.api_client = api_client
        self.audio_service = audio_service
        self.error_handler = error_handler
        self.chat_processor = ChatProcessor(api_client, audio_service)
        self.streaming_processor = StreamingProcessor(api_client, audio_service, error_handler)
        
    def start_conversation(self) -> None:
        """
        开始对话
        """
        # 标记开始对话
        self.audio_service.start_conversation()
        
        # 播放提示音频
        try:
            self.audio_service.play_hello_audio()
        except AudioError as e:
            self.error_handler.handle_error(e, ErrorCategory.AUDIO)
            self.audio_service.end_conversation()
            return
            
        # 循环问话
        while True:
            try:
                # 录音阶段
                audio_path = self._record_user_input()
                if not audio_path:
                    self.audio_service.end_conversation()
                    break
                    
                # 语音转文本
                transcript = self._transcribe_audio(audio_path)
                if not transcript:
                    self.audio_service.end_conversation()
                    break
                    
                # 检测是否为结束对话
                if detect_bye_word(transcript):
                    logger.info("检测到用户输入 再见，伯乐 退出聊天，播放再见音频文件。")
                    self.audio_service.play_bye_audio()
                    break
                    
                # 处理对话
                if not self._process_conversation(transcript):
                    break
                    
            except Exception as e:
                self.error_handler.handle_error(e, ErrorCategory.UNKNOWN)
                self.audio_service.end_conversation()
                break
                
        # 标记对话结束
        self.audio_service.end_conversation()
    
    def _record_user_input(self) -> Optional[str]:
        """
        录制用户输入
        :return: 录音文件路径，如果失败则返回None
        """
        with time_recorder("等待用户输入对话，音频录制"):
            try:
                audio_path = self.audio_service.record_and_transcribe()
                return audio_path
            except AudioError as e:
                self.error_handler.handle_error(e, ErrorCategory.AUDIO)
                return None
    
    def _transcribe_audio(self, audio_path: str) -> Optional[str]:
        """
        将音频转换为文本
        :param audio_path: 音频文件路径
        :return: 转换后的文本，如果失败则返回None
        """
        with time_recorder("语音转文字"):
            try:
                transcript = self.api_client.transcribe_audio(audio_path)
                logger.info("音频文件转文字完成，转换成的文字为：")
                logger.info(transcript)
                return transcript
            except APIError as e:
                self.error_handler.handle_error(e, ErrorCategory.API)
                return None
    
    def _process_conversation(self, transcript: str) -> bool:
        """
        处理对话
        :param transcript: 用户输入的文本
        :return: 是否成功处理
        """
        # 根据配置选择流式处理或非流式处理
        if Config.ENABLE_STREAMING:
            return self.streaming_processor.process(transcript)
        else:
            return self._process_non_streaming(transcript)
    
    def _process_non_streaming(self, transcript: str) -> bool:
        """
        非流式处理对话
        :param transcript: 用户输入的文本
        :return: 是否成功处理
        """
        # 原有非流式处理模式
        with time_recorder("智能体处理"):
            try:
                result_audio = self.chat_processor.process_query(transcript)
                logger.info("将coze返回的文字结果转为音频文件完成。")
            except APIError as e:
                self.error_handler.handle_error(e, ErrorCategory.API)
                return False

        # 播放结果
        with time_recorder("音频播放"):
            try:
                # todo 需要使用上面返回的result_audio作为播放路径
                outputs_path = os.path.join(Config.RECORD_DIR, "outputs.wav")
                if not self.audio_service.play_result_audio(outputs_path):
                    return False
                logging.info("Conversation cycle completed successfully")
                return True
            except AudioError as e:
                self.error_handler.handle_error(e, ErrorCategory.AUDIO)
                return False