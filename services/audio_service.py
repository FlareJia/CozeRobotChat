import os
import logging
from typing import Optional
from config import Config
from hardware.audio_interface import RobotAudioInterface
from services.audio_manager import AudioFileManager
from services.exceptions import AudioError

logger = logging.getLogger(__name__)


class AudioService:
    """统一管理音频相关操作的服务类"""

    def __init__(self, audio_interface: RobotAudioInterface, audio_manager: AudioFileManager):
        self.audio_interface = audio_interface
        self.audio_manager = audio_manager
        self.config = Config()
        self.reserved_audios_dir = os.path.join(self.config.OUTPUT_DIR, self.config.AUDIO_NAMES["reserved_dir"])
        self._current_play_thread = None
        self._in_conversation = False  # 添加对话状态标志

    def start_conversation(self) -> None:
        """开始对话流程"""
        self._in_conversation = True

    def end_conversation(self) -> None:
        """结束对话流程"""
        self._in_conversation = False

    def is_in_conversation(self) -> bool:
        """检查是否在对话流程中"""
        return self._in_conversation

    def is_playing(self) -> bool:
        """检查当前是否有音频正在播放"""
        return self.audio_interface.is_playing() or self._in_conversation

    def play_wait_audio(self) -> None:
        """播放等待音频"""
        wait_audio_dir = os.path.join(self.config.OUTPUT_DIR, self.config.AUDIO_NAMES["wait_dir"])
        wait_audio_path = os.path.join(
            wait_audio_dir,
            Config.AUDIO_NAMES["wait_mp3_policy"]
        )

        if os.path.exists(wait_audio_path):
            logger.info("播放等待音频...")
            self._play_audio_async(wait_audio_path)
        else:
            logger.warning(f"等待音频文件不存在: {wait_audio_path}")

    def play_hello_audio(self) -> None:
        """播放问候音频"""
        hello_audio_dir = os.path.join(self.config.OUTPUT_DIR, self.config.AUDIO_NAMES["hello_dir"])
        hello_audio_path = os.path.join(
            hello_audio_dir,
            Config.AUDIO_NAMES["hello_mp3"]
        )

        if os.path.exists(hello_audio_path):
            logger.info("播放提示音频...")
            self._play_audio(hello_audio_path)
        else:
            logger.warning(f"提示音频文件不存在: {hello_audio_path}")

    def play_reserved_audio(self, audio_number: int) -> None:
        """播放预留音频"""
        audio_path = os.path.join(self.reserved_audios_dir, f"reserved_{audio_number}.mp3")
        if os.path.exists(audio_path):
            logger.info(f"播放预留音频: reserved_{audio_number}.mp3")
            self._play_audio_async(audio_path)
        else:
            logger.warning(f"预留音频文件不存在: {audio_path}")

    def play_bye_audio(self) -> None:
        """播放再见音频"""
        bye_audio_dir = os.path.join(self.config.OUTPUT_DIR, self.config.AUDIO_NAMES["bye_dir"])
        bye_audio_path = os.path.join(
            bye_audio_dir,
            Config.AUDIO_NAMES["bye_mp3"]
        )

        if os.path.exists(bye_audio_path):
            logger.info("播放再见音频...")
            self._play_audio(bye_audio_path)
        else:
            logger.warning(f"再见音频文件不存在: {bye_audio_path}")


    def _play_audio(self, audio_path: str) -> bool:
        """播放音频（同步）"""
        try:
            self.stop_audio()  # 停止当前播放
            if self.audio_interface.play_audio(audio_path):
                # self.audio_manager.register_file(audio_path)
                return True
            return False
        except AudioError as e:
            logger.error(f"播放音频失败: {str(e)}")
            return False

    def _play_audio_async(self, audio_path: str) -> None:
        """异步播放音频"""
        self.stop_audio()  # 停止当前播放
        self._current_play_thread = self.audio_interface.play_audio_async(audio_path)

    def record_and_transcribe(self) -> Optional[str]:
        """录音并转写"""
        try:
            # 录音
            audio_path = self.audio_interface.record_audio()
            if not audio_path:
                return None

            logger.info("录音完成，保存的音频文件为：")
            print(audio_path)

            # 注册音频文件
            self.audio_manager.register_file(audio_path)

            return audio_path
        except AudioError as e:
            logger.error(f"录音过程发生错误: {str(e)}")
            return None

    def play_result_audio(self, audio_path: str) -> bool:
        """播放结果音频"""
        try:
            if audio_path and self._play_audio(audio_path):
                self.audio_manager.register_file(audio_path)
                logger.info("结果音频播放完成")
                return True
            return False
        except AudioError as e:
            logger.error(f"播放结果音频失败: {str(e)}")
            return False

    def stop_audio(self) -> None:
        """停止当前音频播放"""
        self.audio_interface.stop_audio()
        self._current_play_thread = None

    def _is_playing(self) -> bool:
        """检查当前是否有音频正在播放"""
        return self._current_play_thread is not None