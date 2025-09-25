# core/wake_word_detector.py
import logging
from config import Config
from hardware.audio_interface import RobotAudioInterface
from services.exceptions import AudioError

logger = logging.getLogger(__name__)


class WakeWordDetector:
    """
    唤醒词检测器，负责检测用户的唤醒词
    """
    
    def __init__(self, audio_interface: RobotAudioInterface):
        self.audio_interface = audio_interface
        self.config = Config
    
    def detect_wake_word(self) -> bool:
        """
        检测唤醒词
        :return: 是否检测到唤醒词
        """
        try:
            # 使用音频接口检测唤醒词
            return self.audio_interface.detect_wake_word()
        except AudioError as e:
            logger.error(f"唤醒词检测出错: {str(e)}")
            raise