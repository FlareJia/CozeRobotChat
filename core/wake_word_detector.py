# core/wake_word_detector.py
import logging
from config import Config
from core.interfaces.unified_interfaces import IAudioDevice, IWakeWordDetector
from services.exceptions import AudioError

logger = logging.getLogger(__name__)


class WakeWordDetector(IWakeWordDetector):
    """
    唤醒词检测器，负责检测用户的唤醒词
    """
    
    def __init__(self, audio_interface: IAudioDevice):
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