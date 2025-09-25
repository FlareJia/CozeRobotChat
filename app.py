# app.py
import logging
import os
import signal
import sys
from typing import Optional

from config import Config
from core.conversation_manager import ConversationManager
from core.wake_word_detector import WakeWordDetector
from services.resource_manager import ResourceManager, ResourceType
from services.scheduler import CleanupScheduler
from services.keyboard_service2 import KeyboardService
from utils.paths import PathManager

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
    ]
)
logger = logging.getLogger(__name__)


class Application:
    """
    应用程序类，负责初始化和运行应用
    """
    
    def __init__(self):
        # 初始化资源管理器
        self.resource_manager = ResourceManager()
        
        # 初始化服务组件（通过资源管理器）
        self.error_handler = None
        self.api_client = None
        self.audio_service = None
        self.robot_audio = None
        self.cleanup_scheduler = None
        self.keyboard_service = None
        
        # 初始化核心组件
        self.wake_word_detector = None
        self.conversation_manager = None
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self._handle_exit)
        signal.signal(signal.SIGTERM, self._handle_exit)
        
    def initialize(self) -> bool:
        """
        初始化应用
        :return: 是否成功初始化
        """
        try:
            # 创建必要的目录
            PathManager.create_dir(Config.OUTPUT_DIR)
            PathManager.create_dir(Config.RECORD_DIR)
            PathManager.create_dir(os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["reserved_dir"]))
            
            # 使用资源管理器获取长期资源（使用公共接口）
            self.error_handler = self.resource_manager.get_resource(ResourceType.ERROR_HANDLER)
            self.api_client = self.resource_manager.get_resource(ResourceType.API_CLIENT)
            self.audio_service = self.resource_manager.get_resource(ResourceType.AUDIO_SERVICE)
            self.robot_audio = self.resource_manager.get_resource(ResourceType.AUDIO_DEVICE)
            
            # 初始化核心组件
            self.wake_word_detector = WakeWordDetector(self.robot_audio)
            self.conversation_manager = ConversationManager(
                self.api_client, 
                self.audio_service, 
                self.error_handler
            )
            
            # 初始化清理调度器
            self.cleanup_scheduler = CleanupScheduler()
            self.cleanup_scheduler.start()
            
            # 初始化键盘服务
            reserved_audio_dir = os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["reserved_dir"])
            self.keyboard_service = KeyboardService(reserved_audio_dir)
            self._register_keyboard_handlers()
            
            return True
            
        except Exception as e:
            if hasattr(self, 'error_handler') and self.error_handler:
                self.error_handler.handle_error(e)
            else:
                logger.error(f"初始化失败: {str(e)}")
            return False
            
    def run(self) -> None:
        """
        运行应用
        """
        try:
            logger.info("应用程序启动，等待唤醒词...")
            
            while True:
                # 检测唤醒词
                if self.wake_word_detector.detect_wake_word():
                    # 开始对话
                    self.conversation_manager.start_conversation()
                    
        except KeyboardInterrupt:
            logger.info("接收到键盘中断，程序退出")
        except Exception as e:
            self.error_handler.handle_error(e)
        finally:
            self._cleanup()
            
    def _register_keyboard_handlers(self) -> None:
        """
        注册键盘处理器
        """
        for key_combo, audio_name in Config.KEYBOARD_BINDINGS.items():
            self.keyboard_service.register_handler(
                key_combo,
                lambda audio=audio_name: self.audio_service.play_reserved_audio(audio)
            )
            logger.info(f"已注册键盘绑定: {key_combo} -> {audio_name}")
                
    def _handle_exit(self, signum: Optional[int] = None, frame: Optional[object] = None) -> None:
        """
        处理退出信号
        """
        logger.info("接收到退出信号，程序退出")
        self._cleanup()
        sys.exit(0)
        
    def _handle_start_conversation(self) -> None:
        """
        处理开始对话的键盘事件
        """
        logger.info("通过键盘触发对话开始")
        self.conversation_manager.start_conversation()
        
    def _cleanup(self) -> None:
        """
        清理资源
        """
        if self.cleanup_scheduler:
            self.cleanup_scheduler.stop()
            
        if self.keyboard_service:
            self.keyboard_service.stop()
            
        # 清理所有通过资源管理器创建的资源
        if self.resource_manager:
            self.resource_manager.cleanup_all()
            

# 应用入口
def main():
    app = Application()
    if app.initialize():
        app.run()
    else:
        logger.error("应用初始化失败，程序退出")
        sys.exit(1)
        

if __name__ == "__main__":
    main()