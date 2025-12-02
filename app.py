# app.py
import logging
import os
import signal
import sys
from utils.action import Action
from typing import Optional
from config import Config, DIConfig, Environment
from core.di_container import DIContainer
from core.service_registry import ServiceRegistry
from core.interfaces.unified_interfaces import (
    IAudioDevice, IAudioService, IAPIClient, IConversationManager, 
    IWakeWordDetector, IChatProcessor, IStreamingProcessor, IErrorHandler
)
from services.scheduler import CleanupScheduler
from services.keyboard_service_windows import KeyboardService
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
        # 初始化依赖注入配置
        self.di_config = DIConfig()
        
        # 初始化依赖注入容器
        self.di_config = DIConfig()
        self.container = DIContainer()
        
        # 初始化服务注册表
        self.service_registry = ServiceRegistry(self.container, self.di_config)
        
        # 为ResourceManager设置依赖注入容器
        from managers.resource_manager import ResourceManager
        resource_manager = ResourceManager()
        resource_manager.set_container(self.container)
        
        # 服务组件（通过DI容器获取）
        self.error_handler: Optional[IErrorHandler] = None
        self.audio_service: Optional[IAudioService] = None
        self.cleanup_scheduler: Optional[CleanupScheduler] = None
        self.action_handler: Optional[Action] = None
        self.keyboard_service: Optional[KeyboardService] = None
        # 核心组件（通过DI容器获取）
        self.wake_word_detector: Optional[IWakeWordDetector] = None
        self.conversation_manager: Optional[IConversationManager] = None
        
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
            
            # 注册所有服务到DI容器
            self.service_registry.register_all_services()
            
            # 通过DI容器获取服务实例
            self.error_handler = self.container.resolve(IErrorHandler)
            self.audio_service = self.container.resolve(IAudioService)
            self.wake_word_detector = self.container.resolve(IWakeWordDetector)
            self.conversation_manager = self.container.resolve(IConversationManager)
            
            # 初始化清理调度器（暂时不通过DI，因为它没有复杂依赖）
            self.cleanup_scheduler = CleanupScheduler()
            self.cleanup_scheduler.start()
            
            # 初始化键盘服务（暂时不通过DI）
            reserved_audio_dir = os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["reserved_dir"])
            self.keyboard_service = KeyboardService(reserved_audio_dir)
            self._register_keyboard_handlers()
            self.action_handler = Action()
            # 记录DI容器状态（如果启用了DI日志）
            if self.di_config.should_enable_di_logging():
                logger.info(f"DI容器服务状态: {self.container.get_registered_services()}")
            
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
        注册键盘处理器（支持不同快捷键触发不同函数）
        """
        # 1. 前置校验：确保依赖服务已初始化（根据你的实际依赖修改）
        required_services = ["audio_service", "keyboard_service", "conversation_manager"]
        for service_name in required_services:
            if not hasattr(self, service_name) or not getattr(self, service_name):
                logger.warning(f"{service_name}未初始化，无法注册键盘处理器")
                return
        
        # 2. 解绑所有旧绑定（避免冲突，若keyboard_service支持）
        for key_combo in Config.KEYBOARD_BINDINGS.keys():
            if hasattr(self.keyboard_service, "unregister_handler"):
                self.keyboard_service.unregister_handler(key_combo)
                logger.info(f"已解绑旧绑定：{key_combo}")
        
        # 3. 遍历配置，动态注册不同函数
        for key_combo, bind_info in Config.KEYBOARD_BINDINGS.items():
            function_name = bind_info["function"]
            function_params = bind_info["params"]  # 函数所需参数
            
            # 4. 根据函数名，绑定对应的回调函数
            try:
                if function_name == "play_reserved_audio":
                    # 绑定函数1：播放预留音频（需要 audio_name 参数）
                    audio_name = function_params.get("audio_name")
                    if not audio_name:
                        logger.error(f"快捷键 {key_combo} 绑定 {function_name} 缺少参数 audio_name")
                        continue
                    # lambda 绑定参数（避免延迟绑定问题）
                    callback = lambda audio=audio_name: self.audio_service.play_reserved_audio(audio)
                
                elif function_name == "play_action":
                    # 绑定函数2：开始对话（无参数）
                    number = function_params.get("number")
                    callback = lambda num = number:self.action_handler.play_action(num)
                
                else:
                    logger.error(f"快捷键 {key_combo} 绑定未知函数：{function_name}")
                    continue
            
            except Exception as e:
                logger.error(f"快捷键 {key_combo} 绑定函数失败：{str(e)}")
                continue
            
            # 5. 注册当前快捷键的回调
            self.keyboard_service.register_handler(key_combo, callback)
            logger.info(
                f"已注册键盘绑定：{key_combo} → "
                f"{function_name}(参数：{function_params})"
            )       
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
        logger.info("开始清理应用资源...")
        
        try:
            # 停止键盘服务
            if hasattr(self, 'keyboard_service') and self.keyboard_service:
                self.keyboard_service.stop()
                logger.info("键盘服务已停止")
            
            # 停止清理调度器
            if hasattr(self, 'cleanup_scheduler') and self.cleanup_scheduler:
                self.cleanup_scheduler.stop()
                logger.info("清理调度器已停止")
            
            # 清理DI容器
            if hasattr(self, 'container') and self.container:
                self.container.clear()
                logger.info("DI容器已清理")
                
        except Exception as e:
            logger.error(f"清理过程中发生错误: {str(e)}")
        
        logger.info("应用资源清理完成")
            

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