import logging
from typing import Callable, Dict, Set
from pynput import keyboard

logger = logging.getLogger(__name__)

class KeyboardService:
    """键盘监听服务"""
    
    def __init__(self, reserved_audios_dir: str):
        self.reserved_audios_dir = reserved_audios_dir
        self.key_handlers: Dict[str, Callable] = {}
        self.is_running = False
        self.listener = None
        self.current_pressed_keys: Set[str] = set()

    def start(self) -> None:
        """启动键盘监听"""
        if self.is_running:
            return

        self.is_running = True
        #self.listener = threading.Thread(target=self._listen, daemon=True)
        #self.listener.start()
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
        logger.info("键盘监听服务已启动")

    def stop(self) -> None:
        """停止键盘监听"""
        self.is_running = False
        if self.listener:
            self.listener.stop()
            self.listener.join(timeout=1.0)
        logger.info("键盘监听服务已停止")

    def register_handler(self, key: str, handler: Callable) -> None:
        """
        注册按键处理器
        :param key: 按键组合，格式如 "ctrl+1"
        :param handler: 处理函数
        """
        self.key_handlers[key.lower()] = handler
        logger.info(f"注册按键处理器: {key}")

    """
    def _listen(self) -> None:
        #监听键盘输入1
        while self.is_running:
            try:
                event = keyboard.read_event()
                if event.event_type == keyboard.KEY_DOWN:
                    key = event.name
                    if key in self.key_handlers:
                        self.key_handlers[key]()
            except Exception as e:
                logger.error(f"键盘监听错误: {str(e)}")
                continue 
    """

    def _get_key_name(self, key) -> str:
        """获取按键名称"""
        try:
            if isinstance(key, keyboard.Key):
                return key.name.lower()
            elif isinstance(key, keyboard.KeyCode):
                if hasattr(key, 'char') and key.char:
                    return key.char.lower()
                elif hasattr(key, 'vk') and key.vk:
                    return str(key.vk).lower()
            return ""
        except Exception:
            return ""

    def _on_press(self, key) -> None:
        """按键按下回调"""
        try:
            if not self.is_running:
                return

            key_name = self._get_key_name(key)
            if key_name:
                self.current_pressed_keys.add(key_name)
                
                # 检查是否按下了 ctrl 或 alt + 数字的组合
                if 'ctrl' in self.current_pressed_keys or 'alt' in self.current_pressed_keys:
                    for char in [str(i) for i in range(1, 10)] + [chr(ord('a') + i) for i in range(26)]:
                        if char in self.current_pressed_keys:
                            if 'ctrl' in self.current_pressed_keys:
                                combo = f'ctrl+{char}'
                            else:
                                combo = f'alt+{char}'
                            if combo in self.key_handlers:
                                self.key_handlers[combo]()
                                break

        except Exception as e:
            logger.error(f"键盘监听错误: {str(e)}")

    def _on_release(self, key) -> None:
        """按键释放回调"""
        try:
            key_name = self._get_key_name(key)
            if key_name in self.current_pressed_keys:
                self.current_pressed_keys.remove(key_name)
        except Exception as e:
            logger.error(f"键盘释放错误: {str(e)}")