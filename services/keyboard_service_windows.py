from pynput.keyboard import Listener, Key, KeyCode
import logging
import threading
from typing import Callable, Dict, Set
import time

logger = logging.getLogger(__name__)

class KeyboardService:
    def __init__(self, reserved_audios_dir: str):
        self.reserved_audios_dir = reserved_audios_dir
        self.key_handlers: Dict[str, Callable] = {}
        self.is_running = False
        self.device = self._find_keyboard_device()  # 模拟设备查找
        self.current_pressed_keys: Set[str] = set()
        self.thread = None
        self.listener = None

    def _find_keyboard_device(self) -> str:
        """模拟键盘设备查找，返回虚拟设备标识"""
        logger.info("✅ 检测到标准键盘设备 (pynput虚拟设备)")
        return "virtual-keyboard-0"

    def start(self) -> None:
        """启动键盘监听"""
        if self.is_running:
            return

        self.device = self._find_keyboard_device()
        if not self.device:
            logger.error("未找到可用的键盘设备！")
            return

        self.is_running = True
        self.thread = threading.Thread(target=self._start_listener, daemon=True)
        self.thread.start()
        logger.info(f"键盘监听服务已启动 on {self.device}")

    def stop(self) -> None:
        self.is_running = False
        if self.listener:
            self.listener.stop()
        if self.thread:
            self.thread.join(timeout=1.0)
        logger.info("键盘监听服务已停止")

    def register_handler(self, key: str, handler: Callable) -> None:
        self.key_handlers[key.lower()] = handler
        logger.info(f"注册按键处理器: {key}")

    def _start_listener(self) -> None:
        """启动pynput监听器"""
        def on_press(key):
            key_name = self._get_key_name(key)
            if key_name:
                self.current_pressed_keys.add(key_name)
                self._check_combinations()

        def on_release(key):
            key_name = self._get_key_name(key)
            if key_name:
                self.current_pressed_keys.discard(key_name)

        with Listener(on_press=on_press, on_release=on_release) as self.listener:
            while self.is_running:
                time.sleep(0.1)
            self.listener = None

    def _get_key_name(self, key) -> str:
        """将pynput按键转换为标准化名称"""
        try:
            if key == Key.ctrl_l or key == Key.ctrl_r:
                return 'ctrl'
            elif key == Key.alt_l or key == Key.alt_r:
                return 'alt'
            elif isinstance(key, KeyCode):
                if len(key.char) == 1:
                    return key.char.lower()
            elif key == Key.enter:
                return 'enter'
            return ""
        except Exception as e:
            logger.error(f"解析按键错误: {str(e)}", exc_info=True)
            return ""

    def _check_combinations(self) -> None:
        pressed = self.current_pressed_keys
        if 'ctrl' in pressed:
            for char in [str(i) for i in range(1, 10)] + [chr(ord('a') + i) for i in range(26)]:
                if char in pressed:
                    combo = f'ctrl+{char}'
                    if combo in self.key_handlers:
                        try:
                            self.key_handlers[combo]()
                        except Exception as e:
                            logger.error(f"执行处理器失败: {str(e)}")
                        return
        elif 'alt' in pressed:
            for char in [str(i) for i in range(1, 10)] + [chr(ord('a') + i) for i in range(26)]:
                if char in pressed:
                    combo = f'alt+{char}'
                    if combo in self.key_handlers:
                        try:
                            self.key_handlers[combo]()
                        except Exception as e:
                            logger.error(f"执行处理器失败: {str(e)}")
                        return