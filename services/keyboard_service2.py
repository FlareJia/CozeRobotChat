import logging
import os
import select  # 必须新增的模块
from typing import Callable, Dict, Set
from evdev import InputDevice, ecodes, categorize
import threading

logger = logging.getLogger(__name__)


class KeyboardService:
    # 需要安装依赖，命令： sudo pip3 install evdev
    """键盘监听服务（基于 evdev 实现）"""

    def __init__(self, reserved_audios_dir: str):
        self.reserved_audios_dir = reserved_audios_dir
        self.key_handlers: Dict[str, Callable] = {}
        self.is_running = False
        self.device_path = "/dev/input/event8"  # 设备路径已预设
        self.device = None
        self.current_pressed_keys: Set[str] = set()
        self.thread = None

    def start(self) -> None:
        """启动键盘监听"""
        if self.is_running:
            return

        if not self.device_path:
            logger.error("未指定输入设备路径！请设置 device_path")
            return

        try:
            self.device = InputDevice(self.device_path)
            logger.info(f"成功连接设备: {self.device.name}")

            # 新增：独占设备防止其他进程干扰
            try:
                self.device.grab()
            except Exception as e:
                logger.warning(f"设备独占失败（可能已被其他进程占用）: {str(e)}")

        except Exception as e:
            logger.error(f"无法打开设备 {self.device_path}: {str(e)}")
            return

        self.is_running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()
        logger.info("键盘监听服务已启动")

    def stop(self) -> None:
        """停止键盘监听"""
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.device:
            try:
                self.device.ungrab()  # 新增：释放设备
            except:
                pass
        logger.info("键盘监听服务已停止")

    def register_handler(self, key: str, handler: Callable) -> None:
        """注册按键处理器"""
        self.key_handlers[key.lower()] = handler
        logger.info(f"注册按键处理器: {key}")

    def _listen(self) -> None:
        """监听键盘事件（优化阻塞模式）"""
        if not self.device:
            return

        while self.is_running:
            try:
                # 使用 select 实现阻塞等待
                r, w, e = select.select([self.device], [], [], 1.0)

                if r:
                    # 逐个读取事件（更稳定的阻塞方式）
                    event = self.device.read_one()

                    if event and event.type == ecodes.EV_KEY:
                        key_event = categorize(event)
                        key_name = self._get_key_name(key_event)

                        if key_name:
                            if key_event.keystate == key_event.key_down:
                                self.current_pressed_keys.add(key_name)
                                self._check_combinations()
                            elif key_event.keystate == key_event.key_up:
                                self.current_pressed_keys.discard(key_name)

            except Exception as e:
                # 新增详细错误日志
                logger.error(f"键盘监听错误: {str(e)}", exc_info=True)
                break

    def _get_key_name(self, key_event) -> str:
        """获取按键名称（优化异常处理）"""
        try:
            key_code = key_event.keycode
            if isinstance(key_code, list):
                key_code = key_code[0]

            # 特殊键映射
            if key_code == 'KEY_LEFTCTRL' or key_code == 'KEY_RIGHTCTRL':
                return 'ctrl'
            elif key_code == 'KEY_LEFTALT' or key_code == 'KEY_RIGHTALT':
                return 'alt'
            elif key_code.startswith('KEY_'):
                key_char = key_code[4:].lower()
                if key_char.isdigit() or key_char.isalpha():
                    return key_char
                return key_char
            return ""
        except Exception as e:
            logger.error(f"解析按键错误: {str(e)}", exc_info=True)
            return ""

    def _check_combinations(self) -> None:
        """检查组合键（优化性能）"""
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