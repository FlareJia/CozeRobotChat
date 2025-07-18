from evdev import InputDevice, ecodes, categorize, list_devices
import select
import logging
import threading
from typing import Callable, Dict, Set

logger = logging.getLogger(__name__)

class KeyboardService:
    def __init__(self, reserved_audios_dir: str):
        self.reserved_audios_dir = reserved_audios_dir
        self.key_handlers: Dict[str, Callable] = {}
        self.is_running = False
        self.device = self._find_keyboard_device()
        self.current_pressed_keys: Set[str] = set()
        self.thread = None

# ------------------- 新增功能说明 -------------------
# 1. 动态识别键盘设备，不再硬编码 /dev/input/eventX
# 2. 使用 evdev.list_devices() 遍历所有输入设备
# 3. 过滤出具有 EV_KEY 能力，且包含常用按键的设备
# 4. 排除名称中包含 'consumer', 'control', 'mouse' 的非主键盘
# 5. 自动选择合适的键盘设备进行监听
# 6. 启动时打印候选设备和最终选择的设备，便于调试
# -------------------------------------------------
    def _find_keyboard_device(self) -> InputDevice:
        devices = [InputDevice(path) for path in list_devices()]
        candidates = []

        for dev in devices:
            try:
                caps = dev.capabilities()
                if ecodes.EV_KEY in caps:
                    keys = caps[ecodes.EV_KEY]
                    common_keys = {
                        ecodes.KEY_A, ecodes.KEY_B, ecodes.KEY_C, ecodes.KEY_Z,
                        ecodes.KEY_1, ecodes.KEY_2, ecodes.KEY_9, ecodes.KEY_0,
                        ecodes.KEY_ENTER, ecodes.KEY_LEFTCTRL
                    }
                    if any(k in keys for k in common_keys):
                        candidates.append((dev, dev.name.lower()))
                        logger.info(f"候选键盘设备: {dev.path} ({dev.name})")
            except Exception as e:
                logger.warning(f"跳过设备: {dev.path} -> {str(e)}")

        if not candidates:
            logger.error("❌ 未找到合适的键盘输入设备！")
            return None

        # 优先选择不包含 consumer/control/mouse 的设备名
        for dev, name in candidates:
            if all(w not in name for w in ["consumer", "control", "mouse", "sys"]):
                logger.info(f"✅ 优先选择主键盘设备: {dev.path} ({dev.name})")
                return dev

        # 没找到更优的就退而求其次用第一个
        dev = candidates[0][0]
        logger.warning(f"⚠️ 使用次优键盘设备: {dev.path} ({dev.name})")
        return dev

    def start(self) -> None:
        """启动键盘监听"""
        if self.is_running:
            return

        self.device = self._find_keyboard_device()
        if not self.device:
            logger.error("未找到可用的键盘设备！")
            return

        try:
            self.device.grab()
        except Exception as e:
            logger.warning(f"设备独占失败（可能已被其他进程占用）: {str(e)}")

        self.is_running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()
        logger.info(f"键盘监听服务已启动 on {self.device.path} ({self.device.name})")

    def stop(self) -> None:
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.device:
            try:
                self.device.ungrab()
            except:
                pass
        logger.info("键盘监听服务已停止")

    def register_handler(self, key: str, handler: Callable) -> None:
        self.key_handlers[key.lower()] = handler
        logger.info(f"注册按键处理器: {key}")

    def _listen(self) -> None:
        if not self.device:
            return

        while self.is_running:
            try:
                r, _, _ = select.select([self.device], [], [], 1.0)
                if r:
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
                logger.error(f"键盘监听错误: {str(e)}", exc_info=True)
                break

    def _get_key_name(self, key_event) -> str:
        try:
            key_code = key_event.keycode
            if isinstance(key_code, list):
                key_code = key_code[0]
            if key_code == 'KEY_LEFTCTRL' or key_code == 'KEY_RIGHTCTRL':
                return 'ctrl'
            elif key_code == 'KEY_LEFTALT' or key_code == 'KEY_RIGHTALT':
                return 'alt'
            elif key_code.startswith('KEY_'):
                key_char = key_code[4:].lower()
                return key_char
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
