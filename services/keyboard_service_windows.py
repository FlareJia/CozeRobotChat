from pynput.keyboard import Listener, Key, KeyCode
import logging
import threading
from typing import Callable, Dict, Set
import sys
import time

# -------------------------- 自定义日志配置（核心修改） --------------------------
class VSCodeTerminalHandler(logging.StreamHandler):
    """
    适配VS Code终端的日志处理器，强制使用UTF-8编码输出，解决特殊字符（如✅）的GBK编码错误
    """
    def emit(self, record):
        try:
            # 格式化日志信息
            msg = self.format(record)
            terminator = self.terminator if hasattr(self, 'terminator') else '\n'
            # VS Code终端支持UTF-8字节流直接输出，避免编码转换
            if hasattr(sys.stdout, 'buffer'):
                sys.stdout.buffer.write((msg + terminator).encode('utf-8'))
            else:
                # 兼容无buffer的情况（极少出现）
                sys.stdout.write(msg + terminator)
            self.flush()
        except Exception:
            self.handleError(record)

# 初始化日志器并配置自定义处理器
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.handlers.clear()  # 清除默认处理器，避免重复输出

# 创建自定义处理器并设置格式
log_formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
vscode_handler = VSCodeTerminalHandler()
vscode_handler.setFormatter(log_formatter)
logger.addHandler(vscode_handler)

# -------------------------- 键盘服务类（原有逻辑） --------------------------
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
        logger.info(f"✅ 键盘监听服务已启动 on {self.device}")

    def stop(self) -> None:
        """停止键盘监听"""
        self.is_running = False
        if self.listener:
            self.listener.stop()
        if self.thread:
            self.thread.join(timeout=1.0)
        logger.info("✅ 键盘监听服务已停止")

    def register_handler(self, key: str, handler: Callable) -> None:
        """注册按键/组合键处理器"""
        self.key_handlers[key.lower()] = handler
        logger.info(f"✅ 注册按键处理器: {key}")

    def _start_listener(self) -> None:
        """启动pynput监听器（内部方法）"""
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
        """检查组合键并执行对应的处理器"""
        pressed = self.current_pressed_keys
        # 处理Ctrl+数字/字母组合
        if 'ctrl' in pressed:
            for char in [str(i) for i in range(1, 10)] + [chr(ord('a') + i) for i in range(26)]:
                if char in pressed:
                    combo = f'ctrl+{char}'
                    if combo in self.key_handlers:
                        try:
                            self.key_handlers[combo]()
                        except Exception as e:
                            logger.error(f"执行{combo}处理器失败: {str(e)}", exc_info=True)
                        return
        # 处理Alt+数字/字母组合
        elif 'alt' in pressed:
            for char in [str(i) for i in range(1, 10)] + [chr(ord('a') + i) for i in range(26)]:
                if char in pressed:
                    combo = f'alt+{char}'
                    if combo in self.key_handlers:
                        try:
                            self.key_handlers[combo]()
                        except Exception as e:
                            logger.error(f"执行{combo}处理器失败: {str(e)}", exc_info=True)
                        return

# -------------------------- 测试代码（可选） --------------------------
def test_handler():
    """测试处理器函数"""
    logger.info("✅ Alt+4组合键被触发，执行测试处理器！")

if __name__ == "__main__":
    # 初始化键盘服务
    keyboard_service = KeyboardService(reserved_audios_dir="./audios")
    # 注册Alt+4组合键处理器（对应你提到的alt+4）
    keyboard_service.register_handler("alt+4", test_handler)
    # 启动服务
    keyboard_service.start()

    try:
        # 保持程序运行
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # 捕获Ctrl+C，停止服务
        keyboard_service.stop()
        logger.info("程序已退出")