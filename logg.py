# logger_config.py
import logging
import sys
from logging.handlers import RotatingFileHandler

def setup_global_logger(level=logging.INFO):
    """配置全局日志，解决VS Code终端UTF-8编码问题"""
    # 自定义VS Code终端处理器
    class UTF8StreamHandler(logging.StreamHandler):
        def emit(self, record):
            try:
                msg = self.format(record)
                terminator = self.terminator if hasattr(self, 'terminator') else '\n'
                if hasattr(sys.stdout, 'buffer'):
                    sys.stdout.buffer.write((msg + terminator).encode('utf-8'))
                else:
                    sys.stdout.write(msg + terminator)
                self.flush()
            except Exception:
                self.handleError(record)

    # 根日志器配置
    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    root_logger.handlers.clear()  # 清除默认处理器

    # 控制台处理器（UTF-8）
    console_handler = UTF8StreamHandler()
    console_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    console_handler.setFormatter(console_formatter)
    root_logger.addHandler(console_handler)

    # 可选：文件处理器（避免日志丢失，UTF-8编码）
    file_handler = RotatingFileHandler(
        'app.log', maxBytes=10*1024*1024, backupCount=5, encoding='utf-8'
    )
    file_handler.setFormatter(console_formatter)
    root_logger.addHandler(file_handler)

# 项目启动时执行全局配置
setup_global_logger()