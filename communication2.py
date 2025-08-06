import logging
import time
import os
import threading
from contextlib import contextmanager
from config import Config
from services.chat_processor import ChatProcessor
from services.audio_service import AudioService  # 仅保留与语音采集/转写相关的接口
from utils.paths import PathManager
from services.scheduler import CleanupScheduler
from services.error_handler import ErrorCategory
from services.exceptions import AudioError, APIError
from services.resource_manager import ResourceManager, ResourceType

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("logs/app.log"),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)


# 计时上下文管理器（性能监控）
@contextmanager
def time_recorder(step_name):
    start_time = time.perf_counter()
    try:
        yield
    finally:
        elapsed = time.perf_counter() - start_time
        logger.info(f"[性能监控] {step_name}耗时: {elapsed:.3f}秒")


# 字符串相似度计算（用于结束词检测）
def _calculate_similarity(text1: str, text2: str) -> float:
    from difflib import SequenceMatcher
    return SequenceMatcher(None, text1, text2).ratio()


# 结束词匹配逻辑
def _is_bye_word_match(text: str) -> bool:
    if not text:
        return False
    similarity = _calculate_similarity(text, Config.BYE_WORD_SETTINGS["bye_word"])
    logger.info(f"文本相似度: {similarity:.2f}")
    return similarity >= Config.BYE_WORD_SETTINGS["bye_word_threshold"]


# 结束词检测入口
def detect_bye_word(text: str) -> bool:
    if text and _is_bye_word_match(text):
        logger.info("相似度检测，检测到结束词！")
        return True
    if text and Config.BYE_WORD_SETTINGS["bye_word"] in text:
        logger.info("全量匹配，检测到结束词！")
        return True
    return False


def main():
    resource_manager = ResourceManager()

    try:
        with time_recorder("基础设施初始化"):
            # 创建必要的目录（语音文件存储等）
            try:
                PathManager.create_dir(Config.OUTPUT_DIR)
                PathManager.create_dir(Config.RECORD_DIR)
            except Exception as e:
                with resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler:
                    error_handler.handle_error(e, ErrorCategory.FILE)
                raise

            # 启动清理调度器（定期清理语音缓存文件）
            cleanup_scheduler = CleanupScheduler()
            cleanup_scheduler.start()

        # 资源管理上下文（语音设备、API客户端等）
        with resource_manager.manage_resource(ResourceType.AUDIO_MANAGER) as audio_mgr:
            with time_recorder("服务组件初始化"):
                try:
                    with resource_manager.manage_resource(ResourceType.API_CLIENT) as api_client, \
                            resource_manager.manage_resource(ResourceType.AUDIO_DEVICE) as audio_interface, \
                            resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler:

                        # 初始化音频服务（仅保留语音采集/转写相关功能）
                        audio_service = AudioService(audio_interface, audio_mgr)

                        # 创建对话处理器（核心逻辑：语音转文字→处理对话→文字转语音）
                        processor = ChatProcessor(api_client, audio_service)

                        # 主循环：等待唤醒词→进入对话流程
                        while True:
                            try:
                                # 1. 等待唤醒词（触发对话的起点）
                                logger.info("等待唤醒词...")
                                try:
                                    if not audio_interface.detect_wake_word():
                                        continue  # 未检测到唤醒词，继续等待
                                except AudioError as e:
                                    error_handler.handle_error(e, ErrorCategory.AUDIO)
                                    continue

                                # 标记对话开始
                                audio_service.start_conversation()

                                # 2. 进入多轮对话循环
                                while True:
                                    # 3. 录制用户语音并保存
                                    with time_recorder("用户语音录制"):
                                        try:
                                            audio_path = audio_service.record_and_transcribe()
                                            if not audio_path:  # 录制失败
                                                audio_service.end_conversation()
                                                continue
                                        except AudioError as e:
                                            error_handler.handle_error(e, ErrorCategory.AUDIO)
                                            audio_service.end_conversation()
                                            continue

                                    # 4. 语音转文字（调用API将录制的音频转为文本）
                                    with time_recorder("语音转文字"):
                                        try:
                                            transcript = api_client.transcribe_audio(audio_path)
                                            logger.info(f"用户输入文本：{transcript}")
                                        except APIError as e:
                                            error_handler.handle_error(e, ErrorCategory.API)
                                            audio_service.end_conversation()
                                            continue

                                    # 5. 检测结束词（如“再见”），终止对话
                                    if detect_bye_word(transcript):
                                        logger.info("检测到结束词，退出对话")
                                        break

                                    # 6. 处理对话（调用ChatProcessor生成回复）
                                    with time_recorder("对话处理"):
                                        try:
                                            # 生成回复音频（此处仅保留处理逻辑，剔除播放步骤）
                                            result_audio = processor.process_query(transcript)
                                            logger.info("对话回复处理完成")
                                        except APIError as e:
                                            error_handler.handle_error(e, ErrorCategory.API)
                                            audio_service.end_conversation()
                                            continue

                                # 标记对话结束
                                audio_service.end_conversation()

                            except Exception as e:
                                error_handler.handle_error(e, ErrorCategory.UNKNOWN)
                                audio_service.end_conversation()
                                continue

                except KeyboardInterrupt:
                    logger.info("用户中断程序")
                except Exception as e:
                    error_handler.handle_error(e, ErrorCategory.SYSTEM)
                    raise
                finally:
                    error_handler._emergency_cleanup()

    except Exception as e:
        logger.critical(f"致命错误: {str(e)}")
        with resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler:
            error_handler.handle_error(e, ErrorCategory.SYSTEM)
        raise
    finally:
        # 清理所有资源（语音设备、缓存等）
        resource_manager.cleanup_all()


if __name__ == "__main__":
    main()