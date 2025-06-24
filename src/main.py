import logging
import time
import os
import threading
from contextlib import contextmanager
from config import Config
from services.chat_processor import ChatProcessor
from services.audio_service import AudioService
from services.keyboard_service import KeyboardService
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


# 计时上下文管理器
@contextmanager
def time_recorder(step_name):
    start_time = time.perf_counter()
    try:
        yield
    finally:
        elapsed = time.perf_counter() - start_time
        logger.info(f"[性能监控] {step_name}耗时: {elapsed:.3f}秒")


def main():
    resource_manager = ResourceManager()

    try:
        with time_recorder("基础设施初始化"):
            # 初始化基础设施
            try:
                PathManager.create_dir(Config.OUTPUT_DIR)
                PathManager.create_dir(Config.RECORD_DIR)
                PathManager.create_dir(os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["reserved_dir"]))
            except Exception as e:
                with resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler:
                    error_handler.handle_error(e, ErrorCategory.FILE)
                raise

            # 启动清理调度器
            cleanup_scheduler = CleanupScheduler()
            cleanup_scheduler.start()

        # 使用资源管理器管理所有资源
        with resource_manager.manage_resource(ResourceType.AUDIO_MANAGER) as audio_mgr:
            # 初始化服务组件
            with time_recorder("服务组件初始化"):
                try:
                    with resource_manager.manage_resource(ResourceType.API_CLIENT) as api_client, \
                            resource_manager.manage_resource(ResourceType.AUDIO_DEVICE) as audio_interface, \
                            resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler:

                        # 创建音频服务
                        audio_service = AudioService(audio_interface, audio_mgr)

                        # 创建键盘监听服务
                        keyboard_service = KeyboardService(
                            os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["reserved_dir"])
                        )
                        keyboard_service.register_handler("ctrl+1", lambda: audio_service.play_reserved_audio("gaoxiao1"))
                        keyboard_service.register_handler("ctrl+2", lambda: audio_service.play_reserved_audio("gaoxiao2"))

                        # 启动键盘监听
                        keyboard_service.start()

                        # 创建对话处理器
                        processor = ChatProcessor(api_client, audio_service)

                        while True:
                            try:
                                # 检测唤醒词
                                logger.info("等待唤醒词...")
                                try:
                                    if not audio_interface.detect_wake_word():
                                        continue
                                except AudioError as e:
                                    error_handler.handle_error(e, ErrorCategory.AUDIO)
                                    continue

                                # 标记开始对话
                                audio_service.start_conversation()

                                # 播放提示音频
                                try:
                                    audio_service.play_hello_audio()
                                except AudioError as e:
                                    error_handler.handle_error(e, ErrorCategory.AUDIO)
                                    audio_service.end_conversation()
                                    continue

                                # 循环问话
                                while True:
                                    # 录音阶段
                                    with time_recorder("等待用户输入对话，音频录制"):
                                        try:
                                            audio_path = audio_service.record_and_transcribe()
                                            if not audio_path:
                                                audio_service.end_conversation()
                                                continue
                                        except AudioError as e:
                                            error_handler.handle_error(e, ErrorCategory.AUDIO)
                                            audio_service.end_conversation()
                                            continue

                                    # 语音转文本
                                    with time_recorder("语音转文字"):
                                        try:
                                            transcript = api_client.transcribe_audio(audio_path)
                                            logger.info("音频文件转文字完成，转换成的文字为：")
                                            logger.info(transcript)
                                        except APIError as e:
                                            error_handler.handle_error(e, ErrorCategory.API)
                                            audio_service.end_conversation()
                                            continue

                                    # 检测是否为结束对话
                                    if audio_interface.detect_bye_word(transcript):
                                        logger.info("检测到用户输入 再见，伯乐 退出聊天，播放再见音频文件。")
                                        audio_service.play_bye_audio()
                                        break

                                    # 处理对话流程
                                    with time_recorder("智能体处理"):
                                        try:
                                            result_audio = processor.process_query(transcript)
                                            logger.info("将coze返回的文字结果转为音频文件完成。")
                                        except APIError as e:
                                            error_handler.handle_error(e, ErrorCategory.API)
                                            audio_service.end_conversation()
                                            continue

                                    # 播放结果
                                    with time_recorder("音频播放"):
                                        try:
                                            if not audio_service.play_result_audio(result_audio):
                                                audio_service.end_conversation()
                                                continue
                                            logging.info("Conversation cycle completed successfully")
                                        except AudioError as e:
                                            error_handler.handle_error(e, ErrorCategory.AUDIO)
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
                    # 清理资源
                    error_handler._emergency_cleanup()

    except Exception as e:
        logger.critical(f"Critical error occurred: {str(e)}")
        with resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler:
            error_handler.handle_error(e, ErrorCategory.SYSTEM)
        raise
    finally:
        # 清理所有资源
        resource_manager.cleanup_all()


if __name__ == "__main__":
    main()