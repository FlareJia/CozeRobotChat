import logging
import time
import os
import threading
from queue import Queue, Empty  # 新增导入
from difflib import SequenceMatcher
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


# todo 移动到其他地方
def _calculate_similarity(text1: str, text2: str) -> float:
    # 计算两个字符串的相似度
    #:param text1: 第一个字符串
    #:param text2: 第二个字符串
    #:return: 相似度（0-1之间）

    return SequenceMatcher(None, text1, text2).ratio()


def _is_bye_word_match(text: str) -> bool:
    # 检查文本是否匹配结束词
    #:param text: 待检查的文本
    #:return: 是否匹配

    if not text:
        return False

    # 计算相似度
    similarity = _calculate_similarity(text, Config.BYE_WORD_SETTINGS["bye_word"])
    logger.info(f"文本相似度: {similarity:.2f}")

    return similarity >= Config.BYE_WORD_SETTINGS["bye_word_threshold"]

    # todo 移动到其他地方


def detect_bye_word(text: str) -> bool:
    if text and _is_bye_word_match(text):
        logger.info("相似度检测，检测到结束词！")
        return True
    if text and Config.BYE_WORD_SETTINGS["bye_word"] in text:
        logger.info("全量in检测，检测到结束词！")
        return True
    return False


# 新增：预留音频播放工作线程
def reserved_audio_worker(queue: Queue, audio_service, stop_event: threading.Event):
    """
    从队列中获取并播放预留音频。
    """
    logger.info("预留音频播放工作线程已启动。")
    while not stop_event.is_set():
        try:
            # 非阻塞地从队列中获取音频编号，等待1秒
            audio_number = queue.get(timeout=1)
            logger.info(f"从队列中获取到预留音频编号 '{audio_number}'，准备播放。")
            audio_service.play_reserved_audio_sync(audio_number)  # 调用同步播放方法
            queue.task_done()
        except Empty:
            # 队列为空，继续等待
            continue
        except Exception as e:
            logger.error(f"播放预留音频时出错: {e}", exc_info=True)
    logger.info("预留音频播放工作线程已停止。")


def main():
    resource_manager = ResourceManager()

    # 新增：为工作线程创建停止事件
    stop_worker_event = threading.Event()

    # 新增：创建预留音频播放队列
    reserved_audio_queue = Queue() if Config.FEATURE_FLAGS.get('USE_ASYNC_RESERVED_AUDIO') else None

    # 新增：启动预留音频播放工作线程
    if reserved_audio_queue is not None:
        # 我们需要在 audio_service 初始化之后再启动线程
        # 因此线程启动代码将移动到后面
        worker_thread = None

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

                        # 如果启用异步播放，则在这里启动工作线程
                        if reserved_audio_queue is not None:
                            worker_thread = threading.Thread(
                                target=reserved_audio_worker,
                                args=(reserved_audio_queue, audio_service, stop_worker_event),
                                daemon=True
                            )
                            worker_thread.start()

                        # 创建键盘监听服务
                        keyboard_service = KeyboardService(
                            os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["reserved_dir"])
                        )

                        # 修改键盘快捷键的处理器
                        if Config.FEATURE_FLAGS.get('USE_ASYNC_RESERVED_AUDIO'):
                            # 异步方式：将播放任务（音频编号）放入队列
                            logger.info("注册异步音频播放处理器。")

 
                            keyboard_service.register_handler("ctrl+1", lambda: reserved_audio_queue.put(1))
                            keyboard_service.register_handler("ctrl+2", lambda: reserved_audio_queue.put(2))
                            keyboard_service.register_handler("ctrl+3", lambda: reserved_audio_queue.put(3))
                            keyboard_service.register_handler("ctrl+4", lambda: reserved_audio_queue.put(4))
                            keyboard_service.register_handler("ctrl+5", lambda: reserved_audio_queue.put(5))
                            keyboard_service.register_handler("ctrl+6", lambda: reserved_audio_queue.put(6))
                            keyboard_service.register_handler("ctrl+7", lambda: reserved_audio_queue.put(7))
                            keyboard_service.register_handler("ctrl+8", lambda: reserved_audio_queue.put(8))
                            keyboard_service.register_handler("ctrl+9", lambda: reserved_audio_queue.put(9))
                            keyboard_service.register_handler("ctrl+q", lambda: reserved_audio_queue.put(10))
                            keyboard_service.register_handler("ctrl+w", lambda: reserved_audio_queue.put(11))
                            keyboard_service.register_handler("ctrl+e", lambda: reserved_audio_queue.put(12))
                            keyboard_service.register_handler("ctrl+r", lambda: reserved_audio_queue.put(13))
                            keyboard_service.register_handler("ctrl+t", lambda: reserved_audio_queue.put(14))
                            keyboard_service.register_handler("ctrl+y", lambda: reserved_audio_queue.put(15))
                            keyboard_service.register_handler("ctrl+u", lambda: reserved_audio_queue.put(16))
                            keyboard_service.register_handler("ctrl+i", lambda: reserved_audio_queue.put(17))
                            keyboard_service.register_handler("ctrl+o", lambda: reserved_audio_queue.put(18))
                            keyboard_service.register_handler("ctrl+p", lambda: reserved_audio_queue.put(19))
                            keyboard_service.register_handler("ctrl+a", lambda: reserved_audio_queue.put(20))
                            keyboard_service.register_handler("ctrl+s", lambda: reserved_audio_queue.put(21))
                            keyboard_service.register_handler("ctrl+d", lambda: reserved_audio_queue.put(22))
                            keyboard_service.register_handler("ctrl+f", lambda: reserved_audio_queue.put(23))
                            keyboard_service.register_handler("ctrl+g", lambda: reserved_audio_queue.put(24))  # 问

                            keyboard_service.register_handler("alt+1", lambda: reserved_audio_queue.put(91))
                            keyboard_service.register_handler("alt+2", lambda: reserved_audio_queue.put(92))
                            keyboard_service.register_handler("alt+3", lambda: reserved_audio_queue.put(93))
                            keyboard_service.register_handler("alt+4", lambda: reserved_audio_queue.put(94))
                            keyboard_service.register_handler("alt+5", lambda: reserved_audio_queue.put(95))
                            keyboard_service.register_handler("alt+6", lambda: reserved_audio_queue.put(96))
                            keyboard_service.register_handler("alt+7", lambda: reserved_audio_queue.put(97))
                            keyboard_service.register_handler("alt+8", lambda: reserved_audio_queue.put(98))
                            keyboard_service.register_handler("alt+9", lambda: reserved_audio_queue.put(99))
                            keyboard_service.register_handler("alt+q", lambda: reserved_audio_queue.put(910))
                            keyboard_service.register_handler("alt+w", lambda: reserved_audio_queue.put(911))
                            keyboard_service.register_handler("alt+e", lambda: reserved_audio_queue.put(912))
                            keyboard_service.register_handler("alt+r", lambda: reserved_audio_queue.put(913))
                            keyboard_service.register_handler("alt+t", lambda: reserved_audio_queue.put(914))
                            keyboard_service.register_handler("alt+y", lambda: reserved_audio_queue.put(915))
                            keyboard_service.register_handler("alt+u", lambda: reserved_audio_queue.put(916))
                            keyboard_service.register_handler("alt+i", lambda: reserved_audio_queue.put(917))
                            keyboard_service.register_handler("alt+o", lambda: reserved_audio_queue.put(918))
                            keyboard_service.register_handler("alt+p", lambda: reserved_audio_queue.put(919))
                            keyboard_service.register_handler("alt+a", lambda: reserved_audio_queue.put(920))
                            keyboard_service.register_handler("alt+s", lambda: reserved_audio_queue.put(921))
                            keyboard_service.register_handler("alt+d", lambda: reserved_audio_queue.put(922))
                            keyboard_service.register_handler("alt+f", lambda: reserved_audio_queue.put(923))
                            keyboard_service.register_handler("alt+g", lambda: reserved_audio_queue.put(924))  # 答

                            keyboard_service.register_handler("ctrl+v", lambda: reserved_audio_queue.put(991))
                            keyboard_service.register_handler("ctrl+b", lambda: reserved_audio_queue.put(992))
                            keyboard_service.register_handler("ctrl+n", lambda: reserved_audio_queue.put(993))
                            keyboard_service.register_handler("ctrl+m", lambda: reserved_audio_queue.put(994))  # 开场白
                     
                        else:
                            # 同步方式：直接调用播放方法（旧逻辑） 
                            logger.info("注册同步音频播放处理器。")
                            keyboard_service.register_handler("ctrl+1",
                                                              lambda: audio_service.play_reserved_audio_sync(1))
                            keyboard_service.register_handler("ctrl+2",
                                                              lambda: audio_service.play_reserved_audio_sync(2))
                            keyboard_service.register_handler("ctrl+3",
                                                              lambda: audio_service.play_reserved_audio_sync(3))
                            keyboard_service.register_handler("ctrl+4",
                                                              lambda: audio_service.play_reserved_audio_sync(4))
                            keyboard_service.register_handler("ctrl+5",
                                                              lambda: audio_service.play_reserved_audio_sync(5))
                            keyboard_service.register_handler("ctrl+6",
                                                              lambda: audio_service.play_reserved_audio_sync(6))
                            keyboard_service.register_handler("ctrl+7",
                                                              lambda: audio_service.play_reserved_audio_sync(7))
                            keyboard_service.register_handler("ctrl+8",
                                                              lambda: audio_service.play_reserved_audio_sync(8))
                            keyboard_service.register_handler("ctrl+9",
                                                              lambda: audio_service.play_reserved_audio_sync(9))
                            keyboard_service.register_handler("alt+1",
                                                              lambda: audio_service.play_reserved_audio_sync(1))

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
                                    if detect_bye_word(transcript):
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
        # 新增：停止工作线程
        logger.info("正在停止预留音频播放工作线程...")
        stop_worker_event.set()
        if 'worker_thread' in locals() and worker_thread and worker_thread.is_alive():
            worker_thread.join(timeout=2)
            if worker_thread.is_alive():
                logger.warning("预留音频播放工作线程未能正常停止。")

        # 清理所有资源
        resource_manager.cleanup_all()


if __name__ == "__main__":
    main()