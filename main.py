import logging
import time
import os
import threading
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

# 🟢 新增导入
from services.streaming.streaming_handler import StreamingHandler
from services.streaming.audio_playback_queue import AudioPlaybackQueue

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
                        keyboard_service.register_handler("ctrl+1",
                                                          lambda: audio_service.play_reserved_audio("gaoxiao1"))
                        keyboard_service.register_handler("ctrl+2",
                                                          lambda: audio_service.play_reserved_audio("gaoxiao2"))
                        keyboard_service.register_handler("alt+1",
                                                          lambda: audio_service.play_reserved_audio("aochengda1"))
                        keyboard_service.register_handler("alt+2",
                                                          lambda: audio_service.play_reserved_audio("gangchengda1"))
                        keyboard_service.register_handler("alt+3",
                                                          lambda: audio_service.play_reserved_audio("gangchengda2"))
                        keyboard_service.register_handler("alt+4",
                                                          lambda: audio_service.play_reserved_audio("gangchengda3"))

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

                                    # todo:改为流式处理 开始

                                    if Config.ENABLE_STREAMING:
                                        # 🟢 流式处理模式
                                        logger.info("🚀 启动流式处理模式")

                                        # 创建音频播放队列
                                        audio_playback_queue = AudioPlaybackQueue()
                                        audio_playback_queue.start()

                                        # 创建流式处理器
                                        streaming_handler = StreamingHandler(
                                            tts_engine=None,  # 不使用本地TTS
                                            audio_playback_queue=audio_playback_queue,
                                            api_client=api_client
                                        )
                                        streaming_handler.voice_id = Config.VOICE_ID
                                        streaming_handler.speed = Config.AUDIO_SETTINGS["speed"]
                                        streaming_handler.sample_rate = Config.AUDIO_SETTINGS["sample_rate"]

                                        streaming_handler.start_tts_worker()

                                        # 记录提问开始时间
                                        question_start_time = time.time()
                                        audio_playback_queue.set_question_start_time(question_start_time)

                                        try:
                                            # 发送流式请求
                                            bot_id = "7549003041853620264"  # 请根据你的实际配置修改
                                            user_id = "zhengjia003"  # 请根据你的实际配置修改

                                            for event_data in api_client.send_chat_request_stream(bot_id, user_id,
                                                                                                  transcript):
                                                event = event_data["event"]
                                                data = event_data["data"]

                                                if event == "conversation.message.delta":
                                                    if data and data.get('type') == "answer" and data.get('content'):
                                                        content_chunk = data['content']
                                                        streaming_handler.process_chunk(content_chunk)

                                                elif event == "conversation.chat.completed":
                                                    break

                                            # 处理剩余内容
                                            streaming_handler.flush_remaining()

                                            # 等待TTS处理完成
                                            logger.info("⏳ 等待TTS处理完成...")
                                            streaming_handler.sentence_queue.join()

                                            # 等待所有音频播放完成
                                            logger.info("⏳ 等待所有音频播放完成...")
                                            audio_playback_queue.audio_queue.join()

                                            # 打印计时统计
                                            timing_stats = audio_playback_queue.get_timing_stats()
                                            logger.info("⏱️  流式处理计时统计:")
                                            if 'first_play_delay' in timing_stats:
                                                logger.info(
                                                    f"⏱️  提问 → 首句播放延迟: {timing_stats['first_play_delay']:.3f} 秒")
                                            if 'inter_sentence_delays' in timing_stats:
                                                for i, delay in enumerate(timing_stats['inter_sentence_delays'], 1):
                                                    logger.info(
                                                        f"⏱️  第{i}句结束 → 第{i + 1}句开始延迟: {delay:.3f} 秒")
                                            if 'total_playback_duration' in timing_stats:
                                                logger.info(
                                                    f"⏱️  所有音频播放总耗时: {timing_stats['total_playback_duration']:.3f} 秒")

                                        except Exception as e:
                                            logger.error(f"流式处理异常: {str(e)}")
                                            error_handler.handle_error(e, ErrorCategory.API)
                                        finally:
                                            # 停止工作线程
                                            streaming_handler.stop_tts_worker()
                                            audio_playback_queue.stop()

                                    else:
                                        # 🟡 原有非流式处理模式
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
                                                if not audio_service._play_audio(
                                                        "/home/lab/szhr/CozeRobotChat/records/outputs.wav"):
                                                    audio_service.end_conversation()
                                                    continue
                                                logging.info("Conversation cycle completed successfully")
                                            except AudioError as e:
                                                error_handler.handle_error(e, ErrorCategory.AUDIO)
                                                audio_service.end_conversation()
                                                continue

                                    # todo:改为流式处理 结束

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