# core/streaming_processor.py
import logging
import os
import time

from config import Config
from services.api_client import EnhancedCozeAPIClient
from services.audio_service import AudioService
from services.streaming.audio_playback_queue import AudioPlaybackQueue
from services.error_handler import AdvancedErrorHandler, ErrorCategory
from services.exceptions import AudioError, APIError
from services.streaming.streaming_handler import StreamingHandler

logger = logging.getLogger(__name__)


class StreamingProcessor:
    """
    流式处理器，负责处理流式对话
    """
    
    def __init__(self, api_client: EnhancedCozeAPIClient, audio_service: AudioService, error_handler: AdvancedErrorHandler):
        self.api_client = api_client
        self.audio_service = audio_service
        self.error_handler = error_handler
        
    def process(self, transcript: str) -> bool:
        """
        处理流式对话
        :param transcript: 用户输入的文本
        :return: 是否成功处理
        """
        if not Config.ENABLE_STREAMING:
            logger.info("流式处理未启用")
            return False
            
        try:
            # 🟢 流式处理模式
            logger.info("🚀 启动流式处理模式")

            # 创建音频播放队列，传入audio_service以使用其播放方法
            # todo 本地测试就不在AudioPlaybackQueue塞入参数
            #audio_playback_queue = AudioPlaybackQueue(self.audio_service)
            audio_playback_queue = AudioPlaybackQueue()
            audio_playback_queue.start()

            # 创建流式处理器
            streaming_handler = StreamingHandler(
                tts_engine=None,  # 不使用本地TTS
                audio_playback_queue=audio_playback_queue,
                api_client=self.api_client
            )
            streaming_handler.voice_id = Config.VOICE_ID
            streaming_handler.speed = Config.AUDIO_SETTINGS["speed"]
            streaming_handler.sample_rate = Config.AUDIO_SETTINGS["sample_rate"]

            streaming_handler.start_tts_worker()

            # 记录提问开始时间
            question_start_time = time.time()
            audio_playback_queue.set_question_start_time(question_start_time)
            
            # 播放等待音频
            wait_audio_path = os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["wait_dir"], Config.AUDIO_NAMES["wait_wav_policy"])
            audio_playback_queue.enqueue(wait_audio_path, "已收到提问，请稍等。")

            try:
                # 发送流式请求
                for event_data in self.api_client.send_chat_request_stream(Config.BOT_ID, Config.USER_ID, transcript):
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
                self.error_handler.handle_error(e, ErrorCategory.API)
                return False
            finally:
                # 停止工作线程
                streaming_handler.stop_tts_worker()
                audio_playback_queue.stop()
                
            logger.info("流式处理完成")
            return True
            
        except (AudioError, APIError) as e:
            self.error_handler.handle_error(e, ErrorCategory.AUDIO if isinstance(e, AudioError) else ErrorCategory.API)
            return False
        except Exception as e:
            self.error_handler.handle_error(e, ErrorCategory.UNKNOWN)
            return False