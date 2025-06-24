import pyaudio
import time
import logging
import audioop
import wave
import os
import platform
import threading
from typing import Optional
from playsound import playsound
from datetime import datetime
from difflib import SequenceMatcher
from src.config import Config
from src.utils.paths import PathManager
from src.services.exceptions import AudioError

logger = logging.getLogger(__name__)


class RobotAudioInterface:
    """硬件音频接口控制器"""

    def __init__(self):
        self.config = Config()
        self.audio = pyaudio.PyAudio()
        self._validate_audio_devices()
        self.wake_word = self.config.WAKE_WORD_SETTINGS["wake_word"]
        self.wake_word_buffer = self.config.WAKE_WORD_SETTINGS["wake_word_buffer"]
        self.wake_word_threshold = self.config.WAKE_WORD_SETTINGS["wake_word_threshold"]  # 语音识别相似度阈值
        self._play_thread = None
        self._stop_playing = False
        self._is_playing = False  # 添加播放状态标志
        self.bye_word = self.config.BYE_WORD_SETTINGS["bye_word"]
        self.bye_word_threshold = self.config.BYE_WORD_SETTINGS["bye_word_threshold"]  # 语音识别相似度阈值

    def _validate_audio_devices(self) -> None:
        """验证音频设备可用性"""
        if self.audio.get_device_count() == 0:
            raise AudioError("未检测到可用的音频设备")

    def _calculate_similarity(self, text1: str, text2: str) -> float:
        """
        计算两个字符串的相似度
        :param text1: 第一个字符串
        :param text2: 第二个字符串
        :return: 相似度（0-1之间）
        """
        return SequenceMatcher(None, text1, text2).ratio()

    def _is_wake_word_match(self, text: str) -> bool:
        """
        检查文本是否匹配唤醒词
        :param text: 待检查的文本
        :return: 是否匹配
        """
        if not text:
            return False

        # 计算相似度
        similarity = self._calculate_similarity(text, self.wake_word)
        logger.info(f"文本相似度: {similarity:.2f}, 阈值: {self.wake_word_threshold}")

        return similarity >= self.wake_word_threshold
    # todo 移动到其他地方
    def _is_bye_word_match(self, text: str) -> bool:
        """
        检查文本是否匹配结束词
        :param text: 待检查的文本
        :return: 是否匹配
        """
        if not text:
            return False

        # 计算相似度
        similarity = self._calculate_similarity(text, self.bye_word)
        logger.info(f"文本相似度: {similarity:.2f}, 阈值: {self.bye_word_threshold}")

        return similarity >= self.bye_word_threshold

    # todo 移动到其他地方
    def detect_bye_word(self, text: str) -> bool:
        if text and self._is_bye_word_match(text):
            logger.info("相似度检测，检测到结束词！")
            return True
        if text and self.bye_word in text:
            logger.info("全量in检测，检测到结束词！")
            return True
        return False

    def detect_wake_word(self) -> bool:
        """
        检测语音唤醒词
        :return: 是否检测到唤醒词
        """
        try:
            # 配置音频流
            stream = self.audio.open(
                format=self.config.DETECT_SETTINGS["format"],
                channels=self.config.DETECT_SETTINGS["channels"],
                rate=self.config.DETECT_SETTINGS["rate"],
                input=True,
                frames_per_buffer=self.config.DETECT_SETTINGS["chunk"]
            )

            logger.info("开始检测唤醒词...")
            frames = []
            recording = False
            silence_start = None
            start_time = time.time()

            # 录音主循环
            while True:
                data = stream.read(self.config.DETECT_SETTINGS["chunk"])
                rms = audioop.rms(data, 2)  # 计算音频能量值

                # 声音检测逻辑
                if rms > self.config.DETECT_SETTINGS["threshold"]:
                    if not recording:
                        logger.info("检测到声音，开始录音")
                        recording = True
                        start_time = time.time()
                    frames.append(data)
                    silence_start = None
                elif recording:
                    if silence_start is None:
                        silence_start = time.time()
                    elif time.time() - silence_start > self.config.DETECT_SETTINGS["silence_duration"]:  # 静默2秒后停止
                        break

                # 超时检查
                if time.time() - start_time > self.config.DETECT_SETTINGS["max_duration"]:  # 最多录音5秒
                    break

            # 保存临时录音文件
            if len(frames) > 0:
                # 计算录音时长
                recording_duration = len(frames) * self.config.DETECT_SETTINGS["chunk"] / self.config.DETECT_SETTINGS[
                    "rate"]
                logger.info(f"录音时长: {recording_duration:.2f}秒")

                # 如果录音时长小于0.5秒，直接返回False
                if recording_duration < self.config.DETECT_SETTINGS["min_recording_duration_second"]:
                    logger.info("录音时长过短，不进行语音识别")
                    return False

                temp_filename = os.path.join(
                    self.config.RECORD_DIR,
                    f"temp_wake_word_{int(time.time())}.wav"
                )
                with wave.open(temp_filename, 'wb') as wf:
                    wf.setnchannels(self.config.RECORD_SETTINGS["channels"])
                    wf.setsampwidth(self.audio.get_sample_size(self.config.RECORD_SETTINGS["format"]))
                    wf.setframerate(self.config.RECORD_SETTINGS["rate"])
                    wf.writeframes(b''.join(frames))

                # 使用语音识别检查是否包含唤醒词
                from src.services.api_client import EnhancedCozeAPIClient
                api_client = EnhancedCozeAPIClient(Config.BEARER_TOKEN)
                text = api_client.transcribe_audio(temp_filename)

                # 删除临时文件
                try:
                    os.remove(temp_filename)
                except:
                    pass

                logger.info("检测的用户输入语音为：")
                logger.info(text)
                if text and self._is_wake_word_match(text):
                    logger.info("相似度检测，检测到唤醒词！")
                    return True
                if text and self.wake_word in text:
                    logger.info("全量in检测，检测到唤醒词！")
                    return True

            return False

        except Exception as e:
            logger.error(f"唤醒词检测失败: {str(e)}")
            raise AudioError(f"唤醒词检测失败: {str(e)}")
        finally:
            if 'stream' in locals():
                stream.stop_stream()
                stream.close()

    def record_audio(self) -> Optional[str]:
        """
        执行录音操作
        :return: 录音文件路径（成功时）或 None（失败时）
        """
        try:
            # 生成唯一文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = PathManager.safe_join(
                self.config.RECORD_DIR,
                f"recording_{timestamp}.wav"
            )

            # 创建录音目录
            if not PathManager.create_dir(self.config.RECORD_DIR):
                raise AudioError("无法创建录音目录")

            # 配置音频流
            stream = self.audio.open(
                format=self.config.RECORD_SETTINGS["format"],
                channels=self.config.RECORD_SETTINGS["channels"],
                rate=self.config.RECORD_SETTINGS["rate"],
                input=True,
                frames_per_buffer=self.config.RECORD_SETTINGS["chunk"]
            )

            logger.info("开始录音...")
            frames = []
            recording = False
            silence_start = None
            start_time = time.time()

            # 录音主循环
            while True:
                data = stream.read(self.config.RECORD_SETTINGS["chunk"])
                rms = audioop.rms(data, 2)  # 计算音频能量值

                # 声音检测逻辑
                if rms > self.config.RECORD_SETTINGS["threshold"]:
                    if not recording:
                        logger.info("检测到声音，开始录音")
                        recording = True
                        start_time = time.time()
                    frames.append(data)
                    silence_start = None
                elif recording:
                    if silence_start is None:
                        silence_start = time.time()
                    elif time.time() - silence_start > self.config.RECORD_SETTINGS["silence_duration"]:
                        logger.info(f"静默超过{self.config.RECORD_SETTINGS['silence_duration']}秒，停止录音")
                        break

                # 超时检查
                if time.time() - start_time > self.config.RECORD_SETTINGS["max_duration"]:
                    logger.info("达到最大录音时间，停止录音")
                    break

            # 保存录音文件
            if len(frames) > 0:
                with wave.open(filename, 'wb') as wf:
                    wf.setnchannels(self.config.RECORD_SETTINGS["channels"])
                    wf.setsampwidth(self.audio.get_sample_size(self.config.RECORD_SETTINGS["format"]))
                    wf.setframerate(self.config.RECORD_SETTINGS["rate"])
                    wf.writeframes(b''.join(frames))
                logger.info(f"录音文件已保存至: {filename}")
                return filename
            return None

        except audioop.error as e:
            logger.error(f"音频处理错误: {str(e)}")
            raise AudioError(f"音频处理错误: {str(e)}")
        except IOError as e:
            logger.error(f"文件操作失败: {str(e)}")
            raise AudioError(f"文件操作失败: {str(e)}")
        finally:
            if 'stream' in locals():
                stream.stop_stream()
                stream.close()

    def play_audio(self, file_path: str) -> bool:
        """
        播放音频文件
        :param file_path: 音频文件路径
        :return: 是否播放成功
        """
        if not self._validate_audio_file(file_path):
            return False

        try:
            self.stop_audio()
            time.sleep(0.5)
            return self._play_by_system(file_path)
        except Exception as e:
            raise AudioError(f"播放失败: {str(e)}")

    def _validate_audio_file(self, file_path: str) -> bool:
        """验证音频文件有效性"""
        if not os.path.exists(file_path):
            raise AudioError(f"文件不存在: {file_path}")

        if not any(file_path.lower().endswith(fmt) for fmt in Config.AUDIO_FORMATS):
            raise AudioError(f"不支持的音频格式: {os.path.splitext(file_path)[1]}")

        return True

    def _play_by_system(self, file_path: str) -> bool:
        """根据操作系统调用不同的播放方式"""
        abs_path = os.path.abspath(file_path)
        system = platform.system()

        try:
            playsound(abs_path)
            logger.info(f"成功播放音频: {file_path}")
            return True
        except Exception as e:
            raise AudioError(f"播放命令执行失败: {str(e)}")

    def play_audio_async(self, file_path: str) -> threading.Thread:
        """
        异步播放音频文件
        :param file_path: 音频文件路径
        :return: 播放线程对象
        """
        if not self._validate_audio_file(file_path):
            raise AudioError(f"无效的音频文件: {file_path}")

        abs_path = os.path.abspath(file_path)

        # 如果已经有播放线程在运行，先停止它
        if self._play_thread and self._play_thread.is_alive():
            self._stop_playing = True
            self._play_thread.join(timeout=1.0)
            time.sleep(0.5)

        # 创建新的播放线程
        self._stop_playing = False
        self._is_playing = True
        self._play_thread = threading.Thread(
            target=self._play_audio_thread,
            args=(abs_path,),
            daemon=True
        )
        self._play_thread.start()

        logger.info(f"开始异步播放音频: {file_path}")
        return self._play_thread

    def _play_audio_thread(self, file_path: str) -> None:
        """
        在独立线程中播放音频
        :param file_path: 音频文件路径
        """
        try:
            playsound(file_path)
        except Exception as e:
            raise AudioError(f"音频播放失败: {str(e)}")
        finally:
            self._stop_playing = True
            self._is_playing = False

    def stop_audio(self) -> None:
        """
        停止当前正在播放的音频
        """
        if self._play_thread and self._play_thread.is_alive():
            self._stop_playing = True
            self._play_thread.join(timeout=1.0)
            logger.info("已停止音频播放")
            time.sleep(0.5)
        self._is_playing = False

    def is_playing(self) -> bool:
        """
        检查是否有音频正在播放
        :return: 是否正在播放
        """
        return self._is_playing

    def __del__(self):
        """清理PyAudio资源"""
        try:
            self.stop_audio()
            if hasattr(self, 'audio'):
                self.audio.terminate()
            logger.info("音频设备资源已清理")
        except Exception as e:
            logger.error(f"清理音频设备资源时发生错误: {str(e)}")
            # 确保资源被释放
            if hasattr(self, 'audio'):
                try:
                    self.audio.terminate()
                except:
                    pass