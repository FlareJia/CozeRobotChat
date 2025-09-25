import pyaudio
import time
import logging
import audioop
import wave
import os
import platform
import threading
from typing import Optional
from datetime import datetime
from config import Config
from utils.paths import PathManager
from utils.string_utils import calculate_similarity
from services.exceptions import AudioError
import subprocess  # 仅保留subprocess用于ROS服务调用


logger = logging.getLogger(__name__)


class RobotAudioInterface:
    """硬件音频接口控制器（基于ROS服务的音频播放实现）"""

    def __init__(self):
        self.config = Config()
        self.audio = pyaudio.PyAudio()
        self._validate_audio_devices()
        self.wake_word = self.config.WAKE_WORD_SETTINGS["wake_word"]
        self.wake_word_buffer = self.config.WAKE_WORD_SETTINGS["wake_word_buffer"]
        self.wake_word_threshold = self.config.WAKE_WORD_SETTINGS["wake_word_threshold"]  # 语音识别相似度阈值
        self._play_thread = None  # 异步播放线程
        self._stop_playing = False  # 停止播放标志
        self._is_playing = False  # 播放状态标志
        self.bye_word = self.config.BYE_WORD_SETTINGS["bye_word"]
        self.bye_word_threshold = self.config.BYE_WORD_SETTINGS["bye_word_threshold"]  # 语音识别相似度阈值
        self._disable_input = False  # 禁用音频输入标志

    def _validate_audio_devices(self) -> None:
        """验证音频设备可用性"""
        if self.audio.get_device_count() == 0:
            raise AudioError("未检测到可用的音频设备")

    def _calculate_similarity(self, text1: str, text2: str) -> float:
        """计算两个字符串的相似度（0-1之间）"""
        return calculate_similarity(text1, text2)

    def _is_wake_word_match(self, text: str) -> bool:
        """检查文本是否匹配唤醒词"""
        if not text:
            return False
        similarity = self._calculate_similarity(text, self.wake_word)
        logger.info(f"唤醒词相似度: {similarity:.2f}, 阈值: {self.wake_word_threshold}")
        return similarity >= self.wake_word_threshold

    def _is_bye_word_match(self, text: str) -> bool:
        """检查文本是否匹配结束词"""
        if not text:
            return False
        similarity = self._calculate_similarity(text, self.bye_word)
        logger.info(f"结束词相似度: {similarity:.2f}, 阈值: {self.bye_word_threshold}")
        return similarity >= self.bye_word_threshold

    def detect_bye_word(self, text: str) -> bool:
        """检测文本中是否包含结束词"""
        if text and self._is_bye_word_match(text):
            logger.info("相似度检测：检测到结束词！")
            return True
        if text and self.bye_word in text:
            logger.info("全量匹配：检测到结束词！")
            return True
        return False

    def detect_wake_word(self) -> bool:
        """检测语音中的唤醒词"""
        # 如果当前禁用了音频输入，直接返回False
        if self._disable_input:
            logger.info("音频输入已禁用，跳过唤醒词检测")
            return False
            
        stream = None
        try:
            # 配置音频流，添加异常处理参数
            stream = self.audio.open(
                format=self.config.DETECT_SETTINGS["format"],
                channels=self.config.DETECT_SETTINGS["channels"],
                rate=self.config.DETECT_SETTINGS["rate"],
                input=True,
                frames_per_buffer=self.config.DETECT_SETTINGS["chunk"],
                input_device_index=None,  # 使用默认输入设备
                start=False  # 不立即开始流
            )
            
            # 启动音频流
            stream.start_stream()

            logger.info("开始检测唤醒词...")
            frames = []
            recording = False
            silence_start = None
            start_time = time.time()

            # 录音主循环（检测声音并收集音频帧）
            while True:
                try:
                    # 使用非阻塞读取，避免输入溢出
                    data = stream.read(self.config.DETECT_SETTINGS["chunk"], exception_on_overflow=False)
                    rms = audioop.rms(data, 2)  # 计算音频能量（判断是否有声音）
                except Exception as read_error:
                    logger.warning(f"音频读取警告: {read_error}")
                    # 清空缓冲区并继续
                    try:
                        stream.read(stream.get_read_available(), exception_on_overflow=False)
                    except:
                        pass
                    continue

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
                    elif time.time() - silence_start > self.config.DETECT_SETTINGS["silence_duration"]:
                        break  # 静默超过阈值，停止录音

                # 超时检查（防止无限录音）
                if time.time() - start_time > self.config.DETECT_SETTINGS["max_duration"]:
                    break

            # 处理录音结果
            if len(frames) > 0:
                recording_duration = len(frames) * self.config.DETECT_SETTINGS["chunk"] / self.config.DETECT_SETTINGS["rate"]
                logger.info(f"录音时长: {recording_duration:.2f}秒")

                if recording_duration < self.config.DETECT_SETTINGS["min_recording_duration_second"]:
                    logger.info("录音时长过短，不进行语音识别")
                    return False

                # 保存临时录音文件
                temp_filename = os.path.join(
                    self.config.RECORD_DIR,
                    f"temp_wake_word_{int(time.time())}.wav"
                )
                with wave.open(temp_filename, 'wb') as wf:
                    wf.setnchannels(self.config.RECORD_SETTINGS["channels"])
                    wf.setsampwidth(self.audio.get_sample_size(self.config.RECORD_SETTINGS["format"]))
                    wf.setframerate(self.config.RECORD_SETTINGS["rate"])
                    wf.writeframes(b''.join(frames))

                # 调用语音识别API转文字
                from services.api_client import EnhancedCozeAPIClient
                api_client = EnhancedCozeAPIClient(Config.BEARER_TOKEN)
                text = api_client.transcribe_audio(temp_filename)

                # 清理临时文件
                try:
                    os.remove(temp_filename)
                except Exception as e:
                    logger.warning(f"删除临时文件失败: {e}")

                logger.info(f"检测到的语音文本: {text}")
                if text and self._is_wake_word_match(text):
                    logger.info("相似度检测：检测到唤醒词！")
                    return True
                if text and self.wake_word in text:
                    logger.info("全量匹配：检测到唤醒词！")
                    return True

            return False

        except Exception as e:
            logger.error(f"唤醒词检测失败: {str(e)}")
            raise AudioError(f"唤醒词检测失败: {str(e)}")
        finally:
            if stream is not None:
                try:
                    if stream.is_active():
                        stream.stop_stream()
                    stream.close()
                except Exception as cleanup_error:
                    logger.warning(f"音频流清理警告: {cleanup_error}")

    def record_audio(self) -> Optional[str]:
        """录音并返回文件路径"""
        # 如果当前禁用了音频输入，直接返回None
        if self._disable_input:
            logger.info("音频输入已禁用，跳过录音")
            return None
            
        stream = None
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

            # 配置音频流，添加异常处理参数
            stream = self.audio.open(
                format=self.config.RECORD_SETTINGS["format"],
                channels=self.config.RECORD_SETTINGS["channels"],
                rate=self.config.RECORD_SETTINGS["rate"],
                input=True,
                frames_per_buffer=self.config.RECORD_SETTINGS["chunk"],
                input_device_index=None,  # 使用默认输入设备
                start=False  # 不立即开始流
            )
            
            # 启动音频流
            stream.start_stream()

            logger.info("开始录音...")
            frames = []
            recording = False
            silence_start = None
            start_time = time.time()

            # 录音主循环
            while True:
                try:
                    # 使用非阻塞读取，避免输入溢出
                    data = stream.read(self.config.RECORD_SETTINGS["chunk"], exception_on_overflow=False)
                    rms = audioop.rms(data, 2)  # 音频能量检测
                except Exception as read_error:
                    logger.warning(f"音频读取警告: {read_error}")
                    # 清空缓冲区并继续
                    try:
                        stream.read(stream.get_read_available(), exception_on_overflow=False)
                    except:
                        pass
                    continue

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
            if stream is not None:
                try:
                    if stream.is_active():
                        stream.stop_stream()
                    stream.close()
                except Exception as cleanup_error:
                    logger.warning(f"音频流清理警告: {cleanup_error}")

    def play_audio(self, file_path: str) -> bool:
        """同步播放音频（通过ROS服务）"""
        if not self._validate_audio_file(file_path):
            return False

        try:
            self.stop_audio()  # 停止当前播放（如果有）
            time.sleep(0.5)  # 等待停止完成
            
            # 禁用音频输入，防止自己的输出被录入
            self._disable_input = True
            self._is_playing = True
            
            result = self._play_via_ros_service(file_path)
            
            # 恢复音频输入
            self._disable_input = False
            self._is_playing = False
            
            return result
        except Exception as e:
            logger.error(f"同步播放失败: {e}")
            self._disable_input = False  # 确保异常情况下也恢复输入
            self._is_playing = False
            raise AudioError(f"同步播放失败: {str(e)}")

    def play_audio_async(self, file_path: str) -> threading.Thread:
        """异步播放音频（通过ROS服务，在独立线程中执行）"""
        if not self._validate_audio_file(file_path):
            raise AudioError(f"无效的音频文件: {file_path}")

        abs_path = os.path.abspath(file_path)

        # 停止当前播放线程（如果存在）
        if self._play_thread and self._play_thread.is_alive():
            self._stop_playing = True
            self._play_thread.join(timeout=1.0)
            time.sleep(0.5)

        # 禁用音频输入，防止自己的输出被录入
        self._disable_input = True
        
        # 创建新的播放线程
        self._stop_playing = False
        self._is_playing = True
        self._play_thread = threading.Thread(
            target=self._async_play_via_ros,
            args=(abs_path,),
            daemon=True
        )
        self._play_thread.start()
        logger.info(f"已启动异步播放线程，文件: {file_path}")
        return self._play_thread

    def _async_play_via_ros(self, file_path: str) -> None:
        """异步播放的线程执行函数"""
        try:
            # 调用ROS服务播放，通过_stop_playing标志控制中断
            result = subprocess.run(
                f"rosservice call /play_music2 '{file_path}'",
                shell=True,
                check=True,
                timeout=30  # 防止无限阻塞（可根据需求调整）
            )
            logger.info(f"异步播放完成，返回码: {result.returncode}")
        except subprocess.TimeoutExpired:
            if self._stop_playing:
                logger.info("异步播放已被主动终止")
            else:
                logger.error("异步播放超时")
        except Exception as e:
            logger.error(f"异步播放失败: {e}")
        finally:
            self._stop_playing = True
            self._is_playing = False
            # 恢复音频输入
            self._disable_input = False

    def _play_via_ros_service(self, file_path: str) -> bool:
        """通过ROS服务同步播放音频"""
        abs_path = os.path.abspath(file_path)
        try:
            # 调用ROS服务播放音频
            result = subprocess.run(
                f"rosservice call /play_music2 '{abs_path}'",
                shell=True,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            logger.info(f"ROS服务返回: {result.stdout}")
            logger.info(f"成功播放音频: {file_path}")
            
            # 等待音频播放完成
            # 估算音频长度并等待
            try:
                with wave.open(file_path, 'rb') as wf:
                    # 计算音频时长（秒）
                    frames = wf.getnframes()
                    rate = wf.getframerate()
                    duration = frames / float(rate)
                    # 添加一点缓冲时间
                    wait_time = duration + 0.5
                    logger.info(f"等待音频播放完成，预计时长: {wait_time:.2f}秒")
                    time.sleep(wait_time)
            except Exception as e:
                # 如果无法计算时长，使用固定等待时间
                logger.warning(f"无法计算音频时长，使用默认等待时间: {e}")
                time.sleep(1.0)  # 默认等待1秒
                
            return True
        except subprocess.CalledProcessError as e:
            logger.error(f"ROS服务调用失败，错误输出: {e.stderr}")
            raise AudioError(f"ROS服务调用失败: {e.stderr}")

    def _validate_audio_file(self, file_path: str) -> bool:
        """验证音频文件有效性"""
        if not os.path.exists(file_path):
            raise AudioError(f"文件不存在: {file_path}")
        if not any(file_path.lower().endswith(fmt) for fmt in Config.AUDIO_FORMATS):
            raise AudioError(f"不支持的音频格式: {os.path.splitext(file_path)[1]}")
        return True

    def stop_audio(self) -> None:
        """停止当前播放的音频"""
        if self._play_thread and self._play_thread.is_alive():
            self._stop_playing = True
            self._play_thread.join(timeout=1.0)
            logger.info("已停止音频播放")
            time.sleep(0.5)
        self._is_playing = False
        # 恢复音频输入
        self._disable_input = False

    def is_playing(self) -> bool:
        """检查是否正在播放音频"""
        return self._is_playing

    def __del__(self):
        """清理资源"""
        try:
            self.stop_audio()
            if hasattr(self, 'audio'):
                self.audio.terminate()
            logger.info("音频接口资源已清理")
        except Exception as e:
            logger.error(f"资源清理错误: {e}")
            if hasattr(self, 'audio'):
                try:
                    self.audio.terminate()
                except:
                    pass