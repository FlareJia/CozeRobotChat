import pyaudio
import time
import logging
import audioop
import wave
import os
import platform
import threading
from typing import Optional, Tuple
from datetime import datetime
import numpy as np
from scipy.fftpack import fft  # 用于频率特征检测
from config import Config
from core.interfaces.unified_interfaces import IAudioDevice
from utils.paths import PathManager
from utils.string_utils import calculate_similarity
from services.exceptions import AudioError
from utils.string_utils import is_wake_word_match
import subprocess  # 仅保留subprocess用于ROS服务调用

# 忽略无关警告
import warnings
warnings.filterwarnings("ignore", message="No audio backend is available.")

logger = logging.getLogger(__name__)


class RobotAudioInterface(IAudioDevice):
    """硬件音频接口控制器（基于ROS服务的音频播放实现 + 简易VAD语音检测）"""
    # 类级别的校准标记，所有实例共享
    _noise_calibrated = False
    _calibrate_lock = threading.Lock()

    def __init__(self):
        self.config = Config()
        self.audio = pyaudio.PyAudio()
        self._validate_audio_devices()
        self.wake_word = self.config.WAKE_WORD_SETTINGS["wake_word"]
        self.wake_word_buffer = self.config.WAKE_WORD_SETTINGS["wake_word_buffer"]
        self.wake_word_threshold = self.config.WAKE_WORD_SETTINGS["wake_word_threshold"]
        self._play_thread = None
        self._stop_playing = False
        self._is_playing = False
        self.bye_word = self.config.BYE_WORD_SETTINGS["bye_word"]
        self.bye_word_threshold = self.config.BYE_WORD_SETTINGS["bye_word_threshold"]
        self._disable_input = False
        
        # 初始化底噪阈值相关参数
        self.noise_rms = 0
        self.record_dynamic_threshold = 0
        self.detect_dynamic_threshold = 0

        # 简易VAD配置（人声频率集中在300~3400Hz）
        self.human_freq_min = 300
        self.human_freq_max = 3400
        self.human_energy_ratio_threshold = 0.3  # 人声能量占比阈值

        # 仅当未校准时执行
        self.calibrate_noise_threshold_once()

    def _validate_audio_devices(self) -> None:
        """验证音频设备可用性"""
        if self.audio.get_device_count() == 0:
            raise AudioError("未检测到可用的音频设备")

    @staticmethod
    def _calculate_similarity(text1: str, text2: str) -> float:
        """计算两个字符串的相似度（0-1之间）"""
        return calculate_similarity(text1, text2)

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

    def _simple_vad_detect(self, data: bytes, sample_rate: int) -> Tuple[bool, float]:
        """
        简易VAD检测：基于RMS + 人声频率能量占比
        :param data: 音频帧字节数据
        :param sample_rate: 采样率（如16000Hz）
        :return: (是否为有效语音, 人声能量占比)
        """
        # 1. 转换为numpy数组（16bit）
        audio_np = np.frombuffer(data, dtype=np.int16)
        if len(audio_np) == 0:
            return False, 0.0

        # 2. 快速傅里叶变换（FFT）获取频率特征
        fft_data = fft(audio_np)
        freq = np.fft.fftfreq(len(fft_data), 1/sample_rate)

        # 3. 仅保留正频率
        positive_mask = freq >= 0
        positive_freq = freq[positive_mask]
        positive_fft = np.abs(fft_data[positive_mask])

        # 4. 计算人声频率（300~3400Hz）的能量占比
        human_freq_mask = (positive_freq >= self.human_freq_min) & (positive_freq <= self.human_freq_max)
        human_energy = np.sum(positive_fft[human_freq_mask])
        total_energy = np.sum(positive_fft)

        # 避免除零错误
        if total_energy == 0:
            return False, 0.0
        human_energy_ratio = human_energy / total_energy

        # 5. 能量占比超过阈值则判定为有效语音
        is_speech = human_energy_ratio > self.human_energy_ratio_threshold
        return is_speech, human_energy_ratio

    def _is_valid_audio_frame(self, data: bytes, is_detect: bool = True) -> Tuple[bool, int]:
        """
        结合RMS和简易VAD判断是否为有效语音帧
        :param data: 音频帧字节数据
        :param is_detect: True=唤醒词检测，False=普通录音
        :return: (是否为有效语音, 帧RMS值)
        """
        # 1. 计算RMS，获取动态阈值
        rms = audioop.rms(data, 2)
        current_threshold = self.detect_dynamic_threshold if is_detect else self.record_dynamic_threshold
        current_threshold = current_threshold or self.config.DETECT_SETTINGS["threshold"]

        # 2. RMS过滤：低能量帧直接判定为无效
        if rms < current_threshold:
            return False, rms

        # 3. 简易VAD检测：高能量帧进一步判断是否为人类语音
        sample_rate = self.config.DETECT_SETTINGS["rate"] if is_detect else self.config.RECORD_SETTINGS["rate"]
        is_speech, energy_ratio = self._simple_vad_detect(data, sample_rate)
        logger.debug(f"帧RMS: {rms}, 人声能量占比: {energy_ratio:.2f}, 有效语音: {is_speech}")
        return is_speech, rms

    def calibrate_noise_threshold_once(self, record_seconds: int = 5, threshold_multiplier: float = 3):
        """仅执行一次的底噪校准"""
        with self._calibrate_lock:
            if not RobotAudioInterface._noise_calibrated:
                self.calibrate_noise_threshold(record_seconds, threshold_multiplier)
                RobotAudioInterface._noise_calibrated = True
            else:
                # 从类变量中读取已校准的阈值
                self.noise_rms = RobotAudioInterface._class_noise_rms
                self.record_dynamic_threshold = RobotAudioInterface._class_record_threshold
                self.detect_dynamic_threshold = RobotAudioInterface._class_detect_threshold
                logger.info(f"使用已校准的底噪阈值 - 底噪RMS: {self.noise_rms}, 录音阈值: {self.record_dynamic_threshold}")

    def calibrate_noise_threshold(self, record_seconds: int = 5, threshold_multiplier: float = 2.5) -> None:
        """原有校准逻辑，新增将阈值存为类变量"""
        if self._disable_input:
            self._disable_input = False
        
        stream = None
        try:
            stream = self.audio.open(
                format=self.config.RECORD_SETTINGS["format"],
                channels=self.config.RECORD_SETTINGS["channels"],
                rate=self.config.RECORD_SETTINGS["rate"],
                input=True,
                frames_per_buffer=self.config.RECORD_SETTINGS["chunk"],
                input_device_index=None,
                start=False
            )
            stream.start_stream()
            logger.info(f"开始录制{record_seconds}秒环境音，用于计算底噪...")
            
            rms_list = []
            start_time = time.time()

            while time.time() - start_time < record_seconds:
                try:
                    data = stream.read(self.config.RECORD_SETTINGS["chunk"], exception_on_overflow=False)
                    rms = audioop.rms(data, 2)
                    rms_list.append(rms)
                except Exception as e:
                    logger.warning(f"采集环境音时读取错误: {e}")
                    continue
            
            if not rms_list:
                raise AudioError("未采集到环境音的RMS数据，无法计算底噪")
            
            self.noise_rms = sorted(rms_list)[len(rms_list) // 2]
            self.record_dynamic_threshold = int(self.noise_rms * threshold_multiplier)
            self.detect_dynamic_threshold = int(self.noise_rms * threshold_multiplier)

            # 将阈值存为类变量，供其他实例使用
            RobotAudioInterface._class_noise_rms = self.noise_rms
            RobotAudioInterface._class_record_threshold = self.record_dynamic_threshold
            RobotAudioInterface._class_detect_threshold = self.detect_dynamic_threshold

            logger.info(f"环境底噪校准完成 - 底噪RMS值: {self.noise_rms}, 动态录音阈值: {self.record_dynamic_threshold}, 动态检测阈值: {self.detect_dynamic_threshold}")

            if self.config.RECORD_SETTINGS.get("threshold", 0) == 0:
                self.config.RECORD_SETTINGS["threshold"] = self.record_dynamic_threshold
            if self.config.DETECT_SETTINGS.get("threshold", 0) == 0:
                self.config.DETECT_SETTINGS["threshold"] = self.detect_dynamic_threshold

        except Exception as e:
            logger.error(f"环境底噪校准失败: {str(e)}")
            self.record_dynamic_threshold = self.config.RECORD_SETTINGS["threshold"]
            self.detect_dynamic_threshold = self.config.DETECT_SETTINGS["threshold"]
            logger.info(f"使用配置中的默认阈值 - 录音: {self.record_dynamic_threshold}, 检测: {self.detect_dynamic_threshold}")
        finally:
            if stream is not None:
                try:
                    stream.stop_stream()
                    stream.close()
                except Exception as cleanup_error:
                    logger.warning(f"环境音采集流清理错误: {cleanup_error}")
            self._disable_input = False

    def detect_wake_word(self) -> bool:
        """检测语音中的唤醒词（RMS + 简易VAD 结合）"""
        if self._disable_input:
            logger.info("音频输入已禁用，跳过唤醒词检测")
            return False
            
        stream = None
        try:
            # 配置音频流
            stream = self.audio.open(
                format=self.config.DETECT_SETTINGS["format"],
                channels=self.config.DETECT_SETTINGS["channels"],
                rate=self.config.DETECT_SETTINGS["rate"],
                input=True,
                frames_per_buffer=self.config.DETECT_SETTINGS["chunk"],
                input_device_index=None,
                start=False
            )
            
            stream.start_stream()
            logger.info("开始检测唤醒词（RMS + 简易VAD 结合）...")
            
            frames = []
            recording = False
            silence_start = None
            start_time = time.time()
            pre_frames = []  # 前导帧缓存：补全语音开头
            pre_frames_max = 3  # 缓存3帧前导数据

            # 录音主循环
            while True:
                try:
                    # 读取音频数据
                    data = stream.read(self.config.DETECT_SETTINGS["chunk"], exception_on_overflow=False)
                    # 核心：RMS + 简易VAD 检测有效语音
                    is_speech, rms = self._is_valid_audio_frame(data, is_detect=True)
                except Exception as read_error:
                    logger.warning(f"音频读取警告: {read_error}")
                    try:
                        stream.read(stream.get_read_available(), exception_on_overflow=False)
                    except:
                        pass
                    continue

                # 缓存前导帧（未触发录音时的有效帧，补全语音开头）
                if not recording and not is_speech:
                    pre_frames.append(data)
                    if len(pre_frames) > pre_frames_max:
                        pre_frames.pop(0)

                # 有效语音处理
                if is_speech:
                    if not recording:
                        logger.info("检测到有效语音，开始录音")
                        recording = True
                        start_time = time.time()
                        frames.extend(pre_frames)  # 补全前导帧
                        pre_frames = []
                    frames.append(data)
                    silence_start = None  # 重置静默计时
                else:
                    # 静默处理
                    if recording:
                        if silence_start is None:
                            silence_start = time.time()
                        elif time.time() - silence_start > self.config.DETECT_SETTINGS["silence_duration"]:
                            logger.info("静默超过阈值，停止录音")
                            break

                # 超时检查
                if time.time() - start_time > self.config.DETECT_SETTINGS["max_duration"]:
                    logger.info("达到最大检测时长，停止录音")
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
                if text and is_wake_word_match(text):
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
        """录音并返回文件路径（RMS + 简易VAD 结合）"""
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

            # 配置音频流
            stream = self.audio.open(
                format=self.config.RECORD_SETTINGS["format"],
                channels=self.config.RECORD_SETTINGS["channels"],
                rate=self.config.RECORD_SETTINGS["rate"],
                input=True,
                frames_per_buffer=self.config.RECORD_SETTINGS["chunk"],
                input_device_index=None,
                start=False
            )
            
            stream.start_stream()
            logger.info("开始录音（RMS + 简易VAD 结合）...")
            
            frames = []
            recording = False
            silence_start = None
            start_time = time.time()

            # 录音主循环
            while True:
                try:
                    data = stream.read(self.config.RECORD_SETTINGS["chunk"], exception_on_overflow=False)
                    # 核心：RMS + 简易VAD 检测有效语音
                    is_speech, rms = self._is_valid_audio_frame(data, is_detect=False)
                except Exception as read_error:
                    logger.warning(f"音频读取警告: {read_error}")
                    try:
                        stream.read(stream.get_read_available(), exception_on_overflow=False)
                    except:
                        pass
                    continue

                # 有效语音处理
                if is_speech:
                    if not recording:
                        logger.info("检测到有效语音，开始录音")
                        recording = True
                        start_time = time.time()
                    frames.append(data)
                    silence_start = None  # 重置静默计时
                else:
                    # 静默处理
                    if recording:
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
            try:
                with wave.open(file_path, 'rb') as wf:
                    frames = wf.getnframes()
                    rate = wf.getframerate()
                    duration = frames / float(rate)
                    wait_time = duration + 0.5
                    logger.info(f"等待音频播放完成，预计时长: {wait_time:.2f}秒")
                    time.sleep(wait_time)
            except Exception as e:
                logger.warning(f"无法计算音频时长，使用默认等待时间: {e}")
                time.sleep(1.0)
                
            return True
        except subprocess.CalledProcessError as e:
            logger.error(f"ROS服务调用失败，错误输出: {e.stderr}")
            raise AudioError(f"ROS服务调用失败: {e.stderr}")

    @staticmethod
    def _validate_audio_file(file_path: str) -> bool:
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

    def stop_playing(self) -> None:
        """停止播放音频"""
        self.stop_audio()

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