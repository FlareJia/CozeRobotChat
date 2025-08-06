import logging
import time
import os
import numpy as np
import sounddevice as sd
import soundfile as sf
from config import Config
from services.exceptions import AudioError, APIError
from services.error_handler import AdvancedErrorHandler, ErrorCategory
from services.chat_processor import ChatProcessor  # 智能体处理核心组件
from services.audio_service import AudioService  # 音频播放服务
from services.resource_manager import ResourceManager, ResourceType

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("logs/audio_processing.log"),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)


class AudioProcessor:
    """音频处理器，支持完整的“录音→转文字→智能体交互→播放回复”流程"""

    def __init__(self, api_client, error_handler: AdvancedErrorHandler,
                 audio_service: AudioService,
                 device_index: int = 0, sample_rate: int = 48000):
        self.api_client = api_client
        self.error_handler = error_handler
        self.audio_service = audio_service  # 用于播放回复音频
        self.device_index = device_index
        self.sample_rate = sample_rate
        self.is_recording = False
        self.is_listening = False
        self.audio_buffer = []
        self.silence_counter = 0

        # 检查音频设备可用性
        try:
            devices = sd.query_devices()
            if device_index < 0 or device_index >= len(devices):
                raise ValueError(f"设备索引 {device_index} 无效，可用设备数: {len(devices)}")
            logger.info(f"使用音频设备: {devices[device_index]['name']}")
        except Exception as e:
            logger.error(f"音频设备初始化失败: {e}")
            raise AudioError(f"音频设备初始化失败: {e}")

    def listen_and_record(self, 
                          activation_threshold: float = 0.1,
                          silence_threshold: float = 0.05,
                          min_recording_time: float = 1.0,
                          max_recording_time: float = 15.0,
                          silence_duration: float = 1,
                          filename: str = None) -> str:
        """监听并录制语音（保持原逻辑不变）"""
        try:
            logger.info("开始监听环境声音，等待语音输入...")
            print("程序已启动，正在监听环境声音...")
            print("当检测到声音时将自动开始录音，声音结束后将自动停止并进行识别")

            # 生成保存路径
            if not filename:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = f"recording_{timestamp}.wav"
                output_dir = Config.RECORD_DIR
                os.makedirs(output_dir, exist_ok=True)
                filename = os.path.join(output_dir, filename)

            # 初始化参数
            self.is_listening = True
            self.is_recording = False
            self.audio_buffer = []
            silence_frames = int(silence_duration * self.sample_rate)
            min_recording_frames = int(min_recording_time * self.sample_rate)
            max_recording_frames = int(max_recording_time * self.sample_rate)
            self.silence_counter = 0
            total_frames = 0

            # 音频回调函数
            def audio_callback(indata, frames, time_info, status):
                nonlocal total_frames
                if status:
                    logger.warning(f"音频回调状态: {status}")
                
                # 计算音量（RMS）
                rms = np.sqrt(np.mean(indata**2))
                
                # 开始录音判断
                if not self.is_recording:
                    if rms > activation_threshold:
                        self.is_recording = True
                        self.audio_buffer = [indata.copy()]
                        total_frames = frames
                        logger.info(f"检测到声音，开始录音，音量: {rms:.4f}")
                        print("\n开始录音...")
                else:
                    # 录音中，累加数据
                    self.audio_buffer.append(indata.copy())
                    total_frames += frames
                    
                    # 最大时长判断
                    if total_frames >= max_recording_frames:
                        self.is_listening = False
                        logger.info("达到最大录音时长，停止录音")
                        return
                    
                    # 静音判断
                    if rms < silence_threshold:
                        self.silence_counter += frames
                    else:
                        self.silence_counter = 0
                    
                    # 停止录音条件（满足最小时长+足够静音）
                    if total_frames >= min_recording_frames and self.silence_counter >= silence_frames:
                        self.is_listening = False
                        logger.info(f"检测到静音，停止录音，音量: {rms:.4f}")

            # 启动录音流
            with sd.InputStream(
                    device=self.device_index,
                    samplerate=self.sample_rate,
                    channels=1,
                    callback=audio_callback
            ):
                while self.is_listening:
                    time.sleep(0.1)

            # 校验录音数据
            if not self.is_recording or not self.audio_buffer:
                raise AudioError("未录制到有效音频数据")

            # 合并并保存音频
            audio_data = np.concatenate(self.audio_buffer, axis=0)
            sf.write(filename, audio_data, self.sample_rate)
            logger.info(f"音频已保存到: {filename}")

            return filename
        except Exception as e:
            logger.error(f"录音失败: {e}")
            self.error_handler.handle_error(e, ErrorCategory.AUDIO)
            raise AudioError(f"录音失败: {e}")
        finally:
            self.is_recording = False
            self.is_listening = False

    def transcribe_audio(self, audio_file: str) -> str:
        """语音转文字（保持原逻辑不变）"""
        try:
            logger.info(f"开始音频转文字: {audio_file}")
            transcript = self.api_client.transcribe_audio(audio_file)
            logger.info(f"音频转文字完成: {transcript[:50]}...")
            return transcript
        except Exception as e:
            logger.error(f"音频转文字失败: {e}")
            self.error_handler.handle_error(e, ErrorCategory.API)
            raise APIError(f"音频转文字失败: {e}")

    def process_with_agent(self, text: str, chat_processor: ChatProcessor) -> str:
        """通过 ChatProcessor 处理智能体交互（核心修正点）"""
        try:
            logger.info(f"调用智能体处理文本: {text[:50]}...")
            
            # 关键修正：使用 ChatProcessor 处理对话（与主程序逻辑一致）
            # 该方法内部已封装“智能体回复生成+文本转语音”流程
            response_audio_path = chat_processor.process_query(text)
            
            logger.info(f"智能体回复音频生成完成: {response_audio_path}")
            return response_audio_path
        except Exception as e:
            logger.error(f"智能体处理失败: {e}")
            self.error_handler.handle_error(e, ErrorCategory.API)
            raise APIError(f"智能体处理失败: {e}")

    def play_response_audio(self, audio_path: str):
        """播放智能体回复音频"""
        try:
            logger.info(f"开始播放回复音频: {audio_path}")
            self.audio_service.play_result_audio(audio_path)  # 复用主程序播放逻辑
            logger.info("回复音频播放完成")
        except Exception as e:
            logger.error(f"播放回复音频失败: {e}")
            self.error_handler.handle_error(e, ErrorCategory.AUDIO)
            raise AudioError(f"播放回复音频失败: {e}")

    def continuous_interaction(self, chat_processor: ChatProcessor):
        """完整交互流程：录音→转文字→智能体处理→播放回复"""
        try:
            print("开始持续语音交互服务...")
            print("按 Ctrl+C 停止服务")
            
            while True:
                # 1. 录音
                audio_file = self.listen_and_record()
                if not audio_file:
                    print("未录制到有效音频，继续监听...")
                    continue
                
                # 2. 语音转文字
                try:
                    transcript = self.transcribe_audio(audio_file)
                    if transcript:
                        print(f"\n识别结果: {transcript}")
                    else:
                        print("\n未能识别语音内容")
                        continue
                except APIError as e:
                    print(f"\n识别错误: {str(e)}")
                    continue
                
                # 3. 智能体处理（通过 ChatProcessor）
                try:
                    response_audio = self.process_with_agent(transcript, chat_processor)
                except APIError as e:
                    print(f"\n智能体处理错误: {str(e)}")
                    continue
                
                # 4. 播放回复音频
                try:
                    self.play_response_audio(response_audio)
                except AudioError as e:
                    print(f"\n播放音频错误: {str(e)}")
                    continue
                
                # 清理临时文件
                try:
                    os.remove(audio_file)
                    os.remove(response_audio)
                    logger.info(f"已删除临时文件: {audio_file}, {response_audio}")
                except Exception as e:
                    logger.warning(f"无法删除临时文件: {e}")
                
                print("\n继续监听环境声音...")
                
        except KeyboardInterrupt:
            print("\n用户中断服务")
        except Exception as e:
            logger.error(f"持续交互过程中发生错误: {e}")
            raise


def main():
    """初始化资源并启动交互服务"""
    try:
        # 初始化资源管理器（与主程序一致）
        resource_manager = ResourceManager()

        # 获取所有核心资源
        with resource_manager.manage_resource(ResourceType.API_CLIENT) as api_client, \
             resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler, \
             resource_manager.manage_resource(ResourceType.AUDIO_DEVICE) as audio_interface, \
             resource_manager.manage_resource(ResourceType.AUDIO_MANAGER) as audio_mgr:

            # 初始化音频服务（用于播放）
            audio_service = AudioService(audio_interface, audio_mgr)
            
            # 初始化智能体处理器（关键：与主程序共用逻辑）
            chat_processor = ChatProcessor(api_client, audio_service)

            # 初始化音频处理器
            audio_processor = AudioProcessor(
                api_client=api_client,
                error_handler=error_handler,
                audio_service=audio_service,
                device_index=0,  # 根据实际设备调整
                sample_rate=48000
            )

            print("音频交互服务已启动")
            # 启动完整交互流程
            audio_processor.continuous_interaction(chat_processor)

    except KeyboardInterrupt:
        print("用户中断程序")
    except Exception as e:
        print(f"程序发生错误: {e}")
    finally:
        # 清理资源
        if 'resource_manager' in locals():
            resource_manager.cleanup_all()
        print("程序已结束")


if __name__ == "__main__":
    main()