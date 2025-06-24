import os
import pyaudio
from dotenv import load_dotenv

load_dotenv()


class Config:
    # API 配置
    BEARER_TOKEN = os.getenv("BEARER_TOKEN")
    BOT_ID = os.getenv("BOT_ID")
    USER_ID = os.getenv("USER_ID")

    # 音频合成参数
    # VOICE_ID = 7426720361733177353    # 男声
    # VOICE_ID = 7426720361753903141      # 爽快思思
    VOICE_ID = 7468512265134899251      # 知性女音
    AUDIO_SETTINGS = {
        "speed": 1,
        "sample_rate": 16000
    }

    AUDIO_NAMES = {
        "error_dir": "error_audios",
        "error_mp3": "system_error.mp3",
        "hello_dir": "hello_audios",
        "hello_mp3": "hello.mp3",
        "hello_mp3_recruitment": "hello_recruitment.mp3",
        "hello_mp3_policy": "hello_policy.mp3",
        "wait_dir": "wait_audios",
        "wait_mp3_recruitment": "wait_recruitment.mp3",
        "wait_mp3_policy": "wait_policy.mp3",
        "reserved_dir": "reserved_audios",
        "bye_dir": "bye_audios",
        "bye_mp3": "bye.mp3"
    }

    # 路径配置
    OUTPUT_DIR = "outputs"
    RECORD_DIR = "records"


    # 提问录音参数
    RECORD_SETTINGS = {
        "format": pyaudio.paInt16,  # 采样格式（16位）
        "channels": 1,          # 单声道（2为立体声）
        "rate": 44100,          # 提高采样率（原16000太低）
        "chunk": 4096,          # 增大缓冲区块（原1024太小）
        "threshold": 1000,      # 降低阈值提高灵敏度，测试1000
        "silence_duration": 2,  # 延长静默判断时间
        "max_duration": 30      # 最大录音时长（秒）
    }

    # 检测录音参数
    DETECT_SETTINGS = {
        "format": pyaudio.paInt16,  # 采样格式（16位）
        "channels": 1,  # 单声道（2为立体声）
        "rate": 44100,  # 提高采样率（原16000太低）
        "chunk": 4096,  # 增大缓冲区块（原1024太小）
        "silence_duration": 0.5,   # 检测延长静默判断时间
        "max_duration": 5,       # 检测最大录音时长（秒）
        "threshold": 2500,        # 检测-声音阈值灵敏度，测试3000
        "min_recording_duration_second": 0.4,    # 检测-录音检测最小时间（秒）
    }


    WAKE_WORD_SETTINGS = {
        "wake_word": "你好，伯乐",
        "wake_word_buffer": [],
        "wake_word_threshold": 0.5
    }

    BYE_WORD_SETTINGS = {
        "bye_word": "再见，伯乐",
        "bye_word_buffer": [],
        "bye_word_threshold": 0.5
    }


    # 音频管理配置
    # MAX_AUDIO_FILES = 100  # 最大保留文件数
    AUDIO_FORMATS = [".wav", ".mp3", ".ogg"]

    # 文件清理配置
    CLEANUP_CONFIG = {
        'retention_minutes': 10,  # 文件保留分钟数
        'min_retain_files': 5,  # 每个目录至少保留最新文件数
        'cleanup_interval': 10  # 清理间隔分钟数
    }