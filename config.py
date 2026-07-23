import os
import pyaudio
from dotenv import load_dotenv

load_dotenv()


class Config:
    # ==================== API 配置 ====================
    # DashScope API（阿里云百炼）
    DASHSCOPE_API_KEY = os.getenv("DASHSCOPE_API_KEY")
    # OpenAI 兼容端点
    DASHSCOPE_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"

    # Qwen 模型选择
    QWEN_MODEL = os.getenv("QWEN_MODEL", "qwen-plus")

    # 系统提示词（让 Qwen 输出 <<SPEECH_START>>/<<SPEECH_END>> 标记）
    SYSTEM_PROMPT = (
        '你是一个叫"伯乐"的智能招聘助手机器人。'
        '你需要：\n'
        '1. 用简短、口语化的中文回复用户，控制在 2-3 句话以内\n'
        '2. 把你所有要说的话用 <<SPEECH_START>> 和 <<SPEECH_END>> 包裹起来\n'
        '3. 不要在标记之外输出任何其他文字'
    )

    # ==================== TTS 配置 ====================
    TTS_MODEL = "qwen3-tts-flash"
    TTS_VOICE = os.getenv("TTS_VOICE", "Ethan")  # 年轻男声：Ethan（晨煦）

    AUDIO_SETTINGS = {
        "speed": 1,
        "sample_rate": 24000
    }

    # ==================== 路径配置 ====================
    LOWER_AUDIO_TARGET_PATH = "/home/lab/szhr/CozeRobotChat/records/outputs.wav"
    ROS_WS_PATH = os.path.expanduser("~/szhr/CozeRobotChat/ros_ws")
    OUTPUT_DIR = "outputs"
    RECORD_DIR = "records"

    # ==================== 音频文件命名 ====================
    AUDIO_NAMES = {
        "error_dir": "error_audios",
        "error_mp3": "system_error.mp3",
        "error_wav": "system_error.wav",
        "hello_dir": "hello_audios",
        "hello_mp3": "hello.wav",
        "hello_wav": "hello.wav",
        "hello_mp3_recruitment": "hello_recruitment.mp3",
        "hello_wav_recruitment": "hello_recruitment.wav",
        "hello_mp3_policy": "hello_policy.mp3",
        "hello_wav_policy": "hello_policy.wav",
        "wait_dir": "wait_audios",
        "wait_mp3_recruitment": "wait_recruitment.mp3",
        "wait_wav_recruitment": "wait_recruitment.wav",
        "wait_mp3_policy": "wait_policy.mp3",
        "wait_wav_policy": "wait_policy.wav",
        "reserved_dir": "reserved_audios",
        "bye_dir": "bye_audios",
        "bye_mp3": "bye.mp3",
        "bye_wav": "bye.wav"
    }

    # ==================== 录音参数 ====================
    # 采样率改为 16000（Paraformer ASR 要求）
    RECORD_SETTINGS = {
        "format": pyaudio.paInt16,
        "channels": 1,
        "rate": 16000,          # 原 44100 → 16000（Paraformer 要求）
        "chunk": 4096,
        "threshold": 1000,
        "silence_duration": 2,
        "max_duration": 30
    }

    # ==================== 唤醒词检测参数 ====================
    DETECT_SETTINGS = {
        "format": pyaudio.paInt16,
        "channels": 1,
        "rate": 16000,          # 原 44100 → 16000（Paraformer 要求）
        "chunk": 4096,
        "silence_duration": 0.7,
        "max_duration": 5,
        "threshold": 1500,
        "min_recording_duration_second": 0.4,
    }

    # ==================== 唤醒词 / 结束词 ====================
    WAKE_WORD_SETTINGS = {
        "wake_word": "你好，伯乐",
        "wake_word_buffer": [],
        "wake_word_threshold": 0.5
    }

    BYE_WORD_SETTINGS = {
        "bye_word": "再见，伯乐",
        "bye_word_buffer": [],
        "bye_word_threshold": 0.7
    }

    # ==================== 音频管理 ====================
    AUDIO_FORMATS = [".wav", ".mp3", ".ogg"]

    CLEANUP_CONFIG = {
        'retention_minutes': 10,
        'min_retain_files': 5,
        'cleanup_interval': 10
    }

    # ==================== 功能开关 ====================
    FEATURE_FLAGS = {
        'USE_ASYNC_RESERVED_AUDIO': True
    }

    ENABLE_STREAMING = True  # True = 流式处理，False = 非流式
