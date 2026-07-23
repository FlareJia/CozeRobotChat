# services/api_client.py
import os
import logging
import requests
from datetime import datetime
from typing import Optional, Generator

import dashscope
from dashscope.audio.asr import Recognition
from dashscope import MultiModalConversation
from openai import OpenAI

from config import Config
from utils.paths import PathManager

logger = logging.getLogger(__name__)


class QwenAPIError(Exception):
    """API 调用异常"""

    def __init__(self, message: str, status_code: Optional[int] = None):
        super().__init__(message)
        self.status_code = status_code


class QwenAPIClient:
    """阿里云百炼 API 客户端（ASR + Chat + TTS）"""

    def __init__(self):
        self.api_key = Config.DASHSCOPE_API_KEY
        dashscope.api_key = self.api_key

        # OpenAI 兼容客户端（Qwen Chat）
        self.chat_client = OpenAI(
            api_key=self.api_key,
            base_url=Config.DASHSCOPE_BASE_URL,
        )

    # ==================== ASR：语音转文字 ====================

    def transcribe_audio(self, file_path: str) -> Optional[str]:
        """
        语音转文字（Paraformer 实时识别）
        :param file_path: 音频文件路径（WAV, 16kHz, 单声道）
        :return: 识别出的文字，失败返回 None
        """
        if not os.path.isfile(file_path):
            logger.error(f"音频文件不存在: {file_path}")
            return None

        try:
            recognition = Recognition(
                model='fun-asr-realtime',
                format='wav',
                sample_rate=Config.RECORD_SETTINGS['rate'],
                callback=None,  # None = 阻塞式调用
            )
            result = recognition.call(file_path)

            if result.status_code == 200:
                sentences = result.get_sentence()
                # get_sentence() 返回的是列表，取第一条
                if sentences and isinstance(sentences, list) and len(sentences) > 0:
                    text = sentences[0].get('text', '').strip()
                    if text:
                        logger.info(f"ASR 识别结果: {text}")
                        return text
                logger.warning("ASR 返回空文本")
                return None
            else:
                logger.error(f"ASR 失败 (status={result.status_code}): {result.message}")
                return None

        except Exception as e:
            logger.error(f"语音识别异常: {str(e)}")
            return None

    # ==================== Chat：流式对话 ====================

    def chat_stream(self, messages: list) -> Generator[str, None, None]:
        """
        流式聊天（OpenAI 兼容接口，SSE 流式返回）
        :param messages: [{"role":"system","content":...}, {"role":"user","content":...}]
        :yield: 每次 yield 一段增量文本
        """
        try:
            stream = self.chat_client.chat.completions.create(
                model=Config.QWEN_MODEL,
                messages=messages,
                stream=True,
                temperature=0.7,
                max_tokens=1024,
            )
            for chunk in stream:
                if chunk.choices:
                    delta = chunk.choices[0].delta
                    if delta.content:
                        yield delta.content

        except Exception as e:
            logger.error(f"流式聊天异常: {str(e)}")

    # ==================== Chat：非流式对话 ====================

    def chat(self, messages: list) -> Optional[str]:
        """
        非流式聊天（一次性返回完整结果）
        :param messages: 消息列表
        :return: 完整回复文本
        """
        try:
            response = self.chat_client.chat.completions.create(
                model=Config.QWEN_MODEL,
                messages=messages,
                stream=False,
                temperature=0.7,
                max_tokens=1024,
            )
            if response.choices:
                return response.choices[0].message.content
            return None

        except Exception as e:
            logger.error(f"非流式聊天异常: {str(e)}")
            return None

    # ==================== TTS：文字转语音 ====================

    def generate_audio(
        self,
        text: str,
        voice: str = None,
        sample_rate: int = None,
    ) -> Optional[str]:
        """
        文字转语音（qwen-tts，非实时 HTTP）
        :param text: 要合成的文本
        :param voice: 音色名称（默认 Config.TTS_VOICE）
        :param sample_rate: 输出采样率（默认 24000）
        :return: 生成的 WAV 文件路径，失败返回 None
        """
        if not text or not text.strip():
            logger.error("TTS 输入文本为空")
            return None

        if voice is None:
            voice = Config.TTS_VOICE
        if sample_rate is None:
            sample_rate = Config.AUDIO_SETTINGS["sample_rate"]

        try:
            response = MultiModalConversation.call(
                model=Config.TTS_MODEL,
                api_key=self.api_key,
                text=text.strip(),
                voice=voice,
                language_type="Chinese",
                stream=False,
            )

            # 从响应中提取音频 URL
            audio_url = self._extract_audio_url(response)
            if not audio_url:
                return None

            # 下载并保存音频文件
            return self._download_audio(audio_url)

        except Exception as e:
            logger.error(f"TTS 生成异常: {str(e)}")
            return None

    def _extract_audio_url(self, response) -> Optional[str]:
        """从 MultiModalConversation 响应中提取音频 URL"""
        try:
            output = response.output
            if output is None:
                logger.error("TTS 响应 output 为空")
                return None

            # 兼容 dict 和对象两种格式
            if isinstance(output, dict):
                audio = output.get('audio', {})
                url = audio.get('url', '') if isinstance(audio, dict) else ''
            else:
                audio = getattr(output, 'audio', None)
                url = getattr(audio, 'url', '') if audio else ''

            if not url:
                logger.error(f"TTS 响应中未找到音频 URL，output={output}")
                return None

            return url

        except Exception as e:
            logger.error(f"提取音频 URL 异常: {str(e)}")
            return None

    def _download_audio(self, audio_url: str) -> Optional[str]:
        """下载音频 URL 到本地 outputs 目录"""
        try:
            resp = requests.get(audio_url, timeout=30)
            resp.raise_for_status()

            output_dir = PathManager.safe_join(os.getcwd(), Config.OUTPUT_DIR)
            PathManager.create_dir(output_dir)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"qwen_audio_{timestamp}.wav"
            file_path = os.path.join(output_dir, filename)

            with open(file_path, "wb") as f:
                f.write(resp.content)

            logger.info(f"TTS 音频已保存: {file_path}")
            return file_path

        except requests.RequestException as e:
            logger.error(f"下载音频失败: {str(e)}")
            return None
        except IOError as e:
            logger.error(f"保存音频文件失败: {str(e)}")
            return None
