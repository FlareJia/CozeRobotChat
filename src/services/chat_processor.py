# services/chat_processor.py
import logging
import time
from typing import Optional
from src.utils.backoff import BackoffManager
from src.config import Config
from src.services.audio_service import AudioService

logger = logging.getLogger(__name__)


class ChatProcessor:
    """对话流程处理器，协调API调用和音频管理"""

    def __init__(self, api_client, audio_service: AudioService):
        """
        :param api_client: EnhancedCozeAPIClient 实例
        :param audio_service: AudioService 实例
        """
        self.api_client = api_client
        self.audio_service = audio_service
        self.backoff = BackoffManager()
        self.config = Config()

    def process_query(self, query: str) -> Optional[str]:
        """
        处理用户查询
        :param query: 用户查询文本
        :return: 生成的音频文件路径
        """
        try:
            # 发送聊天请求
            response = self.api_client.send_chat_request(
                self.config.BOT_ID,
                self.config.USER_ID,
                self._format_query(query)
            )
            if not response:
                return None

            # todo
            # 播放等待音频
            self.audio_service.play_wait_audio()

            # 提取对话ID
            chat_id = response.get('data', {}).get('id')
            conversation_id = response.get('data', {}).get('conversation_id')
            if not chat_id or not conversation_id:
                logger.error("无法获取对话ID")
                return None

            # 等待处理完成
            if self._wait_for_completion(conversation_id, chat_id):
                # 停止等待音频播放
                self.audio_service.stop_audio()
                return self._handle_response(conversation_id, chat_id)

            return None

        except Exception as e:
            logger.error(f"处理查询失败: {str(e)}")
            # 确保停止等待音频播放
            self.audio_service.stop_audio()
            return None

    def _format_query(self, query: str) -> str:
        """格式化查询内容"""
        # todo
        return f"{query} 精简且快速的输出内容"

    def _wait_for_completion(self, conv_id: str, chat_id: str) -> bool:
        """等待对话完成"""
        self.backoff.reset()
        start_time = time.time()

        while time.time() - start_time < 120:
            try:
                status = self.api_client.check_chat_status(conv_id, chat_id)
                if status.get('data', {}).get('status') == "completed":
                    return True
                self.backoff.wait()
            except Exception as e:
                logger.error(f"状态检查失败: {str(e)}")
                self.backoff.wait()
        return False

    def _handle_response(self, conv_id: str, chat_id: str) -> Optional[str]:
        """处理API响应"""
        messages = self.api_client.get_chat_messages(conv_id, chat_id)
        if not messages:
            return None

        # 提取所有回答内容
        content = "\n".join(
            msg.get('content', '')
            for msg in messages
            if msg.get('type') == "answer"
        )

        #content += " 感谢您的使用，更多岗位信息请进入展馆查看哦。"

        # 测试print
        logger.info("询问智能体返回的结果为：")
        logger.info(content)

        # 生成音频文件
        return self.api_client.generate_audio(content)