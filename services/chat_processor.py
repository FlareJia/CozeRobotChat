# services/chat_processor.py
import logging
import time
from typing import Optional
from utils.backoff import BackoffManager
from config import Config
from services.audio_service import AudioService

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
        """处理API响应，当返回内容以'...'开头时写入txt文件，包含新内容标记"""
        messages = self.api_client.get_chat_messages(conv_id, chat_id)
        if not messages:
            return None

        # 提取所有回答内容
        content = "\n".join(
            msg.get('content', '')
            for msg in messages
            if msg.get('type') == "answer"
        )

        # 测试print
        logger.info("询问智能体返回的结果为：")
        logger.info(content)

        # 检查内容是否以"..."开头
        if content.startswith("...") and len(content) > 3:
            # 获取"..."后面的内容
            content_to_write = content[3:].strip()
            content = "好的"
            
            try:
                # 写入到txt文件，第一行为布尔值表示有新内容，第二行为实际内容
                with open("shared_content.txt", "w", encoding="utf-8") as f:
                    f.write("True\n")  # 第一行：布尔值表示有新写入
                    f.write(content_to_write)  # 第二行：实际内容
                logger.info("内容已成功写入shared_content.txt文件")
            except Exception as e:
                logger.error(f"写入文件失败: {str(e)}")
        else:
            # 如果不是特殊标记内容，可选择写入False表示无新内容
            try:
                with open("shared_content.txt", "w", encoding="utf-8") as f:
                    f.write("False\n")  # 第一行：布尔值表示无新写入
                logger.info("无特殊内容，已更新标记为False")
            except Exception as e:
                logger.warning(f"更新无新内容标记失败: {str(e)}")

        # 生成音频文件
        return self.api_client.generate_audio(content)
    