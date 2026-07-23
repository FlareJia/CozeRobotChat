import logging
import json
import os
import re
import subprocess
from typing import Optional
from config import Config
from services.audio_service import AudioService
logger = logging.getLogger(__name__)
from utils.file_transfer import File_transfer

class ChatProcessor:
    """对话流程处理器，协调API调用和音频管理"""

    def __init__(self, api_client, audio_service: AudioService):
        """
        :param api_client: QwenAPIClient 实例
        :param audio_service: AudioService 实例
        """
        self.api_client = api_client
        self.audio_service = audio_service
        self.config = Config()

    def process_query(self, query: str) -> Optional[str]:
        """
        处理用户查询
        :param query: 用户查询文本
        :return: 生成的音频文件路径
        """
        try:
            # 构建消息
            messages = [
                {"role": "system", "content": Config.SYSTEM_PROMPT},
                {"role": "user", "content": self._format_query(query)}
            ]

            # 播放等待音频
            self.audio_service.play_wait_audio()

            # 发送聊天请求（非流式，直接返回结果）
            response_text = self.api_client.chat(messages)
            if not response_text:
                self.audio_service.stop_audio()
                return None

            # 停止等待音频
            self.audio_service.stop_audio()

            # 处理响应
            return self._handle_response(response_text)

        except Exception as e:
            logger.error(f"处理查询失败: {str(e)}")
            self.audio_service.stop_audio()
            return None

    def _format_query(self, query: str) -> str:
        """格式化查询内容"""
        return f"{query} 精简且快速的输出内容"

    def _handle_response(self, response_text: str) -> Optional[str]:
        """处理API响应，整合获取回答、音频转换和传输逻辑"""
        # 1. 解析智能体的JSON回答
        answer_json = self._parse_agent_answer(response_text)
        if not answer_json:
            return None

        # 2. 根据ismove判断是否执行rosservice命令
        if answer_json.get('ismove', False):
            move_content = answer_json.get('move', 'none')
            self._execute_rosservice(move_content)

        # 3. 将speech内容转换为音频
        audio_path = self._convert_answer_to_audio(answer_json)
        if not audio_path:
            return None  # 音频生成失败则返回

        # 4. 自动传输音频到下位机
        file_transfer = File_transfer(self.config)
        transfer_success = file_transfer._transfer_audio_to_lower(audio_path)
        if not transfer_success:
            logger.warning("音频传输失败，但音频文件已生成")

        return audio_path

    def _execute_rosservice(self, action: str) -> None:
        """
        通过subprocess执行rosservice命令
        :param action: 要执行的动作（如"击掌"）
        """
        try:
            command = [
                "rosservice",
                "call",
                "/coze_execute_arm_action",
                f'"{action}"'
            ]

            logger.info(f"执行命令: {' '.join(command)}")

            result = subprocess.run(
                command,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            logger.info(f"rosservice执行成功，输出: {result.stdout}")
            print(f"动作执行成功: {action}")

        except subprocess.CalledProcessError as e:
            logger.error(f"rosservice执行失败，错误码: {e.returncode}, 错误信息: {e.stderr}")
        except Exception as e:
            logger.error(f"执行rosservice时发生错误: {str(e)}")

    def _parse_agent_answer(self, response_text: str) -> Optional[dict]:
        """
        解析智能体的 JSON 格式回答
        :param response_text: Qwen 返回的原始文本
        :return: 包含 speech、ismove、move 的字典，或 None
        """
        try:
            logger.info(f"智能体原始回答：{response_text}")

            # 尝试直接解析 JSON
            try:
                answer_json = json.loads(response_text)
            except json.JSONDecodeError:
                # 尝试提取 JSON 块（可能在 markdown 代码块中）
                json_match = re.search(r'```(?:json)?\s*\n?(.*?)\n?```', response_text, re.DOTALL)
                if json_match:
                    answer_json = json.loads(json_match.group(1))
                else:
                    # 尝试找到花括号包裹的 JSON
                    brace_match = re.search(r'\{.*\}', response_text, re.DOTALL)
                    if brace_match:
                        answer_json = json.loads(brace_match.group(0))
                    else:
                        logger.error("无法从回答中提取 JSON")
                        return None

            # 验证 JSON 字段
            required_fields = ["image", "ismove", "move", "speech"]
            if not all(field in answer_json for field in required_fields):
                logger.error("返回的JSON缺少必要字段")
                return None

            logger.info("智能体返回的解析结果：")
            logger.info(f"speech: {answer_json['speech']}")
            logger.info(f"ismove: {answer_json['ismove']}, move: {answer_json['move']}")
            return answer_json

        except Exception as e:
            logger.error(f"解析智能体回答失败：{str(e)}")
            return None

    def _convert_answer_to_audio(self, answer_json: dict) -> Optional[str]:
        """
        将JSON中的speech内容转换为音频
        :param answer_json: 包含speech字段的字典
        :return: 音频文件路径或None
        """
        try:
            speech_text = answer_json.get('speech', '').strip()
            if not speech_text:
                logger.error("speech字段为空，无法生成音频")
                return None

            # 调用 Qwen TTS 生成音频
            audio_path = self.api_client.generate_audio(speech_text)
            if not audio_path:
                logger.error("音频生成失败")
                return None

            logger.info(f"音频文件已生成：{audio_path}")
            return audio_path

        except Exception as e:
            logger.error(f"回答转音频失败：{str(e)}")
            return None
        



