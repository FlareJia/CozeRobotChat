import logging
import time
import json
import os
import subprocess  # 新增：导入subprocess模块
from typing import Optional
from utils.backoff import BackoffManager
from config import Config
from core.interfaces.unified_interfaces import IAPIClient, IAudioService, IChatProcessor
logger = logging.getLogger(__name__)
from utils.file_transfer import File_transfer

class ChatProcessor(IChatProcessor):
    """对话流程处理器，协调API调用和音频管理"""

    def __init__(self, api_client: IAPIClient, audio_service: IAudioService):
        """
        :param api_client: IAPIClient 实例
        :param audio_service: IAudioService 实例
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
        """处理API响应，整合获取回答、音频转换和传输逻辑"""
        # 1. 获取智能体的JSON回答
        answer_json = self._get_agent_answer(conv_id, chat_id)
        if not answer_json:
            return None
        
        # 2. 根据ismove判断是否执行rosservice命令
        # todo ismove应该改为is_move
        if answer_json.get('ismove', False):
            move_content = answer_json.get('move', 'none')
            self._execute_rosservice(move_content)
        
        # 3. 将speech内容转换为音频
        audio_path = self._convert_answer_to_audio(answer_json)
        if not audio_path:
            return None  # 音频生成失败则返回
        
        # 4. 自动传输音频到下位机
        # 使用公共接口替代内部方法调用
        try:
            from ros_ws.src.file_transfer.scripts.file_transfer_api import upload_to_lower
            lower_target_path = os.path.join(self.config.RECORD_DIR, Config.AUDIO_NAMES["output_wav"])
            success, message, file_size = upload_to_lower(audio_path, lower_target_path)
            if not success:
                logger.warning(f"音频传输失败: {message}，但音频文件已生成")
        except ImportError as e:
            logger.error(f"无法导入文件传输模块: {e}")
        except Exception as e:
            logger.error(f"音频传输过程中发生错误: {e}")
        
        return audio_path  # 即使传输失败，仍返回本地音频路径（可选）
    
    
    def _execute_rosservice(self, action: str) -> None:
        """
        通过subprocess执行rosservice命令
        :param action: 要执行的动作（如"击掌"）
        """
        try:
            # 构建命令：rosservice call /execute_arm_action "动作内容"
            command = [
                "rosservice", 
                "call", 
                "/coze_execute_arm_action", 
                f'"{action}"'  # 确保动作内容带引号
            ]
            
            logger.info(f"执行命令: {' '.join(command)}")
            
            # 执行命令并捕获输出
            result = subprocess.run(
                command,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True  # 输出为字符串而非字节
            )
            
            # 记录成功信息
            logger.info(f"rosservice执行成功，输出: {result.stdout}")
            print(f"动作执行成功: {action}")
            
        except subprocess.CalledProcessError as e:
            # 命令执行失败（返回非0状态码）
            logger.error(f"rosservice执行失败，错误码: {e.returncode}, 错误信息: {e.stderr}")
        except Exception as e:
            # 其他异常（如命令不存在）
            logger.error(f"执行rosservice时发生错误: {str(e)}")

    def _get_agent_answer(self, conv_id: str, chat_id: str) -> Optional[dict]:
        """
        从智能体获取JSON格式回答，并提取关键字段
        :return: 包含speech、ismove、move的字典，或None
        """
        try:
            # 调用API获取消息列表（智能体返回的内容在消息中）
            messages = self.api_client.get_chat_messages(conv_id, chat_id)
            if not messages:
                logger.error("未获取到消息列表")
                return None

            # 提取智能体的回答消息（type="answer"）
            answer_messages = [
                msg.get('content', '') 
                for msg in messages 
                if msg.get('type') == "answer"
            ]
            if not answer_messages:
                logger.error("未提取到智能体的回答消息")
                return None

            # 解析JSON格式的回答内容（假设消息内容是纯JSON字符串）
            try:
                logger.info(f"智能体回答：{answer_messages[0]}")
                answer_json = json.loads(answer_messages[0])
                 # 取第一条回答消息
            except json.JSONDecodeError as e:
                logger.error(f"智能体回答不是有效的JSON格式：{str(e)}")
                return None

            # 验证JSON字段是否完整
            required_fields = ["image", "ismove", "move", "speech"]
            if not all(field in answer_json for field in required_fields):
                logger.error("智能体返回的JSON缺少必要字段")
                return None

            logger.info("智能体返回的解析结果：")
            logger.info(f"speech: {answer_json['speech']}")
            logger.info(f"ismove: {answer_json['ismove']}, move: {answer_json['move']}")
            return answer_json

        except Exception as e:
            logger.error(f"获取智能体回答失败：{str(e)}")
            return None
        

    def _convert_answer_to_audio(self, answer_json: dict) -> Optional[str]:
        """
        将JSON中的speech内容转换为音频
        :param answer_json: 包含speech字段的字典
        :return: 音频文件路径或None
        """
        try:
            # 提取speech内容
            speech_text = answer_json.get('speech', '').strip()
            if not speech_text:
                logger.error("speech字段为空，无法生成音频")
                return None

            # 调用API生成音频
            audio_path = self.api_client.generate_audio(speech_text)
            if not audio_path:
                logger.error("音频生成失败")
                return None

            logger.info(f"音频文件已生成：{audio_path}")
            return audio_path

        except Exception as e:
            logger.error(f"回答转音频失败：{str(e)}")
            return None
        



