# services/chat_processor.py
import os
import logging
import time
from typing import Optional
from utils.backoff import BackoffManager
from config import Config
from services.audio_service import AudioService
from typing import List, Dict, Optional
import json

logger = logging.getLogger(__name__)


class ChatProcessor:
    """对话流程处理器，协调API调用和音频管理"""

    # def __init__(self, config, api_client, audio_service: AudioService, camera_service):
    #     """
    #     :param api_client: EnhancedCozeAPIClient 实例
    #     :param audio_service: AudioService 实例
    #     """
    #     self.config = config
    #     self.api_client = api_client
    #     self.audio_service = audio_service
    #     self.camera_service = camera_service 
    #     self.backoff = BackoffManager()
    #     self.config = Config()

    def __init__(self, config, api_client, camera_service, audio_service: AudioService):
        """
        :param api_client: EnhancedCozeAPIClient 实例
        :param audio_service: AudioService 实例
        """
        self.config = Config()  # 配置项（如之前的 Config 实例）
        self.api_client = api_client  # API 客户端
        self.camera_service = camera_service  # 相机服务（关键：新增此参数）
        self.audio_service = audio_service
        self.backoff = BackoffManager()

    # def get_raw_response(self, query: str) -> Optional[str]:
    #     """获取智能体原始响应（JSON字符串）"""
    #     # 格式化查询内容为字典（匹配 api_client 要求的格式）
    #     formatted_query = {
    #         "text": query,
    #         "has_image": False  # 此时还未涉及图片，默认false
    #     }
    #     # 调用API时传递格式化后的字典
    #     response = self.api_client.send_chat_request(
    #         self.config.BOT_ID,
    #         self.config.USER_ID,
    #         formatted_query  # 这里从纯文本改为字典
    #     )
    #     # 根据实际API响应格式调整返回路径（确保能拿到包含JSON的原始响应）
    #     # 例如：如果响应是 {"data": {"content": "JSON字符串"}}, 则保持不变
    #     return response.get("data", {}).get("content")

    # services/chat_processor.py 中修改
    # services/chat_processor.py 中修改
    # services/chat_processor.py


    def get_raw_response(self, query: str) -> Optional[Dict]:
        """获取智能体的拍照指令（替换原轮询逻辑，直接用流式响应）"""
        return self.api_client.send_chat_request(
            bot_id=self.config.BOT_ID,  # 用于返回拍照指令的智能体ID
            user_id=self.config.USER_ID,
            content={"text": query}
        )
    # def get_raw_response(self, query: str) -> Optional[str]:
    #     """获取智能体原始响应（JSON字符串）"""
    #     response = self.api_client.send_chat_request(
    #         self.config.BOT_ID,
    #         self.config.USER_ID,
    #         query
    #     )
    #     return response.get("data", {}).get("content")  # 根据实际API响应格式调整

    # 1111
    def process_query(self, query: str, image_path: Optional[str] = None) -> Optional[str]:
        """
        处理用户查询（支持传递图片路径）
        :param query: 用户查询文本
        :param image_path: 图片保存路径（可选，有图片时传递）
        :return: 生成的音频文件路径
        """
        try:
            # 格式化查询内容（包含图片信息如果存在）
            formatted_query = self._format_query(query, image_path)
            
            # 发送聊天请求（传递包含图片信息的查询）
            response = self.api_client.send_chat_request(
                self.config.BOT_ID,
                self.config.USER_ID,
                formatted_query
            )
            if not response:
                return None

          
            if not image_path:  # 当有图片路径时，不播放等待音频
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


    # 新增/修改辅助方法：格式化包含图片的查询
    def _format_query(self, query: str, image_path: Optional[str] = None) -> dict:
        """
        格式化查询内容，支持包含图片路径信息
        """
        formatted = {
            "text": query,
            "has_image": False
        }
        # 如果有图片路径，添加到查询中
        if image_path and os.path.exists(image_path):
            formatted.update({
                "has_image": True,
                "image_path": image_path,
                "image_filename": os.path.basename(image_path)
            })
        return formatted
    
    # def process_image_query(self, query: str, image_path: str) -> Optional[str]:
    #     """上传图片并调用工作流处理"""
    #     # 1. 上传图片获取file_id
    #     file_id = self.api_client.upload_image(image_path)
    #     if not file_id:
    #         logger.error("未获取到file_id，终止图片处理")
    #         return None

    #     # 2. 向工作流发送带图片的请求
    #     workflow_response = self.api_client.send_chat_request(
    #         bot_id=self.config.WORKFLOW_BOT_ID,  # 工作流对应的智能体ID
    #         user_id=self.config.USER_ID,
    #         content={
    #             "text": query,
    #             "additional_messages": [
    #                 {
    #                     "role": "user",
    #                     "content": [
    #                         {"type": "text", "text": query},
    #                         {"type": "image", "file_id": file_id}
    #                     ],
    #                     "content_type": "multimodal"
    #                 }
    #             ]
    #         }
    #     )

    #     # 3. 生成结果音频（复用你的TTS逻辑）
    #     if workflow_response and "content" in workflow_response:
    #         result_text = workflow_response["content"]
    #         return self.audio_service.text_to_speech(result_text)
    #     return None

    def process_image_query(self, query: str, image_path: str, url_or_id: True) -> Optional[str]:
        """调用工作流接口处理图片，严格按文档参数构造请求"""
            # 新增：检查上传开关
        if not self.config.FEATURE_FLAGS.get('ENABLE_UPLOAD_IMAGE', False):
            logger.info("图片上传功能已禁用（通过FEATURE_FLAGS控制）")
            return None  # 或返回提示信息，如"图片上传功能暂未开放"

        
        if url_or_id:
            url = self.api_client.upload_image_url(image_path)
            logger.info("通过url上传图片到工作流")
            if not url:
                logger.error("未获取到有效url，终止处理")
                return None
            
                    # 2. 构造工作流请求参数（按文档规范）
            workflow_payload = {
                "workflow_id": self.config.WORKFLOW_BOT_ID,  # 必须：工作流ID（从URL获取）
                "parameters": {  # 工作流输入参数，包含图片和文本
                    "input": query,  # 文本查询（如"识别场景"）
                    "image": url  # 图片参数（需JSON序列化字符串）
                },
                "is_async": False  # 同步运行（免费版支持）
            }

            try:
                response = self.api_client.session.post(
                    url=f"{self.api_client.API_BASE_V1}workflow/run",  # 工作流接口
                    headers={
                        "Authorization": f"Bearer {self.api_client.bearer_token}",
                        "Content-Type": "application/json"
                    },
                    json=workflow_payload
                )
                response.raise_for_status()
                result = response.json()

                # 4. 解析工作流结果（成功时code=0）
                if result.get("code") != 0:
                    logger.error(f"工作流执行失败: {result.get('msg')}，logid: {result.get('detail', {}).get('logid')}")
                    return None
                
                # 关键修正：反序列化data字段的JSON字符串（因为data是字符串，不是字典）
                data_str = result.get("data", "")  # 获取原始data字符串（如"{\"content_type\":1,\"data\":\"图片内容...\"}"）
                if not data_str:
                    logger.error("工作流返回data为空")
                    return None
                
                # 反序列化为字典
                data_dict = json.loads(data_str)
                
                # 提取实际内容（根据响应结构，内容在data_dict的"data"字段中）
                workflow_result = data_dict.get("data", "")
                logger.info(f"工作流处理结果: {workflow_result}")
                return workflow_result

            except json.JSONDecodeError:
                logger.error(f"工作流data解析失败（非JSON字符串）: {data_str}")
                return None
            except Exception as e:
                logger.error(f"工作流调用失败: {str(e)}，响应: {response.text if 'response' in locals() else '无'}")
                return None



        else:
                # 1. 上传图片获取file_id
            file_id = self.api_client.upload_image(image_path)
            if not file_id:
                logger.error("未获取到有效file_id，终止处理")
                return None



            # 2. 构造工作流请求参数（按文档规范）
            workflow_payload = {
                "workflow_id": self.config.WORKFLOW_BOT_ID,  # 必须：工作流ID（从URL获取）
                "parameters": {  # 工作流输入参数，包含图片和文本
                    "input": query,  # 文本查询（如"识别场景"）
                    "image": json.dumps({"file_id": file_id})  # 图片参数（需JSON序列化字符串）
                },
                "bot_id": self.config.BOT_ID,  # 可选：关联的智能体ID（若工作流需要）
                "is_async": False  # 同步运行（免费版支持）
            }

            # 3. 调用工作流接口
            try:
                response = self.api_client.session.post(
                    url=f"{self.api_client.API_BASE_V1}workflow/run",  # 工作流接口
                    headers={
                        "Authorization": f"Bearer {self.api_client.bearer_token}",
                        "Content-Type": "application/json"
                    },
                    json=workflow_payload
                )
                response.raise_for_status()
                result = response.json()

                # 4. 解析工作流结果（成功时code=0）
                if result.get("code") != 0:
                    logger.error(f"工作流执行失败: {result.get('msg')}，logid: {result.get('detail', {}).get('logid')}")
                    return None
                
                # 假设工作流结果在data.output中（根据实际工作流输出调整）
                workflow_result = result.get("data", {}).get("output", "")
                logger.info(f"工作流处理结果: {workflow_result}")
                return workflow_result

            except Exception as e:
                logger.error(f"工作流调用失败: {str(e)}，响应: {response.text if 'response' in locals() else '无'}")
                return None

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

    def _handle_response(self, conversation_id: str, chat_id: str) -> Optional[str]:
        """从对话中提取工作流的输出结果"""
        # 1. 调用get_chat_messages获取完整消息列表（复用你的已有方法）
        messages = self.api_client.get_chat_messages(conversation_id, chat_id)
        if not messages:
            logger.error("工作流未返回任何消息")
            self.audio_service.play_error_audio("未获取到分析结果")
            return None

        # 2. 过滤出智能体（assistant角色）的回复（工作流的输出在这里）
        # 注意：消息可能是直接列表，或嵌套在'data'/'items'中，根据实际结构调整
        assistant_messages = []
        for msg in messages:
            # 兼容消息可能的嵌套结构（如msg是{'data': {...}}）
            actual_msg = msg.get("data", msg) if isinstance(msg, dict) else msg
            if isinstance(actual_msg, dict) and actual_msg.get("role") == "assistant":
                assistant_messages.append(actual_msg)

        if not assistant_messages:
            logger.error(f"未找到智能体的回复，消息列表: {messages}")
            self.audio_service.play_error_audio("未找到分析结果")
            return None

        # 3. 提取工作流输出的文本内容（假设content是纯文本或JSON字符串）
        result_msg = assistant_messages[0]
        result_content = result_msg.get("content", "")
        if not result_content:
            logger.error("智能体回复内容为空")
            self.audio_service.play_error_audio("分析结果为空")
            return None

        logger.info(f"工作流处理结果: {result_content}")
        return self.api_client.generate_audio(result_content)  # 返回结果用于播放

        #content += " 感谢您的使用，更多岗位信息请进入展馆查看哦。"
