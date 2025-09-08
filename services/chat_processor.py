# services/chat_processor.py
import os
import logging
import time
import json
import os
import subprocess  # 新增：导入subprocess模块
from typing import Optional
from utils.backoff import BackoffManager
from config import Config
from services.audio_service import AudioService
from utils.file_transfer import File_transfer

from typing import List, Dict, Optional
import json

logger = logging.getLogger(__name__)


class ChatProcessor:
    """对话流程处理器，协调API调用和音频管理"""

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

    def get_raw_response(self, query: str) -> Optional[Dict]:
        """获取智能体的拍照指令（替换原轮询逻辑，直接用流式响应）"""
        return self.api_client.send_chat_request(
            bot_id=self.config.BOT_ID,  # 用于返回拍照指令的智能体ID
            user_id=self.config.USER_ID,
            content={"text": query}
        )


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

    def _handle_response(self, conv_id: str, chat_id: str) -> Optional[str]:
        """处理API响应，整合获取回答、音频转换和传输逻辑"""
        # 1. 获取智能体的JSON回答
        answer_json = self._get_agent_answer(conv_id, chat_id)
        if not answer_json:
            return None
        
        # 2. 根据ismove判断是否执行rosservice命令
        if answer_json.get('ismove', False):
            move_content = answer_json.get('move', 'none')
            self._execute_rosservice(move_content)
        


        # 3. 将speech内容转换为音频
        audio_path = self._convert_answer_to_audio(answer_json)
        logger.info(audio_path)
        if not audio_path:
            return None  # 音频生成失败则返回
        file_transfer = File_transfer()
        # 4. 自动传输音频到下位机
        transfer_success = file_transfer._transfer_audio_to_lower(audio_path)
        if not transfer_success:
            logger.warning("音频传输失败，但音频文件已生成")
        
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
                "/execute_arm_action", 
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
                answer_json = json.loads(answer_messages[0])  # 取第一条回答消息
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
        




        



