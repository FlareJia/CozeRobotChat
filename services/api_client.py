import os
import cv2
import json
import requests
import logging
from datetime import datetime
from retrying import retry
from typing import Optional, Dict, List, Union
from urllib.parse import urljoin
from utils.paths import PathManager
from utils.backoff import BackoffManager
from config import Config

logger = logging.getLogger(__name__)


class CozeAPIError(Exception):
    """自定义API异常"""

    def __init__(self, message: str, status_code: Optional[int] = None):
        super().__init__(message)
        self.status_code = status_code


class EnhancedCozeAPIClient:
    API_BASE_V3 = "https://api.coze.cn/v3/"
    API_BASE_V1 = "https://api.coze.cn/v1/"

    def __init__(self, bearer_token: str):
        self.bearer_token = bearer_token
        self.session = requests.Session()
        self._configure_session()
        self.backoff = BackoffManager(initial_delay=2, max_delay=30)
        self.session.verify = True

    def _configure_session(self):
        self.session.headers.update({
            "Authorization": f"Bearer {self.bearer_token}",
            "Content-Type": "application/json",
            "Accept": "application/json"
        })
        # # 添加 SSL 验证选项和超时设置
        # self.session.verify = True
        # self.session.mount('https://', requests.adapters.HTTPAdapter(
        #     max_retries=3,
        #     pool_connections=10,
        #     pool_maxsize=10
        # ))
        self.session.timeout = 30

    def _build_url(self, endpoint: str, version: str = "v3") -> str:
        """构建完整API URL"""
        base_url = self.API_BASE_V3 if version == "v3" else self.API_BASE_V1
        return urljoin(base_url, endpoint)

    @retry(
        stop_max_attempt_number=3,
        wait_exponential_multiplier=1000,
        wait_exponential_max=10000,
        retry_on_exception=lambda e: isinstance(e, CozeAPIError) and 500 <= e.status_code < 600  # 仅重试服务端错误
    )
    def _request(
            self,
            method: str,
            endpoint: str,
            version: str = "v3",
            parse_json: bool = True,  # 新增参数
            **kwargs
    ) -> Optional[Union[Dict, List]]:
        """
        统一请求方法（带重试机制）
        :param method: HTTP方法（GET/POST）
        :param endpoint: API端点路径
        :param version: API版本（v1/v3）
        :return: 解析后的JSON响应或None
        """
        url = self._build_url(endpoint, version)

        try:
            logger.debug(f"Sending {method} request to {url}")
            # 添加 SSL 验证选项和超时设置
            response = self.session.request(
                method,
                url,
                timeout=30,   # 添加超时设置
                verify=True,  # 确保SSL验证
                **kwargs
            )
            response.raise_for_status()
            return response.json() if parse_json else response
        except requests.exceptions.SSLError as e:
            error_msg = f"SSL连接错误: {str(e)}"
            logger.error(error_msg)
            # 添加重试逻辑
            if 'stream' in kwargs:
                kwargs['stream'] = False  # 禁用流式传输
            raise CozeAPIError(error_msg) from e
        except requests.HTTPError as e:
            error_msg = f"HTTP错误 {e.response.status_code} - {e.response.url} - {e.response.text}"
            logger.error(error_msg)
            raise CozeAPIError(error_msg, e.response.status_code) from e
        except requests.RequestException as e:
            error_msg = f"请求失败: {str(e)}"
            logger.error(error_msg)
            raise CozeAPIError(error_msg) from e
        except json.JSONDecodeError as e:
            error_msg = f"无效的JSON响应: {str(e)}"
            logger.error(error_msg)
            raise CozeAPIError(error_msg) from e

    def send_chat_request(self, bot_id: str, user_id: str, content: Dict, stream: bool = True) -> Optional[Dict]:
        """
        发送聊天请求，支持流式响应，返回智能体指令（如拍照指令）
        """
        # 1. 构建URL（包含conversation_id参数，即使为空）
        url = f"{self.API_BASE_V3}chat?conversation_id="

        # 2. 构造请求体（使用official字段additional_messages）
        payload = {
            "bot_id": bot_id,
            "user_id": user_id,
            "stream": stream,
            "auto_save_history": True,
            "additional_messages": [
                {
                    "role": "user",
                    "content": content["text"],
                    "content_type": "text"
                }
            ]
        }

        try:
            response = self.session.post(
                url=url,
                json=payload,
                stream=stream
            )
            response.raise_for_status()

            # 3. 处理流式响应（只保留最后一个完整JSON）
            last_valid_content = ""  # 存储最后一个有效的JSON内容
            is_done = False

            for line in response.iter_lines():
                if line:
                    line_str = line.decode("utf-8").strip()

                    # 标记结束事件
                    if line_str == "event:done":
                        is_done = True
                        continue

                    # 提取智能体回复
                    if ("role\":\"assistant\"" in line_str 
                        and "\"type\":\"answer\"" in line_str 
                        and line_str.startswith("data:")):
                        json_str = line_str[5:].strip()
                        data = json.loads(json_str)
                        # 只保留最后一个content（替换而非追加）
                        last_valid_content = data.get("content", "").replace("\n", "").replace("    ", "")

            # 所有分块处理完毕后，用最后一个有效的JSON解析
            if is_done and last_valid_content:
                try:
                    return json.loads(last_valid_content)
                except json.JSONDecodeError as e:
                    logger.error(f"JSON解析失败，内容: {last_valid_content}，错误: {str(e)}")
            return None

        except Exception as e:
            logger.error(f"API请求失败: {str(e)}", exc_info=True)
        return None

    # 上传到coze服务器
    def upload_image(self, image_path: str) -> Optional[str]:
        try:
            file_name = os.path.basename(image_path)  # 提取实际文件名（如 captured_image.jpg）

            with open(image_path, "rb") as f:
                files = {
                    "file": (
                        file_name,        # 文件名（与 Postman 一致）
                        f,                # 文件对象
                        "image/jpeg"      # MIME 类型（JPG 固定为 image/jpeg，PNG 则为 image/png）
                    )
                }

    

                # 确保不手动设置Content-Type，让requests自动生成（含boundary）
                headers = {
                    "Authorization": f"Bearer {self.bearer_token}",
                    # 移除Content-Type，让requests自动处理（curl也不手动设置具体boundary）
                }

                response = self.session.post(
                    url=f"{self.API_BASE_V1}files/upload",
                    files=files,
                    headers=headers,
                    verify=False  # 临时关闭SSL验证（避免环境问题干扰）
                )
                logger.info(f"上传响应files: {files}")

            # 3. 详细打印响应（用于对比curl）
            logger.info(f"上传响应状态码: {response.status_code}")
            logger.info(f"上传响应头: {response.headers}")
            logger.info(f"上传响应内容: {response.text}")

            response.raise_for_status()
            result = response.json()

            if result.get("code") == 0:
                file_id = result["data"].get("id")
                logger.info(f"上传成功，file_id: {file_id}")
                return file_id
            else:
                logger.error(f"API返回错误: {result.get('msg')}，logid: {result.get('detail', {}).get('logid')}")
                return None

        except Exception as e:
            logger.error(f"上传失败: {str(e)}")
            return None

    # 上传到简历处理服务器
    def upload_image_url(self, image_path: str) -> Optional[str]:
        try:
            file_name = os.path.basename(image_path)  # 提取实际文件名（如 captured_image.jpg）

            with open(image_path, "rb") as f:
                # 2. 显式构造文件字段的元数据（完全匹配curl的格式）
                files = {
                    "file": (
                        file_name,
                        f,
                        "image/jpeg",
                        {"Content-Disposition": f'form-data; name="file"; filename="{file_name}"'}
                    )
                }

                headers ={}
                session = requests.Session()
                response = session.post(
                    url="http://121.40.26.93/api/upload_image",
                    files=files,
                    headers=headers,
                    verify=False
                )
                logger.info(f"上传响应files: {files}")

            # 3. 详细打印响应（用于对比curl）
            logger.info(f"上传响应状态码: {response.status_code}")
            logger.info(f"上传响应头: {response.headers}")
            logger.info(f"上传响应内容: {response.text}")

            response.raise_for_status()
            result = response.json()

            if result.get("success") is True:
                url = result["data"].get("url")
                logger.info(f"上传成功，url: {url}")
                return url
            else:
                logger.error(f"API返回错误: {result.get('msg')}，logid: {result.get('detail', {}).get('logid')}")
                return None

        except Exception as e:
            logger.error(f"上传失败: {str(e)}")
            return None


    def check_chat_status(
            self,
            conversation_id: str,
            chat_id: str
    ) -> Optional[Dict]:
        """
        检查聊天状态
        :param conversation_id: 会话ID
        :param chat_id: 聊天ID
        :return: 状态响应数据
        """
        params = {
            "conversation_id": conversation_id,
            "chat_id": chat_id
        }
        return self._request("GET", "chat/retrieve", params=params)
        #return self._request("GET", "chat/status", params=params)

    def get_chat_messages(self, conversation_id: str, chat_id: str) -> Optional[List[Dict]]:
        params = {
            "conversation_id": conversation_id,
            "chat_id": chat_id,
            # "limit": 10  # 限制条数
        }
        # try:
        #     # 端点从"conversation/messages"改为"chat/messages"
        #     response = self._request("GET", "chat/messages", params=params)
        #     logger.debug(f"get_chat_messages响应: {response}")
        #     # 消息通常在"data"字段（根据v3 API规范）
        #     return response.get("data", []) if response else []
        # except Exception as e:
        #     logger.error(f"获取消息失败: {str(e)}")
        #     return []
        while True:
            try:
                response = self._request("GET", "chat/message/list", params=params)
                return response.get("data", []) if response else []
            except CozeAPIError:
                self.backoff.wait()

    def generate_audio(
            self,
            text: str,
            voice_id: int = Config.VOICE_ID,
            speed: float = Config.AUDIO_SETTINGS["speed"],
            sample_rate: int = Config.AUDIO_SETTINGS["sample_rate"]
    ) -> Optional[str]:
        """
        生成语音文件（正确解析二进制响应）
        :param text: 输入文本
        :param voice_id: 语音模型ID
        :param speed: 语速（0.5-2.0）
        :param sample_rate: 采样率
        :return: 生成的音频文件路径
        """
        payload = {
            "input": text,
            "voice_id": voice_id,
            "response_format": "wav",
            "speed": speed,
            "sample_rate": sample_rate
        }

        try:
            # 关键修改1：直接获取原始响应内容
            response = self._request(
                "POST",
                "audio/speech",
                version="v1",
                json=payload,
                parse_json=False  # 添加自定义参数
            )

            # 关键修改2：验证二进制内容
            if not response or not isinstance(response.content, bytes):
                logger.error("无效的音频响应")
                return None

            # 关键修改3：直接保存音频内容
            return self._save_audio_file(response.content)

        except CozeAPIError as e:
            logger.error(f"音频生成失败: {str(e)}")
            return None

    def transcribe_audio(self, file_path: str) -> Optional[str]:
        """
        语音转文字（严格遵循Coze API格式要求）
        """
        if not os.path.isfile(file_path):
            logger.error(f"音频文件不存在: {file_path}")
            return None

        try:
            # 验证文件扩展名
            ext = os.path.splitext(file_path)[1].lower().lstrip('.')
            if ext not in ['wav', 'mp3', 'ogg']:
                logger.error(f"不支持的音频格式: {ext}")
                return None

            # 验证文件大小（20MB限制）
            file_size = os.path.getsize(file_path)
            if file_size > 20 * 1024 * 1024:
                logger.error(f"文件超过大小限制: {file_size / 1024 / 1024:.2f}MB")
                return None

            # 动态设置MIME类型
            mime_type = {
                'wav': 'audio/wav',
                'mp3': 'audio/mpeg',
                'ogg': 'audio/ogg'
            }[ext]

            # 打印调试信息
            logger.debug(f"准备上传文件: {file_path} (大小: {file_size} bytes, 类型: {mime_type})")

            with open(file_path, "rb") as audio_file:
                files = {
                    'file': (
                        os.path.basename(file_path),  # 保留原始文件名
                        audio_file,
                        mime_type
                    )
                }
                # 明确设置headers覆盖全局配置
                response = self._request(
                    "POST",
                    "audio/transcriptions",
                    version="v1",
                    files=files,
                    headers={"Content-Type": None}  # 关键修复点
                )

                if not response:
                    logger.error("语音识别API返回空响应")
                    return None

                # 添加类型检查
                if not isinstance(response, dict):
                    logger.error("API响应格式无效")
                    return None

                # 检查响应中的状态码
                code = response.get("code")
                if code is None:
                    logger.error("响应中缺少状态码")
                    return None

                if code != 0:
                    error_msg = response.get("msg", "未知错误")
                    logger.error(f"转录失败: {error_msg}")
                    return None

                # 获取数据部分
                data = response.get("data", {})
                if not isinstance(data, dict):
                    logger.error("响应数据格式无效")
                    return None

                # 获取文本内容
                text = data.get("text")
                if not text:
                    logger.error("响应中缺少文本内容")
                    return None

                return text

        except IOError as e:
            logger.error(f"文件操作失败: {str(e)}")
            return None
        except CozeAPIError as e:
            logger.error(f"语音识别API调用失败: {str(e)}")
            return None
        except Exception as e:
            logger.error(f"语音识别过程中发生未知错误: {str(e)}")
            return None

    def _save_audio_file(self, content: bytes) -> str:
        """保存音频文件到指定目录"""
        output_dir = PathManager.safe_join(os.getcwd(), "outputs")
        PathManager.create_dir(output_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"coze_audio_{timestamp}.wav"
        file_path = os.path.join(output_dir, filename)

        try:
            with open(file_path, "wb") as f:
                f.write(content)
            logger.info(f"音频文件已保存: {file_path}")
            return file_path
        except IOError as e:
            logger.error(f"文件保存失败: {str(e)}")
            return ""