import json
import logging
import time
import os
import threading
from difflib import SequenceMatcher
from contextlib import contextmanager
from config import Config
from services.chat_processor import ChatProcessor
from services.audio_service import AudioService
from services.keyboard_service import KeyboardService
from utils.paths import PathManager
from services.scheduler import CleanupScheduler
from services.error_handler import ErrorCategory
from services.exceptions import AudioError, APIError
from services.resource_manager import ResourceManager, ResourceType
from services.camera_service import CameraService

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("logs/app.log"),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)


# 计时上下文管理器
@contextmanager
def time_recorder(step_name):
    start_time = time.perf_counter()
    try:
        yield
    finally:
        elapsed = time.perf_counter() - start_time
        logger.info(f"[性能监控] {step_name}耗时: {elapsed:.3f}秒")


def _calculate_similarity(text1: str, text2: str) -> float:

    return SequenceMatcher(None, text1, text2).ratio()


def _is_bye_word_match(text: str) -> bool:

    if not text:
        return False

    # 计算相似度
    similarity = _calculate_similarity(text, Config.BYE_WORD_SETTINGS["bye_word"])
    logger.info(f"文本相似度: {similarity:.2f}")

    return similarity >= Config.BYE_WORD_SETTINGS["bye_word_threshold"]


def detect_bye_word(text: str) -> bool:
    if text and _is_bye_word_match(text):
        logger.info("相似度检测，检测到结束词！")
        return True
    if text and Config.BYE_WORD_SETTINGS["bye_word"] in text:
        logger.info("全量in检测，检测到结束词！")
        return True
    return False


def main():
    resource_manager = ResourceManager()

    try:
        with time_recorder("基础设施初始化"):
            # 初始化基础设施
            try:
                PathManager.create_dir(Config.OUTPUT_DIR)
                PathManager.create_dir(Config.RECORD_DIR)
                PathManager.create_dir(os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["reserved_dir"]))
                PathManager.create_dir(os.path.join(Config.OUTPUT_DIR, Config.IMAGE_NAMES["images_dir"]))
                
            except Exception as e:
                with resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler:
                    error_handler.handle_error(e, ErrorCategory.FILE)
                raise

            # 启动清理调度器
            cleanup_scheduler = CleanupScheduler()
            cleanup_scheduler.start()

        # 使用资源管理器管理所有资源
        with resource_manager.manage_resource(ResourceType.AUDIO_MANAGER) as audio_mgr:
            # 初始化服务组件
            with time_recorder("服务组件初始化"):
                try:
                    with resource_manager.manage_resource(ResourceType.API_CLIENT) as api_client, \
                            resource_manager.manage_resource(ResourceType.AUDIO_DEVICE) as audio_interface, \
                            resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler:

                        image_output_dir = os.path.join(Config.OUTPUT_DIR, Config.IMAGE_NAMES["images_dir"])  # 保存图片的目录
                        camera_service = CameraService(output_dir=image_output_dir)
                        audio_service = AudioService(audio_interface, audio_mgr)
                        

                        # 创建键盘监听服务
                        keyboard_service = KeyboardService(
                            os.path.join(Config.OUTPUT_DIR, Config.AUDIO_NAMES["reserved_dir"])
                        )
                        keyboard_service.register_handler("ctrl+1",
                                                          lambda: audio_service.play_reserved_audio("gaoxiao1"))
                        keyboard_service.register_handler("ctrl+2",
                                                          lambda: audio_service.play_reserved_audio("gaoxiao2"))
                        keyboard_service.register_handler("alt+1",
                                                          lambda: audio_service.play_reserved_audio("aochengda1"))
                        keyboard_service.register_handler("alt+2",
                                                          lambda: audio_service.play_reserved_audio("gangchengda1"))
                        keyboard_service.register_handler("alt+3",
                                                          lambda: audio_service.play_reserved_audio("gangchengda2"))
                        keyboard_service.register_handler("alt+4",
                                                          lambda: audio_service.play_reserved_audio("gangchengda3"))

                        # 启动键盘监听
                        keyboard_service.start()

                        # 创建对话处理器
                        processor = ChatProcessor(
                            config=Config,  # 配置实例
                            api_client=api_client,  # API 客户端
                            camera_service=camera_service,  # 相机服务（关键：补充此参数）
                            audio_service=audio_service  # 音频服务
                        )

                        while True:
                            try:
                                # 检测唤醒词
                                logger.info("等待唤醒词...")
                                try:
                                    if not audio_interface.detect_wake_word():
                                        continue
                                except AudioError as e:
                                    error_handler.handle_error(e, ErrorCategory.AUDIO)
                                    continue

                                # 标记开始对话
                                audio_service.start_conversation()

                                # 播放提示音频
                                try:
                                    audio_service.play_hello_audio()
                                except AudioError as e:
                                    error_handler.handle_error(e, ErrorCategory.AUDIO)
                                    audio_service.end_conversation()
                                    continue

                                # 循环问话
                                while True:
                                    # 录音阶段
                                    with time_recorder("等待用户输入对话，音频录制"):
                                        try:
                                            audio_path = audio_service.record_and_transcribe()
                                            if not audio_path:
                                                audio_service.end_conversation()
                                                continue
                                        except AudioError as e:
                                            error_handler.handle_error(e, ErrorCategory.AUDIO)
                                            audio_service.end_conversation()
                                            continue

                                    # 语音转文本
                                    with time_recorder("语音转文字"):
                                        try:
                                            transcript = api_client.transcribe_audio(audio_path)
                                            logger.info("音频文件转文字完成，转换成的文字为：")
                                            logger.info(transcript)
                                        except APIError as e:
                                            error_handler.handle_error(e, ErrorCategory.API)
                                            audio_service.end_conversation()
                                            continue

                                    # 检测是否为结束对话
                                    if detect_bye_word(transcript):
                                        logger.info("检测到用户输入 再见，伯乐 退出聊天，播放再见音频文件。")
                                        audio_service.play_bye_audio()
                                        break

                                    # 处理对话流程
                                    with time_recorder("智能体处理"):
                                        try:
                                            # 1. 获取智能体响应（已解析为字典，无需再处理原始字符串）
                                            agent_json = processor.get_raw_response(transcript)  # 重点：变量名改为agent_json，直接接收字典
                                            if not agent_json:
                                                logger.error("未获取到智能体响应，跳过处理")
                                                continue

                                            # 2. 无需再解析JSON（直接使用agent_json）
                                            logger.info(f"获取智能体响应: {agent_json}")

                                            # 3. 提取JSON参数（逻辑不变）
                                            need_capture = agent_json.get("image", False)  # 是否需要拍照
                                            speech_content = agent_json.get("speech", "正在处理")  # 智能体提示文本

                                            # 4. 根据 need_capture 处理（后续逻辑完全不变）
                                            image_path = None
                                            if need_capture and camera_service:
                                                logger.info("智能体要求拍照，开始拍照...")
                                                
                                                # 调用相机服务拍照
                                                # 3. 捕获稳定图像（target_frame 控制等待帧数，越大越稳定，默认50）
                                                target_frame = 50  # 可根据需求调整
                                                image_path = camera_service.capture_stable_image(target_frame=target_frame)
                                                if not image_path or not os.path.exists(image_path):
                                                    logger.error("拍照失败，未生成图片文件")
                                                    continue
                                                logger.info(f"拍照成功: {image_path}")

                                                # 带图片调用智能体处理
                                                result_audio = processor.process_image_query(
                                                    query=transcript,
                                                    image_path=image_path,
                                                    url_or_id=Config.IMAGE_NAMES["process_by_coze"]
                                                )

                                                # 播放结果音频
                                                # todo 需要修改语音文件的位置，现在发送到了下位机
                                                if result_audio and os.path.exists(result_audio):
                                                    logger.info(f"播放图片分析结果音频: {result_audio}")
                                                    audio_service.play_audio(result_audio)
                                                else:
                                                    logger.error("智能体处理图片失败，未生成结果音频")
                                                    audio_service.play_error_audio("图片分析失败")

                                            else:
                                                logger.info("智能体不需要拍照，直接处理文本")
                                                # 用智能体返回的 speech_content 生成音频
                                                result_audio = processor.process_query(speech_content)
                                                # todo 需要修改语音文件的位置，现在发送到了下位机
                                                if result_audio and os.path.exists(result_audio):
                                                    audio_service.play_audio(result_audio)
                                                else:
                                                    logger.error("文本处理失败，未生成结果音频")

                                            logger.info("智能体处理完成，生成音频")

                                        except APIError as e:
                                            error_handler.handle_error(e, ErrorCategory.API)
                                            audio_service.end_conversation()
                                            continue
                                        except Exception as e:
                                            logger.error(f"处理出错: {str(e)}", exc_info=True)
                                            audio_service.end_conversation()
                                            continue
                                    # 播放结果
                                    with time_recorder("音频播放"):
                                        try:
                                            if not audio_service._play_audio("/home/lab/szhr/CozeRobotChat_test/records/outputs.wav"):
                                                audio_service.end_conversation()
                                                continue
                                            logging.info("Conversation cycle completed successfully")
                                        except AudioError as e:
                                            error_handler.handle_error(e, ErrorCategory.AUDIO)
                                            audio_service.end_conversation()
                                            continue

                                # 标记对话结束
                                audio_service.end_conversation()

                            except Exception as e:
                                error_handler.handle_error(e, ErrorCategory.UNKNOWN)
                                audio_service.end_conversation()
                                continue

                except KeyboardInterrupt:
                    logger.info("用户中断程序")
                except Exception as e:
                    error_handler.handle_error(e, ErrorCategory.SYSTEM)
                    raise
                finally:
                    # 清理资源
                    error_handler._emergency_cleanup()

    except Exception as e:
        logger.critical(f"Critical error occurred: {str(e)}")
        with resource_manager.manage_resource(ResourceType.ERROR_HANDLER) as error_handler:
            error_handler.handle_error(e, ErrorCategory.SYSTEM)
        raise
    finally:
        # 清理所有资源
        resource_manager.cleanup_all()


if __name__ == "__main__":
    main()