# utils/file_transfer.py
import os
import logging
import subprocess
from config import Config


logger = logging.getLogger(__name__)
class File_transfer:
    def __init__(self, config:Config):
        self.config = config
    

    def _transfer_audio_to_lower(self, upper_audio_path: str) -> bool:
        try:
            # 1. 直接定义传输脚本路径（无需加载ROS环境，跳过source命令）
            ros_ws_path = os.path.expanduser("~/szhr/CozeRobotChat/ros_ws")
            transfer_script = os.path.join(
                ros_ws_path, "src", "file_transfer", "scripts", "audio_transfer_client.py"
            )
            
            # 2. 验证传输脚本是否存在（新增：避免脚本路径错误导致执行失败）
            if not os.path.exists(transfer_script):
                logger.error(f"传输脚本不存在：{transfer_script}")
                return False
            
            # 3. 获取下位机目标路径（保持不变）
            lower_target_path = self.config.LOWER_AUDIO_TARGET_PATH 
            # 可选：确保下位机目标路径的父目录存在（避免目标路径不存在导致传输失败）
            lower_target_dir = os.path.dirname(lower_target_path)
            if not os.path.exists(lower_target_dir):
                logger.warning(f"下位机目标路径的父目录不存在，尝试创建：{lower_target_dir}")
                # 若有权限，可自动创建父目录（需确保上位机对下位机路径有写入权限）
                try:
                    os.makedirs(lower_target_dir, exist_ok=True)
                except PermissionError:
                    logger.error(f"创建下位机目标目录失败：权限不足（{lower_target_dir}）")
                    return False
            
            # 4. 确保上位机音频路径为绝对路径（保持不变）
            upper_audio_abs = os.path.abspath(upper_audio_path)
            if not os.path.exists(upper_audio_abs):
                logger.error(f"音频文件不存在：{upper_audio_abs}")
                return False

            # -------------------------- 关键：无ROS环境的命令拼接 --------------------------
            # 直接调用python3执行脚本，无需加载ROS环境，确保命令是完整字符串
            bash_command = (
                f"python3 {transfer_script} "  # 调用传输脚本（末尾加空格衔接参数）
                f"--upper_source {upper_audio_abs} "  # 上位机音频路径参数
                f"--lower_target {lower_target_path}"  # 下位机目标路径参数（无末尾空格）
            )

            # 构建最终命令：bash -c "完整命令字符串"（确保bash解析为一个整体）
            command = ["bash", "-c", bash_command]
            # -----------------------------------------------------------------------------

            # 5. 执行命令（新增超时时间，避免卡住；打印完整命令方便调试）
            logger.info(f"执行音频传输命令：{bash_command}")
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=30  # 30秒超时，防止命令无限阻塞
            )

            # 结果判断（保持不变，增加命令打印便于调试）
            if result.returncode == 0:
                logger.info(f"音频传输成功：{upper_audio_abs} -> {lower_target_path}")
                logger.info(f"传输输出：{result.stdout}")
                return True
            else:
                logger.error(f"音频传输失败，错误码：{result.returncode}")
                logger.error(f"错误输出：{result.stderr}")
                logger.error(f"失败的完整命令：{bash_command}")  # 关键：打印命令，方便手动复现调试
                return False

        except Exception as e:
            logger.error(f"音频传输过程中发生错误：{str(e)}")
            return False