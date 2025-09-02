# utils/file_transfer.py
import os
import logging
import subprocess


logger = logging.getLogger(__name__)
class File_transfer:
    

    def _transfer_audio_to_lower(self, upper_audio_path: str) -> bool:
        """
        调用ROS客户端脚本，将生成的音频文件传输到下位机
        :param upper_audio_path: 上位机中音频文件的绝对路径
        :return: 传输是否成功
        """
        try:
            # 1. 定义ROS环境路径和传输脚本路径（根据实际路径修改）
            ros_ws_path = os.path.expanduser("~/szhr/CozeRobotChat_test/ros_ws")
            transfer_script = os.path.join(
                ros_ws_path, "src", "file_transfer", "scripts", "audio_transfer_client.py"
            )
            
            # 2. 下位机保存音频的目标路径（根据下位机实际路径配置）
            # 建议在Config中添加配置项：LOWER_AUDIO_TARGET_PATH
            lower_target_path = self.config.LOWER_AUDIO_TARGET_PATH 
            
            # 3. 确保上位机音频路径为绝对路径
            upper_audio_abs = os.path.abspath(upper_audio_path)
            if not os.path.exists(upper_audio_abs):
                logger.error(f"音频文件不存在：{upper_audio_abs}")
                return False

            # 4. 构建命令：加载ROS环境并调用传输脚本
            command = [
                "bash", "-c",
                # f"source {ros_ws_path}/devel/setup.bash && "
                # f"python3 {transfer_script} "
                f"--upper_source {upper_audio_abs} "
                f"--lower_target {lower_target_path}"
            ]

            # 5. 执行命令并检查结果
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            if result.returncode == 0:
                logger.info(f"音频传输成功：{upper_audio_abs} -> {lower_target_path}")
                logger.info(f"传输输出：{result.stdout}")
                return True
            else:
                logger.error(f"音频传输失败，错误码：{result.returncode}")
                logger.error(f"错误输出：{result.stderr}")
                return False

        except Exception as e:
            logger.error(f"音频传输过程中发生错误：{str(e)}")
            return False    