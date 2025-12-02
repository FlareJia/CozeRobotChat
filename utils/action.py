import logging
import subprocess
import threading
from typing import Optional

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - Action - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class Action:
    """支持异步执行的ROS任务处理器，不阻塞键盘输入"""
    
    def __init__(self):
        self._execution_lock = threading.RLock()
        self._is_running = False
        self._task_thread: Optional[threading.Thread] = None  # 存储当前任务线程

    @property
    def is_running(self) -> bool:
        with self._execution_lock:
            return self._is_running
    
    def play_action(self, number: int) -> None:
        """启动新线程执行ROS任务，避免阻塞主线程"""
        with self._execution_lock:
            if self._is_running:
                logger.warning(f"检测到重复按键（数字{number}），当前有任务正在运行，已忽略")
                return
            
            # 标记为运行状态并启动新线程
            self._is_running = True
            self._task_thread = threading.Thread(
                target=self._execute_ros_command,
                args=(number,),
                daemon=True  # 确保程序退出时线程会被终止
            )
            self._task_thread.start()
            logger.info(f"已启动任务线程（数字{number}），当前状态：运行中")

    def _execute_ros_command(self, number: int) -> None:
        """实际执行ROS命令的内部方法，在单独线程中运行"""
        try:
            file_path = f"/home/lab/szhr/CozeRobotChat/records/b{number}"
            ros_command = [
                "roslaunch", 
                "teach_pendant", 
                "launch_teach_pendant_play_file.launch",
                f"file:={file_path}",
                "interval_sec:=0.5"
            ]
            logger.info(f"执行ROS命令: {' '.join(ros_command)}")
            
            # 这里仍然是阻塞调用，但在独立线程中，不影响主线程
            result = subprocess.run(
                ros_command,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            logger.info(f"任务（数字{number}）执行成功，返回码: {result.returncode}")
            if result.stdout:
                logger.info(f"命令输出: {result.stdout[:500]}")
                
        except subprocess.CalledProcessError as e:
            logger.error(f"任务（数字{number}）执行失败，返回码: {e.returncode}")
            logger.error(f"错误输出: {e.stderr}")
        except Exception as e:
            logger.error(f"任务（数字{number}）发生意外错误: {str(e)}")
        finally:
            with self._execution_lock:
                self._is_running = False
                self._task_thread = None
            logger.info(f"任务（数字{number}）结束，当前状态：已停止")
