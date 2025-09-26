"""API配置模块"""

import os
from dotenv import load_dotenv

load_dotenv()


class APIConfig:
    """API相关配置"""
    
    # API 配置
    BEARER_TOKEN = os.getenv("BEARER_TOKEN")
    BOT_ID = os.getenv("BOT_ID")
    USER_ID = os.getenv("USER_ID")
    WORKFLOW_BOT_ID = os.getenv("WORKFLOW_BOT_ID")
    
    # ROS工作空间路径
    ROS_WS_PATH = os.path.expanduser("~/szhr/CozeRobotChat/ros_ws")