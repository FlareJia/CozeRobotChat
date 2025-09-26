"""环境配置模块"""

import os
from enum import Enum
from dotenv import load_dotenv

load_dotenv()


class Environment(Enum):
    """环境枚举"""
    DEVELOPMENT = "development"
    TESTING = "testing"
    PRODUCTION = "production"

    @classmethod
    def get_current(cls) -> 'Environment':
        """获取当前环境"""
        env_str = os.getenv('ENVIRONMENT', 'development').lower()
        try:
            return cls(env_str)
        except ValueError:
            return cls.DEVELOPMENT