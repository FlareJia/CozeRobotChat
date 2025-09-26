"""配置模块 - 向后兼容性入口

为了保持向后兼容性，此文件重新导出所有配置类。
新代码建议直接从config包导入具体的配置类。
"""

# 导入所有配置类
from config import (
    Environment,
    Config,
    APIConfig,
    AudioConfig,
    PathConfig,
    FeatureConfig,
    DIConfig
)

# 为了完全向后兼容，保持原有的导入方式
__all__ = [
    'Environment',
    'Config',
    'DIConfig',
    'APIConfig',
    'AudioConfig',
    'PathConfig',
    'FeatureConfig'
]
