"""配置模块

统一的配置管理，包含环境配置、API配置、音频配置和依赖注入配置。
"""

from .environment import Environment
from .api_config import APIConfig
from .audio_config import AudioConfig
from .path_config import PathConfig
from .feature_config import FeatureConfig
from .di_config import DIConfig

# 向后兼容的Config类
class Config(APIConfig, AudioConfig, PathConfig, FeatureConfig):
    """统一配置类，继承所有配置模块"""
    pass

__all__ = [
    'Environment',
    'Config',
    'APIConfig',
    'AudioConfig', 
    'PathConfig',
    'FeatureConfig',
    'DIConfig'
]