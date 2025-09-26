"""功能配置模块"""


class FeatureConfig:
    """功能开关和特性配置"""
    
    # 功能开关
    FEATURE_FLAGS = {
        'USE_ASYNC_RESERVED_AUDIO': True  # 是否启用预留音频异步播放
    }

    # 流式处理开关
    ENABLE_STREAMING = True  # False = 一次性返回，True = 流式处理