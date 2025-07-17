import time
import logging
from typing import Optional
import random

logger = logging.getLogger(__name__)


class BackoffManager:
    """智能退避策略管理器，用于请求重试和轮询操作"""

    def __init__(
            self,
            initial_delay: float = 1.0,
            max_delay: float = 30.0,
            factor: float = 2.0,
            jitter: bool = True
    ):
        """
        :param initial_delay: 初始延迟（秒）
        :param max_delay: 最大延迟（秒）
        :param factor: 退避因子
        :param jitter: 是否添加随机抖动
        """
        self.initial_delay = initial_delay
        self.max_delay = max_delay
        self.factor = factor
        self.jitter = jitter
        self._current_delay: Optional[float] = None

    def reset(self) -> None:
        """重置退避状态"""
        self._current_delay = None

    def get_next_delay(self) -> float:
        """计算下一个退避间隔"""
        if self._current_delay is None:
            self._current_delay = self.initial_delay
        else:
            self._current_delay = min(self._current_delay * self.factor, self.max_delay)

        if self.jitter:
            # 添加最多25%的随机抖动
            jitter_amount = self._current_delay * 0.25
            actual_delay = self._current_delay + random.uniform(-jitter_amount, jitter_amount)
            return max(0.1, actual_delay)  # 确保最小延迟0.1秒
        return self._current_delay

    def wait(self) -> None:
        """执行等待并更新退避时间"""
        delay = self.get_next_delay()
        logger.debug(f"Backoff waiting: {delay:.2f}s")
        time.sleep(delay)