# utils/string_utils.py
import logging
from difflib import SequenceMatcher
from config import Config

logger = logging.getLogger(__name__)


def calculate_similarity(text1: str, text2: str) -> float:
    """
    计算两个字符串的相似度
    :param text1: 第一个字符串
    :param text2: 第二个字符串
    :return: 相似度（0-1之间）
    """
    return SequenceMatcher(None, text1, text2).ratio()


def is_bye_word_match(text: str) -> bool:
    """
    检查文本是否匹配结束词
    :param text: 待检查的文本
    :return: 是否匹配
    """
    if not text:
        return False

    # 计算相似度
    similarity = calculate_similarity(text, Config.BYE_WORD_SETTINGS["bye_word"])
    logger.info(f"文本相似度: {similarity:.2f}")

    return similarity >= Config.BYE_WORD_SETTINGS["bye_word_threshold"]


def detect_bye_word(text: str) -> bool:
    """
    检测是否为结束词
    :param text: 待检查的文本
    :return: 是否为结束词
    """
    if text and is_bye_word_match(text):
        logger.info("相似度检测，检测到结束词！")
        return True
    if text and Config.BYE_WORD_SETTINGS["bye_word"] in text:
        logger.info("全量in检测，检测到结束词！")
        return True
    return False

#todo 未被调用
def is_wake_word_match(text: str) -> bool:
    """
    检查文本是否匹配唤醒词
    :param text: 待检查的文本
    :return: 是否匹配
    """
    if not text:
        return False

    # 计算相似度
    similarity = calculate_similarity(text, Config.WAKE_WORD_SETTINGS["wake_word"])
    logger.info(f"唤醒词相似度: {similarity:.2f}")

    return similarity >= Config.WAKE_WORD_SETTINGS["wake_word_threshold"]