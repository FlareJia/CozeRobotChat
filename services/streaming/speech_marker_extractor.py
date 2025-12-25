#!/usr/bin/env python3
# -*- coding: utf-8 -*-


class SpeechMarkerExtractor:
    """只負責標記匹配，不處理分句（最終完美無瑕版）"""

    def __init__(self, start_marker="<<SPEECH_START>>", end_marker="<<SPEECH_END>>"):
        self.start_marker = start_marker
        self.end_marker = end_marker
        self.reset_state()

    def reset_state(self):
        """重置解析狀態"""
        self.in_speech = False
        self.buffer = ""  # 用于状态机匹配
        self.speech_content_buffer = ""  # 用于累积 speech 内容
        self.start_match_state = 0
        self.end_match_state = 0

    def process_chunk(self, chunk):
        """
        處理文本塊
        :param chunk: 新接收的文本塊
        :return: (是否在speech內容中, 是否找到結束標記, speech內容)
        """
        if not chunk:
            return self.in_speech, False, ""

        # 保存旧 buffer 长度，用于确定新增部分
        old_len = len(self.buffer)
        self.buffer += chunk
        found_end_marker = False
        speech_content_to_return = ""

        i = 0
        while i < len(self.buffer):
            char = self.buffer[i]

            if not self.in_speech:
                # 匹配開始標記
                if char == self.start_marker[self.start_match_state]:
                    self.start_match_state += 1
                    if self.start_match_state == len(self.start_marker):
                        # 找到開始標記
                        self.in_speech = True
                        self.start_match_state = 0
                        # 移除已匹配的開始標記（包括标记本身）
                        self.buffer = self.buffer[i + 1:]
                        i = 0
                        old_len = 0  # 重置 old_len，因为 buffer 被截断
                        self.speech_content_buffer = ""
                        continue
                else:
                    self.start_match_state = 0
                    if char == self.start_marker[0]:
                        self.start_match_state = 1
                i += 1
            else:
                # 🟢 关键修复：在匹配结束标记过程中，不累积字符
                matching_end = False
                if char == self.end_marker[self.end_match_state]:
                    self.end_match_state += 1
                    matching_end = True
                    if self.end_match_state == len(self.end_marker):
                        # 找到結束標記
                        found_end_marker = True
                        self.in_speech = False
                        self.end_match_state = 0
                        # 保存當前 speech 內容（不包含结束标记）
                        speech_content_to_return = self.speech_content_buffer
                        self.speech_content_buffer = ""
                        # 移除已匹配的結束標記（包括标记本身）
                        self.buffer = self.buffer[i + 1:]
                        i = 0
                        old_len = 0  # 重置 old_len，因为 buffer 被截断
                        continue
                else:
                    self.end_match_state = 0
                    if char == self.end_marker[0]:
                        self.end_match_state = 1

                # 🟢 关键修复：只有不在匹配结束标记时，才累积字符
                if i >= old_len and not matching_end:
                    self.speech_content_buffer += char
                i += 1

        # 如果还在 speech 中，返回当前累积的内容
        if self.in_speech and not found_end_marker:
            speech_content_to_return = self.speech_content_buffer
            self.speech_content_buffer = ""
            if speech_content_to_return:
                print(f"✅ [CONTENT] 提取内容: '{speech_content_to_return}'")
        return self.in_speech, found_end_marker, speech_content_to_return