#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time


class SentenceBuffer:
    """專注於句子緩衝和分句（最終修復版）"""

    def __init__(self, sentence_queue, min_sentence_length=5):
        self.buffer = ""
        self.sentence_queue = sentence_queue
        self.min_sentence_length = min_sentence_length
        self.end_punctuations = ['。', '！', '？', '.', '!', '?']
        self.last_chunk_time = time.time()

    def process_speech_content(self, speech_content):
        """
        處理speech內容，即時分句
        :param speech_content: 新接收的speech內容
        """
        if not speech_content:
            return

        # 累積到緩衝區
        old_buffer = self.buffer  # 🟢 保存旧缓冲区，用于调试对比
        self.buffer += speech_content
        self.last_chunk_time = time.time()

        sentences_to_send = []

        # 🟢 修复核心：循环从 buffer 开头查找句尾，找到就提取并移除
        while len(self.buffer) > 0:
            found_punct_index = -1
            # 从头开始找第一个句尾标点
            for i, char in enumerate(self.buffer):
                if char in self.end_punctuations:
                    found_punct_index = i
                    break

            if found_punct_index == -1:
                break  # 没找到句尾，退出循环

            # 提取候选句子（从开头到标点）
            candidate = self.buffer[:found_punct_index + 1].strip()

            # 🟢 防御性检查：确保 candidate 是当前 buffer 的合法子串
            if len(candidate) == 0 or len(candidate) > len(self.buffer):
                print(f"⚠️ 无效候选句子，跳过: '{candidate}'")
                break

            if len(candidate) >= self.min_sentence_length:
                sentences_to_send.append(candidate)
                # 🟢 关键修复：按 candidate 实际长度裁剪，而不是按标点位置（避免编码/空格问题）
                cut_length = found_punct_index + 1
                self.buffer = self.buffer[cut_length:]
                # 继续循环，检查剩余部分是否还有完整句子
            else:
                # 句子太短，保留继续累积
                break

        # 发送所有找到的完整句子
        for sentence in sentences_to_send:
            print(f"\n📌 檢測到句子結束: '{sentence}'")
            # 🟢 添加调试：打印发送前的 buffer 状态
            print(f"✂️  发送后缓冲区剩余: '{self.buffer}'")
            self.sentence_queue.put(sentence)

        # 超时处理：防止长句无标点卡住
        if time.time() - self.last_chunk_time > 1.5 and len(self.buffer) > self.min_sentence_length * 2:
            sentence = self.buffer.strip()
            if sentence:
                print(f"\n📌 超時分割: '{sentence}'")
                print(f"✂️  超时分割后缓冲区清空")
                self.sentence_queue.put(sentence)
                self.buffer = ""

    def flush_remaining(self):
        """處理剩餘內容"""
        if self.buffer.strip():
            sentence = self.buffer.strip()
            if sentence and sentence[-1] in self.end_punctuations:
                print(f"\n📌 處理剩餘完整句子: '{sentence}'")
            else:
                print(f"\n📌 強制處理剩餘文本（未完結）: '{sentence}'")
            self.sentence_queue.put(sentence)
            self.buffer = ""