#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import queue
import threading
from config import Config
from services.streaming.speech_marker_extractor import SpeechMarkerExtractor
from services.streaming.sentence_buffer import SentenceBuffer


# ======================
# 流式處理核心邏輯（輕量級整合）
# ======================
class StreamingHandler:
    """流式文本處理與TTS調度（輕量級整合）"""

    def __init__(self, tts_engine, audio_playback_queue, api_client):
        self.tts_engine = tts_engine
        self.audio_playback_queue = audio_playback_queue
        self.api_client = api_client
        self.sentence_queue = queue.Queue()
        self.tts_thread = None
        self.running = False
        self.current_task_id = 0
        self.all_sentences = []  # 用於積累所有句子

        # ===== 🟥🟥🟥 核心組件 🟥🟥🟥 =====
        self.speech_extractor = SpeechMarkerExtractor()
        self.sentence_buffer = SentenceBuffer(self.sentence_queue)

        # qwen-tts 生成音频
        self.voice = Config.TTS_VOICE
        self.sample_rate = Config.AUDIO_SETTINGS["sample_rate"]


    def start_tts_worker(self):
        """啟動TTS處理線程"""
        self.running = True

        # ===== 🟥🟥🟥 TTS工作線程 🟥🟥🟥 =====
        self.tts_thread = threading.Thread(
            target=self._tts_worker,
            daemon=True
        )
        self.tts_thread.start()
        print("✅ TTS工作線程已啟動")

    def stop_tts_worker(self):
        """停止TTS處理線程"""
        self.running = False
        self.sentence_queue.put(None)  # 發送退出信號

        if self.tts_thread:
            self.tts_thread.join(timeout=2.0)

        print("🛑 TTS工作線程已停止")

    def _tts_worker(self):
        """TTS工作線程"""
        while self.running:
            try:
                sentence = self.sentence_queue.get(timeout=1.0)
                if sentence is None:  # 退出信號
                    break

                self.current_task_id += 1
                task_id = self.current_task_id

                print(f"\n{'-' * 30}")
                print(f"🔄 處理TTS任務 #{task_id}")
                print(f"📌 句子: '{sentence}'")
                print(f"{'-' * 30}")

                # 移除句子中的换行符和连字符
                sentence = sentence.replace('\n', '').replace('-', '')

                print(f"🔊 開始TTS生成: '{sentence}'")
                # 用 qwen-tts 生成音频
                audio_file = self.api_client.generate_audio(sentence, self.voice, self.sample_rate)
                if audio_file:
                    print(f"✅ TTS生成成功: {audio_file}")
                    # ===== 🟥🟥🟥 將音頻加入播放隊列 🟥🟥🟥 =====
                    self.audio_playback_queue.enqueue(audio_file, sentence)
                    # ===== 🟥🟥🟥 積累句子 🟥🟥🟥 =====
                    self.all_sentences.append(sentence)
                else:
                    print("⚠️ TTS生成失敗，跳過播放")

                self.sentence_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ TTS工作線程錯誤: {str(e)}")
                import traceback
                traceback.print_exc()

    def process_chunk(self, chunk):
        """
        處理文本塊並即時生成音頻
        :param chunk: 新接收的文本塊
        """
        if not chunk:
            return

        # 處理標記匹配
        in_speech, found_end_marker, speech_content = self.speech_extractor.process_chunk(chunk)

        # ===== 🟥🟥🟥 關鍵：即時處理speech內容 🟥🟥🟥 =====
        if in_speech or found_end_marker:
            # 處理speech內容
            self.sentence_buffer.process_speech_content(speech_content)

            # 如果找到結束標記，處理剩餘內容
            if found_end_marker:
                self.sentence_buffer.flush_remaining()

    def flush_remaining(self):
        """處理剩餘內容"""
        # 處理句子緩衝區剩餘內容
        self.sentence_buffer.flush_remaining()

    def get_all_sentences(self):
        """獲取所有積累的句子"""
        return self.all_sentences