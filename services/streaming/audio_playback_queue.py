#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import queue
import time
import os
import sys


class AudioPlaybackQueue:
    """専業音頻播放隊列，確保按序播放且等上一個播放完再播下一個（帶計時版）"""

    def __init__(self, audio_service=None):
        self.audio_queue = queue.Queue()
        self.running = False
        self.playback_thread = None
        self.current_audio = None
        self.playback_history = []  # 記錄已播放的音頻
        # 🟢 新增：計時相關
        self.first_play_start_time = None  # 第一次播放開始時間
        self.last_play_end_time = None     # 上一次播放結束時間
        self.all_playback_start_time = None # 所有播放開始時間
        self.all_playback_end_time = None   # 所有播放結束時間
        self.play_sequence = []            # 播放順序記錄（用於計算句間延遲）
        # 🟢 新增：音频服务引用
        self.audio_service = audio_service

    def start(self):
        """啟動音頻播放線程"""
        self.running = True
        self.playback_thread = threading.Thread(
            target=self._playback_worker,
            daemon=True
        )
        self.playback_thread.start()
        print("✅ 音頻播放線程已啟動")

    def stop(self):
        """停止音頻播放線程"""
        self.running = False
        self.audio_queue.put(None)  # 發送退出信號
        if self.playback_thread:
            self.playback_thread.join(timeout=2.0)
        self.all_playback_end_time = time.time()  # 記錄所有播放結束時間
        print("🛑 音頻播放線程已停止")

    def enqueue(self, audio_file, sentence=None):
        """
        將音頻文件加入播放隊列
        :param audio_file: 音頻文件路徑
        :param sentence: 對應的句子（用於日誌）
        """
        if audio_file and os.path.exists(audio_file):
            self.audio_queue.put((audio_file, sentence))
            if sentence:
                print(f"🎧 已將音頻加入播放隊列: '{sentence}' -> {audio_file}")
            else:
                print(f"🎧 已將音頻加入播放隊列: {audio_file}")
        else:
            print(f"❌ 無效的音頻文件，無法加入播放隊列: {audio_file}")

    def _playback_worker(self):
        """音頻播放工作線程"""
        while self.running:
            try:
                item = self.audio_queue.get(timeout=1.0)
                if item is None:  # 退出信號
                    break

                audio_file, sentence = item
                self.current_audio = audio_file

                # 🟢 記錄第一次播放開始時間
                play_start_time = time.time()
                if self.first_play_start_time is None:
                    self.first_play_start_time = play_start_time
                if self.all_playback_start_time is None:
                    self.all_playback_start_time = play_start_time

                # 開始播放前日誌
                if sentence:
                    print(f"\n{'=' * 30}")
                    print(f"▶️ 開始播放句子: '{sentence}'")
                    print(f"📁 音頻文件: {audio_file}")
                    print(f"⏰ 播放開始時間: {time.strftime('%H:%M:%S', time.localtime(play_start_time))}")
                    print(f"{'=' * 30}")
                else:
                    print(f"\n▶️ 開始播放音頻: {audio_file}")

                # 記錄句間延遲（如果之前有播放過）
                if self.last_play_end_time is not None:
                    gap = play_start_time - self.last_play_end_time
                    print(f"⏱️  [計時] 上一句結束 → 本句開始 延遲: {gap:.3f} 秒")

                # 播放音頻
                try:
                    # 使用audio_service播放音频
                    if self.audio_service:
                        # 使用同步播放方法
                        self.audio_service._play_audio(audio_file)
                    else:
                        # 如果没有audio_service，则使用系统命令播放（兼容旧代码）
                        print("⚠️ 未提供audio_service，使用系统命令播放音频")
                        if sys.platform == "darwin":  # macOS
                            import subprocess
                            subprocess.run(["afplay", audio_file], check=True)
                        elif sys.platform == "linux":
                            import subprocess
                            players = ["aplay", "mpg123", "paplay"]
                            played = False
                            for player in players:
                                if subprocess.run(["which", player],
                                                stdout=subprocess.DEVNULL,
                                                stderr=subprocess.DEVNULL).returncode == 0:
                                    subprocess.run([player, audio_file], check=True)
                                    played = True
                                    break
                            if not played:
                                print("❌ 未找到可用的音頻播放器")
                        elif sys.platform == "win32":  # Windows
                            import subprocess
                            subprocess.run(["start", "wmplayer", audio_file], shell=True, check=True)
                        else:
                            print(f"❌ 不支持的平台: {sys.platform}")

                    # 記錄播放完成
                    play_end_time = time.time()
                    duration = play_end_time - play_start_time
                    self.last_play_end_time = play_end_time  # 更新最後播放結束時間

                    self.play_sequence.append({
                        'sentence': sentence,
                        'start_time': play_start_time,
                        'end_time': play_end_time,
                        'duration': duration
                    })

                    self.playback_history.append((audio_file, sentence, duration))
                    print(f"⏹️ 音頻播放完成: {audio_file} (耗時: {duration:.2f}秒)")
                    print(f"⏰ 播放結束時間: {time.strftime('%H:%M:%S', time.localtime(play_end_time))}")

                except Exception as e:
                    print(f"❌ 音頻播放失敗: {str(e)}")
                    import traceback
                    traceback.print_exc()

                self.current_audio = None
                self.audio_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ 音頻播放錯誤: {str(e)}")
                self.current_audio = None

    def get_playback_history(self):
        """獲取播放歷史記錄"""
        return self.playback_history

    # 🟢 新增：獲取計時統計
    def get_timing_stats(self):
        """獲取播放計時統計"""
        stats = {}

        if self.first_play_start_time:
            stats['first_play_delay'] = self.first_play_start_time - getattr(self, 'question_start_time', self.first_play_start_time)

        if len(self.play_sequence) > 1:
            inter_sentence_delays = []
            for i in range(1, len(self.play_sequence)):
                gap = self.play_sequence[i]['start_time'] - self.play_sequence[i-1]['end_time']
                inter_sentence_delays.append(gap)
            stats['inter_sentence_delays'] = inter_sentence_delays

        if self.all_playback_start_time and self.all_playback_end_time:
            stats['total_playback_duration'] = self.all_playback_end_time - self.all_playback_start_time

        return stats

    # 🟢 新增：設置提問開始時間
    def set_question_start_time(self, start_time):
        """設置用戶提問開始時間"""
        self.question_start_time = start_time