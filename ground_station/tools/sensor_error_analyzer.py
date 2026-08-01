#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensor_error_analyzer.py — 航電發射前檢查與感測器誤差分析工具 (Pre-Flight & Sensor Analysis Tool)
===========================================================================
本工具經由 USB CDC (或 Serial 埠) 連接 RocketCom 航電主控板，即時擷取遙測封包與日誌串流，
針對全板感測器靜態誤差、雜訊位準 (RMS/StdDev)、零偏漂移、電源紋波、CPU 佔用率（MainTask & EKFTask）
進行全方位評估與發射前 GO / NO-GO 判定。

功能特色：
  1. 【雙模通訊解碼器】：同時支援 USB CDC ASCII 文字/CSV 日誌流與二進制 93-byte 遙測封包。
  2. 【時間自由控制】：使用者可手動隨時按 Enter 或按 Ctrl+C 結束採樣（亦可帶參數 --duration）。
  3. 【即時 Matplotlib 動態圖表】：採樣期間開啟即時圖表視窗，動態觀看加速度、陀螺儀、氣壓/高度與 CPU 佔用率即時波形。
  4. 【超詳細報告輸出】：採樣結束後立即產出 ANSI 終端檢視表、Markdown 報告、JSON 數據與 HTML 儀表板。

用法：
  python3 sensor_error_analyzer.py                        # 開啟即時圖表視窗，使用者手動決定停止時間
  python3 sensor_error_analyzer.py --duration 30          # 固定採樣 30 秒
  python3 sensor_error_analyzer.py --no-gui              # 純文字終端即時模式（不跳圖表視窗）
  python3 sensor_error_analyzer.py --file dump.bin        # 離線分析二進制 dump 檔
  python3 sensor_error_analyzer.py --selftest            # 模擬數據自我測試
"""

import argparse
import datetime
import json
import math
import os
import re
import struct
import sys
import threading
import time

# 載入 matplotlib
HAS_MATPLOTLIB = False
try:
    import matplotlib
    if sys.platform == "darwin":
        try:
            matplotlib.use("TkAgg")
        except Exception:
            try:
                matplotlib.use("MacOSX")
            except Exception:
                pass
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    HAS_MATPLOTLIB = True
except Exception:
    HAS_MATPLOTLIB = False

# 加入上層目錄以載入 serial_link
SYS_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SYS_PARENT not in sys.path:
    sys.path.insert(0, SYS_PARENT)

try:
    import serial_link
except ImportError:
    serial_link = None

# --- 遙測封包結構參數 (與 telemetry.h 同步) ---
# ★ 2026-07-30 兩輪瘦身：先移除 cpu_ekf_x10 + 對端 EKF/丟包率/高G（對端一律只看 VF）
# 並新增 profile_flags（117→103B）；再砍 baro_press_pa、mag 三軸、高G 三軸縮成模長，
# 換上飛行滾動極值 max_alt_m/max_vel_ms/max_acc_cg（103→95B），
# 再加實際開傘高度 drogue_alt_m/main_alt_m（95→99B）。以下必須跟
# TelemetryPacket_t 的欄位順序逐一對齊，順序錯了 CRC 永遠對不上、二進制模式會整段
# 解不出來（曾經整個壞掉過一次）。
#
# ⚠ 本工具是「感測器誤差」分析器，mag 三軸 / 高G 三軸 / 原始氣壓 Pa 正是它的主分析
#   對象——這些欄位現在已經不走 LoRa 下鏈了。但那三組資料本來就該用 USB 直連航電
#   分析（要全速率，2Hz 的 LoRa 取樣本來也做不了雜訊統計），而 USB 文字路徑
#   （[HG]/[MAG]/[BARO] 行 → self.state）完全不受影響，仍是完整的。
#   只有「拿 LoRa 二進位流餵本工具」時那幾張圖會是平的 0，見 _BIN_ABSENT_FIELDS。
PACKET_SIZE = 99
SYNC0, SYNC1 = 0xA5, 0x5A
_STRUCT_FMT = "<4BI2i4hi7h2ih2B2H3B2iHhH2h2BiB2i3BH"
_FIELDS = [
    "sync0", "sync1", "seq", "fsm_state", "tick_ms",
    "ekf_pos_z_cm", "ekf_vel_z_cms",
    "ekf_q0", "ekf_q1", "ekf_q2", "ekf_q3",
    "baro_alt_cm",
    "imu_ax_mg", "imu_ay_mg", "imu_az_mg",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "hg_mag_cg",
    "gps_lat_1e6", "gps_lon_1e6", "gps_alt_m", "gps_sats", "gps_fix",
    "bat_mv", "cpu_main_x10",
    "flags", "health_bits", "sensor_bits",
    "vf_pos_z_cm", "vf_vel_z_cms",
    "max_alt_m", "max_vel_ms", "max_acc_cg",
    "drogue_alt_m", "main_alt_m",
    "peer_fsm_state", "peer_flags", "peer_baro_cm", "peer_link",
    "peer_vf_h_cm", "peer_vf_v_cms",
    "arm_flags", "peer_bench_arb", "profile_flags",
    "crc16",
]
assert struct.calcsize(_STRUCT_FMT) == PACKET_SIZE, struct.calcsize(_STRUCT_FMT)

# 下鏈已不再攜帶、但本工具下游繪圖仍會索引的欄位：二進位解出來後補 0，避免 KeyError。
# （USB 文字路徑會用真值覆蓋這些 key，故只影響純 LoRa 二進位輸入。）
_BIN_ABSENT_FIELDS = {
    "baro_press_pa": 0,
    "hg_ax_cg": 0, "hg_ay_cg": 0, "hg_az_cg": 0,
    "mag_x_mg": 0, "mag_y_mg": 0, "mag_z_mg": 0,
    "cpu_ekf_x10": 0,
}

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

def decode_packet(raw: bytes) -> dict:
    vals = struct.unpack(_STRUCT_FMT, raw)
    d = dict(_BIN_ABSENT_FIELDS)   # 先鋪 0，再讓實際欄位覆蓋（見 _BIN_ABSENT_FIELDS 說明）
    d.update(zip(_FIELDS, vals))
    return d


# ===========================================================================
#  雙模串流解析器 (Dual Stream Parser: Binary & USB CDC ASCII/CSV)
# ===========================================================================
class StreamParser:
    def __init__(self, on_packet_callback, on_diag=None):
        self.on_packet = on_packet_callback
        # on_diag(key, value)：非 CSV 的診斷行（GPS/Flash/SD/LoRa...）走這條單獨通道回報「最新值」，
        # 不塞進 packets（避免像舊版 GPS 那樣被 10Hz CSV 稀釋成假的低比例，見 update_diag 說明）。
        self.on_diag = on_diag
        self.bin_buf = bytearray()
        self.text_buf = ""

        # 正則表達式
        self.csv_re = re.compile(r'^(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)$')
        self.cpu_re = re.compile(r'\[CPU\] MainTask\+ISR:([\d\.]+)\%, EKFTask:([\d\.]+)\%')
        self.mag_re = re.compile(r'\[MAG\] B\[mG\]:(-?\d+),(-?\d+),(-?\d+)')
        self.imu_re = re.compile(r'\[IMU\] a\[mG\]:(-?\d+),(-?\d+),(-?\d+)\s+g\[dps\]:(-?\d+),(-?\d+),(-?\d+)')
        self.highg_re = re.compile(r'\[HIGHG\] a\[mG\]:(-?\d+),(-?\d+),(-?\d+)')
        self.pwr_re = re.compile(r'\[PWR\] bat:(\d+)mV')
        self.health_re = re.compile(r'\[HEALTH\] sens=0x([0-9a-fA-F]+)\s+ekf=0x([0-9a-fA-F]+)\s+fsm=(\d+)')
        self.gps_re = re.compile(r'\[GPS\] fix:(\d+)\s+q:\d+\s+sat:(\d+)')

        # --- Flash / SD / LoRa 診斷行（main.c 對應格式見各 add_check 呼叫處註解） ---
        self.flash_pool_re = re.compile(r'\[FLASH\] pool=(\d+)/(\d+)')
        self.flash_fail_re = re.compile(r'\[FLASH\] WritePacket FAILED, fail_count=(\d+)')
        self.flash_total_re = re.compile(r'\[FLASH_RING\] PKT_TOTAL:(\d+)')
        self.sd_success_re = re.compile(r'\[SD\] \[SUCCESS\] (.+)')
        self.sd_error_re = re.compile(r'\[SD\] \[ERROR\] (.+)')
        self.lora433_re = re.compile(r'\[LORA_TX_LOG\] 433MHz tx ok=(\d+)/try=(\d+)')
        self.lora920_re = re.compile(r'\[LORA_TX_LOG\] 920MHz Sent Pkts: (\d+) \(Last Status: (\w+)\)')
        # E80/LR1121 開機自檢：err=GetErrors 旗標、tcxo=掃描選中的 RegTcxoTune 檔位。
        # err 是「射頻時鐘是否真的起來」的唯一直接證據（設定類命令跑在 HF RC 上，
        # 即使 XOSC 全掛也照樣回 OK，只有這裡看得出來）。舊韌體無此欄位，故為選配。
        self.e80_diag_re = re.compile(r'\[LORA\].*\berr=0x([0-9A-Fa-f]{4})(?:\s+tcxo=0x([0-9A-Fa-f]{2}))?')
        # 主/備雙航電板間鏈路（FEATURE_LINK 才會印；link=NONE 代表單板/尚未連線，非故障）
        self.link_re = re.compile(
            r'\[LINK\] self=\w+ peer=\w+ link=(\w+) state=\w+ flags=0x[0-9A-Fa-f]+ '
            r'age=(\d+)ms sync=(\w+) lost=(\d+) desync=(\d+)')
        # 失效保護計時器強制點火：一次性事件，出現過就該在報告中鎖定顯示
        self.failsafe_re = re.compile(r'\[FSM\] \[FAILSAFE\] (.+)')

        # 920 下行：韌體印的是「開機以來累計」包數，單看最後一筆無法判斷此刻是否還在送。
        # 記首筆與取樣筆數 → 可算出擷取期間實際前進速率（見 (N) 分支與 LoRa Link 檢查）。
        self._l920_first = None
        self._l920_prev = None
        self._l920_n = 0

        # GPS fix 比例改用累計計數（見 (G) 分支），而非讓 fix/sats 隨 CSV 快照被稀釋
        self._gps_lines = 0
        self._gps_fix_n = 0
        self._gps_sat_sum = 0
        self._gps_sat_min = None

        # 當前系統快照狀態
        self.state = {
            "sync0": SYNC0, "sync1": SYNC1,
            "imu_ax_mg": 0, "imu_ay_mg": 0, "imu_az_mg": 1000,
            "gyro_x_dps": 0, "gyro_y_dps": 0, "gyro_z_dps": 0,
            "hg_ax_cg": 0, "hg_ay_cg": 0, "hg_az_cg": 0,
            "baro_press_pa": 101325, "baro_alt_cm": 0,
            "mag_x_mg": 200, "mag_y_mg": -50, "mag_z_mg": 400,
            "bat_mv": 7900, "cpu_main_x10": 150, "cpu_ekf_x10": 200,   # 2S 18650 範圍中段
            "gps_sats": 8, "gps_fix": 1, "gps_lat_1e6": 24789000, "gps_lon_1e6": 120987000, "gps_alt_m": 25,
            "health_bits": 0, "sensor_bits": 0, "fsm_state": 1, "flags": 0,
            "vf_pos_z_cm": 0, "vf_vel_z_cms": 0,
            "peer_fsm_state": 0, "peer_flags": 0, "peer_baro_cm": 0, "peer_link": 0,
            "peer_vf_h_cm": 0, "peer_vf_v_cms": 0,
            "arm_flags": 0, "peer_bench_arb": 0, "profile_flags": 0,
            "seq": 0, "tick_ms": 0, "crc16": 0
        }
        self.seq = 0
        self.start_time = time.time()

    def feed(self, chunk: bytes):
        if not chunk:
            return

        # 1. 嘗試二進制封包解析（PACKET_SIZE bytes，見檔頭 telemetry.h 同步註解）
        self.bin_buf.extend(chunk)
        while len(self.bin_buf) >= PACKET_SIZE:
            if self.bin_buf[0] == SYNC0 and self.bin_buf[1] == SYNC1:
                raw = bytes(self.bin_buf[:PACKET_SIZE])
                crc_recv = raw[PACKET_SIZE - 2] | (raw[PACKET_SIZE - 1] << 8)
                if crc16_ccitt_false(raw[:PACKET_SIZE - 2]) == crc_recv:
                    pkt = decode_packet(raw)
                    # _bin 標記這筆真的來自二進制遙測（seq/tick_ms 是韌體真實值）；
                    # CSV/文字模式的 packet 沒有這個 key，seq/tick_ms 其實是地面站
                    # 本地生成的佔位值（見下方 CSV 分支），兩者不可混算丟包率/重開機偵測。
                    pkt["_bin"] = True
                    self.on_packet(pkt)
                    del self.bin_buf[:PACKET_SIZE]
                    continue
            del self.bin_buf[0]

        # 2. 嘗試 ASCII 文字與 10Hz CSV 解析 (USB CDC 主要傳輸格式)
        self.text_buf += chunk.decode(errors="ignore")
        while "\n" in self.text_buf:
            line, self.text_buf = self.text_buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            # (A) 10Hz CSV 行: ax, ay, az, hg_ax, hg_ay, hg_az, temp, press, alt
            m_csv = self.csv_re.match(line)
            if m_csv:
                v = [int(x) for x in m_csv.groups()]
                self.state["imu_ax_mg"] = v[0]
                self.state["imu_ay_mg"] = v[1]
                self.state["imu_az_mg"] = v[2]
                self.state["hg_ax_cg"] = int(v[3] / 10.0)
                self.state["hg_ay_cg"] = int(v[4] / 10.0)
                self.state["hg_az_cg"] = int(v[5] / 10.0)
                self.state["baro_press_pa"] = v[7]
                self.state["baro_alt_cm"] = v[8]
                self.state["seq"] = self.seq & 0xFF
                self.seq += 1
                self.state["tick_ms"] = int((time.time() - self.start_time) * 1000)

                # 發送當前狀態快照
                self.on_packet(dict(self.state))
                continue

            # (B) CPU 診斷行
            m_cpu = self.cpu_re.search(line)
            if m_cpu:
                self.state["cpu_main_x10"] = int(float(m_cpu.group(1)) * 10.0)
                self.state["cpu_ekf_x10"] = int(float(m_cpu.group(2)) * 10.0)
                continue

            # (C) IMU 診斷行 (含角速度)
            m_imu = self.imu_re.search(line)
            if m_imu:
                v = [int(x) for x in m_imu.groups()]
                self.state["gyro_x_dps"] = v[3]
                self.state["gyro_y_dps"] = v[4]
                self.state["gyro_z_dps"] = v[5]
                continue

            # (D) 磁力計診斷行
            m_mag = self.mag_re.search(line)
            if m_mag:
                v = [int(x) for x in m_mag.groups()]
                self.state["mag_x_mg"] = v[0]
                self.state["mag_y_mg"] = v[1]
                self.state["mag_z_mg"] = v[2]
                continue

            # (E) 電源診斷行
            m_pwr = self.pwr_re.search(line)
            if m_pwr:
                self.state["bat_mv"] = int(m_pwr.group(1))
                continue

            # (F) 健康與 FSM 診斷行
            m_hlth = self.health_re.search(line)
            if m_hlth:
                self.state["sensor_bits"] = int(m_hlth.group(1), 16)
                self.state["health_bits"] = int(m_hlth.group(2), 16)
                self.state["fsm_state"] = int(m_hlth.group(3))
                continue

            # (G) GPS 診斷行 —— fix 比例走累計計數（本行才真的代表一次 GPS 更新事件，
            # 不能靠「之後隨便一行 CSV 帶出去」，那樣會被 10Hz CSV 稀釋成失真的低比例）
            m_gps = self.gps_re.search(line)
            if m_gps:
                fix = int(m_gps.group(1))
                sats = int(m_gps.group(2))
                self.state["gps_fix"] = fix
                self.state["gps_sats"] = sats
                self._gps_lines += 1
                self._gps_fix_n += fix
                self._gps_sat_sum += sats
                self._gps_sat_min = sats if self._gps_sat_min is None else min(self._gps_sat_min, sats)
                if self.on_diag:
                    self.on_diag("gps_fix_ratio", self._gps_fix_n / float(self._gps_lines))
                    self.on_diag("gps_sats_mean", self._gps_sat_sum / float(self._gps_lines))
                    self.on_diag("gps_sats_min", self._gps_sat_min)
                    self.on_diag("gps_diag_lines", self._gps_lines)
                continue

            # (H) Flash 預擦池狀態：pool=剩餘可用 sectors / target 池子容量
            m_fpool = self.flash_pool_re.search(line)
            if m_fpool:
                if self.on_diag:
                    self.on_diag("flash_pool_avail", int(m_fpool.group(1)))
                    self.on_diag("flash_pool_target", int(m_fpool.group(2)))
                continue

            # (I) Flash 寫入失敗累計次數
            m_ffail = self.flash_fail_re.search(line)
            if m_ffail:
                if self.on_diag:
                    self.on_diag("flash_write_fail_count", int(m_ffail.group(1)))
                continue

            # (J) Flash Ring 累積已寫入封包數
            m_ftot = self.flash_total_re.search(line)
            if m_ftot:
                if self.on_diag:
                    self.on_diag("flash_pkt_total", int(m_ftot.group(1)))
                continue

            # (K) SD 成功事件（掛載/建檔/關檔皆共用 [SUCCESS] 前綴，取最後一筆做為狀態摘要）
            m_sds = self.sd_success_re.search(line)
            if m_sds:
                if self.on_diag:
                    self.on_diag("sd_last_success", m_sds.group(1))
                continue

            # (L) SD 錯誤事件（寫入/建檔/掛載失敗，firmware 端視為終止性事件、會停止記錄）
            m_sde = self.sd_error_re.search(line)
            if m_sde:
                if self.on_diag:
                    self.on_diag("sd_last_error", m_sde.group(1))
                continue

            # (M) LoRa 433 下行 TX 成功率（韌體端已是累計 ok/try，取最後一筆即為全程比例）
            m_l433 = self.lora433_re.search(line)
            if m_l433:
                if self.on_diag:
                    self.on_diag("lora433_tx_ok", int(m_l433.group(1)))
                    self.on_diag("lora433_tx_try", int(m_l433.group(2)))
                continue

            # (N) LoRa 920 下行已送封包數 + 最近一次傳送狀態
            m_l920 = self.lora920_re.search(line)
            if m_l920:
                if self.on_diag:
                    sent = int(m_l920.group(1))
                    self.on_diag("lora920_sent", sent)
                    self.on_diag("lora920_last_status", m_l920.group(2))
                    # 首筆 + 筆數 → 供上層算「擷取期間前進了多少」，判斷此刻是否真的還在送。
                    # 計數器是開機以來累計，變小只可能是板子重開機 → 以新值重設基準，
                    # 否則會算出負的前進量而誤報成「完全沒在送」。
                    if self._l920_first is None or (self._l920_prev is not None and sent < self._l920_prev):
                        self._l920_first = sent
                        self._l920_n = 0
                        self.on_diag("lora920_sent_first", sent)
                    self._l920_prev = sent
                    self._l920_n += 1
                    self.on_diag("lora920_samples", self._l920_n)
                continue

            # (N2) E80/LR1121 開機自檢旗標（新韌體才有）
            m_e80d = self.e80_diag_re.search(line)
            if m_e80d:
                if self.on_diag:
                    self.on_diag("e80_errors", int(m_e80d.group(1), 16))
                    if m_e80d.group(2) is not None:
                        self.on_diag("e80_tcxo_tune", int(m_e80d.group(2), 16))
                continue

            # (O) 主/備雙航電板間鏈路狀態（1Hz，取最新一筆）
            m_link = self.link_re.search(line)
            if m_link:
                if self.on_diag:
                    self.on_diag("link_state", m_link.group(1))     # NONE/OK/STALE
                    self.on_diag("link_age_ms", int(m_link.group(2)))
                    self.on_diag("link_sync", m_link.group(3))       # OK/NO
                    self.on_diag("link_lost", int(m_link.group(4)))
                    self.on_diag("link_desync", int(m_link.group(5)))
                continue

            # (P) 失效保護強制點火：一次性事件，鎖定(曾發生過就不會被後面的行洗掉)
            m_fs = self.failsafe_re.search(line)
            if m_fs:
                if self.on_diag:
                    self.on_diag("failsafe_fired_detail", m_fs.group(1))
                continue


# ===========================================================================
#  感測器Datasheet與發射前規格包絡線 (Spec Tolerances & Pre-Flight Limits)
# ===========================================================================
SPEC_LIMITS = {
    "bmi088_accel_norm_diff_max_mg": 35.0,     # 重力向量模長與 1.000g (1000mg) 的最大偏差
    "bmi088_accel_noise_max_mg_rms": 6.0,      # 靜態 RMS 雜訊上限 (mg)
    "bmi088_gyro_offset_max_dps": 1.0,         # 零偏角速度上限 (dps)
    # 靜態 RMS 角速度雜訊上限 (dps)。BMI088 datasheet §1.3：n_rms 標稱 0.1dps 是在 BW=47Hz
    # 條件下量測(NSD=0.014°/s/√Hz)。S5 為了讓 2000Hz ODR 與 1000Hz 讀取率脫鉤(消除拍頻，
    # 即最初 3.79dps RMS 問題的根因)，被迫選用 230Hz BW(唯二支援 2000Hz ODR 的選項之一)。
    # 理論雜訊隨 √BW 增加：0.014×√(230×π/2) ≈ 0.27dps/軸，此處抓 0.30 當 WARN、0.45(×1.5)當 FAIL，
    # 在理論底噪之上留餘裕給板級效應，同時仍能抓到真正異常(如某軸機構共振)。
    "bmi088_gyro_noise_max_dps_rms": 0.60,     # 靜態 RMS 角速度雜訊標準上限 (0.60 dps RMS)
    "lora433_tx_success_min_ratio": 0.30,      # 433MHz 下行 TX 成功率下限 (30%，考量發射前/桌測收聽窗口占空)
    "flash_pool_min_ratio": 0.10,              # Flash 預擦池剩餘可用比例下限 (avail/target)
    "ekf_vf_pos_diff_max_cm": 30.0,            # EKF vs VF 高度差異上限 (cm)，發射台上兩估計器應趨近一致
    "ekf_vf_vel_diff_max_cms": 30.0,           # EKF vs VF 速度差異上限 (cm/s)
    "downlink_seq_loss_max_ratio": 0.05,       # 下行封包丟失率上限 (依 seq 序號推算，5%)
    "adxl375_noise_max_cg_rms": 15.0,          # 靜態 RMS 雜訊上限 (cg)
    "bmp388_press_noise_max_pa_rms": 3.5,      # 氣壓 RMS 雜訊上限 (Pa)
    "bmp388_alt_noise_max_cm_rms": 25.0,       # 相對高度 RMS 雜訊上限 (cm)
    "bmp388_alt_drift_rate_max_ms": 0.08,      # 靜止高度飄移率上限 (m/s)
    "mmc5983_field_norm_min_mg": 200.0,        # 地磁模長合理下限 (mG)
    "mmc5983_field_norm_max_mg": 750.0,        # 地磁模長合理上限 (mG)
    "mmc5983_noise_max_mg_rms": 100.0,         # 磁場 RMS 雜訊上限 (100.0 mG RMS)
    "gps_sats_min": 6,                         # 發射前最少衛星數
    "gps_pos_noise_max_m_rms": 3.0,            # 水平定位靜態飄移 RMS (m)
    "battery_min_mv": 7400,                    # 2S 18650 Li-ion 電池發射前電壓下限 (mV，2×3.7V 標稱)
    "battery_max_mv": 8400,                    # 2S 18650 Li-ion 電池發射前電壓上限 (mV，2×4.2V 滿充)
    "battery_ripple_max_mv_rms": 50.0,         # 電壓紋波 RMS 上限 (mV)
    "cpu_main_max_pct": 35.0,                  # MainTask CPU 佔用率上限 (%)
    "cpu_ekf_max_pct": 45.0,                   # EKFTask CPU 佔用率上限 (%)
}


# ===========================================================================
#  統計計算核心 (Statistical Analysis Engine)
# ===========================================================================
class StatMetrics:
    def __init__(self, values: list):
        self.count = len(values)
        if self.count == 0:
            self.mean = 0.0
            self.stddev = 0.0
            self.min_val = 0.0
            self.max_val = 0.0
            self.p2p = 0.0
            self.rms = 0.0
            return
        self.mean = sum(values) / float(self.count)
        var = sum((x - self.mean) ** 2 for x in values) / float(self.count)
        self.stddev = math.sqrt(var)
        self.min_val = min(values)
        self.max_val = max(values)
        self.p2p = self.max_val - self.min_val
        self.rms = math.sqrt(sum(x ** 2 for x in values) / float(self.count))


def robust_ripple_stats(values: list, mad_k: float = 6.0):
    """回傳 (穩態 stddev, 離群尖峰樣本數)。

    單純的全域 stddev/P2P 對離群值極敏感：一次瞬斷或 ADC 誤讀就能讓 P2P 衝到
    數 V，把「持續性紋波」跟「偶發尖峰」混成同一個數字，兩種問題的意義差很多
    （前者是穩壓/濾波設計問題，後者常是接點/電源瞬斷）。這裡用中位數 + MAD
    (Median Absolute Deviation) 篩掉離群樣本後再算 stddev，離群數量另外回報。
    """
    n = len(values)
    if n < 3:
        return (StatMetrics(values).stddev, 0)
    sorted_v = sorted(values)
    mid = n // 2
    median = sorted_v[mid] if n % 2 else (sorted_v[mid - 1] + sorted_v[mid]) / 2.0
    abs_dev = sorted(abs(x - median) for x in values)
    mad = abs_dev[mid] if n % 2 else (abs_dev[mid - 1] + abs_dev[mid]) / 2.0
    # 1.4826：常態分布下 MAD → stddev 的換算係數。mad==0(訊號太平)時退回用全域 stddev 當門檻尺度。
    thresh = mad_k * mad * 1.4826 if mad > 0 else 3.0 * StatMetrics(values).stddev
    clean = [x for x in values if abs(x - median) <= thresh]
    spikes = n - len(clean)
    clean_std = StatMetrics(clean).stddev if clean else 0.0
    return (clean_std, spikes)


class SensorAnalyzerEngine:
    def __init__(self):
        self.packets = []
        self.diag = {}
        self.lock = threading.Lock()

    def add_packet(self, pkt: dict):
        with self.lock:
            self.packets.append(pkt)

    def get_packets_snapshot(self) -> list:
        with self.lock:
            return list(self.packets)

    def update_diag(self, key: str, value):
        """事件式診斷行（GPS/Flash/SD/LoRa）的最新值快照，與 packets 分開存放。
        這些行不是每個 packet 週期都會出現，硬塞進 packets 會被 10Hz CSV 稀釋成
        失真的統計值（曾在 GPS fix 比例上出現：明明有定位卻算出 0%）。"""
        with self.lock:
            self.diag[key] = value

    def get_diag_snapshot(self) -> dict:
        with self.lock:
            return dict(self.diag)

    def analyze(self) -> dict:
        packets = self.get_packets_snapshot()
        diag = self.get_diag_snapshot()
        n = len(packets)
        if n < 5:
            return {"error": "樣本數不足（需至少 5 筆遙測封包）", "sample_count": n}

        time_secs = (packets[-1]["tick_ms"] - packets[0]["tick_ms"]) / 1000.0
        sample_rate_hz = (n - 1) / time_secs if time_secs > 0 else 0.0

        # 1. BMI088 加速度計
        ax_mg = [p["imu_ax_mg"] for p in packets]
        ay_mg = [p["imu_ay_mg"] for p in packets]
        az_mg = [p["imu_az_mg"] for p in packets]
        a_norms = [math.sqrt(p["imu_ax_mg"]**2 + p["imu_ay_mg"]**2 + p["imu_az_mg"]**2) for p in packets]
        
        stat_ax = StatMetrics(ax_mg)
        stat_ay = StatMetrics(ay_mg)
        stat_az = StatMetrics(az_mg)
        stat_anorm = StatMetrics(a_norms)
        anorm_err = abs(stat_anorm.mean - 1000.0)

        # 2. BMI088 陀螺儀
        gx_dps = [p["gyro_x_dps"] for p in packets]
        gy_dps = [p["gyro_y_dps"] for p in packets]
        gz_dps = [p["gyro_z_dps"] for p in packets]
        g_norms = [math.sqrt(p["gyro_x_dps"]**2 + p["gyro_y_dps"]**2 + p["gyro_z_dps"]**2) for p in packets]

        stat_gx = StatMetrics(gx_dps)
        stat_gy = StatMetrics(gy_dps)
        stat_gz = StatMetrics(gz_dps)
        stat_gnorm = StatMetrics(g_norms)
        max_gyro_offset = max(abs(stat_gx.mean), abs(stat_gy.mean), abs(stat_gz.mean))

        # 3. ADXL375 高 G 加速度計
        hg_ax = [p["hg_ax_cg"] for p in packets]
        hg_ay = [p["hg_ay_cg"] for p in packets]
        hg_az = [p["hg_az_cg"] for p in packets]
        stat_hg_ax = StatMetrics(hg_ax)
        stat_hg_ay = StatMetrics(hg_ay)
        stat_hg_az = StatMetrics(hg_az)

        # 4. BMP388 氣壓計
        press_pa = [p["baro_press_pa"] for p in packets]
        alt_cm = [p["baro_alt_cm"] for p in packets]
        stat_press = StatMetrics(press_pa)
        stat_alt = StatMetrics(alt_cm)
        alt_drift_ms = (stat_alt.max_val - stat_alt.min_val) / (100.0 * time_secs) if time_secs > 0 else 0.0

        # 5. MMC5983MA 磁力計
        mx_mg = [p["mag_x_mg"] for p in packets]
        my_mg = [p["mag_y_mg"] for p in packets]
        mz_mg = [p["mag_z_mg"] for p in packets]
        m_norms = [math.sqrt(p["mag_x_mg"]**2 + p["mag_y_mg"]**2 + p["mag_z_mg"]**2) for p in packets]
        stat_mx = StatMetrics(mx_mg)
        stat_my = StatMetrics(my_mg)
        stat_mz = StatMetrics(mz_mg)
        stat_mnorm = StatMetrics(m_norms)

        # 6. GPS 定位
        gps_sats = [p["gps_sats"] for p in packets]
        gps_fixes = [p["gps_fix"] for p in packets]
        gps_lats = [p["gps_lat_1e6"] / 1e6 for p in packets if p["gps_fix"]]
        gps_lons = [p["gps_lon_1e6"] / 1e6 for p in packets if p["gps_fix"]]
        
        stat_sats = StatMetrics(gps_sats)
        # fix 比例優先用 diag 的累計計數(每一行真實 [GPS] 診斷行各算一次，準確值)；
        # packets 版本會被 10Hz CSV 快照稀釋成失真的低比例(同一次 GPS 更新被複製成多筆
        # packet)，只在 --selftest / 純二進位 dump 沒有 diag 資料時才退回舊算法。
        gps_diag_lines = diag.get("gps_diag_lines", 0)
        if gps_diag_lines > 0:
            fix_ratio = diag["gps_fix_ratio"]
        else:
            fix_ratio = sum(gps_fixes) / float(n)

        if gps_lats and gps_lons:
            mean_lat = sum(gps_lats) / len(gps_lats)
            mean_lon = sum(gps_lons) / len(gps_lons)
            cos_lat = math.cos(math.radians(mean_lat))
            dist_m = [math.sqrt(((lat - mean_lat) * 111000.0)**2 + ((lon - mean_lon) * 111000.0 * cos_lat)**2)
                      for lat, lon in zip(gps_lats, gps_lons)]
            stat_gps_pos = StatMetrics(dist_m)
        else:
            stat_gps_pos = StatMetrics([])

        # 7. 電源系統
        bat_mv = [p["bat_mv"] for p in packets]
        stat_bat = StatMetrics(bat_mv)

        # 8. CPU 資源
        cpu_main = [p["cpu_main_x10"] / 10.0 for p in packets]
        # EKF CPU 只在 USB 直連的 [CPU] 診斷行才有（下行封包 2026-07-30 起不再攜帶），
        # 走 LoRa 二進制路徑的封包沒有這個 key → 預設 0。
        cpu_ekf = [p.get("cpu_ekf_x10", 0) / 10.0 for p in packets]
        stat_cpu_main = StatMetrics(cpu_main)
        stat_cpu_ekf = StatMetrics(cpu_ekf)

        # 9. 健康與故障位
        health_bits_union = 0
        sensor_bits_union = 0
        for p in packets:
            health_bits_union |= p["health_bits"]
            sensor_bits_union |= p["sensor_bits"]

        # ===================================================================
        #  GO / NO-GO 判定規則庫
        # ===================================================================
        checks = []

        def add_check(category: str, name: str, value: float, limit: float, unit: str,
                      condition: str, is_fail: bool, is_warn: bool, detail: str):
            status = "FAIL" if is_fail else ("WARN" if is_warn else "PASS")
            checks.append({
                "category": category,
                "name": name,
                "val": value,
                "limit": limit,
                "unit": unit,
                "status": status,
                "detail": detail
            })

        # --- BMI088 Accel ---
        add_check("BMI088 Accel", "重力向量模長偏差", anorm_err, SPEC_LIMITS["bmi088_accel_norm_diff_max_mg"], "mg",
                  "<=", anorm_err > SPEC_LIMITS["bmi088_accel_norm_diff_max_mg"] * 1.5,
                  anorm_err > SPEC_LIMITS["bmi088_accel_norm_diff_max_mg"],
                  f"實測平均模長={stat_anorm.mean:.1f}mg (1.0g=1000mg)")

        max_acc_noise = max(stat_ax.stddev, stat_ay.stddev, stat_az.stddev)
        add_check("BMI088 Accel", "三軸最大靜態雜訊 (RMS)", max_acc_noise, SPEC_LIMITS["bmi088_accel_noise_max_mg_rms"], "mg RMS",
                  "<=", max_acc_noise > SPEC_LIMITS["bmi088_accel_noise_max_mg_rms"] * 1.5,
                  max_acc_noise > SPEC_LIMITS["bmi088_accel_noise_max_mg_rms"],
                  f"X={stat_ax.stddev:.2f}, Y={stat_ay.stddev:.2f}, Z={stat_az.stddev:.2f} mg RMS")

        # --- BMI088 Gyro ---
        add_check("BMI088 Gyro", "零偏角速度最大值", max_gyro_offset, SPEC_LIMITS["bmi088_gyro_offset_max_dps"], "dps",
                  "<=", max_gyro_offset > SPEC_LIMITS["bmi088_gyro_offset_max_dps"] * 1.5,
                  max_gyro_offset > SPEC_LIMITS["bmi088_gyro_offset_max_dps"],
                  f"X={stat_gx.mean:.3f}, Y={stat_gy.mean:.3f}, Z={stat_gz.mean:.3f} dps")

        max_gyro_noise = max(stat_gx.stddev, stat_gy.stddev, stat_gz.stddev)
        add_check("BMI088 Gyro", "靜態角速度雜訊 (RMS)", max_gyro_noise, SPEC_LIMITS["bmi088_gyro_noise_max_dps_rms"], "dps RMS",
                  "<=", max_gyro_noise > SPEC_LIMITS["bmi088_gyro_noise_max_dps_rms"] * 1.5,
                  max_gyro_noise > SPEC_LIMITS["bmi088_gyro_noise_max_dps_rms"],
                  f"X={stat_gx.stddev:.4f}, Y={stat_gy.stddev:.4f}, Z={stat_gz.stddev:.4f} dps RMS")

        # --- ADXL375 ---
        max_hg_noise = max(stat_hg_ax.stddev, stat_hg_ay.stddev, stat_hg_az.stddev)
        add_check("ADXL375 High-G", "高G感測器雜訊 (RMS)", max_hg_noise, SPEC_LIMITS["adxl375_noise_max_cg_rms"], "cg RMS",
                  "<=", max_hg_noise > SPEC_LIMITS["adxl375_noise_max_cg_rms"] * 1.5,
                  max_hg_noise > SPEC_LIMITS["adxl375_noise_max_cg_rms"],
                  f"X={stat_hg_ax.stddev:.1f}, Y={stat_hg_ay.stddev:.1f}, Z={stat_hg_az.stddev:.1f} cg RMS")

        # --- BMP388 Baro ---
        add_check("BMP388 Baro", "氣壓雜訊 (RMS)", stat_press.stddev, SPEC_LIMITS["bmp388_press_noise_max_pa_rms"], "Pa RMS",
                  "<=", stat_press.stddev > SPEC_LIMITS["bmp388_press_noise_max_pa_rms"] * 1.5,
                  stat_press.stddev > SPEC_LIMITS["bmp388_press_noise_max_pa_rms"],
                  f"氣壓平均值={stat_press.mean:.1f} Pa")

        add_check("BMP388 Baro", "高度雜訊 (RMS)", stat_alt.stddev, SPEC_LIMITS["bmp388_alt_noise_max_cm_rms"], "cm RMS",
                  "<=", stat_alt.stddev > SPEC_LIMITS["bmp388_alt_noise_max_cm_rms"] * 1.5,
                  stat_alt.stddev > SPEC_LIMITS["bmp388_alt_noise_max_cm_rms"],
                  f"相對高度 Peak-to-Peak={stat_alt.p2p:.1f} cm")

        add_check("BMP388 Baro", "靜止高度飄移率", alt_drift_ms, SPEC_LIMITS["bmp388_alt_drift_rate_max_ms"], "m/s",
                  "<=", alt_drift_ms > SPEC_LIMITS["bmp388_alt_drift_rate_max_ms"] * 1.5,
                  alt_drift_ms > SPEC_LIMITS["bmp388_alt_drift_rate_max_ms"],
                  f"統計時長 {time_secs:.1f}s 內高度漂移幅")

        # --- MMC5983MA Mag ---
        add_check("MMC5983MA Mag", "地磁模長合理性", stat_mnorm.mean, SPEC_LIMITS["mmc5983_field_norm_max_mg"], "mG",
                  "range", (stat_mnorm.mean < SPEC_LIMITS["mmc5983_field_norm_min_mg"] or stat_mnorm.mean > SPEC_LIMITS["mmc5983_field_norm_max_mg"]),
                  False, f"實測總磁場={stat_mnorm.mean:.1f} mG (標準範疇 200..750 mG)")

        max_mag_noise = max(stat_mx.stddev, stat_my.stddev, stat_mz.stddev)
        add_check("MMC5983MA Mag", "磁場雜訊 (RMS)", max_mag_noise, SPEC_LIMITS["mmc5983_noise_max_mg_rms"], "mG RMS",
                  "<=", max_mag_noise > SPEC_LIMITS["mmc5983_noise_max_mg_rms"] * 1.5,
                  max_mag_noise > SPEC_LIMITS["mmc5983_noise_max_mg_rms"],
                  f"X={stat_mx.stddev:.2f}, Y={stat_my.stddev:.2f}, Z={stat_mz.stddev:.2f} mG RMS")

        # --- Power System ---
        bat_check_val = stat_bat.mean
        is_usb_power = (bat_check_val < 6000.0)  # 小於 6V 代表桌面 USB CDC 5V 供電模式
        # ripple 用穩態(去離群)stddev 判定，避免單一瞬斷/ADC 誤讀把 P2P 衝爆、
        # 讓「持續性紋波」誤判成 FAIL；離群樣本數另開一項檢查獨立回報。
        ripple_std, ripple_spikes = robust_ripple_stats(bat_mv)
        if is_usb_power:
            add_check("Power System", "電源模式", bat_check_val, 5000.0, "mV",
                      "range", False, False,
                      f"桌面 USB 供電模式 ({bat_check_val/1000.0:.2f}V)")
            add_check("Power System", "電壓紋波 (穩態 RMS)", ripple_std, SPEC_LIMITS["battery_ripple_max_mv_rms"], "mV RMS",
                      "<=", False, False,
                      f"USB 供電紋波={ripple_std:.1f} mV（P2P={stat_bat.p2p:.1f}mV 含離群值）")
        else:
            add_check("Power System", "電池電壓", bat_check_val, SPEC_LIMITS["battery_min_mv"], "mV",
                      "range", bat_check_val < SPEC_LIMITS["battery_min_mv"] or bat_check_val > SPEC_LIMITS["battery_max_mv"],
                      False, f"2S 18650 電池電壓={bat_check_val/1000.0:.2f}V ({bat_check_val:.0f} mV)")
            add_check("Power System", "電壓紋波 (穩態 RMS)", ripple_std, SPEC_LIMITS["battery_ripple_max_mv_rms"], "mV RMS",
                      "<=", ripple_std > SPEC_LIMITS["battery_ripple_max_mv_rms"] * 1.5,
                      ripple_std > SPEC_LIMITS["battery_ripple_max_mv_rms"],
                      f"去離群後穩態 RMS（原始 P2P={stat_bat.p2p:.1f}mV，未去離群，僅供參考）")

        # --- GPS ---
        gps_ratio_src = f"{gps_diag_lines} 筆真實 [GPS] 診斷行" if gps_diag_lines > 0 else f"{n} 筆 packet 快照(退回模式，較不精確)"
        add_check("GPS System", "定位狀態 (3D Fix)", fix_ratio * 100.0, 100.0, "%",
                  "==", fix_ratio < 1.0, False,
                  f"有效 Fix 比例: {fix_ratio*100:.1f}%（統計基礎：{gps_ratio_src}）")

        # 與 fix_ratio 同理：有真實 [GPS] 診斷行時優先用 diag 累計值，避免被 CSV 稀釋
        sats_mean_chk = diag["gps_sats_mean"] if gps_diag_lines > 0 else stat_sats.mean
        sats_min_chk = diag["gps_sats_min"] if gps_diag_lines > 0 else stat_sats.min_val
        add_check("GPS System", "發射前衛星數", sats_mean_chk, SPEC_LIMITS["gps_sats_min"], "sats",
                  ">=", sats_mean_chk < SPEC_LIMITS["gps_sats_min"], False,
                  f"平均衛星數={sats_mean_chk:.1f}, 最小={sats_min_chk:.0f}")

        if stat_gps_pos.count > 0:
            add_check("GPS System", "定位飄移 (RMS)", stat_gps_pos.rms, SPEC_LIMITS["gps_pos_noise_max_m_rms"], "m RMS",
                      "<=", stat_gps_pos.rms > SPEC_LIMITS["gps_pos_noise_max_m_rms"] * 1.5,
                      stat_gps_pos.rms > SPEC_LIMITS["gps_pos_noise_max_m_rms"],
                      f"相對平均位置 RMS 飄移={stat_gps_pos.rms:.2f}m（僅計入有 fix 樣本，n={stat_gps_pos.count}）")
        else:
            add_check("GPS System", "定位飄移 (RMS)", 0.0, SPEC_LIMITS["gps_pos_noise_max_m_rms"], "m RMS",
                      "<=", False, False, "無有效 GPS fix 樣本，無法計算定位飄移")

        # --- State Estimation：EKF vs VF 交叉比對（僅二進制遙測有此欄位，見 _bin 標記） ---
        bin_pkts = [p for p in packets if p.get("_bin")]
        if bin_pkts:
            pos_diffs = [abs(p["ekf_pos_z_cm"] - p["vf_pos_z_cm"]) for p in bin_pkts]
            vel_diffs = [abs(p["ekf_vel_z_cms"] - p["vf_vel_z_cms"]) for p in bin_pkts]
            mean_pos_diff = sum(pos_diffs) / len(pos_diffs)
            mean_vel_diff = sum(vel_diffs) / len(vel_diffs)
            add_check("State Estimation", "EKF vs VF 高度差異", mean_pos_diff, SPEC_LIMITS["ekf_vf_pos_diff_max_cm"], "cm",
                      "<=", mean_pos_diff > SPEC_LIMITS["ekf_vf_pos_diff_max_cm"] * 1.5,
                      mean_pos_diff > SPEC_LIMITS["ekf_vf_pos_diff_max_cm"],
                      f"平均差異={mean_pos_diff:.1f}cm，最大={max(pos_diffs)}cm（發射台上兩個獨立估計器應趨近一致）")
            add_check("State Estimation", "EKF vs VF 速度差異", mean_vel_diff, SPEC_LIMITS["ekf_vf_vel_diff_max_cms"], "cm/s",
                      "<=", mean_vel_diff > SPEC_LIMITS["ekf_vf_vel_diff_max_cms"] * 1.5,
                      mean_vel_diff > SPEC_LIMITS["ekf_vf_vel_diff_max_cms"],
                      f"平均差異={mean_vel_diff:.1f}cm/s，最大={max(vel_diffs)}cm/s")
        else:
            add_check("State Estimation", "EKF vs VF 高度差異", 0, SPEC_LIMITS["ekf_vf_pos_diff_max_cm"], "cm",
                      "<=", False, False,
                      "未收到二進制遙測封包（USB 文字模式沒有 VF 診斷行，需 LoRa 接收或 --file 二進制 dump 才有數據）")

        # --- Telemetry Link：下行丟包率 + 擷取期間重開機偵測（同樣僅二進制遙測有意義，
        # CSV/文字模式的 seq/tick_ms 是地面站本地生成的佔位值，見 StreamParser 註解） ---
        if len(bin_pkts) >= 2:
            seqs = [p["seq"] for p in bin_pkts]
            total_gap = 0
            total_steps = 0
            for i in range(1, len(seqs)):
                gap = (seqs[i] - seqs[i - 1]) % 256
                if gap == 0:
                    continue  # 同 seq 重複(常見於 433/920 雙鏈路都收到同一筆)，不算新進度也不算丟包
                total_gap += gap
                total_steps += 1
            total_lost = total_gap - total_steps
            loss_ratio = total_lost / float(total_gap) if total_gap > 0 else 0.0
            add_check("Telemetry Link", "下行封包丟失率 (seq)", loss_ratio * 100.0,
                      SPEC_LIMITS["downlink_seq_loss_max_ratio"] * 100.0, "%",
                      "<=", loss_ratio > SPEC_LIMITS["downlink_seq_loss_max_ratio"] * 1.5,
                      loss_ratio > SPEC_LIMITS["downlink_seq_loss_max_ratio"],
                      f"依 seq 序號推算：預期 {total_gap} 筆、遺失 {total_lost} 筆")

            reboot_events = 0
            max_drop = 0
            for i in range(1, len(bin_pkts)):
                drop = bin_pkts[i - 1]["tick_ms"] - bin_pkts[i]["tick_ms"]
                if drop > 500:  # tick_ms 是遞增的 HAL_GetTick()，倒退超過 500ms 視為重開機而非量測抖動
                    reboot_events += 1
                    max_drop = max(max_drop, drop)
            add_check("Telemetry Link", "擷取期間主板重開機偵測", reboot_events, 0, "count",
                      "==", reboot_events > 0, False,
                      f"tick_ms 倒退次數={reboot_events}" +
                      (f"，最大倒退={max_drop}ms" if reboot_events else "（連續無倒退）"))
        else:
            add_check("Telemetry Link", "下行封包丟失率 (seq)", 0.0,
                      SPEC_LIMITS["downlink_seq_loss_max_ratio"] * 100.0, "%",
                      "<=", False, False, "未收到足夠二進制遙測封包（需 LoRa 接收或 --file 二進制 dump）")
            add_check("Telemetry Link", "擷取期間主板重開機偵測", 0, 0, "count",
                      "==", False, False, "未收到足夠二進制遙測封包（需 LoRa 接收或 --file 二進制 dump）")

        # --- Dual Avionics：主/備板間鏈路（來自 [LINK] 診斷行，USB 文字模式即可看到） ---
        if "link_state" in diag:
            lstate = diag["link_state"]
            lsync = diag.get("link_sync", "?")
            llost = diag.get("link_lost", 0)
            ldesync = diag.get("link_desync", 0)
            lage = diag.get("link_age_ms", 0)
            if lstate == "NONE":
                add_check("Dual Avionics", "主/備鏈路狀態", 0, 0, "",
                          "==", False, False, "link=NONE（單板飛行或對端尚未連線，非故障）")
            else:
                is_bad = (lstate != "OK") or (lsync != "OK") or (llost != 0) or (ldesync != 0)
                add_check("Dual Avionics", "主/備鏈路狀態", 1 if is_bad else 0, 0, "",
                          "==", is_bad, False,
                          f"link={lstate} sync={lsync} lost={llost} desync={ldesync} age={lage}ms")
        else:
            add_check("Dual Avionics", "主/備鏈路狀態", 0, 0, "",
                      "==", False, False, "未見到 [LINK] 行（FEATURE_LINK=0 或本次擷取未涵蓋）")

        # --- FSM State ---
        _FSM_NAMES = {0: "INIT", 1: "PAD", 2: "PAD_ARMED", 3: "BOOST", 4: "COAST",
                      5: "DEPLOY_DROGUE", 6: "APOGEE", 7: "DESCENT", 8: "MAIN_DEPLOY", 9: "LANDED"}
        final_fsm = packets[-1]["fsm_state"]
        fsm_name = _FSM_NAMES.get(final_fsm, f"UNKNOWN({final_fsm})")
        if final_fsm in (1, 2):          # STATE_PAD / STATE_PAD_ARMED：正常待命
            fsm_fail, fsm_warn = False, False
        elif final_fsm == 0:             # STATE_INIT：仍在開機/校準中，還沒 ready 但不算異常
            fsm_fail, fsm_warn = False, True
        else:                            # STATE_BOOST 以後：擷取當下系統認為已經飛過了
            fsm_fail, fsm_warn = True, False
        add_check("FSM State", "發射前狀態合理性", final_fsm, 2, "",
                  "range", fsm_fail, fsm_warn,
                  f"擷取結束時 fsm_state={fsm_name}（發射前應停在 PAD 或 PAD_ARMED）")

        if "failsafe_fired_detail" in diag:
            add_check("FSM State", "失效保護計時器", 1, 0, "count",
                      "==", True, False, f"曾觸發強制點火：{diag['failsafe_fired_detail']}")
        else:
            add_check("FSM State", "失效保護計時器", 0, 0, "count",
                      "==", False, False, "未觸發（正常）")

        # --- Flash Storage / SD Storage / LoRa Link ---
        # 以下皆來自事件式診斷行的最新值快照(見 StreamParser.on_diag)，短時間桌測、
        # 該鏈路未啟用、或純二進位 dump 都可能完全沒出現對應的 log 行。這裡一律「照樣
        # 加一列 PASS + 註明未見到該行」而不是整列省略——省略會讓「已檢查沒問題」跟
        # 「這次根本沒收集到資料」看起來一樣，之前就因為這樣被誤以為 LoRa 檢查沒做。
        if "flash_write_fail_count" in diag:
            ffail = diag["flash_write_fail_count"]
            add_check("Flash Storage", "寫入失敗累計次數", ffail, 0, "count",
                      "==", ffail > 0, False,
                      f"WritePacket 失敗計數={ffail}（見 [FLASH] WritePacket FAILED）")
        else:
            add_check("Flash Storage", "寫入失敗累計次數", 0, 0, "count",
                      "==", False, False, "未見到 [FLASH] WritePacket FAILED 行（本次擷取無寫入活動或未啟用 Flash 記錄）")

        if "flash_pkt_total" in diag:
            add_check("Flash Storage", "Ring 累積寫入封包數", diag["flash_pkt_total"], 0, "packets",
                      "==", False, False,
                      f"PKT_TOTAL={diag['flash_pkt_total']}（PAD/PAD_ARMED 期間依設計不寫入）")
        else:
            add_check("Flash Storage", "Ring 累積寫入封包數", 0, 0, "packets",
                      "==", False, False, "未見到 [FLASH_RING] PKT_TOTAL 行")

        if "flash_pool_avail" in diag and "flash_pool_target" in diag:
            favail = diag["flash_pool_avail"]
            ftarget = diag["flash_pool_target"]
            pool_ratio = (favail / float(ftarget)) if ftarget > 0 else 1.0
            add_check("Flash Storage", "預擦池剩餘比例", pool_ratio * 100.0,
                      SPEC_LIMITS["flash_pool_min_ratio"] * 100.0, "%",
                      ">=", pool_ratio < SPEC_LIMITS["flash_pool_min_ratio"] * 0.5,
                      pool_ratio < SPEC_LIMITS["flash_pool_min_ratio"],
                      f"pool={favail}/{ftarget} sectors")
        else:
            add_check("Flash Storage", "預擦池剩餘比例", 100.0,
                      SPEC_LIMITS["flash_pool_min_ratio"] * 100.0, "%",
                      ">=", False, False, "未見到 [FLASH] pool=avail/target 行")

        # --- SD Storage ---
        if "sd_last_error" in diag:
            add_check("SD Storage", "寫入/掛載狀態", 1, 0, "errors",
                      "==", True, False, f"曾發生錯誤: {diag['sd_last_error']}")
        elif "sd_last_success" in diag:
            add_check("SD Storage", "寫入/掛載狀態", 0, 0, "errors",
                      "==", False, False, f"最新成功事件: {diag['sd_last_success']}")
        else:
            add_check("SD Storage", "寫入/掛載狀態", 0, 0, "errors",
                      "==", False, False, "未見到任何 [SD] [SUCCESS]/[ERROR] 行（可能未插卡或本次擷取未涵蓋建檔階段）")

        # --- LoRa Link ---
        if diag.get("lora433_tx_try", 0) > 0:
            l433_ok, l433_try = diag["lora433_tx_ok"], diag["lora433_tx_try"]
            l433_ratio = l433_ok / float(l433_try)
            add_check("LoRa Link", "433MHz TX 成功率", l433_ratio * 100.0,
                      SPEC_LIMITS["lora433_tx_success_min_ratio"] * 100.0, "%",
                      ">=", False, l433_ratio < SPEC_LIMITS["lora433_tx_success_min_ratio"],
                      f"ok={l433_ok}/try={l433_try}（發射前/桌測時收聽窗口占空比高，屬正常現象）")
        else:
            add_check("LoRa Link", "433MHz TX 成功率", 100.0,
                      SPEC_LIMITS["lora433_tx_success_min_ratio"] * 100.0, "%",
                      ">=", False, False, "未見到 [LORA_TX_LOG] 433MHz 行（lora433_ok=0 或本次擷取未涵蓋下行期）")

        # 920 下行：判定依據是「累計包數在擷取期間有沒有持續前進」，不是最後一筆的狀態字串。
        # ★ 舊版寫成 is_fail = (last_status != "OK") —— 那是錯的：LoRaE80_Send 的 BUSY 是
        #   正常背壓（上一包還在空中就跳過本時槽）。以 SF9/BW250/CR4-5、~116B 計，單包空中
        #   時間約 300ms，而排程是 200ms 一槽 → 多數時槽本來就回 BUSY，只要抽樣沒撞上那一
        #   瞬間的 OK 就永遠 FAIL，鏈路完全正常也會被誤報成故障。
        if "lora920_sent" in diag:
            l920_status = diag.get("lora920_last_status", "?")
            sent_last   = diag["lora920_sent"]
            sent_first  = diag.get("lora920_sent_first", sent_last)
            nsamp       = diag.get("lora920_samples", 1)
            advanced    = sent_last - sent_first
            # 韌體每 25 個 LORA_TELEM_PERIOD_MS(200ms) 時槽印一次 → 相鄰兩筆間隔約 5s
            span_s = max(nsamp - 1, 0) * 5.0
            rate   = (advanced / span_s) if span_s > 0 else 0.0
            if nsamp >= 2:
                add_check("LoRa Link", "920MHz 下行送出速率", rate, 0.0, "Hz",
                          ">", advanced <= 0, False,
                          f"擷取期間 +{advanced} 包 / 約 {span_s:.0f}s ≈ {rate:.2f} Hz"
                          f"（累計 {sent_last} 包，{nsamp} 筆取樣，最近狀態={l920_status}"
                          f"；BUSY=背壓跳過本槽，屬正常）")
            else:
                add_check("LoRa Link", "920MHz 下行送出速率", 0.0, 0.0, "Hz",
                          ">", False, False,
                          f"累計 {sent_last} 包，但本次擷取僅 1 筆取樣（需 ≥2 筆、約 10s 才能算速率）")
        else:
            add_check("LoRa Link", "920MHz 下行送出速率", 0.0, 0.0, "Hz",
                      ">", False, False, "未見到 [LORA_TX_LOG] 920MHz 行（lora920_ok=0 或本次擷取未涵蓋下行期）")

        # 920 射頻時鐘/校準自檢（GetErrors）。這是唯一能直接看出「TCXO 沒起振、PLL 沒鎖上」
        # 的指標——這兩者掛掉時，SetTx 仍回 OK、晶片仍顯示 rdy，只有 TxDone 永遠不來，
        # 表現為送出速率被逾時兜底鎖在極低值（本專案曾長期 0.6Hz 而無人察覺）。
        E80_ERR_BITS = [
            (0x0020, "HF_XOSC 起振失敗（TCXO 供電/電壓不對）"),
            (0x0080, "PLL 鎖定失敗"),
            (0x0008, "PLL 校準失敗"),
            (0x0010, "影像校準失敗"),
            (0x0004, "ADC 校準失敗"),
            (0x0002, "HF RC 校準失敗"),
            (0x0001, "LF RC 校準失敗"),
            (0x0040, "LF XOSC 起振失敗"),
        ]
        if "e80_errors" in diag:
            e80err = diag["e80_errors"]
            tune   = diag.get("e80_tcxo_tune")
            tune_v = {0: "1.6V", 1: "1.7V", 2: "1.8V", 3: "2.2V",
                      4: "2.4V", 5: "2.7V", 6: "3.0V", 7: "3.3V"}.get(tune)
            tune_s = (f"，TCXO 檔位=0x{tune:02X}"
                      + (f"({tune_v})" if tune_v else "")) if tune is not None else ""
            if e80err == 0:
                detail = f"err=0x0000：TCXO 起振 + PLL 鎖定 + 各項校準全數通過{tune_s}"
            else:
                hits = [d for m, d in E80_ERR_BITS if e80err & m]
                detail = f"err=0x{e80err:04X}：" + "、".join(hits) + tune_s
            add_check("LoRa Link", "920MHz 射頻時鐘自檢 (GetErrors)", e80err, 0, "bits",
                      "==", e80err != 0, False, detail)
        else:
            add_check("LoRa Link", "920MHz 射頻時鐘自檢 (GetErrors)", 0, 0, "bits",
                      "==", False, False,
                      "未見到 [LORA] err= 欄位（韌體版本較舊，未輸出 GetErrors 自檢）")

        # --- CPU ---
        add_check("CPU System", "MainTask CPU 佔用率", stat_cpu_main.mean, SPEC_LIMITS["cpu_main_max_pct"], "%",
                  "<=", stat_cpu_main.mean > SPEC_LIMITS["cpu_main_max_pct"] * 1.5,
                  stat_cpu_main.mean > SPEC_LIMITS["cpu_main_max_pct"],
                  f"平均={stat_cpu_main.mean:.1f}%, 最大={stat_cpu_main.max_val:.1f}%")

        add_check("CPU System", "EKFTask CPU 佔用率", stat_cpu_ekf.mean, SPEC_LIMITS["cpu_ekf_max_pct"], "%",
                  "<=", stat_cpu_ekf.mean > SPEC_LIMITS["cpu_ekf_max_pct"] * 1.5,
                  stat_cpu_ekf.mean > SPEC_LIMITS["cpu_ekf_max_pct"],
                  f"平均={stat_cpu_ekf.mean:.1f}%, 最大={stat_cpu_ekf.max_val:.1f}%")

        # --- Hardware Health Flags ---
        health_detail = "0x00 (EKF 全健康)"
        if health_bits_union != 0:
            h_tags = []
            if health_bits_union & 0x01: h_tags.append("BARO_DIVERGE(氣壓發散)")
            if health_bits_union & 0x02: h_tags.append("STATE_OOB(狀態超界)")
            if health_bits_union & 0x04: h_tags.append("NAN(數值異常)")
            if health_bits_union & 0x08: h_tags.append("BARO_TIMEOUT(氣壓逾時)")
            health_detail = f"0x{health_bits_union:02X} [{'|'.join(h_tags)}]"

        sensor_detail = "0x00 (感測器全健康)"
        if sensor_bits_union != 0:
            s_tags = []
            if sensor_bits_union & 0x01: s_tags.append("BMI088(低G/陀螺失效)")
            if sensor_bits_union & 0x02: s_tags.append("ADXL375(高G失效)")
            if sensor_bits_union & 0x04: s_tags.append("BMP388(氣壓計失流/卡死/離線)")
            sensor_detail = f"0x{sensor_bits_union:02X} [{'|'.join(s_tags)}]"

        add_check("System Health", "EKF 健康旗標 (health_bits)", health_bits_union, 0, "bits",
                  "==", health_bits_union != 0, False, health_detail)

        add_check("System Health", "感測器健康旗標 (sensor_bits)", sensor_bits_union, 0, "bits",
                  "==", sensor_bits_union != 0, False, sensor_detail)

        # 總結 GO / NO-GO 狀態
        has_fail = any(c["status"] == "FAIL" for c in checks)
        has_warn = any(c["status"] == "WARN" for c in checks)
        overall_status = "NO-GO" if has_fail else ("WARN" if has_warn else "GO")

        return {
            "summary": {
                "overall_status": overall_status,
                "sample_count": n,
                "duration_sec": round(time_secs, 2),
                "sample_rate_hz": round(sample_rate_hz, 1),
                "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "fail_count": sum(1 for c in checks if c["status"] == "FAIL"),
                "warn_count": sum(1 for c in checks if c["status"] == "WARN"),
                "pass_count": sum(1 for c in checks if c["status"] == "PASS"),
            },
            "checks": checks,
            "raw_stats": {
                "bmi088_accel": {
                    "ax": stat_ax.__dict__, "ay": stat_ay.__dict__, "az": stat_az.__dict__,
                    "norm": stat_anorm.__dict__, "norm_err_mg": round(anorm_err, 2)
                },
                "bmi088_gyro": {
                    "gx": stat_gx.__dict__, "gy": stat_gy.__dict__, "gz": stat_gz.__dict__,
                    "norm": stat_gnorm.__dict__, "max_offset_dps": round(max_gyro_offset, 3)
                },
                "adxl375": {
                    "ax": stat_hg_ax.__dict__, "ay": stat_hg_ay.__dict__, "az": stat_hg_az.__dict__
                },
                "bmp388": {
                    "press": stat_press.__dict__, "alt": stat_alt.__dict__, "drift_rate_ms": round(alt_drift_ms, 4)
                },
                "mmc5983": {
                    "mx": stat_mx.__dict__, "my": stat_my.__dict__, "mz": stat_mz.__dict__, "norm": stat_mnorm.__dict__
                },
                "gps": {
                    "sats": stat_sats.__dict__, "fix_ratio": round(fix_ratio, 3), "pos_noise": stat_gps_pos.__dict__
                },
                "power": {
                    "bat_mv": stat_bat.__dict__
                },
                "cpu": {
                    "main": stat_cpu_main.__dict__, "ekf": stat_cpu_ekf.__dict__
                }
            },
            "time_series": {
                "ticks": [p["tick_ms"] for p in packets],
                "imu_ax": ax_mg, "imu_ay": ay_mg, "imu_az": az_mg,
                "gyro_x": gx_dps, "gyro_y": gy_dps, "gyro_z": gz_dps,
                "baro_press": press_pa, "baro_alt": alt_cm,
                "bat_mv": bat_mv, "cpu_main": cpu_main, "cpu_ekf": cpu_ekf
            }
        }


# ===========================================================================
#  Matplotlib 即時圖表 GUI 視窗 (Live Real-Time Matplotlib Visualization)
# ===========================================================================
class LivePlotter:
    def __init__(self, engine: SensorAnalyzerEngine, stop_event: threading.Event):
        self.engine = engine
        self.stop_event = stop_event

        self.fig, self.axs = plt.subplots(5, 1, figsize=(10, 10), sharex=True)
        self.fig.canvas.manager.set_window_title("RocketCom 發射前即時感測器監控 (按 Enter 或關閉視窗結束採樣)")

        # 子圖 1: 加速度計
        self.line_ax, = self.axs[0].plot([], [], 'r-', label='ax (mg)')
        self.line_ay, = self.axs[0].plot([], [], 'g-', label='ay (mg)')
        self.line_az, = self.axs[0].plot([], [], 'b-', label='az (mg)')
        self.axs[0].set_ylabel("Accel (mg)")
        self.axs[0].legend(loc='upper right', fontsize=8)
        self.axs[0].grid(True, linestyle=':', alpha=0.6)

        # 子圖 2: 陀螺儀
        self.line_gx, = self.axs[1].plot([], [], 'r-', label='gx (dps)')
        self.line_gy, = self.axs[1].plot([], [], 'g-', label='gy (dps)')
        self.line_gz, = self.axs[1].plot([], [], 'b-', label='gz (dps)')
        self.axs[1].set_ylabel("Gyro (dps)")
        self.axs[1].legend(loc='upper right', fontsize=8)
        self.axs[1].grid(True, linestyle=':', alpha=0.6)

        # 子圖 3: 氣壓計相對高度
        self.line_alt, = self.axs[2].plot([], [], 'm-', label='Baro Alt (cm)')
        self.axs[2].set_ylabel("Alt (cm)")
        self.axs[2].legend(loc='upper right', fontsize=8)
        self.axs[2].grid(True, linestyle=':', alpha=0.6)

        # 子圖 4: 電池電壓（新增：ripple/瞬斷尖峰用肉眼直接看比單一 RMS 數字更直觀）
        self.line_bat, = self.axs[3].plot([], [], 'k-', label='Battery (mV)')
        self.axs[3].set_ylabel("Bat (mV)")
        self.axs[3].legend(loc='upper right', fontsize=8)
        self.axs[3].grid(True, linestyle=':', alpha=0.6)

        # 子圖 5: CPU 佔用率
        self.line_cpum, = self.axs[4].plot([], [], 'c-', label='Main CPU (%)')
        self.line_cpue, = self.axs[4].plot([], [], 'y-', label='EKF CPU (%)')
        self.axs[4].set_ylabel("CPU (%)")
        self.axs[4].set_xlabel("Time (s)")
        self.axs[4].legend(loc='upper right', fontsize=8)
        self.axs[4].grid(True, linestyle=':', alpha=0.6)

        # Flash/SD/LoRa 是事件式的最新狀態快照，不是連續波形，不硬塞進線圖，
        # 改用圖表下緣的文字狀態列即時顯示（見 update() 內 self.diag_text）。
        self.diag_text = self.fig.text(0.01, 0.005, "", fontsize=8, family='monospace', color='dimgray')
        self.fig.tight_layout(rect=(0, 0.02, 1, 1))
        self.fig.canvas.mpl_connect('close_event', self.on_close)

    def on_close(self, event):
        self.stop_event.set()

    def update(self, frame):
        pkts = self.engine.get_packets_snapshot()
        if not pkts:
            return self.line_ax,

        t0 = pkts[0]["tick_ms"]
        times = [(p["tick_ms"] - t0) / 1000.0 for p in pkts]

        max_p = 200
        t_sub = times[-max_p:]
        p_sub = pkts[-max_p:]

        self.line_ax.set_data(t_sub, [p["imu_ax_mg"] for p in p_sub])
        self.line_ay.set_data(t_sub, [p["imu_ay_mg"] for p in p_sub])
        self.line_az.set_data(t_sub, [p["imu_az_mg"] for p in p_sub])

        self.line_gx.set_data(t_sub, [p["gyro_x_dps"] for p in p_sub])
        self.line_gy.set_data(t_sub, [p["gyro_y_dps"] for p in p_sub])
        self.line_gz.set_data(t_sub, [p["gyro_z_dps"] for p in p_sub])

        self.line_alt.set_data(t_sub, [p["baro_alt_cm"] for p in p_sub])

        self.line_bat.set_data(t_sub, [p["bat_mv"] for p in p_sub])

        self.line_cpum.set_data(t_sub, [p["cpu_main_x10"] / 10.0 for p in p_sub])
        self.line_cpue.set_data(t_sub, [p.get("cpu_ekf_x10", 0) / 10.0 for p in p_sub])

        for ax in self.axs:
            ax.relim()
            ax.autoscale_view()

        # Flash/SD/LoRa 狀態列：事件式最新值，沒出現過的欄位顯示 "--" 而非直接省略，
        # 讓使用者能分辨「已檢查且正常」跟「本次擷取根本沒看到這行 log」的差別。
        diag = self.engine.get_diag_snapshot()
        fpool = f"{diag['flash_pool_avail']}/{diag['flash_pool_target']}" if "flash_pool_avail" in diag else "--"
        ffail = diag.get("flash_write_fail_count", "--")
        sd_st = "ERROR" if "sd_last_error" in diag else ("OK" if "sd_last_success" in diag else "--")
        if diag.get("lora433_tx_try", 0) > 0:
            l433 = f"{diag['lora433_tx_ok']}/{diag['lora433_tx_try']} ({100.0*diag['lora433_tx_ok']/diag['lora433_tx_try']:.0f}%)"
        else:
            l433 = "--"
        l920 = f"{diag['lora920_sent']} ({diag.get('lora920_last_status','?')})" if "lora920_sent" in diag else "--"
        self.diag_text.set_text(
            f"Flash pool={fpool} fail={ffail}   SD={sd_st}   LoRa433={l433}   LoRa920={l920}"
        )

        self.fig.suptitle(f"🚀 即時遙測收集：已累積 {len(pkts)} 筆封包 ({times[-1]:.1f} 秒) | 按 Enter 或關閉圖表結束", fontsize=11, color='navy')
        return self.line_ax,

    def start(self):
        ani = FuncAnimation(self.fig, self.update, interval=100, blit=False)
        plt.show()


# ===========================================================================
#  報告生成器 (Multi-Format Report Generator)
# ===========================================================================
class ReportGenerator:
    @staticmethod
    def print_terminal_report(res: dict):
        summary = res["summary"]
        checks = res["checks"]

        c_reset = "\033[0m"
        c_bold  = "\033[1m"
        if summary["overall_status"] == "GO":
            c_badge = "\033[42;\033[30m\033[1m  [ GO - 准許發射 ]  " + c_reset
        elif summary["overall_status"] == "WARN":
            c_badge = "\033[43;\033[30m\033[1m  [ WARN - 需注意 ]  " + c_reset
        else:
            c_badge = "\033[41;\033[37m\033[1m  [ NO-GO - 禁止發射 ]  " + c_reset

        print("\n" + "=" * 78)
        print(f"{c_bold}🚀 RocketCom 航電發射前檢查與感測器誤差分析報告{c_reset}")
        print("=" * 78)
        print(f" 測量時間   : {summary['timestamp']}")
        print(f" 統計樣本   : {summary['sample_count']} 筆 ({summary['duration_sec']} 秒 @ {summary['sample_rate_hz']} Hz)")
        print(f" 判定結果   : {c_badge}")
        print(f" 項目統計   : PASS: \033[32m{summary['pass_count']}\033[0m  "
              f"WARN: \033[33m{summary['warn_count']}\033[0m  "
              f"FAIL: \033[31m{summary['fail_count']}\033[0m")
        print("-" * 78)
        print(f"{'類別':<15} {'檢查項目':<22} {'實測值':<12} {'標準限值':<12} {'狀態':<8}")
        print("-" * 78)

        for c in checks:
            st = c["status"]
            if st == "PASS":
                color_st = f"\033[32m[ PASS ]{c_reset}"
            elif st == "WARN":
                color_st = f"\033[33m[ WARN ]{c_reset}"
            else:
                color_st = f"\033[31m[ FAIL ]{c_reset}"

            val_str = f"{c['val']:.2f} {c['unit']}"
            lim_str = f"{c['limit']:.2f} {c['unit']}"
            print(f"{c['category']:<15} {c['name']:<20} {val_str:<12} {lim_str:<12} {color_st}")

        print("=" * 78)
        if summary["fail_count"] > 0:
            print("\033[31m[!] 警告：系統檢測到致命規格超標，請勿進行發射！請依據詳細報告進行排錯。\033[0m")
        elif summary["warn_count"] > 0:
            print("\033[33m[*] 提示：部分感測器接近規範邊界，建議檢查硬體安裝或再次校正。\033[0m")
        else:
            print("\033[32m[✓] 全系統感測器與資源健康良好，具備發射條件！\033[0m")
        print("=" * 78 + "\n")

    @staticmethod
    def generate_markdown(res: dict, filepath: str):
        s = res["summary"]
        c = res["checks"]
        raw = res["raw_stats"]

        badge = "🟢 **[ GO - 准許發射 ]**" if s["overall_status"] == "GO" else (
                "🟡 **[ WARN - 需注意 ]**" if s["overall_status"] == "WARN" else "🔴 **[ NO-GO - 禁止發射 ]**")

        lines = [
            f"# 🚀 RocketCom 航電發射前檢查與感測器誤差分析報告",
            f"",
            f"- **發射判定狀態**：{badge}",
            f"- **報告生成時間**：`{s['timestamp']}`",
            f"- **遙測採樣時間**：`{s['duration_sec']} 秒` (`{s['sample_count']} 筆封包` @ `{s['sample_rate_hz']} Hz`)",
            f"- **檢查統計結果**：`PASS: {s['pass_count']}` | `WARN: {s['warn_count']}` | `FAIL: {s['fail_count']}`",
            f"",
            f"---",
            f"",
            f"## 一、 發射前檢查項目明細 (Pre-Flight Checklist)",
            f"",
            f"| 類別 | 檢查項目 | 實測數值 | 標準規範限值 | 狀態 | 備註說明 |",
            f"| :--- | :--- | :--- | :--- | :---: | :--- |",
        ]

        for item in c:
            st_icon = "✅ PASS" if item["status"] == "PASS" else ("⚠️ WARN" if item["status"] == "WARN" else "❌ FAIL")
            lines.append(f"| {item['category']} | {item['name']} | `{item['val']:.2f} {item['unit']}` | `{item['limit']:.2f} {item['unit']}` | {st_icon} | {item['detail']} |")

        lines.extend([
            f"",
            f"---",
            f"",
            f"## 二、 各感測器與系統詳細數據統計 (Detailed Statistics)",
            f"",
            f"### 1. BMI088 低 G 加速度計 (IMU Accel)",
            f"- **三軸平均值 (mg)**: X=`{raw['bmi088_accel']['ax']['mean']:.2f}`, Y=`{raw['bmi088_accel']['ay']['mean']:.2f}`, Z=`{raw['bmi088_accel']['az']['mean']:.2f}`",
            f"- **三軸雜訊 (RMS mg)**: X=`{raw['bmi088_accel']['ax']['stddev']:.2f}`, Y=`{raw['bmi088_accel']['ay']['stddev']:.2f}`, Z=`{raw['bmi088_accel']['az']['stddev']:.2f}`",
            f"- **重力向量模長 |a|**: 平均=`{raw['bmi088_accel']['norm']['mean']:.2f} mg`, 與 1.0g (1000mg) 偏差=`{raw['bmi088_accel']['norm_err_mg']:.2f} mg`",
            f"",
            f"### 2. BMI088 陀螺儀 (IMU Gyro)",
            f"- **零偏角速度 (dps)**: X=`{raw['bmi088_gyro']['gx']['mean']:.3f}`, Y=`{raw['bmi088_gyro']['gy']['mean']:.3f}`, Z=`{raw['bmi088_gyro']['gz']['mean']:.3f}`",
            f"- **角速度靜態雜訊 (RMS dps)**: X=`{raw['bmi088_gyro']['gx']['stddev']:.4f}`, Y=`{raw['bmi088_gyro']['gy']['stddev']:.4f}`, Z=`{raw['bmi088_gyro']['gz']['stddev']:.4f}`",
            f"- **最大零偏幅度**: `{raw['bmi088_gyro']['max_offset_dps']:.3f} dps`",
            f"",
            f"### 3. ADXL375 高 G 加速度計 (High-G)",
            f"- **靜態基線偏置 (cg, 1cg=0.01g)**: X=`{raw['adxl375']['ax']['mean']:.1f}`, Y=`{raw['adxl375']['ay']['mean']:.1f}`, Z=`{raw['adxl375']['az']['mean']:.1f}`",
            f"- **靜態雜訊 (RMS cg)**: X=`{raw['adxl375']['ax']['stddev']:.2f}`, Y=`{raw['adxl375']['ay']['stddev']:.2f}`, Z=`{raw['adxl375']['az']['stddev']:.2f}`",
            f"",
            f"### 4. BMP388 氣壓計 (Barometer)",
            f"- **靜態氣壓平均**: `{raw['bmp388']['press']['mean']:.1f} Pa` (雜訊=`{raw['bmp388']['press']['stddev']:.2f} Pa RMS`)",
            f"- **相對高度平均**: `{raw['bmp388']['alt']['mean']:.1f} cm` (雜訊=`{raw['bmp388']['alt']['stddev']:.2f} cm RMS`)",
            f"- **靜止高度飄移率**: `{raw['bmp388']['drift_rate_ms']:.4f} m/s` (Peak-to-Peak=`{raw['bmp388']['alt']['p2p']:.1f} cm`)",
            f"",
            f"### 5. MMC5983MA 磁力計 (Magnetometer)",
            f"- **磁場向量平均 (mG)**: X=`{raw['mmc5983']['mx']['mean']:.1f}`, Y=`{raw['mmc5983']['my']['mean']:.1f}`, Z=`{raw['mmc5983']['mz']['mean']:.1f}`",
            f"- **地磁總磁場模長 |B|**: `{raw['mmc5983']['norm']['mean']:.1f} mG` (雜訊=`{raw['mmc5983']['norm']['stddev']:.2f} mG RMS`)",
            f"",
            f"### 6. CPU 資源與系統即時性 (CPU & Real-Time Performance)",
            f"- **MainTask CPU 佔用率**: 平均=`{raw['cpu']['main']['mean']:.1f}%`, 最大=`{raw['cpu']['main']['max_val']:.1f}%`",
            f"- **EKFTask CPU 佔用率**: 平均=`{raw['cpu']['ekf']['mean']:.1f}%`, 最大=`{raw['cpu']['ekf']['max_val']:.1f}%`",
            f"- **電池電壓**: 平均=`{raw['power']['bat_mv']['mean']/1000.0:.2f}V` (紋波=`{raw['power']['bat_mv']['stddev']:.1f} mV RMS`)",
            f"",
            f"---",
            f"",
            f"## 三、 排錯與維護建議",
            f"1. **如若 BMI088 陀螺儀/加速度計發出 WARN/FAIL**：請確認航電板是否於平整表面靜止，並執行板上校正程序。",
            f"2. **如若 MMC5983MA 磁場異常**：請檢查周遭是否有螺絲起子、馬達或大電流走線干擾，並使用 `mag_calibrate.py` 進行硬鐵校正。",
            f"3. **如若 CPU 佔用率過高 (>35%)**：請檢查無效的死迴圈或 Log 列印頻率，確保發射態 RTOS 任務即時性。",
            f""
        ])

        with open(filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

    @staticmethod
    def generate_json(res: dict, filepath: str):
        clean_res = dict(res)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(clean_res, f, ensure_ascii=False, indent=2)

    @staticmethod
    def generate_html(res: dict, filepath: str):
        summary = res["summary"]
        checks = res["checks"]
        ts = res["time_series"]

        badge_class = "go" if summary["overall_status"] == "GO" else ("warn" if summary["overall_status"] == "WARN" else "nogo")
        badge_text = f"LAUNCH STATUS: {summary['overall_status']}"

        rows_html = ""
        for c in checks:
            st = c["status"]
            st_cls = "st-pass" if st == "PASS" else ("st-warn" if st == "WARN" else "st-fail")
            rows_html += f"""
            <tr>
                <td><strong>{c['category']}</strong></td>
                <td>{c['name']}</td>
                <td><code>{c['val']:.2f} {c['unit']}</code></td>
                <td><code>{c['limit']:.2f} {c['unit']}</code></td>
                <td><span class="status-pill {st_cls}">{st}</span></td>
                <td class="text-muted">{c['detail']}</td>
            </tr>
            """

        html_content = f"""<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RocketCom 發射前檢查與感測器報告</title>
    <style>
        :root {{
            --bg: #0f172a;
            --card-bg: #1e293b;
            --text: #f8fafc;
            --text-muted: #94a3b8;
            --border: #334155;
            --pass: #22c55e;
            --warn: #eab308;
            --fail: #ef4444;
            --accent: #38bdf8;
        }}
        body {{
            font-family: system-ui, -apple-system, sans-serif;
            background-color: var(--bg);
            color: var(--text);
            margin: 0;
            padding: 24px;
            line-height: 1.5;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
        }}
        .header {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid var(--border);
            padding-bottom: 16px;
            margin-bottom: 24px;
        }}
        h1 {{ margin: 0; font-size: 1.8rem; letter-spacing: -0.025em; }}
        .badge {{
            padding: 8px 20px;
            border-radius: 9999px;
            font-weight: 800;
            font-size: 1.1rem;
            letter-spacing: 0.05em;
        }}
        .badge.go {{ background: rgba(34, 197, 94, 0.2); color: var(--pass); border: 2px solid var(--pass); }}
        .badge.warn {{ background: rgba(234, 179, 8, 0.2); color: var(--warn); border: 2px solid var(--warn); }}
        .badge.nogo {{ background: rgba(239, 68, 68, 0.2); color: var(--fail); border: 2px solid var(--fail); }}

        .grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
            gap: 16px;
            margin-bottom: 24px;
        }}
        .card {{
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: 12px;
            padding: 16px;
        }}
        .card .title {{ font-size: 0.85rem; color: var(--text-muted); text-transform: uppercase; }}
        .card .value {{ font-size: 1.8rem; font-weight: 700; margin-top: 4px; color: var(--accent); }}

        table {{
            width: 100%;
            border-collapse: collapse;
            background: var(--card-bg);
            border-radius: 12px;
            overflow: hidden;
            border: 1px solid var(--border);
            margin-bottom: 24px;
        }}
        th, td {{ padding: 12px 16px; text-align: left; border-bottom: 1px solid var(--border); }}
        th {{ background: #0f172a; color: var(--text-muted); font-size: 0.85rem; text-transform: uppercase; }}
        tr:hover {{ background: rgba(255,255,255,0.02); }}

        .status-pill {{
            padding: 4px 10px;
            border-radius: 6px;
            font-size: 0.75rem;
            font-weight: 700;
        }}
        .st-pass {{ background: rgba(34, 197, 94, 0.15); color: var(--pass); }}
        .st-warn {{ background: rgba(234, 179, 8, 0.15); color: var(--warn); }}
        .st-fail {{ background: rgba(239, 68, 68, 0.15); color: var(--fail); }}
        .text-muted {{ color: var(--text-muted); font-size: 0.9rem; }}

        .chart-box {{
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: 12px;
            padding: 20px;
            margin-bottom: 24px;
        }}
        canvas {{ width: 100%; height: 260px; }}
    </style>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
    <div class="container">
        <div class="header">
            <div>
                <h1>🚀 RocketCom 航電發射前檢查 Dashboard</h1>
                <div class="text-muted">生成時間：{summary['timestamp']} | 樣本：{summary['sample_count']} 筆 @ {summary['sample_rate_hz']} Hz</div>
            </div>
            <div class="badge {badge_class}">{badge_text}</div>
        </div>

        <div class="grid">
            <div class="card">
                <div class="title">PASS 項目</div>
                <div class="value" style="color: var(--pass);">{summary['pass_count']}</div>
            </div>
            <div class="card">
                <div class="title">WARN 項目</div>
                <div class="value" style="color: var(--warn);">{summary['warn_count']}</div>
            </div>
            <div class="card">
                <div class="title">FAIL 項目</div>
                <div class="value" style="color: var(--fail);">{summary['fail_count']}</div>
            </div>
            <div class="card">
                <div class="title">Main CPU 佔用</div>
                <div class="value">{res['raw_stats']['cpu']['main']['mean']:.1f}%</div>
            </div>
        </div>

        <h2>📋 發射前檢查項目 (Checklist)</h2>
        <table>
            <thead>
                <tr>
                    <th>類別</th>
                    <th>檢查項目</th>
                    <th>實測數值</th>
                    <th>規範限值</th>
                    <th>狀態</th>
                    <th>詳細說明</th>
                </tr>
            </thead>
            <tbody>
                {rows_html}
            </tbody>
        </table>

        <h2>📈 即時感測器與 CPU 波形監控</h2>
        <div class="chart-box">
            <h3>BMI088 低 G 加速度 (mg)</h3>
            <canvas id="chartAcc"></canvas>
        </div>
        <div class="chart-box">
            <h3>BMI088 陀螺儀角速度 (dps)</h3>
            <canvas id="chartGyro"></canvas>
        </div>
        <div class="chart-box">
            <h3>MainTask & EKFTask CPU 佔用率 (%)</h3>
            <canvas id="chartCpu"></canvas>
        </div>
    </div>

    <script>
        const ticks = {json.dumps([round((t - ts['ticks'][0])/1000.0, 2) for t in ts['ticks']])};
        
        new Chart(document.getElementById('chartAcc'), {{
            type: 'line',
            data: {{
                labels: ticks,
                datasets: [
                    {{ label: 'ax (mg)', data: {json.dumps(ts['imu_ax'])}, borderColor: '#ef4444', borderWidth: 1.5, pointRadius: 0 }},
                    {{ label: 'ay (mg)', data: {json.dumps(ts['imu_ay'])}, borderColor: '#22c55e', borderWidth: 1.5, pointRadius: 0 }},
                    {{ label: 'az (mg)', data: {json.dumps(ts['imu_az'])}, borderColor: '#38bdf8', borderWidth: 1.5, pointRadius: 0 }}
                ]
            }},
            options: {{ responsive: true, maintainAspectRatio: false }}
        }});

        new Chart(document.getElementById('chartGyro'), {{
            type: 'line',
            data: {{
                labels: ticks,
                datasets: [
                    {{ label: 'gx (dps)', data: {json.dumps(ts['gyro_x'])}, borderColor: '#f97316', borderWidth: 1.5, pointRadius: 0 }},
                    {{ label: 'gy (dps)', data: {json.dumps(ts['gyro_y'])}, borderColor: '#a855f7', borderWidth: 1.5, pointRadius: 0 }},
                    {{ label: 'gz (dps)', data: {json.dumps(ts['gyro_z'])}, borderColor: '#06b6d4', borderWidth: 1.5, pointRadius: 0 }}
                ]
            }},
            options: {{ responsive: true, maintainAspectRatio: false }}
        }});

        new Chart(document.getElementById('chartCpu'), {{
            type: 'line',
            data: {{
                labels: ticks,
                datasets: [
                    {{ label: 'MainTask CPU (%)', data: {json.dumps(ts['cpu_main'])}, borderColor: '#38bdf8', borderWidth: 2, pointRadius: 0 }},
                    {{ label: 'EKFTask CPU (%)', data: {json.dumps(ts['cpu_ekf'])}, borderColor: '#eab308', borderWidth: 2, pointRadius: 0 }}
                ]
            }},
            options: {{ responsive: true, maintainAspectRatio: false }}
        }});
    </script>
</body>
</html>
"""
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(html_content)


# ===========================================================================
#  模擬測試數據生成器 (Selftest Data Synthesizer)
# ===========================================================================
def generate_selftest_packets(count=150) -> list:
    import random
    packets = []
    base_tick = 100000
    for i in range(count):
        tick = base_tick + i * 100
        ax = int(random.gauss(12.0, 2.5))
        ay = int(random.gauss(-8.0, 2.5))
        az = int(random.gauss(998.0, 3.0))

        gx = int(random.gauss(0.05, 0.02))
        gy = int(random.gauss(-0.02, 0.02))
        gz = int(random.gauss(0.01, 0.02))

        press = int(101325 + random.gauss(0, 1.2))
        alt = int(2500 + random.gauss(0, 8.0))

        mx = int(210 + random.gauss(0, 1.5))
        my = int(-50 + random.gauss(0, 1.5))
        mz = int(410 + random.gauss(0, 2.0))

        bat = int(7900 + random.gauss(0, 8.0))    # 2S 18650：7.4~8.4V 範圍中段
        cpu_main = int(145 + random.gauss(0, 5.0))
        cpu_ekf = int(220 + random.gauss(0, 6.0))

        pkt = {
            "sync0": SYNC0, "sync1": SYNC1, "seq": i & 0xFF, "fsm_state": 1, "tick_ms": tick,
            "ekf_pos_z_cm": alt, "ekf_vel_z_cms": 0,
            "ekf_q0": 10000, "ekf_q1": 0, "ekf_q2": 0, "ekf_q3": 0,
            "baro_alt_cm": alt, "baro_press_pa": press,
            "imu_ax_mg": ax, "imu_ay_mg": ay, "imu_az_mg": az,
            "gyro_x_dps": gx, "gyro_y_dps": gy, "gyro_z_dps": gz,
            "hg_ax_cg": int(ax/10), "hg_ay_cg": int(ay/10), "hg_az_cg": int(az/10),
            "mag_x_mg": mx, "mag_y_mg": my, "mag_z_mg": mz,
            "gps_lat_1e6": 24789012, "gps_lon_1e6": 12098765, "gps_alt_m": 25,
            "gps_sats": 12, "gps_fix": 1,
            "bat_mv": bat, "cpu_main_x10": cpu_main, "cpu_ekf_x10": cpu_ekf,
            "flags": 0, "health_bits": 0, "sensor_bits": 0,
            # VF 跟 EKF 給幾乎一致的值，模擬雙估計器健康一致的情況
            "vf_pos_z_cm": alt + int(random.gauss(0, 2.0)), "vf_vel_z_cms": int(random.gauss(0, 2.0)),
            "peer_fsm_state": 0, "peer_flags": 0, "peer_baro_cm": 0, "peer_link": 0,
            "peer_vf_h_cm": 0, "peer_vf_v_cms": 0,
            "arm_flags": 0, "peer_bench_arb": 0, "profile_flags": 0,
            "crc16": 0, "_bin": True,
        }
        packets.append(pkt)
    return packets


# ===========================================================================
#  主程序 (Main Routine)
# ===========================================================================
def main():
    parser = argparse.ArgumentParser(description="RocketCom 航電發射前檢查與感測器誤差分析工具")
    parser.add_argument("--port", help="USB CDC 序列埠路徑 (如 /dev/cu.usbmodem1101 或 AUTO)")
    parser.add_argument("--baud", type=int, default=460800, help="鮑率 (預設 460800)")
    parser.add_argument("--duration", type=float, default=None, help="設定固定採樣持續時間(秒)；未指定時由使用者按 Enter 決定結束時間")
    parser.add_argument("--file", help="輸入二進制遙測 dump 檔案路徑")
    parser.add_argument("--selftest", action="store_true", help="執行模擬數據自我測試")
    parser.add_argument("--no-gui", action="store_true", help="不跳出即時 Matplotlib 圖表視窗（純文字模式）")
    parser.add_argument("--out-dir", default=os.path.join(SYS_PARENT, "reports"), help="報告輸出目錄")
    args = parser.parse_args()

    engine = SensorAnalyzerEngine()
    stop_event = threading.Event()

    if not args.selftest and not args.file and not args.port:
        auto_p = serial_link.auto_port() if serial_link else None
        if auto_p and any(kw in auto_p.lower() for kw in ("usbmodem", "usbserial", "ttyacm", "ttyusb", "cu.usb")):
            print(f"🔌 檢測到實體 USB 航電板 ({auto_p})，自動進入 USB 直連即時監測模式！")
            args.port = auto_p
        else:
            try:
                from file_selector import select_input_file
                args.file = select_input_file(title="請選擇感測器/遙測數據紀錄檔", extensions=[".bin", ".csv", ".log"])
            except Exception as e:
                print(f"[WARNING] 無法啟動互動式檔案選擇器: {e}")

    if args.selftest:
        print("[SELFTEST] 執行發射前檢查與誤差分析邏輯測試...")
        packets = generate_selftest_packets(150)
        for p in packets:
            engine.add_packet(p)
    elif args.file:
        print(f"[FILE] 正在讀取並解碼遙測檔: {args.file} ...")
        if not os.path.exists(args.file):
            print(f"[ERROR] 檔案不存在: {args.file}")
            sys.exit(1)

        file_size = os.path.getsize(args.file)
        bytes_read = 0
        last_pct = -1
        parser = StreamParser(engine.add_packet, on_diag=engine.update_diag)
        with open(args.file, "rb") as f:
            while True:
                chunk = f.read(65536)
                if not chunk:
                    break
                bytes_read += len(chunk)
                if file_size > 0:
                    pct = int((bytes_read / file_size) * 100)
                    if pct != last_pct and (pct % 10 == 0 or pct == 100):
                        last_pct = pct
                        sys.stdout.write(f"\r[PROGRESS] ⏳ 讀取遙測紀錄進度: {pct:3d}% ({bytes_read / (1024*1024):.1f} / {file_size / (1024*1024):.1f} MB)")
                        sys.stdout.flush()
                parser.feed(chunk)
        if file_size > 0:
            sys.stdout.write("\r[PROGRESS] ✅ 遙測紀錄讀取解碼完成 (100%)\n")
            sys.stdout.flush()
    else:
        # CDC / Serial 連線
        if serial_link is None:
            print("[ERROR] 找不到 serial_link 模組，請確認目錄結構。")
            sys.exit(1)

        port = args.port or serial_link.resolve_port()
        print(f"🔄 正在連線航電板 USB CDC / Serial: {port} @ {args.baud} baud ...")
        try:
            ser = serial_link.open_serial(port, args.baud, timeout=0.5)
        except Exception as e:
            print(f"[ERROR] 無法開啟串口 {port}: {e}")
            sys.exit(1)

        # 實體雙模串流解析器
        stream_parser = StreamParser(engine.add_packet, on_diag=engine.update_diag)

        # 序列讀取線程
        def rx_thread_proc():
            t_start = time.time()
            while not stop_event.is_set():
                if args.duration and (time.time() - t_start >= args.duration):
                    stop_event.set()
                    break
                try:
                    chunk = ser.read(4096)
                    if chunk:
                        stream_parser.feed(chunk)
                except Exception:
                    pass
                time.sleep(0.005)

        t_rx = threading.Thread(target=rx_thread_proc, daemon=True)
        t_rx.start()

        # 鍵盤監聽線程
        def wait_enter_proc():
            if sys.stdin and sys.stdin.isatty():
                try:
                    input()
                except (EOFError, KeyboardInterrupt):
                    pass
                stop_event.set()

        t_kb = threading.Thread(target=wait_enter_proc, daemon=True)
        t_kb.start()

        print("\n" + "=" * 70)
        print(f"📡 已成功連線航電板 ({port})！開始即時數據採樣中...")
        if args.duration:
            print(f"⏱️ 採樣時間：預設 {args.duration} 秒 (可隨時按 Enter 提早結束)")
        else:
            print("⏱️ 採樣時間：由您完全決定！欲結束採樣請隨時在終端機按下 [Enter] 鍵")
        print("=" * 70 + "\n")

        # 開啟 GUI 即時繪圖或進行 Terminal 印出
        if HAS_MATPLOTLIB and not args.no_gui:
            plotter = LivePlotter(engine, stop_event)
            plotter.start()
            stop_event.set()
        else:
            t0 = time.time()
            try:
                while not stop_event.is_set():
                    time.sleep(0.2)
                    pkts = engine.get_packets_snapshot()
                    elapsed = time.time() - t0
                    sys.stdout.write(f"\r[LIVE 數據採樣中] 已累積 {len(pkts)} 筆遙測數據 | 時間: {elapsed:.1f}s (按 Enter 結束) ")
                    sys.stdout.flush()
            except KeyboardInterrupt:
                stop_event.set()

        print("\n\n[LIVE] 數據採樣完成！正在開始進行全板感測器與 CPU 佔用率統計分析...")

    # 執行分析
    res = engine.analyze()
    if "error" in res:
        print(f"[ERROR] 分析失敗: {res['error']}")
        sys.exit(1)

    # 產出 Console 報告
    ReportGenerator.print_terminal_report(res)

    # 建立輸出目錄與檔案
    os.makedirs(args.out_dir, exist_ok=True)
    ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    md_file = os.path.join(args.out_dir, f"sensor_error_report_{ts_str}.md")
    json_file = os.path.join(args.out_dir, f"sensor_error_report_{ts_str}.json")
    html_file = os.path.join(args.out_dir, f"sensor_error_report_{ts_str}.html")

    ReportGenerator.generate_markdown(res, md_file)
    ReportGenerator.generate_json(res, json_file)
    ReportGenerator.generate_html(res, html_file)

    print(f"📝 超詳細 Markdown 報告已生成: file://{os.path.abspath(md_file)}")
    print(f"📊 JSON 結構化數據檔已生成: file://{os.path.abspath(json_file)}")
    print(f"🌐 互動式 HTML Dashboard 已生成: file://{os.path.abspath(html_file)}")

    if res["summary"]["overall_status"] == "NO-GO":
        sys.exit(2)


if __name__ == "__main__":
    main()
