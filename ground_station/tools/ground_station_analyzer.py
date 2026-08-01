#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ground_station_analyzer.py — 地面站接收品質與紀錄管線分析工具 (Ground Station Link/Recording Analyzer)
===========================================================================
與 sensor_error_analyzer.py（分析「火箭端」感測器誤差）互補：本工具分析「地面站端」
雙鏈路 LoRa 接收系統本身的健康度 —— 收得到嗎（頻率）、收得乾淨嗎（品質）、
記得下來嗎（紀錄速度／管線是否卡頓）。資料來源見 ground_station.c / gs_lora_test.c：

  1. 【即時 Console 擷取】：連上地面站板 UART2（同 gs_lora_test.c 命令列，460800 baud），
     解析 [GS_PKT]/[GS_PKT?]（逐包）、[GS_STAT]（2 秒週期累計計數器）、
     [STATS]（`stats`/`stats auto N` 命令觸發的 RSSI/SNR/速率統計塊）、[GS_GPS]（地面站自身定位）。
  2. 【離線 CSV 分析】：直接分析已落地的 SD 卡紀錄檔 GSLOGnnn.CSV（見 gs_log.c
     GsLog_FormatCsvRow）。rx_utc_ms 欄位是 GPS 紀律牆鐘時間戳，量測「實際落地間隔」
     最準確 —— 這就是本工具判斷「紀錄速度」有無卡頓（如已知 Flash 用前才擦約 400ms
     的 sector erase 阻塞）最直接的證據，不受即時擷取時序列印/序列埠延遲干擾。

功能特色：
  1. 【雙鏈路分開統計】：433(E22，開 REG3 bit7 才有 RSSI、無 SNR) /
     920(E80/LR1121，有 RSSI/SNR) 分別計算
     封包速率、平均/最長封包間隔、CRC 錯誤率、Resync（雜訊）比例。
  2. 【合併封包間隔】：兩鏈路依 seq+時間相近去重後看「地面站兩條鏈路同時斷多久」——
     這才是雙鏈路備援真正該看的指標，比任一單鏈路的間隔更關鍵。
     ★2026-07-30：本工具不再計算任何「到達率丟失／丟包率」。那類指標必須先反推
     「火箭端到底發了幾包」當分母，而 433 的發送機會取決於未實測的空中時間、AUX
     背壓與上行接收窗相位，分母本身就是猜的，算出來的丟失率不可信（歷次 report
     的 Union Loss 甚至換算出遠超硬體上限的隱含 tick 頻率）。判定改用只依賴「實際
     收到什麼」的量測值：有效封包頻率夠不夠、封包間隔會不會出現長空窗。
  3. 【紀錄管線卡頓偵測】：量測連續封包落地間隔，抓出停頓（stall），對應已知的
     Flash erase-ahead 阻塞等紀錄延遲風險。
  4. 【超詳細報告輸出】：ANSI 終端表格、Markdown 報告、JSON 數據、HTML 儀表板。

用法：
  python3 ground_station_analyzer.py                          # 即時連線地面站板，開圖表視窗
  python3 ground_station_analyzer.py --duration 60             # 固定擷取 60 秒
  python3 ground_station_analyzer.py --no-gui                  # 純文字即時模式
  python3 ground_station_analyzer.py --csv GSLOG003.CSV        # 離線分析 SD 卡紀錄檔（最準確）
  python3 ground_station_analyzer.py --file console_dump.log   # 離線分析文字 console log 檔
  python3 ground_station_analyzer.py --selftest                # 模擬資料自我測試
"""

import argparse
import csv
import datetime
import json
import math
import os
import re
import sys
import threading
import time

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

SYS_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SYS_PARENT not in sys.path:
    sys.path.insert(0, SYS_PARENT)

try:
    import serial_link
except ImportError:
    serial_link = None

LINK_433 = 433
LINK_920 = 920
RSSI_SNR_NA = -32768   # 見 gs_log.h GS_RSSI_NA/GS_SNR_NA（433 恆無 SNR；RSSI 視 E22 REG3 bit7 而定）

# 兩鏈路收到「同一次火箭傳送」的到達時間差：main.c 排程 433/920 幾乎同步送出，
# 遠小於下行封包週期(通常 >=100ms)，故拿來判斷合併去重時綽綽有餘、又不會誤併相鄰兩包。
DEDUPE_WINDOW_S = 0.6

# 火箭端下行排程（見 main.c LoRaTelemetry_Task）：Telemetry_Build() 每 LORA_TELEM_PERIOD_MS
# (100ms) 打包一筆並「共用同一個 seq」，920(E80) 每個時槽都發(僅受 BUSY 背壓跳過)。
# ★2026-07-28（main.c:2633 起註解）：433(E22) 的 AUX 背壓已是真的限流，LORA433_TX_EVERY
# 已從 3 改回 1（每槽都嘗試發送），實際發送速率改由空中時間自然限流，不再靠時槽除數硬撐。
LORA_TELEM_PERIOD_MS = 100.0
LORA433_TX_EVERY = 1

# 上行接收窗（main.c UPLINK_LISTEN_PERIOD_MS / UPLINK_LISTEN_HOLD_MS，FEATURE_UPLINK_DEPLOY
# 開啟時生效，主航電預設開）：
# ★2026-07-30：韌體把這個窗從「每 10 個時槽跳 1 槽」改成「每 3000ms 連續 800ms 完全不發」。
# 原因見 main.c 註解——時槽只有 100ms，而一包 433 的空中時間 ~390~580ms，跳過 1 個排程
# 時槽時模組還在把前一包送上空中，空中根本沒有靜默，地面站的 ARM/DEPLOY 永遠打不進來
# （實測：連送 10 次 ARM 全無 ACK）。窗必須用時間定義且長度 > 一包空中時間才成立。
# ★★2026-07-30 第二輪：只有「窗內不餵資料」還是不通（實測 LORA433_RX_ONLY=1 就通、
# 設回 0 就不通，證明 RF 層無罪）。根因是名目窗長 != 真正的靜默長度——listen_slot 只在
# 迴圈開頭判斷一次，一包在窗開始前一瞬間才起飛的封包會吃掉窗的前半段，800ms 的窗真正
# 靜默只剩 ~300ms。韌體因此在窗「之前」加了一段發射守衛帶 UPLINK_TX_GUARD_MS，讓在途
# 封包在窗開始前送完，並把 HOLD 從 800 縮到 400（總不發射時間 800→1000ms）。
# 對本分析器的意義：433 的規律性 seq 間隙是「每 3000ms 有一段連續 1000ms 完全沒發」
# ——換算成時槽就是每 30 個 tick 連續 10 個 tick 不發射。
UPLINK_LISTEN_PERIOD_MS = 3000.0
UPLINK_LISTEN_HOLD_MS = 400.0
UPLINK_TX_GUARD_MS = 600.0
# 不發射總時長（守衛帶 + 窗，兩段在時間軸上相連）佔全部時間的比例：
# 433 可用發送時間只剩 (1 - 這個比例)。
UPLINK_LISTEN_QUIET_MS = UPLINK_TX_GUARD_MS + UPLINK_LISTEN_HOLD_MS       # = 1000
UPLINK_LISTEN_DUTY = UPLINK_LISTEN_QUIET_MS / UPLINK_LISTEN_PERIOD_MS     # = 0.333
UPLINK_LISTEN_PERIOD_TICKS = round(UPLINK_LISTEN_PERIOD_MS / LORA_TELEM_PERIOD_MS)  # = 30
UPLINK_LISTEN_HOLD_TICKS = round(UPLINK_LISTEN_QUIET_MS / LORA_TELEM_PERIOD_MS)     # = 10

# 433/E22 實際可達速率不再是「時槽除數」而是空中時間物理上限，但這個空中時間本身
# ★還沒有實測、只有韌體裡兩份互相矛盾的估計值★：
#   main.c:2635  ≈386.7ms（116*8bit/2400bps，純位元數/鮑率的天真算法）
#   lora_e22.c:546 ≈580ms（同一顆模組同一個封包大小，但沒寫算法來源）
# E22-400T30S 規格書(Datasheets/E22-400T30S_UserManual_EN_v1.8.pdf)證實這顆模組底層
# 是 LoRa 調變(SX1262)，「air data rate」只是展頻參數的抽象標籤，不是序列埠那種純位元
# 速率——跟 920(E80/LR1121) 一樣，真實空中時間還要疊加前導碼/表頭等 LoRa 開銷，386.7ms
# 那個天真算法幾乎必然低估。兩份估計都不可信的情況下，先採用比較保守（考慮了 LoRa 開銷
# 方向、不是純位元數/鮑率）的 580ms，但這仍然是估計值，不是實測——長遠應該在韌體端
# 直接量測 AUX 從忙轉閒的實際耗時（例如在 LoRaE22_Send 判定 AUX 轉閒置那一刻打時間戳，
# 跟上一次成功發送時間戳相減），回填這裡取代猜測值。
LORA433_AIRTIME_S = 0.580   # ← 估計值，見上方註解；非實測
NOMINAL_RATE_HZ = {
    LINK_433: (1.0 / LORA433_AIRTIME_S) * (1.0 - UPLINK_LISTEN_DUTY),  # ≈1.15 Hz(估計)
    LINK_920: 1000.0 / LORA_TELEM_PERIOD_MS,                        # 10 Hz（受空中時間/BUSY 影響）
}

# 433 兩次成功發射之間的物理下限：LoRaE22_Send() 遇到 AUX busy（上一包還在空中）會直接
# 跳過、不等待，所以最少要隔 ceil(空中時間/tick週期) 個 tick。★這只用於 --selftest 模擬
# 資料的產生（讓假資料長得像真排程），不再拿來當任何判定指標的分母——見檔頭說明。
LORA433_MIN_TX_SPACING_TICKS = math.ceil(LORA433_AIRTIME_S / (LORA_TELEM_PERIOD_MS / 1000.0))  # = 6(估計)

# ===========================================================================
#  即時 Console 逐行規則 (與 ground_station.c / gs_lora_test.c printf 格式同步)
# ===========================================================================
_RE_GS_PKT_LINK  = re.compile(r"\[GS_PKT\] link:(\d+)MHz")
_RE_GS_PKT_RSSI  = re.compile(r"\brssi:(-?\d+)")
_RE_GS_PKT_SNR   = re.compile(r"\bsnr:(-?\d+)")
_RE_GS_PKT_SEQ   = re.compile(r"\bseq:(\d+)")
_RE_GS_PKT_BAD   = re.compile(r"\[GS_PKT\?\] link:(\d+)MHz CRC_BAD")

_RE_GS_STAT = re.compile(
    r"\[GS_STAT\] HW:433=(\w+) 920=(\w+) \| "
    r"433 raw=(\d+) ok=(\d+) crc=(\d+) rsync=(\d+) \| "
    r"920 ok=(\d+) crc=(\d+) rsync=(\d+) \| "
    r"pkts 433=(\d+) 920=(\d+)")

_RE_GS_GPS_FIX  = re.compile(r"\[GS_GPS\] FIX sats=(\d+)")
_RE_GS_GPS_SRCH = re.compile(r"\[GS_GPS\] SEARCHING sats=(\d+) q=(\d+) ok=(\d+) err=(\d+)")

_RE_LORA_INIT = re.compile(r"\[GS_LORA_INIT\] 433MHz\(E22\):(.+?) \| 920MHz\(E80\):(.+)$")

# [STATS] 區塊：`stats` / `stats auto N` 命令觸發，一次印出 433 與 920 各一段（見 gs_lora_test.c print_one_stat）
_RE_STATS_HDR    = re.compile(r"\[STATS\] --- (E22-433|E80-920) ---")
_RE_STATS_PKTOK  = re.compile(r"\[STATS\] pkt_ok=(\d+)\s+crc_err=(\d+)")
_RE_STATS_RATE   = re.compile(r"\[STATS\] rate=([\d.]+) pkt/s\s+elapsed=(\d+)s")
_RE_STATS_RSSI   = re.compile(r"\[STATS\] RSSI: last=(-?\d+) min=(-?\d+) max=(-?\d+) avg=(-?\d+) dBm")
_RE_STATS_SNR    = re.compile(r"\[STATS\] SNR:\s+last=(-?\d+) min=(-?\d+) max=(-?\d+) avg=(-?\d+) \(x0\.25dB\)")


# ===========================================================================
#  規格門檻 (啟發式，非晶片絕對極限；理由見各項註解)
# ===========================================================================
SPEC_LIMITS = {
    # CRC 錯誤率：ok/(ok+crc) 的補數。5% 為訊號尚可接受、10% 已明顯偏弱(對齊
    # sensor_error_analyzer 的 downlink_seq_loss_max_ratio 同量級門檻)。
    "crc_err_max_ratio": 0.05,
    # Resync 比例（433 才有 raw byte 計數，920 只有 rsync 計數無 raw）：
    # rsync/(ok+crc+rsync)。偏高代表「同步位元組頻繁對不上」= 空中雜訊/速率不符為主因。
    "resync_max_ratio": 0.15,
    # 通訊頻率相對韌體排定速率(NOMINAL_RATE_HZ)的比例門檻：低於此比例代表
    # RF 前端可能根本沒同步到訊號(而非單純解碼品質問題)。有效頻率(CRC 正確)與
    # 總頻率(含 CRC 無效)共用這組比例——前者是「資料真的到手」的 GO/NO-GO，
    # 後者用來分辨「沒收到訊號」還是「收到但解不出來」。
    "total_rate_warn_ratio": 0.60,
    "total_rate_fail_ratio": 0.30,
    # 封包間隔（gap）門檻：★取代舊的「到達率丟失」，只看實際收到的封包之間隔了多久，
    # 不需要反推火箭端發了幾包（那個分母是猜的，見檔頭說明）。
    #   平均間隔 = 頻率的倒數，用同一組比例換算成倍率（1/0.6≈1.67、1/0.3≈3.33）。
    #   最長空窗 = 連續斷訊的最壞情況，比平均值更能反映「飛行中會不會突然失聯一段」。
    # 倍率以各鏈路名目間隔(1/NOMINAL_RATE_HZ)為基準，並套一個絕對下限避免 920 名目
    # 間隔只有 100ms 時把正常抖動誤判成空窗。
    "gap_mean_warn_mult": 1.67,
    "gap_mean_fail_mult": 3.33,
    "gap_max_warn_mult": 4.0,
    "gap_max_fail_mult": 8.0,
    "gap_max_warn_floor_ms": 1000.0,
    "gap_max_fail_floor_ms": 2000.0,
    # SX126x/LR1121 (920) 常見「可靠接收」門檻，非晶片絕對靈敏度極限（SF/BW視設定
    # 靈敏度可到 -130dBm 以下，但留餘裕才穩）。
    "rssi_warn_dbm": -110.0,
    "rssi_fail_dbm": -122.0,
    "snr_warn_db": 0.0,
    "snr_fail_db": -7.5,
    # 紀錄管線停頓：已知 Flash 用前才擦，~每 30+ 筆會 ~400ms sector erase 阻塞
    # （見 gs_ground_station 專案備忘），此處抓 600ms(WARN)/1000ms(FAIL) 留餘裕。
    "pipeline_stall_warn_ms": 600.0,
    "pipeline_stall_fail_ms": 1000.0,
    "pipeline_stall_ratio_max": 0.05,   # 停頓筆數佔比上限
    "gps_sats_min": 6,
}


# ===========================================================================
#  統計工具
# ===========================================================================
class StatMetrics:
    def __init__(self, values: list):
        self.count = len(values)
        if self.count == 0:
            self.mean = self.stddev = self.min_val = self.max_val = self.p2p = 0.0
            return
        self.mean = sum(values) / float(self.count)
        var = sum((x - self.mean) ** 2 for x in values) / float(self.count)
        self.stddev = math.sqrt(var)
        self.min_val = min(values)
        self.max_val = max(values)
        self.p2p = self.max_val - self.min_val


def merge_dedupe_events(events: list) -> list:
    """合併兩鏈路事件（按時間排序後），同 seq 且到達時間差 < DEDUPE_WINDOW_S 視為
    同一次火箭傳送被兩鏈路都收到，僅保留先到的一筆。"""
    ordered = sorted(events, key=lambda e: e["t"])
    merged = []
    for e in ordered:
        if merged and merged[-1]["seq"] == e["seq"] and (e["t"] - merged[-1]["t"]) <= DEDUPE_WINDOW_S:
            continue
        merged.append(e)
    return merged


def gap_limits(nominal_rate_hz: float) -> dict:
    """由名目速率換算該鏈路的封包間隔（gap）判定門檻（ms）。

    ★2026-07-30：本工具改用 gap 取代舊的「到達率丟失」。差別在於分母：丟失率要先假設
    「火箭端在這段時間內本來該發幾包」，而 433 的發送機會取決於未實測的空中時間、AUX
    背壓與上行接收窗相位——分母是猜的，算出來的百分比自然不可信。gap 只用「實際收到的
    兩包之間隔了多久」這個直接量測值，不需要任何關於發射端排程的假設；名目速率在這裡
    僅用來決定門檻寬鬆度（門檻本來就是啟發式的），不進入量測值本身。

    最長空窗另外套絕對下限：920 名目間隔只有 100ms，光是 4 倍(400ms)會把正常的排程抖動
    /單包空中時間誤判成空窗，實務上「短暫失聯」至少要到秒級才有意義。"""
    nominal_gap_ms = (1000.0 / nominal_rate_hz) if nominal_rate_hz > 0 else 0.0
    return {
        "nominal_gap_ms": nominal_gap_ms,
        "mean_warn_ms": nominal_gap_ms * SPEC_LIMITS["gap_mean_warn_mult"],
        "mean_fail_ms": nominal_gap_ms * SPEC_LIMITS["gap_mean_fail_mult"],
        "max_warn_ms": max(nominal_gap_ms * SPEC_LIMITS["gap_max_warn_mult"],
                           SPEC_LIMITS["gap_max_warn_floor_ms"]),
        "max_fail_ms": max(nominal_gap_ms * SPEC_LIMITS["gap_max_fail_mult"],
                           SPEC_LIMITS["gap_max_fail_floor_ms"]),
    }


def gap_stats(times: list, warn_ms: float, fail_ms: float) -> dict:
    """連續事件時間戳的間隔統計（ms），用來偵測紀錄管線停頓與鏈路空窗。"""
    if len(times) < 2:
        return {"count": 0, "mean_ms": 0.0, "max_ms": 0.0, "stall_count": 0, "stall_ratio": 0.0}
    gaps_ms = [(times[i] - times[i - 1]) * 1000.0 for i in range(1, len(times))]
    stalls = [g for g in gaps_ms if g > warn_ms]
    return {
        "count": len(gaps_ms),
        "mean_ms": sum(gaps_ms) / len(gaps_ms),
        "max_ms": max(gaps_ms),
        "stall_count": len(stalls),
        "stall_ratio": len(stalls) / float(len(gaps_ms)),
    }


# ===========================================================================
#  即時 Console 串流解析器
# ===========================================================================
class GsConsoleParser:
    def __init__(self, on_pkt, on_pkt_bad, on_gs_stat, on_diag):
        self.on_pkt = on_pkt            # (t, link, rssi, snr, seq)
        self.on_pkt_bad = on_pkt_bad    # (t, link)
        self.on_gs_stat = on_gs_stat    # (t, dict)
        self.on_diag = on_diag          # (key, value)
        self.buf = ""
        self._stats_ctx = None          # 目前 [STATS] 區塊屬於哪個鏈路 ("E22-433"/"E80-920")

        self._gps_lines = 0
        self._gps_fix_n = 0
        self._gps_sat_sum = 0
        self._gps_sat_min = None

    def feed(self, chunk: bytes):
        if not chunk:
            return
        self.buf += chunk.decode(errors="ignore")
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            self._feed_line(line.strip(), time.time())

    def _feed_line(self, line: str, t: float):
        if not line:
            return

        m = _RE_GS_PKT_BAD.search(line)
        if m:
            link = int(m.group(1))
            m_seq = _RE_GS_PKT_SEQ.search(line)
            seq = int(m_seq.group(1)) if m_seq else 0
            self.on_pkt_bad(t, link, seq)
            return

        m = _RE_GS_PKT_LINK.search(line)
        if m:
            link = int(m.group(1))
            m_rssi = _RE_GS_PKT_RSSI.search(line)
            m_snr = _RE_GS_PKT_SNR.search(line)
            m_seq = _RE_GS_PKT_SEQ.search(line)
            rssi = int(m_rssi.group(1)) if m_rssi else RSSI_SNR_NA
            snr = int(m_snr.group(1)) if m_snr else RSSI_SNR_NA
            seq = int(m_seq.group(1)) if m_seq else 0
            self.on_pkt(t, link, rssi, snr, seq)
            return

        m = _RE_GS_STAT.search(line)
        if m:
            g = m.groups()
            self.on_gs_stat(t, {
                "hw433_ok": g[0] == "OK", "hw920_ok": g[1] == "OK",
                "raw433": int(g[2]), "ok433": int(g[3]), "crc433": int(g[4]), "rsync433": int(g[5]),
                "ok920": int(g[6]), "crc920": int(g[7]), "rsync920": int(g[8]),
                "pkts433": int(g[9]), "pkts920": int(g[10]),
            })
            return

        m = _RE_GS_GPS_FIX.search(line)
        if m:
            sats = int(m.group(1))
            self._gps_lines += 1
            self._gps_fix_n += 1
            self._gps_sat_sum += sats
            self._gps_sat_min = sats if self._gps_sat_min is None else min(self._gps_sat_min, sats)
            self.on_diag("gs_gps_fix_ratio", self._gps_fix_n / float(self._gps_lines))
            self.on_diag("gs_gps_sats_mean", self._gps_sat_sum / float(self._gps_lines))
            self.on_diag("gs_gps_sats_min", self._gps_sat_min)
            self.on_diag("gs_gps_lines", self._gps_lines)
            return

        m = _RE_GS_GPS_SRCH.search(line)
        if m:
            sats = int(m.group(1))
            self._gps_lines += 1
            self._gps_sat_sum += sats
            self._gps_sat_min = sats if self._gps_sat_min is None else min(self._gps_sat_min, sats)
            self.on_diag("gs_gps_fix_ratio", self._gps_fix_n / float(self._gps_lines))
            self.on_diag("gs_gps_sats_mean", self._gps_sat_sum / float(self._gps_lines))
            self.on_diag("gs_gps_sats_min", self._gps_sat_min)
            self.on_diag("gs_gps_lines", self._gps_lines)
            return

        m = _RE_LORA_INIT.search(line)
        if m:
            self.on_diag("lora_init_433", m.group(1))
            self.on_diag("lora_init_920", m.group(2))
            return

        m = _RE_STATS_HDR.search(line)
        if m:
            self._stats_ctx = m.group(1)
            return
        if self._stats_ctx:
            m = _RE_STATS_PKTOK.search(line)
            if m:
                self.on_diag(f"stats_{self._stats_ctx}_pkt_ok", int(m.group(1)))
                self.on_diag(f"stats_{self._stats_ctx}_crc_err", int(m.group(2)))
                return
            m = _RE_STATS_RATE.search(line)
            if m:
                self.on_diag(f"stats_{self._stats_ctx}_rate_pkt_s", float(m.group(1)))
                return
            m = _RE_STATS_RSSI.search(line)
            if m:
                self.on_diag(f"stats_{self._stats_ctx}_rssi_last", int(m.group(1)))
                self.on_diag(f"stats_{self._stats_ctx}_rssi_min", int(m.group(2)))
                self.on_diag(f"stats_{self._stats_ctx}_rssi_max", int(m.group(3)))
                self.on_diag(f"stats_{self._stats_ctx}_rssi_avg", int(m.group(4)))
                return
            m = _RE_STATS_SNR.search(line)
            if m:
                self.on_diag(f"stats_{self._stats_ctx}_snr_last", int(m.group(1)))
                self.on_diag(f"stats_{self._stats_ctx}_snr_min", int(m.group(2)))
                self.on_diag(f"stats_{self._stats_ctx}_snr_max", int(m.group(3)))
                self.on_diag(f"stats_{self._stats_ctx}_snr_avg", int(m.group(4)))
                return


# ===========================================================================
#  離線 GSLOGnnn.CSV 讀取（見 gs_log.c GsLog_CsvHeader/FormatCsvRow）
# ===========================================================================
def load_gs_log_csv(path: str) -> list:
    """回傳事件 list：{t(秒,已展開跨午夜), link, rssi, snr, seq, gs_sats, gs_fix}。
    rx_utc_ms 是 GPS 紀律牆鐘「當日毫秒」，跨午夜會歸零，此處偵測大幅倒退並展開。"""
    events = []
    day_offset_ms = 0
    prev_raw_ms = None
    with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                raw_ms = int(row["rx_utc_ms"])
                link = int(row["link_mhz"])
                rssi = int(row["rssi_dbm"])
                snr = int(row["snr_cb"])
                seq = int(row["seq"])
                gs_sats = int(row["gs_sats"])
                gs_fix = int(row["gs_fix"])
            except (KeyError, ValueError):
                continue
            if raw_ms == 0:
                continue  # 尚未有 GPS 紀律錨點，時間戳不可信，跳過時序相關統計
            if prev_raw_ms is not None and raw_ms < prev_raw_ms - 43200000:
                day_offset_ms += 86400000
            prev_raw_ms = raw_ms
            t = (raw_ms + day_offset_ms) / 1000.0
            events.append({"t": t, "link": link, "rssi": rssi, "snr": snr, "seq": seq,
                           "gs_sats": gs_sats, "gs_fix": gs_fix})
    return events


# ===========================================================================
#  轉播飛行數據：圖表 + GPS 地圖（讀「地面站實際收到」的內容，與 flash_analyzer.py
#  讀「火箭自己 Flash Ring 存的完整內容」互補——雙鏈路丟包會讓兩者有落差，這正是
#  地面站備援想觀察的東西）。與 load_gs_log_csv() 不同：後者只留鏈路品質欄位供
#  seq/gap 統計；這裡留飛行物理量 + 雙板 peer 摘要 + 兩端 GPS 全量。
# ===========================================================================
def load_relay_flight_csv(csv_path: str) -> list:
    """解析地面站 Flash/SD CSV（gs_log.c GsLog_CsvHeader 格式）為轉播飛行事件 list。
    每筆同時附上兩種時間基準，不在此處依時間篩掉任何列——是否需要有效 UTC 由呼叫端
    （畫圖表 vs 畫地圖）各自決定：
      t_utc      —— 地面站 GPS 校時後的當日 UTC 秒數；地面站當下還沒 GPS lock 時該筆是 None
                     （rx_utc_ms==0，見 gs_timesync.h GsTimeSync_GroundUtcMs：未拿到 UTC 錨點
                     前一律回傳 0）。
      t_fallback —— rkt_tick_ms + offset_ms（火箭開機相對時間，經地面本機 tick 偏移量 EMA
                     修正），兩欄皆不依賴地面站 GPS，任何一筆都有效；供整段 session 都沒
                     GPS lock 時，圖表時間軸的備援基準（見 generate_relay_flight_chart）。
    地圖（generate_relay_gps_map）只看 gps_fix/gs_fix，不需要時間，因此完全不受兩者影響。"""
    events = []
    day_offset_ms = 0
    prev_raw_ms = None
    with open(csv_path, "r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                raw_ms = int(row["rx_utc_ms"])
                rkt_tick_ms = int(row["rkt_tick_ms"])
                offset_ms = int(row["offset_ms"])
                t_utc = None
                if raw_ms != 0:
                    if prev_raw_ms is not None and raw_ms < prev_raw_ms - 43200000:
                        day_offset_ms += 86400000
                    prev_raw_ms = raw_ms
                    t_utc = (raw_ms + day_offset_ms) / 1000.0

                events.append({
                    "t_utc": t_utc,
                    "t_fallback": (rkt_tick_ms + offset_ms) / 1000.0,
                    "link_mhz": int(row["link_mhz"]),
                    "rssi_dbm": int(row["rssi_dbm"]),
                    "snr_cb": int(row["snr_cb"]),
                    "seq": int(row["seq"]),
                    "fsm_state": int(row["fsm_state"]),
                    "ekf_alt_m": int(row["ekf_alt_cm"]) / 100.0,
                    "ekf_vel_ms": int(row["ekf_vel_cms"]) / 100.0,
                    "baro_alt_m": int(row["baro_alt_cm"]) / 100.0,
                    "vf_alt_m": int(row["vf_alt_cm"]) / 100.0,
                    "vf_vel_ms": int(row["vf_vel_cms"]) / 100.0,
                    "gps_lat": int(row["gps_lat_1e6"]) / 1e6,
                    "gps_lon": int(row["gps_lon_1e6"]) / 1e6,
                    "gps_alt_m": int(row["gps_alt_m"]),
                    "gps_sats": int(row["gps_sats"]),
                    "gps_fix": int(row["gps_fix"]),
                    "bat_mv": int(row["bat_mv"]),
                    "peer_fsm": int(row["peer_fsm"]),
                    # ★2026-07-30：下鏈的對端摘要只剩 VF（EKF 高度/速度、丟包率已從封包移除）
                    "peer_vf_h_m": int(row["peer_vf_h_cm"]) / 100.0,
                    "peer_vf_v_ms": int(row["peer_vf_v_cms"]) / 100.0,
                    # ★飛行滾動極值：火箭端全速率追蹤、每包重複攜帶（見 telemetry.h）。
                    # 下鏈只有 ~2Hz，抓不到真正的頂點與峰值 G，這三個才是可信數字。
                    # 用 .get 容忍舊 CSV——直接 row["max_alt_m"] 會在舊檔上 KeyError，
                    # 而外層 except 是 continue，等於「整份舊紀錄一列都讀不進來」。
                    "max_alt_m": int(row.get("max_alt_m") or 0),
                    "max_vel_ms": int(row.get("max_vel_ms") or 0),
                    "max_acc_g": int(row.get("max_acc_cg") or 0) / 100.0,
                    # 開傘高度；-32768 = 未開傘哨兵（telemetry.h TELEM_DEPLOY_ALT_NA）
                    "drogue_alt_m": int(row.get("drogue_alt_m") or -32768),
                    "main_alt_m": int(row.get("main_alt_m") or -32768),
                    "gs_lat": int(row["gs_lat_1e6"]) / 1e6,
                    "gs_lon": int(row["gs_lon_1e6"]) / 1e6,
                    "gs_alt_m": int(row["gs_alt_m"]),
                    "gs_fix": int(row["gs_fix"]),
                })
            except (KeyError, ValueError):
                continue
    return events


def generate_relay_flight_chart(csv_path: str, output_dir="."):
    """2x2 轉播飛行分析圖：高度（主 EKF/Baro + 副航電 peer）、垂直速度（主/副）、
    雙鏈路 RSSI/SNR（依實際收到的時刻描點，看得出丟包造成的資料空隙）、電池電壓。"""
    if not HAS_MATPLOTLIB:
        print("[RELAY] 未安裝 matplotlib，略過轉播飛行圖表")
        return None
    all_events = load_relay_flight_csv(csv_path)
    if not all_events:
        print("[RELAY] CSV 內未解析到有效轉播飛行數據，略過圖表")
        return None

    # 只要 session 中有任何一筆拿到過地面站 GPS UTC 錨點，就整段用 t_utc（丟掉沒錨點前的
    # 少數幾筆即可，不影響絕對牆鐘時間軸的意義）；若整段 session 地面站 GPS 全程沒 lock
    # （室內/長凳測試常見），t_utc 全部是 None，改用不依賴地面 GPS 的 t_fallback（火箭
    # tick+鏈路偏移量重建的相對時間），圖表照樣畫得出來，只是不能拿來對絕對時刻。
    use_fallback_time = not any(e["t_utc"] is not None for e in all_events)
    if use_fallback_time:
        print("[RELAY] 本次匯出全程無地面站 GPS UTC 校時錨點，圖表時間軸改用「火箭 tick + 鏈路偏移量」重建的相對時間")
        events = sorted(all_events, key=lambda e: e["t_fallback"])
        for e in events:
            e["t"] = e["t_fallback"]
    else:
        events = [e for e in all_events if e["t_utc"] is not None]
        for e in events:
            e["t"] = e["t_utc"]
        events.sort(key=lambda r: r["t"])

    t0 = events[0]["t"]
    times = [e["t"] - t0 for e in events]

    fig, axs = plt.subplots(2, 2, figsize=(16, 10), dpi=140, facecolor="#101010")
    title = "RocketCom Ground-Relayed Flight Data"
    if use_fallback_time:
        title += "  (Relative Time — No GPS UTC Lock)"
    fig.suptitle(title, fontsize=16, fontweight="bold", color="#00e676")
    for ax in axs.flat:
        ax.set_facecolor("#161616")
        for spine in ax.spines.values():
            spine.set_color("#444")
        ax.tick_params(colors="#ccc")
        ax.xaxis.label.set_color("#ccc"); ax.yaxis.label.set_color("#ccc")

    ax1 = axs[0, 0]
    ax1.plot(times, [e["ekf_alt_m"] for e in events], label="Primary EKF Alt", color="#00e676", linewidth=1.8)
    ax1.plot(times, [e["baro_alt_m"] for e in events], label="Primary Baro Alt", color="#ff9f43",
             linestyle="--", linewidth=1.1, alpha=0.8)
    ax1.plot(times, [e["peer_vf_h_m"] for e in events], label="Backup VF Alt (peer)", color="#38bdf8",
             linestyle=":", linewidth=1.3)
    # 火箭端全速率追蹤的最大高度（階梯線）：下鏈 ~2Hz 的取樣點永遠低估真正頂點，
    # 這條線的最終高度才是可信的 apogee。與上面的取樣曲線同屏對照。
    if any(e["max_alt_m"] for e in events):
        ax1.plot(times, [e["max_alt_m"] for e in events], label="MAX Alt (rocket-tracked)",
                 color="#e879f9", linewidth=1.4, drawstyle="steps-post", alpha=0.9)
    # 實際開傘高度（火箭端在開傘那一刻就地鎖存，非事後從稀疏取樣反推）
    for key, colour, lbl in (("drogue_alt_m", "#f43f5e", "Drogue deploy"),
                             ("main_alt_m", "#22d3ee", "Main deploy")):
        vals = [e[key] for e in events if e[key] != -32768]
        if vals:
            ax1.axhline(vals[-1], color=colour, linewidth=1.0, linestyle="-.",
                        alpha=0.85, label=f"{lbl} @ {vals[-1]}m")
    ax1.set_title("Altitude (as relayed to ground)", color="#00e676")
    ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Altitude (m)")
    ax1.grid(True, linestyle=":", alpha=0.25); ax1.legend(fontsize=8, facecolor="#161616", labelcolor="#ddd")

    ax2 = axs[0, 1]
    ax2.plot(times, [e["ekf_vel_ms"] for e in events], label="Primary Vz", color="#38ef7d", linewidth=1.6)
    ax2.plot(times, [e["peer_vf_v_ms"] for e in events], label="Backup VF Vz (peer)", color="#fbbf24",
             linestyle=":", linewidth=1.3)
    if any(e["max_vel_ms"] for e in events):
        ax2.plot(times, [e["max_vel_ms"] for e in events], label="MAX Vz (rocket-tracked)",
                 color="#e879f9", linewidth=1.4, drawstyle="steps-post", alpha=0.9)
    ax2.axhline(0, color="#888", linewidth=0.7, linestyle=":")
    ax2.set_title("Vertical Velocity", color="#38ef7d")
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Velocity (m/s)")
    ax2.grid(True, linestyle=":", alpha=0.25); ax2.legend(fontsize=8, facecolor="#161616", labelcolor="#ddd")

    ax3 = axs[1, 0]
    ax3t = ax3.twinx(); ax3t.tick_params(colors="#ccc")
    l433 = [(t, e["rssi_dbm"]) for t, e in zip(times, events) if e["link_mhz"] == 433 and e["rssi_dbm"] != -32768]
    l920 = [(t, e["rssi_dbm"]) for t, e in zip(times, events) if e["link_mhz"] == 920]
    s920 = [(t, e["snr_cb"] / 4.0) for t, e in zip(times, events) if e["link_mhz"] == 920 and e["snr_cb"] != -32768]
    handles = []
    if l433:
        handles += ax3.plot(*zip(*l433), '.', label="433 RSSI", color="#ff7675", markersize=3, alpha=0.6)
    if l920:
        handles += ax3.plot(*zip(*l920), '.', label="920 RSSI", color="#74b9ff", markersize=3, alpha=0.6)
    if s920:
        handles += ax3t.plot(*zip(*s920), '.', label="920 SNR", color="#ffeaa7", markersize=3, alpha=0.5)
    ax3.set_title("Link Quality (as received)", color="#74b9ff")
    ax3.set_xlabel("Time (s)"); ax3.set_ylabel("RSSI (dBm)"); ax3t.set_ylabel("SNR (dB)", color="#ccc")
    ax3.grid(True, linestyle=":", alpha=0.25)
    if handles:
        ax3.legend(handles, [h.get_label() for h in handles], fontsize=8, facecolor="#161616", labelcolor="#ddd")

    ax4 = axs[1, 1]
    ax4.plot(times, [e["bat_mv"] / 1000.0 for e in events], color="#f1c40f", linewidth=1.6, label="Primary Battery (V)")
    ax4.axhline(7.0, color="#e74c3c", linewidth=0.8, linestyle=":", alpha=0.7)
    ax4.set_title("Battery Voltage (relayed)", color="#f1c40f")
    ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Voltage (V)")
    ax4.grid(True, linestyle=":", alpha=0.25); ax4.legend(fontsize=8, facecolor="#161616", labelcolor="#ddd")

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    chart_path = os.path.join(output_dir, f"ground_relay_analysis_{ts_str}.png")
    plt.savefig(chart_path, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"[RELAY] 已生成轉播飛行分析圖: {chart_path}")
    return chart_path


def generate_relay_gps_map(csv_path: str, output_dir="."):
    """互動式 GPS 地圖（folium）：火箭轉播的 GPS 航跡 + 地面站自身定位（固定點）。
    無 folium 或雙方皆無有效定位點時安全略過（回傳 None）。只看 gps_fix/gs_fix，不碰
    load_relay_flight_csv() 回傳的 t_utc/t_fallback——地面站自己有沒有 GPS UTC 校時錨點
    跟「這筆封包裡的火箭/地面站定位點準不準」無關，不該互相拖累。"""
    try:
        import folium
    except ImportError:
        print("[RELAY] 未安裝 folium，略過轉播 GPS 地圖（pip install folium）")
        return None

    events = load_relay_flight_csv(csv_path)
    rocket_pts = [e for e in events if e["gps_fix"] == 1 and abs(e["gps_lat"]) > 0.01]
    gs_pts = [e for e in events if e["gs_fix"] == 1 and abs(e["gs_lat"]) > 0.01]
    if not rocket_pts and not gs_pts:
        print("[RELAY] CSV 內無有效 GPS 定位點（火箭與地面站皆無），略過地圖")
        return None

    center = [rocket_pts[0]["gps_lat"], rocket_pts[0]["gps_lon"]] if rocket_pts \
        else [gs_pts[0]["gs_lat"], gs_pts[0]["gs_lon"]]
    fmap = folium.Map(location=center, zoom_start=15, tiles="CartoDB dark_matter")
    folium.TileLayer("OpenStreetMap", name="OpenStreetMap").add_to(fmap)

    if len(rocket_pts) >= 2:
        pts = [(e["gps_lat"], e["gps_lon"]) for e in rocket_pts]
        folium.PolyLine(pts, color="#00e676", weight=4, opacity=0.85,
                        tooltip="Rocket GPS track (as relayed to ground)").add_to(fmap)
        folium.Marker(pts[0], tooltip="First relayed fix",
                      icon=folium.Icon(color="green", icon="rocket", prefix="fa")).add_to(fmap)
        folium.Marker(pts[-1], tooltip="Last relayed fix",
                      icon=folium.Icon(color="red", icon="flag-checkered", prefix="fa")).add_to(fmap)

    if gs_pts:
        gs_lat = sum(e["gs_lat"] for e in gs_pts) / len(gs_pts)
        gs_lon = sum(e["gs_lon"] for e in gs_pts) / len(gs_pts)
        folium.Marker([gs_lat, gs_lon], tooltip="Ground Station",
                      icon=folium.Icon(color="blue", icon="wifi", prefix="fa")).add_to(fmap)

    folium.LayerControl().add_to(fmap)
    all_lats = [e["gps_lat"] for e in rocket_pts] + [e["gs_lat"] for e in gs_pts]
    all_lons = [e["gps_lon"] for e in rocket_pts] + [e["gs_lon"] for e in gs_pts]
    if len(all_lats) >= 2:
        fmap.fit_bounds([[min(all_lats), min(all_lons)], [max(all_lats), max(all_lons)]])

    ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    map_path = os.path.join(output_dir, f"ground_relay_map_{ts_str}.html")
    fmap.save(map_path)
    print(f"[RELAY] 已生成轉播 GPS 地圖: {map_path}")
    return map_path


# ===========================================================================
#  分析引擎
# ===========================================================================
class GsAnalyzerEngine:
    def __init__(self):
        self.pkt_events = []       # {t, link, rssi, snr, seq} — CRC 正確
        self.crc_bad_events = []   # {t, link, seq} — CRC 錯誤，但仍證明「有收到東西」
        self.gs_stat_snapshots = []  # {t, ...}
        self.diag = {}
        self.lock = threading.Lock()

    def add_pkt(self, t, link, rssi, snr, seq):
        with self.lock:
            self.pkt_events.append({"t": t, "link": link, "rssi": rssi, "snr": snr, "seq": seq})

    def add_pkt_bad(self, t, link, seq=0):
        with self.lock:
            self.crc_bad_events.append({"t": t, "link": link, "seq": seq})

    def add_gs_stat(self, t, d):
        with self.lock:
            d = dict(d)
            d["t"] = t
            self.gs_stat_snapshots.append(d)

    def update_diag(self, key, value):
        with self.lock:
            self.diag[key] = value

    def snapshot(self):
        with self.lock:
            return (list(self.pkt_events), list(self.crc_bad_events),
                    list(self.gs_stat_snapshots), dict(self.diag))

    def load_offline_csv_events(self, events: list):
        with self.lock:
            self.pkt_events.extend(events)
            for e in events:
                if e.get("gs_fix") is not None:
                    pass  # gs GPS 由 analyze() 直接掃 pkt_events 算（CSV 模式每列都帶）

    # -----------------------------------------------------------------
    def analyze(self, csv_mode=False, synth_time=False) -> dict:
        # synth_time=True（--file 純文字 log 模式）：時間戳是按行號合成的，不是真實到達
        # 時間。所有以「時間間隔」為量測值的項目（gap / 紀錄管線停頓）在這個模式下沒有
        # 物理意義，因此照算照印、但一律不判 WARN/FAIL，避免假 PASS 被當成鏈路健康。
        pkt_events, crc_bad_events, gs_stats, diag = self.snapshot()
        n = len(pkt_events)
        if n < 3:
            return {"error": "樣本數不足（需至少 3 筆封包事件）", "sample_count": n}

        t0 = pkt_events[0]["t"]
        tN = pkt_events[-1]["t"]
        span_s = max(tN - t0, 1e-6)

        by_link = {LINK_433: [e for e in pkt_events if e["link"] == LINK_433],
                   LINK_920: [e for e in pkt_events if e["link"] == LINK_920]}
        by_link_bad = {LINK_433: [e for e in crc_bad_events if e["link"] == LINK_433],
                       LINK_920: [e for e in crc_bad_events if e["link"] == LINK_920]}

        checks = []

        def add_check(category, name, val, limit, unit, cond, is_fail, is_warn, detail):
            status = "FAIL" if is_fail else ("WARN" if is_warn else "PASS")
            checks.append({"category": category, "name": name, "val": float(val), "limit": float(limit),
                            "unit": unit, "status": status, "detail": detail})

        link_summaries = {}
        for link, name in ((LINK_433, "433 Link (E22)"), (LINK_920, "920 Link (E80/LR1121)")):
            evs = by_link[link]
            bad_evs = by_link_bad[link]
            ok_count = len(evs)
            bad_count = len(bad_evs)

            # --- 有效通訊頻率（CRC 正確才算數）：優先用 [STATS] 韌體自算速率(pkt_ok/elapsed，
            # 視窗較長較穩，見 lora_calc.h lora_stats_rate_x10)，其次用 [GS_STAT] 累計 ok 計數
            # 差分，最後才退回本次擷取樣本數/時長(擷取起訖可能切在封包中間，較不準)。
            key_prefix = "stats_E22-433" if link == LINK_433 else "stats_E80-920"
            ok_key = "ok433" if link == LINK_433 else "ok920"
            crc_key = "crc433" if link == LINK_433 else "crc920"
            rate_hz = diag.get(f"{key_prefix}_rate_pkt_s")
            rate_src = "[STATS] 韌體累計速率 (pkt_ok/elapsed)"
            if rate_hz is None and len(gs_stats) >= 2:
                d_ok = gs_stats[-1][ok_key] - gs_stats[0][ok_key]
                d_t = gs_stats[-1]["t"] - gs_stats[0]["t"]
                rate_hz = (d_ok / d_t) if d_t > 0 else 0.0
                rate_src = "[GS_STAT] 累計 ok 計數差分"
            if rate_hz is None:
                rate_hz = (ok_count / span_s) if span_s > 0 else 0.0
                rate_src = "本次擷取樣本(較不準)"

            # ★這一項現在是本工具對該鏈路的主要 GO/NO-GO：不問「漏了幾包」（那需要猜
            # 火箭端發了幾包），只問「有效封包進來的頻率夠不夠用」。門檻取韌體排定速率
            # 的比例，與下面總頻率共用同一組比例。433 的排定速率含估計成分（見
            # LORA433_AIRTIME_S），所以貼著門檻時要一併看 CRC 錯誤率再下結論。
            nominal = NOMINAL_RATE_HZ[link]
            add_check(name, "有效通訊頻率 (CRC 正確)", rate_hz,
                      nominal * SPEC_LIMITS["total_rate_warn_ratio"], "pkt/s", ">=",
                      ok_count == 0 or rate_hz < nominal * SPEC_LIMITS["total_rate_fail_ratio"],
                      rate_hz < nominal * SPEC_LIMITS["total_rate_warn_ratio"],
                      f"{rate_src}；本次擷取到 {ok_count} 筆有效封包 / {bad_count} 筆 CRC 錯誤"
                      f"（排定速率≈{nominal:.2f}Hz，WARN<{SPEC_LIMITS['total_rate_warn_ratio']*100:.0f}%、"
                      f"FAIL<{SPEC_LIMITS['total_rate_fail_ratio']*100:.0f}%）")

            # --- 總通訊頻率（含 CRC 無效）：只要有觸發同步/CRC 檢查就算「有通訊」，不論解碼
            # 是否成功。跟韌體設計排程(見 NOMINAL_RATE_HZ：920≈10Hz，433≈空中時間上限扣上行窗
            # 後≈1.55Hz，其中 433 這個空中時間本身是估計值、非實測，見 LORA433_AIRTIME_S 註解)
            # 比較，能分辨「RF 前端根本沒收到東西」(總頻率遠低於排定值) 跟「有收到但解不出來」
            # (總頻率接近排定值、但有效頻率偏低，即 CRC 錯誤率高) 這兩種完全不同的問題。
            if len(gs_stats) >= 2:
                d_total = (gs_stats[-1][ok_key] + gs_stats[-1][crc_key]) - (gs_stats[0][ok_key] + gs_stats[0][crc_key])
                d_t = gs_stats[-1]["t"] - gs_stats[0]["t"]
                total_rate_hz = (d_total / d_t) if d_t > 0 else 0.0
                total_src = "[GS_STAT] 累計(ok+crc)計數差分"
            elif len(gs_stats) == 1:
                total_rate_hz = (gs_stats[0][ok_key] + gs_stats[0][crc_key]) / span_s if span_s > 0 else 0.0
                total_src = "[GS_STAT] 單筆累計(ok+crc)/擷取時長"
            else:
                total_rate_hz = (ok_count + bad_count) / span_s if span_s > 0 else 0.0
                total_src = "本次擷取樣本(較不準)"
            add_check(name, "總通訊頻率 (含 CRC 無效)", total_rate_hz, nominal, "pkt/s",
                      ">=", total_rate_hz < nominal * SPEC_LIMITS["total_rate_fail_ratio"],
                      total_rate_hz < nominal * SPEC_LIMITS["total_rate_warn_ratio"],
                      f"{total_src}；韌體排定速率≈{nominal:.2f}Hz（920 見 main.c "
                      f"LORA_TELEM_PERIOD_MS；433 為空中時間物理上限扣掉上行接收窗"
                      f"（每 {UPLINK_LISTEN_PERIOD_MS:.0f}ms 靜默 {UPLINK_LISTEN_HOLD_MS:.0f}ms）"
                      f"後的值，★空中時間本身是估計值(580ms)非實測，見 "
                      f"LORA433_AIRTIME_S 註解）。遠低於排定值代表 RF 前端可能根本沒收到訊號，"
                      f"而非單純解碼品質問題——但若持續卡在排定值邊緣，也可能是這個估計值"
                      f"本身偏樂觀，非真的異常")

            # --- CRC 錯誤率：優先用 [GS_STAT] 累計計數（跨整個連線期間，較不受擷取窗切點影響）
            if len(gs_stats) >= 1:
                cum_ok = gs_stats[-1][ok_key]
                cum_crc = gs_stats[-1][crc_key]
                denom = cum_ok + cum_crc
                crc_ratio = (cum_crc / float(denom)) if denom > 0 else 0.0
                crc_src = f"[GS_STAT] 累計 ok={cum_ok} crc={cum_crc}"
            else:
                denom = ok_count + bad_count
                crc_ratio = (bad_count / float(denom)) if denom > 0 else 0.0
                crc_src = f"本次擷取 ok={ok_count} crc={bad_count}"
            add_check(name, "CRC 錯誤率", crc_ratio * 100.0, SPEC_LIMITS["crc_err_max_ratio"] * 100.0, "%",
                      "<=", crc_ratio > SPEC_LIMITS["crc_err_max_ratio"] * 1.5,
                      crc_ratio > SPEC_LIMITS["crc_err_max_ratio"], crc_src)

            # --- Resync 比例（雜訊指標，僅 [GS_STAT] 有）
            if len(gs_stats) >= 1:
                rs_key = "rsync433" if link == LINK_433 else "rsync920"
                rsync = gs_stats[-1][rs_key]
                denom2 = gs_stats[-1]["ok433" if link == LINK_433 else "ok920"] + \
                    gs_stats[-1]["crc433" if link == LINK_433 else "crc920"] + rsync
                resync_ratio = (rsync / float(denom2)) if denom2 > 0 else 0.0
                add_check(name, "Resync 比例 (雜訊指標)", resync_ratio * 100.0,
                          SPEC_LIMITS["resync_max_ratio"] * 100.0, "%",
                          "<=", resync_ratio > SPEC_LIMITS["resync_max_ratio"] * 1.5,
                          resync_ratio > SPEC_LIMITS["resync_max_ratio"],
                          f"rsync={rsync}（同步位元組頻繁對不上，常見於空中速率不符/雜訊）")
            else:
                add_check(name, "Resync 比例 (雜訊指標)", 0.0, SPEC_LIMITS["resync_max_ratio"] * 100.0, "%",
                          "<=", False, False, "未見到 [GS_STAT] 行（純 CSV 離線分析無此資料）")

            # --- 封包間隔（gap）：取代舊的「到達率丟失」。只看實際收到的相鄰兩包差多久，
            # 不反推火箭端發了幾包。CRC 錯誤的封包仍證明「這個時刻有訊號進來」，所以併入
            # any_rx 一起看間隔——這樣量到的是「RF 靜默多久」，跟 CRC 錯誤率(解碼品質)分開。
            # ★433 的間隔天生較長且不規則：上行接收窗每 3000ms 有 800ms 完全不發，加上單包
            # 空中時間本身就佔數百 ms，所以 433 的門檻是照它自己的名目速率放寬的。
            # ★間隔變長的成因不只 RF：航電端卡住/排隊延遲(SPI3 mutex 跟 Flash 記錄搶用、
            # 任務被高優先權搶佔)也會讓實際發射變稀疏。單靠地面站封包無法分辨，需對照
            # 航電板自己的 [LORA_TX_LOG](main.c，ok/try 計數，每 25 槽印一次)。
            gl = gap_limits(nominal)
            any_rx_times = sorted(e["t"] for e in (evs + bad_evs))
            lg = gap_stats(any_rx_times, gl["max_warn_ms"], gl["max_fail_ms"])
            time_note = ("SD 卡 rx_utc_ms（GPS 紀律牆鐘）" if csv_mode else
                         "★行號合成時間，非真實到達時間——本項僅供參考、不列入判定"
                         if synth_time else
                         "console 到達時間（含序列埠/列印延遲，僅供參考）")
            if lg["count"] >= 1:
                add_check(name, "平均封包間隔 (Gap)", lg["mean_ms"], gl["mean_warn_ms"], "ms", "<=",
                          (not synth_time) and lg["mean_ms"] > gl["mean_fail_ms"],
                          (not synth_time) and lg["mean_ms"] > gl["mean_warn_ms"],
                          f"{lg['count']} 個間隔樣本（CRC 錯誤也算有到達）；名目間隔"
                          f"≈{gl['nominal_gap_ms']:.0f}ms（=1/{nominal:.2f}Hz），"
                          f"WARN>{gl['mean_warn_ms']:.0f}ms、FAIL>{gl['mean_fail_ms']:.0f}ms。"
                          f"時間基準：{time_note}")
                add_check(name, "最長封包空窗 (Max Gap)", lg["max_ms"], gl["max_warn_ms"], "ms", "<=",
                          (not synth_time) and lg["max_ms"] > gl["max_fail_ms"],
                          (not synth_time) and lg["max_ms"] > gl["max_warn_ms"],
                          f"這段擷取內此鏈路最久一次連續收不到訊號（FAIL>{gl['max_fail_ms']:.0f}ms）；"
                          f"共 {lg['stall_count']} 次落在警告值以上"
                          f"（占 {lg['stall_ratio']*100:.1f}%）。單鏈路空窗未必等於失聯，"
                          f"要看下面雙鏈路合併後的空窗。時間基準：{time_note}")
            else:
                add_check(name, "平均封包間隔 (Gap)", 0.0, gl["mean_warn_ms"], "ms",
                          "<=", False, False, f"樣本不足（{len(any_rx_times)} 筆）")
                add_check(name, "最長封包空窗 (Max Gap)", 0.0, gl["max_warn_ms"], "ms",
                          "<=", False, False, f"樣本不足（{len(any_rx_times)} 筆）")

            link_summaries[link] = {"ok_count": ok_count, "bad_count": bad_count,
                                     "rate_hz": rate_hz, "total_rate_hz": total_rate_hz}

            # --- RSSI：兩個鏈路都可能有。920 由 LR1121 晶片直接給；
            #     433 在 E22 開啟 REG3 bit7（每包附加 RSSI 位元組）時才有，沒有就全是哨兵值。
            #     舊版整段包在 `if link == LINK_920:` 內，433 即使有值也被丟掉。
            rssi_vals = [e["rssi"] for e in evs if e["rssi"] != RSSI_SNR_NA]
            if rssi_vals:
                rssi_avg = diag.get(f"{key_prefix}_rssi_avg")
                if rssi_avg is None:
                    rssi_avg = StatMetrics(rssi_vals).mean
                    rssi_src = "本次擷取樣本平均"
                else:
                    rssi_src = "[STATS] 韌體累計"
                add_check(name, "RSSI 平均", rssi_avg, SPEC_LIMITS["rssi_warn_dbm"], "dBm",
                          ">=", rssi_avg < SPEC_LIMITS["rssi_fail_dbm"], rssi_avg < SPEC_LIMITS["rssi_warn_dbm"],
                          f"{rssi_src}（{len(rssi_vals)} 筆樣本）")

            # --- SNR：920 專屬（E22 不提供）
            if link == LINK_920:
                snr_raw_vals = [e["snr"] for e in evs if e["snr"] != RSSI_SNR_NA]
                snr_avg_raw = diag.get(f"{key_prefix}_snr_avg")
                if snr_avg_raw is None:
                    snr_avg_raw = StatMetrics(snr_raw_vals).mean if snr_raw_vals else 0.0
                    snr_src = "本次擷取樣本平均"
                else:
                    snr_src = "[STATS] 韌體累計"
                snr_avg_db = snr_avg_raw / 4.0   # snr_cb 為 SX126x 原始值，單位 0.25dB
                add_check(name, "SNR 平均", snr_avg_db, SPEC_LIMITS["snr_warn_db"], "dB",
                          ">=", snr_avg_db < SPEC_LIMITS["snr_fail_db"], snr_avg_db < SPEC_LIMITS["snr_warn_db"],
                          f"{snr_src}（原始值/4 換算 dB，{len(snr_raw_vals)} 筆樣本）")

        # --- 合併封包間隔（雙鏈路去重後、CRC 好壞都算到達）：★這是本工具最關鍵的 GO/NO-GO。
        # 「兩條鏈路同時都沒東西進來」持續多久，才是地面站真正失聯的時間——即使某一鏈路整個
        # 掛掉，只要另一鏈路頂住，這裡就該維持在名目 tick 間隔附近。用 920 的名目速率(每個
        # 100ms tick 都發)當基準，跟 433 自己的降速排程無關。
        # ★不再算 Union Loss 百分比：那需要先算「這段時間本來共有幾個時槽該收到」，而該數字
        # 得靠 seq/時間反推 tick，跨鏈路到達時間差一倒置就會炸掉分母（歷次 report 曾反推出
        # 270~370Hz 這種遠超硬體上限的隱含頻率）。空窗時間是直接量到的，沒有這個問題。
        any_rx_all = sorted(pkt_events + crc_bad_events, key=lambda e: e["t"])
        merged_any = merge_dedupe_events(any_rx_all)
        cgl = gap_limits(NOMINAL_RATE_HZ[LINK_920])
        cg = gap_stats([e["t"] for e in merged_any], cgl["max_warn_ms"], cgl["max_fail_ms"])
        if cg["count"] >= 1:
            add_check("封包完整性（雙鏈路合併）", "合併平均封包間隔", cg["mean_ms"], cgl["mean_warn_ms"], "ms", "<=",
                      (not synth_time) and cg["mean_ms"] > cgl["mean_fail_ms"],
                      (not synth_time) and cg["mean_ms"] > cgl["mean_warn_ms"],
                      f"雙鏈路去重後共 {len(merged_any)} 筆、{cg['count']} 個間隔；名目 tick 間隔"
                      f"≈{cgl['nominal_gap_ms']:.0f}ms，WARN>{cgl['mean_warn_ms']:.0f}ms、"
                      f"FAIL>{cgl['mean_fail_ms']:.0f}ms（CRC 錯誤也算有到達）"
                      f"{'。★行號合成時間，不列入判定' if synth_time else ''}")
            add_check("封包完整性（雙鏈路合併）", "合併最長空窗 (Union Max Gap)", cg["max_ms"],
                      cgl["max_warn_ms"], "ms", "<=",
                      (not synth_time) and cg["max_ms"] > cgl["max_fail_ms"],
                      (not synth_time) and cg["max_ms"] > cgl["max_warn_ms"],
                      f"★兩條鏈路同時靜默最久 {cg['max_ms']:.0f}ms（FAIL>{cgl['max_fail_ms']:.0f}ms）；"
                      f"共 {cg['stall_count']} 次超過警告值（占 {cg['stall_ratio']*100:.1f}%）。"
                      f"這是地面站真正「看不到火箭」的時間，飛行中最該盯的就是這一項")
        else:
            add_check("封包完整性（雙鏈路合併）", "合併最長空窗 (Union Max Gap)", 0.0,
                      cgl["max_warn_ms"], "ms", "<=", False, False,
                      f"樣本不足（去重後 {len(merged_any)} 筆）")

        # 雙鏈路重複「成功解碼」比例：只算 CRC 正確的封包，才是真正驗證到「備援真的幫上忙」
        # （CRC 壞掉的重複收到並沒有實質備援價值，資料仍然不可用）。
        merged_ok = merge_dedupe_events(pkt_events)
        n_433, n_920 = len(by_link[LINK_433]), len(by_link[LINK_920])
        n_dup = n_433 + n_920 - len(merged_ok)
        overlap_ratio = (n_dup / float(len(merged_ok))) if merged_ok else 0.0
        add_check("封包完整性（雙鏈路合併）", "雙鏈路重複接收比例", overlap_ratio * 100.0, 0.0, "%",
                  "==", False, False,
                  f"{n_dup} 筆同時被兩鏈路成功解碼（433={n_433}, 920={n_920}, 去重後={len(merged_ok)}）"
                  f"——比例越高代表備援餘裕越大")

        # --- 紀錄管線：連續封包(不分鏈路，依到達/落地時間排序)間隔統計
        all_times = sorted(e["t"] for e in pkt_events)
        gaps = gap_stats(all_times, SPEC_LIMITS["pipeline_stall_warn_ms"], SPEC_LIMITS["pipeline_stall_fail_ms"])
        time_basis = ("SD 卡 rx_utc_ms（GPS 紀律牆鐘，最準確）" if csv_mode else
                      "★行號合成時間，非真實落地時間——停頓偵測在此模式不列入判定"
                      if synth_time else
                      "即時 console 列印到達時間（含序列埠/列印延遲，僅供參考）")
        add_check("紀錄管線 (Recording Pipeline)", "平均封包落地間隔", gaps["mean_ms"], 0.0, "ms",
                  ">", False, False, f"時間基準：{time_basis}；共 {gaps['count']} 個間隔樣本")
        add_check("紀錄管線 (Recording Pipeline)", "最大停頓 (Stall)", gaps["max_ms"],
                  SPEC_LIMITS["pipeline_stall_warn_ms"], "ms", "<=",
                  (not synth_time) and gaps["max_ms"] > SPEC_LIMITS["pipeline_stall_fail_ms"],
                  (not synth_time) and gaps["max_ms"] > SPEC_LIMITS["pipeline_stall_warn_ms"],
                  "疑似對應 Flash 用前才擦的 sector erase 阻塞（每 30+ 筆一次，約 400ms）"
                  if gaps["max_ms"] > SPEC_LIMITS["pipeline_stall_warn_ms"] else "無明顯停頓")
        add_check("紀錄管線 (Recording Pipeline)", "停頓次數比例", gaps["stall_ratio"] * 100.0,
                  SPEC_LIMITS["pipeline_stall_ratio_max"] * 100.0, "%", "<=",
                  (not synth_time) and gaps["stall_ratio"] > SPEC_LIMITS["pipeline_stall_ratio_max"] * 2.0,
                  (not synth_time) and gaps["stall_ratio"] > SPEC_LIMITS["pipeline_stall_ratio_max"],
                  f"{gaps['stall_count']}/{gaps['count']} 個間隔超過 {SPEC_LIMITS['pipeline_stall_warn_ms']:.0f}ms")

        # --- 地面站自身 GPS
        if csv_mode:
            gs_fix_vals = [e.get("gs_fix", 0) for e in pkt_events if "gs_fix" in e]
            gs_sats_vals = [e.get("gs_sats", 0) for e in pkt_events if "gs_sats" in e]
            fix_ratio = (sum(gs_fix_vals) / float(len(gs_fix_vals))) if gs_fix_vals else 0.0
            sats_mean = StatMetrics(gs_sats_vals).mean if gs_sats_vals else 0.0
            sats_min = StatMetrics(gs_sats_vals).min_val if gs_sats_vals else 0.0
            gps_src = f"CSV {len(gs_fix_vals)} 列（隨每筆遙測紀錄同時落地）"
        elif "gs_gps_lines" in diag:
            fix_ratio = diag["gs_gps_fix_ratio"]
            sats_mean = diag["gs_gps_sats_mean"]
            sats_min = diag["gs_gps_sats_min"]
            gps_src = f"{diag['gs_gps_lines']} 筆 [GS_GPS] 診斷行"
        else:
            fix_ratio, sats_mean, sats_min, gps_src = None, None, None, None

        if fix_ratio is not None:
            add_check("地面站自身 GPS", "定位狀態 (Fix)", fix_ratio * 100.0, 100.0, "%",
                      "==", fix_ratio <= 0.0, fix_ratio < 1.0,
                      f"有效 Fix 比例: {fix_ratio*100:.1f}%（統計基礎：{gps_src}）"
                      f"——僅影響時間對齊/地理標記，非鏈路本身健康度")
            add_check("地面站自身 GPS", "衛星數", sats_mean, SPEC_LIMITS["gps_sats_min"], "sats",
                      ">=", False, sats_mean < SPEC_LIMITS["gps_sats_min"],
                      f"平均={sats_mean:.1f}, 最小={sats_min:.0f}")
        else:
            add_check("地面站自身 GPS", "定位狀態 (Fix)", 0.0, 100.0, "%",
                      "==", False, False, "未見到 [GS_GPS] 行或 CSV 無 gs_fix 欄位")
            add_check("地面站自身 GPS", "衛星數", 0.0, SPEC_LIMITS["gps_sats_min"], "sats",
                      ">=", False, False, "未見到 [GS_GPS] 行或 CSV 無 gs_sats 欄位")

        # --- 硬體初始化狀態（僅即時 console 擷取才有）
        if "lora_init_433" in diag:
            i433, i920 = diag["lora_init_433"], diag["lora_init_920"]
            both_ok = ("OK" in i433) and ("OK" in i920)
            one_ok = ("OK" in i433) or ("OK" in i920)
            add_check("硬體狀態", "雙鏈路初始化", 1 if both_ok else 0, 1, "",
                      "==", not one_ok, not both_ok, f"433={i433} | 920={i920}")

        has_fail = any(c["status"] == "FAIL" for c in checks)
        has_warn = any(c["status"] == "WARN" for c in checks)
        overall_status = "NOT_READY" if has_fail else ("DEGRADED" if has_warn else "READY")

        return {
            "summary": {
                "overall_status": overall_status,
                "sample_count": n,
                "duration_sec": round(span_s, 2),
                "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "fail_count": sum(1 for c in checks if c["status"] == "FAIL"),
                "warn_count": sum(1 for c in checks if c["status"] == "WARN"),
                "pass_count": sum(1 for c in checks if c["status"] == "PASS"),
                "n_433": n_433, "n_920": n_920,
                "data_source": ("CSV (SD 卡離線)" if csv_mode else
                                "文字 log 離線（時間戳為行號合成，間隔類指標不列入判定）"
                                if synth_time else "即時 Console 擷取"),
            },
            "checks": checks,
            "link_summaries": {"433": link_summaries.get(LINK_433, {}), "920": link_summaries.get(LINK_920, {})},
            "time_series": {
                "t_rel": [e["t"] - t0 for e in pkt_events],
                "link": [e["link"] for e in pkt_events],
                "rssi": [e["rssi"] for e in pkt_events],
                "snr": [e["snr"] / 4.0 if e["snr"] != RSSI_SNR_NA else None for e in pkt_events],
                "gap_ms": [None] + [(all_times[i] - all_times[i - 1]) * 1000.0 for i in range(1, len(all_times))],
            },
        }


# ===========================================================================
#  即時圖表 GUI
# ===========================================================================
class LivePlotter:
    def __init__(self, engine: GsAnalyzerEngine, stop_event: threading.Event):
        self.engine = engine
        self.stop_event = stop_event
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
        self.fig.canvas.manager.set_window_title("RocketCom 地面站接收品質即時監控 (按 Enter 或關閉視窗結束擷取)")

        self.ax_rate = self.axs[0]
        self.line_433, = self.ax_rate.plot([], [], 'g.-', label='433 累計封包數')
        self.line_920, = self.ax_rate.plot([], [], 'b.-', label='920 累計封包數')
        self.ax_rate.set_ylabel("累計封包數")
        self.ax_rate.legend(loc='upper left', fontsize=8)
        self.ax_rate.grid(True, linestyle=':', alpha=0.6)

        self.ax_rssi = self.axs[1]
        self.line_rssi, = self.ax_rssi.plot([], [], 'r.', label='920 RSSI (dBm)', markersize=3)
        self.ax_rssi2 = self.ax_rssi.twinx()
        self.line_snr, = self.ax_rssi2.plot([], [], 'c.', label='920 SNR (dB)', markersize=3)
        self.ax_rssi.set_ylabel("RSSI (dBm)")
        self.ax_rssi2.set_ylabel("SNR (dB)")
        self.ax_rssi.grid(True, linestyle=':', alpha=0.6)

        self.ax_gap = self.axs[2]
        self.line_gap, = self.ax_gap.plot([], [], 'm.-', label='封包落地間隔 (ms)')
        self.ax_gap.axhline(SPEC_LIMITS["pipeline_stall_warn_ms"], color='orange', linestyle='--', linewidth=1)
        self.ax_gap.set_ylabel("Gap (ms)")
        self.ax_gap.set_xlabel("Time (s)")
        self.ax_gap.legend(loc='upper left', fontsize=8)
        self.ax_gap.grid(True, linestyle=':', alpha=0.6)

        self.diag_text = self.fig.text(0.01, 0.005, "", fontsize=8, family='monospace', color='dimgray')
        self.fig.tight_layout(rect=(0, 0.02, 1, 1))
        self.fig.canvas.mpl_connect('close_event', self.on_close)

    def on_close(self, event):
        self.stop_event.set()

    def update(self, frame):
        pkts, _crc_bad_events, gs_stats, diag = self.engine.snapshot()
        if not pkts:
            return self.line_433,
        t0 = pkts[0]["t"]

        t433, t920 = [], []
        for e in pkts:
            (t433 if e["link"] == LINK_433 else t920).append(e["t"] - t0)
        self.line_433.set_data(t433, list(range(1, len(t433) + 1)))
        self.line_920.set_data(t920, list(range(1, len(t920) + 1)))
        self.ax_rate.relim(); self.ax_rate.autoscale_view()

        rssi_t, rssi_v, snr_t, snr_v = [], [], [], []
        for e in pkts:
            if e["link"] == LINK_920 and e["rssi"] != RSSI_SNR_NA:
                rssi_t.append(e["t"] - t0); rssi_v.append(e["rssi"])
            if e["link"] == LINK_920 and e["snr"] != RSSI_SNR_NA:
                snr_t.append(e["t"] - t0); snr_v.append(e["snr"] / 4.0)
        self.line_rssi.set_data(rssi_t, rssi_v)
        self.line_snr.set_data(snr_t, snr_v)
        self.ax_rssi.relim(); self.ax_rssi.autoscale_view()
        self.ax_rssi2.relim(); self.ax_rssi2.autoscale_view()

        all_times = sorted(e["t"] for e in pkts)
        gap_t = [t - t0 for t in all_times[1:]]
        gap_v = [(all_times[i] - all_times[i - 1]) * 1000.0 for i in range(1, len(all_times))]
        self.line_gap.set_data(gap_t, gap_v)
        self.ax_gap.relim(); self.ax_gap.autoscale_view()

        gstat_str = "--"
        if gs_stats:
            gs = gs_stats[-1]
            gstat_str = (f"HW 433={'OK' if gs['hw433_ok'] else 'OFF'} 920={'OK' if gs['hw920_ok'] else 'OFF'}  "
                         f"433 ok={gs['ok433']} crc={gs['crc433']} rsync={gs['rsync433']}  "
                         f"920 ok={gs['ok920']} crc={gs['crc920']} rsync={gs['rsync920']}")
        gps_str = "--"
        if "gs_gps_lines" in diag:
            gps_str = f"GS-GPS fix={diag['gs_gps_fix_ratio']*100:.0f}% sats={diag['gs_gps_sats_mean']:.1f}"
        self.diag_text.set_text(f"{gstat_str}   {gps_str}")

        self.fig.suptitle(f"📡 地面站接收監控：433={len(t433)} 筆 / 920={len(t920)} 筆 | 按 Enter 或關閉圖表結束",
                           fontsize=11, color='navy')
        return self.line_433,

    def start(self):
        ani = FuncAnimation(self.fig, self.update, interval=200, blit=False)
        plt.show()


# ===========================================================================
#  報告生成器
# ===========================================================================
class ReportGenerator:
    _BADGE = {"READY": ("🟢", "[ READY - 地面站接收系統就緒 ]"),
              "DEGRADED": ("🟡", "[ DEGRADED - 部分項目需注意 ]"),
              "NOT_READY": ("🔴", "[ NOT_READY - 接收/紀錄管線異常 ]")}

    @staticmethod
    def print_terminal_report(res: dict):
        s, checks = res["summary"], res["checks"]
        c_reset, c_bold = "\033[0m", "\033[1m"
        color = {"READY": "\033[42;30m", "DEGRADED": "\033[43;30m", "NOT_READY": "\033[41;37m"}[s["overall_status"]]
        badge = f"{color}{c_bold}  {ReportGenerator._BADGE[s['overall_status']][1]}  {c_reset}"

        print("\n" + "=" * 82)
        print(f"{c_bold}📡 RocketCom 地面站接收品質與紀錄管線分析報告{c_reset}")
        print("=" * 82)
        print(f" 測量時間   : {s['timestamp']}")
        print(f" 資料來源   : {s['data_source']}")
        print(f" 統計樣本   : {s['sample_count']} 筆封包 ({s['duration_sec']} 秒)  433={s['n_433']} / 920={s['n_920']}")
        print(f" 判定結果   : {badge}")
        print(f" 項目統計   : PASS: \033[32m{s['pass_count']}\033[0m  WARN: \033[33m{s['warn_count']}\033[0m  "
              f"FAIL: \033[31m{s['fail_count']}\033[0m")
        print("-" * 82)
        print(f"{'類別':<26} {'檢查項目':<22} {'實測值':<14} {'限值':<12} {'狀態':<8}")
        print("-" * 82)
        for c in checks:
            st = c["status"]
            col = {"PASS": "\033[32m[ PASS ]", "WARN": "\033[33m[ WARN ]", "FAIL": "\033[31m[ FAIL ]"}[st] + c_reset
            val_str = f"{c['val']:.2f} {c['unit']}"
            lim_str = f"{c['limit']:.2f} {c['unit']}"
            print(f"{c['category']:<26} {c['name']:<20} {val_str:<14} {lim_str:<12} {col}")
        print("=" * 82)
        if s["fail_count"] > 0:
            print("\033[31m[!] 偵測到嚴重問題：接收或紀錄管線可能有明顯漏包/停頓，請依報告排錯。\033[0m")
        elif s["warn_count"] > 0:
            print("\033[33m[*] 部分指標接近門檻，建議留意天線/RF 參數或紀錄管線負載。\033[0m")
        else:
            print("\033[32m[✓] 地面站雙鏈路接收與紀錄管線健康良好。\033[0m")
        print("=" * 82 + "\n")

    @staticmethod
    def generate_markdown(res: dict, filepath: str):
        s, c = res["summary"], res["checks"]
        icon, label = ReportGenerator._BADGE[s["overall_status"]]
        lines = [
            "# 📡 RocketCom 地面站接收品質與紀錄管線分析報告", "",
            f"- **判定狀態**：{icon} **{label}**",
            f"- **報告生成時間**：`{s['timestamp']}`",
            f"- **資料來源**：`{s['data_source']}`",
            f"- **統計樣本**：`{s['sample_count']} 筆封包` / `{s['duration_sec']} 秒`"
            f"（433=`{s['n_433']}`, 920=`{s['n_920']}`）",
            f"- **檢查統計**：`PASS: {s['pass_count']}` | `WARN: {s['warn_count']}` | `FAIL: {s['fail_count']}`",
            "", "---", "",
            "## 檢查項目明細", "",
            "| 類別 | 檢查項目 | 實測值 | 限值 | 狀態 | 備註 |",
            "| :--- | :--- | :--- | :--- | :---: | :--- |",
        ]
        for item in c:
            icon2 = {"PASS": "✅ PASS", "WARN": "⚠️ WARN", "FAIL": "❌ FAIL"}[item["status"]]
            lines.append(f"| {item['category']} | {item['name']} | `{item['val']:.2f} {item['unit']}` | "
                         f"`{item['limit']:.2f} {item['unit']}` | {icon2} | {item['detail']} |")
        lines.extend([
            "", "---", "",
            "## 排錯建議",
            "1. **總通訊頻率遠低於排定值(920≈10Hz/433≈1.55Hz，433 這個值是估計非實測)**："
            "RF 前端可能根本沒收到訊號（天線/接線/供電/模組未初始化），先查這個再查 CRC，"
            "順序不能反；433 若卡在排定值邊緣、CRC 又是 0%，也可能是估計值本身偏樂觀。",
            "2. **總通訊頻率接近排定值，但 CRC 錯誤率偏高**：代表 RF 前端有收到東西、只是解不出來——"
            "檢查天線接頭/駐波、RF 參數(SF/BW/CR 兩端須一致)，或空中速率與雜訊環境不符。",
            "3. **Resync 比例偏高（433）**：多半是空中位元速率不符或強雜訊源干擾，非單純距離問題。",
            f"4. **單鏈路平均間隔/最長空窗偏長**：433 的間隔天生不規則——上行接收窗每 "
            f"{UPLINK_LISTEN_PERIOD_MS:.0f}ms 會連續靜默 {UPLINK_LISTEN_HOLD_MS:.0f}ms"
            f"（佔 {UPLINK_LISTEN_DUTY*100:.0f}% 時間），加上單包空中時間本身就數百 ms，"
            f"門檻已照它自己的名目速率放寬；超標且合併空窗也跟著長，才要查天線/距離/遮蔽。"
            f"另外，間隔變長不一定是 RF：航電端卡住晚發也會這樣，需對照該板 [LORA_TX_LOG] 分辨。",
            "5. **合併最長空窗偏長但單鏈路正常**：檢查兩鏈路是不是在同一段時間一起斷"
            "（火箭端 TX 排程/供電異常，而非各自的 RF 環境問題）——備援等於沒有備援。",
            "6. **紀錄管線停頓**：對應已知 Flash sector erase 阻塞，嚴重時可能造成飛行末段掉包，"
            "建議改用 --csv 對實際 SD 卡 GSLOGnnn.CSV 做離線分析確認真實影響。",
            "",
        ])
        with open(filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

    @staticmethod
    def generate_json(res: dict, filepath: str):
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(res, f, ensure_ascii=False, indent=2)

    @staticmethod
    def generate_html(res: dict, filepath: str):
        s, checks, ts = res["summary"], res["checks"], res["time_series"]
        badge_class = {"READY": "go", "DEGRADED": "warn", "NOT_READY": "nogo"}[s["overall_status"]]

        rows_html = ""
        for c in checks:
            st_cls = {"PASS": "st-pass", "WARN": "st-warn", "FAIL": "st-fail"}[c["status"]]
            rows_html += (f"<tr><td><strong>{c['category']}</strong></td><td>{c['name']}</td>"
                          f"<td><code>{c['val']:.2f} {c['unit']}</code></td>"
                          f"<td><code>{c['limit']:.2f} {c['unit']}</code></td>"
                          f"<td><span class=\"status-pill {st_cls}\">{c['status']}</span></td>"
                          f"<td class=\"text-muted\">{c['detail']}</td></tr>")

        gap_vals = [g if g is not None else 0 for g in ts["gap_ms"]]
        html_content = f"""<!DOCTYPE html>
<html lang="zh-TW">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RocketCom 地面站接收品質報告</title>
<style>
:root {{ --bg:#0f172a; --card-bg:#1e293b; --text:#f8fafc; --text-muted:#94a3b8; --border:#334155;
--pass:#22c55e; --warn:#eab308; --fail:#ef4444; --accent:#38bdf8; }}
body {{ font-family: system-ui, -apple-system, sans-serif; background:var(--bg); color:var(--text);
margin:0; padding:24px; line-height:1.5; }}
.container {{ max-width:1200px; margin:0 auto; }}
.header {{ display:flex; justify-content:space-between; align-items:center; border-bottom:2px solid var(--border);
padding-bottom:16px; margin-bottom:24px; flex-wrap:wrap; gap:12px; }}
h1 {{ margin:0; font-size:1.8rem; letter-spacing:-0.025em; }}
.badge {{ padding:8px 20px; border-radius:9999px; font-weight:800; font-size:1.0rem; letter-spacing:0.03em; }}
.badge.go {{ background:rgba(34,197,94,.2); color:var(--pass); border:2px solid var(--pass); }}
.badge.warn {{ background:rgba(234,179,8,.2); color:var(--warn); border:2px solid var(--warn); }}
.badge.nogo {{ background:rgba(239,68,68,.2); color:var(--fail); border:2px solid var(--fail); }}
.grid {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr)); gap:16px; margin-bottom:24px; }}
.card {{ background:var(--card-bg); border:1px solid var(--border); border-radius:12px; padding:16px; }}
.card .title {{ font-size:.85rem; color:var(--text-muted); text-transform:uppercase; }}
.card .value {{ font-size:1.8rem; font-weight:700; margin-top:4px; color:var(--accent); }}
table {{ width:100%; border-collapse:collapse; background:var(--card-bg); border-radius:12px; overflow:hidden;
border:1px solid var(--border); margin-bottom:24px; }}
th,td {{ padding:12px 16px; text-align:left; border-bottom:1px solid var(--border); }}
th {{ background:#0f172a; color:var(--text-muted); font-size:.85rem; text-transform:uppercase; }}
tr:hover {{ background:rgba(255,255,255,.02); }}
.status-pill {{ padding:4px 10px; border-radius:6px; font-size:.75rem; font-weight:700; }}
.st-pass {{ background:rgba(34,197,94,.15); color:var(--pass); }}
.st-warn {{ background:rgba(234,179,8,.15); color:var(--warn); }}
.st-fail {{ background:rgba(239,68,68,.15); color:var(--fail); }}
.text-muted {{ color:var(--text-muted); font-size:.9rem; }}
.chart-box {{ background:var(--card-bg); border:1px solid var(--border); border-radius:12px; padding:20px;
margin-bottom:24px; }}
canvas {{ width:100%; height:260px; }}
</style>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
<div class="container">
<div class="header">
<div><h1>📡 RocketCom 地面站接收品質 Dashboard</h1>
<div class="text-muted">生成時間：{s['timestamp']} | 來源：{s['data_source']} | 樣本：{s['sample_count']} 筆
（433={s['n_433']}, 920={s['n_920']}）</div></div>
<div class="badge {badge_class}">{s['overall_status']}</div>
</div>
<div class="grid">
<div class="card"><div class="title">PASS 項目</div><div class="value" style="color:var(--pass);">{s['pass_count']}</div></div>
<div class="card"><div class="title">WARN 項目</div><div class="value" style="color:var(--warn);">{s['warn_count']}</div></div>
<div class="card"><div class="title">FAIL 項目</div><div class="value" style="color:var(--fail);">{s['fail_count']}</div></div>
<div class="card"><div class="title">擷取時長</div><div class="value">{s['duration_sec']}s</div></div>
</div>
<h2>📋 檢查項目</h2>
<table><thead><tr><th>類別</th><th>檢查項目</th><th>實測值</th><th>限值</th><th>狀態</th><th>備註</th></tr></thead>
<tbody>{rows_html}</tbody></table>
<h2>📈 封包落地間隔 (ms)</h2>
<div class="chart-box"><canvas id="chartGap"></canvas></div>
</div>
<script>
new Chart(document.getElementById('chartGap'), {{
  type: 'line',
  data: {{
    labels: {json.dumps([round(t, 2) for t in ts["t_rel"]])},
    datasets: [{{ label: 'Gap (ms)', data: {json.dumps(gap_vals)}, borderColor: '#a855f7',
                  borderWidth: 1.5, pointRadius: 0 }}]
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
#  模擬測試資料
# ===========================================================================
def generate_selftest_events(engine: GsAnalyzerEngine):
    """依 main.c LoRaTelemetry_Task 的真實排程模擬：兩鏈路共用同一個每 100ms 遞增的全域
    seq，920 幾乎每個 tick 都發；433 每個 tick 都嘗試呼叫 LoRaE22_Send()(LORA433_TX_EVERY=1)，
    但 AUX busy（上一包空中時間還沒跑完，見 LORA433_AIRTIME_S 估計值）時會直接跳過不等待，
    所以兩次成功發射之間最少要隔 LORA433_MIN_TX_SPACING_TICKS 個 tick；此外每
    UPLINK_LISTEN_PERIOD_TICKS(=30) 個 tick 會有連續 UPLINK_LISTEN_HOLD_TICKS(=10) 個
    tick 完全不發、留給上行接收窗（守衛帶 600ms + 窗 400ms，見常數宣告處）。這兩個規律間隙都是設計行為、不是遺失——433 的 gap
    門檻就是照這個實際排程換算出的名目速率放寬的，selftest 資料若不照這個排程生成，
    就測不出 gap 判定的門檻有沒有設歪。"""
    import random
    t = time.time() - 60.0
    gs_stat = {"hw433_ok": True, "hw920_ok": True, "raw433": 0, "ok433": 0, "crc433": 0, "rsync433": 0,
               "ok920": 0, "crc920": 0, "rsync920": 0, "pkts433": 0, "pkts920": 0}
    last_433_tx = -LORA433_MIN_TX_SPACING_TICKS
    for i in range(900):   # 900 個 100ms tick ≈ 90 秒
        t += LORA_TELEM_PERIOD_MS / 1000.0
        if i == 450:
            t += 0.45   # 模擬一次 Flash sector erase 阻塞造成的停頓
        global_seq = i & 0xFF

        # 920：幾乎每個 tick 都發，良好但偶有 CRC 錯誤
        if random.random() > 0.03:
            rssi = int(random.gauss(-85, 6))
            snr = int(random.gauss(34, 8))
            engine.add_pkt(t, LINK_920, rssi, snr, global_seq)
            gs_stat["ok920"] += 1
        else:
            engine.add_pkt_bad(t, LINK_920, global_seq)
            gs_stat["crc920"] += 1

        # 433：每個 tick 都嘗試(LORA433_TX_EVERY=1)，但 AUX busy（兩次成功發射至少間隔
        # LORA433_MIN_TX_SPACING_TICKS 個 tick）與上行接收窗都會讓這次嘗試直接被跳過、
        # 不真的上空——跳過不佔用下一次的 spacing 起點。
        # 接收窗比照韌體改成「連續一段不發」（每 PERIOD_TICKS 開頭連續 HOLD_TICKS 個
        # tick），而不是舊的每 10 個跳 1 個。
        listen_slot = (i % UPLINK_LISTEN_PERIOD_TICKS) < UPLINK_LISTEN_HOLD_TICKS
        aux_busy = (i - last_433_tx) < LORA433_MIN_TX_SPACING_TICKS
        if (i % LORA433_TX_EVERY) == 0 and not listen_slot and not aux_busy:
            last_433_tx = i
            if random.random() > 0.06:
                engine.add_pkt(t + 0.02, LINK_433, RSSI_SNR_NA, RSSI_SNR_NA, global_seq)
                gs_stat["ok433"] += 1
            else:
                engine.add_pkt_bad(t + 0.02, LINK_433, global_seq)
                gs_stat["crc433"] += 1
            gs_stat["raw433"] += 12

        gs_stat["rsync433"] += 1 if random.random() < 0.02 else 0
        gs_stat["rsync920"] += 1 if random.random() < 0.01 else 0
        gs_stat["pkts433"], gs_stat["pkts920"] = gs_stat["ok433"], gs_stat["ok920"]
        if i % 10 == 0:
            engine.add_gs_stat(t, gs_stat)
    engine.update_diag("gs_gps_lines", 30)
    engine.update_diag("gs_gps_fix_ratio", 1.0)
    engine.update_diag("gs_gps_sats_mean", 9.5)
    engine.update_diag("gs_gps_sats_min", 8)
    engine.update_diag("lora_init_433", "OK (Ready)")
    engine.update_diag("lora_init_920", "OK (Ready)")


# ===========================================================================
#  主程序
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="RocketCom 地面站接收品質與紀錄管線分析工具")
    ap.add_argument("--port", help="地面站板 UART2 序列埠路徑（如 /dev/cu.usbserial-0001 或 AUTO）")
    ap.add_argument("--baud", type=int, default=460800, help="鮑率 (預設 460800)")
    ap.add_argument("--duration", type=float, default=None, help="固定擷取時間(秒)；未指定則按 Enter 結束")
    ap.add_argument("--csv", help="離線分析 SD 卡紀錄檔 GSLOGnnn.CSV（最準確的紀錄速度量測）")
    ap.add_argument("--file", help="離線分析文字 console log 檔（[GS_PKT]/[GS_STAT] 等行）")
    ap.add_argument("--selftest", action="store_true", help="執行模擬資料自我測試")
    ap.add_argument("--no-gui", action="store_true", help="不開即時圖表視窗（純文字模式）")
    ap.add_argument("--stats-auto", type=int, default=5,
                    help="即時連線時，自動發送 `stats auto N` 命令啟用韌體週期性 RSSI/SNR/速率回報"
                         "（0=不發送，見 gs_lora_test.c）")
    ap.add_argument("--out-dir", default=os.path.join(SYS_PARENT, "reports"), help="報告輸出目錄")
    args = ap.parse_args()

    engine = GsAnalyzerEngine()
    stop_event = threading.Event()
    csv_mode = False
    synth_time = False     # --file 模式：時間戳由行號合成，間隔類指標不列入判定

    if args.selftest:
        print("[SELFTEST] 執行地面站接收品質分析邏輯測試...")
        generate_selftest_events(engine)

    elif args.csv:
        print(f"[CSV] 正在讀取地面站紀錄檔: {args.csv} ...")
        if not os.path.exists(args.csv):
            print(f"[ERROR] 檔案不存在: {args.csv}")
            sys.exit(1)
        events = load_gs_log_csv(args.csv)
        if not events:
            print("[ERROR] CSV 內未解析到任何有效列（欄位需與 GsLog_CsvHeader 一致）")
            sys.exit(1)
        engine.load_offline_csv_events(events)
        csv_mode = True

    elif args.file:
        print(f"[FILE] 正在讀取地面站 console log 檔: {args.file} ...")
        if not os.path.exists(args.file):
            print(f"[ERROR] 檔案不存在: {args.file}")
            sys.exit(1)
        parser = GsConsoleParser(engine.add_pkt, engine.add_pkt_bad, engine.add_gs_stat, engine.update_diag)
        # 離線 log 檔沒有真實到達時間戳，退而以行號合成遞增秒數（僅供計數/比率類指標使用）。
        # ★間隔(gap)/停頓偵測在此模式下沒有物理意義，analyze(synth_time=True) 會照算照印
        # 但不判 WARN/FAIL，避免合成時間算出的漂亮數字被誤讀成鏈路健康。
        synth_time = True
        synth_t = [0.0]

        def _feed_line_synth(line):
            synth_t[0] += 0.05
            parser._feed_line(line.strip(), synth_t[0])

        with open(args.file, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                _feed_line_synth(line)

    else:
        if serial_link is None:
            print("[ERROR] 找不到 serial_link 模組，請確認目錄結構。")
            sys.exit(1)
        port = args.port or serial_link.resolve_port()
        print(f"🔄 正在連線地面站板 UART2: {port} @ {args.baud} baud ...")
        try:
            # ★2026-07-28：pyserial 的 read(size) 在湊不滿 size 前會一路卡到 timeout 才
            # 返回剩下的部分——0.5s 太大，這個 baud/印出速率下 ser.read(4096) 幾乎每次都
            # 卡滿整整 500ms 才回來，把這段時間內所有 console 行一次性吐出，害「紀錄管線」
            # gap 統計變成人工的 0ms(同批瞬間蓋章)/500ms(批間卡 timeout) 兩極值，跟真實
            # 封包到達間隔/SD 記錄卡頓無關。降到遠小於封包週期(100ms)的值才不會把時間攤平。
            ser = serial_link.open_serial(port, args.baud, timeout=0.02)
        except Exception as e:
            print(f"[ERROR] 無法開啟串口 {port}: {e}")
            sys.exit(1)

        if args.stats_auto > 0:
            try:
                ser.write(f"stats auto {args.stats_auto}\r\n".encode())
            except Exception:
                pass

        parser = GsConsoleParser(engine.add_pkt, engine.add_pkt_bad, engine.add_gs_stat, engine.update_diag)

        def rx_thread_proc():
            t_start = time.time()
            while not stop_event.is_set():
                if args.duration and (time.time() - t_start >= args.duration):
                    stop_event.set()
                    break
                try:
                    chunk = ser.read(4096)
                    if chunk:
                        parser.feed(chunk)
                except Exception:
                    pass
                time.sleep(0.005)

        t_rx = threading.Thread(target=rx_thread_proc, daemon=True)
        t_rx.start()

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
        print(f"📡 已連線地面站板 ({port})！開始即時擷取接收品質數據中...")
        if args.duration:
            print(f"⏱️ 擷取時間：{args.duration} 秒 (可隨時按 Enter 提早結束)")
        else:
            print("⏱️ 擷取時間：由您決定，按 [Enter] 結束擷取")
        print("=" * 70 + "\n")

        if HAS_MATPLOTLIB and not args.no_gui:
            plotter = LivePlotter(engine, stop_event)
            plotter.start()
            stop_event.set()
        else:
            t0 = time.time()
            try:
                while not stop_event.is_set():
                    time.sleep(0.2)
                    pkts, _, _, _ = engine.snapshot()
                    elapsed = time.time() - t0
                    sys.stdout.write(f"\r[LIVE 擷取中] 已累積 {len(pkts)} 筆封包事件 | 時間: {elapsed:.1f}s (按 Enter 結束) ")
                    sys.stdout.flush()
            except KeyboardInterrupt:
                stop_event.set()

        try:
            if args.stats_auto > 0:
                ser.write(b"stats auto 0\r\n")
        except Exception:
            pass

        print("\n\n[LIVE] 擷取完成！正在進行接收品質與紀錄管線分析...")

    res = engine.analyze(csv_mode=csv_mode, synth_time=synth_time)
    if "error" in res:
        print(f"[ERROR] 分析失敗: {res['error']}")
        sys.exit(1)

    ReportGenerator.print_terminal_report(res)

    os.makedirs(args.out_dir, exist_ok=True)
    ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    md_file = os.path.join(args.out_dir, f"ground_station_report_{ts_str}.md")
    json_file = os.path.join(args.out_dir, f"ground_station_report_{ts_str}.json")
    html_file = os.path.join(args.out_dir, f"ground_station_report_{ts_str}.html")

    ReportGenerator.generate_markdown(res, md_file)
    ReportGenerator.generate_json(res, json_file)
    ReportGenerator.generate_html(res, html_file)

    print(f"📝 Markdown 報告已生成: file://{os.path.abspath(md_file)}")
    print(f"📊 JSON 結構化數據檔已生成: file://{os.path.abspath(json_file)}")
    print(f"🌐 互動式 HTML Dashboard 已生成: file://{os.path.abspath(html_file)}")

    # 離線 CSV 模式下，同一份紀錄還帶著完整飛行物理量 + 雙板 peer 摘要 + 兩端 GPS，
    # 順手一併產出轉播飛行圖表 + GPS 地圖（與上面的鏈路品質報告互補，見函式註解）。
    if csv_mode:
        try:
            generate_relay_flight_chart(args.csv, args.out_dir)
            generate_relay_gps_map(args.csv, args.out_dir)
        except Exception as e:
            print(f"[RELAY] 轉播飛行圖表/地圖生成失敗: {e}")

    if res["summary"]["overall_status"] == "NOT_READY":
        sys.exit(2)


if __name__ == "__main__":
    main()
