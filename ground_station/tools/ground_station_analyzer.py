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
     封包速率、CRC 錯誤率、Resync（雜訊）比例、單鏈路序號丟包率。
  2. 【合併丟包率】：兩鏈路依 seq+時間相近去重後合併計算，才是地面站「實際到手」的
     完整度 —— 這才是雙鏈路備援真正該看的指標，比任一單鏈路的丟包率更關鍵。
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
            matplotlib.use("MacOSX")
        except Exception:
            matplotlib.use("TkAgg")
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
# (100ms) 打包一筆並「共用同一個 seq」，920(E80) 幾乎每個時槽都發(僅受 BUSY 背壓跳過)，
# 433(E22) 因空中時間較長，韌體刻意每 LORA433_TX_EVERY(=3) 個時槽才發一次（~3.33Hz）。
# 這代表 433 連續兩筆「正常」封包之間 seq 本來就該差 3，不能套用「差 1 才正常」的丟包演算法
# ——早期版本沒做這個正規化，曾把 433 的合法降速誤判成 90%+ 丟包（見兩次真實 report 比對）。
LORA_TELEM_PERIOD_MS = 100.0
LORA433_TX_EVERY = 3
NOMINAL_RATE_HZ = {
    LINK_433: 1000.0 / (LORA_TELEM_PERIOD_MS * LORA433_TX_EVERY),  # ≈3.33 Hz
    LINK_920: 1000.0 / LORA_TELEM_PERIOD_MS,                        # 10 Hz（受空中時間/BUSY 影響）
}
SEQ_STEP = {LINK_433: LORA433_TX_EVERY, LINK_920: 1}

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
    # 單鏈路 seq 丟包率：允許比合併值寬鬆，因為單鏈路失手本就可能被另一鏈路補上。
    "single_link_loss_max_ratio": 0.10,
    # 總通訊頻率(含 CRC 無效)相對韌體排定速率(NOMINAL_RATE_HZ)的比例門檻：低於此比例代表
    # RF 前端可能根本沒同步到訊號(而非單純解碼品質問題)。
    "total_rate_warn_ratio": 0.60,
    "total_rate_fail_ratio": 0.30,
    # 合併(兩鏈路去重後)丟包率：地面站「實際到手」的完整度，是本工具最重要的
    # GO/NO-GO 指標——即使某一鏈路整個掛掉，只要另一鏈路頂住，這裡就該還在低位。
    "combined_loss_max_ratio": 0.05,
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


def calc_seq_loss(seqs: list, step: int = 1) -> dict:
    """依 uint8 seq 序號（可能重複/繞回）算丟包率。同 seq 重複（常見於雙鏈路都收到同一筆，
    或去重前同一封包被算兩次）不算新進度也不算丟包。

    step：此鏈路每隔幾個火箭端 100ms 全域 tick 才會被排到發送一次（見 NOMINAL_RATE_HZ 註解，
    433=3、920=1）。若不做這個正規化，433 因排程本就每 3 個 tick 才發一次，會被誤判成
    每次「seq 差 3」都是丟了 2 筆——這正是早期版本把合法降速誤報成 90%+ 丟包的根因。"""
    total_gap = 0
    total_steps = 0
    for i in range(1, len(seqs)):
        gap = (seqs[i] - seqs[i - 1]) % 256
        if gap == 0:
            continue
        total_gap += gap
        total_steps += 1
    expected = total_gap / float(step)
    lost = max(expected - total_steps, 0.0)
    ratio = (lost / expected) if expected > 0 else 0.0
    return {"expected": expected, "lost": lost, "ratio": ratio}


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


def gap_stats(times: list, warn_ms: float, fail_ms: float) -> dict:
    """連續事件時間戳的間隔統計（ms），用來偵測紀錄管線停頓。"""
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
    def analyze(self, csv_mode=False) -> dict:
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

            add_check(name, "有效通訊頻率 (CRC 正確)", rate_hz, 0.0, "pkt/s", ">",
                      ok_count == 0, rate_hz <= 0.0 and ok_count > 0,
                      f"{rate_src}；本次擷取到 {ok_count} 筆有效封包 / {bad_count} 筆 CRC 錯誤")

            # --- 總通訊頻率（含 CRC 無效）：只要有觸發同步/CRC 檢查就算「有通訊」，不論解碼
            # 是否成功。跟韌體設計排程(見 NOMINAL_RATE_HZ：920≈10Hz，433≈3.33Hz)比較，能分辨
            # 「RF 前端根本沒收到東西」(總頻率遠低於排定值) 跟「有收到但解不出來」(總頻率接近
            # 排定值、但有效頻率偏低，即 CRC 錯誤率高) 這兩種完全不同的問題。
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
            nominal = NOMINAL_RATE_HZ[link]
            add_check(name, "總通訊頻率 (含 CRC 無效)", total_rate_hz, nominal, "pkt/s",
                      ">=", total_rate_hz < nominal * SPEC_LIMITS["total_rate_fail_ratio"],
                      total_rate_hz < nominal * SPEC_LIMITS["total_rate_warn_ratio"],
                      f"{total_src}；韌體排定速率≈{nominal:.2f}Hz（見 main.c LORA_TELEM_PERIOD_MS/"
                      f"LORA433_TX_EVERY）。遠低於排定值代表 RF 前端可能根本沒收到訊號，"
                      f"而非單純解碼品質問題")

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

            # --- 單鏈路 RF 到達率丟失：CRC 錯誤的封包仍證明「這個排定時槽有訊號進來」，
            # 只是解不出來，不該跟「完全沒收到」混為一談——併入 any_rx 才能單獨反映真正的
            # RF 靜默(如死角/斷線)，跟上面的 CRC 錯誤率(解碼品質)分開看。step 依此鏈路的
            # 排程正規化(433 每 3 個 tick 才發一次是設計行為，不是丟包)。
            any_rx = sorted(evs + bad_evs, key=lambda e: e["t"])
            if len(any_rx) >= 2:
                loss = calc_seq_loss([e["seq"] for e in any_rx], step=SEQ_STEP[link])
                add_check(name, "單鏈路 RF 到達率丟失", loss["ratio"] * 100.0,
                          SPEC_LIMITS["single_link_loss_max_ratio"] * 100.0, "%",
                          "<=", loss["ratio"] > SPEC_LIMITS["single_link_loss_max_ratio"] * 1.5,
                          loss["ratio"] > SPEC_LIMITS["single_link_loss_max_ratio"],
                          f"已依排程正規化(step={SEQ_STEP[link]})、CRC 錯誤也算「有到達」："
                          f"預期 {loss['expected']:.1f} 個時槽、遺失 {loss['lost']:.1f} 個"
                          f"（僅此鏈路，未計入另一鏈路補收；純 RF 靜默，非解碼品質）")
            else:
                add_check(name, "單鏈路 RF 到達率丟失", 0.0,
                          SPEC_LIMITS["single_link_loss_max_ratio"] * 100.0, "%",
                          "<=", False, False, f"樣本不足（{len(any_rx)} 筆）")

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

        # --- 合併「到達率」丟失（雙鏈路去重後、CRC 好壞都算到達）：地面站「有沒有收到訊號」
        # 的完整度，是本工具最關鍵的 GO/NO-GO 指標——即使某一鏈路整個掛掉，只要另一鏈路頂住，
        # 這裡就該還在低位。step=1，因為 920 名義上每個全域 tick 都會發，兩鏈路合併後的涵蓋
        # 範圍就是「有沒有漏掉任何一個 100ms tick」，跟 433 自己的降速排程無關。
        any_rx_all = sorted(pkt_events + crc_bad_events, key=lambda e: e["t"])
        merged_any = merge_dedupe_events(any_rx_all)
        combined_loss = calc_seq_loss([e["seq"] for e in merged_any], step=1)
        add_check("封包完整性（雙鏈路合併）", "合併到達率丟失 (Union Loss)", combined_loss["ratio"] * 100.0,
                  SPEC_LIMITS["combined_loss_max_ratio"] * 100.0, "%",
                  "<=", combined_loss["ratio"] > SPEC_LIMITS["combined_loss_max_ratio"] * 1.5,
                  combined_loss["ratio"] > SPEC_LIMITS["combined_loss_max_ratio"],
                  f"去重後預期 {combined_loss['expected']:.1f} 個時槽、仍遺失 {combined_loss['lost']:.1f} 個"
                  f"（兩鏈路都完全靜默才算真正遺失；CRC 錯誤也算到達，不計入此項）")

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
        time_basis = "SD 卡 rx_utc_ms（GPS 紀律牆鐘，最準確）" if csv_mode else \
            "即時 console 列印到達時間（含序列埠/列印延遲，僅供參考）"
        add_check("紀錄管線 (Recording Pipeline)", "平均封包落地間隔", gaps["mean_ms"], 0.0, "ms",
                  ">", False, False, f"時間基準：{time_basis}；共 {gaps['count']} 個間隔樣本")
        add_check("紀錄管線 (Recording Pipeline)", "最大停頓 (Stall)", gaps["max_ms"],
                  SPEC_LIMITS["pipeline_stall_warn_ms"], "ms",
                  "<=", gaps["max_ms"] > SPEC_LIMITS["pipeline_stall_fail_ms"],
                  gaps["max_ms"] > SPEC_LIMITS["pipeline_stall_warn_ms"],
                  "疑似對應 Flash 用前才擦的 sector erase 阻塞（每 30+ 筆一次，約 400ms）"
                  if gaps["max_ms"] > SPEC_LIMITS["pipeline_stall_warn_ms"] else "無明顯停頓")
        add_check("紀錄管線 (Recording Pipeline)", "停頓次數比例", gaps["stall_ratio"] * 100.0,
                  SPEC_LIMITS["pipeline_stall_ratio_max"] * 100.0, "%",
                  "<=", gaps["stall_ratio"] > SPEC_LIMITS["pipeline_stall_ratio_max"] * 2.0,
                  gaps["stall_ratio"] > SPEC_LIMITS["pipeline_stall_ratio_max"],
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
                "data_source": "CSV (SD 卡離線)" if csv_mode else "即時 Console 擷取",
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
            "1. **總通訊頻率遠低於排定值(920≈10Hz/433≈3.33Hz)**：RF 前端可能根本沒收到訊號"
            "（天線/接線/供電/模組未初始化），先查這個再查 CRC，順序不能反。",
            "2. **總通訊頻率接近排定值，但 CRC 錯誤率偏高**：代表 RF 前端有收到東西、只是解不出來——"
            "檢查天線接頭/駐波、RF 參數(SF/BW/CR 兩端須一致)，或空中速率與雜訊環境不符。",
            "3. **Resync 比例偏高（433）**：多半是空中位元速率不符或強雜訊源干擾，非單純距離問題。",
            "4. **單鏈路 RF 到達率丟失偏高**：已扣掉 433 每 3 個時槽才發一次的正常降速排程，"
            "剩下的才是真正的 RF 靜默，需查天線/距離/遮蔽。",
            "5. **合併到達率丟失偏高但單鏈路正常**：檢查是否兩鏈路收到的其實是同一時間窗（火箭端 TX 排程異常）。",
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
    seq，920 幾乎每個 tick 都發，433 每 LORA433_TX_EVERY(=3) 個 tick 才發一次——這是
    calc_seq_loss() 需要 step 正規化的原因，selftest 資料若不照這個排程生成就測不出問題。"""
    import random
    t = time.time() - 60.0
    gs_stat = {"hw433_ok": True, "hw920_ok": True, "raw433": 0, "ok433": 0, "crc433": 0, "rsync433": 0,
               "ok920": 0, "crc920": 0, "rsync920": 0, "pkts433": 0, "pkts920": 0}
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

        # 433：只在每 LORA433_TX_EVERY 個 tick 才發一次，稍高雜訊
        if (i % LORA433_TX_EVERY) == 0:
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
        # 離線 log 檔沒有真實到達時間戳，退而以行號合成遞增秒數（僅供計數/比率類指標使用，
        # 停頓偵測在此模式下不具意義，report 會標明資料來源避免誤讀）。
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
            ser = serial_link.open_serial(port, args.baud, timeout=0.5)
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

    res = engine.analyze(csv_mode=csv_mode)
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

    if res["summary"]["overall_status"] == "NOT_READY":
        sys.exit(2)


if __name__ == "__main__":
    main()
