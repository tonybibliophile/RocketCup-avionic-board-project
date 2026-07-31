#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lora_param_autotest — LoRa 參數自動掃描測試儀（433/920 雙鏈路）
===========================================================================
連上地面站板（ROLE_GROUND，須為 TX-capable 版：make flash-ground-tx），自動把
「目前參數」與「附近參數」逐一套用到雙端（本機地面站 + 靠 gs_lora_test.c 的
`tx <cmd>` 中繼到火箭），量測每組參數的實際收包狀況，找出堪用的組合：

  · 433 (E22)：只掃空速 air ±1 檔（頻率/功率固定），因為兩端必須完全一致，
    調太多檔位一次很容易兩端對不上、白白燒時間。
  · 920 (E80)：SF/BW/CR 在目前值附近各 ±1 檔組合，但只留下「預測封包頻率落在
    5–10Hz」的組合才實際上板測試（低於 5Hz 太慢、超過 10Hz 沒意義——韌體排程
    本來就是 10Hz 送一次，見 gs_lora_test.c LORA_TELEM_PERIOD_MS）。預測用的
    time-on-air 公式逐字搬自 lora_calc.h（與韌體 host 測試 tests/test_lora_calc.c
    同一份，避免另外發明公式跟韌體算出來的兜不起來）。

安全模型與 ground_gui.py 一致：是否真的會發射由板子韌體變體決定（RX-only 版一律
拒絕 `tx` 中繼，見 gs_lora_test.c: gs_tx_allowed()）。掃描開始前會先查詢角色，
非 TX-capable（tx!=1）直接中止（見 _run_sweep()），不再另外加一道軟體 LNA 鎖。

測試結束（或按下「停止」）一律會把兩端還原回掃描開始前讀到的原始參數，
不會讓現場留在某個中途試驗值上。

用法：
    python3 lora_param_autotest.py
"""

import os
import re
import sys
import queue
import threading
import time
from datetime import datetime
from collections import namedtuple


def install_and_import(package, import_name=None):
    if import_name is None:
        import_name = package
    try:
        __import__(import_name)
    except ImportError:
        print(f"[*] 偵測到未安裝 {package}，正在自動進行 pip 安裝...")
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", package])


install_and_import("pyserial", "serial")

import serial
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import serial_link

import gui_theme as gt

GS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPORTS_DIR = os.path.join(GS_DIR, "reports")

# ============================================================
#  LoRa 換算：逐字搬自 firmware/main_flight_code/Core/Inc/lora_calc.h
#  （地面站板 e80 airtime/e80 show 用同一份公式；這裡搬一份 Python 版純粹是為了
#   「不用真的上板燒測就能先篩出候選組合」，公式本身不能跟韌體那份分岔）。
# ============================================================
_BW_KHZ = {0x00: 8, 0x08: 10, 0x01: 16, 0x09: 21, 0x02: 31, 0x0A: 42,
           0x03: 63, 0x04: 125, 0x05: 250, 0x06: 500}


def bw_to_khz(bw_idx):
    return _BW_KHZ.get(bw_idx, 0)


def symbol_time_us(sf, bw_idx):
    bw_khz = bw_to_khz(bw_idx)
    if bw_khz == 0 or sf < 5 or sf > 12:
        return 0
    return (1 << sf) * 1000 // bw_khz


def ldro_required(sf, bw_idx):
    ts = symbol_time_us(sf, bw_idx)
    return 1 if (ts != 0 and ts >= 16000) else 0


def time_on_air_us(sf, bw_idx, cr, preamble, payload_len):
    ts = symbol_time_us(sf, bw_idx)
    if ts == 0 or cr < 1 or cr > 4:
        return 0
    de = ldro_required(sf, bw_idx)
    t_pre = ((preamble * 4 + 17) * ts) // 4
    num = 8 * payload_len - 4 * sf + 28 + 16
    den = 4 * (sf - 2 * de)
    ceil_term = 0
    if num > 0 and den > 0:
        ceil_term = (num + den - 1) // den
    payload_symb = 8 + ceil_term * (cr + 4)
    if payload_symb < 8:
        payload_symb = 8
    return t_pre + payload_symb * ts


# 韌體排程固定 10Hz 送一次（main.c LoRaTelemetry_Task，100ms 週期），空中時間再短
# 也不會送更快，故預測到達率恆為 min(10, 1000/airtime_ms)。
SCHED_HZ = 10.0


def predicted_hz(sf, bw_idx, cr, preamble, payload_len):
    toa_us = time_on_air_us(sf, bw_idx, cr, preamble, payload_len)
    if toa_us <= 0:
        return None
    return min(SCHED_HZ, 1000000.0 / toa_us)


# ============================================================
#  E22 暫存器字串表（逐字搬自 lora_e22.c s_air_rate_str / s_tx_power_str）
# ============================================================
E22_AIR_STR = ["0.3k", "1.2k", "2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"]
E22_PWR_STR = ["30dBm", "27dBm", "24dBm", "21dBm"]
E80_BW_ALLOWED = (3, 4, 5, 6)   # 對照 ground_gui.py E80_BW_OPTIONS 的實務可用範圍


def _idx_of(lst, val):
    try:
        return lst.index(val)
    except ValueError:
        return None


# ============================================================
#  韌體輸出行 regex
# ============================================================
RE_ROLE = re.compile(r'\[ROLE_ID\]\s+role=GROUND(?:\s+fw=\S+)?\s+tx=(\d)')
RE_E22_SHOW = re.compile(
    r'\[LORA433\] E22-400T30S \| Freq=(\d+)\.000MHz\(CH=(\d+)\)'
    r'(?: \| Power=(\S+) \| AirRate=(\S+))?')
RE_E80_PARAMS = re.compile(
    r'\[E80\] freq=(\d+) Hz\s+SF(\d+)\s+BW([\d.]+) kHz \(idx=(\d+)\)\s+'
    r'CR 4/(\d)\s+pwr=(-?\d+) dBm\s+pre=(\d+)')
RE_E80_AIRTIME = re.compile(
    r'\[E80\] payload=(\d+) B\s+airtime=([\d.]+) ms\s+~(\d+) bps')
RE_STATS_RESET = re.compile(r'\[STATS\] reset OK')
RE_STATS_HDR = re.compile(r'\[STATS\] --- (E22-433|E80-920) ---')


def re_ack(cmd_base):
    return re.compile(r'\[ACK\]\s+status:(OK|BADARG|REJECTED)\s+cmd:"' + re.escape(cmd_base) + r'"')


def parse_stats_block(lines):
    """解析一次 `stats` 印出的兩段 [STATS] 區塊，回傳 {'433': {...}, '920': {...}}。"""
    text = "\n".join(lines)
    parts = re.split(r'\[STATS\] --- (E22-433|E80-920) ---', text)
    out = {"433": {}, "920": {}}
    key_map = {"E22-433": "433", "E80-920": "920"}
    # parts = [pre, name1, body1, name2, body2, ...]
    i = 1
    while i + 1 < len(parts):
        name, body = parts[i], parts[i + 1]
        key = key_map.get(name)
        if key:
            out[key] = _parse_stat_body(body, is_920=(key == "920"))
        i += 2
    return out


def _parse_stat_body(body, is_920):
    d = {"pkt_ok": 0, "crc_err": 0, "rate": None, "elapsed_s": None,
         "rssi_avg": None, "rssi_min": None, "rssi_max": None,
         "snr_avg": None}
    m = re.search(r'pkt_ok=(\d+)\s+crc_err=(\d+)', body)
    if m:
        d["pkt_ok"], d["crc_err"] = int(m.group(1)), int(m.group(2))
    m = re.search(r'rate=([\d.]+) pkt/s\s+elapsed=(\d+)s', body)
    if m:
        d["rate"], d["elapsed_s"] = float(m.group(1)), int(m.group(2))
    m = re.search(r'RSSI: last=-?\d+ min=(-?\d+) max=(-?\d+) avg=(-?\d+) dBm', body)
    if m:
        d["rssi_min"], d["rssi_max"], d["rssi_avg"] = int(m.group(1)), int(m.group(2)), int(m.group(3))
    if is_920:
        m = re.search(r'SNR:\s+last=-?\d+ min=(-?\d+) max=(-?\d+) avg=(-?\d+)', body)
        if m:
            d["snr_avg"] = int(m.group(3))
    return d


TestPoint = namedtuple("TestPoint", "link label params")


class AutoTestApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RocketCom LoRa 參數自動掃描測試儀")
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry(f"{min(1180, sw - 60)}x{min(820, sh - 80)}")
        self.root.minsize(920, 620)
        self.root.configure(bg=gt.BG_ROOT)

        self.serial_port = tk.StringVar()
        self.baud_rate = tk.IntVar(value=serial_link.DEFAULT_BAUD)
        self.is_connected = False
        self.running = False
        self.ser = None
        self.rx_thread = None

        self.test_433_var = tk.BooleanVar(value=True)
        self.test_920_var = tk.BooleanVar(value=True)
        self.window_s_var = tk.IntVar(value=20)

        self.ui_queue = queue.Queue()
        self._lines = []          # [(monotonic_t, line)]
        self._lines_lock = threading.Lock()
        self._parse_pos = 0

        self._stop_event = threading.Event()
        self._worker_thread = None
        self._row_index = {}      # (link, label) -> treeview item id

        gt.setup_styles()
        self.build_ui()
        self.scan_ports()
        self.root.after(10, self.poll_ui_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        gt.install_signal_handlers(self.root, self.on_close)

    # ------------------------------------------------------------------ UI
    def build_ui(self):
        top = tk.Frame(self.root, bg=gt.BG_ROOT)
        top.pack(fill=tk.X, padx=10, pady=(8, 4))

        conn = tk.Frame(top, bg=gt.BG_ROOT)
        conn.pack(side=tk.LEFT)
        tk.Label(conn, text="PORT:", bg=gt.BG_ROOT, fg=gt.TXT_DIM, font=gt.F_BOX_LABEL).pack(side=tk.LEFT, padx=2)
        self.port_combo = ttk.Combobox(conn, width=22, font=gt.F_HINT, textvariable=self.serial_port)
        self.port_combo.pack(side=tk.LEFT, padx=2)
        ttk.Button(conn, text="🔄", width=3, command=self.scan_ports).pack(side=tk.LEFT, padx=2)
        tk.Label(conn, text="BAUD:", bg=gt.BG_ROOT, fg=gt.TXT_DIM, font=gt.F_BOX_LABEL).pack(side=tk.LEFT, padx=(8, 2))
        ttk.Combobox(conn, values=[9600, 38400, 115200, 460800], width=8,
                     font=gt.F_HINT, textvariable=self.baud_rate).pack(side=tk.LEFT, padx=2)
        self.btn_connect = ttk.Button(conn, text="CONNECT", width=10, command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=8)

        self.lbl_role = tk.Label(top, text="❓ 角色/TX 能力未知", bg=gt.BG_ROOT, fg=gt.TXT_MUTED,
                                  font=gt.F_BOX_LABEL)
        self.lbl_role.pack(side=tk.LEFT, padx=12)

        # ------ 測試選項列 ------
        opts = tk.Frame(self.root, bg=gt.BG_ROOT)
        opts.pack(fill=tk.X, padx=10, pady=(0, 4))
        tk.Checkbutton(opts, text="測試 433 (E22 空速 ±1 檔)", variable=self.test_433_var,
                       bg=gt.BG_ROOT, fg=gt.CYAN, selectcolor=gt.BG_ROOT, font=gt.F_BOX_LABEL
                       ).pack(side=tk.LEFT, padx=4)
        tk.Checkbutton(opts, text="測試 920 (E80 SF/BW/CR 附近，篩 5–10Hz)", variable=self.test_920_var,
                       bg=gt.BG_ROOT, fg=gt.CYAN, selectcolor=gt.BG_ROOT, font=gt.F_BOX_LABEL
                       ).pack(side=tk.LEFT, padx=4)
        tk.Label(opts, text="每組觀察秒數:", bg=gt.BG_ROOT, fg=gt.TXT_DIM, font=gt.F_BOX_LABEL
                 ).pack(side=tk.LEFT, padx=(16, 2))
        tk.Spinbox(opts, from_=5, to=120, textvariable=self.window_s_var, width=5,
                   font=gt.F_HINT).pack(side=tk.LEFT, padx=2)

        self.btn_start = gt.StyledButton(opts, text="▶ 開始自動掃描", command=self.start_sweep,
                                          bg="#0a5c36", hover_bg="#0f8a4f", font=gt.F_BOX_LABEL,
                                          padx=10, pady=4)
        self.btn_start.pack(side=tk.RIGHT, padx=4)
        self.btn_stop = gt.StyledButton(opts, text="■ 停止並還原", command=self.stop_sweep,
                                         bg="#7a1414", hover_bg="#b91c1c", font=gt.F_BOX_LABEL,
                                         padx=10, pady=4)
        self.btn_stop.pack(side=tk.RIGHT, padx=4)
        self.btn_stop.config_state(tk.DISABLED)
        gt.StyledButton(opts, text="💾 存報告", command=self.save_report,
                         bg="#00435a", hover_bg="#005f7f", font=gt.F_BOX_LABEL,
                         padx=10, pady=4).pack(side=tk.RIGHT, padx=4)

        # ------ 主體：結果表 + log ------
        main_pane = tk.PanedWindow(self.root, orient=tk.VERTICAL, bg=gt.BG_ROOT,
                                    sashwidth=6, sashrelief="flat", bd=0)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=6)

        table_frame = tk.Frame(main_pane, bg=gt.BG_PANEL)
        gt.section_title(table_frame, "測試結果（依送出順序；BASELINE=掃描前原始值）")
        cols = ("link", "param", "status", "rate", "pred", "pkt_ok", "crc_err", "rssi", "snr", "note")
        headers = {"link": "鏈路", "param": "參數", "status": "狀態", "rate": "實測Hz",
                   "pred": "預測Hz", "pkt_ok": "pkt_ok", "crc_err": "crc_err",
                   "rssi": "RSSI avg", "snr": "SNR avg", "note": "備註"}
        widths = {"link": 50, "param": 190, "status": 60, "rate": 70, "pred": 70,
                  "pkt_ok": 60, "crc_err": 60, "rssi": 70, "snr": 60, "note": 220}
        self.tree = ttk.Treeview(table_frame, columns=cols, show="headings", height=12)
        for c in cols:
            self.tree.heading(c, text=headers[c])
            self.tree.column(c, width=widths[c], anchor="center" if c != "note" else "w")
        self.tree.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        main_pane.add(table_frame, height=280, minsize=160)

        log_frame = tk.Frame(main_pane, bg=gt.BG_PANEL)
        gt.section_title(log_frame, "執行紀錄")
        self.console = ScrolledText(log_frame, bg=gt.BG_CONSOLE, fg="#33ff33",
                                     insertbackground="white", font=gt.F_MONO,
                                     borderwidth=0, highlightthickness=0)
        self.console.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        gt.config_console_tags(self.console, {
            "INFO": dict(foreground=gt.TXT_MUTED), "CMD": dict(foreground=gt.CYAN),
            "OK": dict(foreground=gt.GREEN), "WARN": dict(foreground=gt.YELLOW),
            "ERR": dict(foreground=gt.RED_ALT), "RAW": dict(foreground="#666666"),
        })
        main_pane.add(log_frame, minsize=160)

    # ------------------------------------------------------------------ 連線
    def scan_ports(self):
        ports = serial_link.list_candidate_ports()
        self.port_combo['values'] = ports
        if ports:
            pref = [p for p in ports if "usbserial" in p or "usbmodem" in p or "tty.usb" in p]
            self.serial_port.set(pref[0] if pref else ports[0])

    def toggle_connection(self):
        if not self.is_connected:
            port = self.serial_port.get()
            if not port:
                messagebox.showerror("錯誤", "未偵測到任何可用序列埠！")
                return
            try:
                self.ser = serial.Serial(port, self.baud_rate.get(), timeout=0.1)
                self.is_connected = True
                self.running = True
                self.btn_connect.config(text="DISCONNECT")
                self.rx_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
                self.rx_thread.start()
                self._log(f"連線開啟 {port}", "INFO")
                self.root.after(300, lambda: self._raw_send("role"))
            except Exception as e:
                messagebox.showerror("連線失敗", f"無法開啟 {port}: {e}")
        else:
            self.disconnect_serial()

    def disconnect_serial(self):
        self.is_connected = False
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.btn_connect.config(text="CONNECT")
        self._log("序列埠已中斷", "INFO")
        self.lbl_role.config(text="❓ 角色/TX 能力未知", fg=gt.TXT_MUTED)

    def _serial_read_loop(self):
        buf = b""
        while self.running:
            try:
                if self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting)
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode("utf-8", errors="replace").strip()
                        if not line:
                            continue
                        now = time.monotonic()
                        with self._lines_lock:
                            self._lines.append((now, line))
                        self.ui_queue.put(("raw", line))
                        if "role=GROUND" in line:
                            m = RE_ROLE.search(line)
                            if m:
                                self.ui_queue.put(("role", int(m.group(1))))
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.ui_queue.put(("log", (f"讀取異常中斷: {e}", "ERR")))
                break

    # ------------------------------------------------------------------ UI 執行緒佇列消費
    def poll_ui_queue(self):
        n = 0
        while not self.ui_queue.empty() and n < 200:
            kind, payload = self.ui_queue.get()
            n += 1
            if kind == "raw":
                tag = ("OK" if "[ACK] status:OK" in payload else
                       "ERR" if ("[ACK] status:REJECTED" in payload or "REJECTED" in payload) else
                       "WARN" if "[ACK] status:BADARG" in payload else "RAW")
                gt.append_console(self.console, payload + "\n", tag)
            elif kind == "log":
                text, tag = payload
                gt.append_console(self.console, f"[{datetime.now().strftime('%H:%M:%S')}] {text}\n", tag)
            elif kind == "role":
                tx = payload
                if tx == 1:
                    self.lbl_role.config(text="📡 GROUND / TX-capable", fg=gt.RED_ALT)
                else:
                    self.lbl_role.config(text="🔒 GROUND / RX-only（無法中繼到火箭，掃描會被拒）", fg=gt.YELLOW)
            elif kind == "row_new":
                key, values = payload
                item = self.tree.insert("", tk.END, values=values)
                self._row_index[key] = item
            elif kind == "row_update":
                key, values = payload
                item = self._row_index.get(key)
                if item is not None:
                    self.tree.item(item, values=values)
            elif kind == "sweep_done":
                self.btn_start.config_state(tk.NORMAL)
                self.btn_stop.config_state(tk.DISABLED)
        try:
            if self.root.winfo_exists():
                self.root.after(30, self.poll_ui_queue)
        except tk.TclError:
            pass

    def _log(self, text, tag="INFO"):
        self.ui_queue.put(("log", (text, tag)))

    # ------------------------------------------------------------------ 背景執行緒用的底層送出/等待
    def _raw_send(self, cmd_str):
        """逐字元慢送（板端命令台 20ms 輪詢、1 byte 緩衝，見 gui_theme.CommandSender 說明），
        直接寫 serial（不經 Tkinter），因為這個工具的送出動作全部發生在背景執行緒。"""
        if not self.ser or not self.running:
            return False
        if not cmd_str.endswith('\n'):
            cmd_str += '\n'
        data = ('\n' + cmd_str).encode('utf-8')   # 前置換行先終止殘留半截命令
        self.ui_queue.put(("log", (f"➡ {cmd_str.strip()}", "CMD")))
        for i in range(len(data)):
            if self._stop_event.is_set() and not getattr(self, "_restoring", False):
                return False
            try:
                self.ser.write(data[i:i + 1])
                self.ser.flush()
            except Exception as e:
                self._log(f"串口寫入失敗: {e}", "ERR")
                return False
            time.sleep(0.1)
        return True

    def _wait_for(self, pattern, timeout):
        """從共用游標往後找第一個符合 pattern 的行；逾時回傳 None。不論有沒有找到，
        游標都會前進到目前緩衝區尾端，避免下一次呼叫誤配到「已經看過但這次沒中」的舊行
        （例如本地 ACK 跟稍後的火箭端 ACK 文字完全相同，靠游標區分先後）。"""
        rx = re.compile(pattern)
        deadline = time.monotonic() + timeout
        while True:
            with self._lines_lock:
                n = len(self._lines)
                start = self._parse_pos
                for i in range(start, n):
                    m = rx.search(self._lines[i][1])
                    if m:
                        self._parse_pos = i + 1
                        return self._lines[i][1]
                self._parse_pos = n
            still_stopping = self._stop_event.is_set() and not getattr(self, "_restoring", False)
            if still_stopping or time.monotonic() >= deadline:
                return None
            time.sleep(0.1)

    def _snapshot_pos(self):
        with self._lines_lock:
            return self._parse_pos

    def _lines_since(self, start_pos):
        with self._lines_lock:
            end = len(self._lines)
            out = [self._lines[i][1] for i in range(start_pos, end)]
            self._parse_pos = end
        return out

    # ------------------------------------------------------------------ 查詢
    def query_role(self, timeout=5.0):
        self._raw_send("role")
        line = self._wait_for(RE_ROLE, timeout)
        if not line:
            return None
        m = RE_ROLE.search(line)
        return int(m.group(1)) if m else None

    def query_e22(self, timeout=5.0):
        self._raw_send("e22 show")
        line = self._wait_for(RE_E22_SHOW, timeout)
        if not line:
            return None
        m = RE_E22_SHOW.search(line)
        freq_mhz, ch = int(m.group(1)), int(m.group(2))
        pwr_str, air_str = m.group(3), m.group(4)
        air_idx = _idx_of(E22_AIR_STR, air_str) if air_str else None
        pwr_idx = _idx_of(E22_PWR_STR, pwr_str) if pwr_str else None
        if air_idx is None or pwr_idx is None:
            self._log("⚠ e22 show 未回讀到暫存器（模組可能未上線），無法掃描 433", "WARN")
            return None
        return {"freq_mhz": freq_mhz, "ch": ch, "pwr_idx": pwr_idx, "air_idx": air_idx}

    def query_e80(self, timeout=5.0):
        self._raw_send("e80 show")
        line1 = self._wait_for(RE_E80_PARAMS, timeout)
        if not line1:
            return None
        m = RE_E80_PARAMS.search(line1)
        freq_hz, sf, bw_khz, bw_idx, cr, pwr, pre = (
            int(m.group(1)), int(m.group(2)), float(m.group(3)), int(m.group(4)),
            int(m.group(5)), int(m.group(6)), int(m.group(7)))
        line2 = self._wait_for(RE_E80_AIRTIME, 3.0)
        payload_len, airtime_ms = 115, 0.0
        if line2:
            m2 = RE_E80_AIRTIME.search(line2)
            payload_len, airtime_ms = int(m2.group(1)), float(m2.group(2))
        return {"freq_hz": freq_hz, "sf": sf, "bw_idx": bw_idx, "cr": cr,
                "pwr_dbm": pwr, "pre": pre, "payload_len": payload_len, "airtime_ms": airtime_ms}

    def reset_stats(self):
        self._raw_send("stats reset")
        self._wait_for(RE_STATS_RESET, 5.0)

    def read_stats(self):
        start = self._snapshot_pos()
        self._raw_send("stats")
        time.sleep(1.5)
        lines = self._lines_since(start)
        return parse_stats_block(lines)

    # ------------------------------------------------------------------ 套用參數（本地 + 中繼火箭）
    def _apply_local_and_relay(self, cmd_base, cmd_full):
        """cmd_full 例：'e22 air 3' / 'e80 sf 10'；cmd_base 為 ACK 比對用的前綴（無參數值）。"""
        self._raw_send(cmd_full)
        line = self._wait_for(re_ack(cmd_base), 5.0)
        if not line:
            self._log(f"⚠ 本地 '{cmd_full}' 未見 ACK（逾時，仍繼續）", "WARN")
        elif "status:OK" not in line:
            self._log(f"⚠ 本地 '{cmd_full}' 回應非 OK: {line}", "WARN")

        self._raw_send(f"tx {cmd_full}")
        line = self._wait_for(re_ack(cmd_base), 15.0)
        if not line:
            self._log(f"⚠ 火箭端 '{cmd_full}' 未收到 ACK（433 上行/下行可能已中斷），仍繼續量測", "WARN")
        elif "status:OK" not in line:
            self._log(f"⚠ 火箭端 '{cmd_full}' 回應非 OK: {line}", "WARN")
        else:
            self._log(f"✓ 火箭端確認 '{cmd_full}'", "OK")
        return True

    def apply_433(self, air_idx):
        return self._apply_local_and_relay("e22 air", f"e22 air {air_idx}")

    def apply_920(self, applied, sf, bw_idx, cr):
        ok = True
        if sf != applied.get("sf"):
            ok = self._apply_local_and_relay("e80 sf", f"e80 sf {sf}") and ok
        if not self._stop_event.is_set() and bw_idx != applied.get("bw_idx"):
            ok = self._apply_local_and_relay("e80 bw", f"e80 bw {bw_idx}") and ok
        if not self._stop_event.is_set() and cr != applied.get("cr"):
            ok = self._apply_local_and_relay("e80 cr", f"e80 cr {cr}") and ok
        applied.update(sf=sf, bw_idx=bw_idx, cr=cr)
        return ok

    # ------------------------------------------------------------------ 候選組合
    @staticmethod
    def build_433_candidates(baseline):
        cur = baseline["air_idx"]
        cands = sorted({c for c in (cur - 1, cur, cur + 1) if 0 <= c <= 7})
        return cands

    @staticmethod
    def build_920_candidates(baseline):
        sf0, bw0, cr0 = baseline["sf"], baseline["bw_idx"], baseline["cr"]
        pre, payload_len = baseline["pre"], baseline["payload_len"]
        sf_c = [v for v in (sf0 - 1, sf0, sf0 + 1) if 7 <= v <= 12]
        bw_c = [v for v in (bw0 - 1, bw0, bw0 + 1) if v in E80_BW_ALLOWED]
        cr_c = [v for v in (cr0 - 1, cr0, cr0 + 1) if 1 <= v <= 4]
        combos = []
        seen = set()
        for sf in sf_c:
            for bw in bw_c:
                for cr in cr_c:
                    hz = predicted_hz(sf, bw, cr, pre, payload_len)
                    if hz is None:
                        continue
                    is_baseline = (sf, bw, cr) == (sf0, bw0, cr0)
                    if not (5.0 - 1e-6 <= hz <= 10.0 + 1e-6) and not is_baseline:
                        continue
                    key = (sf, bw, cr)
                    if key in seen:
                        continue
                    seen.add(key)
                    combos.append({"sf": sf, "bw_idx": bw, "cr": cr, "pred_hz": hz,
                                   "is_baseline": is_baseline})
        # baseline 排最前面方便當對照組，其餘依預測 Hz 由高到低（Hz 越高＝越接近 10Hz 上限，
        # 越省下行頻寬餘裕；使用者仍可從結果表自行挑更看重靈敏度/SF 的組合）。
        combos.sort(key=lambda c: (not c["is_baseline"], -c["pred_hz"]))
        return combos

    # ------------------------------------------------------------------ 掃描主流程
    def start_sweep(self):
        if not self.is_connected:
            messagebox.showwarning("提示", "請先連線地面站板序列埠。")
            return
        if not self.test_433_var.get() and not self.test_920_var.get():
            messagebox.showwarning("提示", "至少要勾選 433 或 920 其中一項。")
            return
        self._stop_event.clear()
        self.tree.delete(*self.tree.get_children())
        self._row_index.clear()
        self._sweep_results = []
        self._sweep_baseline = {}
        self.btn_start.config_state(tk.DISABLED)
        self.btn_stop.config_state(tk.NORMAL)

        self._worker_thread = threading.Thread(target=self._sweep_worker, daemon=True)
        self._worker_thread.start()

    def stop_sweep(self):
        self._log("使用者要求停止，正在還原參數…", "WARN")
        self._stop_event.set()

    def _add_row(self, link, label, status="待測", rate="--", pred="--", pkt_ok="--",
                 crc_err="--", rssi="--", snr="--", note=""):
        key = (link, label)
        values = (link, label, status, rate, pred, pkt_ok, crc_err, rssi, snr, note)
        if key in self._row_index:
            self.ui_queue.put(("row_update", (key, values)))
        else:
            self.ui_queue.put(("row_new", (key, values)))

    def _sweep_worker(self):
        try:
            self._run_sweep()
        except Exception as e:
            self._log(f"掃描發生例外中止: {e}", "ERR")
        finally:
            self.ui_queue.put(("sweep_done", None))

    def _run_sweep(self):
        self._log("=== 查詢角色 / TX 能力 ===", "INFO")
        tx = self.query_role()
        if tx != 1:
            self._log("此地面站板非 TX-capable（RX-only 或無回應），無法用 tx 中繼到火箭，中止掃描。"
                       "請改燒 make flash-ground-tx 版本。", "ERR")
            return

        self._log("=== 查詢目前 (BASELINE) 參數 ===", "INFO")
        want_433 = self.test_433_var.get()
        want_920 = self.test_920_var.get()
        e22_base = self.query_e22() if want_433 else None
        e80_base = self.query_e80() if want_920 else None
        if want_433 and e22_base is None:
            self._log("433 基準參數查詢失敗，本次略過 433 掃描。", "WARN")
            want_433 = False
        if want_920 and e80_base is None:
            self._log("920 基準參數查詢失敗，本次略過 920 掃描。", "WARN")
            want_920 = False
        if not want_433 and not want_920:
            self._log("沒有可掃描的鏈路，中止。", "ERR")
            return

        window_s = max(5, int(self.window_s_var.get()))
        self._sweep_baseline = {"e22": e22_base, "e80": e80_base, "window_s": window_s}

        plan_433 = self.build_433_candidates(e22_base) if want_433 else []
        plan_920 = self.build_920_candidates(e80_base) if want_920 else []

        if want_433:
            self._log(f"433 候選 air 檔位: {plan_433}（目前={e22_base['air_idx']}）", "INFO")
            for air in plan_433:
                tag = "BASELINE" if air == e22_base["air_idx"] else ""
                self._add_row("433", f"air={air} ({E22_AIR_STR[air]})", note=tag)
        if want_920:
            self._log(f"920 候選組合數: {len(plan_920)}（篩選條件：預測到達率落在 5–10Hz）", "INFO")
            for c in plan_920:
                label = f"SF{c['sf']}/BW{bw_to_khz(c['bw_idx'])}k/CR4-{c['cr']+4}"
                tag = "BASELINE" if c["is_baseline"] else ""
                self._add_row("920", label, pred=f"{c['pred_hz']:.1f}", note=tag)

        eta_s = window_s * (len(plan_433) + len(plan_920)) + 10 * (len(plan_433) + 3 * len(plan_920))
        self._log(f"預估總耗時約 {eta_s // 60} 分 {eta_s % 60} 秒（不含中途 ACK 逾時）。", "INFO")

        results = []

        # ---- 433 ----
        if want_433:
            for air in plan_433:
                if self._stop_event.is_set():
                    break
                label = f"air={air} ({E22_AIR_STR[air]})"
                self._log(f"--- 433 測試 {label} ---", "INFO")
                self._add_row("433", label, status="設定中")
                if not self.apply_433(air):
                    break
                self._add_row("433", label, status="量測中")
                self.reset_stats()
                self._sleep_countdown(window_s, f"433 {label}")
                stat = self.read_stats()["433"]
                row = self._format_row("433", label, stat, pred_hz=None,
                                        is_baseline=(air == e22_base["air_idx"]))
                results.append(row)
                self._add_row(*row["row_args"], **row["row_kwargs"])

        # ---- 920 ----
        if want_920 and not self._stop_event.is_set():
            applied = {"sf": e80_base["sf"], "bw_idx": e80_base["bw_idx"], "cr": e80_base["cr"]}
            for c in plan_920:
                if self._stop_event.is_set():
                    break
                label = f"SF{c['sf']}/BW{bw_to_khz(c['bw_idx'])}k/CR4-{c['cr']+4}"
                self._log(f"--- 920 測試 {label}（預測 {c['pred_hz']:.1f} Hz）---", "INFO")
                self._add_row("920", label, status="設定中", pred=f"{c['pred_hz']:.1f}")
                if not self.apply_920(applied, c["sf"], c["bw_idx"], c["cr"]):
                    break
                self._add_row("920", label, status="量測中", pred=f"{c['pred_hz']:.1f}")
                self.reset_stats()
                self._sleep_countdown(window_s, f"920 {label}")
                stat = self.read_stats()["920"]
                row = self._format_row("920", label, stat, pred_hz=c["pred_hz"],
                                        is_baseline=c["is_baseline"])
                results.append(row)
                self._add_row(*row["row_args"], **row["row_kwargs"])

        self._sweep_results = results

        # ---- 還原 ----
        self._log("=== 還原掃描前原始參數 ===", "INFO")
        self._restoring = True
        try:
            if want_433:
                self.apply_433(e22_base["air_idx"])
            if want_920:
                applied = {"sf": None, "bw_idx": None, "cr": None}   # 強制全送，確保真的回到原值
                self.apply_920(applied, e80_base["sf"], e80_base["bw_idx"], e80_base["cr"])
        finally:
            self._restoring = False

        if self._stop_event.is_set():
            self._log("=== 掃描已停止，參數已還原 ===", "WARN")
        else:
            self._log("=== 掃描完成，參數已還原 ===", "OK")
        self._write_report_file(e22_base, e80_base, results, window_s)

    def _sleep_countdown(self, seconds, label):
        end = time.monotonic() + seconds
        last_logged = None
        while time.monotonic() < end and not self._stop_event.is_set():
            remain = int(end - time.monotonic())
            if remain != last_logged and remain % 5 == 0 and remain > 0:
                last_logged = remain
                self._log(f"觀察中… {label}，剩餘 {remain}s", "INFO")
            time.sleep(0.2)

    def _format_row(self, link, label, stat, pred_hz, is_baseline):
        pkt_ok = stat.get("pkt_ok", 0)
        crc_err = stat.get("crc_err", 0)
        rate = stat.get("rate")
        note_bits = []
        if is_baseline:
            note_bits.append("BASELINE")
        if pkt_ok == 0 and crc_err == 0:
            note_bits.append("⚠ 完全沒收到封包，檢查鏈路是否中斷")
        elif crc_err > 0 and pkt_ok > 0 and crc_err / (pkt_ok + crc_err) > 0.1:
            note_bits.append("⚠ CRC 錯誤率偏高")
        return {
            "row_args": (link, label),
            "row_kwargs": dict(
                status="完成",
                rate=f"{rate:.1f}" if rate else "--",
                pred=f"{pred_hz:.1f}" if pred_hz is not None else "--",
                pkt_ok=pkt_ok, crc_err=crc_err,
                rssi=stat.get("rssi_avg", "--") if stat.get("rssi_avg") is not None else "--",
                snr=stat.get("snr_avg", "--") if stat.get("snr_avg") is not None else "--",
                note=" / ".join(note_bits),
            ),
            "raw": stat,
        }

    # ------------------------------------------------------------------ 報告輸出
    def _write_report_file(self, e22_base, e80_base, results, window_s):
        os.makedirs(REPORTS_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(REPORTS_DIR, f"lora_param_autotest_{ts}.md")
        lines = ["# LoRa 參數自動掃描報告", "", f"時間: {datetime.now().isoformat(timespec='seconds')}",
                  f"每組觀察秒數: {window_s}", ""]
        if e22_base:
            lines += [f"## 433 (E22) BASELINE", "",
                      f"freq={e22_base['freq_mhz']}MHz  pwr_idx={e22_base['pwr_idx']}"
                      f"({E22_PWR_STR[e22_base['pwr_idx']]})  air_idx={e22_base['air_idx']}"
                      f"({E22_AIR_STR[e22_base['air_idx']]})", ""]
        if e80_base:
            lines += [f"## 920 (E80) BASELINE", "",
                      f"freq={e80_base['freq_hz']}Hz  SF{e80_base['sf']}  "
                      f"BW{bw_to_khz(e80_base['bw_idx'])}kHz  CR4/{e80_base['cr']+4}  "
                      f"pwr={e80_base['pwr_dbm']}dBm  pre={e80_base['pre']}  "
                      f"payload={e80_base['payload_len']}B  airtime={e80_base['airtime_ms']:.1f}ms", ""]
        lines += ["## 結果", "",
                  "| 鏈路 | 參數 | 實測Hz | 預測Hz | pkt_ok | crc_err | RSSI avg | SNR avg | 備註 |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for r in results:
            link, label = r["row_args"]
            kw = r["row_kwargs"]
            lines.append(f"| {link} | {label} | {kw['rate']} | {kw['pred']} | {kw['pkt_ok']} | "
                         f"{kw['crc_err']} | {kw['rssi']} | {kw['snr']} | {kw['note']} |")
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        self._log(f"報告已存到 {path}", "OK")
        self._last_report_path = path

    def save_report(self):
        results = getattr(self, "_sweep_results", None)
        if not results:
            messagebox.showinfo("提示", "尚未有掃描結果可存（跑一次自動掃描後會自動存檔，也可再按此鈕另存一份）。")
            return
        e22_base = self._sweep_baseline.get("e22")
        e80_base = self._sweep_baseline.get("e80")
        window_s = self._sweep_baseline.get("window_s", self.window_s_var.get())
        self._write_report_file(e22_base, e80_base, results, window_s)

    # ------------------------------------------------------------------
    def on_close(self):
        self._stop_event.set()
        self.disconnect_serial()
        self.root.destroy()


def main():
    root = tk.Tk()
    AutoTestApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
