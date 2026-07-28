#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ground_gui — 地面站板監測儀（ROLE_GROUND，經 USB 直連地面站板，非直連航電）。

地面站板是雙鏈路 LoRa（433MHz E22 / 920MHz E80）接收端：它把火箭下行的二進制遙測解碼後
以 `[GS_PKT]`/`[GS_STAT]`/`[GS_GPS]`/`[ACK]` 等文字行印到自己的 USB 序列埠，本 GUI 就是接
在那個序列埠上的監看＋操作台。航電板直連除錯請用 GUI_avionic.py；旗艦儀表板見 gui_monitor.py。

視覺語彙、StyledButton、卡片/徽章/高亮框工廠、深色 matplotlib 配方全部來自
`gui_theme.py`（抽自 gui_monitor.py，同一套配色不重新發明）。

控制能力：ARM/DISARM、手動開傘（副傘/主傘/雙傘）、落海回收確認、BENCH 桌面測試——
這些指令送到「地面站板」自己的文字命令台後，由地面站板轉成 433 上行二進制幀打給火箭
（見 firmware gs_lora_test.c: uplink_send()），不需要韌體改動。真正會發射 RF 的正是這支
GUI，所以沿用 gui_monitor 的「發射鎖」：未勾選「已確認未安裝 LNA」前，所有指令一律擋下——
天線若裝有 LNA（僅供接收增益），直接發射會讓 TX 功率回灌燒毀 LNA。
"""

import argparse
import os
import re
import sys
import time
import queue
import threading
from datetime import datetime
from collections import deque


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
install_and_import("matplotlib")

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import serial_link

import gui_theme as gt

DEFAULT_BAUD = serial_link.DEFAULT_BAUD
CHART_HIST_SEC = 60.0
LINK_HIST_LEN = 60


class GroundStationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RocketCom 地面站監測儀 (Ground Station Monitor)")
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry(f"{min(1620, sw - 60)}x{min(980, sh - 80)}")
        self.root.minsize(1150, 760)
        self.root.configure(bg=gt.BG_ROOT)

        self.serial_port = tk.StringVar()
        self.baud_rate = tk.IntVar(value=DEFAULT_BAUD)
        self.save_log_var = tk.BooleanVar(value=False)
        self.lna_lock_var = tk.BooleanVar(value=False)
        self.is_connected = False
        self.running = False
        self.ser = None
        self.rx_thread = None
        self.log_file = None
        self.data_queue = queue.Queue()

        self.fsm_state = None            # "STATE_xxx" 全名（由 fsm:%u 換算）
        self.rocket_state = {
            "seq": None, "fsm": 0, "alt_m": 0.0, "vf_alt_m": 0.0,
            "vz_ms": 0.0, "vf_vz_ms": 0.0, "baro_m": 0.0, "bat_v": 0.0,
            "gps_sats": 0, "gps_fix": 0, "lat": None, "lon": None, "gps_alt_m": 0,
            "accel_mag": 0.0,
            "peer_fsm": 0, "peer_flags": 0, "peer_h_m": 0.0, "peer_v_ms": 0.0,
            "peer_baro_m": 0.0, "peer_az_g": 0.0, "peer_vf_h_m": 0.0, "peer_vf_v_ms": 0.0,
            "peer_link": 0, "peer_loss_pct": 0.0, "peer_bench_arb": 0,
            "last_pkt_time": None,
        }
        self._seq_hist = {"433": deque(maxlen=40), "920": deque(maxlen=40)}
        self._last_seq = {"433": None, "920": None}
        self._loss_pct = {"433": 0.0, "920": 0.0}

        self.link_stats = {
            key: {"rssi": None, "snr": None, "last_pkt_time": None,
                  "rssi_hist": deque(maxlen=LINK_HIST_LEN), "snr_hist": deque(maxlen=LINK_HIST_LEN)}
            for key in ("433", "920")
        }

        self.gs_state = {
            "hw_433": "OFF", "hw_920": "OFF", "raw_433": 0,
            "ok_433": 0, "crc_433": 0, "rsync_433": 0,
            "ok_920": 0, "crc_920": 0, "rsync_920": 0,
            "pkts_433": 0, "pkts_920": 0, "crc_bad_433": 0, "crc_bad_920": 0,
            "gps_sats": 0, "gps_q": 0, "gps_ok": 0, "gps_err": 0, "gps_fix": False,
            "gps_lat": 0.0, "gps_lon": 0.0, "gps_alt": 0,
        }

        self.deploy_latch = gt.DeployLatch(on_change=self._refresh_deploy_label)
        self._uplink_seq_label = {}

        # 本機地面站板 TX 能力（None=偵測中, 0=RX-only, 1=TX-capable）——純顯示用徽章，
        # 兩種 firmware（make flash-ground / make flash-ground-tx）是否接受發射一律由板子
        # 自己判斷回應（gs_lora_test.c gs_tx_allowed()），GUI 不因此禁用 ARM/BENCH/DEPLOY
        # 等任何按鈕，一律照送、以 [UPLINK] 回應為準。
        self.gs_tx_capable = None
        self._role_query_attempts = 0
        self._DEPLOY_INFO = {
            "drogue": ("副傘 DROGUE", "deploy drogue"),
            "main":   ("主傘 MAIN", "deploy main"),
            "both":   ("副傘+主傘 BOTH", "deploy both"),
        }
        self._DEPLOY_LABEL_ZH = {
            "DEPLOY-DROGUE": "副傘 DROGUE", "DEPLOY-MAIN": "主傘 MAIN", "DEPLOY-BOTH": "副傘+主傘 BOTH",
        }

        self.chart_t0 = time.monotonic()
        self.chart_dirty = False
        self.gps_track = []
        self.gps_home = None

        _N = 4000
        self.ts_alt_ekf = deque(maxlen=_N)
        self.ts_alt_vf = deque(maxlen=_N)
        self.ts_alt_baro = deque(maxlen=_N)
        self.ts_vz_ekf = deque(maxlen=_N)
        self.ts_vz_vf = deque(maxlen=_N)
        self.ts_peer_alt = deque(maxlen=_N)
        self.ts_peer_vf_alt = deque(maxlen=_N)
        self.ts_peer_baro = deque(maxlen=_N)
        self.ts_peer_vz = deque(maxlen=_N)
        self.ts_peer_vf_vz = deque(maxlen=_N)

        self.cmd_sender = gt.CommandSender(
            get_ser=lambda: self.ser,
            get_running=lambda: self.running,
            console_echo=self.append_console,
            event_log=self._event_log,
            lock_var=self.lna_lock_var,
            lock_warning=("⚠️ 發射鎖：未確認 LNA 狀態",
                          "此地面站僅可在【未安裝 LNA】的情況下發送 LoRa 指令！\n\n"
                          "天線若裝有 LNA（低雜訊放大器，僅供接收增益用），直接發射會讓 "
                          "TX 功率回灌，燒毀 LNA。\n\n"
                          "請確認天線端未安裝 LNA 後，勾選上方「🔓 已確認未安裝 LNA」再發送。"),
        )

        gt.setup_styles()
        self.build_ui()
        self.scan_ports()
        self.root.after(10, self.poll_queue)
        self.root.after(200, self.charts_redraw_loop)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------------
    def send_command(self, cmd_str):
        ok = self.cmd_sender.send(cmd_str, parent=self.root)
        return ok

    # ------------------------------------------------------------------ UI
    def build_ui(self):
        top_container = tk.Frame(self.root, bg=gt.BG_ROOT)
        top_container.pack(fill=tk.X, side=tk.TOP, padx=10, pady=(5, 0))

        # ── 第一排：火箭/副航電飛行狀態 + 雙鏈路硬體/收包/健康度 ──
        top_bar = tk.Frame(top_container, bg=gt.BG_ROOT)
        top_bar.pack(fill=tk.X, side=tk.TOP)

        self.lbl_fsm = tk.Label(top_bar, text="🚀 主航電: --", bg=gt.BG_ROOT, fg=gt.CYAN_HI,
                                 font=gt.F_STATE, width=24, anchor="w")
        self.lbl_fsm.pack(side=tk.LEFT, padx=4, pady=2)

        self.lbl_peer_fsm = tk.Label(top_bar, text="🛸 副航電: --", bg=gt.BG_ROOT, fg=gt.TXT_OFF,
                                      font=gt.F_TOPBAR, width=22, anchor="w")
        self.lbl_peer_fsm.pack(side=tk.LEFT, padx=4, pady=2)

        gt.vsep(top_bar)

        self.lbl_lora433 = tk.Label(top_bar, text="📻 433: --", bg=gt.BG_ROOT, fg=gt.TXT_OFF,
                                     font=gt.F_TOPBAR, width=12, anchor="w")
        self.lbl_lora433.pack(side=tk.LEFT, padx=4, pady=2)
        self.lbl_lora920 = tk.Label(top_bar, text="📡 920: --", bg=gt.BG_ROOT, fg=gt.TXT_OFF,
                                     font=gt.F_TOPBAR, width=12, anchor="w")
        self.lbl_lora920.pack(side=tk.LEFT, padx=4, pady=2)

        # State1(433)/State2(920) 收包閃燈——每條鏈路各自收到一筆封包才閃一下
        self.link_leds = {}
        for key, label in (("433", "State1"), ("920", "State2")):
            holder = tk.Frame(top_bar, bg=gt.BG_ROOT)
            holder.pack(side=tk.LEFT, padx=6)
            canvas = tk.Canvas(holder, width=14, height=14, bg=gt.BG_ROOT, highlightthickness=0)
            oval = canvas.create_oval(2, 2, 12, 12, fill=gt.SEP, outline="")
            canvas.pack(side=tk.LEFT)
            tk.Label(holder, text=label, bg=gt.BG_ROOT, fg=gt.TXT_MUTED, font=gt.F_HINT).pack(side=tk.LEFT, padx=3)
            self.link_leds[key] = {"canvas": canvas, "oval": oval}

        self.lbl_gs_pkts = tk.Label(top_bar, text="📦 PKTS: 0 (433:0|920:0)", bg=gt.BG_ROOT, fg=gt.CYAN_HI,
                                     font=gt.F_TOPBAR, width=26, anchor="w")
        self.lbl_gs_pkts.pack(side=tk.LEFT, padx=6, pady=2)

        gt.vsep(top_bar)

        # 鏈路健康度：CRC 錯誤 / resync / 丟包率（各鏈路已解出卻原本沒顯示的欄位）
        self.lbl_link_health = tk.Label(top_bar, text="⚠ 433 CRC:0/RS:0/丟:0.0%  920 CRC:0/RS:0/丟:0.0%",
                                         bg=gt.BG_ROOT, fg=gt.TXT_MUTED, font=("Monaco", 9), anchor="w")
        self.lbl_link_health.pack(side=tk.LEFT, padx=6, pady=2)

        # ── 第二排：發射鎖 + ARM + BENCH + 連線 ──
        bot_bar = tk.Frame(top_container, bg=gt.BG_ROOT)
        bot_bar.pack(fill=tk.X, side=tk.TOP, pady=(2, 4))

        lna_box = gt.hilite_box(bot_bar, *gt.BOX_LNA)
        tk.Checkbutton(lna_box, text="🔓 已確認未安裝 LNA（勾選才能發送指令）",
                        variable=self.lna_lock_var, bg=gt.BOX_LNA[0], fg="#ffcc66",
                        selectcolor=gt.BOX_LNA[0], activebackground=gt.BOX_LNA[0],
                        activeforeground="#ffcc66", font=gt.F_BOX_LABEL).pack(side=tk.LEFT, padx=2)

        # 本機板子韌體變體徽章（RX-only / TX-capable）：由連線後主動查詢 'role' 回應的
        # tx=0/1 決定，純顯示用——是否真能發射由板子本身判斷，這裡不做任何攔截。
        self.lbl_gs_tx = tk.Label(bot_bar, text="❓ TX 狀態偵測中…", bg=gt.BG_ROOT, fg=gt.TXT_MUTED,
                                   font=gt.F_BOX_LABEL, anchor="w")
        self.lbl_gs_tx.pack(side=tk.LEFT, padx=6)

        arm_box = gt.hilite_box(bot_bar, *gt.BOX_ARM, thickness=1)
        self.lbl_arm_status = gt.pill(arm_box, "🛡️ DISARMED", "#1b4332", "#2ec4b6", width=12, side=tk.LEFT, padx=(2, 4))
        self.btn_arm = gt.StyledButton(arm_box, text="⚡ ARM", command=lambda: self.send_command("arm"),
                                        bg="#851414", hover_bg="#b91c1c", fg=gt.TXT,
                                        font=gt.F_BOX_LABEL, width=7, padx=4, pady=3)
        self.btn_arm.pack(side=tk.LEFT, padx=2)
        self.btn_disarm = gt.StyledButton(arm_box, text="🛡️ DISARM", command=lambda: self.send_command("disarm"),
                                           bg="#27272a", hover_bg="#3f3f46", fg=gt.TXT,
                                           font=gt.F_BOX_LABEL, width=8, padx=4, pady=3)
        self.btn_disarm.pack(side=tk.LEFT, padx=2)

        bench_box = gt.hilite_box(bot_bar, *gt.BOX_BENCH)
        self.btn_bench = gt.StyledButton(bench_box, text="🖥️ BENCH TEST", command=self._on_bench_test,
                                          bg="#6d28d9", hover_bg="#8b5cf6", fg=gt.TXT,
                                          font=gt.F_BOX_LABEL, width=13, padx=6, pady=3)
        self.btn_bench.pack(side=tk.LEFT, padx=2)

        conn = tk.Frame(bot_bar, bg=gt.BG_ROOT)
        conn.pack(side=tk.RIGHT, pady=1)
        tk.Label(conn, text="PORT:", bg=gt.BG_ROOT, fg=gt.TXT_DIM, font=gt.F_BOX_LABEL).pack(side=tk.LEFT, padx=2)
        self.port_combo = ttk.Combobox(conn, width=20, font=gt.F_HINT, textvariable=self.serial_port)
        self.port_combo.pack(side=tk.LEFT, padx=2)
        ttk.Button(conn, text="🔄", width=3, command=self.scan_ports).pack(side=tk.LEFT, padx=2)
        tk.Label(conn, text="BAUD:", bg=gt.BG_ROOT, fg=gt.TXT_DIM, font=gt.F_BOX_LABEL).pack(side=tk.LEFT, padx=2)
        self.baud_combo = ttk.Combobox(conn, values=[9600, 38400, 115200, 460800], width=8,
                                        font=gt.F_HINT, textvariable=self.baud_rate)
        self.baud_combo.pack(side=tk.LEFT, padx=2)
        tk.Checkbutton(conn, text="LOG", variable=self.save_log_var, bg=gt.BG_ROOT, fg=gt.CYAN,
                        selectcolor=gt.BG_ROOT, font=gt.F_BOX_LABEL).pack(side=tk.LEFT, padx=4)
        self.btn_connect = ttk.Button(conn, text="CONNECT", width=10, command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=4)

        # ── 第三排：手動開傘 / 落海回收（危險操作，獨立一排避免誤觸）──
        danger_bar = tk.Frame(top_container, bg=gt.BG_ROOT)
        danger_bar.pack(fill=tk.X, side=tk.TOP, pady=(0, 4))

        deploy_box = gt.hilite_box(danger_bar, *gt.BOX_DEPLOY)
        tk.Label(deploy_box, text="🪂 手動開傘", bg=gt.BOX_DEPLOY[0], fg="#ff8080",
                 font=gt.F_BOX_LABEL).pack(side=tk.LEFT, padx=(2, 4))
        self.btn_deploy_drogue = gt.StyledButton(deploy_box, text="副傘 DROGUE", command=lambda: self._on_deploy("drogue"),
                                                  bg="#991b1b", hover_bg="#dc2626", fg=gt.TXT, font=gt.F_BOX_LABEL, padx=6, pady=3)
        self.btn_deploy_drogue.pack(side=tk.LEFT, padx=2)
        self.btn_deploy_main = gt.StyledButton(deploy_box, text="主傘 MAIN", command=lambda: self._on_deploy("main"),
                                                bg="#991b1b", hover_bg="#dc2626", fg=gt.TXT, font=gt.F_BOX_LABEL, padx=6, pady=3)
        self.btn_deploy_main.pack(side=tk.LEFT, padx=2)
        self.btn_deploy_both = gt.StyledButton(deploy_box, text="雙傘 BOTH", command=lambda: self._on_deploy("both"),
                                                bg="#991b1b", hover_bg="#dc2626", fg=gt.TXT, font=gt.F_BOX_LABEL, padx=6, pady=3)
        self.btn_deploy_both.pack(side=tk.LEFT, padx=2)
        self.lbl_deploy_status = tk.Label(deploy_box, text="尚未送出", bg=gt.BOX_ARM[0], fg=gt.TXT_MUTED,
                                           font=gt.F_BOX_LABEL, anchor="w", padx=6, pady=3)
        self.lbl_deploy_status.pack(side=tk.LEFT, padx=(6, 2))

        recovery_box = gt.hilite_box(danger_bar, *gt.BOX_RECOVERY)
        self.btn_recovery = gt.StyledButton(recovery_box, text="🔍 落海回收確認", command=self._on_recovery,
                                             bg="#0369a1", hover_bg="#0ea5e9", fg=gt.TXT, font=gt.F_BOX_LABEL, padx=6, pady=3)
        self.btn_recovery.pack(side=tk.LEFT, padx=2)
        self.lbl_recovery_status = tk.Label(recovery_box, text="尚未送出", bg=gt.BOX_ARM[0], fg=gt.TXT_MUTED,
                                             font=gt.F_BOX_LABEL, anchor="w", padx=6, pady=3)
        self.lbl_recovery_status.pack(side=tk.LEFT, padx=(6, 2))

        latch_box = gt.hilite_box(danger_bar, *gt.BOX_LATCH)
        self.lbl_deploy_latch = tk.Label(latch_box, text=self.deploy_latch.label_text()[0],
                                          bg=gt.BOX_LATCH[0], fg=gt.TXT_MUTED, font=gt.F_BOX_LABEL,
                                          anchor="w", padx=4, pady=3)
        self.lbl_deploy_latch.pack(side=tk.LEFT, padx=2)

        # ------------------ 卡片列 ------------------
        cards_frame = tk.Frame(self.root, bg=gt.BG_ROOT)
        cards_frame.pack(fill=tk.X, side=tk.TOP, padx=15, pady=5)
        card_defs = [
            ("火箭電池", "bat", "-- V", gt.ORANGE),
            ("EKF 高度", "alt", "-- m", gt.GREEN),
            ("VF 高度", "vf_alt", "-- m", gt.AMBER),
            ("EKF Vz", "vz", "-- m/s", gt.GREEN),
            ("VF Vz", "vf_vz", "-- m/s", gt.AMBER),
            ("加速度 |a|", "acc", "-- g", gt.GREEN),
            ("火箭 GPS", "rkt_gps", "-- sats", gt.MAGENTA),
            ("地面站 GPS", "gs_gps", "-- sats", gt.MAGENTA),
            ("相對距離", "dist", "-- m", gt.CYAN),
            ("鏈路品質", "link_q", "-- dBm", gt.CYAN_HI),
        ]
        self.cards = gt.make_cards(cards_frame, card_defs)

        # ------------------ 主體：左終端 / 右（分頁圖表 + GPS + 副航電） ------------------
        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg=gt.BG_ROOT,
                                    sashwidth=6, sashrelief="flat", bd=0)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)

        left_frame = tk.Frame(main_pane, bg=gt.BG_PANEL)
        main_pane.add(left_frame, width=600, minsize=380)
        self._build_left_pane(left_frame)

        right_pane = tk.PanedWindow(main_pane, orient=tk.VERTICAL, bg=gt.BG_ROOT,
                                     sashwidth=6, sashrelief="flat", bd=0)
        main_pane.add(right_pane, minsize=460)
        self._build_right_pane(right_pane)

    def _build_left_pane(self, left_frame):
        tk.Label(left_frame, text=" ★ 指令 / 回應", bg=gt.BG_PANEL, fg=gt.YELLOW,
                 font=gt.F_SECTION).pack(anchor="w", padx=10, pady=(5, 0))
        self.event_console = ScrolledText(left_frame, bg=gt.BG_CONSOLE_EVENT, fg="#e0e0e0",
                                           insertbackground="white", font=gt.F_MONO,
                                           borderwidth=0, highlightthickness=1,
                                           highlightbackground="#3a3a1a", height=8)
        self.event_console.pack(fill=tk.X, padx=5, pady=(2, 6))
        gt.config_console_tags(self.event_console, gt.EVENT_TAGS)

        gt.section_title(left_frame, "GROUND STATION LOG STREAM")

        tools = tk.Frame(left_frame, bg=gt.BG_PANEL)
        tools.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=5)
        gt.StyledButton(tools, text="清除終端", command=self.clear_console, bg="#2d2d2d",
                         hover_bg="#3d3d3d", padx=8, pady=4).pack(side=tk.LEFT, padx=3)

        cmd_bar = tk.Frame(left_frame, bg=gt.BG_PANEL)
        cmd_bar.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=(0, 4))
        tk.Label(cmd_bar, text="指令 ➤", bg=gt.BG_PANEL, fg=gt.CYAN, font=gt.F_SECTION).pack(side=tk.LEFT, padx=(6, 4))
        self._cmd_history = []
        self._cmd_history_idx = 0
        self.cmd_entry = tk.Entry(cmd_bar, bg=gt.BG_ENTRY, fg="#33ff33", insertbackground="white",
                                   font=gt.F_MONO_INPUT, relief="flat", highlightthickness=1,
                                   highlightbackground=gt.SEP, highlightcolor=gt.CYAN)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.cmd_entry.bind("<Return>", self._on_manual_cmd)
        self.cmd_entry.bind("<Up>", self._manual_cmd_history_prev)
        self.cmd_entry.bind("<Down>", self._manual_cmd_history_next)
        gt.StyledButton(cmd_bar, text="送出", command=self._on_manual_cmd, bg="#006699",
                         hover_bg="#0088cc", padx=10, pady=3).pack(side=tk.LEFT, padx=4)
        gt.StyledButton(cmd_bar, text="help", command=lambda: self.send_command("help"),
                         bg="#2a2a2a", hover_bg="#3a3a3a", fg="#cccccc", font=gt.F_MONO,
                         padx=8, pady=3).pack(side=tk.LEFT, padx=(2, 4))

        self.console = ScrolledText(left_frame, bg=gt.BG_CONSOLE, fg="#33ff33", insertbackground="white",
                                     font=gt.F_MONO, borderwidth=0, highlightthickness=0)
        self.console.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        gt.config_console_tags(self.console, {
            "GS_PKT": dict(foreground="#4CAF50"), "GS_PKT_BAD": dict(foreground="#FF5722"),
            "GS_STAT": dict(foreground="#00BCD4"), "GS_GPS": dict(foreground="#FFC107"),
            "ACK": dict(foreground="#c77dff"), "INFO": dict(foreground=gt.TXT_MUTED),
            "rate": dict(foreground=gt.CYAN),
        })

    def _build_right_pane(self, right_pane):
        # --- 上：分頁圖表（鏈路 RSSI/SNR ｜ 高度·速度比對） ---
        notebook_frame = tk.Frame(right_pane, bg=gt.BG_PANEL)
        nb = ttk.Notebook(notebook_frame)
        nb.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        link_tab = tk.Frame(nb, bg=gt.BG_PANEL)
        nb.add(link_tab, text=" 鏈路 RSSI/SNR ")
        self.build_link_charts(link_tab)

        alt_tab = tk.Frame(nb, bg=gt.BG_PANEL)
        nb.add(alt_tab, text=" 高度・速度比對 ")
        self.build_alt_charts(alt_tab)

        right_pane.add(notebook_frame, height=320, minsize=220)

        # --- 中：GPS（火箭 / 地面站 / 相對位置） ---
        gps_frame = tk.Frame(right_pane, bg=gt.BG_PANEL)
        gt.section_title(gps_frame, "GPS 定位（火箭 / 地面站 / 相對位置）")
        gps_row = tk.Frame(gps_frame, bg=gt.BG_PANEL)
        gps_row.pack(fill=tk.BOTH, expand=True, padx=6, pady=2)
        for i in range(3):
            gps_row.columnconfigure(i, weight=1, uniform="gps")

        rkt_box = tk.LabelFrame(gps_row, text=" 🚀 火箭 GPS ", bg=gt.BG_BOX, fg=gt.CYAN,
                                 font=gt.F_BOX_LABEL, padx=8, pady=6)
        rkt_box.grid(row=0, column=0, sticky="nsew", padx=4)
        self.lbl_rkt_gps_pos = tk.Label(rkt_box, text="--", bg=gt.BG_BOX, fg=gt.TXT, font=("Monaco", 10, "bold"))
        self.lbl_rkt_gps_pos.pack(anchor="w")
        self.lbl_rkt_gps_sub = tk.Label(rkt_box, text="--", bg=gt.BG_BOX, fg=gt.TXT_MUTED, font=gt.F_HINT)
        self.lbl_rkt_gps_sub.pack(anchor="w")

        gs_box = tk.LabelFrame(gps_row, text=" 📍 地面站本機 GPS ", bg=gt.BG_BOX, fg=gt.CYAN,
                                font=gt.F_BOX_LABEL, padx=8, pady=6)
        gs_box.grid(row=0, column=1, sticky="nsew", padx=4)
        self.lbl_gs_gps_pos = tk.Label(gs_box, text="--", bg=gt.BG_BOX, fg=gt.TXT, font=("Monaco", 10, "bold"))
        self.lbl_gs_gps_pos.pack(anchor="w")
        self.lbl_gs_gps_sub = tk.Label(gs_box, text="--", bg=gt.BG_BOX, fg=gt.TXT_MUTED, font=gt.F_HINT)
        self.lbl_gs_gps_sub.pack(anchor="w")

        rel_box = tk.LabelFrame(gps_row, text=" 📐 相對位置 ", bg=gt.BG_BOX, fg=gt.CYAN,
                                 font=gt.F_BOX_LABEL, padx=8, pady=6)
        rel_box.grid(row=0, column=2, sticky="nsew", padx=4)
        self.lbl_rel_pos = tk.Label(rel_box, text="--", bg=gt.BG_BOX, fg=gt.TXT, font=("Monaco", 10, "bold"))
        self.lbl_rel_pos.pack(anchor="w")
        self.lbl_rel_sub = tk.Label(rel_box, text="需兩端皆定位", bg=gt.BG_BOX, fg=gt.TXT_MUTED, font=gt.F_HINT)
        self.lbl_rel_sub.pack(anchor="w")

        right_pane.add(gps_frame, height=150, minsize=130)

        # --- 下：副航電 (BACKUP) 狀態 ---
        peer_frame = tk.Frame(right_pane, bg=gt.BG_PANEL)
        gt.section_title(peer_frame, "副航電 (BACKUP) 狀態")
        peer_box = tk.LabelFrame(peer_frame, text=" 🛸 BACKUP 中繼摘要 ", bg=gt.BG_BOX, fg=gt.AMBER,
                                  font=gt.F_BOX_LABEL, padx=8, pady=6)
        peer_box.pack(fill=tk.BOTH, expand=True, padx=10, pady=3)
        self.peer_pill_row = tk.Frame(peer_box, bg=gt.BG_BOX)
        self.peer_pill_row.pack(anchor="w", fill="x")
        self.lbl_peer_val = tk.Label(peer_box, text="尚未收到副航電資料", bg=gt.BG_BOX, fg=gt.TXT_MUTED,
                                      font=("Helvetica", 10, "bold"), justify="left", anchor="w")
        self.lbl_peer_val.pack(anchor="w", fill="x", pady=(4, 0))
        self.lbl_peer_sub = tk.Label(peer_box, text="--", bg=gt.BG_BOX, fg=gt.TXT_MUTED,
                                      font=gt.F_HINT, justify="left", anchor="w")
        self.lbl_peer_sub.pack(anchor="w", fill="x")

        right_pane.add(peer_frame, minsize=110)

    def build_link_charts(self, parent):
        self.chart_fig = plt.figure(facecolor=gt.BG_PANEL)
        gs = self.chart_fig.add_gridspec(2, 1, hspace=0.4, left=0.11, right=0.97, top=0.90, bottom=0.14)
        self.ax_rssi = self.chart_fig.add_subplot(gs[0])
        self.ax_snr = self.chart_fig.add_subplot(gs[1], sharex=self.ax_rssi)
        for ax, ylab in ((self.ax_rssi, "RSSI (dBm)"), (self.ax_snr, "SNR (dB)")):
            gt.style_axes(ax, ylab, facecolor=gt.BG_PANEL)
        self.ax_snr.set_xlabel("t (s)", color=gt.TXT_DIM, fontsize=9)
        self.ax_rssi.set_ylim(-120, 0)
        self.ax_snr.set_ylim(-20, 20)

        self.ln_rssi, self.ln_snr = {}, {}
        for key in ("433", "920"):
            self.ln_rssi[key], = self.ax_rssi.plot([], [], color=gt.LINK_COLORS[key], lw=1.6, label=f"{key}MHz")
            self.ln_snr[key], = self.ax_snr.plot([], [], color=gt.LINK_COLORS[key], lw=1.6, label=f"{key}MHz")
        for ax in (self.ax_rssi, self.ax_snr):
            gt.style_legend(ax, loc="upper right", ncol=1)

        self.chart_canvas = FigureCanvasTkAgg(self.chart_fig, master=parent)
        self.chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def build_alt_charts(self, parent):
        """高度（EKF/Baro/VF，主板 vs 副板）與速度（EKF Vz/VF Vz，主板 vs 副板）比對。
        線型比照 gui_monitor 慣例：實線=主板、虛線=副板。"""
        self.alt_fig = plt.figure(facecolor=gt.BG_PANEL)
        gs = self.alt_fig.add_gridspec(2, 1, hspace=0.4, left=0.11, right=0.97, top=0.90, bottom=0.14)
        self.ax_alt = self.alt_fig.add_subplot(gs[0])
        self.ax_vz = self.alt_fig.add_subplot(gs[1], sharex=self.ax_alt)
        for ax, ylab in ((self.ax_alt, "Alt (m)"), (self.ax_vz, "Vz (m/s)")):
            gt.style_axes(ax, ylab, facecolor=gt.BG_PANEL)
        self.ax_vz.set_xlabel("t (s)", color=gt.TXT_DIM, fontsize=9)

        self.ln_alt_ekf, = self.ax_alt.plot([], [], color=gt.CYAN_HI, lw=1.6, label="EKF")
        self.ln_alt_baro, = self.ax_alt.plot([], [], color="#ff9800", lw=0.9, label="Baro")
        self.ln_alt_vf, = self.ax_alt.plot([], [], color=gt.AMBER, lw=1.1, label="VF")
        self.ln_alt_peer, = self.ax_alt.plot([], [], color=gt.GREEN, lw=1.2, ls="--", label="副板 EKF")
        self.ln_alt_peer_baro, = self.ax_alt.plot([], [], color="#ff9800", lw=0.9, ls=":", alpha=0.7, label="副板 Baro")
        self.ln_alt_peer_vf, = self.ax_alt.plot([], [], color=gt.AMBER, lw=1.1, ls=":", alpha=0.85, label="副板 VF")

        self.ln_vz_ekf, = self.ax_vz.plot([], [], color=gt.CYAN_HI, lw=1.6, label="EKF Vz")
        self.ln_vz_vf, = self.ax_vz.plot([], [], color=gt.AMBER, lw=1.1, label="VF Vz")
        self.ln_vz_peer, = self.ax_vz.plot([], [], color=gt.GREEN, lw=1.2, ls="--", label="副板 Vz")
        self.ln_vz_peer_vf, = self.ax_vz.plot([], [], color=gt.AMBER, lw=1.1, ls=":", alpha=0.85, label="副板 VF Vz")

        for ax in (self.ax_alt, self.ax_vz):
            gt.style_legend(ax, ncol=2)

        self.alt_canvas = FigureCanvasTkAgg(self.alt_fig, master=parent)
        self.alt_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    # ------------------------------------------------------------------
    def now_t(self):
        return time.monotonic() - self.chart_t0

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
                if self.save_log_var.get():
                    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
                    os.makedirs(log_dir, exist_ok=True)
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    self.log_file = open(os.path.join(log_dir, f"ground_gui_{ts}.log"), "a", encoding="utf-8")
                self.rx_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
                self.rx_thread.start()
                self.append_console(f"[{datetime.now().strftime('%H:%M:%S')}] 連線開啟 {port}\n", "INFO")
                self.gs_tx_capable = None
                self._role_query_attempts = 0
                self.lbl_gs_tx.config(text="❓ TX 狀態偵測中…", fg=gt.TXT_MUTED)
                self.root.after(300, self.query_gs_tx)
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
        if self.log_file:
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None
        self.btn_connect.config(text="CONNECT")
        self.append_console(f"[{datetime.now().strftime('%H:%M:%S')}] 序列埠已中斷\n", "INFO")
        self.gs_tx_capable = None
        self.lbl_gs_tx.config(text="❓ TX 狀態偵測中…", fg=gt.TXT_MUTED)

    def query_gs_tx(self):
        """主動送 'role' 查詢本機板子是 RX-only 還是 TX-capable（tx=0/1）；最多重試 5 次。"""
        if not self.is_connected or self.gs_tx_capable is not None:
            return
        if self._role_query_attempts >= 5:
            self.lbl_gs_tx.config(text="❓ TX 狀態未知（舊版 firmware？）", fg=gt.TXT_MUTED)
            return
        self._role_query_attempts += 1
        try:
            self.ser.write(b"role\n")
            self.ser.flush()
        except Exception:
            return
        self.root.after(1500, self.query_gs_tx)

    def _parse_gs_tx_badge(self, line):
        """從 'role' 回應（[ROLE_ID] role=GROUND ... tx=N）或開機橫幅
        （[BOOT] ... ROLE=GROUND(地面站,RX-only|TX可發射)）判斷本機是哪種地面站韌體，
        更新徽章。純顯示，不影響任何指令是否送出——是否接受一律由板子自己回應決定。"""
        m = re.search(r"\[ROLE_ID\]\s+role=GROUND(?:\s+fw=\S+)?\s+tx=(\d)", line)
        if m:
            self._set_gs_tx_capable(int(m.group(1)))
            return
        if "ROLE=GROUND" in line and "[BOOT]" in line:
            if "RX-only" in line:
                self._set_gs_tx_capable(0)
            elif "TX可發射" in line:
                self._set_gs_tx_capable(1)

    def _set_gs_tx_capable(self, tx):
        if self.gs_tx_capable == tx:
            return
        self.gs_tx_capable = tx
        if tx == 1:
            self.lbl_gs_tx.config(text="📡 本機：TX-capable（可發射上行指令）", fg="#ff5555")
            self._event_log("📡 偵測到本機為 TX-capable 地面站韌體", "cmd")
        else:
            self.lbl_gs_tx.config(text="🔒 本機：RX-only（不接受發射指令）", fg=gt.CYAN_HI)
            self._event_log("🔒 偵測到本機為 RX-only 地面站韌體（韌體會拒絕發射類指令）", "cmd")

    def serial_read_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    raw = self.ser.readline()
                    if raw:
                        line = raw.decode('utf-8', errors='replace').strip()
                        if line:
                            self.data_queue.put(line)
                            if self.log_file:
                                self.log_file.write(line + "\n")
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.data_queue.put(f"[ERR] 讀取異常中斷: {e}")
                break

    def clear_console(self):
        self.console.delete('1.0', tk.END)

    def append_console(self, text, tag=None):
        gt.append_console(self.console, text, tag)

    def _event_log(self, text, tag=None):
        self.event_console.insert(tk.END, text + "\n", tag)
        self.event_console.see(tk.END)

    # ------------------------------------------------------------------
    def poll_queue(self):
        max_lines = 60
        n = 0
        while not self.data_queue.empty() and n < max_lines:
            line = self.data_queue.get()
            n += 1
            if line.startswith("[ERR]"):
                self.console.insert(tk.END, line + "\n", "GS_PKT_BAD")
                self.disconnect_serial()
                break
            tag = ("GS_PKT_BAD" if "[GS_PKT?]" in line else
                   "GS_PKT" if "[GS_PKT]" in line else
                   "GS_STAT" if "[GS_STAT]" in line else
                   "GS_GPS" if "[GS_GPS]" in line else
                   "ACK" if "[ACK]" in line else
                   "GS_PKT_BAD" if "[UPLINK] REJECTED" in line else
                   "rate" if "[UPLINK]" in line else "INFO")
            self.console.insert(tk.END, line + "\n", tag)
            try:
                self._parse_gs_tx_badge(line)

                m_uplink = re.search(r"\[UPLINK\] 送 (\S+) \(cmd=0x[0-9A-Fa-f]+ seq=(\d+)\)", line)
                if m_uplink:
                    self._uplink_seq_label[int(m_uplink.group(2))] = m_uplink.group(1)

                if "[UPLINK] REJECTED" in line:
                    self._event_log("🔒 " + line, "err")
                elif "[ACK]" in line:
                    self._event_log(line, "resp")
                    if 'cmd:"arm"' in line:
                        if "status:OK" in line:
                            self._set_fsm_full("STATE_PAD_ARMED")
                            self._event_log("⚡ 航電確認武裝 (ARM OK)", "resp")
                        else:
                            self._event_log("❌ 航電拒絕武裝 (ARM REJECTED)", "err")
                    elif 'cmd:"disarm"' in line:
                        if "status:OK" in line:
                            self._set_fsm_full("STATE_PAD")
                            self._event_log("🛡️ 航電確認上鎖 (DISARM OK)", "resp")
                        else:
                            self._event_log("❌ 航電拒絕上鎖 (DISARM REJECTED)", "err")
                    elif 'cmd:"bench"' in line:
                        self._event_log("🖥️ 桌面測試 " + ("完成 (OK)" if "status:OK" in line else "被拒"),
                                        "resp" if "status:OK" in line else "err")
                    elif 'cmd:"deploy"' in line:
                        self._handle_deploy_ack(line)
                    elif 'cmd:"recovery"' in line:
                        self._handle_recovery_ack(line)
                elif "[CMD]" in line or "[UPLINK]" in line:
                    self._event_log(line, "cmd")

                if "[GS_PKT?]" in line:
                    if "link:433MHz" in line:
                        self.gs_state["crc_bad_433"] += 1
                    elif "link:920MHz" in line:
                        self.gs_state["crc_bad_920"] += 1
                    self._update_link_health_label()
                elif "[GS_PKT]" in line:
                    self.parse_gs_pkt(line)
                elif "[GS_STAT]" in line:
                    self.parse_gs_stat(line)
                elif "[GS_GPS]" in line:
                    self.parse_gs_gps(line)
            except Exception as e:
                try:
                    self.console.insert(tk.END, f"[GUI] ⚠ 處理行例外: {e}\n", "GS_PKT_BAD")
                except Exception:
                    pass
        if n > 0:
            try:
                self.console.see(tk.END)
            except tk.TclError:
                pass
        try:
            if self.root.winfo_exists():
                self.root.after(10, self.poll_queue)
        except tk.TclError:
            pass

    # ------------------------------------------------------------------ [GS_PKT]
    _GS_PKT_RE = re.compile(
        r"link:(?P<link>\d+)MHz rssi:(?P<rssi>-?\d+) snr:(?P<snr>-?\d+) seq:(?P<seq>\d+) "
        r"fsm:(?P<fsm>\d+) alt:(?P<alt>-?\d+)cm vz:(?P<vz>-?\d+)cms baro:(?P<baro>-?\d+)cm "
        r"bat:(?P<bat>\d+)mV vfh:(?P<vfh>-?\d+)cm vfv:(?P<vfv>-?\d+)cms "
        r"gps:(?P<sats>\d+)/(?P<fix>\d+) pos:(?P<lat_s>[+-])(?P<lat>\d+\.\d+),(?P<lon_s>[+-])(?P<lon>\d+\.\d+) "
        r"galt:(?P<galt>-?\d+)m accel:(?P<ax>-?\d+),(?P<ay>-?\d+),(?P<az>-?\d+) "
        r"peer:(?P<peer_fsm>\d+) pflags:0x(?P<pflags>[0-9A-Fa-f]+) ph:(?P<ph>-?\d+)cm pv:(?P<pv>-?\d+)cms "
        r"plink:0x(?P<plink>[0-9A-Fa-f]+) ploss:(?P<ploss>\d+) "
        r"pbaro:(?P<pbaro>-?\d+)cm paz:(?P<paz>-?\d+) pvfh:(?P<pvfh>-?\d+)cm pvfv:(?P<pvfv>-?\d+)cms"
        r"(?:\s+pbarb:(?P<pbarb>\d+))?")

    def parse_gs_pkt(self, line):
        m = self._GS_PKT_RE.search(line)
        if not m:
            return
        g = m.groupdict()
        link = g["link"]
        t = self.now_t()

        seq = int(g["seq"])
        self._update_loss(link, seq)
        self.gs_state[f"pkts_{link}"] = self.gs_state.get(f"pkts_{link}", 0) + 1
        self.lbl_gs_pkts.config(text=f"📦 接收包數: {self.gs_state.get('pkts_433', 0) + self.gs_state.get('pkts_920', 0)} "
                                      f"(433:{self.gs_state.get('pkts_433', 0)} | 920:{self.gs_state.get('pkts_920', 0)})")
        self._flash_led(link)

        rssi, snr = int(g["rssi"]), int(g["snr"])
        stats = self.link_stats[link]
        if rssi != gt.NA_SENTINEL:
            stats["rssi"] = rssi
            stats["rssi_hist"].append((t, rssi))
        if snr != gt.NA_SENTINEL:
            stats["snr"] = snr
            stats["snr_hist"].append((t, snr))
        stats["last_pkt_time"] = time.time()
        self._update_link_quality_card()

        fsm_idx = int(g["fsm"])
        self.rocket_state["fsm"] = fsm_idx
        full = gt.FSM_INDEX_TO_FULL.get(fsm_idx, f"STATE_{fsm_idx}")
        self._set_fsm_full(full)

        alt = int(g["alt"]) / 100.0
        vz = int(g["vz"]) / 100.0
        baro = int(g["baro"]) / 100.0
        bat_v = int(g["bat"]) / 1000.0
        vfh = int(g["vfh"]) / 100.0
        vfv = int(g["vfv"]) / 100.0
        self.rocket_state.update(alt_m=alt, vz_ms=vz, baro_m=baro, bat_v=bat_v, vf_alt_m=vfh, vf_vz_ms=vfv)
        self.ts_alt_ekf.append((t, alt))
        self.ts_alt_baro.append((t, baro))
        self.ts_alt_vf.append((t, vfh))
        self.ts_vz_ekf.append((t, vz))
        self.ts_vz_vf.append((t, vfv))

        ax_, ay_, az_ = int(g["ax"]), int(g["ay"]), int(g["az"])
        acc_g = (ax_ ** 2 + ay_ ** 2 + az_ ** 2) ** 0.5 / 1000.0
        self.rocket_state["accel_mag"] = acc_g

        self.cards["bat"].config(text=f"{bat_v:.2f} V", foreground=gt.battery_color(bat_v))
        self.cards["alt"].config(text=f"{alt:.1f} m")
        self.cards["vf_alt"].config(text=f"{vfh:.1f} m")
        self.cards["vz"].config(text=f"{vz:+.1f} m/s")
        self.cards["vf_vz"].config(text=f"{vfv:+.1f} m/s")
        self.cards["acc"].config(text=f"{acc_g:.2f} g")

        sats, fix = int(g["sats"]), int(g["fix"])
        self.cards["rkt_gps"].config(text=f"{sats} sats" + (" 🔒" if fix else ""))
        lat = float(g["lat"]) * (1.0 if g["lat_s"] == '+' else -1.0)
        lon = float(g["lon"]) * (1.0 if g["lon_s"] == '+' else -1.0)
        if fix and (abs(lat) > 0.01 or abs(lon) > 0.01):
            self.rocket_state["lat"], self.rocket_state["lon"] = lat, lon
            self.rocket_state["gps_alt_m"] = int(g["galt"])
            self.lbl_rkt_gps_pos.config(text=f"{lat:+.6f}, {lon:+.6f}")
            self.lbl_rkt_gps_sub.config(text=f"sats={sats} alt={g['galt']}m")
            self._update_relative_position()

        pbarb = int(g["pbarb"]) if g["pbarb"] else 0
        self.update_peer_relay(int(g["peer_fsm"]), int(g["pflags"], 16), int(g["ph"]) / 100.0, int(g["pv"]) / 100.0,
                                int(g["plink"], 16), int(g["ploss"]), int(g["pbaro"]) / 100.0, int(g["paz"]) / 100.0,
                                int(g["pvfh"]) / 100.0, int(g["pvfv"]) / 100.0, pbarb)

        order = gt.SELF_STATE_ORDER.get(full)
        self.deploy_latch.mark_by_order("self", order)
        self.chart_dirty = True

    def _update_loss(self, link, seq):
        last = self._last_seq[link]
        if last is not None:
            gap = (seq - last) % 256
            if gap == 0:
                gap = 1
            hist = self._seq_hist[link]
            hist.append(gap)
            total = sum(hist)
            lost = sum(g - 1 for g in hist)
            self._loss_pct[link] = (lost / total * 100.0) if total > 0 else 0.0
        self._last_seq[link] = seq
        self._update_link_health_label()

    def _update_link_health_label(self):
        # [GS_PKT?] CRC_BAD 與 [GS_STAT] 的 crc=%lu 是同一個火韌體事件（見
        # ground_station.c gs_deliver_433 / GsLoraTest_UpdateStats）——兩者不可相加，
        # 否則會把同一次 CRC 錯誤算兩遍。取 max：crc_bad_* 逐包即時更新（GS_STAT 每 2
        # 秒才回報一次，中間會落後），GS_STAT 的 crc_* 則是開機以來的累積值，GUI 重新
        # 連線時 crc_bad_* 會歸零但 GS_STAT 仍是舊累積值，取 max 兩種情況都不會失真。
        s = self.gs_state
        crc_433 = max(s["crc_433"], s["crc_bad_433"])
        crc_920 = max(s["crc_920"], s["crc_bad_920"])
        text = (f"⚠ 433 CRC:{crc_433}/RS:{s['rsync_433']}/丟:{self._loss_pct['433']:.1f}%  "
                f"920 CRC:{crc_920}/RS:{s['rsync_920']}/丟:{self._loss_pct['920']:.1f}%")
        worst = max(self._loss_pct["433"], self._loss_pct["920"])
        color = gt.RED_ALT if worst > 20 else (gt.YELLOW if worst > 5 else gt.TXT_MUTED)
        self.lbl_link_health.config(text=text, fg=color)

    def _flash_led(self, link):
        led = self.link_leds[link]
        led["canvas"].itemconfig(led["oval"], fill=gt.GREEN)
        self.root.after(150, lambda: led["canvas"].itemconfig(led["oval"], fill=gt.SEP))

    def _update_link_quality_card(self):
        cands = [(k, v["rssi"]) for k, v in self.link_stats.items() if v["rssi"] is not None]
        if not cands:
            return
        key, rssi = max(cands, key=lambda kv: kv[1])
        color = gt.GREEN if rssi > -90 else (gt.YELLOW if rssi > -105 else gt.RED_ALT)
        self.cards["link_q"].config(text=f"{rssi} dBm ({key})", foreground=color)

    def _set_fsm_full(self, full):
        self.fsm_state = full
        suffix, color, pill_text, pill_bg, pill_fg = gt.fsm_style(full)
        self.lbl_fsm.config(text=f"🚀 主航電: {suffix}", fg=color)
        if pill_text:
            self.lbl_arm_status.config(text=pill_text, bg=pill_bg, fg=pill_fg)

    def _refresh_deploy_label(self, _state):
        text, color = self.deploy_latch.label_text()
        self.lbl_deploy_latch.config(text=text, fg=color)

    def update_peer_relay(self, pfsm, pflags, ph_m, pv_ms, plink, ploss_pmil,
                           pbaro_m=0.0, paz_g=0.0, pvfh_m=0.0, pvfv_ms=0.0, pbarb=0):
        name = gt.PEER_FSM_NAMES[pfsm] if pfsm < len(gt.PEER_FSM_NAMES) else f"?{pfsm}"
        ever, fresh = bool(plink & gt.PEER_LINK_EVER), bool(plink & gt.PEER_LINK_FRESH)
        lost, desync = bool(plink & gt.PEER_LINK_LOST), bool(plink & gt.PEER_LINK_DESYNC)
        loss = ploss_pmil / 10.0

        self.lbl_peer_fsm.config(text=f"🛸 副航電: {name}", fg=gt.GREEN if fresh else gt.TXT_OFF)

        for w in self.peer_pill_row.winfo_children():
            w.destroy()
        if not ever:
            gt.pill(self.peer_pill_row, "🛸 NO LINK", "#3b1a1a", gt.RED_ALT)
        else:
            state_bg, state_fg, _ = gt.FSM_PHASE.get(f"STATE_{name}", (gt.BOX_ARM[0], gt.TXT_DIM, name))
            gt.pill(self.peer_pill_row, name, state_bg, state_fg)
            if lost:
                gt.pill(self.peer_pill_row, "⚠LOST", "#3b1a1a", gt.RED_ALT)
            if desync:
                gt.pill(self.peer_pill_row, "⚠DESYNC", "#3b2d00", gt.YELLOW)
            if not fresh:
                gt.pill(self.peer_pill_row, f"逾時 {loss:.1f}%", "#3b2d00", gt.YELLOW)
            arb_name = gt.BENCH_ARB_NAMES.get(pbarb, f"?{pbarb}")
            if pbarb:
                gt.pill(self.peer_pill_row, f"🖥️ {arb_name}", "#2d1b4e", gt.PURPLE)

        if fresh:
            self.lbl_peer_val.config(
                text=f"h={ph_m:.1f}m  v={pv_ms:+.1f}m/s  baro={pbaro_m:.1f}m  |a_z|={paz_g:.2f}g",
                fg=gt.TXT)
            self.lbl_peer_sub.config(text=f"VF h={pvfh_m:.1f}m v={pvfv_ms:+.1f}m/s  丟包率 {loss:.1f}%",
                                      fg=gt.TXT_MUTED)
            t = self.now_t()
            self.ts_peer_alt.append((t, ph_m))
            self.ts_peer_vf_alt.append((t, pvfh_m))
            self.ts_peer_baro.append((t, pbaro_m))
            self.ts_peer_vz.append((t, pv_ms))
            self.ts_peer_vf_vz.append((t, pvfv_ms))
            self.chart_dirty = True

        order = gt.PEER_STATE_ORDER.get(name)
        self.deploy_latch.mark_by_order("peer", order)
        if pflags & 0x01:
            self.deploy_latch.mark_flag("peer", "drogue")
        if pflags & 0x02:
            self.deploy_latch.mark_flag("peer", "main")

    def _update_relative_position(self):
        rlat, rlon = self.rocket_state.get("lat"), self.rocket_state.get("lon")
        glat, glon = self.gs_state.get("gps_lat"), self.gs_state.get("gps_lon")
        if rlat is None or not self.gs_state.get("gps_fix") or (abs(glat) < 0.01 and abs(glon) < 0.01):
            return
        east, north = gt.latlon_to_en(rlat, rlon, glat, glon)
        dist, bearing = gt.distance_bearing(east, north)
        self.lbl_rel_pos.config(text=f"{dist:.0f} m @ {bearing:.0f}°")
        self.lbl_rel_sub.config(text=f"E={east:+.0f}m N={north:+.0f}m（自地面站看火箭）")
        self.cards["dist"].config(text=f"{dist:.0f} m")

    # ------------------------------------------------------------------ [GS_STAT]
    _GS_STAT_RE = re.compile(
        r"HW:433=(?P<hw433>\w+) 920=(?P<hw920>\w+) \| "
        r"433 raw=(?P<raw433>\d+) ok=(?P<ok433>\d+) crc=(?P<crc433>\d+) rsync=(?P<rsync433>\d+) \| "
        r"920 ok=(?P<ok920>\d+) crc=(?P<crc920>\d+) rsync=(?P<rsync920>\d+) \| "
        r"pkts 433=(?P<pk433>\d+) 920=(?P<pk920>\d+)")

    def parse_gs_stat(self, line):
        m = self._GS_STAT_RE.search(line)
        if not m:
            return
        g = m.groupdict()
        s = self.gs_state
        s["hw_433"], s["hw_920"] = g["hw433"], g["hw920"]
        s["raw_433"] = int(g["raw433"])
        s["ok_433"], s["crc_433"], s["rsync_433"] = int(g["ok433"]), int(g["crc433"]), int(g["rsync433"])
        s["ok_920"], s["crc_920"], s["rsync_920"] = int(g["ok920"]), int(g["crc920"]), int(g["rsync920"])

        self.lbl_lora433.config(text="📻 433: READY" if g["hw433"] == "OK" else "📻 433: OFF",
                                 fg=gt.GREEN if g["hw433"] == "OK" else gt.RED_ALT)
        self.lbl_lora920.config(text="📡 920: READY" if g["hw920"] == "OK" else "📡 920: OFF",
                                 fg=gt.GREEN if g["hw920"] == "OK" else gt.RED_ALT)
        self._update_link_health_label()

    # ------------------------------------------------------------------ [GS_GPS]
    def parse_gs_gps(self, line):
        if "FIX" in line:
            m = re.search(r"sats=(\d+) Pos:([+-])(\d+\.\d+),([+-])(\d+\.\d+) Alt:(-?\d+)m", line)
            if not m:
                return
            sats = int(m.group(1))
            lat = float(m.group(3)) * (1.0 if m.group(2) == '+' else -1.0)
            lon = float(m.group(5)) * (1.0 if m.group(4) == '+' else -1.0)
            alt = int(m.group(6))
            self.gs_state.update(gps_sats=sats, gps_fix=True, gps_lat=lat, gps_lon=lon, gps_alt=alt)
            self.lbl_gs_gps_pos.config(text=f"{lat:+.6f}, {lon:+.6f}")
            self.lbl_gs_gps_sub.config(text=f"sats={sats} alt={alt}m")
            self.cards["gs_gps"].config(text=f"{sats} sats 🔒")
            self._update_relative_position()
        elif "SEARCHING" in line:
            m = re.search(r"sats=(\d+) q=(\d+) ok=(\d+) err=(\d+)", line)
            if not m:
                return
            sats = int(m.group(1))
            self.gs_state.update(gps_sats=sats, gps_fix=False, gps_q=int(m.group(2)),
                                  gps_ok=int(m.group(3)), gps_err=int(m.group(4)))
            self.lbl_gs_gps_pos.config(text="搜尋中…")
            self.lbl_gs_gps_sub.config(text=f"sats={sats} q={m.group(2)}")
            self.cards["gs_gps"].config(text=f"{sats} sats")

    # ------------------------------------------------------------------ 手動指令列
    def _on_manual_cmd(self, event=None):
        cmd = self.cmd_entry.get().strip()
        if not cmd:
            return "break"
        if self.send_command(cmd):
            if not self._cmd_history or self._cmd_history[-1] != cmd:
                self._cmd_history.append(cmd)
            self._cmd_history_idx = len(self._cmd_history)
            self.cmd_entry.delete(0, tk.END)
        return "break"

    def _manual_cmd_history_prev(self, event=None):
        if not self._cmd_history:
            return "break"
        self._cmd_history_idx = max(0, self._cmd_history_idx - 1)
        self.cmd_entry.delete(0, tk.END)
        self.cmd_entry.insert(0, self._cmd_history[self._cmd_history_idx])
        return "break"

    def _manual_cmd_history_next(self, event=None):
        if not self._cmd_history:
            return "break"
        self._cmd_history_idx = min(len(self._cmd_history), self._cmd_history_idx + 1)
        self.cmd_entry.delete(0, tk.END)
        if self._cmd_history_idx < len(self._cmd_history):
            self.cmd_entry.insert(0, self._cmd_history[self._cmd_history_idx])
        return "break"

    # ------------------------------------------------------------------ 危險操作
    def _on_deploy(self, kind):
        zh, cmd_text = self._DEPLOY_INFO[kind]
        st = self.fsm_state
        ans = messagebox.askyesno(
            f"⚠️ 手動開傘確認（{zh}）",
            f"確定要立即經 433 上行送出『手動開傘 - {zh}』指令嗎？\n\n"
            "⚠️ 注意：這會直接導通火箭上的點火/舵機機構，若已裝藥將實際點燃！\n"
            f"當前航電狀態：{st or '未知'}\n\n"
            "本按鈕只在收到航電回傳 [ACK] 確認後才會顯示「已確認開傘」。\n"
            "是否立即執行？")
        if not ans:
            return
        self.lbl_deploy_status.config(text=f"⏳ 已送出「{zh}」，等待航電 ACK…", bg="#3b2d00", fg=gt.YELLOW)
        self.send_command(cmd_text)

    def _handle_deploy_ack(self, line):
        m = re.search(r"seq:(\d+)", line)
        seq = int(m.group(1)) if m else None
        raw_label = self._uplink_seq_label.pop(seq, None) if seq is not None else None
        zh = self._DEPLOY_LABEL_ZH.get(raw_label, "開傘")
        if "status:OK" in line:
            self.lbl_deploy_status.config(text=f"✅ 航電已確認開傘：{zh}", bg="#064e3b", fg="#34d399")
            self._event_log(f"🪂 航電確認開傘動作已執行：{zh}", "resp")
            if raw_label in ("DEPLOY-DROGUE", "DEPLOY-BOTH"):
                self.deploy_latch.mark_flag("self", "drogue")
            if raw_label in ("DEPLOY-MAIN", "DEPLOY-BOTH"):
                self.deploy_latch.mark_flag("self", "main")
        else:
            self.lbl_deploy_status.config(text=f"❌ 開傘遭拒：{zh}", bg="#3b1a1a", fg="#ff4d4d")
            self._event_log(f"❌ 開傘指令被拒（{zh}）：{line.strip()}", "err")

    def _on_recovery(self):
        st = self.fsm_state
        ans = messagebox.askyesno(
            "落海回收確認",
            "確定要經 433 上行送出『尋回指令 (RECOVERY)』嗎？\n\n"
            "⚠️ 這會停止板載尋標蜂鳴器，並安全關閉 SD/Flash 數據記錄（無法復原）。\n"
            f"當前航電狀態：{st or '未知'}\n\n"
            "本按鈕只在收到航電回傳 [ACK] 確認後才會顯示「已確認」。\n"
            "確定已完成落點回收、要停止蜂鳴器與記錄嗎？")
        if not ans:
            return
        self.lbl_recovery_status.config(text="⏳ 已送出，等待航電 ACK…", bg="#3b2d00", fg=gt.YELLOW)
        self.send_command("recovery")

    def _handle_recovery_ack(self, line):
        if "status:OK" in line:
            self.lbl_recovery_status.config(text="✅ 已確認：蜂鳴器已停止／SD·Flash 記錄已安全關閉",
                                             bg="#064e3b", fg="#34d399")
            self._event_log("🔇 航電確認尋回指令已執行（蜂鳴器/SD/Flash 已停止）", "resp")
        else:
            self.lbl_recovery_status.config(text="❌ 尋回指令遭拒（可能仍在飛行中）", bg="#3b1a1a", fg="#ff4d4d")
            self._event_log(f"❌ 尋回指令被拒：{line.strip()}", "err")

    def _on_bench_test(self):
        ans = messagebox.askyesno(
            "手動桌面測試確認 (BENCH)",
            "確定要經 433 上行發送『桌面點火/舵機測試 (BENCH)』指令嗎？\n\n"
            "⚠️ 注意：\n"
            "1. 航電將依序執行 PD13 點火通電 (8 秒)、1s Guard 意圖確認與 PD14 舵機轉動測試。\n"
            "2. 全程耗時約 25 秒，測試完成後自動復位並回歸正常 FSM。\n"
            "3. 航電必須處於解鎖狀態 (STATE_PAD_ARMED)。\n\n"
            "是否立即執行？")
        if ans:
            self.send_command("bench")

    # ------------------------------------------------------------------
    def charts_redraw_loop(self):
        if self.chart_dirty:
            self.redraw_charts()
            self.chart_dirty = False
        try:
            if self.root.winfo_exists():
                self.root.after(200, self.charts_redraw_loop)
        except tk.TclError:
            pass

    def redraw_charts(self):
        now = self.now_t()
        t0 = now - CHART_HIST_SEC

        def clipped(hist):
            pts = [(t, v) for t, v in hist if t >= t0]
            if not pts:
                return [], []
            xs, ys = zip(*pts)
            return xs, ys

        for key in ("433", "920"):
            xs, ys = clipped(self.link_stats[key]["rssi_hist"])
            self.ln_rssi[key].set_data(xs, ys)
            xs, ys = clipped(self.link_stats[key]["snr_hist"])
            self.ln_snr[key].set_data(xs, ys)
        for ax in (self.ax_rssi, self.ax_snr):
            ax.set_xlim(t0, max(now, t0 + 1))
        self.chart_canvas.draw_idle()

        self.ln_alt_ekf.set_data(*clipped(self.ts_alt_ekf))
        self.ln_alt_baro.set_data(*clipped(self.ts_alt_baro))
        self.ln_alt_vf.set_data(*clipped(self.ts_alt_vf))
        self.ln_alt_peer.set_data(*clipped(self.ts_peer_alt))
        self.ln_alt_peer_baro.set_data(*clipped(self.ts_peer_baro))
        self.ln_alt_peer_vf.set_data(*clipped(self.ts_peer_vf_alt))
        self.ln_vz_ekf.set_data(*clipped(self.ts_vz_ekf))
        self.ln_vz_vf.set_data(*clipped(self.ts_vz_vf))
        self.ln_vz_peer.set_data(*clipped(self.ts_peer_vz))
        self.ln_vz_peer_vf.set_data(*clipped(self.ts_peer_vf_vz))
        for ax in (self.ax_alt, self.ax_vz):
            ax.set_xlim(t0, max(now, t0 + 1))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        self.alt_canvas.draw_idle()

    def on_close(self):
        self.disconnect_serial()
        try:
            self.root.destroy()
        except Exception:
            pass
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description="RocketCom Ground Station Monitor")
    parser.add_argument("--port", type=str, default=None)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    root = tk.Tk()
    app = GroundStationGUI(root)
    gt.install_signal_handlers(root, app.on_close)
    if args.port:
        app.serial_port.set(args.port)
        app.baud_rate.set(args.baud)
        app.root.after(500, app.toggle_connection)
    root.mainloop()


if __name__ == "__main__":
    main()
