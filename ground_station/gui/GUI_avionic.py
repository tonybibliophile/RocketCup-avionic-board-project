#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GUI_avionic — 航電板直連監視／操作台（USB-TTL/SWO 直接接 PRIMARY/BACKUP 板，非經 LoRa）。

視覺語彙、StyledButton、卡片/徽章/高亮框工廠、深色 matplotlib 配方全部來自
`gui_theme.py`（抽自 gui_monitor.py 旗艦儀表板，同一套配色不重新發明）。與 gui_monitor
的差異只在於輕量化：不畫 3D 火箭姿態模型（改用數值面板顯示 roll/pitch/yaw/傾角）、
不含地圖/LoRa 參數設定/Flash 匯出面板——那些請用 gui_monitor.py 本尊。

★ 安全性提醒：本工具直連航電板 USB，沒有 LoRa 距離當緩衝——按下「開傘」按鈕會直接、
立即導通火箭上的點火/舵機機構，中間沒有任何實體阻隔。因此不採用 ground_gui.py 那把針對
「發射會燒毀 LNA」設計的發射鎖，改用「⚠️ 已確認點火頭未接／現場淨空」現場安全鎖：未勾選
前，ARM/開傘/回收/BENCH 全部指令一律被 `gui_theme.CommandSender` 擋下。
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


class AvionicMonitorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RocketCom 航電板直連監視儀 (Avionics Debug Monitor)")
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry(f"{min(1500, sw - 60)}x{min(940, sh - 80)}")
        self.root.minsize(1150, 760)
        self.root.configure(bg=gt.BG_ROOT)

        self.serial_port = tk.StringVar()
        self.baud_rate = tk.IntVar(value=DEFAULT_BAUD)
        self.save_log_var = tk.BooleanVar(value=False)
        self.safety_lock_var = tk.BooleanVar(value=False)
        self.is_connected = False
        self.running = False
        self.ser = None
        self.rx_thread = None
        self.log_file = None
        self.data_queue = queue.Queue()

        self.role = None            # "PRIMARY" / "BACKUP"
        self.fsm_state = None       # "STATE_xxx" 全名
        self._last_deploy_kind = None   # 直連單一指令通道：不需要 seq 比對，記上一個送出的種類即可

        self._DEPLOY_INFO = {
            "drogue": ("副傘 DROGUE", "deploy drogue"),
            "main":   ("主傘 MAIN", "deploy main"),
            "both":   ("副傘+主傘 BOTH", "deploy both"),
        }
        self.deploy_latch = gt.DeployLatch(on_change=self._refresh_deploy_label)

        self.chart_t0 = time.monotonic()
        self.chart_dirty = False
        _N = 6000
        self.ts_alt_ekf = deque(maxlen=_N)
        self.ts_alt_baro = deque(maxlen=_N)      # 氣壓高度，已扣掉 pad_ref → 相對起點 (m)
        self.ts_alt_ground = deque(maxlen=_N)    # 航電原生 braw_cm（1Hz），未經 VF/EKF 融合
        self.ts_alt_vf = deque(maxlen=_N)
        self.pad_ref = gt.PadRefTracker()
        # 電梯測試 profile 醒目警示：本 GUI 直連 USB，來源只有航電板自己印的
        # [PAD_CFG] Flight Profile:（10s 一行，權威、會明講已切回正式版）與
        # [ELEVATOR_TEST_WARNING]（1s 一行，只在仍是電梯 profile 時才印）。
        # 兩個時間戳走 3 秒新鮮度窗口，語意與 gui_monitor.py 同一套。
        self._elevator_self_ts = 0.0
        self._elevator_peer_ts = 0.0
        self.ts_vz_ekf = deque(maxlen=_N)
        self.ts_vz_vf = deque(maxlen=_N)
        self.ts_acc_bmi = deque(maxlen=_N)
        self.ts_acc_adxl = deque(maxlen=_N)
        self.ts_tilt = deque(maxlen=_N)

        self.last_q = [1.0, 0.0, 0.0, 0.0]
        self.latest_imu = {}     # BMI088 a[mG]/g[dps]
        self.latest_highg = {}   # ADXL375 a[mG]
        self.latest_mag = {}     # MMC5983 B[mG]/hdg

        self.cmd_sender = gt.CommandSender(
            get_ser=lambda: self.ser,
            get_running=lambda: self.running,
            console_echo=self.append_console,
            event_log=self._event_log,
            lock_var=self.safety_lock_var,
            lock_warning=("⚠️ 現場安全鎖：未確認淨空",
                          "本工具直連航電板 USB，沒有 LoRa 距離當緩衝——送出的指令會立即、"
                          "直接執行！\n\n"
                          "開傘/BENCH 指令會直接導通火箭上的點火/舵機機構，若已裝藥將實際點燃。\n\n"
                          "請確認點火頭未接、現場已淨空後，勾選上方「⚠️ 已確認點火頭未接／"
                          "現場淨空」再發送。"),
        )

        gt.setup_styles()
        self.build_ui()
        self.scan_ports()
        self.root.after(10, self.poll_queue)
        self.root.after(200, self.charts_redraw_loop)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------------
    def send_command(self, cmd_str):
        return self.cmd_sender.send(cmd_str, parent=self.root)

    # ------------------------------------------------------------------ UI
    def build_ui(self):
        top_container = tk.Frame(self.root, bg=gt.BG_ROOT)
        top_container.pack(fill=tk.X, side=tk.TOP, padx=10, pady=(5, 0))
        # 橫幅擠在最上方（pack(before=...)，見 gui_theme.update_elevator_banner），
        # 平時不 pack、偵測到電梯 profile 才出現並閃爍。
        self.elevator_banner = gt.make_elevator_banner(self.root)

        top_bar = tk.Frame(top_container, bg=gt.BG_ROOT)
        top_bar.pack(fill=tk.X, side=tk.TOP)

        self.lbl_fsm = tk.Label(top_bar, text="🚀 STATE: --", bg=gt.BG_ROOT, fg=gt.CYAN_HI,
                                 font=gt.F_STATE, width=24, anchor="w")
        self.lbl_fsm.pack(side=tk.LEFT, padx=4, pady=2)

        self.lbl_role = tk.Label(top_bar, text="ROLE: --", bg=gt.BG_ROOT, fg=gt.TXT_DIM,
                                  font=gt.F_TOPBAR, width=18, anchor="w")
        self.lbl_role.pack(side=tk.LEFT, padx=4, pady=2)

        gt.vsep(top_bar)

        self.lbl_link = tk.Label(top_bar, text="🔗 LINK: --", bg=gt.BG_ROOT, fg=gt.TXT_OFF,
                                  font=gt.F_TOPBAR, width=32, anchor="w")
        self.lbl_link.pack(side=tk.LEFT, padx=4, pady=2)

        gt.vsep(top_bar)

        self.lbl_health = tk.Label(top_bar, text="❤️ HEALTH: --", bg=gt.BG_ROOT, fg=gt.TXT_OFF,
                                    font=gt.F_TOPBAR, width=26, anchor="w")
        self.lbl_health.pack(side=tk.LEFT, padx=4, pady=2)

        self.lbl_cpu = tk.Label(top_bar, text="⚙️ CPU: --", bg=gt.BG_ROOT, fg=gt.TXT_OFF,
                                 font=gt.F_TOPBAR, width=24, anchor="w")
        self.lbl_cpu.pack(side=tk.LEFT, padx=4, pady=2)

        self.lbl_drops = tk.Label(top_bar, text="EKF Drops: 0  SD:--", bg=gt.BG_ROOT, fg=gt.TXT_MUTED,
                                   font=("Monaco", 9), anchor="w")
        self.lbl_drops.pack(side=tk.LEFT, padx=6, pady=2)

        # ── 第二排：現場安全鎖 + ARM + BENCH + 連線 ──
        bot_bar = tk.Frame(top_container, bg=gt.BG_ROOT)
        bot_bar.pack(fill=tk.X, side=tk.TOP, pady=(2, 4))

        safety_box = gt.hilite_box(bot_bar, *gt.BOX_LNA)
        tk.Checkbutton(safety_box, text="⚠️ 已確認點火頭未接／現場淨空（勾選才能發送指令）",
                        variable=self.safety_lock_var, bg=gt.BOX_LNA[0], fg="#ffcc66",
                        selectcolor=gt.BOX_LNA[0], activebackground=gt.BOX_LNA[0],
                        activeforeground="#ffcc66", font=gt.F_BOX_LABEL).pack(side=tk.LEFT, padx=2)

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
        self.baud_combo = ttk.Combobox(conn, values=[9600, 38400, 115200, 460800, 921600], width=8,
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
        tk.Label(deploy_box, text="🪂 手動開傘（直連，按下即刻導通）", bg=gt.BOX_DEPLOY[0], fg="#ff8080",
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
            ("BMI088 A", "bmi_a", "0.00 Hz", gt.CYAN),
            ("BMI088 G", "bmi_g", "0.00 Hz", gt.CYAN),
            ("ADXL375", "adxl", "0.00 Hz", gt.YELLOW),
            ("BMP388", "bmp", "0.00 Hz", gt.GREEN_ALT),
            ("MMC5983", "mag", "0.00 Hz", gt.RED),
            ("GPS Update", "gps", "0.00 Hz", gt.MAGENTA),
            ("高度 (EKF)", "alt", "-- m", gt.GREEN),
            ("相對起點 (baro)", "ground_alt", "-- m", gt.ORANGE),
            ("零點 pad_ref", "pad_ref", "-- m", gt.TXT_MUTED),
            ("Vz 垂直速度", "vz", "-- m/s", gt.GREEN),
            ("加速度 |a|", "acc", "-- g", gt.GREEN),
            ("電池", "bat", "-- V", gt.ORANGE),
        ]
        self.cards = gt.make_cards(cards_frame, card_defs)

        # ------------------ 主體：左終端 / 右（圖表 + 數值面板） ------------------
        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg=gt.BG_ROOT,
                                    sashwidth=6, sashrelief="flat", bd=0)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)

        left_frame = tk.Frame(main_pane, bg=gt.BG_PANEL)
        main_pane.add(left_frame, width=560, minsize=380)
        self._build_left_pane(left_frame)

        right_pane = tk.PanedWindow(main_pane, orient=tk.VERTICAL, bg=gt.BG_ROOT,
                                     sashwidth=6, sashrelief="flat", bd=0)
        main_pane.add(right_pane, minsize=460)
        self._build_right_pane(right_pane)

    def _build_left_pane(self, left_frame):
        gt.section_title(left_frame, "TERMINAL STREAM")

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
        gt.config_console_tags(self.console, gt.TERMINAL_TAGS)

    def _build_right_pane(self, right_pane):
        charts_frame = tk.Frame(right_pane, bg=gt.BG_PANEL)
        gt.section_title(charts_frame, "ALT / VELOCITY / ACCEL (EKF + BARO + VF + RAW)")
        self.build_charts(charts_frame)
        right_pane.add(charts_frame, height=430, minsize=260)

        values_frame = tk.Frame(right_pane, bg=gt.BG_PANEL)
        gt.section_title(values_frame, "姿態 / 三軸分量 / GPS")
        row = tk.Frame(values_frame, bg=gt.BG_PANEL)
        row.pack(fill=tk.BOTH, expand=True, padx=6, pady=2)
        for i in range(3):
            row.columnconfigure(i, weight=1, uniform="val")
        self._build_attitude_panel(row).grid(row=0, column=0, sticky="nsew", padx=4)
        self._build_axes_panel(row).grid(row=0, column=1, sticky="nsew", padx=4)
        self._build_gps_panel(row).grid(row=0, column=2, sticky="nsew", padx=4)
        right_pane.add(values_frame, minsize=200)

    def _grid_kv(self, parent, rows):
        """[(label, key), ...] → 兩欄網格，回傳 {key: value_label}。"""
        grid = tk.Frame(parent, bg=gt.BG_GPS_PANEL, padx=6, pady=4)
        grid.pack(fill=tk.BOTH, expand=True)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)
        labels = {}
        for i, (text, key) in enumerate(rows):
            tk.Label(grid, text=text, bg=gt.BG_GPS_PANEL, fg=gt.TXT_MUTED, font=gt.F_HINT,
                     anchor="w").grid(row=i, column=0, sticky="w", pady=3)
            lbl = tk.Label(grid, text="--", bg=gt.BG_GPS_PANEL, fg=gt.TXT, font=gt.F_HINT, anchor="e")
            lbl.grid(row=i, column=1, sticky="e", pady=3)
            labels[key] = lbl
        return labels

    def _build_attitude_panel(self, parent):
        box = tk.Frame(parent, bg=gt.BG_GPS_PANEL, highlightbackground="#2a2a30", highlightthickness=1)
        tk.Label(box, text="📐 姿態 (EKF q)", bg=gt.BG_GPS_PANEL, fg=gt.CYAN,
                 font=gt.F_BOX_LABEL, pady=6).pack(fill=tk.X)
        self.lbl_att = self._grid_kv(box, [
            ("Roll:", "roll"), ("Pitch:", "pitch"), ("Yaw:", "yaw"), ("傾角 Tilt:", "tilt"),
        ])
        return box

    def _build_axes_panel(self, parent):
        box = tk.Frame(parent, bg=gt.BG_GPS_PANEL, highlightbackground="#2a2a30", highlightthickness=1)
        tk.Label(box, text="📊 三軸分量", bg=gt.BG_GPS_PANEL, fg=gt.CYAN,
                 font=gt.F_BOX_LABEL, pady=6).pack(fill=tk.X)
        self.lbl_axes = self._grid_kv(box, [
            ("BMI a [mG]:", "bmi_a"), ("BMI g [dps]:", "bmi_g"),
            ("ADXL a [mG]:", "adxl_a"), ("MAG B [mG]:", "mag_b"), ("MAG hdg:", "mag_hdg"),
        ])
        return box

    def _build_gps_panel(self, parent):
        box = tk.Frame(parent, bg=gt.BG_GPS_PANEL, highlightbackground="#2a2a30", highlightthickness=1)
        tk.Label(box, text="🛰️ GPS 尋星狀態", bg=gt.BG_GPS_PANEL, fg=gt.CYAN,
                 font=gt.F_BOX_LABEL, pady=6).pack(fill=tk.X)
        self.lbl_gps = self._grid_kv(box, [
            ("定位狀態:", "fix"), ("衛星數量:", "sats"), ("信號狀態:", "stale"),
            ("解析成功:", "ok"), ("解析失敗:", "err"), ("經緯度:", "pos"),
        ])
        return box

    def build_charts(self, parent):
        self.chart_fig = plt.figure(facecolor=gt.BG_ROOT)
        gs = self.chart_fig.add_gridspec(3, 1, hspace=0.35, left=0.11, right=0.97, top=0.97, bottom=0.08)
        self.ax_alt = self.chart_fig.add_subplot(gs[0])
        self.ax_vel = self.chart_fig.add_subplot(gs[1], sharex=self.ax_alt)
        self.ax_acc = self.chart_fig.add_subplot(gs[2], sharex=self.ax_alt)
        for ax, ylab in ((self.ax_alt, "Alt (m)"), (self.ax_vel, "Vz (m/s)"), (self.ax_acc, "|a| (g)")):
            gt.style_axes(ax, ylab)
        self.ax_acc.set_xlabel("t (s)", color=gt.TXT_DIM, fontsize=9)

        # 三條線同零點（發射台）：EKF/VF 本來就是相對值，Baro 這條在 append 時已扣掉
        # pad_ref。原本畫的是絕對海拔，跟另兩條差一個發射台海拔的常數偏移（場測 log 出現過
        # 25 m / 47.6 m / 101 m），高度軸被撐開後相對線全糊在一起。
        self.ln_alt_ekf, = self.ax_alt.plot([], [], color=gt.CYAN_HI, lw=1.6, label="EKF")
        self.ln_alt_baro, = self.ax_alt.plot([], [], color="#ff9800", lw=0.9, label="Baro 相對 10Hz")
        self.ln_alt_ground, = self.ax_alt.plot([], [], color="#ffffff", lw=1.0, ls="-.", alpha=0.85,
                                                label="航電 braw 1Hz")
        self.ln_alt_vf, = self.ax_alt.plot([], [], color=gt.AMBER, lw=1.1, label="VF")

        self.ln_vz_ekf, = self.ax_vel.plot([], [], color=gt.CYAN_HI, lw=1.6, label="EKF Vz")
        self.ln_vz_vf, = self.ax_vel.plot([], [], color=gt.AMBER, lw=1.1, label="VF Vz")

        self.ln_acc_bmi, = self.ax_acc.plot([], [], color="#28d745", lw=0.9, label="BMI088 raw")
        self.ln_acc_adxl, = self.ax_acc.plot([], [], color=gt.RED, lw=0.9, alpha=0.8, label="ADXL375 raw")

        for ax in (self.ax_alt, self.ax_vel, self.ax_acc):
            gt.style_legend(ax)

        self.chart_canvas = FigureCanvasTkAgg(self.chart_fig, master=parent)
        self.chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

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
                    self.log_file = open(os.path.join(log_dir, f"avionic_gui_{ts}.log"), "a", encoding="utf-8")
                self.rx_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
                self.rx_thread.start()
                self.append_console(f"[{datetime.now().strftime('%H:%M:%S')}] 連線開啟 {port}\n", "ok")
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
        self.append_console(f"[{datetime.now().strftime('%H:%M:%S')}] 序列埠已中斷\n", "err")

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
        # 本 GUI 沒有獨立「指令/回應」小視窗，直接標記進終端即可（tag=rate 高亮 CMD 回音）
        self.append_console(text + "\n", "rate" if tag == "cmd" else ("ok" if tag == "resp" else tag))

    # ---------------- 電梯測試 profile 醒目警示（語意同 gui_monitor.py） ----------------
    def _handle_pad_cfg_profile_line(self, line):
        """[PAD_CFG] Flight Profile: ELEVATOR_TEST(...) / REAL_FLIGHT(...)，10s 一行，只講本板。
        是唯一會明講「已切回正式版」的權威來源，故 REAL_FLIGHT 直接歸零、不等窗口逾時。"""
        self._elevator_self_ts = time.time() if "ELEVATOR_TEST" in line else 0.0
        self._refresh_elevator_banner()

    def _handle_elevator_warning_line(self, line):
        """!!! [ELEVATOR_TEST_WARNING] THIS BOARD / PEER BOARD ... !!!（1s 一行，見 main.c）"""
        now = time.time()
        who = []
        if "THIS BOARD" in line:
            self._elevator_self_ts = now
            who.append("本板")
        if "PEER BOARD" in line:
            self._elevator_peer_ts = now
            who.append("對端(副板)")
        self._refresh_elevator_banner()
        gt.spam_elevator_console_warning(self.console, "+".join(who))

    def _refresh_elevator_banner(self):
        """3 秒新鮮度窗口；由 poll_queue 每輪呼叫，故不需要另設逾時計時器。"""
        now = time.time()
        self_active = (now - self._elevator_self_ts) < 3.0
        peer_active = (now - self._elevator_peer_ts) < 3.0
        who = []
        if self_active: who.append("本板")
        if peer_active: who.append("對端(副板)")
        gt.update_elevator_banner(self.root, self.elevator_banner,
                                  self_active or peer_active, "+".join(who))

    # ------------------------------------------------------------------
    def poll_queue(self):
        max_lines = 80
        n = 0
        while not self.data_queue.empty() and n < max_lines:
            line = self.data_queue.get()
            n += 1
            if line.startswith("[ERR]"):
                self.console.insert(tk.END, line + "\n", "err")
                self.disconnect_serial()
                break
            # 電梯測試 profile：先於一般 tag 判斷（否則會被下面的 "WARNING"→err 吃掉）
            if "[ELEVATOR_TEST_WARNING]" in line:
                self._handle_elevator_warning_line(line)
            elif "[PAD_CFG]" in line and "Flight Profile:" in line:
                self._handle_pad_cfg_profile_line(line)
            tag = ("elevator_warn" if "[ELEVATOR_TEST_WARNING]" in line else
                   "ack" if "[ACK]" in line else
                   "rate" if "[RATE]" in line else
                   "mag" if "[MAG]" in line else
                   "gps" if "[GPS]" in line else
                   "link" if "[LINK]" in line else
                   "lora" if ("[ROLE" in line or "[BOOT]" in line) else
                   "err" if ("ERR" in line or "FAIL" in line or "WARNING" in line) else
                   "ok" if ("OK" in line or "SUCCESS" in line) else "tele")
            self.console.insert(tk.END, line + "\n", tag)
            try:
                self.parse_line(line)
            except Exception as e:
                try:
                    self.console.insert(tk.END, f"[GUI] ⚠ 處理行例外: {e}\n", "err")
                except Exception:
                    pass
        if n > 0:
            try:
                self.console.see(tk.END)
            except tk.TclError:
                pass
        # 新鮮度窗口逾時（斷線或已切回正式版）要自動收合橫幅
        try:
            self._refresh_elevator_banner()
        except tk.TclError:
            pass
        try:
            if self.root.winfo_exists():
                self.root.after(10, self.poll_queue)
        except tk.TclError:
            pass

    # ------------------------------------------------------------------ 解析
    # BENCH 副板延後量（韌體 DROGUE_LEAD_TIME_S，依 profile 1s/4s）自韌體行擷取，
    # 供 _on_bench_test 的確認框顯示實際值而非寫死秒數。
    _BENCH_LEAD_RE = re.compile(r"副板延後\s*(\d+(?:\.\d+)?)s|對應飛行提前\s*(\d+(?:\.\d+)?)s")

    def parse_line(self, line):
        if "[PYRO-SELFTEST]" in line:
            m_lead = self._BENCH_LEAD_RE.search(line)
            if m_lead:
                self.bench_lead_s = m_lead.group(1) or m_lead.group(2)

        # 角色偵測：優先 [ROLE_ID] role=xxx（主動查詢回應），退回 [BOOT] ROLE=xxx
        if self.role is None:
            m = re.search(r"\[ROLE_ID\]\s+role=(PRIMARY|BACKUP)", line) or re.search(r"ROLE=(PRIMARY|BACKUP)", line)
            if m:
                self.role = m.group(1)
                color = gt.GREEN if self.role == "PRIMARY" else gt.AMBER
                self.lbl_role.config(text=f"ROLE: {self.role}", fg=color)

        if "[ARM] SUCCESS:" in line:
            if "STATE_PAD_ARMED" in line:
                self._set_fsm_full("STATE_PAD_ARMED")
            elif "STATE_PAD" in line:
                self._set_fsm_full("STATE_PAD")
            return
        if "[ARM] WARNING:" in line:
            return

        if "[ACK]" in line:
            if 'cmd:"arm"' in line:
                self._set_fsm_full("STATE_PAD_ARMED" if "status:OK" in line else self.fsm_state)
            elif 'cmd:"disarm"' in line:
                self._set_fsm_full("STATE_PAD" if "status:OK" in line else self.fsm_state)
            elif 'cmd:"deploy"' in line:
                self._handle_deploy_ack(line)
            elif 'cmd:"recovery"' in line:
                self._handle_recovery_ack(line)
            elif 'cmd:"bench"' in line:
                pass  # BENCH 進度另有 [BENCH]/[PYRO-SELFTEST] 行，終端已可見，此工具不做彈窗監控
            return

        if "[FSM]" in line:
            # 發射台氣壓零點重零（PAD 期每 30s，ARM 後停）——相對起點高度的分母
            if self.pad_ref.feed(line):
                self._refresh_pad_ref_card()
                return
            m_hb = re.search(r"state=([A-Z_]+)\s+role=(\w+)", line)
            m_ev = re.search(r"(STATE_[A-Z_]+)", line)
            _SHORTNAME_MAP = {
                "INIT": "STATE_INIT", "PAD": "STATE_PAD", "BOOST": "STATE_BOOST",
                "COAST": "STATE_COAST", "DEP_DROGUE": "STATE_DROGUE", "APOGEE": "STATE_APOGEE",
                "DESCENT": "STATE_DESCENT", "MAIN_DEPLOY": "STATE_MAIN", "LANDED": "STATE_LANDED",
                "PAD_ARMED": "STATE_PAD_ARMED",
            }
            if m_hb:
                self._set_fsm_full(_SHORTNAME_MAP.get(m_hb.group(1), f"STATE_{m_hb.group(1)}"))
            elif m_ev:
                self._set_fsm_full(m_ev.group(1))
            return

        if "[RATE]" in line:
            for key, pat in (("bmi_a", r"BMI088_A:([\d\.]+)Hz"), ("bmi_g", r"BMI088_G:([\d\.]+)Hz"),
                              ("adxl", r"ADXL375:([\d\.]+)Hz"), ("bmp", r"BMP388:([\d\.]+)Hz"),
                              ("mag", r"MMC5983:([\d\.]+)Hz"), ("gps", r"GPS:([\d\.]+)Hz")):
                m = re.search(pat, line)
                if m:
                    self.cards[key].config(text=f"{float(m.group(1)):.2f} Hz")
            m_sd = re.search(r"SD_DET:(\d+)", line)
            m_drop = re.search(r"EKF_DROP:(\d+)", line)
            if m_sd or m_drop:
                self.lbl_drops.config(
                    text=f"EKF Drops: {m_drop.group(1) if m_drop else '?'}  SD:{'插入' if m_sd and m_sd.group(1) == '1' else '未插'}")
            return

        if "[TELE]" in line:
            t = self.now_t()
            m = re.search(r"pos:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                alt = float(m.group(3))
                self.ts_alt_ekf.append((t, alt))
                self.cards["alt"].config(text=f"{alt:.1f} m")
                self.chart_dirty = True
            m = re.search(r"vel:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                vz = float(m.group(3))
                self.ts_vz_ekf.append((t, vz))
                self.cards["vz"].config(text=f"{vz:+.1f} m/s")
                self.chart_dirty = True
            m = re.search(r"q:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                q = [float(m.group(i)) for i in range(1, 5)]
                self.last_q = q
                roll, pitch, yaw = gt.quaternion_to_euler(q)
                tilt = gt.tilt_angle_deg(q)
                self.ts_tilt.append((t, tilt))
                self.lbl_att["roll"].config(text=f"{roll:+.1f}°")
                self.lbl_att["pitch"].config(text=f"{pitch:+.1f}°")
                self.lbl_att["yaw"].config(text=f"{yaw:.1f}°")
                self.lbl_att["tilt"].config(text=f"{tilt:.1f}°",
                                             fg=(gt.RED_ALT if tilt > 20 else gt.TXT))
            return

        # 10Hz 裸 CSV：bmi_ax,ay,az(mG), adxl_ax,ay,az(mG), temp(x100), press(Pa), baro_alt(cm)
        if line and (line[0] == '-' or line[0].isdigit()) and line.count(',') == 8:
            try:
                v = [int(x) for x in line.split(',')]
            except ValueError:
                return
            t = self.now_t()
            acc_bmi = (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5 / 1000.0
            acc_adxl = (v[3] ** 2 + v[4] ** 2 + v[5] ** 2) ** 0.5 / 1000.0
            baro_alt = v[8] / 100.0          # 絕對海拔（韌體 baro_data.altitude）
            self.ts_acc_bmi.append((t, acc_bmi))
            self.ts_acc_adxl.append((t, acc_adxl))
            # 扣掉當下生效的 pad_ref（逐點扣，不是畫圖時才扣）——這樣重零後舊點仍保留
            # 當時航電實際採用的零點，跟韌體 in.baro_alt_rel 的逐筆算法一致。
            baro_rel = self.pad_ref.rel(baro_alt)
            if baro_rel is not None:
                self.ts_alt_baro.append((t, baro_rel))
            self.cards["acc"].config(text=f"{acc_bmi:.2f} g")
            self.chart_dirty = True
            return

        if "[VF]" in line:
            m = re.search(r"h_cm=(-?\d+) v_cms=(-?\d+)", line)
            if m:
                t = self.now_t()
                vfh = int(m.group(1)) / 100.0
                vfv = int(m.group(2)) / 100.0
                self.ts_alt_vf.append((t, vfh))
                self.ts_vz_vf.append((t, vfv))
                self.chart_dirty = True
            # braw_cm = 航電自己算的 baro_alt_rel（未經 VF/EKF 融合），1Hz。
            # 這是「航電認為自己相對起點多高」的原始值，優先於上面 10Hz 推導版顯示於卡片。
            m_braw = re.search(r"braw_cm=(-?\d+)", line)
            if m_braw:
                braw = int(m_braw.group(1)) / 100.0
                self.ts_alt_ground.append((self.now_t(), braw))
                self.cards["ground_alt"].config(text=f"{braw:+.1f} m")
                self.chart_dirty = True
            return

        if "[IMU]" in line:
            m = re.search(r"a\[mG\]:(-?\d+),(-?\d+),(-?\d+)\s+g\[dps\]:(-?\d+),(-?\d+),(-?\d+)", line)
            if m:
                self.latest_imu = {"ax": int(m.group(1)), "ay": int(m.group(2)), "az": int(m.group(3)),
                                    "gx": int(m.group(4)), "gy": int(m.group(5)), "gz": int(m.group(6))}
                self.lbl_axes["bmi_a"].config(text=f"{self.latest_imu['ax']:+d},{self.latest_imu['ay']:+d},{self.latest_imu['az']:+d}")
                self.lbl_axes["bmi_g"].config(text=f"{self.latest_imu['gx']:+d},{self.latest_imu['gy']:+d},{self.latest_imu['gz']:+d}")
            return

        if "[HIGHG]" in line:
            m = re.search(r"a\[mG\]:(-?\d+),(-?\d+),(-?\d+)", line)
            if m:
                self.latest_highg = {"ax": int(m.group(1)), "ay": int(m.group(2)), "az": int(m.group(3))}
                self.lbl_axes["adxl_a"].config(
                    text=f"{self.latest_highg['ax']:+d},{self.latest_highg['ay']:+d},{self.latest_highg['az']:+d}")
            return

        if "[MAG] B[mG]" in line:
            m = re.search(r"B\[mG\]:(-?\d+),(-?\d+),(-?\d+)\s+hdg:(-?\d+)", line)
            if m:
                self.latest_mag = {"mx": int(m.group(1)), "my": int(m.group(2)), "mz": int(m.group(3)),
                                    "hdg": int(m.group(4))}
                self.lbl_axes["mag_b"].config(
                    text=f"{self.latest_mag['mx']:+d},{self.latest_mag['my']:+d},{self.latest_mag['mz']:+d}")
                self.lbl_axes["mag_hdg"].config(text=f"{self.latest_mag['hdg']}°")
            return

        if "[PWR]" in line and "bat:" in line:
            m = re.search(r"bat:(\d+)mV", line)
            if m:
                bat_v = int(m.group(1)) / 1000.0
                self.cards["bat"].config(text=f"{bat_v:.2f} V", foreground=gt.battery_color(bat_v))
            return

        if "[GPS]" in line and "fix:" in line:
            m = re.search(r"fix:(\d+) q:(\d+) sat:(\d+) ([+-])(\d+\.\d+),([+-])(\d+\.\d+) "
                           r"alt:(-?\d+)m spd:(-?\d+)cm/s(?: stale:(\d+) ok:(\d+) err:(\d+))?", line)
            if m:
                fix_val, sats = int(m.group(1)), int(m.group(3))
                fix_text, fix_color = gt.gps_fix_style(fix_val)
                self.lbl_gps["fix"].config(text=fix_text, fg=fix_color)
                self.lbl_gps["sats"].config(text=str(sats), fg=gt.TXT if sats >= 4 else gt.YELLOW)
                self.cards["gps"].config(text=self.cards["gps"].cget("text"))  # Hz 卡片由 [RATE] 行更新，這裡不動
                if m.group(10) is not None:
                    stale = int(m.group(10))
                    self.lbl_gps["stale"].config(text="資料逾時" if stale else "即時更新",
                                                  fg=gt.RED_ALT if stale else gt.GREEN)
                if m.group(11) is not None:
                    self.lbl_gps["ok"].config(text=m.group(11))
                if m.group(12) is not None:
                    err = m.group(12)
                    self.lbl_gps["err"].config(text=err, fg=gt.TXT if err == "0" else gt.RED_ALT)
                if fix_val == 1:
                    lat = float(m.group(5)) * (1.0 if m.group(4) == '+' else -1.0)
                    lon = float(m.group(7)) * (1.0 if m.group(6) == '+' else -1.0)
                    self.lbl_gps["pos"].config(text=f"{lat:+.5f},{lon:+.5f}")
            return

        if "[HEALTH]" in line:
            m = re.search(r"sens=0x([0-9A-Fa-f]+) ekf=0x([0-9A-Fa-f]+) fsm=(\d+)", line)
            if m:
                sens, ekf = int(m.group(1), 16), int(m.group(2), 16)
                ok = (sens == 0 and ekf == 0)
                self.lbl_health.config(text=f"❤️ HEALTH: sens=0x{m.group(1)} ekf=0x{m.group(2)}",
                                        fg=gt.GREEN if ok else gt.RED_ALT)
            return

        if "[CPU]" in line:
            m = re.search(r"MainTask\+ISR:([\d\.]+)%, EKFTask:([\d\.]+)%", line)
            if m:
                main_pct, ekf_pct = float(m.group(1)), float(m.group(2))
                color = gt.RED_ALT if max(main_pct, ekf_pct) > 85 else (gt.YELLOW if max(main_pct, ekf_pct) > 60 else gt.TXT_MUTED)
                self.lbl_cpu.config(text=f"⚙️ CPU main={main_pct:.1f}% ekf={ekf_pct:.1f}%", fg=color)
            return

        if "[LINK]" in line:
            m = re.search(r"self=(\w+)\s+peer=(\w+)\s+link=(\w+)\s+state=(\w+)\s+flags=0x([0-9A-Fa-f]+)\s+age=(\d+)ms", line)
            if m:
                self_role, peer_role, link_ok, peer_state, flags_hex, age_ms = m.groups()
                if link_ok == "OK":
                    text, color = f"🔗 LINK: OK peer={peer_state} ({age_ms}ms)", gt.GREEN
                elif link_ok == "STALE":
                    text, color = f"🔗 LINK: STALE peer={peer_state} ({age_ms}ms)", gt.YELLOW
                else:
                    text, color = "🔗 LINK: OFF (單機/對端未連)", gt.RED_ALT
                self.lbl_link.config(text=text, fg=color)
                p_order = gt.PEER_STATE_ORDER.get(peer_state)
                self.deploy_latch.mark_by_order("peer", p_order)
                flags = int(flags_hex, 16)
                if flags & 0x01:
                    self.deploy_latch.mark_flag("peer", "drogue")
                if flags & 0x02:
                    self.deploy_latch.mark_flag("peer", "main")
            return

    def _set_fsm_full(self, full):
        if not full:
            return
        self.fsm_state = full
        suffix, color, pill_text, pill_bg, pill_fg = gt.fsm_style(full)
        self.lbl_fsm.config(text=f"🚀 STATE: {suffix}", fg=color)
        if pill_text:
            self.lbl_arm_status.config(text=pill_text, bg=pill_bg, fg=pill_fg)
        order = gt.SELF_STATE_ORDER.get(full)
        self.deploy_latch.mark_by_order("self", order)

    def _refresh_deploy_label(self, _state):
        text, color = self.deploy_latch.label_text()
        self.lbl_deploy_latch.config(text=text, fg=color)

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
            f"⚠️ 手動開傘確認（{zh}）— 直連板即刻執行",
            f"確定要立即送出『手動開傘 - {zh}』指令嗎？\n\n"
            "⚠️ 本工具直連航電板 USB，沒有 LoRa 距離當緩衝——這會「立即」直接導通"
            "點火/舵機機構，若已裝藥將實際點燃！\n\n"
            f"當前航電狀態：{st or '未知'}\n\n"
            "是否立即執行？")
        if not ans:
            return
        self._last_deploy_kind = kind
        self.lbl_deploy_status.config(text=f"⏳ 已送出「{zh}」，等待 ACK…", bg="#3b2d00", fg=gt.YELLOW)
        self.send_command(cmd_text)

    def _handle_deploy_ack(self, line):
        kind = self._last_deploy_kind
        self._last_deploy_kind = None
        zh = self._DEPLOY_INFO.get(kind, (None, None))[0] or "開傘"
        if "status:OK" in line:
            self.lbl_deploy_status.config(text=f"✅ 已確認開傘：{zh}", bg="#064e3b", fg="#34d399")
            self.append_console(f"[GUI] ✅ 開傘動作已執行：{zh}\n", "ok")
            if kind in ("drogue", "both"):
                self.deploy_latch.mark_flag("self", "drogue")
            if kind in ("main", "both"):
                self.deploy_latch.mark_flag("self", "main")
        else:
            self.lbl_deploy_status.config(text=f"❌ 開傘遭拒：{zh}", bg="#3b1a1a", fg="#ff4d4d")
            self.append_console(f"[GUI] ❌ 開傘指令被拒（{zh}）：{line.strip()}\n", "err")

    def _on_recovery(self):
        st = self.fsm_state
        ans = messagebox.askyesno(
            "落海回收確認",
            "確定要送出『尋回指令 (RECOVERY)』嗎？\n\n"
            "⚠️ 這會停止板載尋標蜂鳴器，並安全關閉 SD/Flash 數據記錄（無法復原）。\n"
            f"當前航電狀態：{st or '未知'}\n\n"
            "確定已完成落點回收、要停止蜂鳴器與記錄嗎？")
        if not ans:
            return
        self.lbl_recovery_status.config(text="⏳ 已送出，等待 ACK…", bg="#3b2d00", fg=gt.YELLOW)
        self.send_command("recovery")

    def _handle_recovery_ack(self, line):
        if "status:OK" in line:
            self.lbl_recovery_status.config(text="✅ 已確認：蜂鳴器已停止／SD·Flash 記錄已安全關閉",
                                             bg="#064e3b", fg="#34d399")
            self.append_console("[GUI] 🔇 尋回指令已執行（蜂鳴器/SD/Flash 已停止）\n", "ok")
        else:
            self.lbl_recovery_status.config(text="❌ 尋回指令遭拒（可能仍在飛行中）", bg="#3b1a1a", fg="#ff4d4d")
            self.append_console(f"[GUI] ❌ 尋回指令被拒：{line.strip()}\n", "err")

    def _on_bench_test(self):
        # 副板延後量 = 韌體 DROGUE_LEAD_TIME_S，依 FLIGHT_PROFILE_ELEVATOR 為 1s(電梯場測)
        # /4s(飛行)，GUI 不寫死；沿用上次自 [PYRO-SELFTEST] 行解析到的值（見 _feed_line）。
        lead_txt = (f"{self.bench_lead_s}s" if getattr(self, "bench_lead_s", None)
                    else "DROGUE_LEAD_TIME_S（依 profile 為 1s/4s，以韌體開場行為準）")
        ans = messagebox.askyesno(
            "手動桌面測試確認 (BENCH)",
            "確定要發送『桌面開傘測試 (BENCH)』指令嗎？\n\n"
            "⚠️ 注意（時序與飛行邏輯 1:1 對應）：\n"
            f"1. 引傘 PD13：主板 t=0 起通電 8s；副板延後 {lead_txt}（模擬頂點提前量）後通電 3s，\n"
            "   兩板通電窗會重疊（PD13 為 diode-OR 準位訊號，同時拉高無妨）。\n"
            "2. 主傘 PD14：★不啟動 PWM，兩板『同時』純 GPIO 拉高 1.5s（已取消互斥握手）。\n"
            "3. 全程耗時約 20 秒，測試完成後自動復位並回歸正常 FSM。\n"
            "4. 航電必須處於解鎖狀態 (STATE_PAD_ARMED)。\n\n"
            "是否立即執行？")
        if ans:
            self.send_command("bench")

    # ------------------------------------------------------------------
    def _refresh_pad_ref_card(self):
        """零點卡片：值 + 「幾秒前重零」/「已凍結」。文字沒變就不動 widget。"""
        text, color = self.pad_ref.label_text()
        if getattr(self, "_pad_ref_text", None) != text:
            self._pad_ref_text = text
            self.cards["pad_ref"].config(text=text, foreground=color)

    def charts_redraw_loop(self):
        # 「幾秒前重零」要自己走鐘，不能只在收到 pad_ref 行時更新
        self._refresh_pad_ref_card()
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

        self.ln_alt_ekf.set_data(*clipped(self.ts_alt_ekf))
        self.ln_alt_baro.set_data(*clipped(self.ts_alt_baro))
        self.ln_alt_ground.set_data(*clipped(self.ts_alt_ground))
        self.ln_alt_vf.set_data(*clipped(self.ts_alt_vf))
        self.ln_vz_ekf.set_data(*clipped(self.ts_vz_ekf))
        self.ln_vz_vf.set_data(*clipped(self.ts_vz_vf))
        self.ln_acc_bmi.set_data(*clipped(self.ts_acc_bmi))
        self.ln_acc_adxl.set_data(*clipped(self.ts_acc_adxl))

        for ax in (self.ax_alt, self.ax_vel, self.ax_acc):
            ax.set_xlim(t0, max(now, t0 + 1))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self.chart_canvas.draw_idle()

    def on_close(self):
        self.disconnect_serial()
        try:
            self.root.destroy()
        except Exception:
            pass
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description="RocketCom Avionics Debug Monitor")
    parser.add_argument("--port", type=str, default=None)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    root = tk.Tk()
    app = AvionicMonitorGUI(root)
    gt.install_signal_handlers(root, app.on_close)
    if args.port:
        app.serial_port.set(args.port)
        app.baud_rate.set(args.baud)
        app.root.after(500, app.toggle_connection)
    root.mainloop()


if __name__ == "__main__":
    main()
