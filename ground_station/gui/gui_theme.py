#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gui_theme — RocketCom 地面站 GUI 共用視覺語彙與小工具庫。

本模組把 gui_monitor.py（旗艦儀表板，4674 行）裡散落的硬編碼配色／字體／StyledButton／
卡片與徽章工廠函式抽出來，供 ground_gui.py 與 GUI_avionic.py 匯入，讓三支 GUI 呈現一致的
深色主題與版面骨架。gui_monitor.py 原則上不匯入本模組（其工作區有大量未 commit 的變更，
故意不去動它），但抽出的每一段都是從它「原樣搬」出來的，沒有另外發明新配色。
例外：PadRefTracker 三支 GUI 共用（重零凍結門檻這種規則複製兩份遲早會走鐘），
gui_monitor.py 只 import 這一個名字。

只依賴 stdlib + tkinter + matplotlib（numpy 是 matplotlib 的既有依賴，四元數換算借用它）。
"""

import queue
import re
import signal
import threading
import time

import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox

# ==================== 色票（取自 gui_monitor.py 各建構點，見 plan 對照表） ====================
BG_ROOT = "#151515"
BG_PANEL = "#1e1e1e"
BG_CARD = "#222222"
BG_BOX = "#1c1c1c"
BG_CONSOLE = "#101010"
BG_CONSOLE_EVENT = "#0d0d0d"
BG_INPUT = "#252525"
BG_ENTRY = "#0e0e0e"
BG_GPS_PANEL = "#101014"
SEP = "#333333"

CYAN = "#00d2ff"
CYAN_HI = "#00e5ff"
GREEN = "#00e676"
GREEN_ALT = "#28a745"
YELLOW = "#ffcc00"
AMBER = "#ffca28"
ORANGE = "#ffa500"
RED = "#ff3b30"
RED_ALT = "#ff3366"
MAGENTA = "#e03bfb"
PURPLE = "#a855f7"
INDIGO = "#818cf8"

TXT = "#ffffff"
TXT_DIM = "#aaaaaa"
TXT_MUTED = "#888888"
TXT_OFF = "#555555"
TXT_HINT = "#8a8a8a"

# 高亮框 tint/accent 配對（ARM / 開傘 / 回收 / BENCH / 發射鎖 / 開傘紀錄，gui_monitor.py 各建構點）
BOX_LNA = ("#4a1a00", "#ff8c00")
BOX_ARM = ("#1a1a1a", "#333333")
BOX_BENCH = ("#2d1b4e", "#8b5cf6")
BOX_DEPLOY = ("#3b1414", "#dc2626")
BOX_RECOVERY = ("#0a2030", "#0ea5e9")
BOX_LATCH = ("#1a1a1a", "#f59e0b")

# ==================== 字體 ====================
F_HDR = ("Helvetica", 14, "bold")
F_STATE = ("Helvetica", 11, "bold")
F_TOPBAR = ("Helvetica", 10, "bold")
F_BOX_LABEL = ("Helvetica", 9, "bold")
F_HINT = ("Helvetica", 9)
F_CARD_TITLE = ("Helvetica", 9)
F_CARD_VAL = ("Helvetica", 13, "bold")
F_SECTION = ("Monaco", 10, "bold")
F_MONO = ("Monaco", 9)
F_MONO_BOLD = ("Monaco", 9, "bold")
F_MONO_BIG = ("Monaco", 13, "bold")
F_MONO_INPUT = ("Monaco", 10)

# ==================== 狀態語意表 ====================
NA_SENTINEL = -32768  # 韌體「無此值」哨兵（gs_log.h: GS_RSSI_NA / GS_SNR_NA）

# FSM 0-9（韌體 fsm.h 順序），index 8 統一用 MAIN（原 ground_gui.py 曾誤寫 MAIN_DEPLOY）
FSM_STATES = {
    0: ("INIT", GREEN), 1: ("PAD", GREEN), 2: ("PAD_ARMED", RED),
    3: ("BOOST", YELLOW), 4: ("COAST", AMBER), 5: ("DROGUE", PURPLE),
    6: ("APOGEE", INDIGO), 7: ("DESCENT", "#c084fc"), 8: ("MAIN", INDIGO),
    9: ("LANDED", CYAN_HI),
}

# 圖表色帶用：STATE_* 全名 → (底色, 文字色, 短標籤)，取自 gui_monitor.py:2900-2911 _FSM_PHASE
FSM_PHASE = {
    "STATE_INIT":      ("#1a2e1a", GREEN, "INIT"),
    "STATE_PAD":       ("#1a2e1a", GREEN, "PAD"),
    "STATE_PAD_ARMED": ("#3b1a1a", "#ff4d4d", "ARMED"),
    "STATE_BOOST":     ("#3b2d00", YELLOW, "BOOST"),
    "STATE_COAST":     ("#2d2d00", "#ffe566", "COAST"),
    "STATE_DROGUE":    ("#2e1a4a", PURPLE, "DROGUE"),
    "STATE_APOGEE":    ("#1a1a4a", INDIGO, "APOGEE"),
    "STATE_DESCENT":   ("#2e1a4a", "#c084fc", "DESCENT"),
    "STATE_MAIN":      ("#1a1a4a", INDIGO, "MAIN"),
    "STATE_LANDED":    ("#0a2030", CYAN_HI, "LANDED"),
}

SELF_STATE_ORDER = {
    "STATE_INIT": 0, "STATE_PAD": 1, "STATE_PAD_ARMED": 2, "STATE_BOOST": 3,
    "STATE_COAST": 4, "STATE_DROGUE": 5, "STATE_APOGEE": 6, "STATE_DESCENT": 7,
    "STATE_MAIN": 8, "STATE_LANDED": 9,
}
PEER_STATE_ORDER = {
    "INIT": 0, "PAD": 1, "PAD_ARMED": 2, "BOOST": 3, "COAST": 4,
    "DEP_DROGUE": 5, "APOGEE": 6, "DESCENT": 7, "MAIN_DEPLOY": 8, "LANDED": 9,
}
PEER_FSM_NAMES = ["INIT", "PAD", "PAD_ARMED", "BOOST", "COAST", "DROGUE",
                   "APOGEE", "DESCENT", "MAIN", "LANDED"]

# TELEM_FLAG_*（telemetry.h），含 ground_gui.py 舊版漏掉的 0x04 SD
FLAG_NAMES = [
    (0x01, "副傘已點火"), (0x02, "主傘已展開"), (0x04, "SD 記錄中"),
    (0x08, "GPS逾時"), (0x10, "EKF異常"), (0x20, "感測器故障"),
    (0x40, "FAILSAFE"), (0x80, "熱啟動"),
]
PEER_LINK_EVER, PEER_LINK_FRESH, PEER_LINK_LOST, PEER_LINK_DESYNC = 0x01, 0x02, 0x04, 0x08
# ServoArb_MsgName() 字串 → 頂部鏈路列短標籤（★互斥握手已取消，見下方 BENCH_ARB_NAMES）
ARB_SHORT = {
    "NONE": "IDLE", "BENCH_START": "BENCH", "BENCH_PRI_FIRE": "P-FIRE",
    "BENCH_SEC_FIRE": "S-FIRE", "BENCH_MAIN_HIGH": "CO-HI", "MAIN_HIGH": "HIGH",
    "DONE": "DONE", "LEGACY_DRIVING": "OLD-PWM",
}

# LinkPacket.main_arb（韌體 servo_arb.h 的 SERVO_ARB_MSG_*）→ 顯示字串。
# ★主傘已取消 PWM 舵機與互斥握手：改為兩板同時把 PD14 純 GPIO 拉高 1.5s。
#   1 由舊 INTENT 改作「BENCH 序列開始宣告」；2(DRIVING) 隨 PWM 一起廢除，僅保留以看舊紀錄。
BENCH_ARB_NAMES = {
    0: "NONE",
    1: "BENCH_START (桌測開始)",
    2: "LEGACY_DRIVING (舊版PWM，已廢除)",
    3: "DONE (完成)",
    4: "BENCH_PRI_FIRE (主板引傘通電)",
    5: "BENCH_SEC_FIRE (副板引傘通電)",
    6: "BENCH_MAIN_HIGH (雙板主傘共開)",
    7: "MAIN_HIGH (主傘PD14拉高中)",
}

LINK_COLORS = {"433": CYAN, "920": "#c77dff"}

# fsm:%u（[GS_PKT]/[TELE] 數值）→ STATE_* 全名，供 fsm_style()/SELF_STATE_ORDER 查表用
FSM_INDEX_TO_FULL = {
    0: "STATE_INIT", 1: "STATE_PAD", 2: "STATE_PAD_ARMED", 3: "STATE_BOOST",
    4: "STATE_COAST", 5: "STATE_DROGUE", 6: "STATE_APOGEE", 7: "STATE_DESCENT",
    8: "STATE_MAIN", 9: "STATE_LANDED",
}


def fsm_style(full_state):
    """回傳 (label_suffix, label_color, pill_text_or_None, pill_bg, pill_fg)。
    逐一對照 gui_monitor.py:2913-2945 `update_fsm_state_ui` 的 if/elif 分支——
    刻意保留原本 STATE_APOGEE/STATE_DESCENT 落入 else（不更新 ARM pill）的行為，
    不要「順手」把它們也併進降落分支，那不是原本的設計。"""
    if full_state in ("STATE_PAD", "STATE_INIT"):
        return full_state, GREEN, "🛡️ DISARMED (安全)", "#1b4332", "#2ec4b6"
    if full_state == "STATE_PAD_ARMED":
        return "PAD_ARMED (解鎖 待發射!)", RED, "⚡ ARMED (解鎖/待發射!)", "#7f1d1d", "#ff4d4d"
    if full_state in ("STATE_BOOST", "STATE_COAST"):
        return f"{full_state} (飛行中)", YELLOW, "🚀 IN FLIGHT (飛行中)", "#b45309", "#fef08a"
    if "DROGUE" in full_state or "MAIN" in full_state or "RECOVERY" in full_state:
        return f"{full_state} (降落中)", PURPLE, "🪂 RECOVERY (降落中)", "#581c87", "#e9d5ff"
    if full_state == "STATE_LANDED":
        return "LANDED (已著陸)", CYAN_HI, "🏁 LANDED (著陸)", "#0284c7", "#e0f2fe"
    return full_state, CYAN_HI, None, None, None


def battery_color(bat_v):
    """電池電壓三段門檻變色（gui_monitor.py:2101-2102）"""
    return GREEN_ALT if bat_v >= 7.4 else (YELLOW if bat_v >= 6.8 else RED)


def gps_fix_style(fix):
    """定位狀態文字＋顏色（gui_monitor.update_gps_status_panel, gui_monitor.py:1268-1275）"""
    if fix == 0:
        return "未定位 (No Fix)", RED_ALT
    if fix == 1:
        return "已定位 (3D Fix)", GREEN
    if fix == 2:
        return "差分定位 (DGPS)", GREEN
    return f"定位中 ({fix})", YELLOW


def latlon_to_en(lat, lon, lat0, lon0):
    """經緯度 → 相對 Home 的 East/North 公尺（等距圓柱近似，短距離足夠）
    取自 gui_monitor.py:1297-1300 `_latlon_to_en`。"""
    k = 111320.0
    east = (lon - lon0) * k * np.cos(np.radians(lat0))
    north = (lat - lat0) * k
    return east, north


def distance_bearing(east, north):
    """由相對 East/North 算距離(m)與方位角(度，正北=0，順時針)。"""
    dist = float(np.hypot(east, north))
    bearing = (np.degrees(np.arctan2(east, north)) + 360.0) % 360.0
    return dist, float(bearing)


def quaternion_to_euler(q):
    """四元數 [qw,qx,qy,qz] → Roll/Pitch/Yaw（度），Z-Y-X 航太順序。
    逐字取自 gui_monitor.py:191-224。"""
    w, x, y, z = q
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0
    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    deg_yaw = np.degrees(yaw)
    if deg_yaw < 0:
        deg_yaw += 360.0
    return np.degrees(roll), np.degrees(pitch), deg_yaw


def tilt_angle_deg(q):
    """姿態相對「筆直朝上」的傾角（度）：body Z 軸與 nav Z 軸夾角，比 roll/pitch 更直覺。"""
    w, x, y, z = q
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-6:
        return 0.0
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    # body Z 軸在 nav 座標下的分量（旋轉矩陣第三欄，只需要 z 分量算夾角）
    bz_z = 1 - 2 * (x * x + y * y)
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, bz_z)))))


# ==================== 跨平台高對比按鈕（逐字取自 gui_monitor.py:228-277） ====================
class StyledButton(tk.Label):
    """跨平台 (特別優化 macOS Aqua) 高對比自訂按鈕，解決 macOS 下 tk.Button 底色白化與文字隱形問題"""

    def __init__(self, master, text="", command=None, bg="#2a2a2a", fg="#ffffff",
                 hover_bg="#3a3a3a", active_bg="#00a8e8", font=("Helvetica", 9, "bold"),
                 padx=10, pady=5, state=tk.NORMAL, relief="flat", bd=0, **kwargs):
        super().__init__(master, text=text, bg=bg, fg=fg, font=font,
                          relief=relief, bd=bd, cursor="hand2" if state == tk.NORMAL else "arrow",
                          padx=padx, pady=pady, **kwargs)
        self.command = command
        self.bg_normal = bg
        self.fg_normal = fg
        self.bg_hover = hover_bg
        self.bg_active = active_bg
        self.state = state

        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<Button-1>", self._on_click)

    def _on_enter(self, e):
        if self.state == tk.NORMAL:
            self.config(bg=self.bg_hover)

    def _on_leave(self, e):
        if self.state == tk.NORMAL:
            self.config(bg=self.bg_normal)

    def _on_click(self, e):
        if self.state == tk.NORMAL and self.command:
            self.config(bg=self.bg_active)
            self.after(100, lambda: self.config(
                bg=self.bg_hover if self.winfo_exists() and self.winfo_containing(
                    self.winfo_pointerx(), self.winfo_pointery()) == self else self.bg_normal
            ))
            self.command()

    def config_state(self, state, text=None, bg=None, fg=None):
        self.state = state
        if text is not None:
            self.config(text=text)
        if bg is not None:
            self.bg_normal = bg
            self.bg_hover = bg
        if fg is not None:
            self.fg_normal = fg

        if state == tk.DISABLED:
            self.config(fg="#666666", cursor="arrow", bg="#222222")
        else:
            self.config(fg=self.fg_normal, bg=self.bg_normal, cursor="hand2")


# ==================== ttk 樣式（gui_monitor.py:735-758 完整版） ====================
def setup_styles():
    style = ttk.Style()
    style.theme_use("clam")

    style.configure("TFrame", background=BG_PANEL)
    style.configure("Card.TFrame", background=BG_CARD, borderwidth=1, relief="ridge")
    style.configure("Header.TLabel", background=BG_ROOT, foreground=TXT, font=F_HDR)
    style.configure("Card.TLabel", background=BG_CARD, foreground=TXT_DIM, font=F_CARD_TITLE)
    style.configure("CardVal.TLabel", background=BG_CARD, foreground=CYAN, font=("Helvetica", 12, "bold"))

    style.configure("TButton", font=F_TOPBAR, background=SEP, foreground=TXT, borderwidth=0)
    style.map("TButton", background=[("active", CYAN)], foreground=[("active", BG_ROOT)])
    style.configure("Connect.TButton", font=F_TOPBAR, background=GREEN_ALT, foreground=TXT)
    style.configure("Disconnect.TButton", font=F_TOPBAR, background="#dc3545", foreground=TXT)

    style.configure("TNotebook", background=BG_BOX, borderwidth=0)
    style.configure("TNotebook.Tab", background="#2a2a2a", foreground="#cccccc",
                     font=F_TOPBAR, padding=[14, 6])
    style.map("TNotebook.Tab", background=[("selected", "#00435a")], foreground=[("selected", CYAN)])
    return style


# ==================== 版面工廠 ====================
def make_cards(parent, spec):
    """卡片列工廠。spec: [(title, key, default_text, color), ...]。
    回傳 {key: ttk.Label}，抽自 gui_monitor.py:576-602。"""
    cards = {}
    for idx, (title, key, default, color) in enumerate(spec):
        parent.columnconfigure(idx, weight=1, uniform="equal")
        card = ttk.Frame(parent, style="Card.TFrame")
        card.grid(row=0, column=idx, padx=5, sticky="nsew")
        ttk.Label(card, text=title, style="Card.TLabel").pack(anchor="w", padx=8, pady=4)
        lbl_val = ttk.Label(card, text=default, font=F_CARD_VAL, background=BG_CARD, foreground=color)
        lbl_val.pack(anchor="w", padx=8, pady=4)
        cards[key] = lbl_val
    return cards


def hilite_box(parent, tint, accent, thickness=2, side=tk.LEFT, padx=(4, 6), pady=1):
    """高亮強調框（ARM／開傘／回收／BENCH／發射鎖），取自 gui_monitor.py 各建構點。"""
    box = tk.Frame(parent, bg=tint, highlightbackground=accent, highlightthickness=thickness,
                    padx=4, pady=2)
    box.pack(side=side, padx=padx, pady=pady)
    return box


def pill(parent, text, bg, fg, width=None, font=F_BOX_LABEL, side=tk.LEFT, padx=(2, 4)):
    """狀態徽章（gui_monitor.py:496 樣式的 tk.Label pill）。"""
    lbl = tk.Label(parent, text=text, bg=bg, fg=fg, font=font,
                    anchor="center", padx=4, pady=3, width=width)
    lbl.pack(side=side, padx=padx)
    return lbl


def section_title(parent, text, **pack_kw):
    """`" > TITLE"` Monaco 青色區塊標題，取自 gui_monitor.py:619/637/757。"""
    kw = dict(anchor="w", padx=10, pady=5)
    kw.update(pack_kw)
    lbl = tk.Label(parent, text=f" > {text}", bg=BG_PANEL, fg=CYAN, font=F_SECTION)
    lbl.pack(**kw)
    return lbl


def vsep(parent, height=18):
    """頂欄垂直分隔線，取自 gui_monitor.py:406。"""
    f = tk.Frame(parent, bg=SEP, width=1, height=height)
    f.pack(side=tk.LEFT, padx=4, fill=tk.Y)
    return f


def labeled_row(parent, row, label_text, hint_text, width=16):
    """三欄式設定列：左標籤／中容器（回傳給呼叫者放輸入元件）／右提示。
    泛化自 gui_monitor.py:3436-3448 `_lora_row`。"""
    tk.Label(parent, text=label_text, bg=BG_BOX, fg=TXT, font=F_TOPBAR,
              anchor="w", width=width).grid(row=row, column=0, sticky="w", padx=(14, 6), pady=6)
    holder = tk.Frame(parent, bg=BG_BOX)
    holder.grid(row=row, column=1, sticky="w", pady=6)
    tk.Label(parent, text=hint_text, bg=BG_BOX, fg=TXT_HINT, font=F_HINT,
              anchor="w", justify=tk.LEFT).grid(row=row, column=2, sticky="w", padx=10, pady=6)
    return holder


def style_axes(ax, ylabel, facecolor=BG_ROOT):
    """深色 matplotlib 座標軸配方，取自 gui_monitor.py:857-864。"""
    ax.set_facecolor(facecolor)
    ax.tick_params(colors=TXT_MUTED, labelsize=8)
    for sp in ax.spines.values():
        sp.set_color(SEP)
    ax.grid(color="#2a2a2a", linewidth=0.5, alpha=0.6)
    ax.set_ylabel(ylabel, color=TXT_DIM, fontsize=9)


def style_legend(ax, **kwargs):
    """深色圖例配方，取自 gui_monitor.py:886-889。"""
    kw = dict(loc="upper left", fontsize=7, facecolor=BG_BOX,
              edgecolor=SEP, labelcolor="#cccccc", ncol=3)
    kw.update(kwargs)
    leg = ax.legend(**kw)
    leg.get_frame().set_alpha(0.7)
    return leg


def config_console_tags(widget, tags):
    """批次設定 ScrolledText/Text 的 tag_config，tags: {name: {kwargs...}}。"""
    for name, kw in tags.items():
        widget.tag_config(name, **kw)


TERMINAL_TAGS = {
    "rate": dict(foreground=CYAN),
    "mag": dict(foreground=RED),
    "gps": dict(foreground=YELLOW),
    "ok": dict(foreground=GREEN_ALT),
    "err": dict(foreground="#dc3545", background="#2a0000"),
    "tele": dict(foreground=TXT_DIM),
    "lora": dict(foreground="#e07bfb"),
    "link": dict(foreground=GREEN),
    "ack": dict(foreground=CYAN_HI, background="#00303a"),
    # 電梯測試 profile 警告（見本檔下方 spam_elevator_console_warning）。放進共用表，
    # 凡是用 config_console_tags(TERMINAL_TAGS) 的 GUI 都自動拿到，不必各自註冊。
    "elevator_warn": dict(foreground="#000000", background=RED, font=F_MONO_BOLD),
}
EVENT_TAGS = {
    "cmd": dict(foreground=CYAN),
    "resp": dict(foreground="#33ff88"),
    "boot": dict(foreground="#e07bfb"),
    "err": dict(foreground="#ff5b5b", background="#2a0000"),
}


def append_console(widget, text, tag=None, max_lines=800):
    """append + 行數上限裁切 + 捲到底，取自兩支小 GUI 既有的 `append_console`。"""
    widget.insert(tk.END, text, tag)
    num_lines = float(widget.index('end-1c').split('.')[0])
    if num_lines > max_lines:
        widget.delete('1.0', '2.0')
    widget.see(tk.END)


def install_signal_handlers(root, on_close):
    """修 Tk 在 macOS 下 Ctrl+C 觸發 Fatal Python error/SIGABRT 的問題。
    逐字取自 gui_monitor.py:4664-4673 的說明與作法。"""
    signal.signal(signal.SIGINT, lambda *_: root.after(0, on_close))
    signal.signal(signal.SIGTERM, lambda *_: root.after(0, on_close))


# ==================== 電梯測試 profile 醒目警示（三支 GUI 共用） ====================
# 只要主/副任一板仍以 board_config.h FLIGHT_PROFILE_ELEVATOR=1 編譯（正式版已改回 0，
# 這代表忘記重燒/換錯板），就必須讓操作員在任何一支 GUI 上都不可能漏看——閃爍紅/黃
# 頂欄橫幅 + 主控台每收一筆就洗版警告。三支 GUI（gui_monitor/GUI_avionic/ground_gui）
# 資料來源不同（直連 USB 文字行 vs. 經地面站板 LoRa 中繼的 [GS_PKT] 行），但視覺與
# log 語彙統一由這裡出，不重新發明。
ELEVATOR_BANNER_TEXT = "🚨 ELEVATOR TEST PROFILE ACTIVE — 電梯測試 profile 尚未切回正式版，禁止真實發射 DO NOT LAUNCH 🚨"
ELEVATOR_WARN_TAG_CFG = dict(foreground="#000000", background=RED, font=F_MONO_BOLD)


def make_elevator_banner(parent):
    """建立橫幅（預設不 pack，由 show/hide 控制顯示）。呼叫端在最上層容器建立後立刻呼叫，
    之後每收到一筆遙測/狀態行就呼叫 update_elevator_banner() 依偵測結果顯示/隱藏。"""
    banner = tk.Label(parent, text=ELEVATOR_BANNER_TEXT, font=F_HDR, fg="#000000", bg=RED,
                       anchor="center", pady=6, cursor="")
    banner._blinking = False
    banner._blink_on = False
    return banner


def _blink_elevator_banner(root, banner):
    # 用 winfo_manager()（"pack"／""）判斷是否仍掛在版面上，不可用 winfo_ismapped()：
    # 視窗被 withdraw/iconify、或 pack 後尚未跑到下一輪 idle 時 ismapped 都是 False，
    # 會讓閃爍誤停、隱藏誤判成「已經隱藏」而留下過期的警示橫幅。
    if not banner.winfo_manager():
        banner._blinking = False
        return
    banner._blink_on = not banner._blink_on
    banner.config(bg=YELLOW if banner._blink_on else RED,
                   fg="#000000")
    root.after(500, lambda: _blink_elevator_banner(root, banner))


def update_elevator_banner(root, banner, active, who_text=""):
    """active=True 時顯示（並啟動閃爍，若尚未啟動）、False 時隱藏並停止閃爍。
    who_text 例如 "本板"/"對端(副板)"/"本板+對端(副板)"，併入橫幅文字方便辨識是哪一板。"""
    if active:
        text = ELEVATOR_BANNER_TEXT + (f"\n（{who_text}）" if who_text else "")
        banner.config(text=text)
        if not banner.winfo_manager():   # 見 _blink_elevator_banner 註解：不可用 winfo_ismapped()
            sib = _first_sibling(banner)
            if sib is not None:
                banner.pack(side=tk.TOP, fill=tk.X, before=sib)
            else:
                banner.pack(side=tk.TOP, fill=tk.X)
        if not banner._blinking:
            banner._blinking = True
            _blink_elevator_banner(root, banner)
    else:
        if banner.winfo_manager():
            banner.pack_forget()


def _first_sibling(widget):
    """回傳 widget 所在容器內目前排在最前面的其他子元件，供 pack(before=...) 把橫幅擠到最上方；
    容器目前空的（banner 尚未 pack 過）則回傳 None。"""
    siblings = [w for w in widget.master.pack_slaves() if w is not widget]
    return siblings[0] if siblings else None


def spam_elevator_console_warning(console_widget, who_text="", tag="elevator_warn"):
    """在主控台洗一行醒目警告（呼叫端需先對 console_widget 執行一次
    `console_widget.tag_config("elevator_warn", **gui_theme.ELEVATOR_WARN_TAG_CFG)`）。
    刻意每收一筆觸發就洗一次（而非只在邊緣觸發時提示一次）——「瘋狂提示」，避免操作員
    捲動略過就忘記。"""
    msg = f"🚨🚨🚨 [ELEVATOR TEST WARNING] {who_text or '仍為電梯測試 profile'} — 嚴禁真實發射 DO NOT LAUNCH FOR REAL FLIGHT 🚨🚨🚨\n"
    append_console(console_widget, msg, tag)


# ==================== 指令傳送（逐字慢送，抽自 gui_monitor.py:2441-2491） ====================
class CommandSender:
    """板端命令台是 20ms 低優先權輪詢、1 byte 緩衝，不能整串 burst 送——必須逐字慢送
    （CMD_CHAR_GAP 秒/字）。可選一個「安全鎖」`lock_var`（tk.BooleanVar）：未勾選時
    一律擋下並跳出 `lock_warning`（title, message）。"""

    CMD_CHAR_GAP = 0.1

    def __init__(self, get_ser, get_running, console_echo=None, event_log=None,
                 lock_var=None, lock_warning=None):
        self._get_ser = get_ser
        self._get_running = get_running
        self._console_echo = console_echo
        self._event_log = event_log
        self._lock_var = lock_var
        self._lock_warning = lock_warning
        self._tx_queue = queue.Queue()
        self._tx_thread = None

    def send(self, cmd_str, parent=None, bypass_lock=False):
        """bypass_lock=True：跳過 LNA 安全鎖，僅供呼叫端明確知道這個指令不會讓 LoRa
        模組真的發射 RF（本地查詢/本地參數/本地 Flash 操作）時使用——呼叫端自行負責
        判斷，這裡不做字串內容檢查。預設 False（沿用既有「未解鎖一律擋下」行為）。"""
        if not bypass_lock and self._lock_var is not None and not self._lock_var.get():
            if self._lock_warning:
                messagebox.showwarning(*self._lock_warning, parent=parent)
            return False
        if not self._get_running() or not self._get_ser():
            messagebox.showwarning("警告", "串口未連接！請先連線。", parent=parent)
            return False
        if not cmd_str.endswith('\n'):
            cmd_str += '\n'
        # 前置換行：先終止板端命令緩衝內殘留的半截命令，避免「垃圾+命令」黏在一起解析失敗。
        cmd_str = '\n' + cmd_str

        if self._tx_thread is None or not self._tx_thread.is_alive():
            self._tx_thread = threading.Thread(target=self._worker, daemon=True)
            self._tx_thread.start()
        self._tx_queue.put(cmd_str)

        if self._console_echo:
            self._console_echo(f"[CMD] ➡️ {cmd_str.strip()}（背景慢送中…）\n", "rate")
        if self._event_log:
            self._event_log(f"[CMD] ➡️ {cmd_str.strip()}", "cmd")
        return True

    def _worker(self):
        while True:
            cmd = self._tx_queue.get()
            if cmd is None:
                return
            data = cmd.encode('utf-8')
            for i in range(len(data)):
                if not self._get_running() or not self._get_ser():
                    break
                ser = self._get_ser()
                try:
                    ser.write(data[i:i + 1])
                    ser.flush()
                except Exception:
                    break
                time.sleep(self.CMD_CHAR_GAP)


# ==================== 開傘 latch（本板／對端「曾經觸發過」追蹤） ====================
class DeployLatch:
    """開傘狀態單向鎖存：FSM 狀態單向前進，副傘馬達 8 秒後斷電、flags bit 會歸零，
    但那不代表沒點火，只是不會再點第二次——不能只看即時 flags，要 latch。
    邏輯抽自 gui_monitor.py:2964-2991 `_mark_deploy_flag`/`_mark_deploy`/`_refresh_deploy_label`。"""

    def __init__(self, on_change=None):
        self.state = {"self_drogue": False, "self_main": False,
                       "peer_drogue": False, "peer_main": False}
        self._on_change = on_change

    def mark_flag(self, prefix, which):
        key = f"{prefix}_{which}"
        if not self.state[key]:
            self.state[key] = True
            if self._on_change:
                self._on_change(self.state)

    def mark_by_order(self, prefix, order):
        """order>=5 蘊含副傘已觸發，order>=8 蘊含主傘已觸發（FSM 單向前進語意）。"""
        if order is None:
            return
        if order >= 5:
            self.mark_flag(prefix, "drogue")
        if order >= 8:
            self.mark_flag(prefix, "main")

    def label_text(self):
        d = self.state
        mark = lambda v: "✓" if v else "○"
        text = (f"🪂 開傘紀錄  本板[副:{mark(d['self_drogue'])} 主:{mark(d['self_main'])}]"
                f"  對端[副:{mark(d['peer_drogue'])} 主:{mark(d['peer_main'])}]")
        color = "#ff8080" if any(d.values()) else TXT_MUTED
        return text, color


class PadRefTracker:
    """發射台氣壓零點（pad_ref）追蹤器 —— 「航電認為自己相對起點多高」的分母。

    韌體側（main.c FSM_Update()）：`state <= STATE_PAD` 期間每 30s 用**單筆**氣壓快照
    重零，之後 `baro_alt_rel = baro_alt - pad_ref` 就是 FSM 起飛／頂點／開傘判斷實際吃的
    高度。注意重零停止的時機是 **ARM（STATE_PAD_ARMED=2 > STATE_PAD=1）**，不是韌體註解
    寫的「起飛後」——ARM 完就凍結了。

    唯一來源是 USB 文字行 `[FSM] pad_ref locked: <cm> cm (was first|refresh)`。LoRa 下鏈
    的 `baro_alt_cm` 送的是**絕對海拔**（telemetry.c 填 baro_data.altitude），封包裡沒有
    pad_ref 也沒有相對值，所以純走 LoRa 的地面站 GUI 用不到本追蹤器。

    GUI 端讀不到 fsm_state 時，用「距上次重零超過 FROZEN_AFTER_S」反推已凍結（正常重零
    週期 30s，留 10s 餘裕吸收排程抖動與掉行）。
    """

    RE_LOCK = re.compile(r"\[FSM\]\s*pad_ref locked:\s*(-?\d+)\s*cm(?:\s*\(was\s+(\w+)\))?")
    FROZEN_AFTER_S = 40.0

    def __init__(self):
        self.pad_ref_m = None    # 最近一次鎖定的零點（m，絕對海拔尺度）
        self.locked_at = None    # time.monotonic()，供計算「幾秒前重零」
        self.lock_count = 0      # 已看到幾次重零（含首次）

    def feed(self, line):
        """餵一行終端輸出；命中 pad_ref 鎖定行時更新狀態並回傳 True。"""
        m = self.RE_LOCK.search(line)
        if not m:
            return False
        self.pad_ref_m = int(m.group(1)) / 100.0
        self.locked_at = time.monotonic()
        self.lock_count += 1
        return True

    @property
    def valid(self):
        return self.pad_ref_m is not None

    @property
    def age_s(self):
        return None if self.locked_at is None else time.monotonic() - self.locked_at

    @property
    def frozen(self):
        """已停止 30s 重零（多半代表已 ARM）。零點未知時不算凍結。"""
        age = self.age_s
        return age is not None and age > self.FROZEN_AFTER_S

    def rel(self, abs_alt_m):
        """絕對氣壓高度 → 相對發射台高度；零點還沒建立時回 None（呼叫端別畫）。"""
        return None if self.pad_ref_m is None else abs_alt_m - self.pad_ref_m

    def label_text(self):
        """卡片文字 + 顏色：`47.6 m ｜12s` / `47.6 m ｜已凍結` / `-- m`。"""
        if not self.valid:
            return "-- m", TXT_MUTED
        if self.frozen:
            return f"{self.pad_ref_m:.1f} m ｜已凍結", AMBER
        return f"{self.pad_ref_m:.1f} m ｜{self.age_s:.0f}s", GREEN
