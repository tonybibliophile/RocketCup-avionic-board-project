#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
航電板 3D 即時姿態儀與終端監控儀 (Premium Dark Mode Dashboard)

功能特色:
  1. 自動依賴安裝：啟動時自動檢測並安裝 pyserial, numpy, matplotlib 套件。
  2. 執行緒安全設計：背景執行緒負責 UART 高速收發（支援 460800 鮑率），透過 Thread-safe Queue 與 GUI 交互。
  3. 即時 3D 姿態渲染：解析 EKF [TELE] 的四元數 q:qw,qx,qy,qz，以 Matplotlib 3D 繪製火箭姿態（圓柱體身+圓錐體頭+4個穩定翼）。
  4. 雙窗格儀表板：
     - 左側：終端滾動日誌，支援重要關鍵字高亮（RATE、MAG、GPS、OK、ERROR）。
     - 右側：即時 3D 火箭姿態，支援視角旋轉與暫停/繼續渲染。
  5. 頂部狀態列：數位卡片顯示 BMI088、ADXL、BMP、MAG、GPS 最新採樣率及 Flash 寫入包數。
  6. 串口自動偵測：自動掃描 Mac 上的 USB 串口（支援 cu.usbserial 等）。
"""

import sys
import os
import time
import re
import queue
import csv
import signal
import threading
from datetime import datetime
from collections import deque

# ==================== 1. 自動安裝依賴套件 ====================
def install_and_import(package, import_name=None):
    if import_name is None:
        import_name = package
    try:
        __import__(import_name)
    except ImportError:
        print(f"[*] 偵測到未安裝 {package}，正在自動進行 pip 安裝...")
        try:
            subprocess = __import__("subprocess")
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])
            print(f"[+] {package} 安裝成功！")
        except Exception as e:
            print(f"[-] 安裝 {package} 失敗，請手動執行: pip install {package}。錯誤: {e}")
            sys.exit(1)

install_and_import("pyserial", "serial")
install_and_import("numpy")
install_and_import("matplotlib")

# 選配：GPS 地圖元件（OpenStreetMap 圖磚，線上載入）。安裝失敗不擋主程式，
# GPS 分頁會自動退回「相對軌跡圖」（原點=第一筆定位，離線可用）。
try:
    import tkintermapview
    HAVE_MAPVIEW = True
except ImportError:
    try:
        __import__("subprocess").check_call([sys.executable, "-m", "pip", "install", "tkintermapview"])
        import tkintermapview
        HAVE_MAPVIEW = True
    except Exception:
        tkintermapview = None
        HAVE_MAPVIEW = False
        print("[*] tkintermapview 未安裝（無網路？），GPS 地圖改用相對軌跡圖。")

# 成功載入依賴套件
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure   # 圖表獨立視窗用：不登記進 pyplot 全域 Gcf manager，開關視窗不漏 Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter.scrolledtext import ScrolledText

# 本檔刻意不吃 gui_theme 的配色／版面（見該檔 docstring），只借共用「規則」：
# 發射台氣壓零點的重零／凍結判定規則，三支 GUI 必須一致；電梯測試 profile 醒目警示
# 同理——是否顯示/怎麼措辭三支 GUI 必須一致，複製兩份遲早會走鐘，不重新發明。
# 放在依賴自動安裝之後 —— gui_theme 會 import numpy/tkinter。
from gui_theme import (PadRefTracker, make_elevator_banner, update_elevator_banner,
                        spam_elevator_console_warning, ELEVATOR_WARN_TAG_CFG,
                        make_flash_banner, update_flash_banner)

# Add parent directory to sys.path to load serial_link
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import serial_link   # P3：port/baud 預設與自動偵測統一於此

# ==================== 2. 全域配置參數 ====================
DEFAULT_BAUD = serial_link.DEFAULT_BAUD
DEFAULT_PORT = serial_link.PREFERRED_PORT

FLASH_EXPORT_COLUMNS = [
    "addr", "flight_id", "seq", "tick_ms", "fsm_state", "flags", "bat_mv",
    "bmi_ax", "bmi_ay", "bmi_az", "bmi_gx", "bmi_gy", "bmi_gz",
    "adxl_x", "adxl_y", "adxl_z",
    "baro_temp_c_x100", "baro_press_pa", "baro_alt_cm",
    "ekf_alt_cm", "ekf_vel_cms",
    "ekf_q0", "ekf_q1", "ekf_q2", "ekf_q3",
    "gps_lat", "gps_lon", "gps_alt_m", "gps_spd_cms", "gps_sats", "gps_fix",
]
FLASH_EXPORT_NUMERIC_RE = re.compile(r"^-?\d+(?:\.\d+)?$")
FLASH_EXPORT_ADDR_RE = re.compile(r"^0x[0-9A-Fa-f]{6}$")
FLASH_SYSFLAGS_HEX_RE = re.compile(r"^[0-9A-Fa-f]{6}:(?: [0-9A-Fa-f]{2}){16}$")

# ==================== 3. 3D 幾何生成工具 ====================
def generate_rocket_geometry(r=0.25, h_body=1.8, h_nose=0.8):
    """
    在局部座標系（火箭長軸為 Z 軸）中生成圓柱體身（三分段）、圓錐體頭、黑色噴嘴、尾焰與 3D 穩定翼的頂點
    """
    theta = np.linspace(0, 2*np.pi, 24)

    # 1. 圓柱體身分段 (Cylinder segments)
    # Segment 1: 下段 (Lower body, Z: -h_body/2 -> -h_body/6)
    z_b1 = np.linspace(-h_body/2, -h_body/6, 6)
    theta_grid, z_grid_b1 = np.meshgrid(theta, z_b1)
    x_grid_b1 = r * np.cos(theta_grid)
    y_grid_b1 = r * np.sin(theta_grid)

    # Segment 2: 中段 (Mid stripe, Z: -h_body/6 -> h_body/6)
    z_b2 = np.linspace(-h_body/6, h_body/6, 6)
    theta_grid, z_grid_b2 = np.meshgrid(theta, z_b2)
    x_grid_b2 = r * np.cos(theta_grid)
    y_grid_b2 = r * np.sin(theta_grid)

    # Segment 3: 上段 (Upper body, Z: h_body/6 -> h_body/2)
    z_b3 = np.linspace(h_body/6, h_body/2, 6)
    theta_grid, z_grid_b3 = np.meshgrid(theta, z_b3)
    x_grid_b3 = r * np.cos(theta_grid)
    y_grid_b3 = r * np.sin(theta_grid)

    # 2. 圓錐體頭 (Nose Cone, Z: h_body/2 -> h_body/2 + h_nose)
    z_nose = np.linspace(h_body/2, h_body/2 + h_nose, 8)
    theta_grid_n, z_grid_n = np.meshgrid(theta, z_nose)
    r_taper = r * ( (h_body/2 + h_nose) - z_grid_n ) / h_nose
    x_grid_n = r_taper * np.cos(theta_grid_n)
    y_grid_n = r_taper * np.sin(theta_grid_n)

    # 3. 噴嘴 (Nozzle Cone, Z: -h_body/2 - 0.2 -> -h_body/2)
    z_nozzle = np.linspace(-h_body/2 - 0.2, -h_body/2, 5)
    theta_grid_noz, z_grid_noz = np.meshgrid(theta, z_nozzle)
    r_noz_taper = r * 0.75 + (r * 0.25) * (z_grid_noz - (-h_body/2 - 0.2)) / 0.2
    x_grid_noz = r_noz_taper * np.cos(theta_grid_noz)
    y_grid_noz = r_noz_taper * np.sin(theta_grid_noz)

    # 4. 引擎火光 (Thrust Plume Base, Z: -0.9 -> 0.0, top is 0.0 at nozzle base)
    z_flame = np.linspace(-0.9, 0.0, 8)
    theta_grid_fl, z_grid_fl = np.meshgrid(theta, z_flame)
    r_fl_taper = r * 0.65 * (z_grid_fl - (-0.9)) / 0.9
    x_grid_fl = r_fl_taper * np.cos(theta_grid_fl)
    y_grid_fl = r_fl_taper * np.sin(theta_grid_fl)

    # 5. 四個穩定翼 (Fins - 3D 梯形面，每個鰭翼由 4 個頂點定義的 3D 多邊形)
    fins = [
        # Fin 1: X+
        np.array([[r, 0, -h_body/2], [r + 0.4, 0, -h_body/2], [r + 0.3, 0, -h_body/2 + 0.45], [r, 0, -h_body/2 + 0.45]]),
        # Fin 2: X-
        np.array([[-r, 0, -h_body/2], [-r - 0.4, 0, -h_body/2], [-r - 0.3, 0, -h_body/2 + 0.45], [-r, 0, -h_body/2 + 0.45]]),
        # Fin 3: Y+
        np.array([[0, r, -h_body/2], [0, r + 0.4, -h_body/2], [0, r + 0.3, -h_body/2 + 0.45], [0, r, -h_body/2 + 0.45]]),
        # Fin 4: Y-
        np.array([[0, -r, -h_body/2], [0, -r - 0.4, -h_body/2], [0, -r - 0.3, -h_body/2 + 0.45], [0, -r, -h_body/2 + 0.45]])
    ]

    return (x_grid_b1, y_grid_b1, z_grid_b1,
            x_grid_b2, y_grid_b2, z_grid_b2,
            x_grid_b3, y_grid_b3, z_grid_b3,
            x_grid_n, y_grid_n, z_grid_n,
            x_grid_noz, y_grid_noz, z_grid_noz,
            x_grid_fl, y_grid_fl, z_grid_fl,
            fins)

def rotate_points(x, y, z, R):
    """將網格點套用 3D 旋轉矩陣 R"""
    orig_shape = x.shape
    pts = np.vstack((x.flatten(), y.flatten(), z.flatten()))
    rot_pts = R @ pts
    rx = rot_pts[0].reshape(orig_shape)
    ry = rot_pts[1].reshape(orig_shape)
    rz = rot_pts[2].reshape(orig_shape)
    return rx, ry, rz

def quaternion_to_matrix(q):
    """將四元數 [qw, qx, qy, qz] 轉換為旋轉矩陣 R (航空慣用基準)"""
    w, x, y, z = q
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-6:
        return np.eye(3)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])
    return R

def quaternion_to_euler(q):
    """
    將四元數 [qw, qx, qy, qz] 轉換為歐拉角 Roll, Pitch, Yaw (以度為單位)
    採用航太常用 Z-Y-X 順序 (Tait-Bryan angles)
    """
    w, x, y, z = q
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # Roll (X-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (Y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (Z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    # 轉換成角度，並將 Yaw 映射到 0~360 度
    deg_yaw = np.degrees(yaw)
    if deg_yaw < 0:
        deg_yaw += 360.0
        
    return np.degrees(roll), np.degrees(pitch), deg_yaw


def _generate_static_ground_geometry():
    """發射台地平面/同心圈/方位射線/水平全息環——這些點位跟姿態四元數無關，每幀都不變。
    update_3d_plot() 之前每幀（5Hz）重算 np.linspace/meshgrid/cos/sin，抽成模組層級常數
    只算一次，畫圖時直接吃現成陣列（逐字同步 gui_attitude3d.py 的同一份優化）。"""
    z_ground = -1.2
    r_vals = np.linspace(0, 2.0, 15)
    theta_vals = np.linspace(0, 2 * np.pi, 48)
    R_grid, T_grid = np.meshgrid(r_vals, theta_vals)
    xg = R_grid * np.cos(T_grid)
    yg = R_grid * np.sin(T_grid)
    zg = np.full_like(xg, z_ground)

    cx = 2.0 * np.cos(theta_vals)
    cy = 2.0 * np.sin(theta_vals)
    cz = np.full_like(cx, z_ground)

    rings = []
    for r_c in (0.7, 1.4):
        cx_c = r_c * np.cos(theta_vals)
        cy_c = r_c * np.sin(theta_vals)
        cz_c = np.full_like(cx_c, z_ground)
        rings.append((cx_c, cy_c, cz_c))

    compass_lines = []
    for angle in (0, 45, 90, 135, 180, 225, 270, 315):
        rad = np.radians(angle)
        compass_lines.append(([0, 2.0 * np.cos(rad)], [0, 2.0 * np.sin(rad)], [z_ground, z_ground]))

    th_h = np.linspace(0, 2 * np.pi, 60)
    hx = 0.65 * np.cos(th_h)
    hy = 0.65 * np.sin(th_h)
    hz = np.zeros_like(th_h)
    horizon_surf = (np.array([np.zeros_like(th_h), hx]),
                     np.array([np.zeros_like(th_h), hy]),
                     np.array([np.zeros_like(th_h), hz]))

    return {
        "z_ground": z_ground, "xg": xg, "yg": yg, "zg": zg,
        "cx": cx, "cy": cy, "cz": cz, "rings": rings,
        "compass_lines": compass_lines,
        "hx": hx, "hy": hy, "hz": hz, "horizon_surf": horizon_surf,
    }


_STATIC_GROUND = _generate_static_ground_geometry()


# ==================== 3b. 跨平台自訂高對比按鈕 ====================
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
                bg=self.bg_hover if self.winfo_exists() and self.winfo_containing(self.winfo_pointerx(), self.winfo_pointery()) == self else self.bg_normal
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

# ==================== 4. 主 GUI 應用程式 ====================
class RocketDashboardApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Antigravity Rocket Avionics 3D Real-Time Dashboard")
        # 四象限同屏（log/圖表/3D/地圖）需要較大視窗：依螢幕自適應，小螢幕縮到可用範圍
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry(f"{min(1680, sw - 60)}x{min(1000, sh - 80)}")
        self.root.minsize(1150, 760)
        self.root.configure(bg="#151515")
        
        # 狀態配置
        self.serial_thread = None
        self.running = False
        self.paused = False
        self.data_queue = queue.Queue()
        self.fsm_state = "STATE_PAD"
        # FSM 狀態轉換事件（時間戳, 狀態名）：供 live chart 畫飛行階段色帶
        self.fsm_events = []  # list of (t_rel, state_str)
        # 開傘狀態 latch：本板/對端一旦「曾經」進入對應 FSM 狀態就鎖定顯示已觸發，
        # 不能只看即時 flags bit —— 副傘馬達只導通 8 秒(FSM_DROGUE_MOTOR_RUN_MS)就會
        # 斷電，telemetry 的 TELEM_FLAG_DROGUE_FIRED 之後會變回 0，但那不代表沒點火，
        # 只是不會再點第二次（FSM 狀態單向前進，見 fsm.c）。
        self.deploy_latch = {"self_drogue": False, "self_main": False,
                              "peer_drogue": False, "peer_main": False}

        # 角色自動偵測（PRIMARY 主航電 / BACKUP 備援航電 / GROUND 地面站）
        # 被動：解析 [BOOT]/[ROLE]/[ROLE_ID]/[GS_*] 特徵行；主動：連線後送 'role' 命令。
        self.detected_role = None
        self.detected_fw = ""
        self.detected_gs_tx = None   # GROUND 專用：None=未知, 0=RX-only, 1=TX-capable（純顯示，不做攔截）
        self.role_query_attempts = 0
        self.gs_tx_query_attempts = 0

        # 手動開傘 / 回收指令 seq → 標籤對照（由 [UPLINK] 送出行填入，供比對回傳 [ACK] 用；
        # ACK cmd 文字對三種開傘皆是通用 "deploy"，須靠 seq 才分得出副傘/主傘/雙傘）
        self._uplink_seq_label = {}

        # LoRa 參數現值快取（由 [E80]/[E22]/[LORA433]/[LORA 920MHz] 回報行解析）
        self.lora_e80_state = {}   # freq_hz, sf, bw_idx, bw_khz, cr, pwr, pre
        self.lora_e22_state = {}   # freq_mhz, ch, power, air_rate

        # Ground Station packet counters
        self.gs_pkt_cnt_433 = 0
        self.gs_pkt_cnt_920 = 0
        self.gs_pkt_cnt_total = 0

        # 飛行圖表時間序列：(t, 值)，t = time.monotonic() − chart_t0（秒）
        # 資料源：EKF=[TELE]/[GS_PKT]；原始=10Hz 裸 CSV 行(BMI/ADXL/Baro)、[GPS]、[GS_PKT] accel
        self.chart_t0 = time.monotonic()
        _N = 6000
        self.ts_alt_ekf  = deque(maxlen=_N)   # EKF 高度 m
        self.ts_alt_baro = deque(maxlen=_N)   # 氣壓高度 m，已扣 pad_ref → 相對起點（見 self.pad_ref）
        self.ts_alt_gps  = deque(maxlen=_N)   # GPS 海拔 m（原始）
        self.ts_vz_ekf   = deque(maxlen=_N)   # EKF 垂直速度 m/s
        self.ts_spd_gps  = deque(maxlen=_N)   # GPS 地速 m/s（原始）
        self.ts_acc_bmi  = deque(maxlen=_N)   # BMI088 |a| g（原始）
        self.ts_acc_adxl = deque(maxlen=_N)   # ADXL375 |a| g（原始）
        self.ts_vf_alt   = deque(maxlen=_N)   # 本板 VF 高度 m
        self.ts_vf_vz    = deque(maxlen=_N)   # 本板 VF 垂直速度 m/s
        self.ts_ground_alt = deque(maxlen=_N) # 本板 相對地面高度原始值 m（pad_ref 未濾波，每 30s 於 PAD 重零）
        self.pad_ref = PadRefTracker()        # 發射台氣壓零點；ts_alt_baro 逐點扣它
        self.ts_backup_alt  = deque(maxlen=_N) # 副航電 EKF 高度 m
        self.ts_backup_vz   = deque(maxlen=_N) # 副航電 EKF 垂直速度 m/s
        self.ts_backup_baro = deque(maxlen=_N) # 副航電 氣壓高度 m
        self.ts_backup_acc  = deque(maxlen=_N) # 副航電 高G 垂直加速度 g
        self.ts_backup_vf_alt = deque(maxlen=_N) # 副航電 VF 高度 m
        self.ts_backup_vf_vz  = deque(maxlen=_N) # 副航電 VF 垂直速度 m/s
        self.ts_backup_acc_bmi  = deque(maxlen=_N) # 副航電 BMI088 加速度模長 g（原始）
        self.ts_backup_acc_adxl = deque(maxlen=_N) # 副航電 ADXL375 加速度模長 g（原始）
        self.backup_state   = None             # 副航電實時狀態
        self.backup_fsm_events = []             # 副航電 FSM 狀態轉換（時間戳, 狀態名），供 live chart 同時標示主副航電
        self.chart_dirty = False
        self.chart_paused = False

        # GPS 地圖狀態
        self.gps_track = []       # 火箭軌跡 [(lat, lon)]
        self.gps_last = None      # 最新定位 {lat, lon, alt, spd, sats, fix}
        self.gs_own_pos = None    # 地面站自身 (lat, lon)（[GS_GPS]）
        self.map_home = None      # 第一筆定位（Home/原點）
        self.map_dirty = False
        self.map_zoomed = False   # 首筆定位時自動 zoom-in 一次
        
        # GPS 尋星與解包狀態
        self.gps_status_info = {
            "fix": 0,
            "sats": 0,
            "stale": 1,
            "ok": 0,
            "err": 0,
            "rate": 0.0
        }
        
        # 記錄檔配置
        self.log_file = None
        self.save_log_var = tk.BooleanVar(value=True)

        # 初始化 3D 幾何本地數據
        self.local_geom = generate_rocket_geometry()
        self.last_q = [1.0, 0.0, 0.0, 0.0]
        # 姿態重畫節流旗標：遙測每包都同步重畫會吃滿 GUI 執行緒（一幀約 50ms，
        # 而 poll_queue 每 10ms 最多吃 30 行），故比照圖表改為「設髒旗標 + 定時重畫」
        self.att_dirty = False
        
        # 暫存最新感測器數據以進行一致性測試
        self.latest_imu = None
        self.latest_highg = None
        self.latest_mag = None
        
        # 地磁計校正與鎖定狀態
        self.board_mag_offsets = [131072.0, 131072.0, 131072.0]
        self.calib_x = []
        self.calib_y = []
        self.collecting_data = False
        self.new_ox = 131072.0
        self.new_oy = 131072.0
        
        # 設置 UI 樣式與結構
        self.setup_styles()

        # 電梯測試 profile 醒目橫幅：預設隱藏，收到 [PAD_CFG]/瘋狂警告行才顯示（見對應 elif 分支）。
        self.elevator_banner = make_elevator_banner(self.root)
        # 三種資料來源（直連航電 [PAD_CFG]/[ELEVATOR_TEST_WARNING]、經地面站板 LoRa 中繼的
        # [GS_PKT] prof:）共用這兩個時間戳，用「最近看到」的新鮮度窗口判斷是否仍要顯示，
        # 避免任一來源各自直接切換橫幅互相蓋掉彼此狀態（同一個 process 同時只會接一種來源，
        # 但用時間戳統一收斂邏輯最單純）。0.0 = 從未看過 / 已被權威來源明確清除。
        self._elevator_self_ts = 0.0
        self._elevator_peer_ts = 0.0

        # ★2026-07-31：Flash 未擦除橫幅。航電開機不再自動擦除（改為使用者下 `flash erase`／
        # `flash pool`），池未達標時 ARM 會被擋——但在按下 ARM 之前完全沒有徵兆，所以這裡
        # 也用「最近看到」的新鮮度窗口顯示橫幅。兩個來源：直連航電的 [FLASH_NOT_READY] 行，
        # 以及經地面站板中繼的 [GS_PKT] armf: 位元。0.0 = 從未看過 / 已確認擦好。
        self.flash_banner = make_flash_banner(self.root)
        self._flash_need_erase_ts = 0.0
        self._flash_need_erase_detail = ""

        # ---- 頂部狀態列：兩排設計，避免右側標籤被截斷 ----
        top_container = tk.Frame(self.root, bg="#151515")
        top_container.pack(fill=tk.X, side=tk.TOP, padx=10, pady=(5, 0))

        # ── 第一排：飛行狀態指示 ──────────────────────────────
        top_bar = tk.Frame(top_container, bg="#151515")
        top_bar.pack(fill=tk.X, side=tk.TOP)

        # FSM state (固定寬度 24，防止微調動態跳動)
        self.lbl_fsm = tk.Label(top_bar, text="🚀 STATE: --", bg="#151515", fg="#00e5ff",
                                font=("Helvetica", 11, "bold"), width=24, anchor="w")
        self.lbl_fsm.pack(side=tk.LEFT, padx=4, pady=2)

        # 板角色 PRIMARY / BACKUP / GROUND (固定寬度 18)
        self.lbl_role = tk.Label(top_bar, text="ROLE: --", bg="#151515", fg="#aaaaaa",
                                 font=("Helvetica", 10, "bold"), width=18, anchor="w")
        self.lbl_role.pack(side=tk.LEFT, padx=4, pady=2)

        tk.Frame(top_bar, bg="#333333", width=1, height=18).pack(side=tk.LEFT, padx=4, fill=tk.Y)

        # 板間鏈路與主傘 PD14 共開狀態 (固定寬度 45，防文字裁切)
        self.lbl_link = tk.Label(top_bar, text="🔗 LINK: --", bg="#151515", fg="#555555",
                                 font=("Helvetica", 10, "bold"), width=45, anchor="w")
        self.lbl_link.pack(side=tk.LEFT, padx=4, pady=2)

        tk.Frame(top_bar, bg="#333333", width=1, height=18).pack(side=tk.LEFT, padx=4, fill=tk.Y)

        # 雙航電 Flash 預擦除進度 (固定寬度 38)
        self.lbl_erase = tk.Label(top_bar, text="💾 ERASE: --", bg="#151515", fg="#555555",
                                   font=("Helvetica", 10, "bold"), width=38, anchor="w")
        self.lbl_erase.pack(side=tk.LEFT, padx=4, pady=2)

        tk.Frame(top_bar, bg="#333333", width=1, height=18).pack(side=tk.LEFT, padx=4, fill=tk.Y)

        # 遙測中繼副板狀態 (固定寬度 34)
        self.lbl_peer = tk.Label(top_bar, text="🛸 PEER: --", bg="#151515", fg="#555555",
                                 font=("Helvetica", 10, "bold"), width=34, anchor="w")
        self.lbl_peer.pack(side=tk.LEFT, padx=4, pady=2)

        tk.Frame(top_bar, bg="#333333", width=1, height=18).pack(side=tk.LEFT, padx=4, fill=tk.Y)

        # E22 (433) status (固定寬度 12)
        self.lbl_lora433 = tk.Label(top_bar, text="📻 433: --", bg="#151515", fg="#555555",
                                    font=("Helvetica", 10, "bold"), width=12, anchor="w")
        self.lbl_lora433.pack(side=tk.LEFT, padx=4, pady=2)

        # E80 (920) status (固定寬度 12)
        self.lbl_lora920 = tk.Label(top_bar, text="📡 920: --", bg="#151515", fg="#555555",
                                    font=("Helvetica", 10, "bold"), width=12, anchor="w")
        self.lbl_lora920.pack(side=tk.LEFT, padx=4, pady=2)

        # Ground Station packet counter (固定寬度 28)
        self.lbl_gs_pkts = tk.Label(top_bar, text="📦 PKTS: 0 (433:0|920:0)", bg="#151515", fg="#00e5ff",
                                    font=("Helvetica", 10, "bold"), width=28, anchor="w")
        self.lbl_gs_pkts.pack(side=tk.LEFT, padx=6, pady=2)

        # ── 第二排：操控與連線 ──────────────────────────────
        bot_bar = tk.Frame(top_container, bg="#151515")
        bot_bar.pack(fill=tk.X, side=tk.TOP, pady=(2, 4))

        # ARM / DISARM 安全解鎖與狀態指示控制區
        arm_box = tk.Frame(bot_bar, bg="#1a1a1a", highlightbackground="#333333", highlightthickness=1, padx=4, pady=2)
        arm_box.pack(side=tk.LEFT, padx=(4, 6), pady=1)

        self.lbl_arm_status = tk.Label(arm_box, text="🛡️ DISARMED", bg="#1b4332", fg="#2ec4b6",
                                       font=("Helvetica", 9, "bold"), width=12, anchor="center", padx=4, pady=3)
        self.lbl_arm_status.pack(side=tk.LEFT, padx=(2, 4))

        self.btn_arm = StyledButton(arm_box, text="⚡ ARM", command=lambda: self.send_command("arm"),
                                    bg="#851414", hover_bg="#b91c1c", fg="#ffffff",
                                    font=("Helvetica", 9, "bold"), width=7, padx=4, pady=3)
        self.btn_arm.pack(side=tk.LEFT, padx=2)

        self.btn_disarm = StyledButton(arm_box, text="🛡️ DISARM", command=lambda: self.send_command("disarm"),
                                       bg="#27272a", hover_bg="#3f3f46", fg="#ffffff",
                                       font=("Helvetica", 9, "bold"), width=8, padx=4, pady=3)
        self.btn_disarm.pack(side=tk.LEFT, padx=2)

        # BENCH 桌面開傘測試獨立高亮控制區
        bench_box = tk.Frame(bot_bar, bg="#2d1b4e", highlightbackground="#8b5cf6", highlightthickness=2, padx=4, pady=2)
        bench_box.pack(side=tk.LEFT, padx=(4, 6), pady=1)

        self.btn_bench = StyledButton(bench_box, text="🖥️ BENCH TEST", command=self._on_bench_test,
                                      bg="#6d28d9", hover_bg="#8b5cf6", fg="#ffffff",
                                      font=("Helvetica", 9, "bold"), width=13, padx=6, pady=3)
        self.btn_bench.pack(side=tk.LEFT, padx=2)

        # 連接控制區（右側）
        conn_frame = tk.Frame(bot_bar, bg="#151515")
        conn_frame.pack(side=tk.RIGHT, pady=1)

        tk.Label(conn_frame, text="PORT:", bg="#151515", fg="#aaaaaa", font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=2)
        self.port_combo = ttk.Combobox(conn_frame, width=18, font=("Helvetica", 9))
        self.port_combo.pack(side=tk.LEFT, padx=2)

        btn_scan = ttk.Button(conn_frame, text="🔄", width=3, command=self.scan_ports)
        btn_scan.pack(side=tk.LEFT, padx=2)

        tk.Label(conn_frame, text="BAUD:", bg="#151515", fg="#aaaaaa", font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=2)
        self.baud_combo = ttk.Combobox(conn_frame, values=[9600, 38400, 115200, 460800, 921600], width=7, font=("Helvetica", 9))
        self.baud_combo.set(DEFAULT_BAUD)
        self.baud_combo.pack(side=tk.LEFT, padx=2)

        chk_log = tk.Checkbutton(conn_frame, text="LOG", variable=self.save_log_var, bg="#151515", fg="#00d2ff", selectcolor="#151515", font=("Helvetica", 9, "bold"))
        chk_log.pack(side=tk.LEFT, padx=4)

        self.btn_connect = ttk.Button(conn_frame, text="CONNECT", width=10, command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=4)

        # ── 第三排：手動開傘 / 落海回收確認（危險操作，獨立一排避免誤觸）─────────
        danger_bar = tk.Frame(top_container, bg="#151515")
        danger_bar.pack(fill=tk.X, side=tk.TOP, pady=(0, 4))

        # 手動開傘控制區（經地面站 433 上行，須先 ARM 或已在飛行中）
        deploy_box = tk.Frame(danger_bar, bg="#3b1414", highlightbackground="#dc2626", highlightthickness=2, padx=4, pady=2)
        deploy_box.pack(side=tk.LEFT, padx=(4, 6), pady=1)

        tk.Label(deploy_box, text="🪂 手動開傘", bg="#3b1414", fg="#ff8080",
                 font=("Helvetica", 9, "bold")).pack(side=tk.LEFT, padx=(2, 4))

        self.btn_deploy_drogue = StyledButton(deploy_box, text="副傘 DROGUE", command=lambda: self._on_deploy("drogue"),
                                              bg="#991b1b", hover_bg="#dc2626", fg="#ffffff",
                                              font=("Helvetica", 9, "bold"), padx=6, pady=3)
        self.btn_deploy_drogue.pack(side=tk.LEFT, padx=2)

        self.btn_deploy_main = StyledButton(deploy_box, text="主傘 MAIN", command=lambda: self._on_deploy("main"),
                                            bg="#991b1b", hover_bg="#dc2626", fg="#ffffff",
                                            font=("Helvetica", 9, "bold"), padx=6, pady=3)
        self.btn_deploy_main.pack(side=tk.LEFT, padx=2)

        self.btn_deploy_both = StyledButton(deploy_box, text="雙傘 BOTH", command=lambda: self._on_deploy("both"),
                                            bg="#991b1b", hover_bg="#dc2626", fg="#ffffff",
                                            font=("Helvetica", 9, "bold"), padx=6, pady=3)
        self.btn_deploy_both.pack(side=tk.LEFT, padx=2)

        self.lbl_deploy_status = tk.Label(deploy_box, text="尚未送出", bg="#1a1a1a", fg="#888888",
                                          font=("Helvetica", 9, "bold"), anchor="w", padx=6, pady=3)
        self.lbl_deploy_status.pack(side=tk.LEFT, padx=(6, 2))

        # 落海回收確認控制區（停蜂鳴器 + 安全關閉 SD/Flash 記錄；飛行中航電會拒絕）
        recovery_box = tk.Frame(danger_bar, bg="#0a2030", highlightbackground="#0ea5e9", highlightthickness=2, padx=4, pady=2)
        recovery_box.pack(side=tk.LEFT, padx=(4, 6), pady=1)

        self.btn_recovery = StyledButton(recovery_box, text="🔍 落海回收確認", command=self._on_recovery,
                                         bg="#0369a1", hover_bg="#0ea5e9", fg="#ffffff",
                                         font=("Helvetica", 9, "bold"), padx=6, pady=3)
        self.btn_recovery.pack(side=tk.LEFT, padx=2)

        self.lbl_recovery_status = tk.Label(recovery_box, text="尚未送出", bg="#1a1a1a", fg="#888888",
                                            font=("Helvetica", 9, "bold"), anchor="w", padx=6, pady=3)
        self.lbl_recovery_status.pack(side=tk.LEFT, padx=(6, 2))

        # 開傘狀態紀錄（被動顯示，非按鈕）：本板與對端「是否曾經觸發過」副傘/主傘。
        # 用 FSM 狀態單向前進 latch，不能只看即時 flags —— 副傘馬達 8 秒後斷電，
        # telemetry flags bit 會變回 0，但那不代表沒點火，只是不會再點第二次。
        deploy_status_box = tk.Frame(danger_bar, bg="#1a1a1a", highlightbackground="#f59e0b",
                                     highlightthickness=2, padx=4, pady=2)
        deploy_status_box.pack(side=tk.LEFT, padx=(4, 6), pady=1)

        self.lbl_deploy_latch = tk.Label(deploy_status_box,
                                         text="🪂 開傘紀錄  本板[副:○ 主:○]  對端[副:○ 主:○]",
                                         bg="#1a1a1a", fg="#888888",
                                         font=("Helvetica", 9, "bold"), anchor="w", padx=4, pady=3)
        self.lbl_deploy_latch.pack(side=tk.LEFT, padx=2)

        # ------------------ 頂部數字感測卡片 ------------------
        self.cards_frame = tk.Frame(self.root, bg="#151515")
        self.cards_frame.pack(fill=tk.X, side=tk.TOP, padx=15, pady=5)
        
        self.cards = {}
        card_labels = [
            ("BMI088 A", "bmi_a", "0.00 Hz", "#00d2ff"),
            ("BMI088 G", "bmi_g", "0.00 Hz", "#00d2ff"),
            ("ADXL375", "adxl", "0.00 Hz", "#ffcc00"),
            ("BMP388", "bmp", "0.00 Hz", "#28a745"),
            ("MMC5983", "mag", "0.00 Hz", "#ff3b30"),
            ("GPS Update", "gps", "0.00 Hz", "#e03bfb"),
            ("Flash PKTs", "flash_pkt", "0 Pkts", "#ffffff"),
            ("ALT 高度", "alt", "-- m", "#00e676"),
            ("相對起點 (baro)", "ground_alt", "-- m", "#ff9800"),
            ("零點 pad_ref", "pad_ref", "-- m", "#888888"),
            ("Vz 垂直速度", "vz", "-- m/s", "#00e676"),
            ("BATTERY 電池", "bat", "-- V", "#ffa500")
        ]
        
        for idx, (title, key, default, color) in enumerate(card_labels):
            self.cards_frame.columnconfigure(idx, weight=1, uniform="equal")
            card = ttk.Frame(self.cards_frame, style="Card.TFrame")
            card.grid(row=0, column=idx, padx=5, sticky="nsew")
            
            lbl_title = ttk.Label(card, text=title, style="Card.TLabel")
            lbl_title.pack(anchor="w", padx=8, pady=4)
            
            lbl_val = ttk.Label(card, text=default, font=("Helvetica", 13, "bold"), background="#222222", foreground=color)
            lbl_val.pack(anchor="w", padx=8, pady=4)
            
            self.cards[key] = lbl_val

        # ------------------ 中部四象限面板（log / 圖表 / 3D 姿態 / 地圖 同時可見） ------------------
        # 巢狀 PanedWindow：水平分左右兩欄，各欄再垂直分上下，四條分隔線皆可拖曳調整。
        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg="#151515",
                                   sashwidth=6, sashrelief="flat", bd=0)
        main_pane.pack(fill=tk.BOTH, expand=True, side=tk.TOP, padx=15, pady=10)

        left_pane = tk.PanedWindow(main_pane, orient=tk.VERTICAL, bg="#151515",
                                   sashwidth=6, sashrelief="flat", bd=0)
        right_pane = tk.PanedWindow(main_pane, orient=tk.VERTICAL, bg="#151515",
                                    sashwidth=6, sashrelief="flat", bd=0)
        main_pane.add(left_pane, width=660, minsize=380)
        main_pane.add(right_pane, minsize=420)

        # --- 左上: 終端日誌 (Terminal Console) ---
        left_frame = tk.Frame(left_pane, bg="#1e1e1e")

        # --- 指令/回應區：只顯示使用者指令與韌體對指令的回應（不含開機/系統/遙測）---
        tk.Label(left_frame, text=" ★ 指令 / 回應", bg="#1e1e1e",
                 fg="#ffcc00", font=("Monaco", 10, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        self.event_console = ScrolledText(left_frame, bg="#0d0d0d", fg="#e0e0e0",
                                          insertbackground="white", font=("Monaco", 9),
                                          borderwidth=0, highlightthickness=1,
                                          highlightbackground="#3a3a1a", height=9)
        self.event_console.pack(fill=tk.X, padx=5, pady=(2, 6))
        self.event_console.tag_config("cmd", foreground="#00d2ff")
        self.event_console.tag_config("resp", foreground="#33ff88")
        self.event_console.tag_config("boot", foreground="#e07bfb")
        self.event_console.tag_config("err", foreground="#ff5b5b", background="#2a0000")

        tk.Label(left_frame, text=" > TERMINAL TELEMETRY STREAM（完整資訊流）", bg="#1e1e1e", fg="#00d2ff", font=("Monaco", 10, "bold")).pack(anchor="w", padx=10, pady=5)

        # 終端底部按鈕區（先 pack 於底部，讓 console 撐滿剩餘空間）
        console_tools = tk.Frame(left_frame, bg="#1e1e1e")
        console_tools.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=5)

        StyledButton(console_tools, text="清除終端", command=self.clear_console, bg="#2d2d2d", hover_bg="#3d3d3d", padx=8, pady=4).pack(side=tk.LEFT, padx=3)
        StyledButton(console_tools, text="📈 飛行圖表", command=self.open_charts_window, bg="#00435a", hover_bg="#005f7f", padx=10, pady=4).pack(side=tk.LEFT, padx=3)
        StyledButton(console_tools, text="📡 LoRa 參數設定", command=self.open_lora_panel, bg="#00435a", hover_bg="#005f7f", padx=10, pady=4).pack(side=tk.LEFT, padx=3)
        StyledButton(console_tools, text="🧭 軸向對齊測試", command=self.open_test_wizard, bg="#2d2d2d", hover_bg="#3d3d3d", padx=10, pady=4).pack(side=tk.LEFT, padx=3)
        StyledButton(console_tools, text="🧲 磁強計校正與鎖定", command=self.open_mag_calibration, bg="#2d2d2d", hover_bg="#3d3d3d", padx=10, pady=4).pack(side=tk.LEFT, padx=3)
        StyledButton(console_tools, text="💾 Flash 紀錄管理", command=self.open_flash_panel, bg="#005577", hover_bg="#007799", padx=10, pady=4).pack(side=tk.LEFT, padx=3)

        # 大字即時讀數（原本在 build_charts_tab 裡；圖表移到獨立視窗後這三個留在主視窗——
        # 飛行中最重要的即時數字不能被藏進另一個可能沒開的視窗）。由 _update_big_readouts()
        # 每次 charts_redraw_loop tick 更新，不受圖表視窗是否開啟影響。
        self.lbl_big_alt = tk.Label(console_tools, text="ALT --.- m", bg="#1e1e1e", fg="#00e676", font=("Monaco", 11, "bold"))
        self.lbl_big_alt.pack(side=tk.LEFT, padx=(14, 6))
        self.lbl_big_vz = tk.Label(console_tools, text="Vz --.- m/s", bg="#1e1e1e", fg="#00e5ff", font=("Monaco", 11, "bold"))
        self.lbl_big_vz.pack(side=tk.LEFT, padx=(0, 6))
        self.lbl_big_acc = tk.Label(console_tools, text="|a| --.- g", bg="#1e1e1e", fg="#ffcc00", font=("Monaco", 11, "bold"))
        self.lbl_big_acc.pack(side=tk.LEFT)

        self.lbl_drops = tk.Label(console_tools, text="EKF Queue Drops: 0", bg="#1e1e1e", fg="#aaaaaa", font=("Monaco", 9))
        self.lbl_drops.pack(side=tk.RIGHT, padx=10)

        # --- 手動指令列：自己打指令送到航電板（回應會隨遙測串流進終端）---
        cmd_bar = tk.Frame(left_frame, bg="#1e1e1e")
        cmd_bar.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=(0, 4))
        tk.Label(cmd_bar, text="指令 ➤", bg="#1e1e1e", fg="#00d2ff",
                 font=("Monaco", 10, "bold")).pack(side=tk.LEFT, padx=(6, 4))
        self._cmd_history = []
        self._cmd_history_idx = 0
        self.cmd_entry = tk.Entry(cmd_bar, bg="#0e0e0e", fg="#33ff33",
                                  insertbackground="white", font=("Monaco", 10),
                                  relief="flat", highlightthickness=1,
                                  highlightbackground="#333333", highlightcolor="#00d2ff")
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.cmd_entry.bind("<Return>", self._on_manual_cmd)
        self.cmd_entry.bind("<Up>", self._manual_cmd_history_prev)
        self.cmd_entry.bind("<Down>", self._manual_cmd_history_next)
        StyledButton(cmd_bar, text="送出", command=self._on_manual_cmd, bg="#006699", hover_bg="#0088cc", padx=10, pady=3).pack(side=tk.LEFT, padx=4)
        StyledButton(cmd_bar, text="help", command=lambda: self.send_command("help"), bg="#2a2a2a", hover_bg="#3a3a3a", fg="#cccccc", font=("Monaco", 9), padx=8, pady=3).pack(side=tk.LEFT, padx=(2, 4))

        # 終端文本框，設定為深色主題
        self.console = ScrolledText(left_frame, bg="#101010", fg="#33ff33", insertbackground="white", font=("Monaco", 9), borderwidth=0, highlightthickness=0)
        self.console.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 字體高亮色彩規則
        self.console.tag_config("rate", foreground="#00d2ff")
        self.console.tag_config("mag", foreground="#ff3b30")
        self.console.tag_config("gps", foreground="#ffcc00")
        self.console.tag_config("ok", foreground="#28a745")
        self.console.tag_config("err", foreground="#dc3545", background="#2a0000")
        self.console.tag_config("tele", foreground="#aaaaaa")
        self.console.tag_config("lora", foreground="#e07bfb")
        self.console.tag_config("link", foreground="#00e676")
        self.console.tag_config("ack", foreground="#00e5ff", background="#00303a")
        self.console.tag_config("elevator_warn", **ELEVATOR_WARN_TAG_CFG)

        left_pane.add(left_frame, minsize=180)   # 圖表移到獨立視窗後，log 欄撐滿整個左側直欄

        # --- 右上: 3D 姿態 ---
        att_frame = tk.Frame(right_pane, bg="#1e1e1e")
        title_3d_frame = tk.Frame(att_frame, bg="#1e1e1e")
        title_3d_frame.pack(fill=tk.X, side=tk.TOP, padx=10, pady=5)

        tk.Label(title_3d_frame, text=" > 3D ROCKET ATTITUDE (EKF ESTIMATE)", bg="#1e1e1e", fg="#00d2ff", font=("Monaco", 10, "bold")).pack(side=tk.LEFT)

        self.btn_pause = ttk.Button(title_3d_frame, text="暫停渲染", width=8, command=self.toggle_pause)
        self.btn_pause.pack(side=tk.RIGHT, padx=5)

        ttk.Button(title_3d_frame, text="重設視角", width=8, command=self.reset_view).pack(side=tk.RIGHT, padx=5)

        # Matplotlib 3D 畫布整合
        self.fig = plt.figure(facecolor="#1e1e1e")
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor("#1e1e1e")
        self.reset_view()

        self.canvas = FigureCanvasTkAgg(self.fig, master=att_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        right_pane.add(att_frame, height=430, minsize=240)

        # --- 右下: GPS 即時地圖 ---
        map_frame = tk.Frame(right_pane, bg="#1e1e1e")
        tk.Label(map_frame, text=" > GPS LIVE MAP", bg="#1e1e1e", fg="#00d2ff",
                 font=("Monaco", 10, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        self.build_map_tab(map_frame)
        right_pane.add(map_frame, minsize=220)

        # 初始化靜態 3D 火箭渲染
        self.update_3d_plot([1.0, 0.0, 0.0, 0.0])

        # 3D 姿態 / 圖表 / 地圖獨立刷新迴圈（有新資料才重繪）
        self.root.after(500, self.attitude_redraw_loop)
        self.root.after(400, self.charts_redraw_loop)
        self.root.after(1000, self.map_redraw_loop)

        # 開機自動掃描串口
        self.scan_ports()

        # 啟動 Tkinter 佇列定時輪詢
        self.root.after(10, self.poll_queue)

        # 關閉視窗處理
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")

        # 深色控制卡片風格
        style.configure("TFrame", background="#1e1e1e")
        style.configure("Card.TFrame", background="#222222", borderwidth=1, relief="ridge")
        style.configure("Header.TLabel", background="#151515", foreground="#ffffff", font=("Helvetica", 14, "bold"))
        style.configure("Card.TLabel", background="#222222", foreground="#aaaaaa", font=("Helvetica", 9))
        style.configure("CardVal.TLabel", background="#222222", foreground="#00d2ff", font=("Helvetica", 12, "bold"))

        # 按鈕風格
        style.configure("TButton", font=("Helvetica", 10, "bold"), background="#333333", foreground="#ffffff", borderwidth=0)
        style.map("TButton", background=[("active", "#00d2ff")], foreground=[("active", "#151515")])
        style.configure("Connect.TButton", font=("Helvetica", 10, "bold"), background="#28a745", foreground="#ffffff")
        style.configure("Disconnect.TButton", font=("Helvetica", 10, "bold"), background="#dc3545", foreground="#ffffff")

        # LoRa 設定面板分頁（深色 Notebook）
        style.configure("TNotebook", background="#1c1c1c", borderwidth=0)
        style.configure("TNotebook.Tab", background="#2a2a2a", foreground="#cccccc",
                        font=("Helvetica", 10, "bold"), padding=[14, 6])
        style.map("TNotebook.Tab",
                  background=[("selected", "#00435a")],
                  foreground=[("selected", "#00d2ff")])

    # ------------------ UI 操作函數 ------------------
    def scan_ports(self):
        """自動偵測可用串口（偵測優先序統一在 serial_link.auto_port）"""
        ports = serial_link.list_candidate_ports()
        self.port_combo['values'] = ports
        self.port_combo.set(serial_link.auto_port() or DEFAULT_PORT)

    def clear_console(self):
        self.console.delete("1.0", tk.END)
        self._console_line_count = 0
        if hasattr(self, 'event_console'):
            self.event_console.delete("1.0", tk.END)

    def toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.config(text="繼續渲染" if self.paused else "暫停渲染")

    def reset_view(self):
        self.ax.view_init(elev=20, azim=45)
        if hasattr(self, 'canvas') and self.canvas:
            self.canvas.draw()

    def clear_rate_cards(self):
        defaults = {"flash_pkt": "0 Pkts", "alt": "-- m", "vz": "-- m/s", "bat": "-- V",
                    "ground_alt": "-- m", "pad_ref": "-- m"}
        for key in self.cards:
            self.cards[key].config(text=defaults.get(key, "0.00 Hz"))
        # 讓 _refresh_pad_ref_card 的「文字沒變就不動」快取失效，否則零點卡片會一直停在 "-- m"
        self._pad_ref_text = None
        self.lbl_drops.config(text="EKF Queue Drops: 0")

    # ==================== 飛行圖表 / GPS 地圖（與 3D、log 四象限同屏） ====================
    def now_t(self):
        return time.monotonic() - self.chart_t0

    def clear_chart_data(self):
        for dq in (self.ts_alt_ekf, self.ts_alt_baro, self.ts_alt_gps,
                   self.ts_vz_ekf, self.ts_spd_gps, self.ts_acc_bmi, self.ts_acc_adxl,
                   self.ts_vf_alt, self.ts_vf_vz, self.ts_ground_alt,
                   self.ts_backup_alt, self.ts_backup_vz, self.ts_backup_baro,
                   self.ts_backup_acc, self.ts_backup_vf_alt, self.ts_backup_vf_vz):
            dq.clear()
        self.fsm_events = []
        self.deploy_latch = {"self_drogue": False, "self_main": False,
                              "peer_drogue": False, "peer_main": False}
        self.backup_fsm_events = []
        self.backup_state = None
        self.chart_t0 = time.monotonic()
        self.chart_dirty = True
        self.reset_packet_counters()

    def clear_gps_track(self):
        self.gps_track = []
        self.gps_last = None
        self.map_home = None
        self.map_zoomed = False
        self.map_dirty = True
        if HAVE_MAPVIEW and hasattr(self, 'map_widget'):
            try:
                if getattr(self, 'map_path', None):
                    self.map_path.delete()
                if getattr(self, 'map_marker', None):
                    self.map_marker.delete()
            except Exception:
                pass
            self.map_path = None
            self.map_marker = None

    # ---- 飛行圖表：獨立視窗（方便拖到第二顆螢幕）----
    # 大字即時讀數（ALT/Vz/|a|）留在主視窗 console_tools（飛行中最重要的即時數字不能被
    # 藏進一個可能沒開的視窗），只有歷史時序圖表本身（含暫停/清除/儲存/時間窗控制）搬
    # 進這個 Toplevel。沿用既有 open_lora_panel/open_flash_panel 等 5 個 Toplevel 共用的
    # singleton idiom（hasattr+winfo_exists+lift，不 grab_set，讓主視窗終端/地圖/姿態
    # 保持即時更新）。
    def open_charts_window(self):
        if getattr(self, "charts_win", None) is not None and self.charts_win.winfo_exists():
            self.charts_win.lift()
            return
        self.charts_win = tk.Toplevel(self.root)
        self.charts_win.title("📈 FLIGHT CHARTS (RAW + EKF)")
        self.charts_win.geometry("1400x760")
        self.charts_win.configure(bg="#1e1e1e")
        # 不用 transient()：見 open_lora_panel 註解，子視窗要能各自獨立縮小/關閉。
        self.charts_win.protocol("WM_DELETE_WINDOW", self._on_charts_window_close)
        self.build_charts_tab(self.charts_win)
        self.chart_dirty = True   # 立刻用既有歷史補畫一幀

    def _on_charts_window_close(self):
        """關閉時務必清空 figure/canvas/line 參照，否則 charts_redraw_loop 之後每次 tick
        都會對著已銷毀的 widget 呼叫 draw_idle() 拋 TclError，且重開視窗也救不回來。"""
        try:
            self.charts_win.destroy()
        finally:
            self.charts_win = None
            for attr in ("chart_fig", "chart_canvas", "ax_alt", "ax_vel", "ax_acc",
                        "ax_alt_backup", "ax_vel_backup", "ax_acc_backup",
                        "ln_alt_ekf", "ln_alt_baro", "ln_alt_vf", "ln_ground_alt", "ln_alt_backup",
                        "ln_alt_backup_baro", "ln_alt_backup_vf", "ln_alt_gps",
                        "ln_vz_ekf", "ln_vz_vf", "ln_vz_backup", "ln_vz_backup_vf", "ln_spd_gps",
                        "ln_acc_bmi", "ln_acc_adxl", "ln_acc_ekf", "ln_acc_backup",
                        "ln_acc_backup_bmi", "ln_acc_backup_adxl", "ln_acc_backup_ekf",
                        "btn_chart_pause", "chart_win_combo"):
                setattr(self, attr, None)

    # ---- 飛行圖表分頁 ----
    def build_charts_tab(self, tab):
        # 時間窗/暫停/清除/儲存 控制列（大字即時讀數已移到主視窗 console_tools，見
        # open_charts_window() 呼叫處旁的說明——這裡只剩圖表本身相關的控制）
        bar = tk.Frame(tab, bg="#1e1e1e")
        bar.pack(fill=tk.X, padx=8, pady=(4, 2))

        ttk.Button(bar, text="儲存圖表", width=8, command=self.save_chart_snapshot).pack(side=tk.RIGHT, padx=(4, 2))
        ttk.Button(bar, text="清除", width=5, command=self.clear_chart_data).pack(side=tk.RIGHT, padx=2)
        self.btn_chart_pause = ttk.Button(bar, text="暫停", width=5, command=self.toggle_chart_pause)
        self.btn_chart_pause.pack(side=tk.RIGHT, padx=4)
        self.chart_win_combo = ttk.Combobox(bar, values=["30 s", "60 s", "120 s", "300 s"],
                                            width=6, state="readonly", font=("Helvetica", 9))
        self.chart_win_combo.current(1)
        self.chart_win_combo.pack(side=tk.RIGHT)
        tk.Label(bar, text="時間窗:", bg="#1e1e1e", fg="#aaaaaa",
                 font=("Helvetica", 9)).pack(side=tk.RIGHT, padx=(8, 3))

        # 雙欄三聯圖：左欄=主航電、右欄=副航電，各自 高度/速度/加速度（共用時間軸；
        # 標籤用 ASCII 避免 matplotlib 缺中文字型）。原本主副疊在同一張圖上，副航電
        # 的線常被主航電蓋掉幾乎看不到，改成左右分開兩欄各自獨立可讀。
        # 用 Figure(...) 不用 plt.figure()：後者會登記進 pyplot 全域 Gcf manager 永久持有
        # 參照，這個視窗現在會被重複開關，plt.figure() 會漏 Figure。
        self.chart_fig = Figure(facecolor="#101010")
        gs = self.chart_fig.add_gridspec(3, 2, hspace=0.32, wspace=0.20,
                                          left=0.07, right=0.98, top=0.94, bottom=0.08)
        self.ax_alt = self.chart_fig.add_subplot(gs[0, 0])
        self.ax_vel = self.chart_fig.add_subplot(gs[1, 0], sharex=self.ax_alt)
        self.ax_acc = self.chart_fig.add_subplot(gs[2, 0], sharex=self.ax_alt)
        self.ax_alt_backup = self.chart_fig.add_subplot(gs[0, 1], sharex=self.ax_alt)
        self.ax_vel_backup = self.chart_fig.add_subplot(gs[1, 1], sharex=self.ax_alt)
        self.ax_acc_backup = self.chart_fig.add_subplot(gs[2, 1], sharex=self.ax_alt)

        all_axes = (self.ax_alt, self.ax_vel, self.ax_acc,
                    self.ax_alt_backup, self.ax_vel_backup, self.ax_acc_backup)
        ylabels = {self.ax_alt: "Alt (m)", self.ax_vel: "Vel (m/s)", self.ax_acc: "Acc (g)",
                   self.ax_alt_backup: "Alt (m)", self.ax_vel_backup: "Vel (m/s)", self.ax_acc_backup: "Acc (g)"}
        for ax in all_axes:
            ax.set_facecolor("#151515")
            ax.tick_params(colors="#888888", labelsize=8)
            for sp in ax.spines.values():
                sp.set_color("#333333")
            ax.grid(color="#2a2a2a", linewidth=0.5, alpha=0.6)
            ax.set_ylabel(ylabels[ax], color="#aaaaaa", fontsize=9)
        self.ax_acc.set_xlabel("t (s)", color="#aaaaaa", fontsize=9)
        self.ax_acc_backup.set_xlabel("t (s)", color="#aaaaaa", fontsize=9)
        self.ax_alt.set_title("主航電 Primary", color="#00e5ff", fontsize=9, pad=4)
        self.ax_alt_backup.set_title("副航電 Backup", color="#00e676", fontsize=9, pad=4)

        # 主航電高度四條線全部同零點（發射台 pad_ref）：EKF/VF 本來就是相對值，
        # Baro 這條在 append 時已扣掉 pad_ref（原本畫絕對海拔，跟其他線差一個常數偏移）。
        self.ln_alt_ekf,  = self.ax_alt.plot([], [], color="#00e5ff", lw=1.6, label="EKF")
        self.ln_alt_baro, = self.ax_alt.plot([], [], color="#ff9800", lw=0.9, label="Baro 相對 10Hz")
        self.ln_alt_vf,   = self.ax_alt.plot([], [], color="#ffca28", lw=1.1, label="VF")
        self.ln_ground_alt, = self.ax_alt.plot([], [], color="#ffffff", lw=1.0, ls="-.", alpha=0.85, label="航電 braw 1Hz")
        self.ln_alt_gps,  = self.ax_alt.plot([], [], color="#e03bfb", lw=0, marker=".", ms=3, label="_nolegend_", visible=False)

        self.ln_alt_backup, = self.ax_alt_backup.plot([], [], color="#00e676", lw=1.6, label="EKF")
        # 副板 baro（下鏈 peer_baro_cm）也是絕對海拔，但副板自己的 pad_ref 不在封包裡，
        # 只能借主板零點扣 —— 兩板同一個發射台，差距是兩次快照的氣壓雜訊（實測 ±30cm）。
        # legend 寫明「用主板零點」，別當成副板自己的判斷值。
        self.ln_alt_backup_baro, = self.ax_alt_backup.plot([], [], color="#ff9800", lw=0.9,
                                                            label="Baro 相對(用主板零點)")
        self.ln_alt_backup_vf, = self.ax_alt_backup.plot([], [], color="#ffca28", lw=1.1, label="VF")

        self.ln_vz_ekf,   = self.ax_vel.plot([], [], color="#00e5ff", lw=1.6, label="EKF Vz")
        self.ln_vz_vf,    = self.ax_vel.plot([], [], color="#ffca28", lw=1.1, label="VF Vz")
        self.ln_spd_gps,  = self.ax_vel.plot([], [], color="#ffcc00", lw=0, marker=".", ms=3, label="_nolegend_", visible=False)

        self.ln_vz_backup,  = self.ax_vel_backup.plot([], [], color="#00e676", lw=1.6, label="EKF Vz")
        self.ln_vz_backup_vf, = self.ax_vel_backup.plot([], [], color="#ffca28", lw=1.1, label="VF Vz")

        self.ln_acc_bmi,  = self.ax_acc.plot([], [], color="#28d745", lw=0.9, label="BMI088 raw")
        self.ln_acc_adxl, = self.ax_acc.plot([], [], color="#ff3b30", lw=0.9, alpha=0.8, label="ADXL375 raw")
        self.ln_acc_ekf,  = self.ax_acc.plot([], [], color="#00e5ff", lw=1.4, ls="--", label="EKF dVz/dt")

        self.ln_acc_backup, = self.ax_acc_backup.plot([], [], color="#00e676", lw=0.9, alpha=0.6, label="Accel (Z)")
        self.ln_acc_backup_bmi,  = self.ax_acc_backup.plot([], [], color="#28d745", lw=0.9, label="BMI088 raw")
        self.ln_acc_backup_adxl, = self.ax_acc_backup.plot([], [], color="#ff3b30", lw=0.9, alpha=0.8, label="ADXL375 raw")
        self.ln_acc_backup_ekf,  = self.ax_acc_backup.plot([], [], color="#00e676", lw=1.4, ls="--", label="EKF dVz/dt")

        for ax in all_axes:
            leg = ax.legend(loc="upper left", fontsize=7, facecolor="#1c1c1c",
                            edgecolor="#333333", labelcolor="#cccccc", ncol=3)
            leg.get_frame().set_alpha(0.7)

        self.chart_canvas = FigureCanvasTkAgg(self.chart_fig, master=tab)
        self.chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def generate_flight_summary(self, now_str=None):
        """生成並儲存飛行任務總結報告 (flight_summary_YYYYMMDD_HHMMSS.txt)"""
        if now_str is None:
            now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
        os.makedirs(log_dir, exist_ok=True)
        summary_path = os.path.join(log_dir, f"flight_summary_{now_str}.txt")

        max_alt_ekf = max([v for _, v in self.ts_alt_ekf], default=0.0)
        max_alt_baro = max([v for _, v in self.ts_alt_baro], default=0.0)
        max_alt_gps = max([v for _, v in self.ts_alt_gps], default=0.0)

        max_vz_up = max([v for _, v in self.ts_vz_ekf], default=0.0)
        max_vz_down = min([v for _, v in self.ts_vz_ekf], default=0.0)

        max_acc_bmi = max([v for _, v in self.ts_acc_bmi], default=0.0)
        max_acc_adxl = max([v for _, v in self.ts_acc_adxl], default=0.0)

        t_start = min([t for t, _ in self.ts_alt_ekf] + [t for t, _ in self.ts_alt_baro], default=0.0)
        t_end = max([t for t, _ in self.ts_alt_ekf] + [t for t, _ in self.ts_alt_baro], default=0.0)
        duration_s = max(0.0, t_end - t_start)

        gps_first = self.gps_track[0] if self.gps_track else (None, None)
        gps_last = self.gps_last if self.gps_last else (None, None)

        report_lines = [
            "==================================================",
            "        RocketCom 飛行任務總結報告 (Flight Summary)",
            "==================================================",
            f" 報告生成時間 : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f" 記錄資料來源 : 地面站雙鏈路遙測整合 (433MHz + 920MHz)",
            f" 飛行持續時間 : {duration_s:.1f} 秒",
            "--------------------------------------------------",
            "【1. 高度與速度指標 (Altitude & Velocity)】",
            f"  • EKF 估算最高高度  (Apogee)   : {max_alt_ekf:.2f} m",
            f"  • 氣壓計最高高度 (相對起點)     : {max_alt_baro:.2f} m",
            f"  • 發射台氣壓零點 pad_ref       : "
            f"{f'{self.pad_ref.pad_ref_m:.2f} m (絕對海拔尺度)' if self.pad_ref.valid else '未收到 [FSM] pad_ref locked'}",
            f"  • GPS 海拔最高高度            : {max_alt_gps:.2f} m",
            f"  • 最大上升速度 (Max Ascent Vz) : {max_vz_up:+.2f} m/s",
            f"  • 最大下降速度 (Max Descent Vz): {max_vz_down:+.2f} m/s",
            "",
            "【2. 加速度指標 (Acceleration)】",
            f"  • BMI088 IMU 最大過載          : {max_acc_bmi:.2f} g",
            f"  • ADXL375 高 G 感測器最大過載  : {max_acc_adxl:.2f} g",
            "",
            "【3. 通訊與鏈路品質 (Communication Links)】",
            f"  • 地面站總接收封包數           : {self.gs_pkt_cnt_total} 筆",
            f"  • 433MHz LoRa 接收包數        : {self.gs_pkt_cnt_433} 筆",
            f"  • 920MHz LoRa 接收包數        : {self.gs_pkt_cnt_920} 筆",
            "",
            "【4. GPS 定位與落點資訊 (GPS Landing)】",
            f"  • 起飛/解鎖點 GPS 座標         : {f'{gps_first[0]:.6f}, {gps_first[1]:.6f}' if gps_first[0] is not None else 'N/A'}",
            f"  • 最終落點 GPS 座標            : {f'{gps_last[0]:.6f}, {gps_last[1]:.6f}' if gps_last[0] is not None else 'N/A'}",
            "=================================================="
        ]

        summary_text = "\n".join(report_lines)

        try:
            with open(summary_path, "w", encoding="utf-8") as f:
                f.write(summary_text + "\n")
        except Exception as e:
            print(f"[ERROR] 儲存飛行任務總結報告失敗: {e}")

        return summary_path, summary_text

    def save_chart_snapshot(self, silent=False):
        """將目前飛行圖表截圖 (PNG)、時間序列數據 (CSV) 與飛行總結報告 (TXT) 儲存至 logs 資料夾"""
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
        os.makedirs(log_dir, exist_ok=True)

        png_path = os.path.join(log_dir, f"chart_{now_str}.png")
        csv_path = os.path.join(log_dir, f"flight_telemetry_{now_str}.csv")

        # 1. 儲存 Matplotlib 圖表 PNG
        try:
            self.chart_fig.savefig(png_path, dpi=150, facecolor=self.chart_fig.get_facecolor(), bbox_inches="tight")
        except Exception as e:
            if not silent:
                messagebox.showerror("儲存失敗", f"無法儲存圖表圖片: {e}")
            return False

        # 2. 儲存時間序列 CSV 數據
        try:
            with open(csv_path, "w", encoding="utf-8") as f:
                f.write("t_rel_s,alt_ekf_m,alt_baro_m,alt_gps_m,vz_ekf_ms,spd_gps_ms,acc_bmi_g,acc_adxl_g\n")
                times = sorted(list(set(
                    [t for t, _ in self.ts_alt_ekf] +
                    [t for t, _ in self.ts_alt_baro] +
                    [t for t, _ in self.ts_vz_ekf] +
                    [t for t, _ in self.ts_acc_bmi]
                )))
                
                d_alt_ekf  = dict(self.ts_alt_ekf)
                d_alt_baro = dict(self.ts_alt_baro)
                d_alt_gps  = dict(self.ts_alt_gps)
                d_vz_ekf   = dict(self.ts_vz_ekf)
                d_spd_gps  = dict(self.ts_spd_gps)
                d_acc_bmi  = dict(self.ts_acc_bmi)
                d_acc_adxl = dict(self.ts_acc_adxl)

                for t in times:
                    alt_ekf  = f"{d_alt_ekf[t]:.2f}" if t in d_alt_ekf else ""
                    alt_baro = f"{d_alt_baro[t]:.2f}" if t in d_alt_baro else ""
                    alt_gps  = f"{d_alt_gps[t]:.2f}" if t in d_alt_gps else ""
                    vz_ekf   = f"{d_vz_ekf[t]:.2f}" if t in d_vz_ekf else ""
                    spd_gps  = f"{d_spd_gps[t]:.2f}" if t in d_spd_gps else ""
                    acc_bmi  = f"{d_acc_bmi[t]:.3f}" if t in d_acc_bmi else ""
                    acc_adxl = f"{d_acc_adxl[t]:.3f}" if t in d_acc_adxl else ""
                    f.write(f"{t:.3f},{alt_ekf},{alt_baro},{alt_gps},{vz_ekf},{spd_gps},{acc_bmi},{acc_adxl}\n")
        except Exception as e:
            print(f"[ERROR] 儲存 CSV 失敗: {e}")

        # 3. 生成並儲存飛行任務總結報告 TXT
        summary_path, summary_text = self.generate_flight_summary(now_str)

        if not silent:
            msg = f"📊 飛行圖表、數據與總結報告已成功儲存！\n\n- 圖片: {os.path.basename(png_path)}\n- 數據: {os.path.basename(csv_path)}\n- 總結: {os.path.basename(summary_path)}"
            messagebox.showinfo("儲存成功", msg)
            if hasattr(self, 'console'):
                self.console.insert(tk.END, f"\n{summary_text}\n\n[SYSTEM] 📊 飛行任務總結報告已儲存至 {summary_path}\n")
        return True

    def toggle_chart_pause(self):
        self.chart_paused = not self.chart_paused
        self.btn_chart_pause.config(text="繼續" if self.chart_paused else "暫停")

    @staticmethod
    def _series_window(dq, tmin):
        """deque[(t,v)] → 視窗內 (ts, vs) numpy 陣列"""
        if not dq:
            return np.array([]), np.array([])
        arr = np.array(dq, dtype=float)
        sel = arr[:, 0] >= tmin
        return arr[sel, 0], arr[sel, 1]

    def redraw_charts(self):
        win = float(self.chart_win_combo.get().split()[0])
        tmax = self.now_t()
        tmin = max(0.0, tmax - win)

        pairs = [
            (self.ln_alt_ekf, self.ts_alt_ekf), (self.ln_alt_baro, self.ts_alt_baro),
            (self.ln_alt_vf, self.ts_vf_alt),
            (self.ln_ground_alt, self.ts_ground_alt),
            (self.ln_alt_backup, self.ts_backup_alt),
            (self.ln_alt_backup_baro, self.ts_backup_baro),
            (self.ln_alt_backup_vf, self.ts_backup_vf_alt),
            (self.ln_alt_gps, self.ts_alt_gps),
            (self.ln_vz_ekf, self.ts_vz_ekf), (self.ln_vz_vf, self.ts_vf_vz),
            (self.ln_vz_backup, self.ts_backup_vz), (self.ln_vz_backup_vf, self.ts_backup_vf_vz),
            (self.ln_spd_gps, self.ts_spd_gps),
            (self.ln_acc_bmi, self.ts_acc_bmi), (self.ln_acc_adxl, self.ts_acc_adxl),
            (self.ln_acc_backup, self.ts_backup_acc),
            (self.ln_acc_backup_bmi, self.ts_backup_acc_bmi),
            (self.ln_acc_backup_adxl, self.ts_backup_acc_adxl),
        ]
        for ln, dq in pairs:
            if not ln.get_visible():
                continue
            ts, vs = self._series_window(dq, tmin)
            ln.set_data(ts, vs)

        # EKF 加速度 = EKF Vz 數值微分（中央差分，/9.81 轉 g）——主/副航電分開算
        ts, vs = self._series_window(self.ts_vz_ekf, tmin)
        if len(ts) >= 3:
            self.ln_acc_ekf.set_data(ts, np.gradient(vs, ts) / 9.81)
        else:
            self.ln_acc_ekf.set_data([], [])

        ts, vs = self._series_window(self.ts_backup_vz, tmin)
        if len(ts) >= 3:
            self.ln_acc_backup_ekf.set_data(ts, np.gradient(vs, ts) / 9.81)
        else:
            self.ln_acc_backup_ekf.set_data([], [])

        all_axes = (self.ax_alt, self.ax_vel, self.ax_acc,
                    self.ax_alt_backup, self.ax_vel_backup, self.ax_acc_backup)
        for ax in all_axes:
            ax.set_xlim(tmin, max(tmax, tmin + 1.0))
            ax.relim(visible_only=True)
            ax.autoscale_view(scalex=False, scaley=True)

        # FSM 飛行階段色帶（每次 redraw 重畫，移除舊色帶再重繪）
        if hasattr(self, '_fsm_spans'):
            for coll in self._fsm_spans:
                try:
                    coll.remove()
                except Exception:
                    pass
        self._fsm_spans = []

        def _draw_fsm_events(events, axes, alt_ax, label_prefix=""):
            for i, (t_start_ev, state) in enumerate(events):
                t_end_ev = events[i + 1][0] if i + 1 < len(events) else tmax
                if t_end_ev <= tmin or t_start_ev >= tmax:
                    continue   # 視窗外略過
                phase = self._FSM_PHASE.get(state, ("#1a1a2e", "#aaaaaa", state.replace("STATE_", "")))
                bg_col, txt_col, label = phase
                x0 = max(t_start_ev, tmin)
                x1 = min(t_end_ev, tmax)
                for ax in axes:
                    span = ax.axvspan(x0, x1, color=bg_col, alpha=0.25, zorder=0, linewidth=0)
                    self._fsm_spans.append(span)
                # 在高度子圖頂部畫標籤線 + 文字
                vline = alt_ax.axvline(t_start_ev, color=txt_col, lw=0.8, ls="--", alpha=0.6, zorder=1)
                self._fsm_spans.append(vline)
                ylims = alt_ax.get_ylim()
                y_top = ylims[1] - (ylims[1] - ylims[0]) * 0.05
                txt = alt_ax.text(
                    t_start_ev + (x1 - x0) * 0.03, y_top, f"{label_prefix}{label}",
                    color=txt_col, fontsize=6.5, va="top", ha="left",
                    alpha=0.85, zorder=2,
                    bbox=dict(boxstyle="round,pad=0.15", fc=bg_col, ec=txt_col, lw=0.5, alpha=0.7)
                )
                self._fsm_spans.append(txt)

        # 主航電 FSM 狀態轉換 → 只畫在左欄（主航電）三張子圖
        if self.fsm_events:
            _draw_fsm_events(self.fsm_events, (self.ax_alt, self.ax_vel, self.ax_acc), self.ax_alt)

        # 副航電 FSM 狀態轉換 → 只畫在右欄（副航電）三張子圖，各自獨立不再與主航電互相遮擋
        if self.backup_fsm_events:
            _draw_fsm_events(self.backup_fsm_events,
                              (self.ax_alt_backup, self.ax_vel_backup, self.ax_acc_backup),
                              self.ax_alt_backup)

        self.chart_canvas.draw_idle()

    def _update_big_readouts(self):
        """大字即時讀數（ALT/Vz/|a|，主視窗 console_tools）——獨立於圖表視窗開關之外，
        每次 charts_redraw_loop tick 都呼叫，不受圖表視窗是否開啟影響（原本內嵌在
        redraw_charts() 尾端，圖表移到獨立視窗後若不拆開，視窗一關這三個數字就會凍結——
        那是飛行中最重要的即時讀數，不能被藏進一個可能沒開的視窗）。"""
        if self.ts_alt_ekf:
            self.lbl_big_alt.config(text=f"ALT {self.ts_alt_ekf[-1][1]:.1f} m")
        elif self.ts_alt_baro:
            self.lbl_big_alt.config(text=f"ALT {self.ts_alt_baro[-1][1]:.1f} m (baro)")
        if self.ts_vz_ekf:
            self.lbl_big_vz.config(text=f"Vz {self.ts_vz_ekf[-1][1]:+.1f} m/s")
        if self.ts_acc_bmi:
            self.lbl_big_acc.config(text=f"|a| {self.ts_acc_bmi[-1][1]:.2f} g")

    def attitude_redraw_loop(self):
        """3D 姿態定時重畫（5Hz）。一幀約 50ms，200ms 間隔約佔 GUI 執行緒 25%，
        留下足夠餘裕給 log 滾動、飛行圖表與 GPS 地圖。"""
        try:
            if not self.root.winfo_exists():
                return
            if (not self.paused) and self.att_dirty:
                self.update_3d_plot(self.last_q)
                self.att_dirty = False
            self.root.after(200, self.attitude_redraw_loop)
        except tk.TclError:
            pass

    def _refresh_pad_ref_card(self):
        """零點卡片：值 + 「幾秒前重零」/「已凍結」。文字沒變就不動 widget。"""
        text, color = self.pad_ref.label_text()
        if getattr(self, "_pad_ref_text", None) != text:
            self._pad_ref_text = text
            card = self.cards.get("pad_ref")
            if card is not None and card.winfo_exists():
                card.config(text=text, foreground=color)

    def charts_redraw_loop(self):
        try:
            if not self.root.winfo_exists():
                return
            self._refresh_pad_ref_card()  # 「幾秒前重零」要自己走鐘，不能只在收到 pad_ref 行時更新
            self._update_big_readouts()   # 主視窗永遠更新，不受圖表視窗開關影響
            if not self.chart_paused:
                charts_win = getattr(self, "charts_win", None)
                if self.chart_dirty and charts_win is not None and charts_win.winfo_exists():
                    self.redraw_charts()
                self.chart_dirty = False   # 無條件清（視窗未開時也一併清掉，避免旗標白白堆積）
            self.root.after(300, self.charts_redraw_loop)
        except tk.TclError:
            pass


    # ---- GPS 地圖分頁 ----
    def build_map_tab(self, tab):
        ctrl = tk.Frame(tab, bg="#1e1e1e")
        ctrl.pack(fill=tk.X, padx=8, pady=(6, 2))
        self.map_follow_var = tk.BooleanVar(value=True)
        tk.Checkbutton(ctrl, text="跟隨最新位置", variable=self.map_follow_var,
                       bg="#1e1e1e", fg="#00d2ff", selectcolor="#151515",
                       font=("Helvetica", 9)).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="清除軌跡", width=8, command=self.clear_gps_track).pack(side=tk.LEFT, padx=6)
        mode = "OpenStreetMap 線上地圖" if HAVE_MAPVIEW else "相對軌跡圖（tkintermapview 未安裝/離線）"
        tk.Label(ctrl, text=mode, bg="#1e1e1e", fg="#777777",
                 font=("Helvetica", 9)).pack(side=tk.RIGHT, padx=6)

        self.lbl_gps_info = tk.Label(tab, text="等待 GPS 定位…（火箭=[GPS]/[GS_PKT]，地面站自身=[GS_GPS]）",
                                     bg="#101014", fg="#ffcc00", font=("Monaco", 10),
                                     anchor="w", justify=tk.LEFT, padx=10, pady=6)
        self.lbl_gps_info.pack(fill=tk.X, padx=8, pady=(0, 4))

        # 建立地圖與狀態欄的左右對齊容器
        container = tk.Frame(tab, bg="#1e1e1e")
        container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左側地圖容器
        map_container = tk.Frame(container, bg="#1e1e1e")
        map_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 右側 GPS 尋星與封包解析狀態面板
        status_panel = tk.Frame(container, bg="#101014", width=180, highlightbackground="#2a2a30", highlightthickness=1)
        status_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(6, 0))
        status_panel.pack_propagate(False)

        # 狀態面板標題
        tk.Label(status_panel, text="📡 GPS 尋星狀態", bg="#101014", fg="#00d2ff",
                 font=("Helvetica", 9, "bold"), pady=6).pack(fill=tk.X)
        
        # 狀態面板網格
        grid = tk.Frame(status_panel, bg="#101014", padx=6, pady=4)
        grid.pack(fill=tk.BOTH, expand=True)
        
        # 1. 定位狀態
        tk.Label(grid, text="定位狀態:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=0, column=0, sticky="w", pady=4)
        self.lbl_gps_status_fix = tk.Label(grid, text="等待數據", bg="#101014", fg="#ffcc00", font=("Helvetica", 9, "bold"), anchor="e")
        self.lbl_gps_status_fix.grid(row=0, column=1, sticky="e", pady=4)
        
        # 2. 衛星數量
        tk.Label(grid, text="衛星數量:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=1, column=0, sticky="w", pady=4)
        self.lbl_gps_status_sats = tk.Label(grid, text="0", bg="#101014", fg="#ffffff", font=("Helvetica", 9, "bold"), anchor="e")
        self.lbl_gps_status_sats.grid(row=1, column=1, sticky="e", pady=4)
        
        # 3. 訊號延遲/Stale狀態
        tk.Label(grid, text="信號狀態:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=2, column=0, sticky="w", pady=4)
        self.lbl_gps_status_stale = tk.Label(grid, text="等待數據", bg="#101014", fg="#777777", font=("Helvetica", 9), anchor="e")
        self.lbl_gps_status_stale.grid(row=2, column=1, sticky="e", pady=4)
        
        # 4. 解析成功
        tk.Label(grid, text="解析成功:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=3, column=0, sticky="w", pady=4)
        self.lbl_gps_status_ok = tk.Label(grid, text="0", bg="#101014", fg="#00e676", font=("Helvetica", 9), anchor="e")
        self.lbl_gps_status_ok.grid(row=3, column=1, sticky="e", pady=4)
        
        # 5. 解析失敗
        tk.Label(grid, text="解析失敗:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=4, column=0, sticky="w", pady=4)
        self.lbl_gps_status_err = tk.Label(grid, text="0", bg="#101014", fg="#ffffff", font=("Helvetica", 9), anchor="e")
        self.lbl_gps_status_err.grid(row=4, column=1, sticky="e", pady=4)
        
        # 6. 更新頻率
        tk.Label(grid, text="更新頻率:", bg="#101014", fg="#888888", font=("Helvetica", 9), anchor="w").grid(row=5, column=0, sticky="w", pady=4)
        self.lbl_gps_status_rate = tk.Label(grid, text="0.00 Hz", bg="#101014", fg="#e03bfb", font=("Helvetica", 9), anchor="e")
        self.lbl_gps_status_rate.grid(row=5, column=1, sticky="e", pady=4)

        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        # 載入實體地圖或後備相對軌跡圖
        if HAVE_MAPVIEW:
            self.map_widget = tkintermapview.TkinterMapView(map_container, corner_radius=0)
            self.map_widget.pack(fill=tk.BOTH, expand=True)
            self.map_widget.set_position(23.97, 120.97)   # 預設台灣中心，待首筆定位跳轉
            self.map_widget.set_zoom(8)
            self.map_marker = None
            self.map_path = None
            self.gs_marker = None
        else:
            # 離線後備：E/N 相對軌跡（原點 = 第一筆定位 Home）
            self.trk_fig = plt.figure(facecolor="#101010")
            self.trk_ax = self.trk_fig.add_subplot(111)
            self.trk_ax.set_facecolor("#151515")
            self.trk_ax.tick_params(colors="#888888", labelsize=8)
            for sp in self.trk_ax.spines.values():
                sp.set_color("#333333")
            self.trk_ax.grid(color="#2a2a2a", linewidth=0.5, alpha=0.6)
            self.trk_ax.set_xlabel("East (m)", color="#aaaaaa", fontsize=9)
            self.trk_ax.set_ylabel("North (m)", color="#aaaaaa", fontsize=9)
            self.trk_ax.set_aspect("equal", adjustable="datalim")
            self.trk_line, = self.trk_ax.plot([], [], color="#00e5ff", lw=1.2)
            self.trk_pt,   = self.trk_ax.plot([], [], color="#ff1744", marker="o", ms=8, lw=0)
            self.trk_home, = self.trk_ax.plot([], [], color="#00e676", marker="^", ms=9, lw=0)
            self.trk_canvas = FigureCanvasTkAgg(self.trk_fig, master=map_container)
            self.trk_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.update_gps_status_panel()

    def update_gps_status_panel(self):
        """重新整理 UI 面板上的 GPS 尋星與解析狀態數據"""
        if not hasattr(self, 'lbl_gps_status_fix'):
            return
        
        info = self.gps_status_info
        
        # 1. 定位狀態標籤
        if info["fix"] == 0:
            self.lbl_gps_status_fix.config(text="未定位 (No Fix)", fg="#ff3366")
        elif info["fix"] == 1:
            self.lbl_gps_status_fix.config(text="已定位 (3D Fix)", fg="#00e676")
        elif info["fix"] == 2:
            self.lbl_gps_status_fix.config(text="差分定位 (DGPS)", fg="#00e676")
        else:
            self.lbl_gps_status_fix.config(text=f"定位中 ({info['fix']})", fg="#ffcc00")
            
        # 2. 衛星數量
        self.lbl_gps_status_sats.config(text=str(info["sats"]), fg="#ffffff" if info["sats"] >= 4 else "#ffcc00")
        
        # 3. 訊號延遲/Stale狀態
        if info["stale"] == 1:
            self.lbl_gps_status_stale.config(text="資料逾時 (Stale)", fg="#ff3366")
        else:
            self.lbl_gps_status_stale.config(text="即時更新 (Active)", fg="#00e676")
            
        # 4. 解析成功語句數
        self.lbl_gps_status_ok.config(text=str(info["ok"]))
        
        # 5. 解析失敗語句數
        err_fg = "#ffffff" if info["err"] == 0 else "#ff3366"
        self.lbl_gps_status_err.config(text=str(info["err"]), fg=err_fg)
        
        # 6. 更新頻率
        self.lbl_gps_status_rate.config(text=f"{info['rate']:.2f} Hz")

    @staticmethod
    def _latlon_to_en(lat, lon, lat0, lon0):
        """經緯度 → 相對 Home 的 East/North 公尺（等距圓柱近似，短距離足夠）"""
        k = 111320.0
        return (lon - lon0) * k * np.cos(np.radians(lat0)), (lat - lat0) * k

    def update_map(self):
        # 資訊列
        if self.gps_last:
            g = self.gps_last
            info = (f"🚀 lat={g['lat']:+.6f}  lon={g['lon']:+.6f}  alt={g.get('alt', 0)} m  "
                    f"spd={g.get('spd', 0.0):.1f} m/s  sats={g.get('sats', '?')}")
            if self.map_home:
                e, n = self._latlon_to_en(g['lat'], g['lon'], self.map_home[0], self.map_home[1])
                info += f"  距Home {float(np.hypot(e, n)):.0f} m"
            self.lbl_gps_info.config(text=info, fg="#00e676")
        elif self.gs_own_pos:
            self.lbl_gps_info.config(
                text=f"📡 地面站 lat={self.gs_own_pos[0]:+.6f} lon={self.gs_own_pos[1]:+.6f}（等待火箭 GPS…）",
                fg="#ffcc00")

        if HAVE_MAPVIEW:
            if self.gps_track:
                lat, lon = self.gps_track[-1]
                if self.map_marker is None:
                    self.map_marker = self.map_widget.set_marker(lat, lon, text="🚀")
                else:
                    self.map_marker.set_position(lat, lon)
                if len(self.gps_track) >= 2:
                    pts = self.gps_track[-600:]
                    try:
                        if self.map_path is None:
                            self.map_path = self.map_widget.set_path(pts)
                        else:
                            self.map_path.set_position_list(pts)
                    except Exception:   # 版本差異：退回重建路徑
                        try:
                            if self.map_path:
                                self.map_path.delete()
                        except Exception:
                            pass
                        self.map_path = self.map_widget.set_path(pts)
                if not self.map_zoomed:
                    self.map_widget.set_zoom(16)
                    self.map_zoomed = True
                if self.map_follow_var.get():
                    self.map_widget.set_position(lat, lon)
            if self.gs_own_pos:
                if self.gs_marker is None:
                    self.gs_marker = self.map_widget.set_marker(
                        self.gs_own_pos[0], self.gs_own_pos[1], text="📡GS")
                else:
                    self.gs_marker.set_position(self.gs_own_pos[0], self.gs_own_pos[1])
                if not self.gps_track and not self.map_zoomed:
                    self.map_widget.set_position(self.gs_own_pos[0], self.gs_own_pos[1])
                    self.map_widget.set_zoom(15)
                    self.map_zoomed = True
        else:
            if self.gps_track and self.map_home:
                lat0, lon0 = self.map_home
                arr = np.array(self.gps_track, dtype=float)
                e, n = self._latlon_to_en(arr[:, 0], arr[:, 1], lat0, lon0)
                self.trk_line.set_data(e, n)
                self.trk_pt.set_data([e[-1]], [n[-1]])
                self.trk_home.set_data([0.0], [0.0])
                self.trk_ax.relim()
                self.trk_ax.autoscale_view()
                self.trk_canvas.draw_idle()

    def map_redraw_loop(self):
        try:
            if not self.root.winfo_exists():
                return
            if self.map_dirty:
                self.update_map()
                self.map_dirty = False
            self.root.after(1000, self.map_redraw_loop)
        except tk.TclError:
            pass

    def on_gps_fix(self, lat, lon, alt_m=None, spd_ms=None, sats=None):
        """收到一筆有效火箭定位（來自 [GPS] 或 [GS_PKT] pos:）"""
        if self.map_home is None:
            self.map_home = (lat, lon)
        # 去抖：與上一點相同就不重複入軌跡
        if not self.gps_track or self.gps_track[-1] != (lat, lon):
            self.gps_track.append((lat, lon))
            if len(self.gps_track) > 5000:
                self.gps_track = self.gps_track[-4000:]
        g = self.gps_last or {}
        g.update({"lat": lat, "lon": lon})
        if alt_m is not None:
            g["alt"] = alt_m
        if spd_ms is not None:
            g["spd"] = spd_ms
        if sats is not None:
            g["sats"] = sats
        self.gps_last = g
        self.map_dirty = True

    # ------------------ 連線控制 ------------------
    def toggle_connection(self):
        if not self.running:
            # 建立連線
            port = self.port_combo.get().strip()
            try:
                baud = int(self.baud_combo.get())
            except ValueError:
                baud = DEFAULT_BAUD
                
            try:
                self.ser = serial_link.open_serial(port, baud, timeout=0.5)
            except Exception as e:
                messagebox.showerror("串口連線失敗", f"無法開啟 {port}，請檢查硬體接線！\n錯誤: {e}")
                return

            # 記住這次連的 port/baud，供斷線自動重連使用（板子重置＝USB-CDC 整個消失
            # 再重新列舉，見 serial_read_task 例外處理）。
            self.connect_port = port
            self.connect_baud = baud

            # 打開記錄檔
            if self.save_log_var.get():
                now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
                self.log_file = open(os.path.join(log_dir, f"gui_serial_continuous_{now_str}.log"), "w", encoding="utf-8")
                self.log_file.write(f"--- GUI CONTINUOUS MONITOR SESSION START AT {datetime.now()} ---\n")
                self.log_file.flush()
                
            self.running = True
            self.clear_console()
            self.clear_rate_cards()
            self.clear_chart_data()
            self.clear_gps_track()
            
            # 建立串列背景執行緒
            self.serial_thread = threading.Thread(target=self.serial_read_task, daemon=True)
            self.serial_thread.start()
            
            self.btn_connect.config(text="中斷連線", style="Disconnect.TButton")
            self.console.insert(tk.END, f"[SYSTEM] 🟢 已成功連接 {port} @ {baud} baud\n")

            # 角色偵測：重置後主動送 'role' 查詢（1.5s 後，讓開機/雜訊先過）
            self.set_role(None)
            self.role_query_attempts = 0
            self.gs_tx_query_attempts = 0
            self.root.after(1500, self.query_role)
            self.root.after(1500, self.query_gs_tx)
        else:
            # 中斷連線
            self.running = False
            if hasattr(self, 'ser') and self.ser:
                try:
                    self.ser.close()
                except:
                    pass
            if self.save_log_var.get() and (len(self.ts_alt_ekf) > 0 or len(self.ts_alt_baro) > 0):
                self.save_chart_snapshot(silent=True)

            if self.log_file:
                try:
                    self.log_file.close()
                except:
                    pass
                self.log_file = None
                
            self.btn_connect.config(text="連接 GPS / 航電", style="TButton")
            self.console.insert(tk.END, "[SYSTEM] 🔴 串口已關閉，連線中止。\n")
            self.set_role(None)

    # ------------------ 背景執行緒：高速串口讀取 ------------------
    def serial_read_task(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    raw = self.ser.readline()
                    if not raw:
                        continue
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    # 放入佇列傳送至主執行緒
                    self.data_queue.put((ts, line))

                    # 同步寫入日誌
                    if self.log_file:
                        self.log_file.write(f"[{ts}] {line}\n")
                        self.log_file.flush()
                else:
                    time.sleep(0.002) # 減少 CPU 飆升
            except Exception as e:
                if not self.running:
                    break
                # 串口中斷不視為致命錯誤直接停止監控：MCU 重置時 USB-CDC 會整個消失、
                # 重新列舉成新裝置，若在此停手要求使用者自己按「連接」，等使用者反應過來
                # 開機/角色判斷等最早期的 log 早已錯過（USB best-effort、PC 沒接就丟，
                # 韌體端也不落地）。故改為原地快速輪詢重開同一個 port，搶開機最早窗口。
                self.data_queue.put(("SYS", f"[SYSTEM] ⚠️ 串口中斷（{e}），偵測是否為板子重置並自動重連…"))
                try:
                    self.ser.close()
                except Exception:
                    pass
                self._serial_reconnect_loop()
                # running 可能在等待期間被使用者按「中斷連線」清掉，迴圈條件會自然跳出

        # 清除連線狀態
        try:
            if self.root.winfo_exists():
                self.root.after(0, self.update_disconnect_ui)
        except tk.TclError:
            pass

    def _serial_reconnect_loop(self):
        """MCU 重置＝USB-CDC 整個消失再重新列舉；50ms 高頻輪詢重開同一個 port，
        搶開機最早期窗口（愈快接上，愈不會錯過 [BOOT]/角色判斷等只印一次的訊息）。
        直接呼叫 serial.Serial 而非 serial_link.open_serial：後者每次失敗都會 print，
        高頻輪詢會洗版終端機。"""
        t0 = time.monotonic()
        warned = False
        while self.running:
            try:
                ser = serial.Serial(self.connect_port, self.connect_baud, timeout=0.5)
                try:
                    ser.dtr = False
                    ser.rts = False
                except Exception:
                    pass
                ser.reset_input_buffer()
                self.ser = ser
                self.data_queue.put(("SYS", f"[SYSTEM] 🟢 已自動重新連接 {self.connect_port}"))
                return
            except Exception:
                if not warned and (time.monotonic() - t0) > 5.0:
                    self.data_queue.put(("SYS", f"[SYSTEM] ⏳ 仍在嘗試自動重連 {self.connect_port}…"))
                    warned = True
                time.sleep(0.05)

    def update_disconnect_ui(self):
        try:
            if hasattr(self, 'btn_connect') and self.btn_connect.winfo_exists():
                self.btn_connect.config(text="連接 GPS / 航電", style="TButton")
        except tk.TclError:
            pass
        if self.log_file:
            try: self.log_file.close()
            except: pass
            self.log_file = None

    # ------------------ 電梯測試 profile 醒目警示（三種資料來源共用一組時間戳，見 __init__ 註解） ------------------
    def _handle_pad_cfg_profile_line(self, line):
        """[PAD_CFG] Flight Profile: ELEVATOR_TEST(...) / REAL_FLIGHT(...)，每 10s 一行，
        僅回報「本板」——是唯一會明確講「已經切回正式版」的權威來源，REAL_FLIGHT 時直接
        歸零時間戳（立即清除），不必等新鮮度窗口逾時。"""
        self._elevator_self_ts = time.time() if "ELEVATOR_TEST" in line else 0.0
        self._refresh_elevator_banner()

    def _handle_elevator_warning_line(self, line):
        """!!! [ELEVATOR_TEST_WARNING] THIS BOARD / PEER BOARD STILL ON ELEVATOR-TEST PROFILE !!!
        每 1s 一行、只在仍為電梯測試 profile 時才會印出（見 main.c），瘋狂洗版故意不節流。"""
        now = time.time()
        who = []
        if "THIS BOARD" in line:
            self._elevator_self_ts = now
            who.append("本板")
        if "PEER BOARD" in line:
            self._elevator_peer_ts = now
            who.append("對端(副板)")
        self._refresh_elevator_banner()
        spam_elevator_console_warning(self.console, "+".join(who))

    def _handle_gs_pkt_profile(self, prof_hex):
        """[GS_PKT] ... prof:0x%02X（經地面站板 LoRa 中繼時的路徑，見 ground_station.c）。
        bit0=主航電、bit1=副航電，語意與 telemetry.h TELEM_PROFILE_* 一致。"""
        prof = int(prof_hex, 16)
        now = time.time()
        who = []
        if prof & 0x01:
            self._elevator_self_ts = now
            who.append("本板(主航電)")
        if prof & 0x02:
            self._elevator_peer_ts = now
            who.append("對端(副航電)")
        self._refresh_elevator_banner()
        if who:
            spam_elevator_console_warning(self.console, "+".join(who))

    def _refresh_elevator_banner(self):
        """兩個時間戳都採 3 秒新鮮度窗口（斷線/恢復正式版超過窗口即視為已清除），
        由 poll_queue 每輪都呼叫一次，故不需要另外設定逾時計時器。"""
        now = time.time()
        self_active = (now - self._elevator_self_ts) < 3.0
        peer_active = (now - self._elevator_peer_ts) < 3.0
        active = self_active or peer_active
        who = []
        if self_active: who.append("本板")
        if peer_active: who.append("對端(副板)")
        update_elevator_banner(self.root, self.elevator_banner, active, "+".join(who))

    # ---- ★2026-07-31：Flash 未擦除提醒（開機不再自動擦除，見 main.c 開機序列） ----
    def _handle_flash_not_ready_line(self, line):
        """[FLASH_NOT_READY] 已擦池 320/1500 sectors —— ARM 已被擋下（航電 1Hz 直連輸出）"""
        self._flash_need_erase_ts = time.time()
        m = re.search(r"已擦池\s*(\d+)\s*/\s*(\d+)", line)
        self._flash_need_erase_detail = (f"池 {m.group(1)}/{m.group(2)} sectors"
                                         if m else "池未達標")
        self._refresh_flash_banner()

    def _handle_gs_pkt_arm_flags(self, arm_flags):
        """[GS_PKT] armf:0x%02X（經地面站板 LoRa 中繼）。bit1 = TELEM_ARM_NEED_ERASE。
        與直連來源共用同一個時間戳：同一 process 同時只會接一種來源，用時間戳統一收斂。"""
        if arm_flags & 0x02:
            self._flash_need_erase_ts = time.time()
            if not self._flash_need_erase_detail:
                self._flash_need_erase_detail = "航電回報：尚未擦除"
        else:
            # 權威來源明確說「不需要擦」→ 立刻清除，不等新鮮度窗口過期
            self._flash_need_erase_ts = 0.0
            self._flash_need_erase_detail = ""
        self._refresh_flash_banner()

    def _refresh_flash_banner(self):
        """新鮮度窗口取 5 秒：航電端橫幅是 1Hz、下鏈 [GS_PKT] 約 2Hz，5 秒足以容忍
        少量丟包又不會在擦完之後還留著過期警示。"""
        active = (time.time() - self._flash_need_erase_ts) < 5.0
        update_flash_banner(self.root, self.flash_banner, active,
                            self._flash_need_erase_detail if active else "")

    # ------------------ 主執行緒：定時處理佇列 ------------------
    def poll_queue(self):
        # 批次處理佇列中的資料，避免界面阻塞
        max_lines_per_frame = 30
        lines_processed = 0
        
        while not self.data_queue.empty() and lines_processed < max_lines_per_frame:
            ts, line = self.data_queue.get()
            lines_processed += 1
            
            if ts == "ERR":
                self.console.insert(tk.END, f"{line}\n", "err")
                continue

            if ts == "SYS":
                # 斷線/自動重連提示（不是錯誤，見 serial_read_task/_serial_reconnect_loop）
                self.console.insert(tk.END, f"{line}\n", "ok")
                continue

            # 輸出滾動字元日誌，高亮重要字眼
            tag = "tele"
            if "[ACK]" in line:
                tag = "ack"
            elif "[UPLINK]" in line:
                tag = "lora"
            elif "[LINK]" in line:
                tag = "link"
            elif "[RATE]" in line:
                tag = "rate"
            elif "[PAD_CFG]" in line:
                tag = "err" if "WARNING" in line else "ok"
                if "Flight Profile:" in line:
                    self._handle_pad_cfg_profile_line(line)
            elif "[ELEVATOR_TEST_WARNING]" in line:
                tag = "elevator_warn"
                self._handle_elevator_warning_line(line)
            elif "[FLASH_NOT_READY]" in line:
                tag = "err"
                self._handle_flash_not_ready_line(line)
            elif "[MAG]" in line:
                tag = "mag"
            elif "[GPS]" in line or "[GS_GPS]" in line:
                tag = "gps"
            elif ("[E80]" in line or "[E22]" in line or "[LORA" in line
                  or "[STATS]" in line or "[ROLE_ID]" in line or "[BOOT]" in line):
                tag = "lora"
            elif "ok=" in line or "Ready" in line or "PASS" in line:
                tag = "ok"
            elif "err=" in line and "err:0" not in line and "err:0," not in line:
                tag = "err"
                
            self.console.insert(tk.END, f"[{ts}] {line}\n", tag)

            # 定期截斷終端，防記憶體飆升 (上限 2000 行)。用 Python 計數器取代逐行呼叫
            # console.index('end-1c')：後者每次都要跟 Tcl 往返一次，高頻遙測時累積下來
            # 是主執行緒卡頓的來源之一，現在只有真的超過門檻要裁切時才觸發一次 Tcl 呼叫。
            self._console_line_count = getattr(self, "_console_line_count", 0) + 1
            if self._console_line_count > 2000:
                self.console.delete("1.0", "200.0")
                self._console_line_count -= 199

            # 指令/回應分流 + 解析（單行出錯不得中斷輪詢，否則 poll_queue 停止重排→GUI 凍結）
            try:
                # Flash CSV 導出串流捕獲
                self._handle_flash_export_stream_line(line)

                # 地面站送出上行命令時記下 seq→標籤，供稍後 [ACK] 比對（deploy 三種共用
                # cmd 文字 "deploy"，只能靠 seq 分辨副傘/主傘/雙傘）
                m_uplink = re.search(r"\[UPLINK\] 送 (\S+) \(cmd=0x[0-9A-Fa-f]+ seq=(\d+)\)", line)
                if m_uplink:
                    self._uplink_seq_label[int(m_uplink.group(2))] = m_uplink.group(1)

                # 判斷航電 [ARM] / [ACK] 回應並更新 UI 狀態標籤與事件紀錄
                if "[ARM] SUCCESS:" in line:
                    if "STATE_PAD_ARMED" in line:
                        self.update_fsm_state_ui("STATE_PAD_ARMED")
                        self._event_log(f"[{ts}] ⚡ [ARM ACK] 航電已成功武裝！(STATE_PAD_ARMED)", "ok")
                    elif "STATE_PAD" in line:
                        self.update_fsm_state_ui("STATE_PAD")
                        self._event_log(f"[{ts}] 🛡️ [ARM ACK] 航電已解除武裝 (STATE_PAD)", "ok")
                elif "[ARM] WARNING:" in line:
                    self._event_log(f"[{ts}] ⚠️ [ARM ACK] 航電拒絕武裝變更: {line}", "err")
                elif "[ACK]" in line:
                    if "cmd:\"arm\"" in line or "cmd:arm" in line:
                        if "status:OK" in line:
                            self.update_fsm_state_ui("STATE_PAD_ARMED")
                            self._event_log(f"[{ts}] ⚡ [ACK] 航電確認武裝 (ARM OK)", "ok")
                        else:
                            self._event_log(f"[{ts}] ❌ [ACK] 航電拒絕武裝 (ARM REJECTED)", "err")
                    elif "cmd:\"disarm\"" in line or "cmd:disarm" in line:
                        if "status:OK" in line:
                            self.update_fsm_state_ui("STATE_PAD")
                            self._event_log(f"[{ts}] 🛡️ [ACK] 航電確認上鎖 (DISARM OK)", "ok")
                        else:
                            self._event_log(f"[{ts}] ❌ [ACK] 航電拒絕上鎖 (DISARM REJECTED)", "err")
                    elif "cmd:\"bench\"" in line or "cmd:bench" in line:
                        self.update_bench_monitor(line)
                        if "status:OK" in line:
                            self._event_log(f"[{ts}] 🖥️ [ACK] 桌面測試完成 (BENCH OK)", "ok")
                        else:
                            self._event_log(f"[{ts}] ❌ [ACK] 桌面測試被拒 (BENCH REJECTED)", "err")
                    elif "cmd:\"deploy\"" in line or "cmd:deploy" in line:
                        self._handle_deploy_ack(ts, line)
                    elif "cmd:\"recovery\"" in line or "cmd:recovery" in line:
                        self._handle_recovery_ack(ts, line)
                elif "[BENCH]" in line or "[PYRO-SELFTEST]" in line:
                    self.update_bench_monitor(line)
                    if "桌面測試" in line or "序列完成" in line:
                        self._event_log(f"[{ts}] 🖥️ {line.strip()}", "ok")
                    elif "拒絕" in line:
                        self._event_log(f"[{ts}] ⚠️ {line.strip()}", "err")

                if "[LINK]" in line and ("self_arb" in line or "peer_arb" in line):
                    self.update_bench_monitor(line)

                if self._is_event_line(line):
                    self._event_log(f"[{ts}] {line}", self._event_tag(line))
                self.parse_telemetry(line)
            except Exception as e:
                try:
                    self.console.insert(tk.END, f"[GUI] ⚠ 處理行例外: {e}\n", "err")
                except Exception:
                    pass

        # 整批處理完才捲動一次到底部，取代逐行 see(tk.END)（每次 see 都會觸發 Text
        # 版面配置計算，逐行呼叫在高頻遙測時是主執行緒卡頓的一大來源）。
        if lines_processed > 0:
            try:
                self.console.see(tk.END)
            except tk.TclError:
                pass

        # 對端電梯測試 profile 新鮮度窗口到期要自動收合橫幅（見 _refresh_elevator_banner），
        # 沒有新的警告行進來時也得靠這裡定時檢查，不能只在收到行時才刷新。
        self._refresh_elevator_banner()

        # 繼續定時輪詢
        try:
            if self.root.winfo_exists():
                self.root.after(10, self.poll_queue)
        except tk.TclError:
            pass

    # ------------------ 指令/回應區（只收 CMD 與指令回應） ------------------
    # 白名單：使用者送出的 [CMD] + 韌體對「指令」的回應標籤。開機/角色/系統/遙測一律不進。
    # [UPLINK]=地面站送上行命令的狀態；[ACK]=主航電對遠端指令的下行回覆。
    _EVENT_KEEP_TAGS = ("[CMD]", "[CAL]", "[E22]", "[E80]", "[LORA433]", "[UPLINK]", "[ACK]")

    def _is_event_line(self, line):
        """只有使用者指令與其回應才進「指令/回應」區。"""
        return bool(line) and any(tag in line for tag in self._EVENT_KEEP_TAGS)

    def _event_tag(self, line):
        if "[ACK]" in line:
            # ACK 依 status 著色：非 OK（UNKNOWN/BADARG/UNARMED/REJECTED）視為警示
            if "status:OK" in line:
                return "resp"
            return "err"
        if "[UPLINK]" in line:
            # burst 結束行帶實際發射次數（gs_lora_test.c uplink_burst）：ok=0 代表本機
            # E22 全程忙線、命令根本沒上空中——那是失敗，不能跟一般送出訊息同色，
            # 否則又會像舊版「一律印送出完畢」那樣看不出來。
            if "ok=0 " in line or "⚠" in line:
                return "err"
            return "cmd"
        if "[CMD]" in line:
            return "cmd"
        if "ERROR" in line or "FAIL" in line or "❌" in line:
            return "err"
        return "resp"

    def _event_log(self, text, tag="resp"):
        """寫入重要訊息區（若已建立）；截斷上限 800 行、自動捲到底。"""
        if not hasattr(self, 'event_console'):
            return
        try:
            self.event_console.insert(tk.END, text + "\n", tag)
            if float(self.event_console.index('end-1c')) > 800.0:
                self.event_console.delete("1.0", "100.0")
            self.event_console.see(tk.END)
        except tk.TclError:
            pass

    def _set_flash_export_status(self, text, color="#00d2ff"):
        if hasattr(self, 'lbl_flash_export_status') and self.lbl_flash_export_status.winfo_exists():
            self.lbl_flash_export_status.config(text=text, fg=color)

    def _close_flash_export_file(self):
        export_file = getattr(self, '_flash_export_file', None)
        if export_file:
            try:
                export_file.flush()
                export_file.close()
            except Exception:
                pass
        self._flash_export_file = None
        self._flash_export_writer = None
        sysflags_file = getattr(self, '_flash_export_sysflags_file', None)
        if sysflags_file:
            try:
                sysflags_file.flush()
                sysflags_file.close()
            except Exception:
                pass
        self._flash_export_sysflags_file = None
        for flight_file in getattr(self, '_flash_export_flight_files', {}).values():
            try:
                flight_file.flush()
                flight_file.close()
            except Exception:
                pass
        self._flash_export_flight_files = {}
        self._flash_export_flight_writers = {}

    def _write_flash_export_info(self, finished_at=None):
        # 總覽檔放頂層（同時涵蓋 raw/ 與 processed/ 兩個子資料夾），方便使用者一眼看懂結構。
        export_dir = getattr(self, '_flash_export_dir', None)
        if not export_dir:
            return
        try:
            info_path = os.path.join(export_dir, "export_info.txt")
            flight_counts = getattr(self, '_flash_export_flight_counts', {})
            with open(info_path, "w", encoding="utf-8") as info:
                info.write("RocketCom Flash Export\n")
                info.write(f"started_at={getattr(self, '_flash_export_started_at', '')}\n")
                if finished_at:
                    info.write(f"finished_at={finished_at}\n")
                info.write(f"total_ring_rows={getattr(self, '_flash_export_count', 0)}\n")
                info.write(f"skipped_lines={getattr(self, '_flash_export_skipped', 0)}\n")
                info.write("raw/       = 原始資料：ring_buffer_all.csv, sysflags_sector0.hex, flight_id_*.csv\n")
                info.write("processed/ = 整理後資料：flight_report_*.txt/.md, flight_analysis_*.png,\n")
                info.write("             flight_map_*.html（GPS 互動式地圖）, sysflags_summary.txt\n")
                for flight_id in sorted(flight_counts, key=lambda x: (0, int(x)) if str(x).isdigit() else (1, str(x))):
                    info.write(f"flight_id_{flight_id}_rows={flight_counts[flight_id]}\n")
        except Exception:
            pass

    def _finish_flash_export(self, ok=True, message=None):
        self._flash_export_writing = False
        self._flash_export_active = False
        finished_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._write_flash_export_info(finished_at)
        self._close_flash_export_file()

        rows = getattr(self, '_flash_export_count', 0)
        skipped = getattr(self, '_flash_export_skipped', 0)
        export_dir = getattr(self, '_flash_export_dir', '')
        raw_dir = getattr(self, '_flash_export_raw_dir', export_dir)
        processed_dir = getattr(self, '_flash_export_processed_dir', export_dir)

        # 自動生成人類可讀分析報告與圖表（讀 raw/，寫入 processed/）
        if ok and raw_dir and os.path.exists(raw_dir):
            try:
                tools_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools"))
                if tools_dir not in sys.path:
                    sys.path.append(tools_dir)
                import flash_analyzer

                os.makedirs(processed_dir, exist_ok=True)

                # 1. 解碼 Sector 0 系統參數與校準旗標（即便飛行紀錄為 0 筆也會解析）
                sysflags_hex = os.path.join(raw_dir, "sysflags_sector0.hex")
                if os.path.exists(sysflags_hex):
                    flash_analyzer.generate_sysflags_report(sysflags_hex, processed_dir)

                # 2. 解析 Ring Buffer CSV 飛行數據 → 事件偵測 + 指標 + 健康檢核 → 報告/圖表/地圖
                all_csv = os.path.join(raw_dir, "ring_buffer_all.csv")
                if os.path.exists(all_csv):
                    flights, _parse_stats = flash_analyzer.parse_flash_csv(all_csv)
                    for fid, recs in flights.items():
                        t0 = recs[0]['tick_ms'] / 1000.0
                        times = [(r['tick_ms'] / 1000.0) - t0 for r in recs]
                        events = flash_analyzer.detect_events(recs, times)
                        metrics = flash_analyzer.compute_metrics(recs, times, events)
                        health = flash_analyzer.audit_health(recs, events)
                        flash_analyzer.generate_flight_report(fid, recs, events, metrics, health, processed_dir)
                        flash_analyzer.generate_flight_charts(fid, recs, events, metrics, processed_dir)
                        flash_analyzer.generate_flight_map(fid, recs, events, metrics, processed_dir)
                    print(f"[ANALYZER] 已為 {len(flights)} 次飛行生成分析報告/圖表/地圖於 {processed_dir}")
            except Exception as e:
                print(f"[ANALYZER] 自動生成圖表報告失敗: {e}")

        if message is None:
            if ok:
                message = (f"✅ 匯出完成！有效 {rows} 筆，原始資料於 raw/，"
                           f"已自動生成報告/圖表/地圖於 processed/（{os.path.basename(export_dir)}）")
            else:
                message = f"⚠️ 匯出中止。有效 {rows} 筆，略過 {skipped} 行"
        self._set_flash_export_status(message, "#00e676" if ok else "#ffcc00")

    def _parse_flash_export_row(self, line):
        try:
            row = next(csv.reader([line]))
        except csv.Error:
            return None
        row = [cell.strip() for cell in row]
        if row == FLASH_EXPORT_COLUMNS:
            return "header"
        if len(row) != len(FLASH_EXPORT_COLUMNS):
            return None
        if not FLASH_EXPORT_ADDR_RE.match(row[0] or ""):
            return None
        if not all(FLASH_EXPORT_NUMERIC_RE.match(cell or "") for cell in row[1:]):
            return None
        return row

    def _flash_export_writer_for_flight(self, flight_id):
        writers = getattr(self, '_flash_export_flight_writers', {})
        if flight_id in writers:
            return writers[flight_id]

        raw_dir = getattr(self, '_flash_export_raw_dir', None)
        if not raw_dir:
            return None
        filename = f"flight_id_{int(flight_id):06d}.csv" if str(flight_id).isdigit() else f"flight_id_{flight_id}.csv"
        path = os.path.join(raw_dir, filename)
        flight_file = open(path, "w", encoding="utf-8", newline="")
        writer = csv.writer(flight_file, lineterminator="\n")
        writer.writerow(FLASH_EXPORT_COLUMNS)
        self._flash_export_flight_files[flight_id] = flight_file
        self._flash_export_flight_writers[flight_id] = writer
        return writer

    def _handle_flash_export_stream_line(self, line):
        if not getattr(self, '_flash_export_active', False):
            return

        if "--- SYSFLAGS_START ---" in line:
            self._flash_export_mode = "sysflags"
            sysflags_path = os.path.join(self._flash_export_raw_dir, "sysflags_sector0.hex")
            self._flash_export_sysflags_file = open(sysflags_path, "w", encoding="utf-8")
            self._flash_export_sysflags_file.write(f"# exported_at={datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self._set_flash_export_status("📥 正在讀取旗標區 Sector 0...", "#00d2ff")
            return

        if "--- SYSFLAGS_END ---" in line:
            self._flash_export_mode = None
            sysflags_file = getattr(self, '_flash_export_sysflags_file', None)
            if sysflags_file:
                sysflags_file.flush()
            self._set_flash_export_status("📥 旗標區已儲存，等待 Ring Buffer CSV...", "#00d2ff")
            return

        if "--- CSV_START ---" in line:
            if not getattr(self, '_flash_export_file', None):
                return
            self._flash_export_mode = "csv"
            self._flash_export_writing = True
            self._flash_export_count = 0
            self._flash_export_skipped = 0
            writer = getattr(self, '_flash_export_writer', None)
            if writer:
                writer.writerow(FLASH_EXPORT_COLUMNS)
            self._set_flash_export_status("📥 正在匯出中... 已建立結構化 CSV 表頭", "#00d2ff")
            return

        if "--- CSV_END ---" in line or "[FLASH] Export finished" in line:
            self._finish_flash_export(ok=True)
            return

        if getattr(self, '_flash_export_mode', None) == "sysflags":
            if FLASH_SYSFLAGS_HEX_RE.match(line):
                sysflags_file = getattr(self, '_flash_export_sysflags_file', None)
                if sysflags_file:
                    sysflags_file.write(line + "\n")
            else:
                self._flash_export_skipped = getattr(self, '_flash_export_skipped', 0) + 1
            return

        if not getattr(self, '_flash_export_writing', False):
            return

        parsed = self._parse_flash_export_row(line)
        if parsed == "header":
            return
        if parsed is None:
            self._flash_export_skipped = getattr(self, '_flash_export_skipped', 0) + 1
            return

        writer = getattr(self, '_flash_export_writer', None)
        if not writer:
            self._flash_export_skipped = getattr(self, '_flash_export_skipped', 0) + 1
            return

        writer.writerow(parsed)
        flight_id = parsed[1]
        flight_writer = self._flash_export_writer_for_flight(flight_id)
        if flight_writer:
            flight_writer.writerow(parsed)
            counts = getattr(self, '_flash_export_flight_counts', {})
            counts[flight_id] = counts.get(flight_id, 0) + 1
            self._flash_export_flight_counts = counts
        self._flash_export_count = getattr(self, '_flash_export_count', 0) + 1
        if self._flash_export_count % 10 == 0:
            self._set_flash_export_status(
                f"📥 正在匯出中... 已寫入 {self._flash_export_count} 筆結構化資料",
                "#00d2ff"
            )
        if self._flash_export_count % 50 == 0:
            try:
                self._flash_export_file.flush()
            except Exception:
                pass

    def parse_telemetry(self, line):
        """解析串口封包，更新 3D 姿態與頂部狀態數位卡片"""
        # 0. 角色偵測 + LoRa 參數回報（三角色通用）
        self.parse_role_and_lora(line)

        # 0b. 10Hz 裸 CSV 原始遙測行（飛行板）：
        #     bmi_ax,ay,az(mG), adxl_ax,ay,az(mG), temp(x100), press(Pa), baro_alt(cm)
        if line and (line[0] == '-' or line[0].isdigit()) and line.count(',') == 8:
            try:
                v = [int(x) for x in line.split(',')]
            except ValueError:
                v = None
            if v is not None:
                t = self.now_t()
                self.ts_acc_bmi.append((t, float(np.sqrt(v[0]**2 + v[1]**2 + v[2]**2)) / 1000.0))
                self.ts_acc_adxl.append((t, float(np.sqrt(v[3]**2 + v[4]**2 + v[5]**2)) / 1000.0))
                # v[8] 是絕對海拔；逐點扣掉當下生效的 pad_ref，跟韌體 in.baro_alt_rel 的
                # 算法一致，也讓這條線跟 EKF/VF 共用發射台零點（原本差一個海拔常數偏移）。
                baro_rel = self.pad_ref.rel(v[8] / 100.0)
                if baro_rel is not None:
                    self.ts_alt_baro.append((t, baro_rel))
                self.chart_dirty = True
                return

        # A. 解析四元數 EKF 姿態封包 [TELE] pos:x,y,z vel:x,y,z q:qw,qx,qy,qz
        if "[TELE]" in line and "q:" in line:
            m = re.search(r"q:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                q = [float(m.group(i)) for i in range(1, 5)]
                self.last_q = q
                # 只設旗標，實際重畫交給 attitude_redraw_loop 節流（見該函式說明）
                self.att_dirty = True
            # EKF 高度（pos z）與垂直速度（vel z）→ 飛行圖表 + 即時卡片
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

        # A2. USB 直連飛行板的本板 VF 原始輸出（1Hz）：
        #     [VF] h_cm=.. v_cms=.. | EKF h_cm=.. v_cms=.. | braw_cm=..
        #     （EKF 那半段跟 [TELE] pos/vel 重複，這裡只取 VF 半段 + braw_cm 相對地面高度原始值）
        elif "[VF]" in line and "h_cm=" in line:
            m = re.search(r"h_cm=(-?\d+)\s+v_cms=(-?\d+)", line)
            if m:
                t = self.now_t()
                self.ts_vf_alt.append((t, int(m.group(1)) / 100.0))
                self.ts_vf_vz.append((t, int(m.group(2)) / 100.0))
                self.chart_dirty = True
            m_braw = re.search(r"braw_cm=(-?\d+)", line)
            if m_braw:
                # 航電自己算的 baro_alt_rel：「航電認為自己相對起點多高」的原始值，
                # 優先於 ts_alt_baro 的 10Hz 推導版顯示於卡片。
                braw = int(m_braw.group(1)) / 100.0
                self.ts_ground_alt.append((self.now_t(), braw))
                self.cards["ground_alt"].config(text=f"{braw:+.1f} m")
                self.chart_dirty = True

        # B. 解析頻率封包 [RATE] BMI088_A:xxHz, ...
        elif "[RATE]" in line:
            # 提取採樣率數字
            m_a = re.search(r"BMI088_A:([\d\.]+)Hz", line)
            m_g = re.search(r"BMI088_G:([\d\.]+)Hz", line)
            m_x = re.search(r"ADXL375:([\d\.]+)Hz", line)
            m_b = re.search(r"BMP388:([\d\.]+)Hz", line)
            m_m = re.search(r"MMC5983:([\d\.]+)Hz", line)
            m_gps = re.search(r"GPS:([\d\.]+)Hz", line)
            m_drop = re.search(r"EKF_DROP:(\d+)", line)
            
            if m_a: self.cards["bmi_a"].config(text=f"{float(m_a.group(1)):.2f} Hz")
            if m_g: self.cards["bmi_g"].config(text=f"{float(m_g.group(1)):.2f} Hz")
            if m_x: self.cards["adxl"].config(text=f"{float(m_x.group(1)):.2f} Hz")
            if m_b: self.cards["bmp"].config(text=f"{float(m_b.group(1)):.2f} Hz")
            if m_m: self.cards["mag"].config(text=f"{float(m_m.group(1)):.2f} Hz")
            if m_gps:
                gps_rate = float(m_gps.group(1))
                self.cards["gps"].config(text=f"{gps_rate:.2f} Hz")
                self.gps_status_info["rate"] = gps_rate
                self.update_gps_status_panel()
            if m_drop: self.lbl_drops.config(text=f"EKF Queue Drops: {m_drop.group(1)}")
            
        # C. 解析 Flash 封包 [FLASH_RING] PKT_TOTAL:xx ADDR:xx
        elif "[FLASH_RING]" in line and "PKT_TOTAL:" in line:
            m_pkt = re.search(r"PKT_TOTAL:(\d+)", line)
            if m_pkt:
                self.cards["flash_pkt"].config(text=f"{m_pkt.group(1)} Pkts")

        # C1. 預擦除雙板進度 [FLASH_RING] primary=45%(rdy) backup=30%  self=PRIMARY 432/960  peer=OK
        elif "[FLASH_RING]" in line and "primary=" in line and "self=" in line:
            m_er = re.search(
                r"primary=(\d+)%(?:\(rdy\))?\s+backup=(\d+)%(?:\(rdy\))?"
                r"\s+self=(\w+)\s+(?:(\d+)/(\d+)\s+)?peer=(\w+)", line)
            if m_er:
                pri_pct = int(m_er.group(1))
                bak_pct = int(m_er.group(2))
                cur_sec = m_er.group(4)
                tot_sec = m_er.group(5)
                self._update_erase_progress(pri_pct, bak_pct, cur_sec, tot_sec)

        # C2. 主/備板間鏈路溝通狀態 [LINK] self:.. peer:.. link:OK/STALE/NONE state:.. flags:.. age:..ms
        #     （新版尾段：sync=OK/NO lost=.. desync=.. self_arb=.. peer_arb=.. peer_flash=.. primary_erase=..% backup_erase=..%
        #      ph_cm=.. pv_cms=.. pbaro_cm=.. paz_cg=.. pvfh_cm=.. pvfv_cms=.. pbmi_cg=.. padxl_cg=..
        #      ← 對端最近回報的 EKF 高度/速度、baro 相對高度、加速度(單軸)、VF 高度/速度、
        #      BMI088/ADXL375 加速度模長，讓 USB 直連單板時不必透過地面站 LoRa 轉發也能畫出
        #      完整副航電曲線，見 main.c [LINK] printf 旁註解）
        elif "[LINK]" in line:
            m = re.search(
                r"self=(\w+)\s+peer=(\w+)\s+link=(\w+)\s+state=(\w+)\s+flags=0x([0-9A-Fa-f]+)\s+age=(\d+)ms"
                r"(?:\s+sync=(\w+)\s+lost=(\d+)\s+desync=(\d+))?"
                r"(?:\s+self_arb=(\w+)\s+peer_arb=(\w+))?"
                r"(?:\s+peer_flash=(\w+))?"
                r"(?:\s+primary_erase=(\d+)%\s+backup_erase=(\d+)%)?"
                r"(?:\s+ph_cm=(-?\d+)\s+pv_cms=(-?\d+))?"
                r"(?:\s+pbaro_cm=(-?\d+)\s+paz_cg=(-?\d+)\s+pvfh_cm=(-?\d+)\s+pvfv_cms=(-?\d+))?"
                r"(?:\s+pbmi_cg=(-?\d+)\s+padxl_cg=(-?\d+))?",
                line)
            if m:
                (self_role, peer_role, link_ok, peer_state, flags_hex, age_ms,
                 sync, lost, desync, self_arb, peer_arb,
                 peer_flash, primary_erase, backup_erase,
                 ph_cm, pv_cms, pbaro_cm, paz_cg, pvfh_cm, pvfv_cms,
                 pbmi_cg, padxl_cg) = m.groups()
                self.update_link_status(self_role, peer_role, link_ok, peer_state,
                                        int(flags_hex, 16), int(age_ms),
                                        sync=sync, lost=lost, desync=desync,
                                        self_arb=self_arb, peer_arb=peer_arb)
                # 更新擦除進度（從 [LINK] 1Hz 診斷取得持續更新）
                if primary_erase is not None and backup_erase is not None:
                    self._update_erase_progress(int(primary_erase), int(backup_erase))
                # USB 直連主航電時，對端（backup）EKF/baro/VF/加速度靠這條 1Hz 鏈路帶進來
                # （底層板間鏈路本身是 20Hz，這裡只是把已經在收的資料印出來），不需要地面站
                # LoRa 轉發。只在 peer 確實回報 BACKUP 角色時才收，避免 USB 若改接副航電時
                # 把對端(此時是 PRIMARY)資料誤標成 backup 序列。
                if ph_cm is not None and pv_cms is not None and peer_role == "BACKUP":
                    t = self.now_t()
                    self.ts_backup_alt.append((t, int(ph_cm) / 100.0))
                    self.ts_backup_vz.append((t, int(pv_cms) / 100.0))
                    if pbaro_cm is not None:
                        # 副板 baro 是絕對海拔，借主板 pad_ref 扣（見 ln_alt_backup_baro 註解）
                        pbaro_rel = self.pad_ref.rel(int(pbaro_cm) / 100.0)
                        if pbaro_rel is not None:
                            self.ts_backup_baro.append((t, pbaro_rel))
                        self.ts_backup_acc.append((t, int(paz_cg) / 100.0))
                        self.ts_backup_vf_alt.append((t, int(pvfh_cm) / 100.0))
                        self.ts_backup_vf_vz.append((t, int(pvfv_cms) / 100.0))
                    if pbmi_cg is not None:
                        self.ts_backup_acc_bmi.append((t, int(pbmi_cg) / 100.0))
                        self.ts_backup_acc_adxl.append((t, int(padxl_cg) / 100.0))
                    self.chart_dirty = True

        # D. 解析 FSM 狀態轉移以動態更新 HUD
        elif "[FSM]" in line:
            # 發射台氣壓零點重零（PAD 期每 30s，ARM 後停）——相對起點高度的分母
            if self.pad_ref.feed(line):
                self._refresh_pad_ref_card()
                return
            # 格式 A（1Hz heartbeat）：[FSM] state=PAD role=PRIMARY
            m_heartbeat = re.search(r"state=([A-Z_]+)", line)
            # 格式 B（事件）：[FSM] [ARMED] ... Entering STATE_PAD_ARMED.
            m_state = re.search(r"(STATE_[A-Z_]+)", line)

            # 短名稱（link_fsm_state_name）→ STATE_ 前綴對照表
            _SHORTNAME_MAP = {
                "INIT": "STATE_INIT", "PAD": "STATE_PAD",
                "BOOST": "STATE_BOOST", "COAST": "STATE_COAST",
                "DEP_DROGUE": "STATE_DROGUE", "APOGEE": "STATE_APOGEE",
                "DESCENT": "STATE_DESCENT", "MAIN_DEPLOY": "STATE_MAIN",
                "LANDED": "STATE_LANDED", "PAD_ARMED": "STATE_PAD_ARMED",
            }
            if m_heartbeat:
                short = m_heartbeat.group(1)
                full = _SHORTNAME_MAP.get(short, f"STATE_{short}")
                self.update_fsm_state_ui(full)
                # 同時更新 lbl_role 欄位（本板角色）
                m_role = re.search(r"role=(\w+)", line)
                if m_role and hasattr(self, 'lbl_role') and self.lbl_role.winfo_exists():
                    role_map = {"PRIMARY": "🚀 主航電 PRIMARY", "BACKUP": "🛸 副航電 BACKUP", "GROUND": "🌍 地面站 GS"}
                    self.lbl_role.config(text=role_map.get(m_role.group(1), m_role.group(1)), fg="#aaaaaa")
            elif m_state:
                self.update_fsm_state_ui(m_state.group(1))
            elif "LIFTOFF" in line:
                self.update_fsm_state_ui("STATE_BOOST")
            elif "BURNOUT" in line:
                self.update_fsm_state_ui("STATE_COAST")
            elif "APOGEE" in line:
                self.update_fsm_state_ui("STATE_RECOVERY")
            elif "LANDED" in line:
                self.update_fsm_state_ui("STATE_LANDED")
                
        # E. 解析 [IMU]
        elif "[IMU]" in line:
            m = re.search(r"a\[mG\]:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+) g\[dps\]:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                self.latest_imu = {
                    "ax": float(m.group(1)), "ay": float(m.group(2)), "az": float(m.group(3)),
                    "gx": float(m.group(4)), "gy": float(m.group(5)), "gz": float(m.group(6))
                }
                
        # F. 解析 [HIGHG]
        elif "[HIGHG]" in line:
            m = re.search(r"a\[mG\]:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)", line)
            if m:
                self.latest_highg = {
                    "ax": float(m.group(1)), "ay": float(m.group(2)), "az": float(m.group(3))
                }
                
        # G. 解析 [MAG]
        elif "[MAG] B[mG]" in line or ("[MAG]" in line and "B[mG]" in line):
            m = re.search(r"B\[mG\]:(-?[\d\.]+),(-?[\d\.]+),(-?[\d\.]+)\s+hdg:(-?[\d\.]+)", line)
            if m:
                self.latest_mag = {
                    "mx": float(m.group(1)), "my": float(m.group(2)), "mz": float(m.group(3)),
                    "hdg": float(m.group(4))
                }
                if self.collecting_data:
                    self.calib_x.append(self.latest_mag["mx"])
                    self.calib_y.append(self.latest_mag["my"])
                    self.calib_z.append(self.latest_mag["mz"])

        # H. 解析 [PWR]（飛行板 1Hz 電池電壓）
        elif "[PWR]" in line and "bat:" in line:
            m = re.search(r"bat:(\d+)mV", line)
            if m:
                bat_v = float(m.group(1)) / 1000.0
                bat_color = "#4CAF50" if bat_v >= 7.4 else ("#FFC107" if bat_v >= 6.8 else "#E91E63")
                self.cards["bat"].config(text=f"{bat_v:.2f} V", foreground=bat_color)

        # I. 解析 [GPS]（飛行板 1Hz）：定位 → 地圖軌跡；海拔/地速 → 圖表原始序列
        elif "[GPS]" in line and "fix:" in line:
            m = re.search(r"fix:(\d+) q:\d+ sat:(\d+) ([+-])(\d+\.\d+),([+-])(\d+\.\d+) alt:(-?\d+)m spd:(-?\d+)cm/s(?: stale:(\d+) ok:(\d+) err:(\d+))?", line)
            if m:
                fix_val = int(m.group(1))
                sats_val = int(m.group(2))
                self.gps_status_info["fix"] = fix_val
                self.gps_status_info["sats"] = sats_val
                if m.group(9) is not None:
                    self.gps_status_info["stale"] = int(m.group(9))
                if m.group(10) is not None:
                    self.gps_status_info["ok"] = int(m.group(10))
                if m.group(11) is not None:
                    self.gps_status_info["err"] = int(m.group(11))
                self.update_gps_status_panel()
                
                if fix_val == 1:
                    lat = float(m.group(4)) * (1.0 if m.group(3) == '+' else -1.0)
                    lon = float(m.group(6)) * (1.0 if m.group(5) == '+' else -1.0)
                    if abs(lat) > 0.01 or abs(lon) > 0.01:   # 排除 0,0 假定位
                        alt = int(m.group(7))
                        spd = int(m.group(8)) / 100.0
                        t = self.now_t()
                        self.ts_alt_gps.append((t, float(alt)))
                        self.ts_spd_gps.append((t, spd))
                        self.on_gps_fix(lat, lon, alt_m=alt, spd_ms=spd, sats=sats_val)
                        self.chart_dirty = True

        # J. 解析 [GS_PKT]（地面站收到的火箭下行封包摘要）→ 圖表 + 地圖
        elif "[GS_PKT]" in line:
            t = self.now_t()
            
            # Parse link frequency to increment packet counters
            m_link = re.search(r"link:(\d+)", line)
            if m_link:
                freq = int(m_link.group(1))
                if freq == 433:
                    self.gs_pkt_cnt_433 += 1
                elif freq == 920:
                    self.gs_pkt_cnt_920 += 1
                self.gs_pkt_cnt_total += 1
                self.lbl_gs_pkts.config(text=f"📦 接收包數: {self.gs_pkt_cnt_total} (433:{self.gs_pkt_cnt_433} | 920:{self.gs_pkt_cnt_920})")

            # 主/副協同（Phase A/D）：主板經下鏈中繼的副板摘要 → 雙板監看徽章 + live chart 比對
            # ★2026-07-30：下鏈的對端摘要只剩 VF + baro + 鏈路健康位（EKF 高度/速度、
            # 丟包率、高G 已從封包移除，見 telemetry.h 同日註解）。USB 直連主航電時
            # 上面 [LINK] 那條路徑仍有完整的對端 EKF/高G，不受影響。
            mp = re.search(r"peer:(\d+) pflags:0x([0-9A-Fa-f]+) "
                           r"plink:0x([0-9A-Fa-f]+) "
                           r"pbaro:(-?\d+)cm pvfh:(-?\d+)cm pvfv:(-?\d+)cms"
                           r"(?:\s+pbarb:(\d+))?", line)
            if mp:
                self.update_peer_relay(int(mp.group(1)), int(mp.group(2), 16),
                                       int(mp.group(3), 16),
                                       int(mp.group(4)) / 100.0,
                                       int(mp.group(5)) / 100.0, int(mp.group(6)) / 100.0,
                                       pbarb=int(mp.group(7)) if mp.group(7) is not None else 0)

            # 電梯測試 profile 醒目提示（經地面站板 LoRa 中繼時走這條路徑；直連航電板走
            # [PAD_CFG]/[ELEVATOR_TEST_WARNING]，見 _handle_pad_cfg_profile_line 一帶）。
            m_prof = re.search(r"prof:0x([0-9A-Fa-f]+)", line)
            if m_prof:
                self._handle_gs_pkt_profile(m_prof.group(1))

            # ★2026-07-31：Flash 未擦除提醒（bit1 = TELEM_ARM_NEED_ERASE）。舊版地面站韌體
            # 的 [GS_PKT] 沒有 armf 欄位，match 不到就完全不動橫幅（維持既有行為）。
            m_armf = re.search(r"armf:0x([0-9A-Fa-f]+)", line)
            if m_armf:
                self._handle_gs_pkt_arm_flags(int(m_armf.group(1), 16))

            m = re.search(r"alt:(-?\d+)cm", line)
            if m:
                alt = int(m.group(1)) / 100.0
                self.ts_alt_ekf.append((t, alt))
                self.cards["alt"].config(text=f"{alt:.1f} m")
            m = re.search(r"vz:(-?\d+)cms", line)
            if m:
                vz = int(m.group(1)) / 100.0
                self.ts_vz_ekf.append((t, vz))
                self.cards["vz"].config(text=f"{vz:+.1f} m/s")
            m = re.search(r"baro:(-?\d+)cm", line)
            if m:
                # 同 10Hz CSV：下鏈的 baro 是絕對海拔，扣 pad_ref 才跟 EKF/VF 同零點
                baro_rel = self.pad_ref.rel(int(m.group(1)) / 100.0)
                if baro_rel is not None:
                    self.ts_alt_baro.append((t, baro_rel))
            m = re.search(r"vfh:(-?\d+)cm vfv:(-?\d+)cms", line)
            if m:
                self.ts_vf_alt.append((t, int(m.group(1)) / 100.0))
                self.ts_vf_vz.append((t, int(m.group(2)) / 100.0))
            m = re.search(r"accel:(-?\d+),(-?\d+),(-?\d+)", line)
            if m:
                ax_, ay_, az_ = (int(m.group(i)) for i in range(1, 4))
                self.ts_acc_bmi.append((t, float(np.sqrt(ax_**2 + ay_**2 + az_**2)) / 1000.0))
            m = re.search(r"gps:(\d+)/(\d+)", line)
            sats = int(m.group(1)) if m else None
            fix = (m and m.group(2) == "1")
            if sats is not None:
                self.gps_status_info["sats"] = sats
                self.gps_status_info["fix"] = 1 if fix else 0
                self.update_gps_status_panel()
            m = re.search(r"pos:([+-])(\d+\.\d+),([+-])(\d+\.\d+)", line)
            if fix and m:
                lat = float(m.group(2)) * (1.0 if m.group(1) == '+' else -1.0)
                lon = float(m.group(4)) * (1.0 if m.group(3) == '+' else -1.0)
                if abs(lat) > 0.01 or abs(lon) > 0.01:
                    ma = re.search(r"galt:(-?\d+)m", line)
                    galt = int(ma.group(1)) if ma else None
                    if galt is not None:
                        self.ts_alt_gps.append((t, float(galt)))
                    self.on_gps_fix(lat, lon, alt_m=galt, sats=sats)
            self.chart_dirty = True

        # K. 解析 [GS_GPS]（地面站自身定位）→ 地圖 GS 標記
        elif "[GS_GPS]" in line and "FIX" in line:
            m = re.search(r"Pos:([+-])(\d+\.\d+),([+-])(\d+\.\d+)", line)
            if m:
                lat = float(m.group(2)) * (1.0 if m.group(1) == '+' else -1.0)
                lon = float(m.group(4)) * (1.0 if m.group(3) == '+' else -1.0)
                if abs(lat) > 0.01 or abs(lon) > 0.01:
                    self.gs_own_pos = (lat, lon)
                    self.map_dirty = True

        # H. 解析 MMC5983 初始或儲存的偏移量
        if "[MAG] MMC5983MA online. offset[X,Y,Z]=" in line:
            m = re.search(r"offset\[X,Y,Z\]=(-?\d+),(-?\d+),(-?\d+)", line)
            if m:
                self.board_mag_offsets = [float(m.group(1)), float(m.group(2)), float(m.group(3))]
        elif "[CAL] Mag hard-iron offset saved to Flash" in line:
            # 韌體格式（ekf.c:1303）：
            #   [CAL] Mag hard-iron offset saved to Flash (x10): <x*10>, <y*10>, <z*10>
            # 值是實際 counts 的 10 倍（整數傳輸保一位小數），需 ÷10 還原。
            m = re.search(r"saved to Flash \(x10\):\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)", line)
            if m:
                self.board_mag_offsets = [int(m.group(1)) / 10.0,
                                          int(m.group(2)) / 10.0,
                                          int(m.group(3)) / 10.0]
                self._on_mag_write_confirmed()
        elif "[CAL] EKF Mag Yaw Lock set to:" in line:
            m = re.search(r"Mag Yaw Lock set to:\s*(\d+)", line)
            if m:
                self._on_yaw_lock_confirmed(int(m.group(1)) != 0)
        elif "[CAL] ERROR" in line:
            self._on_mag_write_failed(line)

    # ------------------ 3D 繪圖更新 ------------------
    def update_3d_plot(self, q):
        """依據四元數旋轉 3D 火箭模型，繪製地平面與投影輔助線，並更新 HUD 歐拉角與姿態資訊"""
        # 1. 計算旋轉矩陣 R 與歐拉角（直接使用 EKF 輸出的四元數，body→nav ENU）
        # ZYX 分解中「roll」= 繞 body X(右軸) = 火箭 PITCH（前後仰俯）
        #                「pitch」= 繞 body Y(前軸) = 火箭 ROLL（左右翻滾）
        #                「yaw」  = 繞 body Z(上軸) = 航向
        R = quaternion_to_matrix(q)
        zyx_roll, zyx_pitch, yaw_corr = quaternion_to_euler(q)
        pitch_corr = zyx_roll   # 繞 X 軸 = 俯仰
        roll_corr  = zyx_pitch  # 繞 Y 軸 = 翻滾

        # 讀取局部座標點
        (xb1, yb1, zb1, xb2, yb2, zb2, xb3, yb3, zb3,
         xn, yn, zn, xnoz, ynoz, znoz, xfl, yfl, zfl, fins) = self.local_geom

        h_body = 1.8
        h_nose = 0.8
        tail_z, tip_z = -h_body/2 - 0.2, h_body/2 + h_nose

        # 計算 3D 空間中旋轉後的火箭各網格段點
        rxb1, ryb1, rzb1 = rotate_points(xb1, yb1, zb1, R)
        rxb2, ryb2, rzb2 = rotate_points(xb2, yb2, zb2, R)
        rxb3, ryb3, rzb3 = rotate_points(xb3, yb3, zb3, R)
        rxn, ryn, rzn = rotate_points(xn, yn, zn, R)
        rxnoz, rynoz, rznoz = rotate_points(xnoz, ynoz, znoz, R)

        # 引擎火光隨機抖動 (Flicker)
        flicker_factor = random.uniform(0.75, 1.25)

        # 外部大火焰 (橘紅色)
        zfl_outer = zfl * flicker_factor + tail_z
        rxfl, ryfl, rzfl = rotate_points(xfl, yfl, zfl_outer, R)

        # 內部核心小火焰 (黃色)
        zfl_inner = zfl * 0.65 * flicker_factor + tail_z
        rxfl_inner, ryfl_inner, rzfl_inner = rotate_points(xfl * 0.55, yfl * 0.55, zfl_inner, R)

        # 清除前一影格的 3D 繪圖，保留滑鼠拖曳視角
        elev, azim = self.ax.elev, self.ax.azim
        self.ax.clear()
        self.ax.set_facecolor("#101010") # 深邃太空黑背景
        self.ax.view_init(elev=elev, azim=azim)

        # 地平面/同心圈/方位射線/水平環都跟姿態無關，每幀不變，改吃模組層級快取
        # _STATIC_GROUND，不再每幀重算 linspace/meshgrid/cos/sin（5Hz 下省下最大宗的
        # 重複 numpy 運算，減少卡頓）。
        sg = _STATIC_GROUND
        z_ground = sg["z_ground"]

        # 1. 繪製圓形發射台平面 (Launchpad Disk)：暗碳灰色、帶有細緻格線的發射底座面
        self.ax.plot_surface(sg["xg"], sg["yg"], sg["zg"], color="#161616", alpha=0.6, edgecolor='#2c2c2c', linewidth=0.4, shade=False)

        # 繪製發射台霓虹邊緣外圈 (螢光青綠)
        self.ax.plot(sg["cx"], sg["cy"], sg["cz"], color="#00ffcc", linestyle="-", linewidth=1.5, alpha=0.7)

        # 繪製內部同心雷達圈
        for cx_c, cy_c, cz_c in sg["rings"]:
            self.ax.plot(cx_c, cy_c, cz_c, color="#3e3e3e", linestyle="--", linewidth=0.8, alpha=0.5)

        # 繪製方位十字參考射線 (N E S W)
        for lx, ly, lz in sg["compass_lines"]:
            self.ax.plot(lx, ly, lz, color="#333333", linestyle=":", linewidth=0.6)

        # 方位標籤
        self.ax.text(2.2, 0, z_ground, "E (90°)", color="#00ffcc", fontsize=8, fontweight="bold", ha="left", va="center")
        self.ax.text(-2.2, 0, z_ground, "W (270°)", color="#00ffcc", fontsize=8, fontweight="bold", ha="right", va="center")
        self.ax.text(0, 2.2, z_ground, "N (0°)", color="#00ffcc", fontsize=8, fontweight="bold", ha="center", va="bottom")
        self.ax.text(0, -2.2, z_ground, "S (180°)", color="#00ffcc", fontsize=8, fontweight="bold", ha="center", va="top")

        # 2. 繪製水平面參考系 (Z = 0.0 橫切面半透明全息環)
        hx, hy, hz = sg["hx"], sg["hy"], sg["hz"]
        hsx, hsy, hsz = sg["horizon_surf"]
        self.ax.plot_surface(hsx, hsy, hsz, color="#00e5ff", alpha=0.12, shade=False)
        self.ax.plot(hx, hy, hz, color="#00e5ff", linestyle="-", linewidth=0.8, alpha=0.4)
        self.ax.plot([-0.65, 0.65], [0, 0], [0, 0], color="#00e5ff", linestyle=":", linewidth=0.5, alpha=0.4)
        self.ax.plot([0, 0], [-0.65, 0.65], [0, 0], color="#00e5ff", linestyle=":", linewidth=0.5, alpha=0.4)

        # 3. 計算火箭底部與頂部在 3D 空間中的旋轉位置
        p_base = R @ np.array([0, 0, tail_z])
        p_tip = R @ np.array([0, 0, tip_z])

        # 4. 繪製天頂重力參考向量 (Zenith Vector - 螢光綠 arrow 指向正上方)
        self.ax.quiver(0, 0, 0, 0, 0, 1.2, color='#00e676', linewidth=1.5, arrow_length_ratio=0.15, alpha=0.6)
        self.ax.text(0, 0, 1.35, "ZENITH", color="#00e676", fontsize=7, fontweight="bold", ha="center")

        # 5. 繪製火箭鼻錐指向軸心向量 (Heading Vector - 鮮紅 arrow)
        dir_z = R[:, 2]  # 火箭 Z 軸在世界空間的向量
        self.ax.quiver(p_tip[0], p_tip[1], p_tip[2], dir_z[0]*0.5, dir_z[1]*0.5, dir_z[2]*0.5, color='#ff1744', linewidth=2.0, arrow_length_ratio=0.3, alpha=0.9)

        # 6. 繪製火箭對地平面的垂直輔助投影線 (垂足投影 guide)
        self.ax.plot([p_base[0], p_base[0]], [p_base[1], p_base[1]], [p_base[2], z_ground], color="#00e5ff", linestyle="--", linewidth=1.0, alpha=0.6)
        self.ax.plot([p_tip[0], p_tip[0]], [p_tip[1], p_tip[1]], [p_tip[2], z_ground], color="#ff1744", linestyle="--", linewidth=1.0, alpha=0.6)
        self.ax.plot([0, 0], [0, 0], [0, z_ground], color="#ffcc00", linestyle="-.", linewidth=0.8, alpha=0.4)

        # 7. 繪製地平面投影陰影向量 (Quiver Arrow 表示方位偏角)
        self.ax.quiver(p_base[0], p_base[1], z_ground,
                       p_tip[0] - p_base[0], p_tip[1] - p_base[1], 0,
                       color="#00e5ff", alpha=0.35, arrow_length_ratio=0.15, linewidth=2.5)

        # 8. 繪製火箭主體曲面 (利用 plot_surface 本身陰影呈現 3D 質感)：實機塗裝為
        # 鼻錐白、彈體酒紅、尾翼金
        # 引擎噴嘴 (碳黑)
        self.ax.plot_surface(rxnoz, rynoz, rznoz, color="#212121", alpha=0.9, edgecolor='none', shade=True)
        # 下段船身 (酒紅)
        self.ax.plot_surface(rxb1, ryb1, rzb1, color="#722f37", alpha=0.9, edgecolor='none', shade=True)
        # 中段條紋 (酒紅)
        self.ax.plot_surface(rxb2, ryb2, rzb2, color="#722f37", alpha=0.9, edgecolor='none', shade=True)
        # 上段船身 (酒紅)
        self.ax.plot_surface(rxb3, ryb3, rzb3, color="#722f37", alpha=0.9, edgecolor='none', shade=True)
        # 彈頭 (白色)
        self.ax.plot_surface(rxn, ryn, rzn, color="#f5f5f0", alpha=0.95, edgecolor='none', shade=True)

        # 9. 引擎火焰渲染 (雙層漸變噴射)
        self.ax.plot_surface(rxfl, ryfl, rzfl, color="#ff5722", alpha=0.4, edgecolor='none', shade=True)
        self.ax.plot_surface(rxfl_inner, ryfl_inner, rzfl_inner, color="#ffeb3b", alpha=0.7, edgecolor='none', shade=True)

        # 10. 繪製 3D 尾翼 (使用 Poly3DCollection 具備 solid 填充與 shading 特色)
        for f in fins:
            rf = f @ R.T
            poly = Poly3DCollection([rf], facecolors='#d4af37', edgecolors='#8a6f1f', linewidths=1.0, alpha=0.9, shade=True)
            self.ax.add_collection3d(poly)

        # 11. 設定 3D 限制。三軸跨距必須維持接近相等（3.6 / 3.6 / 3.7）：matplotlib 3D 會把
        # 資料範圍拉伸填滿近似立方的顯示框，一旦各軸跨距差太多，模型會被拉扁、傾角也跟著畫錯
        # （實測曾出現 10° 傾角畫成約 40°，這其實是本來就存在的舊 bug，這次一併修正）。
        self.ax.set_xlim([-1.8, 1.8])
        self.ax.set_ylim([-1.8, 1.8])
        self.ax.set_zlim([-1.5, 2.2]) # Z 的上限需要容納傾斜後的火箭鼻錐
        # 顯示框比例必須跟著資料跨距 (3.6, 3.6, 3.7)，否則傾角會被畫錯：
        # matplotlib 3D 預設 box_aspect 是扁的 (約 1.14, 1.14, 0.86)，Z 被壓縮後
        # 傾角會被誇大約 1.36 倍（實測 5° 畫成 6.8°）。設成與跨距同比例後，
        # 實測 5/10/20/30/45° 全部與 HUD 的 TILT 數值完全一致。
        self.ax.set_box_aspect((3.6, 3.6, 3.7))

        # 關閉原生的灰盒背景與格線，呈現極簡懸浮全息圖感
        self.ax.axis('off')

        # 12. 計算與垂直天頂的絕對偏角 (Tilt Angle from Vertical)
        tilt_cos = np.clip(dir_z[2], -1.0, 1.0)
        tilt_deg = np.degrees(np.arccos(tilt_cos))

        # 將 Yaw 顯示角度維持在 0~360 度範圍內
        yaw_disp = yaw_corr % 360.0

        # 13. 繪製 HUD 面板文字 (疊加於 3D 左上角 - 全息透明玻璃面板風)
        hud_text = (
            f"STATE: {self.fsm_state}\n"
            f"TILT:  {tilt_deg:.1f}°\n"
            f"PITCH: {pitch_corr:+.1f}°\n"
            f"YAW:   {yaw_disp:.1f}°\n"
            f"ROLL:  {roll_corr:+.1f}°"
        )
        self.ax.text2D(0.05, 0.95, hud_text, transform=self.ax.transAxes, color="#00e5ff", fontsize=10, fontweight="bold", fontname="Monaco", bbox=dict(facecolor="#0a0a0a", alpha=0.8, edgecolor="#00e5ff", boxstyle="round,pad=0.6", linewidth=1.2))

        # 更新畫布
        self.canvas.draw_idle()

    # ------------------ 視窗生命週期 ------------------
    def on_close(self):
        # 可能被重入觸發（視窗關閉按鈕 + 訊號處理器都可能呼叫到）：
        # 第二次進來時視窗多半已 destroy，直接結束即可，避免下面重複操作報錯。
        if getattr(self, '_closing', False):
            return
        self._closing = True
        self.running = False
        if hasattr(self, 'ser') and self.ser:
            try: self.ser.close()
            except: pass
        if self.log_file:
            try: self.log_file.close()
            except: pass
        self._close_flash_export_file()
        # 關閉測試精靈視窗（若開啟）
        if hasattr(self, 'wizard_win') and self.wizard_win and self.wizard_win.winfo_exists():
            try: self.wizard_win.destroy()
            except: pass
        # 關閉 LoRa 參數面板（若開啟）
        if hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists():
            try: self.lora_win.destroy()
            except: pass
        # quit() 先跳出 mainloop() 的事件迴圈（在 macOS 上 destroy() 之後 mainloop()
        # 有時不會自己返回，行程會卡住而非結束），destroy() 再實際銷毀視窗；
        # os._exit() 是最後保險，強制結束行程，避免任何殘留狀態讓程式掛著不退出。
        try: self.root.quit()
        except Exception: pass
        try: self.root.destroy()
        except Exception: pass
        os._exit(0)

    # 板端命令台是 osPriorityLow 的 ~20ms 輪詢 + 硬體僅 1 byte 緩衝：整串 burst 送會
    # 掉字元（見 mag_calibrate.py 實測，30ms/字仍掉、100ms/字可靠）→ 命令組不起來、
    # 板子毫無回應。故改用背景執行緒逐字慢送，不阻塞 GUI 主迴圈。
    CMD_CHAR_GAP = 0.1   # 秒/字

    def send_command(self, cmd_str):
        if not self.running or not hasattr(self, 'ser') or not self.ser:
            messagebox.showwarning("警告", "串口未連接！請先連線。")
            return False
        if not cmd_str.endswith('\n'):
            cmd_str += '\n'
        # 前置換行：先終止板端命令緩衝 g_cmd_buf 內殘留的半截命令（來自過去 burst 送掉字
        # 的殘骸），讓本命令從乾淨狀態開始組裝，避免「垃圾+命令」黏在一起解析失敗。
        cmd_str = '\n' + cmd_str
        # 逐字慢送（背景執行緒）：避免板端命令台掉字元。
        if not hasattr(self, '_tx_queue'):
            self._tx_queue = queue.Queue()
        if getattr(self, '_tx_thread', None) is None or not self._tx_thread.is_alive():
            self._tx_thread = threading.Thread(target=self._tx_worker, daemon=True)
            self._tx_thread.start()
        self._tx_queue.put(cmd_str)
        # 本地回顯（指令/回應區 + 完整終端）立即顯示；實際位元由背景慢送（數秒）。
        self.console.insert(tk.END, f"[CMD] ➡️ {cmd_str.strip()}（背景慢送中…）\n", "rate")
        self.console.see(tk.END)
        self._event_log(f"[CMD] ➡️ {cmd_str.strip()}", "cmd")
        return True

    def _tx_worker(self):
        """背景逐字慢送佇列中的命令（CMD_CHAR_GAP 秒/字），遷就板端低優先權命令輪詢。"""
        while True:
            cmd = self._tx_queue.get()
            if cmd is None:
                return
            data = cmd.encode('utf-8')
            for i in range(len(data)):
                if not self.running or not getattr(self, 'ser', None):
                    break
                try:
                    self.ser.write(data[i:i + 1])
                    self.ser.flush()
                except Exception:
                    break
                time.sleep(self.CMD_CHAR_GAP)

    # ------------------ 手動指令列 ------------------
    def _on_manual_cmd(self, event=None):
        """送出手動輸入的原始指令；回應會隨遙測串流進終端。"""
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
        """↑：回上一條送過的指令。"""
        if not self._cmd_history:
            return "break"
        self._cmd_history_idx = max(0, self._cmd_history_idx - 1)
        self.cmd_entry.delete(0, tk.END)
        self.cmd_entry.insert(0, self._cmd_history[self._cmd_history_idx])
        return "break"

    def _manual_cmd_history_next(self, event=None):
        """↓：往下一條；到底則清空輸入框。"""
        if not self._cmd_history:
            return "break"
        self._cmd_history_idx = min(len(self._cmd_history), self._cmd_history_idx + 1)
        self.cmd_entry.delete(0, tk.END)
        if self._cmd_history_idx < len(self._cmd_history):
            self.cmd_entry.insert(0, self._cmd_history[self._cmd_history_idx])
        return "break"

    # ==================== BENCH 桌面測試實時監控彈出視窗 ====================
    def open_bench_monitor_window(self):
        """開啟 BENCH 桌面測試實時監控彈出視窗。

        ★時序已改版（與飛行邏輯 1:1 對應）：
          步驟1 引傘 PD13：主板 t=0 起 8s；副板延後 4s（頂點提前量）後 3s —— 兩窗**重疊**。
          步驟3 主傘 PD14：★不啟 PWM，兩板**同時**純 GPIO 拉高 1.5s（互斥握手/舵機掃描已取消）。
        故本視窗不再顯示「1s Guard 意圖確認 / 讓位 / PWM 掃描 / 互斥防打架」那套語意，
        也不再需要舊的 _bench_step1_done 階段閘：新的 arb 值（BENCH_START / BENCH_PRI_FIRE /
        BENCH_SEC_FIRE / BENCH_MAIN_HIGH / DONE）本身就分得出階段，不會像舊版 INTENT 那樣
        在「序列剛開始」與「步驟3 舵機交接」共用同一個列舉值而誤判。
        """
        if hasattr(self, 'bench_win') and self.bench_win and self.bench_win.winfo_exists():
            self.bench_win.lift()
            return

        self.bench_win = tk.Toplevel(self.root)
        self.bench_win.title("🖥️ BENCH TEST REALTIME MONITOR — 桌面開傘雙板協同測試")
        self.bench_win.geometry("860x650")
        self.bench_win.configure(bg="#121212")

        # 頂部標題列
        hdr_frame = tk.Frame(self.bench_win, bg="#27104e", padx=18, pady=12)
        hdr_frame.pack(fill=tk.X)
        tk.Label(hdr_frame, text="🖥️ BENCH TEST REALTIME MONITOR", bg="#27104e", fg="#a855f7",
                 font=("Helvetica", 14, "bold")).pack(anchor="w")
        tk.Label(hdr_frame, text="即時監控：引傘 PD13（主 8s / 副 延後 4s 後 3s，窗重疊）｜ 主傘 PD14（雙板同時拉高 1.5s，無 PWM、無握手）",
                 bg="#27104e", fg="#d8b4fe", font=("Helvetica", 9)).pack(anchor="w", pady=(2, 0))

        main_frame = tk.Frame(self.bench_win, bg="#121212", padx=18, pady=14)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 區塊 1：主副航電與互斥防打架狀態徽章 (3列寬幅獨立卡片，徹底防止文字擠壓)
        badge_frame = tk.LabelFrame(main_frame, text=" 🛡️ 主副航電開傘輸出狀態 (PD13 引傘 / PD14 主傘共開) ",
                                   bg="#18181b", fg="#a855f7", font=("Helvetica", 11, "bold"), padx=12, pady=10)
        badge_frame.pack(fill=tk.X, pady=(0, 12))

        b_row1 = tk.Frame(badge_frame, bg="#18181b")
        b_row1.pack(fill=tk.X, pady=3)
        self.lbl_bench_primary_status = tk.Label(b_row1, text="🚀 PRIMARY BOARD : STANDBY (就緒)", bg="#1e293b", fg="#38bdf8",
                                                 font=("Helvetica", 10, "bold"), anchor="w", padx=10, pady=6)
        self.lbl_bench_primary_status.pack(fill=tk.X)

        b_row2 = tk.Frame(badge_frame, bg="#18181b")
        b_row2.pack(fill=tk.X, pady=3)
        self.lbl_bench_backup_status = tk.Label(b_row2, text="🛟 BACKUP BOARD  : STANDBY (就緒)", bg="#1e293b", fg="#fbbf24",
                                                font=("Helvetica", 10, "bold"), anchor="w", padx=10, pady=6)
        self.lbl_bench_backup_status.pack(fill=tk.X)

        b_row3 = tk.Frame(badge_frame, bg="#18181b")
        b_row3.pack(fill=tk.X, pady=3)
        self.lbl_bench_arb_status = tk.Label(b_row3, text="🪂 主傘共開 (PD14 CO-FIRE): 🟢 IDLE (兩板皆靜置)", bg="#064e3b", fg="#34d399",
                                             font=("Helvetica", 10, "bold"), anchor="w", padx=10, pady=6)
        self.lbl_bench_arb_status.pack(fill=tk.X)

        # 區塊 2：當前步驟與進度條
        prog_frame = tk.LabelFrame(main_frame, text=" 📊 測試進度 (Progress) ", bg="#18181b", fg="#a855f7",
                                    font=("Helvetica", 11, "bold"), padx=12, pady=10)
        prog_frame.pack(fill=tk.X, pady=(0, 12))

        self.lbl_bench_step = tk.Label(prog_frame, text="當前步驟: 就緒 (準備執行測試)", bg="#18181b", fg="#00e5ff",
                                       font=("Helvetica", 10, "bold"))
        self.lbl_bench_step.pack(anchor="w", pady=(0, 6))

        self.bench_progressbar = ttk.Progressbar(prog_frame, orient="horizontal", mode="determinate", maximum=100)
        self.bench_progressbar.pack(fill=tk.X, ipady=4)
        self.bench_progressbar['value'] = 0

        # 區塊 3：實時日誌流 (Live Log Stream)
        log_frame = tk.LabelFrame(main_frame, text=" 📜 實時測試數據串流 (Live Stream: USB / LoRa) ",
                                  bg="#18181b", fg="#a855f7", font=("Helvetica", 11, "bold"), padx=12, pady=10)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.bench_console = tk.Text(log_frame, bg="#080808", fg="#00ff66", font=("Monaco", 9),
                                     insertbackground="white", relief="flat")
        bench_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.bench_console.yview)
        self.bench_console.configure(yscrollcommand=bench_scroll.set)

        bench_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.bench_console.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Console 標籤樣式
        self.bench_console.tag_config("sys", foreground="#00d2ff")
        self.bench_console.tag_config("fire", foreground="#ff5555", font=("Monaco", 9, "bold"))
        self.bench_console.tag_config("guard", foreground="#fde047", font=("Monaco", 9, "bold"))
        self.bench_console.tag_config("servo", foreground="#a855f7", font=("Monaco", 9, "bold"))
        self.bench_console.tag_config("arb", foreground="#38bdf8", font=("Monaco", 9, "bold"))
        self.bench_console.tag_config("ok", foreground="#00e676")

        # 底部操作列
        bot_frame = tk.Frame(self.bench_win, bg="#121212", padx=18, pady=8)
        bot_frame.pack(fill=tk.X, side=tk.BOTTOM)

        def _close_bench_win():
            self._save_bench_log(notify=False)   # 關閉前靜默保存一份，避免忘記存就關掉
            self.bench_win.destroy()

        StyledButton(bot_frame, text="關閉視窗", command=_close_bench_win, bg="#27272a", hover_bg="#3f3f46", padx=14, pady=5).pack(side=tk.RIGHT)
        StyledButton(bot_frame, text="💾 儲存 LOG", command=self._save_bench_log, bg="#1e3a5f", hover_bg="#2c5282", padx=14, pady=5).pack(side=tk.RIGHT, padx=(0, 8))

    def _save_bench_log(self, notify=True):
        """把 BENCH 監控視窗目前的 log 存成檔案（gui/bench_logs/bench_<時間戳>.log），
        方便事後回放或回報問題（例如卡在某個交接步驟時，把完整 log 存下來比對）。"""
        if not hasattr(self, 'bench_console') or not self.bench_console.winfo_exists():
            return
        content = self.bench_console.get("1.0", tk.END)
        if not content.strip():
            if notify:
                messagebox.showinfo("儲存 LOG", "目前 LOG 是空的，沒有內容可存。")
            return
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bench_logs")
        os.makedirs(log_dir, exist_ok=True)
        fname = f"bench_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        fpath = os.path.join(log_dir, fname)
        try:
            with open(fpath, "w", encoding="utf-8") as f:
                f.write(content)
        except OSError as e:
            if notify:
                messagebox.showerror("儲存 LOG 失敗", str(e))
            return
        if notify:
            messagebox.showinfo("儲存 LOG", f"已儲存至：\n{fpath}")

    # BENCH 期間 [LINK] 行的 arb 欄位 → 兩張板子徽章。
    # main_arb 值語意見韌體 servo_arb.h（★互斥握手已取消，2=LEGACY_DRIVING 僅存在於舊紀錄）。
    _BENCH_ARB_BADGE = {
        # arb 名稱: (主板顯示, 副板顯示, bg, fg)
        "BENCH_START":     ("🖥️ BENCH 序列開始 (待命)",       "🖥️ BENCH 序列開始 (待命)",       "#1e293b", "#38bdf8"),
        "BENCH_PRI_FIRE":  ("🔥 PD13 HIGH (引傘通電 8s)",       "🟢 PD13 LOW (等模擬頂點)",        "#7f1d1d", "#ff6b6b"),
        "BENCH_SEC_FIRE":  ("🔥 PD13 HIGH (8s 窗內，已通知副板)", "🔥 PD13 HIGH (引傘通電 3s)",     "#7f1d1d", "#ff6b6b"),
        "BENCH_MAIN_HIGH": ("⚡ PD14 HIGH 1.5s (共開，無 PWM)",  "⚡ PD14 HIGH 1.5s (共開，無 PWM)", "#581c87", "#c084fc"),
        "MAIN_HIGH":       ("⚡ PD14 HIGH 1.5s (飛行共開)",      "⚡ PD14 HIGH 1.5s (飛行共開)",     "#581c87", "#c084fc"),
        "DONE":            ("🟢 DONE (PD14 已回低)",            "🟢 DONE (PD14 已回低)",           "#064e3b", "#34d399"),
    }

    def _bench_apply_link_arb(self, line):
        """解析 [LINK] 行的 self=/self_arb=/peer_arb=，把兩板 arb 狀態各自貼到對應徽章。

        舊版是「整行含 BENCH_PRI_FIRE 就當主板在燒」——self/peer 誰是誰全靠猜；這裡直接
        依 self= 角色分派，主/副接哪一條 USB 線都不會貼錯板。"""
        m_self = re.search(r"self=(\w+)", line)
        m_sarb = re.search(r"self_arb=(\w+)", line)
        m_parb = re.search(r"peer_arb=(\w+)", line)
        if not m_self:
            return
        self_is_primary = (m_self.group(1) == "PRIMARY")
        pairs = []
        if m_sarb:
            pairs.append((self_is_primary, m_sarb.group(1)))
        if m_parb:
            pairs.append((not self_is_primary, m_parb.group(1)))
        for is_primary, arb in pairs:
            badge = self._BENCH_ARB_BADGE.get(arb)
            if not badge:
                continue   # NONE / LEGACY_DRIVING / 未知：不覆蓋既有顯示
            pri_txt, sec_txt, bg, fg = badge
            if is_primary:
                self.lbl_bench_primary_status.config(text=f"🚀 PRIMARY BOARD : {pri_txt}", bg=bg, fg=fg)
            else:
                self.lbl_bench_backup_status.config(text=f"🛟 BACKUP BOARD  : {sec_txt}", bg=bg, fg=fg)
            if arb in ("BENCH_MAIN_HIGH", "MAIN_HIGH"):
                self.lbl_bench_arb_status.config(
                    text="🪂 主傘共開 (PD14 CO-FIRE): 🟣 CO-FIRE 中 (兩板同時拉高＝設計目的)",
                    bg="#581c87", fg="#c084fc")

    def update_bench_monitor(self, line):
        """當收到 [BENCH]、[PYRO-SELFTEST] 或 [LINK] servo_arb 行時更新實時監控小視窗。"""
        if not hasattr(self, 'bench_win') or not self.bench_win or not self.bench_win.winfo_exists():
            if "[BENCH]" in line or "[PYRO-SELFTEST]" in line or "UPLINK_CMD_BENCH" in line:
                self.open_bench_monitor_window()
            else:
                return

        if not hasattr(self, 'bench_console') or not self.bench_console.winfo_exists():
            return

        ts = datetime.now().strftime("%H:%M:%S")
        tag = "sys"
        clean_line = line.strip()

        # ── 日誌分類高亮與步驟進度更新 ──
        # 時序（與飛行 1:1）：
        #   步驟1 引傘 PD13：主板 t=0 起 8s；副板延後 4s 後 3s（兩窗重疊，diode-OR 準位訊號無妨）
        #   步驟3 主傘 PD14：兩板「同時」純 GPIO 拉高 1.5s（★無 PWM、無互斥握手、無讓位 guard）
        if ("開傘電火自測" in line or "序列：" in line or "對應飛行：" in line
                or "LED:" in line or "⚠ PD13" in line or "倒數" in line or "同脈同步" in line):
            tag = "sys"
        elif "[1a]" in line:
            tag = "fire"
            self.lbl_bench_step.config(text="當前步驟: 1a/3 — 🔥 主板引傘 PD13 通電中 (8s，對應飛行提前 4s 開)")
            self.bench_progressbar['value'] = 20
            self.lbl_bench_primary_status.config(text="🚀 PRIMARY BOARD : 🔥 PD13 HIGH (引傘通電 8s)", bg="#7f1d1d", fg="#ff6b6b")
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🟢 PD13 LOW (等模擬頂點，延後 4s)", bg="#064e3b", fg="#34d399")
            self.lbl_bench_arb_status.config(text="🪂 主傘共開 (PD14 CO-FIRE): 🟢 IDLE (步驟1 進行中，PD14 未動作)", bg="#064e3b", fg="#34d399")
        elif "[1b]" in line:
            tag = "guard"
            self.lbl_bench_step.config(text="當前步驟: 1b/3 — ⏳ 副板等待模擬頂點（延後 4s＝飛行提前量）")
            self.bench_progressbar['value'] = 30
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : ⏳ 等待模擬頂點 (PD13 仍 LOW)", bg="#78350f", fg="#fde047")
        elif "[1c]" in line and "已達模擬頂點" in line:
            tag = "fire"
            self.lbl_bench_step.config(text="當前步驟: 1c/3 — 📢 主板已達模擬頂點，通知副板開引傘（主板 PD13 仍 HIGH）")
            self.bench_progressbar['value'] = 40
            self.lbl_bench_primary_status.config(text="🚀 PRIMARY BOARD : 🔥 PD13 HIGH (8s 窗內，已通知副板)", bg="#7f1d1d", fg="#ff6b6b")
        elif "[1c]" in line and "回 LOW" in line:
            tag = "fire"
            self.lbl_bench_step.config(text="當前步驟: 1c/3 — 🟢 副板引傘 3s 到，PD13 回 LOW（主板可能仍在導通）")
            self.bench_progressbar['value'] = 50
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🟢 PD13 LOW (3s 窗結束)", bg="#064e3b", fg="#34d399")
        elif "[1c]" in line:
            tag = "fire"
            self.lbl_bench_step.config(text="當前步驟: 1c/3 — 🔥 副板引傘 PD13 通電中 (3s，對應飛行真頂點才開)")
            self.bench_progressbar['value'] = 45
            self.lbl_bench_primary_status.config(text="🚀 PRIMARY BOARD : 🔥 PD13 HIGH (8s 窗內，與副板重疊)", bg="#7f1d1d", fg="#ff6b6b")
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🔥 PD13 HIGH (引傘通電 3s)", bg="#7f1d1d", fg="#ff6b6b")
            self.lbl_bench_arb_status.config(text="🪂 主傘共開 (PD14 CO-FIRE): 🟢 IDLE (引傘兩窗重疊屬正常設計)", bg="#064e3b", fg="#34d399")
        elif "[1]" in line and "通電測試結束" in line:
            tag = "fire"
            self.lbl_bench_step.config(text="當前步驟: 1/3 完成 — 🟢 兩板引傘 PD13 皆已回 LOW")
            self.bench_progressbar['value'] = 55
            self.lbl_bench_primary_status.config(text="🚀 PRIMARY BOARD : 🟢 PD13 LOW (通電結束)", bg="#064e3b", fg="#34d399")
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🟢 PD13 LOW (通電結束)", bg="#064e3b", fg="#34d399")
        elif "[2]" in line:
            tag = "sys"
            self.lbl_bench_step.config(text="當前步驟: 2/3 — ⏳ 引傘結束，冷卻等待中 (5s)")
            self.bench_progressbar['value'] = 65
            self.lbl_bench_primary_status.config(text="🚀 PRIMARY BOARD : 🟢 冷卻中 (PD13/PD14 皆 LOW)", bg="#064e3b", fg="#34d399")
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🟢 冷卻中 (PD13/PD14 皆 LOW)", bg="#064e3b", fg="#34d399")
        elif "呼叫副板一起開主傘" in line or "等待主板呼叫" in line or "未見主板呼叫" in line:
            tag = "arb"
            self.lbl_bench_step.config(text="當前步驟: 3/3 — 📢 主傘共開呼叫（無握手隔離，兩板同時拉高）")
            self.bench_progressbar['value'] = 78
            self.lbl_bench_arb_status.config(text="🪂 主傘共開 (PD14 CO-FIRE): 🟡 呼叫中 (主板廣播 BENCH_MAIN_HIGH)", bg="#78350f", fg="#fde047")
        elif "[3]" in line and "PD14 拉高" in line:
            tag = "servo"
            is_sec = "副板" in line
            self.lbl_bench_step.config(text="當前步驟: 3/3 — ⚡ 主傘 PD14 純 GPIO 拉高 1.5s（雙板同時，★不啟 PWM）")
            self.bench_progressbar['value'] = 88
            label = ("⚡ PD14 HIGH 1.5s (純 GPIO，無 PWM)")
            if is_sec:
                self.lbl_bench_backup_status.config(text=f"🛟 BACKUP BOARD  : {label}", bg="#581c87", fg="#c084fc")
            else:
                self.lbl_bench_primary_status.config(text=f"🚀 PRIMARY BOARD : {label}", bg="#581c87", fg="#c084fc")
            self.lbl_bench_arb_status.config(text="🪂 主傘共開 (PD14 CO-FIRE): 🟣 CO-FIRE 中 (兩板同時拉高＝設計目的)", bg="#581c87", fg="#c084fc")
        elif "拉高" in line and "完成" in line:
            tag = "servo"
            is_sec = "副板" in line
            self.lbl_bench_step.config(text=f"當前步驟: 3/3 — 🟢 {'副板' if is_sec else '主板'} 主傘拉高完成，PD14 回 LOW")
            self.bench_progressbar['value'] = 94
            if is_sec:
                self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🟢 PD14 LOW (拉高完成)", bg="#064e3b", fg="#34d399")
            else:
                self.lbl_bench_primary_status.config(text="🚀 PRIMARY BOARD : 🟢 PD14 LOW (拉高完成)", bg="#064e3b", fg="#34d399")
        elif "已確認副板同步完成主傘拉高" in line:
            tag = "ok"
            self.lbl_bench_step.config(text="當前步驟: 3/3 — ✅ 已確認雙板同時共開主傘成功")
            self.bench_progressbar['value'] = 98
            self.lbl_bench_arb_status.config(text="🪂 主傘共開 (PD14 CO-FIRE): ✅ 雙板共開已確認 (兩板皆回報 DONE)", bg="#064e3b", fg="#34d399")
        elif "未收到副板 DONE" in line:
            tag = "guard"
            self.lbl_bench_arb_status.config(text="🪂 主傘共開 (PD14 CO-FIRE): ⚠ 未收到副板 DONE (單板 bench 或鏈路異常)", bg="#78350f", fg="#fde047")
        elif "序列完成" in line or "BENCH OK" in line:
            tag = "ok"
            self.lbl_bench_step.config(text="當前步驟: ✅ 桌面測試完成 (BENCH COMPLETE)")
            self.bench_progressbar['value'] = 100
            self.lbl_bench_primary_status.config(text="🚀 PRIMARY BOARD : 🟢 STANDBY (復位完成)", bg="#1e293b", fg="#38bdf8")
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🟢 STANDBY (復位完成)", bg="#1e293b", fg="#fbbf24")
            self.lbl_bench_arb_status.config(text="🪂 主傘共開 (PD14 CO-FIRE): 🟢 IDLE (兩板皆已回低，回歸正常 FSM)", bg="#064e3b", fg="#34d399")
        elif "[LINK]" in line and ("self_arb" in line or "peer_arb" in line):
            tag = "arb"
            self._bench_apply_link_arb(line)

        self.bench_console.insert(tk.END, f"[{ts}] {clean_line}\n", tag)
        self.bench_console.see(tk.END)

    # ==================== 手動開傘（DEPLOY_DROGUE/MAIN/BOTH） ====================
    _DEPLOY_INFO = {
        "drogue": ("副傘 DROGUE", "deploy drogue"),
        "main":   ("主傘 MAIN",   "deploy main"),
        "both":   ("副傘+主傘 BOTH", "deploy both"),
    }
    _DEPLOY_LABEL_ZH = {
        "DEPLOY-DROGUE": "副傘 DROGUE",
        "DEPLOY-MAIN":   "主傘 MAIN",
        "DEPLOY-BOTH":   "副傘+主傘 BOTH",
    }

    def _on_deploy(self, kind):
        """觸發手動開傘（經地面站 433 上行 UPLINK_CMD_DEPLOY_*）。是否接受由航電板判斷
        （須先 ARM，或已在飛行中）；GUI 不做本地攔截，一律送出，接受/拒絕以航電回傳的
        [ACK] 為準（見 _handle_deploy_ack）。"""
        zh, cmd_text = self._DEPLOY_INFO[kind]
        st = self.fsm_state
        ans = messagebox.askyesno(
            f"⚠️ 手動開傘確認（{zh}）",
            f"確定要立即送出『手動開傘 - {zh}』指令嗎？\n\n"
            "⚠️ 注意：這會直接導通點火/舵機機構，若已裝藥將實際點燃！\n"
            f"當前航電狀態：{st or '未知'}\n\n"
            "本按鈕只在收到航電回傳 [ACK] 確認後才會顯示「已確認開傘」。\n"
            "是否立即執行？"
        )
        if not ans:
            return
        self.lbl_deploy_status.config(text=f"⏳ 已送出「{zh}」，等待航電 ACK…", bg="#3b2d00", fg="#ffcc00")
        self.send_command(cmd_text)

    def _handle_deploy_ack(self, ts, line):
        """比對 [UPLINK] 送出時記下的 seq，分辨這筆 [ACK] cmd:"deploy" 對應副傘/主傘/雙傘。"""
        m = re.search(r"seq:(\d+)", line)
        seq = int(m.group(1)) if m else None
        raw_label = self._uplink_seq_label.pop(seq, None) if seq is not None else None
        zh = self._DEPLOY_LABEL_ZH.get(raw_label, "開傘")
        if "status:OK" in line:
            self.lbl_deploy_status.config(text=f"✅ 航電已確認開傘：{zh}", bg="#064e3b", fg="#34d399")
            self._event_log(f"[{ts}] 🪂 [ACK] 航電確認開傘動作已執行：{zh}", "ok")
            # 手動開傘（UplinkCmd_TakeDeploy）直接驅動硬體，不經過 FSM 狀態轉移
            # （main.c:2146-2168），單靠 _mark_deploy 的 FSM order 判斷會完全漏掉
            # 這條路徑（ARM 後手動開傘顯示不會更新）——這裡用 ACK 確認直接補上。
            if raw_label in ("DEPLOY-DROGUE", "DEPLOY-BOTH"):
                self._mark_deploy_flag("self", "drogue")
            if raw_label in ("DEPLOY-MAIN", "DEPLOY-BOTH"):
                self._mark_deploy_flag("self", "main")
        else:
            self.lbl_deploy_status.config(text=f"❌ 開傘遭拒：{zh}", bg="#3b1a1a", fg="#ff4d4d")
            self._event_log(f"[{ts}] ❌ [ACK] 開傘指令被拒（{zh}）：{line.strip()}", "err")

    # ==================== 落海回收確認（RECOVERY：停蜂鳴器 + 安全關閉 SD/Flash） ====================
    def _on_recovery(self):
        """觸發落海回收確認（經地面站 433 上行 UPLINK_CMD_RECOVERY）。是否接受由航電板判斷
        （僅 STATE_LANDED 才會執行，見 uplink_cmd.c/main.c）；GUI 不做本地攔截，一律送出，
        接受/拒絕以航電回傳的 [ACK] 為準（見 _handle_recovery_ack）。"""
        st = self.fsm_state
        ans = messagebox.askyesno(
            "落海回收確認",
            "確定要送出『尋回指令 (RECOVERY)』嗎？\n\n"
            "⚠️ 這會停止板載尋標蜂鳴器，並安全關閉 SD/Flash 數據記錄（無法復原）。\n"
            f"當前航電狀態：{st or '未知'}\n\n"
            "本按鈕只在收到航電回傳 [ACK] 確認後才會顯示「已確認」。\n"
            "確定已完成落點回收、要停止蜂鳴器與記錄嗎？"
        )
        if not ans:
            return
        self.lbl_recovery_status.config(text="⏳ 已送出，等待航電 ACK…", bg="#3b2d00", fg="#ffcc00")
        self.send_command("recovery")

    def _handle_recovery_ack(self, ts, line):
        if "status:OK" in line:
            self.lbl_recovery_status.config(text="✅ 已確認：蜂鳴器已停止／SD·Flash 記錄已安全關閉",
                                            bg="#064e3b", fg="#34d399")
            self._event_log(f"[{ts}] 🔇 [ACK] 航電確認尋回指令已執行（蜂鳴器/SD/Flash 已停止）", "ok")
        else:
            self.lbl_recovery_status.config(text="❌ 尋回指令遭拒（可能仍在飛行中）", bg="#3b1a1a", fg="#ff4d4d")
            self._event_log(f"[{ts}] ❌ [ACK] 尋回指令被拒：{line.strip()}", "err")

    def _on_bench_test(self):
        """觸發手動桌面開傘測試 (BENCH)。是否接受由航電板判斷（須先 ARM）；GUI 不做
        本地攔截，一律送出，接受/拒絕以航電回傳的 [ACK] 為準（見 update_bench_monitor）。"""
        ans = messagebox.askyesno(
            "手動桌面測試確認 (BENCH)",
            "確定要發送『桌面開傘測試 (BENCH)』指令嗎？\n\n"
            "⚠️ 注意（時序與飛行邏輯 1:1 對應）：\n"
            "1. 引傘 PD13：主板 t=0 起通電 8s；副板延後 4s（模擬頂點提前量）後通電 3s，\n"
            "   兩板通電窗會重疊（PD13 為 diode-OR 準位訊號，同時拉高無妨）。\n"
            "2. 主傘 PD14：★不啟動 PWM，兩板『同時』純 GPIO 拉高 1.5s（已取消互斥握手）。\n"
            "3. 全程耗時約 20 秒，測試完成後自動復位 PD14 並回歸正常 FSM。\n"
            "4. 航電必須處於解鎖狀態 (STATE_PAD_ARMED)。\n\n"
            "是否立即執行？"
        )
        if ans:
            self.open_bench_monitor_window()
            self.send_command("bench")

    # ==================== 角色自動偵測（主航電/備援航電/地面站） ====================
    ROLE_STYLES = {
        "PRIMARY": ("🚀 主航電 PRIMARY", "#00e676"),
        "BACKUP":  ("🛟 備援航電 BACKUP", "#ffcc00"),
        "GROUND":  ("📡 地面站 GROUND", "#e07bfb"),
        None:      ("⚫ 角色偵測中…", "#777777"),
    }

    def set_role(self, role, fw="", gs_tx=None):
        """更新偵測到的角色徽章；role=None 表示未知/重置。
        gs_tx：僅 GROUND 角色有意義（None=未知/舊版韌體, 0=RX-only, 1=TX-capable）——
        單純顯示用的徽章區分，兩種地面站韌體是否接受發射一律由韌體本身判斷回應，
        GUI 不因此禁用任何按鈕（ARM/BENCH/DEPLOY 等一律照送，見 _on_bench_test 等）。"""
        if fw:
            self.detected_fw = fw
        if role == "GROUND" and gs_tx is not None:
            self.detected_gs_tx = gs_tx
        elif role != "GROUND":
            self.detected_gs_tx = None
        changed = (role != self.detected_role)
        self.detected_role = role
        text, color = self.ROLE_STYLES.get(role, self.ROLE_STYLES[None])
        if role is None:
            self.detected_fw = ""
            self.detected_gs_tx = None
            if not self.running:
                text = "⚫ 未連線"
        else:
            if role == "GROUND":
                if self.detected_gs_tx == 1:
                    text += "  📡TX"
                    color = "#ff5555"
                elif self.detected_gs_tx == 0:
                    text += "  🔒RX-ONLY"
            if self.detected_fw:
                text += f"  ({self.detected_fw})"
        self.lbl_role.config(text=text, fg=color)
        if changed and role:
            self.console.insert(tk.END, f"[SYSTEM] ✅ 偵測到裝置角色: {text}\n", "ok")
            self.console.see(tk.END)
        if role is None:
            self.lbl_link.config(text="🔗 --", fg="#555555")
            self.lbl_erase.config(text="💾 ERASE: --", fg="#555555")
            self.link_last_age_ms = None
        # 同步 LoRa 面板（若已開啟）
        if hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists():
            self.refresh_lora_panel_role()

    # FSM 階段色帶設定：狀態 → (背景色, 字色, 短標籤)
    _FSM_PHASE = {
        "STATE_INIT":       ("#1a2e1a", "#00e676", "INIT"),
        "STATE_PAD":        ("#1a2e1a", "#00e676", "PAD"),
        "STATE_PAD_ARMED":  ("#3b1a1a", "#ff4d4d", "ARMED"),
        "STATE_BOOST":      ("#3b2d00", "#ffcc00", "BOOST"),
        "STATE_COAST":      ("#2d2d00", "#ffe566", "COAST"),
        "STATE_DROGUE":     ("#2e1a4a", "#a855f7", "DROGUE"),
        "STATE_APOGEE":     ("#1a1a4a", "#818cf8", "APOGEE"),
        "STATE_DESCENT":    ("#2e1a4a", "#c084fc", "DESCENT"),
        "STATE_MAIN":       ("#1a1a4a", "#818cf8", "MAIN"),
        "STATE_LANDED":     ("#0a2030", "#00e5ff", "LANDED"),
    }

    def update_fsm_state_ui(self, new_state):
        """動態更新頂部狀態列文字、顏色與 ARM/DISARM 安全指示燈"""
        # 記錄 FSM 轉換時間點（首次呼叫或狀態改變時）
        if not self.fsm_events or self.fsm_events[-1][1] != new_state:
            self.fsm_events.append((self.now_t(), new_state))
            self.chart_dirty = True

        self.fsm_state = new_state
        if not hasattr(self, 'lbl_fsm') or not self.lbl_fsm.winfo_exists():
            return

        if new_state in ("STATE_PAD", "STATE_INIT"):
            self.lbl_fsm.config(text=f"🚀 STATE: {new_state}", fg="#00e676")
            if hasattr(self, 'lbl_arm_status') and self.lbl_arm_status.winfo_exists():
                self.lbl_arm_status.config(text="🛡️ DISARMED (安全)", bg="#1b4332", fg="#2ec4b6")
        elif new_state == "STATE_PAD_ARMED":
            self.lbl_fsm.config(text=f"⚡ STATE: PAD_ARMED (解鎖 待發射!)", fg="#ff3b30")
            if hasattr(self, 'lbl_arm_status') and self.lbl_arm_status.winfo_exists():
                self.lbl_arm_status.config(text="⚡ ARMED (解鎖/待發射!)", bg="#7f1d1d", fg="#ff4d4d")
        elif new_state in ("STATE_BOOST", "STATE_COAST"):
            self.lbl_fsm.config(text=f"🔥 STATE: {new_state} (飛行中)", fg="#ffcc00")
            if hasattr(self, 'lbl_arm_status') and self.lbl_arm_status.winfo_exists():
                self.lbl_arm_status.config(text="🚀 IN FLIGHT (飛行中)", bg="#b45309", fg="#fef08a")
        elif "DROGUE" in new_state or "MAIN" in new_state or "RECOVERY" in new_state:
            self.lbl_fsm.config(text=f"🪂 STATE: {new_state} (降落中)", fg="#a855f7")
            if hasattr(self, 'lbl_arm_status') and self.lbl_arm_status.winfo_exists():
                self.lbl_arm_status.config(text="🪂 RECOVERY (降落中)", bg="#581c87", fg="#e9d5ff")
        elif new_state == "STATE_LANDED":
            self.lbl_fsm.config(text=f"🏁 STATE: LANDED (已著陸)", fg="#00e5ff")
            if hasattr(self, 'lbl_arm_status') and self.lbl_arm_status.winfo_exists():
                self.lbl_arm_status.config(text="🏁 LANDED (著陸)", bg="#0284c7", fg="#e0f2fe")
        else:
            self.lbl_fsm.config(text=f"🚀 STATE: {new_state}", fg="#00e5ff")

        order = self._SELF_STATE_ORDER.get(new_state)
        if order is not None:
            self._mark_deploy("self", order)

    # 本板 update_fsm_state_ui() 收到的是內部全名（STATE_DROGUE/STATE_MAIN 等短化名，
    # 見上方 _SHORTNAME_MAP）；對端 update_link_status() 收到的是韌體 link_fsm_state_name()
    # 原始短名（main.c:2288: DEP_DROGUE/MAIN_DEPLOY）。兩份對照表分開放，避免互相牽動。
    _SELF_STATE_ORDER = {
        "STATE_INIT": 0, "STATE_PAD": 1, "STATE_PAD_ARMED": 2, "STATE_BOOST": 3,
        "STATE_COAST": 4, "STATE_DROGUE": 5, "STATE_APOGEE": 6, "STATE_DESCENT": 7,
        "STATE_MAIN": 8, "STATE_LANDED": 9,
    }
    _PEER_STATE_ORDER = {
        "INIT": 0, "PAD": 1, "PAD_ARMED": 2, "BOOST": 3, "COAST": 4,
        "DEP_DROGUE": 5, "APOGEE": 6, "DESCENT": 7, "MAIN_DEPLOY": 8, "LANDED": 9,
    }

    def _mark_deploy_flag(self, prefix: str, which: str):
        """單一旗標直接 latch，不做任何隱含推論（給 ACK 確認 / flags bit 這類「各自
        獨立的硬體訊號」用——副傘 PD13 與主傘 PD14/TIM4 是兩個獨立輸出，觸發其一不
        代表另一也觸發過）。"""
        key = f"{prefix}_{which}"
        if not self.deploy_latch[key]:
            self.deploy_latch[key] = True
            self._refresh_deploy_label()

    def _mark_deploy(self, prefix: str, order: int):
        """prefix: 'self' 或 'peer'。專給「FSM 狀態順位」訊號用：FSM 狀態只會單向前進
        （見 fsm.c 全部 switch-case 沒有任何路徑轉回較早狀態），故 order>=8(MAIN_DEPLOY)
        必然蘊含 order>=5(DEPLOY_DROGUE) 也已發生過，這個蘊含關係只在狀態機語意下成立。
        ACK 確認 / flags bit 是各自獨立的硬體訊號，不能共用這裡，改呼叫 _mark_deploy_flag。"""
        if order >= 5:
            self._mark_deploy_flag(prefix, "drogue")
        if order >= 8:
            self._mark_deploy_flag(prefix, "main")

    def _refresh_deploy_label(self):
        if not hasattr(self, 'lbl_deploy_latch') or not self.lbl_deploy_latch.winfo_exists():
            return
        d = self.deploy_latch
        mark = lambda v: "✓" if v else "○"
        text = (f"🪂 開傘紀錄  本板[副:{mark(d['self_drogue'])} 主:{mark(d['self_main'])}]"
                f"  對端[副:{mark(d['peer_drogue'])} 主:{mark(d['peer_main'])}]")
        color = "#ff8080" if any(d.values()) else "#888888"
        self.lbl_deploy_latch.config(text=text, fg=color)

    _LINK_PEER_LABEL = {
        "BACKUP": "BACKUP",
        "PRIMARY": "PRIMARY",
        "NONE": "PEER",
    }
    # 韌體 ServoArb_MsgName() 回傳值 → 頂部鏈路列的短標籤。
    # ★主傘已改為「兩板同時把 PD14 拉高 1.5s」，無互斥握手；LEGACY_DRIVING 僅出現在舊紀錄。
    _ARB_SHORT = {
        "NONE": "IDLE",
        "BENCH_START": "BENCH",
        "BENCH_PRI_FIRE": "P-FIRE",
        "BENCH_SEC_FIRE": "S-FIRE",
        "BENCH_MAIN_HIGH": "CO-HI",
        "MAIN_HIGH": "HIGH",
        "DONE": "DONE",
        "LEGACY_DRIVING": "OLD-PWM",
    }

    def update_link_status(self, self_role, peer_role, link_ok, peer_state, flags, age_ms,
                           sync=None, lost=None, desync=None, self_arb=None, peer_arb=None):
        """更新頂部鏈路與主傘 PD14 共開狀態 (1Hz)。"""
        self.link_last_age_ms = age_ms

        p_order = self._PEER_STATE_ORDER.get(peer_state)
        if p_order is not None:
            self._mark_deploy("peer", p_order)
        # 對端手動開傘（其自身 UplinkCmd_TakeDeploy）同樣不會移動對端 FSM 狀態，
        # 上面的 p_order 判斷會漏掉；但手動/自動都是動同一組硬體(PD13/PD14)，
        # peer->flags 的即時位元讀得到，這裡補上（同樣經 _mark_deploy latch，
        # 不受副傘導通窗結束後 flags 歸零影響）。TELEM_FLAG_DROGUE_FIRED=0x01,
        # TELEM_FLAG_MAIN_DEPLOYED=0x02（telemetry.h）。
        if flags & 0x01:
            self._mark_deploy_flag("peer", "drogue")
        if flags & 0x02:
            self._mark_deploy_flag("peer", "main")
        if link_ok == "OK":
            text, color = f"🔗 LINK: OK ({age_ms}ms)", "#00e676"
        elif link_ok == "STALE":
            text, color = f"🔗 LINK: STALE ({age_ms}ms)", "#ffcc00"
        else:
            text, color = "🔗 LINK: OFF", "#ff3366"

        # echo-ACK 失同步/丟包警示
        if desync == "1" or lost == "1":
            warn = ("DESYNC " if desync == "1" else "") + ("LOST " if lost == "1" else "")
            text += f" ⚠{warn.strip()}"
            if color == "#00e676":
                color = "#ffcc00"

        # 主傘 PD14 共開狀態（★互斥握手已取消：兩板同時 HIGH 才是正常，不再是異常）
        sa = self._ARB_SHORT.get(self_arb or "", "")
        pa = self._ARB_SHORT.get(peer_arb or "", "")
        if sa or pa:
            s_tag = "PRI" if self_role == "PRIMARY" else ("BAC" if self_role == "BACKUP" else "SELF")
            p_tag = "BAC" if peer_role == "BACKUP" else ("PRI" if peer_role == "PRIMARY" else "PEER")
            text += f" | 🪂 ARB[{s_tag}:{sa or '—'} {p_tag}:{pa or '—'}]"

        self.lbl_link.config(text=text, fg=color)

        if hasattr(self, 'lbl_peer') and self.lbl_peer.winfo_exists():
            if link_ok == "OK":
                # 對端已解鎖 (PAD_ARMED) 比照本板 update_fsm_state_ui 的紅字警示——
                # 對端已武裝代表其點火/舵機機構也已待發，不該只有本板 ARMED 才紅字。
                if peer_state == "PAD_ARMED":
                    peer_text, peer_color = f"⚡ PEER: {peer_state} ({age_ms}ms)", "#ff3b30"
                else:
                    peer_text, peer_color = f"🛸 PEER: {peer_state} ({age_ms}ms)", "#00e676"
            elif link_ok == "STALE":
                peer_text, peer_color = f"🛸 PEER: LOST ({age_ms}ms)", "#ffcc00"
            else:
                peer_text, peer_color = "🛸 PEER: NO LINK", "#ff3366"
            self.lbl_peer.config(text=peer_text, fg=peer_color)

    def _update_erase_progress(self, pri_pct, bak_pct, cur_sec=None, tot_sec=None):
        """更新頂部雙航電 Flash 預擦除進度標籤。pri_pct/bak_pct 為絕對角色百分比
        （firmware 端已算好 primary=/backup=，這裡不再需要 self/peer 相對映射）。"""
        if pri_pct >= 100 and bak_pct >= 100:
            text = "💾 ✅ 雙板擦除完成 (Ready to ARM)"
            color = "#00e676"
        else:
            sec_info = ""
            if cur_sec and tot_sec:
                sec_info = f" [{cur_sec}/{tot_sec}]"
            text = f"💾 PRI:{pri_pct}% BAK:{bak_pct}%{sec_info}"
            color = "#ffcc00" if (pri_pct > 0 or bak_pct > 0) else "#ff3366"
        self.lbl_erase.config(text=text, fg=color)

    # 對稱獨立冗餘下副板無自身電台；主板把副板摘要中繼進下鏈，地面經 [GS_PKT] peer: 看到。
    _PEER_FSM_NAMES = ["INIT", "PAD", "PAD_ARMED", "BOOST", "COAST", "DROGUE",
                       "APOGEE", "DESCENT", "MAIN", "LANDED"]

    def _apply_peer_bench_arb(self, pbarb):
        """依主板下鏈中繼的副板 main_arb 值（韌體 servo_arb.h 的 SERVO_ARB_MSG_*：
        1=BENCH_START 3=DONE 4=BENCH_PRI_FIRE 5=BENCH_SEC_FIRE 6=BENCH_MAIN_HIGH
        7=MAIN_HIGH）更新 BENCH 監控面板的副板徽章。
        走二進位遙測(LoRa 433/920 → 地面站 → USB [GS_PKT])，與 update_bench_monitor() 的
        文字行解析互補，GUI 不需直接接航電板 USB 也能看到副板進度。
        ★2 = LEGACY_DRIVING（舊 PWM 掃描）僅存在於改版前的紀錄，這裡刻意不再處理。"""
        if not hasattr(self, 'bench_win') or not self.bench_win or not self.bench_win.winfo_exists():
            return
        if not hasattr(self, 'lbl_bench_backup_status'):
            return
        if pbarb in (6, 7):   # BENCH_MAIN_HIGH / MAIN_HIGH：副板 PD14 拉高 1.5s（與主板共開）
            self.lbl_bench_step.config(text="當前步驟: 3/3 — ⚡ 副板 PD14 拉高 1.5s（與主板同時共開，無 PWM）")
            self.bench_progressbar['value'] = 88
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : ⚡ PD14 HIGH 1.5s (共開，無 PWM)", bg="#581c87", fg="#c084fc")
            self.lbl_bench_arb_status.config(text="🪂 主傘共開 (PD14 CO-FIRE): 🟣 CO-FIRE 中 (兩板同時拉高＝設計目的)", bg="#581c87", fg="#c084fc")
        elif pbarb == 5:    # BENCH_SEC_FIRE：副板引傘 PD13 通電 3s（主板 8s 窗內，兩窗重疊）
            self.lbl_bench_step.config(text="當前步驟: 1c/3 — 🔥 副板引傘 PD13 通電中 (3s，真頂點才開)")
            self.bench_progressbar['value'] = 45
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🔥 PD13 HIGH (引傘通電 3s)", bg="#7f1d1d", fg="#ff6b6b")
        elif pbarb == 4:    # BENCH_PRI_FIRE：主板引傘中，副板應仍 LOW（等模擬頂點）
            self.lbl_bench_step.config(text="當前步驟: 1a/3 — 🔥 主板引傘 PD13 通電中 (8s)")
            self.bench_progressbar['value'] = 20
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🟢 PD13 LOW (等模擬頂點，延後 4s)", bg="#064e3b", fg="#34d399")
        elif pbarb == 3:    # DONE：副板 PD14 拉高窗結束、已回低
            self.lbl_bench_step.config(text="當前步驟: 3/3 — 🟢 副板主傘拉高完成，PD14 已回低")
            self.bench_progressbar['value'] = 96
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🟢 DONE (PD14 已回低)", bg="#064e3b", fg="#34d399")
        elif pbarb == 1:    # BENCH_START：副板已收到序列開始宣告，待命
            self.lbl_bench_step.config(text="當前步驟: 0/3 — 🖥️ 副板已進入 BENCH 序列，待命中")
            self.bench_progressbar['value'] = 8
            self.lbl_bench_backup_status.config(text="🛟 BACKUP BOARD  : 🖥️ BENCH 序列開始 (待命)", bg="#1e293b", fg="#fbbf24")

    def update_peer_relay(self, pfsm, pflags, plink,
                          pbaro_m=0.0, pvfh_m=0.0, pvfv_ms=0.0, pbarb=0):
        """主板經下鏈中繼的副板摘要（Phase A/D：地面雙板監看）。同步餵入 live chart 的
        副航電 VF/baro 序列與 FSM 狀態轉換記號，讓主副航電狀態與估計器差異能同屏比對。
        ★2026-07-30 起下鏈只帶 VF + baro + 鏈路健康位；USB 直連時走 [LINK] 那條路徑
        仍有對端 EKF/高G 全量。pbarb：副板 BENCH/主傘共開狀態（見 _apply_peer_bench_arb）。"""
        self._apply_peer_bench_arb(pbarb)
        # 副板開傘 latch：韌體自 2026-07-31 起把 peer_flags 的 bit0/bit1 改成「開過」鎖存值
        # （Telemetry_Build 由 LinkPeer 的 drogue_latched/main_latched OR 進來），開過之後
        # 每一包都帶著。舊版這裡完全沒讀 pflags —— 走 LoRa 下鏈時副板開傘在本 GUI 是看不到的
        # （只有 USB 直連吃 [LINK] 那條 update_link_status 路徑才 latch），這裡補上。
        # TELEM_FLAG_DROGUE_FIRED=0x01, TELEM_FLAG_MAIN_DEPLOYED=0x02（telemetry.h）。
        if pflags & 0x01:
            self._mark_deploy_flag("peer", "drogue")
        if pflags & 0x02:
            self._mark_deploy_flag("peer", "main")
        name = self._PEER_FSM_NAMES[pfsm] if pfsm < len(self._PEER_FSM_NAMES) else f"?{pfsm}"
        ever, fresh = bool(plink & 0x01), bool(plink & 0x02)
        lost, desync = bool(plink & 0x04), bool(plink & 0x08)
        if not ever:
            text, color = "🛸 PEER: NO LINK", "#ff3366"
        elif fresh:
            warn = ("⚠LOST " if lost else "") + ("⚠DESYNC " if desync else "")
            text = f"🛸 PEER: {name} (VF h={pvfh_m:.0f}m, v={pvfv_ms:+.0f}m/s) {warn}".rstrip()
            color = "#ffcc00" if (lost or desync) else "#00e676"
        else:
            text, color = "🛸 PEER: LOST", "#ffcc00"

        if fresh:
            t = self.now_t()
            # 副板 baro 是絕對海拔，借主板 pad_ref 扣（見 ln_alt_backup_baro 註解）
            pbaro_rel = self.pad_ref.rel(pbaro_m)
            if pbaro_rel is not None:
                self.ts_backup_baro.append((t, pbaro_rel))
            self.ts_backup_vf_alt.append((t, pvfh_m))
            self.ts_backup_vf_vz.append((t, pvfv_ms))
            self.backup_state = name
            state_full = f"STATE_{name}"
            if not self.backup_fsm_events or self.backup_fsm_events[-1][1] != state_full:
                self.backup_fsm_events.append((t, state_full))
            self.chart_dirty = True

        if hasattr(self, 'lbl_peer'):
            self.lbl_peer.config(text=text, fg=color)

    def update_lora_hw_status(self, is_433_ok, is_920_ok):
        if is_433_ok:
            self.lbl_lora433.config(text="📻 433: READY", fg="#00e676")
        else:
            self.lbl_lora433.config(text="📻 433: OFF", fg="#ff3366")
            
        if is_920_ok:
            self.lbl_lora920.config(text="📡 920: READY", fg="#00e676")
        else:
            self.lbl_lora920.config(text="📡 920: OFF", fg="#ff3366")

    def reset_packet_counters(self):
        self.gs_pkt_cnt_433 = 0
        self.gs_pkt_cnt_920 = 0
        self.gs_pkt_cnt_total = 0
        if hasattr(self, 'lbl_gs_pkts'):
            self.lbl_gs_pkts.config(text="📦 接收包數: 0 (433:0 | 920:0)")
        if hasattr(self, 'lbl_lora433'):
            self.lbl_lora433.config(text="📻 433: --", fg="#555555")
        if hasattr(self, 'lbl_lora920'):
            self.lbl_lora920.config(text="📡 920: --", fg="#555555")

    def query_role(self):
        """主動送 'role' 命令查詢角色（三種角色 firmware 都會回 [ROLE_ID]）。
        最多重試 5 次；期間若被動解析（[BOOT]/[GS_*] 等）已判定則停止。
        ★注意：這支只負責「角色」本身。GROUND 板的 tx=0/1（RX-only/TX-capable）
        另外由 query_gs_tx() 負責——地面站板若在 GUI 連線前就已開機、持續狂印
        [GS_PKT]/[GS_STAT] 等遙測行，這裡的被動偵測會在 1.5s 之內就把 detected_role
        搶先設成 GROUND，導致本函式提早 return、永遠不會真的送出 'role\\n'；
        但 tx= 只有主動送 'role' 才問得到，所以不能共用同一個「角色已知即停」的
        判斷式，否則 TX 徽章永遠停在「偵測中」。"""
        if not self.running or self.detected_role is not None:
            return
        if self.role_query_attempts >= 5:
            self.console.insert(tk.END, "[SYSTEM] ⚠️ 角色查詢無回應（舊版 firmware 無 'role' 命令？"
                                        "仍可由 [BOOT]/遙測特徵行被動判定）\n", "err")
            self.console.see(tk.END)
            return
        self.role_query_attempts += 1
        try:
            self.ser.write(b"role\n")
            self.ser.flush()
        except Exception:
            return
        self.root.after(2500, self.query_role)

    def query_gs_tx(self):
        """獨立於 query_role 之外，主動查 GROUND 板的 tx=0/1（RX-only/TX-capable）。
        見 query_role 註解：地面站板常在連線前就已開機並持續狂印遙測行，被動偵測
        會搶先判定 detected_role=GROUND，讓 query_role 提早停手——但那不會送出
        'role\\n'，firmware 也就没機會回 tx= 欄位。這裡改用「detected_gs_tx 是否已
        知」當停止條件，不管角色是被動還是主動判定的都會持續問，直到問到為止。
        純顯示用（見 set_role）：連線的是哪種地面站韌體都一律照送指令，不因此
        禁用任何按鈕。"""
        if not self.running:
            return
        if self.detected_role not in (None, "GROUND"):
            return   # 已知是 PRIMARY/BACKUP，tx 欄位不適用
        if self.detected_gs_tx is not None:
            return   # 已經拿到 tx 資訊
        if self.gs_tx_query_attempts >= 5:
            return
        self.gs_tx_query_attempts += 1
        try:
            self.ser.write(b"role\n")
            self.ser.flush()
        except Exception:
            return
        self.root.after(1500, self.query_gs_tx)

    def parse_role_and_lora(self, line):
        """解析角色特徵行與 LoRa 參數回報行（三角色輸出格式已於 firmware 端統一）。"""
        # --- 角色偵測（未判定時全面掃描；[ROLE_ID]/[BOOT] 永遠重判，支援重刷角色後熱插拔） ---
        if self.detected_role is None or "[ROLE_ID]" in line or "[BOOT]" in line:
            m = re.search(r"\[ROLE_ID\]\s+role=(PRIMARY|BACKUP|GROUND)(?:\s+fw=(\S+))?(?:\s+tx=(\d))?", line)
            if m:
                gs_tx = int(m.group(3)) if m.group(3) is not None else None
                self.set_role(m.group(1), m.group(2) or "", gs_tx=gs_tx)
            else:
                m = re.search(r"\[BOOT\].*ROLE=(PRIMARY|BACKUP|GROUND)", line)
                if m:
                    mf = re.search(r"FW=(\S+)", line)
                    self.set_role(m.group(1), mf.group(1) if mf else "")
                elif re.search(r"\[ROLE\]\s+PRIMARY_AV", line):
                    self.set_role("PRIMARY")
                elif re.search(r"\[ROLE\]\s+BACKUP_AV", line):
                    self.set_role("BACKUP")
                elif ("LoRa 通訊測試模組就緒" in line or line.startswith("[GS_")
                      or "[GS_PKT]" in line or "[GS_STAT]" in line or "[GS_LORA_INIT]" in line):
                    self.set_role("GROUND")

        # --- E80 920MHz 參數回報（gs / 航電命令台同格式） ---
        m = re.search(r"\[E80\] freq=(\d+) Hz\s+SF(\d+)\s+BW([\d\.]+) kHz \(idx=(\d+)\)\s+"
                      r"CR 4/(\d)\s+pwr=(-?\d+) dBm\s+pre=(\d+)", line)
        if m:
            self.lora_e80_state = {
                "freq_hz": int(m.group(1)), "sf": int(m.group(2)),
                "bw_khz": float(m.group(3)), "bw_idx": int(m.group(4)),
                "cr": int(m.group(5)) - 4,   # 顯示為 4/5..4/8，換算回 firmware cr 值 1..4
                "pwr": int(m.group(6)), "pre": int(m.group(7)),
            }
            self.refresh_lora_panel_values()
            return

        # E80 開機組態行 [LORA 920MHz] ... Freq: 920.000 MHz | Power: +22dBm | BW: 250kHz | SF: 9 ...
        m = re.search(r"\[LORA 920MHz\].*Freq:\s*(\d+)\.(\d+)\s*MHz.*Power:\s*\+?(-?\d+)dBm.*"
                      r"BW:\s*(\d+)kHz.*SF:\s*(\d+)", line)
        if m:
            self.lora_e80_state.update({
                "freq_hz": int(m.group(1)) * 1000000 + int(m.group(2)) * 1000,
                "pwr": int(m.group(3)), "bw_khz": float(m.group(4)), "sf": int(m.group(5)),
            })
            self.refresh_lora_panel_values()
            return

        # --- E22 433MHz 參數回報 ---
        # 地面站格式: [E22] freq=432 MHz  CH=22
        m = re.search(r"\[E22\] freq=(\d+) MHz\s+CH=(\d+)", line)
        if m:
            self.lora_e22_state.update({"freq_mhz": int(m.group(1)), "ch": int(m.group(2))})
            self.refresh_lora_panel_values()
            return

        # 設定成功回報: [E22] freq set 435 MHz (CH=25) OK
        m = re.search(r"\[E22\] freq set (\d+) MHz \(CH=(\d+)\) OK", line)
        if m:
            self.lora_e22_state.update({"freq_mhz": int(m.group(1)), "ch": int(m.group(2))})
            self.refresh_lora_panel_values()
            return

        # 模組暫存器回讀: [LORA433] E22-400T30S | Freq=432.000MHz(CH=22) | Power=21dBm | AirRate=2.4k | ...
        m = re.search(r"\[LORA433\] E22-400T30S \| Freq=(\d+)\.000MHz\(CH=(\d+)\)"
                      r"(?: \| Power=(\S+) \| AirRate=(\S+))?", line)
        if m:
            self.lora_e22_state.update({"freq_mhz": int(m.group(1)), "ch": int(m.group(2))})
            if m.group(3):
                self.lora_e22_state["power"] = m.group(3)
            if m.group(4):
                self.lora_e22_state["air_rate"] = m.group(4)
            self.refresh_lora_panel_values()
            return

        # E22 功率設定成功回報: [E22] pwr set level 3 OK （韌體只回等級，換算成 dBm 顯示）
        m = re.search(r"\[E22\] pwr set level (\d+) OK", line)
        if m:
            lvl = int(m.group(1))
            if 0 <= lvl < len(self.E22_PWR_LEVELS):
                self.lora_e22_state["power"] = self.E22_PWR_LEVELS[lvl][0].split("=")[-1].strip()
            self.refresh_lora_panel_values()
            return

        # E22 空速設定成功回報: [E22] air rate set 2 OK（兩端須一致）
        m = re.search(r"\[E22\] air rate set (\d+) OK", line)
        if m:
            ar = int(m.group(1))
            if 0 <= ar < len(self.E22_AIR_RATES):
                self.lora_e22_state["air_rate"] = self.E22_AIR_RATES[ar][0].split("=")[-1].strip()
            self.refresh_lora_panel_values()
            return

        # --- LoRa 433/920 模組狀態偵測 ---
        if "[LORA]" in line:
            m_st = re.search(r"433:(\w+)\s+920:(\w+)", line)
            if m_st:
                lora433, lora920 = m_st.groups()
                self.update_lora_hw_status(lora433.lower() == "rdy", lora920.lower() == "rdy")
        elif "[GS_STAT]" in line:
            m_st = re.search(r"HW:433=(\w+)\s+920=(\w+)", line)
            if m_st:
                lora433, lora920 = m_st.groups()
                self.update_lora_hw_status(lora433.upper() == "OK", lora920.upper() == "OK")
        elif "[GS_LORA_INIT]" in line:
            m_st = re.search(r"433MHz\(E22\):(\w+)\s+\|\s+920MHz\(E80\):(\w+)", line)
            if m_st:
                lora433, lora920 = m_st.groups()
                self.update_lora_hw_status(lora433.upper() == "OK", lora920.upper() == "OK")

    # ==================== LoRa 參數設定面板 ====================
    # 可調範圍（與 firmware 檢查一致；顯示於 GUI 並用於本地預檢）
    E22_FREQ_RANGE = (410, 493)          # MHz
    E80_FREQ_SUGGEST = (862000000, 928000000)  # Hz（僅警告，不硬擋）
    E80_SF_RANGE = (7, 12)
    E80_PWR_RANGE = (-9, 22)             # dBm
    E80_PRE_RANGE = (6, 65535)
    E22_PWR_LEVELS = [
        ("0 = 30 dBm (1W)", "⚠ 3V3 供電會欠壓斷線，勿用"),
        ("1 = 27 dBm", ""),
        ("2 = 24 dBm", ""),
        ("3 = 21 dBm", "✅ 建議（預設）"),
    ]
    E22_AIR_RATES = [
        ("0 = 0.3k bps", "最遠射程"),
        ("1 = 1.2k bps", ""),
        ("2 = 2.4k bps", "✅ 建議（預設）"),
        ("3 = 4.8k bps", ""),
        ("4 = 9.6k bps", ""),
        ("5 = 19.2k bps", ""),
        ("6 = 38.4k bps", ""),
        ("7 = 62.5k bps", "⚠ 台面實測大量 CRC 錯"),
    ]
    E80_BW_OPTIONS = [
        ("3 = 62.5 kHz", "靈敏度最好、最慢"),
        ("4 = 125 kHz", ""),
        ("5 = 250 kHz", "✅ 預設"),
        ("6 = 500 kHz", "最快、靈敏度 -3dB/檔"),
    ]
    E80_CR_OPTIONS = [
        ("1 = 4/5", "✅ 預設（開銷最小）"),
        ("2 = 4/6", ""),
        ("3 = 4/7", ""),
        ("4 = 4/8", "糾錯最強、有效資料率最低"),
    ]

    LORA_TUTORIAL = """📡 LoRa 可調參數教學與範圍速查
════════════════════════════════════════════

【通則 — 先讀這段】
• 下列參數「兩端（火箭主航電 ↔ 地面站）必須完全一致」才能通訊：
    E22：頻率、空中速率(air)
    E80：頻率、SF、BW、CR、前導碼、(SyncWord 固定 0x12 不開放調整)
• 發射功率兩端可以不同（只影響自己發出的訊號強度）。
• 建議流程：兩塊板分別接上 USB-TTL → 各自按「查詢目前設定」核對現值
  → 依序把兩端改成相同參數 → 在地面站用 stats 確認收包率/RSSI。
• 航電端改參數時遙測會暫停約 0.3 秒（防止資料流打斷模組設定），屬正常現象。
• 備援航電 BACKUP 無 LoRa 硬體，無參數可調。

【E22-400T30S — 433MHz UART 透傳模組】
• 頻率 freq：410 ~ 493 MHz（頻道 CH = 頻率 − 410，共 84 個頻道）。
  設定存入模組 EEPROM 掉電不遺失；但重新開機時 firmware 會強制寫回
  預設 432 MHz（保證兩端一致），要長期改頻率需改 firmware 巨集 E22_TX_FREQ_MHZ。
• 發射功率 pwr：0=30dBm 1=27dBm 2=24dBm 3=21dBm。
  ⚠ 本板 3V3 供電無法穩定驅動 30dBm（突波電流 ~600mA 拉垮 3V3 → 模組欠壓
  →「發幾秒就斷」），建議維持 3 (21dBm)。
• 空中速率 air：0=0.3k 1=1.2k 2=2.4k 3=4.8k 4=9.6k 5=19.2k 6=38.4k 7=62.5k bps。
  速率越低 → 靈敏度越好、射程越遠、抗干擾越強，但資料率越低。
  實測 62.5k 台面就大量 CRC 錯；2.4k 約多 15~20dB 鏈路餘裕（預設）。⚠ 兩端須一致。
• E22 為透傳模組：RSSI/SNR 無法回讀；UART baud/parity 位元 firmware
  刻意不開放（寫錯會與模組永久失聯）。

【E80-900M2213S — 920MHz LR1121 SPI 模組】
• 頻率 freq：以 Hz 輸入（例 920000000 = 920 MHz）。
  晶片支援 862~1020 MHz，但板上天線與帶通濾波器調在 920 MHz，
  偏離越遠衰減越大 → 建議只在 915~923 MHz 內微調。
• 展頻因子 SF：7 ~ 12。每 +1 → 靈敏度約 +2.5dB（射程更遠），
  但空中時間 ×2（資料率減半）。預設 SF9。
• 頻寬 BW（idx）：3=62.5k 4=125k 5=250k(預設) 6=500k。
  頻寬加倍 → 資料率加倍、靈敏度約 −3dB。
• 編碼率 CR：1=4/5(預設) 2=4/6 3=4/7 4=4/8。數字越大糾錯越強、有效資料率越低。
• 發射功率 pwr：−9 ~ +22 dBm（22 = HP PA 上限，預設）。
• 前導碼 pre：6 ~ 65535 符號。弱訊號時加長可提升同步成功率，一般 8 即可。
• LDRO（低資料率最佳化）由 firmware 依 SF/BW 自動計算，無需手動設。

【空中時間參考（77-byte 遙測封包）】
• SF7 / BW500k / CR4-5  ≈ 0.035 秒/包（台面除錯、快速刷新）
• SF9 / BW250k / CR4-5  ≈ 0.23 秒/包（目前預設，均衡）
• SF12 / BW125k / CR4-8 > 3 秒/包（極限射程；遙測 5Hz 完全跟不上，
  發送端會自動跳包 = 更新率大幅下降）

【角色差異】
• 主航電 PRIMARY：可調 E22 + E80（E80 為發射端）。
• 備援航電 BACKUP：無 LoRa 硬體。
• 地面站 GROUND：可調 E22 + E80（E80 為接收端），
  另有 stats 統計 / e80 init / e80 rxstart / airtime 估算工具。

【改完不通了怎麼辦】
• E80：對兩端各下一次「查詢」核對六個參數逐項一致；仍不通 → 地面站按
  「e80 init」重新初始化再「e80 rxstart」。
• E22：重開機兩端（firmware 開機會強制寫回預設頻道 432MHz、功率 21dBm、
  空速 2.4k），即可回到已知良好狀態。
"""

    def open_lora_panel(self):
        if not self.running:
            messagebox.showwarning("提示", "請先開啟串口連接裝置，再進行 LoRa 參數設定。")
            return
        if hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists():
            self.lora_win.lift()
            return

        self.lora_win = tk.Toplevel(self.root)
        self.lora_win.title("📡 LoRa 通訊參數設定（E22 433MHz / E80 920MHz）")
        self.lora_win.geometry("980x700")
        self.lora_win.configure(bg="#1c1c1c")
        # 不用 transient(self.root)：macOS 上 transient 視窗沒有獨立 Dock 圖示，縮小/關閉
        # 主視窗或任一子視窗會把整組一起帶走，子視窗需要能各自獨立縮小/關閉。

        self.lora_widgets_common = []   # BACKUP 時停用
        self.lora_widgets_ground = []   # 僅 GROUND 啟用

        # ---- 頂部：角色 + 提示列 ----
        self.lbl_lora_role = tk.Label(self.lora_win, text="", bg="#1c1c1c", fg="#00d2ff",
                                      font=("Helvetica", 12, "bold"), anchor="w")
        self.lbl_lora_role.pack(fill=tk.X, padx=16, pady=(12, 2))

        tk.Label(self.lora_win,
                 text="⚠ 頻率/SF/BW/CR/空速 兩端（火箭 ↔ 地面站）必須一致才能通訊；詳見「📖 參數教學」分頁",
                 bg="#1c1c1c", fg="#ffcc00", font=("Helvetica", 10), anchor="w").pack(fill=tk.X, padx=16, pady=(0, 8))

        nb = ttk.Notebook(self.lora_win)
        nb.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

        tab_e22 = tk.Frame(nb, bg="#1c1c1c")
        tab_e80 = tk.Frame(nb, bg="#1c1c1c")
        tab_gs = tk.Frame(nb, bg="#1c1c1c")
        tab_doc = tk.Frame(nb, bg="#1c1c1c")
        nb.add(tab_e22, text=" E22 433MHz ")
        nb.add(tab_e80, text=" E80 920MHz ")
        nb.add(tab_gs, text=" 統計/工具（地面站） ")
        nb.add(tab_doc, text=" 📖 參數教學 ")

        self.build_e22_tab(tab_e22)
        self.build_e80_tab(tab_e80)
        self.build_gs_tab(tab_gs)
        self.build_doc_tab(tab_doc)

        self.refresh_lora_panel_role()
        self.refresh_lora_panel_values()

        # 開啟即查詢兩鏈路現值（錯開避免回報交錯）
        self.lora_send("e22 show")
        self.lora_win.after(700, lambda: self.lora_send("e80 show"))

    # ---- 小工具 ----
    def lora_send(self, cmd):
        """LoRa 面板統一發送入口（沿用 send_command 的 [CMD] 終端回顯）：e22/e80 頻率/
        功率/空速/SF/BW/CR/前導碼、stats、ver、e80 init/rxstart/airtime 等。"""
        return self.send_command(cmd)

    def lora_confirm_link_break(self, what):
        return messagebox.askyesno(
            "確認變更",
            f"即將變更 {what}。\n\n此參數兩端必須一致：只改一端會令鏈路中斷，"
            f"需到另一塊板上設定相同值才能恢復通訊。\n\n確定要套用嗎？",
            parent=self.lora_win)

    def _lora_row(self, parent, row, label_text, hint_text):
        """網格列：左標籤 + 右提示；回傳中欄容器供放輸入元件"""
        tk.Label(parent, text=label_text, bg="#1c1c1c", fg="#ffffff",
                 font=("Helvetica", 10, "bold"), anchor="w", width=16).grid(
            row=row, column=0, sticky="w", padx=(14, 6), pady=6)
        holder = tk.Frame(parent, bg="#1c1c1c")
        holder.grid(row=row, column=1, sticky="w", pady=6)
        tk.Label(parent, text=hint_text, bg="#1c1c1c", fg="#8a8a8a",
                 font=("Helvetica", 9), anchor="w", justify=tk.LEFT).grid(
            row=row, column=2, sticky="w", padx=10, pady=6)
        return holder

    # ---- E22 433MHz 分頁 ----
    def build_e22_tab(self, tab):
        self.lbl_e22_cur = tk.Label(tab, text="目前板上設定：（按「查詢」讀取）", bg="#101014",
                                    fg="#00d2ff", font=("Monaco", 10), anchor="w",
                                    padx=10, pady=8, justify=tk.LEFT)
        self.lbl_e22_cur.pack(fill=tk.X, padx=14, pady=(12, 8))

        grid = tk.Frame(tab, bg="#1c1c1c")
        grid.pack(fill=tk.X, anchor="w")

        # 頻率
        h = self._lora_row(grid, 0, "頻率 (MHz)",
                           f"範圍 {self.E22_FREQ_RANGE[0]}–{self.E22_FREQ_RANGE[1]} MHz（CH=頻率−410）\n"
                           "開機會被 firmware 強制寫回預設 432 MHz")
        self.e22_freq_var = tk.StringVar(value="432")
        tk.Spinbox(h, from_=self.E22_FREQ_RANGE[0], to=self.E22_FREQ_RANGE[1],
                   textvariable=self.e22_freq_var, width=8, font=("Monaco", 11),
                   bg="#252525", fg="#ffffff", insertbackground="white").pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用頻率", width=9, command=self.apply_e22_freq)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # 發射功率
        h = self._lora_row(grid, 1, "發射功率",
                           "⚠ 本板 3V3 供電，30dBm 會欠壓斷線\n建議固定 3 (21dBm)")
        self.e22_pwr_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E22_PWR_LEVELS],
                                          width=30, state="readonly", font=("Helvetica", 10))
        self.e22_pwr_combo.current(3)
        self.e22_pwr_combo.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用功率", width=9, command=self.apply_e22_pwr)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # 空中速率
        h = self._lora_row(grid, 2, "空中速率",
                           "越低 → 射程越遠/抗干擾越強、資料率越低\n⚠ 兩端必須一致")
        self.e22_air_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E22_AIR_RATES],
                                          width=30, state="readonly", font=("Helvetica", 10))
        self.e22_air_combo.current(2)
        self.e22_air_combo.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用空速", width=9, command=self.apply_e22_air)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        tools = tk.Frame(tab, bg="#1c1c1c")
        tools.pack(fill=tk.X, padx=14, pady=12)
        b = ttk.Button(tools, text="🔍 查詢目前設定 (e22 show)", width=24,
                       command=lambda: self.lora_send("e22 show"))
        b.pack(side=tk.LEFT)
        self.lora_widgets_common.append(b)

        tk.Label(tab, text="說明：E22 為 UART 透傳模組，設定存入模組 EEPROM。RSSI/SNR 無法回讀；\n"
                           "UART baud/parity 不開放調整（寫錯會與模組永久失聯）。",
                 bg="#1c1c1c", fg="#8a8a8a", font=("Helvetica", 9), justify=tk.LEFT,
                 anchor="w").pack(fill=tk.X, padx=14, pady=(4, 0))

    def apply_e22_freq(self):
        try:
            mhz = int(self.e22_freq_var.get())
        except ValueError:
            messagebox.showerror("錯誤", "頻率須為整數 MHz", parent=self.lora_win)
            return
        lo, hi = self.E22_FREQ_RANGE
        if not (lo <= mhz <= hi):
            messagebox.showerror("超出範圍", f"E22 頻率範圍 {lo}–{hi} MHz", parent=self.lora_win)
            return
        if self.lora_confirm_link_break(f"E22 頻率 → {mhz} MHz"):
            self.lora_send(f"e22 freq {mhz}")

    def apply_e22_pwr(self):
        lvl = self.e22_pwr_combo.current()
        if lvl == 0 and not messagebox.askyesno(
                "高功率警告", "30dBm 需 1W 發射，3V3 供電會欠壓造成模組斷線！\n仍要套用嗎？",
                parent=self.lora_win):
            return
        self.lora_send(f"e22 pwr {lvl}")

    def apply_e22_air(self):
        ar = self.e22_air_combo.current()
        if self.lora_confirm_link_break(f"E22 空中速率 → {self.E22_AIR_RATES[ar][0]}"):
            self.lora_send(f"e22 air {ar}")

    # ---- E80 920MHz 分頁 ----
    def build_e80_tab(self, tab):
        self.lbl_e80_cur = tk.Label(tab, text="目前板上參數：（按「查詢」讀取）", bg="#101014",
                                    fg="#00d2ff", font=("Monaco", 10), anchor="w",
                                    padx=10, pady=8, justify=tk.LEFT)
        self.lbl_e80_cur.pack(fill=tk.X, padx=14, pady=(12, 8))

        grid = tk.Frame(tab, bg="#1c1c1c")
        grid.pack(fill=tk.X, anchor="w")

        # 頻率
        h = self._lora_row(grid, 0, "頻率 (Hz)",
                           "建議 862–928 MHz；天線/濾波器調在 920 MHz\n"
                           "偏離越遠訊號衰減越大（建議 915–923 內微調）")
        self.e80_freq_var = tk.StringVar(value="920000000")
        e = tk.Entry(h, textvariable=self.e80_freq_var, width=12, font=("Monaco", 11),
                     bg="#252525", fg="#ffffff", insertbackground="white")
        e.pack(side=tk.LEFT)
        self.lbl_e80_freq_mhz = tk.Label(h, text="= 920.000 MHz", bg="#1c1c1c", fg="#00e676",
                                         font=("Monaco", 10))
        self.lbl_e80_freq_mhz.pack(side=tk.LEFT, padx=6)
        self.e80_freq_var.trace_add("write", lambda *_: self._update_e80_mhz_hint())
        b = ttk.Button(h, text="套用頻率", width=9, command=self.apply_e80_freq)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # SF
        h = self._lora_row(grid, 1, "展頻因子 SF",
                           "7–12；每 +1 → 靈敏度 +~2.5dB、空中時間 ×2\n預設 SF9")
        self.e80_sf_var = tk.StringVar(value="9")
        tk.Spinbox(h, from_=self.E80_SF_RANGE[0], to=self.E80_SF_RANGE[1],
                   textvariable=self.e80_sf_var, width=5, font=("Monaco", 11),
                   bg="#252525", fg="#ffffff", insertbackground="white").pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用 SF", width=9, command=self.apply_e80_sf)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # BW
        h = self._lora_row(grid, 2, "頻寬 BW",
                           "加倍 → 資料率加倍、靈敏度 −~3dB\n預設 5 (250kHz)")
        self.e80_bw_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E80_BW_OPTIONS],
                                         width=28, state="readonly", font=("Helvetica", 10))
        self.e80_bw_combo.current(2)
        self.e80_bw_combo.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用 BW", width=9, command=self.apply_e80_bw)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # CR
        h = self._lora_row(grid, 3, "編碼率 CR",
                           "越大 → 糾錯越強、有效資料率越低\n預設 1 (4/5)")
        self.e80_cr_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E80_CR_OPTIONS],
                                         width=28, state="readonly", font=("Helvetica", 10))
        self.e80_cr_combo.current(0)
        self.e80_cr_combo.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用 CR", width=9, command=self.apply_e80_cr)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # 功率
        h = self._lora_row(grid, 4, "發射功率 (dBm)",
                           f"範圍 {self.E80_PWR_RANGE[0]} ~ +{self.E80_PWR_RANGE[1]} dBm\n"
                           "22 = HP PA 上限（預設）；兩端可不同")
        self.e80_pwr_var = tk.StringVar(value="22")
        tk.Spinbox(h, from_=self.E80_PWR_RANGE[0], to=self.E80_PWR_RANGE[1],
                   textvariable=self.e80_pwr_var, width=5, font=("Monaco", 11),
                   bg="#252525", fg="#ffffff", insertbackground="white").pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用功率", width=9, command=self.apply_e80_pwr)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        # 前導碼
        h = self._lora_row(grid, 5, "前導碼 (符號)",
                           f"範圍 {self.E80_PRE_RANGE[0]}–{self.E80_PRE_RANGE[1]}；一般 8 即可\n"
                           "弱訊號時加長可提升同步成功率")
        self.e80_pre_var = tk.StringVar(value="8")
        e = tk.Entry(h, textvariable=self.e80_pre_var, width=7, font=("Monaco", 11),
                     bg="#252525", fg="#ffffff", insertbackground="white")
        e.pack(side=tk.LEFT)
        b = ttk.Button(h, text="套用前導碼", width=9, command=self.apply_e80_pre)
        b.pack(side=tk.LEFT, padx=8)
        self.lora_widgets_common.append(b)

        tools = tk.Frame(tab, bg="#1c1c1c")
        tools.pack(fill=tk.X, padx=14, pady=12)
        b = ttk.Button(tools, text="🔍 查詢目前參數 (e80 show)", width=24,
                       command=lambda: self.lora_send("e80 show"))
        b.pack(side=tk.LEFT)
        self.lora_widgets_common.append(b)

        tk.Label(tab, text="說明：SyncWord 固定 0x12（兩端 firmware 相同即一致）；LDRO 由 firmware 依 SF/BW 自動計算。\n"
                           "改任一參數 firmware 會整組重新配置並回報 [E80] 現值行（自動更新上方欄位）。",
                 bg="#1c1c1c", fg="#8a8a8a", font=("Helvetica", 9), justify=tk.LEFT,
                 anchor="w").pack(fill=tk.X, padx=14, pady=(4, 0))

    def _update_e80_mhz_hint(self):
        try:
            hz = int(self.e80_freq_var.get())
            self.lbl_e80_freq_mhz.config(text=f"= {hz/1e6:.3f} MHz")
        except (ValueError, tk.TclError):
            self.lbl_e80_freq_mhz.config(text="= ?")

    def apply_e80_freq(self):
        try:
            hz = int(self.e80_freq_var.get())
        except ValueError:
            messagebox.showerror("錯誤", "頻率須為整數 Hz（例 920000000）", parent=self.lora_win)
            return
        lo, hi = self.E80_FREQ_SUGGEST
        if not (lo <= hz <= hi):
            if not messagebox.askyesno("頻率超出建議範圍",
                                       f"{hz/1e6:.3f} MHz 超出建議 862–928 MHz。\n"
                                       "板上天線/濾波器調在 920 MHz，偏離會嚴重衰減。\n仍要套用嗎？",
                                       parent=self.lora_win):
                return
        if self.lora_confirm_link_break(f"E80 頻率 → {hz/1e6:.3f} MHz"):
            self.lora_send(f"e80 freq {hz}")

    def apply_e80_sf(self):
        try:
            sf = int(self.e80_sf_var.get())
        except ValueError:
            return
        lo, hi = self.E80_SF_RANGE
        if not (lo <= sf <= hi):
            messagebox.showerror("超出範圍", f"SF 範圍 {lo}–{hi}", parent=self.lora_win)
            return
        if self.lora_confirm_link_break(f"E80 SF → {sf}"):
            self.lora_send(f"e80 sf {sf}")

    def apply_e80_bw(self):
        idx = int(self.E80_BW_OPTIONS[self.e80_bw_combo.current()][0].split(" ")[0])
        if self.lora_confirm_link_break(f"E80 BW → idx {idx}"):
            self.lora_send(f"e80 bw {idx}")

    def apply_e80_cr(self):
        cr = int(self.E80_CR_OPTIONS[self.e80_cr_combo.current()][0].split(" ")[0])
        if self.lora_confirm_link_break(f"E80 CR → {cr}"):
            self.lora_send(f"e80 cr {cr}")

    def apply_e80_pwr(self):
        try:
            pwr = int(self.e80_pwr_var.get())
        except ValueError:
            return
        lo, hi = self.E80_PWR_RANGE
        if not (lo <= pwr <= hi):
            messagebox.showerror("超出範圍", f"功率範圍 {lo} ~ {hi} dBm", parent=self.lora_win)
            return
        self.lora_send(f"e80 pwr {pwr}")   # 功率不破壞鏈路，免確認

    def apply_e80_pre(self):
        try:
            pre = int(self.e80_pre_var.get())
        except ValueError:
            return
        lo, hi = self.E80_PRE_RANGE
        if not (lo <= pre <= hi):
            messagebox.showerror("超出範圍", f"前導碼範圍 {lo}–{hi}", parent=self.lora_win)
            return
        if self.lora_confirm_link_break(f"E80 前導碼 → {pre}"):
            self.lora_send(f"e80 pre {pre}")

    # ---- 統計/工具（地面站）分頁 ----
    def build_gs_tab(self, tab):
        tk.Label(tab, text="以下工具僅地面站 (GROUND) firmware 支援：", bg="#1c1c1c", fg="#ffcc00",
                 font=("Helvetica", 10, "bold"), anchor="w").pack(fill=tk.X, padx=14, pady=(12, 6))

        row1 = tk.Frame(tab, bg="#1c1c1c")
        row1.pack(fill=tk.X, padx=14, pady=4)
        for text, cmd in [("📊 顯示統計 (stats)", "stats"),
                          ("🧹 清除統計 (stats reset)", "stats reset"),
                          ("🛰 E80 版本診斷 (ver)", "ver")]:
            b = ttk.Button(row1, text=text, width=24, command=lambda c=cmd: self.lora_send(c))
            b.pack(side=tk.LEFT, padx=4)
            self.lora_widgets_ground.append(b)

        row2 = tk.Frame(tab, bg="#1c1c1c")
        row2.pack(fill=tk.X, padx=14, pady=4)
        tk.Label(row2, text="自動列印統計間隔(秒, 0=關閉):", bg="#1c1c1c", fg="#ffffff",
                 font=("Helvetica", 10)).pack(side=tk.LEFT)
        self.gs_auto_var = tk.StringVar(value="5")
        tk.Spinbox(row2, from_=0, to=3600, textvariable=self.gs_auto_var, width=6,
                   font=("Monaco", 11), bg="#252525", fg="#ffffff",
                   insertbackground="white").pack(side=tk.LEFT, padx=6)
        b = ttk.Button(row2, text="套用 (stats auto N)", width=18,
                       command=lambda: self.lora_send(f"stats auto {self.gs_auto_var.get()}"))
        b.pack(side=tk.LEFT, padx=4)
        self.lora_widgets_ground.append(b)

        row3 = tk.Frame(tab, bg="#1c1c1c")
        row3.pack(fill=tk.X, padx=14, pady=4)
        for text, cmd in [("♻️ E80 重新初始化+收 (e80 init)", "e80 init"),
                          ("▶️ 重新進入連續接收 (e80 rxstart)", "e80 rxstart")]:
            b = ttk.Button(row3, text=text, width=28, command=lambda c=cmd: self.lora_send(c))
            b.pack(side=tk.LEFT, padx=4)
            self.lora_widgets_ground.append(b)

        row4 = tk.Frame(tab, bg="#1c1c1c")
        row4.pack(fill=tk.X, padx=14, pady=4)
        tk.Label(row4, text="空中時間估算 payload 長度(B):", bg="#1c1c1c", fg="#ffffff",
                 font=("Helvetica", 10)).pack(side=tk.LEFT)
        self.gs_airtime_var = tk.StringVar(value="77")
        tk.Spinbox(row4, from_=1, to=255, textvariable=self.gs_airtime_var, width=5,
                   font=("Monaco", 11), bg="#252525", fg="#ffffff",
                   insertbackground="white").pack(side=tk.LEFT, padx=6)
        b = ttk.Button(row4, text="估算 (e80 airtime N)", width=18,
                       command=lambda: self.lora_send(f"e80 airtime {self.gs_airtime_var.get()}"))
        b.pack(side=tk.LEFT, padx=4)
        self.lora_widgets_ground.append(b)

        tk.Label(tab, text="統計輸出範例：[STATS] pkt_ok / crc_err / rate / RSSI / SNR（920 才有 RSSI/SNR，\n"
                           "E22 透傳模式無法回讀）。改完參數後看 rate 與 crc_err 判斷鏈路品質。",
                 bg="#1c1c1c", fg="#8a8a8a", font=("Helvetica", 9), justify=tk.LEFT,
                 anchor="w").pack(fill=tk.X, padx=14, pady=(10, 0))

    # ---- 教學分頁 ----
    def build_doc_tab(self, tab):
        doc = ScrolledText(tab, bg="#101010", fg="#d0d0d0", font=("Monaco", 10),
                           borderwidth=0, highlightthickness=0, wrap=tk.WORD)
        doc.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        doc.insert(tk.END, self.LORA_TUTORIAL)
        doc.config(state=tk.DISABLED)

    # ---- 面板刷新 ----
    def refresh_lora_panel_role(self):
        if not (hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists()):
            return
        role = self.detected_role
        if role == "BACKUP":
            header = "🛟 備援航電 BACKUP — 無 LoRa 硬體，無參數可調"
            color = "#ffcc00"
            common, ground = tk.DISABLED, tk.DISABLED
        elif role == "GROUND":
            header = "📡 地面站 GROUND — E80 為接收端；可用「統計/工具」分頁驗證收包"
            color = "#e07bfb"
            common, ground = tk.NORMAL, tk.NORMAL
        elif role == "PRIMARY":
            header = "🚀 主航電 PRIMARY — E80 為發射端；套用參數時遙測暫停約 0.3 秒"
            color = "#00e676"
            common, ground = tk.NORMAL, tk.DISABLED
        else:
            header = "⚫ 角色偵測中… 仍可手動操作（三種角色命令語法相同）"
            color = "#aaaaaa"
            common, ground = tk.NORMAL, tk.NORMAL
        self.lbl_lora_role.config(text=header, fg=color)
        for w in self.lora_widgets_common:
            try:
                w.config(state=common)
            except tk.TclError:
                pass
        for w in self.lora_widgets_ground:
            try:
                w.config(state=ground)
            except tk.TclError:
                pass

    def refresh_lora_panel_values(self):
        """把最近解析到的板上現值刷到面板（僅更新「目前設定」顯示，不覆寫使用者輸入框）"""
        if not (hasattr(self, 'lora_win') and self.lora_win and self.lora_win.winfo_exists()):
            return
        s = self.lora_e22_state
        if s:
            parts = []
            if "freq_mhz" in s:
                parts.append(f"頻率 {s['freq_mhz']} MHz (CH={s.get('ch', '?')})")
            if "power" in s:
                parts.append(f"功率 {s['power']}")
            if "air_rate" in s:
                parts.append(f"空速 {s['air_rate']}")
            self.lbl_e22_cur.config(text="目前板上設定：" + "  |  ".join(parts))
        s = self.lora_e80_state
        if s:
            parts = []
            if "freq_hz" in s:
                parts.append(f"頻率 {s['freq_hz']/1e6:.3f} MHz")
            if "sf" in s:
                parts.append(f"SF{s['sf']}")
            if "bw_khz" in s:
                idx = f" (idx={s['bw_idx']})" if "bw_idx" in s else ""
                parts.append(f"BW {s['bw_khz']:g} kHz{idx}")
            if "cr" in s:
                parts.append(f"CR 4/{s['cr'] + 4}")
            if "pwr" in s:
                parts.append(f"功率 {s['pwr']:+d} dBm")
            if "pre" in s:
                parts.append(f"前導碼 {s['pre']}")
            self.lbl_e80_cur.config(text="目前板上參數：" + "  |  ".join(parts))

    def open_mag_calibration(self):
        if not self.running:
            messagebox.showwarning("提示", "請先開啟串口連接航電板，再進行磁強計校正。")
            return
            
        # 若已開啟，提升焦點
        if hasattr(self, 'calib_win') and self.calib_win and self.calib_win.winfo_exists():
            self.calib_win.lift()
            return
            
        # 建立彈出視窗
        self.calib_win = tk.Toplevel(self.root)
        self.calib_win.title("🧲 地磁計硬鐵校正與航向鎖定")
        self.calib_win.geometry("900x520")
        self.calib_win.configure(bg="#1c1c1c")
        # 不用 transient()：見 open_lora_panel 註解，子視窗要能各自獨立縮小/關閉；
        # grab_set() 仍保留，校正過程需要 modal 獨佔輸入焦點。
        self.calib_win.grab_set()
        # 關窗時保證恢復 1Hz（否則板子會卡在 10Hz 提速模式）
        self.calib_win.protocol("WM_DELETE_WINDOW", self._close_mag_calibration)

        # 重設校正收集狀態
        self.calib_x = []
        self.calib_y = []
        self.calib_z = []
        self.collecting_data = False
        
        # 分割視窗為左側(繪圖)與右側(控制)
        left_frame = tk.Frame(self.calib_win, bg="#1c1c1c")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=15, pady=15)
        
        right_frame = tk.Frame(self.calib_win, bg="#222222", width=350)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=15, pady=15)
        right_frame.pack_propagate(False)
        
        tk.Label(left_frame, text="3D 地磁數據分佈 (Bx, By, Bz)", bg="#1c1c1c", fg="#00d2ff", font=("Helvetica", 11, "bold")).pack(anchor="w", pady=(0,5))
        
        # 左側 Matplotlib 3D 繪圖
        self.calib_fig = plt.figure(facecolor="#1c1c1c")
        self.calib_ax = self.calib_fig.add_subplot(111, projection='3d')
        self.calib_ax.set_facecolor("#151515")
        
        self.calib_ax.set_xlabel("Bx (mG)", color="#aaaaaa")
        self.calib_ax.set_ylabel("By (mG)", color="#aaaaaa")
        self.calib_ax.set_zlabel("Bz (mG)", color="#aaaaaa")
        self.calib_ax.tick_params(colors="#aaaaaa")
        
        self.calib_canvas = FigureCanvasTkAgg(self.calib_fig, master=left_frame)
        self.calib_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 右側控制面板
        tk.Label(right_frame, text="磁力計控制面板 (3D)", bg="#222222", fg="#00d2ff", font=("Helvetica", 12, "bold")).pack(anchor="w", padx=15, pady=15)
        
        # 當前偏置與即時強度
        self.lbl_board_offsets = tk.Label(right_frame, text=f"當前板載偏置 (Counts):\n  X={int(self.board_mag_offsets[0])}, Y={int(self.board_mag_offsets[1])}, Z={int(self.board_mag_offsets[2])}", bg="#222222", fg="#ffffff", font=("Monaco", 9), justify=tk.LEFT, anchor="w")
        self.lbl_board_offsets.pack(fill=tk.X, padx=15, pady=5)
        
        self.lbl_live_mag = tk.Label(right_frame, text="即時強度: mx=0.0, my=0.0, mz=0.0 mG\n即時航向: hdg=0.0°", bg="#222222", fg="#aaaaaa", font=("Monaco", 9), justify=tk.LEFT, anchor="w")
        self.lbl_live_mag.pack(fill=tk.X, padx=15, pady=5)
        
        # 數據收集按鈕
        btn_data_frame = tk.Frame(right_frame, bg="#222222")
        btn_data_frame.pack(fill=tk.X, padx=15, pady=10)
        
        self.btn_toggle_collect = ttk.Button(btn_data_frame, text="開始收集數據", command=self.toggle_collecting)
        self.btn_toggle_collect.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,5))
        
        btn_clear_collect = ttk.Button(btn_data_frame, text="清除數據", command=self.clear_calib_data)
        btn_clear_collect.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5,0))
        
        # 擬合結果與按鈕
        self.lbl_fit_results = tk.Label(right_frame, text="擬合結果:\n  (無擬合數據)", bg="#222222", fg="#888888", font=("Helvetica", 9), justify=tk.LEFT, anchor="w")
        self.lbl_fit_results.pack(fill=tk.X, padx=15, pady=8)
        
        self.btn_fit = ttk.Button(right_frame, text="計算 3D 硬鐵校正", command=self.fit_mag_3d_sphere)
        self.btn_fit.pack(fill=tk.X, padx=15, pady=5)
        
        self.btn_write_calib = ttk.Button(right_frame, text="寫入偏置至 Flash", state=tk.DISABLED, command=self.write_mag_calibration)
        self.btn_write_calib.pack(fill=tk.X, padx=15, pady=5)
        
        tk.Frame(right_frame, height=2, bg="#3d3d3d").pack(fill=tk.X, padx=15, pady=10)
        
        # EKF 鎖定選項
        tk.Label(right_frame, text="EKF 絕對磁北航向鎖定", bg="#222222", fg="#00d2ff", font=("Helvetica", 10, "bold")).pack(anchor="w", padx=15, pady=2)

        # 目前鎖定狀態：韌體無查詢命令，開窗先顯示預設 (g_mag_yaw_lock=1)，收到 [CAL] 回傳才轉「已確認」
        self.lbl_yaw_lock = tk.Label(right_frame, text="磁北鎖定：🔒 開（預設，未確認）",
                                     bg="#222222", fg="#888888", font=("Helvetica", 9), anchor="w")
        self.lbl_yaw_lock.pack(anchor="w", padx=15, pady=(0, 4))

        btn_yaw_frame = tk.Frame(right_frame, bg="#222222")
        btn_yaw_frame.pack(fill=tk.X, padx=15, pady=5)
        
        ttk.Button(btn_yaw_frame, text="啟用磁北鎖定", command=lambda: self.toggle_mag_yaw_lock(True)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,5))
        ttk.Button(btn_yaw_frame, text="停用磁北鎖定", command=lambda: self.toggle_mag_yaw_lock(False)).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5,0))
        
        ttk.Button(right_frame, text="重置硬鐵偏置 (預設 131072)", command=self.reset_mag_calibration).pack(fill=tk.X, padx=15, pady=10)
        
        # 啟動定時繪圖更新
        self.update_calib_plot()

        # 開窗即把 [MAG] 提速到 10Hz（CMD_MAG_CAL_START），整個校正過程都快；
        # 韌體有 60s auto-stop guard，關窗也會送 STOP。
        self.send_command("CMD_MAG_CAL_START")

    def toggle_collecting(self):
        self.collecting_data = not self.collecting_data
        if self.collecting_data:
            self.btn_toggle_collect.config(text="停止收集數據")
            self.calib_x = []
            self.calib_y = []
            self.calib_z = []
            self.lbl_fit_results.config(text="擬合結果:\n  (收集中，請繞 X/Y/Z 三軸旋轉航電板或畫 8 字...)", fg="#ffcc00")
            self.btn_write_calib.config(state=tk.DISABLED)
        else:
            self.btn_toggle_collect.config(text="開始收集數據")
            self.lbl_fit_results.config(text=f"擬合結果:\n  (已停止收集，共 {len(self.calib_x)} 點數據)", fg="#ffffff")

    def clear_calib_data(self):
        self.calib_x = []
        self.calib_y = []
        self.calib_z = []
        self.collecting_data = False
        self.btn_toggle_collect.config(text="開始收集數據")
        self.lbl_fit_results.config(text="擬合結果:\n  (無擬合數據)", fg="#888888")
        self.btn_write_calib.config(state=tk.DISABLED)
        self.calib_ax.clear()
        self.calib_ax.set_facecolor("#151515")
        self.calib_ax.set_xlabel("Bx (mG)", color="#aaaaaa")
        self.calib_ax.set_ylabel("By (mG)", color="#aaaaaa")
        self.calib_ax.set_zlabel("Bz (mG)", color="#aaaaaa")
        self.calib_ax.tick_params(colors="#aaaaaa")
        self.calib_canvas.draw()

    def _close_mag_calibration(self):
        """關閉校正視窗：恢復 [MAG] 1Hz（CMD_MAG_CAL_STOP），再關窗。"""
        self.collecting_data = False
        self.send_command("CMD_MAG_CAL_STOP")
        try:
            self.calib_win.grab_release()
        except Exception:
            pass
        self.calib_win.destroy()

    def fit_mag_3d_sphere(self):
        if len(self.calib_x) < 16:
            messagebox.showwarning("警告", "收集的數據點太少（至少需要 16 點以上，建議繞 X/Y/Z 三軸旋轉或畫 8 字）。", parent=self.calib_win)
            return
            
        try:
            x = np.array(self.calib_x)
            y = np.array(self.calib_y)
            z = np.array(self.calib_z)
            
            # 3D 球面擬合 (最小二乘法解 x^2 + y^2 + z^2 = D*x + E*y + F*z + G)
            A = np.column_stack((x, y, z, np.ones_like(x)))
            B = x**2 + y**2 + z**2
            res, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
            
            cx_fit = res[0] / 2.0
            cy_fit = res[1] / 2.0
            cz_fit = res[2] / 2.0
            R_fit = np.sqrt(max(0.0, res[3] + cx_fit**2 + cy_fit**2 + cz_fit**2))
            
            current_ox = self.board_mag_offsets[0]   # 晶片 X 軸 offset (counts)
            current_oy = self.board_mag_offsets[1]   # 晶片 Y 軸 offset (counts)
            current_oz = self.board_mag_offsets[2]   # 晶片 Z 軸 offset (counts)

            # calib_x/y/z 收的是 body 軸 (mx_body, my_body, mz_body)，但 offset[] 是晶片軸。
            # 依 sensor_axis.h 與 mag_calibrate.py：
            #   bx=+sy, by=-sx, bz=-sz
            # 還原到晶片軸為 chipX_center=-cy, chipY_center=+cx, chipZ_center=-cz
            # 16.384 = 16384 counts/Gauss ÷ 1000 = counts/mG。
            new_ox = current_ox - cy_fit * 16.384   # 晶片 X ← −cy
            new_oy = current_oy + cx_fit * 16.384   # 晶片 Y ← +cx
            new_oz = current_oz - cz_fit * 16.384   # 晶片 Z ← −cz
            
            self.lbl_fit_results.config(
                text=f"3D 球面擬合結果:\n"
                     f"  球心偏移 (mG):\n    cx={cx_fit:.1f}, cy={cy_fit:.1f}, cz={cz_fit:.1f}\n"
                     f"  球體半徑 (mG): R={R_fit:.1f}\n\n"
                     f"建議寫入偏置 (Counts):\n"
                     f"  X_offset = {int(round(new_ox))}\n"
                     f"  Y_offset = {int(round(new_oy))}\n"
                     f"  Z_offset = {int(round(new_oz))}",
                fg="#00d2ff"
            )
            
            self.draw_fit_sphere(cx_fit, cy_fit, cz_fit, R_fit)
            
            self.new_ox = new_ox
            self.new_oy = new_oy
            self.new_oz = new_oz
            self.btn_write_calib.config(state=tk.NORMAL)
            
        except Exception as e:
            messagebox.showerror("錯誤", f"擬合計算失敗: {e}", parent=self.calib_win)

    def draw_fit_sphere(self, cx, cy, cz, R):
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 10)
        sphere_x = cx + R * np.outer(np.cos(u), np.sin(v))
        sphere_y = cy + R * np.outer(np.sin(u), np.sin(v))
        sphere_z = cz + R * np.outer(np.ones(np.size(u)), np.cos(v))
        
        self.calib_ax.clear()
        self.calib_ax.set_facecolor("#151515")
        
        self.calib_ax.scatter(self.calib_x, self.calib_y, self.calib_z, color="#00e5ff", s=10, alpha=0.7, label="收集數據")
        self.calib_ax.scatter([cx], [cy], [cz], color="red", s=50, label="擬合球心")
        self.calib_ax.plot_wireframe(sphere_x, sphere_y, sphere_z, color="#ff4444", alpha=0.3, linewidth=0.8, label="擬合球面")
        
        self.calib_ax.set_xlabel("Bx (mG)", color="#aaaaaa")
        self.calib_ax.set_ylabel("By (mG)", color="#aaaaaa")
        self.calib_ax.set_zlabel("Bz (mG)", color="#aaaaaa")
        self.calib_ax.tick_params(colors="#aaaaaa")
        self.calib_ax.legend(facecolor="#222222", edgecolor="none", labelcolor="#ffffff")
        self.calib_canvas.draw()

    def write_mag_calibration(self):
        if not hasattr(self, 'new_ox') or not hasattr(self, 'new_oy') or not hasattr(self, 'new_oz'):
            return

        # ★ 先關掉 50Hz 高頻 [MAG] 印刷，否則 CDC TX 被洗版，寫入回應會被 _write() 丟棄。
        #   STOP 經逐字慢送(~2s) → 板端處理後 [MAG] 恢復 1Hz → CDC TX 通道清空。
        #   延遲 3s 後再送寫入命令，確保 STOP 已完整送達並生效。
        self.send_command("CMD_MAG_CAL_STOP")
        self.lbl_board_offsets.config(
            text="正在停止高頻採集…請稍候",
            fg="#ffcc00"
        )
        self.root.after(3000, self._do_mag_write)

    def _do_mag_write(self):
        """延遲送出 CMD_MAG_CAL 寫入命令（確保 50Hz 印刷已停止、CDC TX 通道清空）。"""
        if not hasattr(self, 'new_ox') or not hasattr(self, 'new_oy') or not hasattr(self, 'new_oz'):
            return
        # 韌體 CMD_MAG_CAL 只接受「整數 raw ADC counts」（main.c sscanf %ld；
        # nano.specs 不支援 %f，送浮點會被拒收）。故一律取整數。
        ox, oy, oz = int(round(self.new_ox)), int(round(self.new_oy)), int(round(self.new_oz))
        cmd = f"CMD_MAG_CAL:{ox},{oy},{oz}"
        if self.send_command(cmd):
            self._pending_mag_write = (ox, oy, oz)
            self.lbl_board_offsets.config(
                text=f"當前板載偏置 (Counts):\n  X={ox}, Y={oy}, Z={oz}  ⏳ 等待韌體寫入確認…",
                fg="#ffcc00"
            )
            self._arm_mag_write_timeout()

    def toggle_mag_yaw_lock(self, enable):
        val = 1 if enable else 0
        cmd = f"CMD_MAG_YAW_LOCK:{val}"
        if self.send_command(cmd):
            # 不樂觀報成功：等韌體 [CAL] EKF Mag Yaw Lock set to: N 回傳再確認。
            self._pending_yaw_lock = enable
            if hasattr(self, 'lbl_yaw_lock') and self.lbl_yaw_lock.winfo_exists():
                status = "啟用" if enable else "停用"
                self.lbl_yaw_lock.config(text=f"磁北鎖定：⏳ 等待韌體確認{status}…", fg="#ffcc00")

    def _on_yaw_lock_confirmed(self, locked):
        """收到韌體 [CAL] EKF Mag Yaw Lock set to: N 確認：更新狀態顯示。"""
        if hasattr(self, 'lbl_yaw_lock') and self.lbl_yaw_lock.winfo_exists():
            if locked:
                self.lbl_yaw_lock.config(text="磁北鎖定：🔒 開（已確認）", fg="#28a745")
            else:
                self.lbl_yaw_lock.config(text="磁北鎖定：🔓 關（已確認）", fg="#ff9500")
        self.console.insert(tk.END, f"[CAL] ✅ 磁北鎖定已{'開啟' if locked else '關閉'}\n", "ok")
        self.console.see(tk.END)
        self._pending_yaw_lock = None

    def reset_mag_calibration(self):
        if messagebox.askyesno("確認", "是否確定要重置地磁偏置至預設值 (131072)？", parent=self.calib_win):
            # ★ 同 write_mag_calibration：先關 50Hz 印刷再送寫入命令。
            self.send_command("CMD_MAG_CAL_STOP")
            self.lbl_board_offsets.config(
                text="正在停止高頻採集…請稍候",
                fg="#ffcc00"
            )
            self.root.after(3000, self._do_mag_reset)

    def _do_mag_reset(self):
        """延遲送出重置命令（確保 50Hz 印刷已停止、CDC TX 通道清空）。"""
        cmd = "CMD_MAG_CAL:131072,131072,131072"
        if self.send_command(cmd):
            self._pending_mag_write = (131072, 131072, 131072)
            self.clear_calib_data()
            self.lbl_board_offsets.config(
                text="當前板載偏置 (Counts):\n  X=131072, Y=131072, Z=131072  ⏳ 等待韌體寫入確認…",
                fg="#ffcc00"
            )
            self._arm_mag_write_timeout()

    # ------------------ Flash 紀錄管理對話視窗 ------------------
    def open_flash_panel(self):
        """開啟 Flash 紀錄管理視窗（清空與 CSV 格式化導出）。"""
        if hasattr(self, 'flash_win') and self.flash_win and self.flash_win.winfo_exists():
            self.flash_win.lift()
            return

        self.flash_win = tk.Toplevel(self.root)
        self.flash_win.title("💾 W25Q128 Flash 記憶體紀錄管理")
        self.flash_win.geometry("560x420")
        self.flash_win.configure(bg="#1c1c1c")

        # 頂部標題列
        title_frame = tk.Frame(self.flash_win, bg="#2a2a2a", padx=15, pady=10)
        title_frame.pack(fill=tk.X)
        tk.Label(title_frame, text="💾 W25Q128 Flash 記憶體管理", bg="#2a2a2a", fg="#00d2ff",
                 font=("Helvetica", 14, "bold")).pack(anchor="w")
        tk.Label(title_frame, text="管理主航電板 16MB SPI Flash 飛行數據封包、遠端清空與格式化 CSV 導出",
                 bg="#2a2a2a", fg="#aaaaaa", font=("Helvetica", 9)).pack(anchor="w", pady=(2, 0))

        # 主內容區域
        main_frame = tk.Frame(self.flash_win, bg="#1c1c1c", padx=15, pady=15)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 區塊 1: 記憶體狀態資訊
        status_box = tk.LabelFrame(main_frame, text=" 📊 當前 Flash 狀態 ", bg="#1c1c1c", fg="#00d2ff",
                                   font=("Helvetica", 10, "bold"), padx=12, pady=10)
        status_box.pack(fill=tk.X, pady=(0, 15))

        current_pkts = self.cards["flash_pkt"].cget("text") if "flash_pkt" in self.cards else "未知"
        self.lbl_flash_info = tk.Label(status_box, text=f"• 容量：16 MB (W25Q128JV)\n• 飛行紀錄：{current_pkts}\n• 環形緩衝區：0x010000 - 0xFFFFFF",
                                       bg="#1c1c1c", fg="#cccccc", font=("Monaco", 9), justify=tk.LEFT)
        self.lbl_flash_info.pack(anchor="w")

        # 區塊 2: 匯出與擦除操作按鈕
        op_box = tk.LabelFrame(main_frame, text=" ⚡ 操作控制 ", bg="#1c1c1c", fg="#00d2ff",
                               font=("Helvetica", 10, "bold"), padx=12, pady=12)
        op_box.pack(fill=tk.BOTH, expand=True)

        # A. 匯出 CSV 按鈕與說明
        exp_frame = tk.Frame(op_box, bg="#1c1c1c")
        exp_frame.pack(fill=tk.X, pady=5)
        btn_exp = StyledButton(exp_frame, text="📥 匯出 Flash 資料夾", command=self.export_flash_csv,
                               bg="#006699", hover_bg="#0088cc", fg="#ffffff",
                               font=("Helvetica", 10, "bold"), padx=14, pady=7)
        btn_exp.pack(side=tk.LEFT, padx=(0, 10))
        tk.Label(exp_frame, text="完整 Ring Buffer、旗標區，並依 flight_id 自動分 CSV", bg="#1c1c1c", fg="#aaaaaa",
                 font=("Helvetica", 9)).pack(side=tk.LEFT)

        # B. 清空 Ring Buffer 飛行紀錄按鈕（保留校準）
        erase_frame = tk.Frame(op_box, bg="#1c1c1c")
        erase_frame.pack(fill=tk.X, pady=12)
        btn_erase = StyledButton(erase_frame, text="🔴 清空飛行紀錄", command=self.erase_flash_ring,
                                bg="#b71c1c", hover_bg="#d32f2f", fg="#ffffff",
                                font=("Helvetica", 10, "bold"), padx=14, pady=7)
        btn_erase.pack(side=tk.LEFT, padx=(0, 10))
        tk.Label(erase_frame, text="只清 Ring Buffer 飛行紀錄，保留校準/mag/LoRa 參數 (需二次確認)",
                 bg="#1c1c1c", fg="#ff8888", font=("Helvetica", 9)).pack(side=tk.LEFT)

        # B2. 清空整顆 Flash 按鈕（含校準，危險）
        erase_all_frame = tk.Frame(op_box, bg="#1c1c1c")
        erase_all_frame.pack(fill=tk.X, pady=(0, 8))
        btn_erase_all = StyledButton(erase_all_frame, text="☠️ 清空全部 (含校準)", command=self.erase_flash_all,
                                bg="#7a0000", hover_bg="#a30000", fg="#ffffff",
                                font=("Helvetica", 10, "bold"), padx=14, pady=7)
        btn_erase_all.pack(side=tk.LEFT, padx=(0, 10))
        tk.Label(erase_all_frame, text="連同校準/mag/LoRa/總結整顆清空，之後須重新校正 (需二次確認)",
                 bg="#1c1c1c", fg="#ff5555", font=("Helvetica", 9)).pack(side=tk.LEFT)

        # B3. ★2026-07-31：快速填池（航電開機不再自動擦除，池未達標會擋 ARM）
        pool_frame = tk.Frame(op_box, bg="#1c1c1c")
        pool_frame.pack(fill=tk.X, pady=(0, 8))
        btn_pool = StyledButton(pool_frame, text="⚡ 快速填池", command=self.topup_flash_pool,
                                bg="#005a5a", hover_bg="#008080", fg="#ffffff",
                                font=("Helvetica", 10, "bold"), padx=14, pady=7)
        btn_pool.pack(side=tk.LEFT, padx=(0, 10))
        tk.Label(pool_frame, text="只把已擦池補到 ARM 門檻，不動整環 (bench 省時用；飛前仍應走「清空飛行紀錄」)",
                 bg="#1c1c1c", fg="#00cccc", font=("Helvetica", 9)).pack(side=tk.LEFT)

        # C. 狀態提示標籤
        self.lbl_flash_export_status = tk.Label(op_box, text="就緒。請選擇欲執行的操作。", bg="#1c1c1c", fg="#888888",
                                               font=("Monaco", 9, "bold"))
        self.lbl_flash_export_status.pack(anchor="w", pady=(10, 0))

    def topup_flash_pool(self):
        """★2026-07-31：快速填池（`flash pool`）。航電開機不再自動擦除，池未達
        FLASH_RING_PREERASE_TARGET 時 ARM 會被擋；本指令只補不足的部分、不動整環。
        不需二次確認：它只擦「本來就沒資料」的區域，不會毀掉既有飛行紀錄。"""
        if not self.running or not getattr(self, 'ser', None):
            messagebox.showwarning("警告", "串口未連接！無法發送填池命令。", parent=getattr(self, 'flash_win', None))
            return
        if self.send_command("flash pool"):
            if hasattr(self, 'lbl_flash_export_status') and self.lbl_flash_export_status.winfo_exists():
                self.lbl_flash_export_status.config(text="⏳ 正在填池中（只補不足部分）...", fg="#00cccc")

    def erase_flash_ring(self):
        """只清 Ring Buffer 飛行紀錄，保留 Sector 0 校準/mag/LoRa 參數與任務總結。"""
        if not self.running or not getattr(self, 'ser', None):
            messagebox.showwarning("警告", "串口未連接！無法發送清空命令。", parent=getattr(self, 'flash_win', None))
            return
        if messagebox.askyesno("⚠️ 危險操作確認",
                               "是否確定要清空 Ring Buffer 飛行紀錄？\n"
                               "（保留校準/mag/LoRa 參數）\n此操作無法復原！",
                               parent=getattr(self, 'flash_win', None)):
            if self.send_command("flash erase ring"):
                if hasattr(self, 'lbl_flash_export_status') and self.lbl_flash_export_status.winfo_exists():
                    self.lbl_flash_export_status.config(text="⏳ 正在清空飛行紀錄中 (耗時約 30-60 秒)...", fg="#ffcc00")

    def erase_flash_all(self):
        """清空整顆 Flash（含 Sector 0 校準/mag/LoRa 參數與任務總結）。"""
        if not self.running or not getattr(self, 'ser', None):
            messagebox.showwarning("警告", "串口未連接！無法發送清空命令。", parent=getattr(self, 'flash_win', None))
            return
        if messagebox.askyesno("☠️ 高危操作確認",
                               "是否確定要清空『整顆』Flash？\n"
                               "將一併抹除加速度計/陀螺儀/磁力計校準與 LoRa 參數，\n"
                               "下次開機前務必重新校正！\n此操作無法復原！",
                               parent=getattr(self, 'flash_win', None)):
            if self.send_command("flash erase all"):
                if hasattr(self, 'lbl_flash_export_status') and self.lbl_flash_export_status.winfo_exists():
                    self.lbl_flash_export_status.config(text="⏳ 正在清空整顆 Flash 中 (耗時約 30-60 秒)...", fg="#ffcc00")

    def export_flash_csv(self):
        """選擇資料夾並啟動 Flash 結構化匯出。"""
        if not self.running or not getattr(self, 'ser', None):
            messagebox.showwarning("警告", "串口未連接！無法進行 Flash 導出。", parent=getattr(self, 'flash_win', None))
            return

        parent_dir = filedialog.askdirectory(
            parent=getattr(self, 'flash_win', None),
            title="選擇 Flash 匯出資料夾"
        )
        if not parent_dir:
            return

        try:
            now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            export_dir = os.path.join(parent_dir, f"flash_export_{now_str}")
            raw_dir = os.path.join(export_dir, "raw")
            processed_dir = os.path.join(export_dir, "processed")
            os.makedirs(export_dir, exist_ok=False)
            os.makedirs(raw_dir, exist_ok=False)
            os.makedirs(processed_dir, exist_ok=False)
            filepath = os.path.join(raw_dir, "ring_buffer_all.csv")

            f = open(filepath, "w", encoding="utf-8", newline="")
            self._flash_export_dir = export_dir
            self._flash_export_raw_dir = raw_dir
            self._flash_export_processed_dir = processed_dir
            self._flash_export_started_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self._flash_export_file = f
            self._flash_export_writer = csv.writer(f, lineterminator="\n")
            self._flash_export_sysflags_file = None
            self._flash_export_flight_files = {}
            self._flash_export_flight_writers = {}
            self._flash_export_flight_counts = {}
            self._flash_export_mode = None
            self._flash_export_active = True
            self._flash_export_writing = False
            self._flash_export_count = 0
            self._flash_export_skipped = 0
            self._write_flash_export_info()
            self._set_flash_export_status(f"📥 正在匯出到 {os.path.basename(export_dir)}...", "#00d2ff")
            if not self.send_command("flash export"):
                self._finish_flash_export(ok=False, message="⚠️ 匯出未開始：指令送出失敗")
        except Exception as e:
            self._close_flash_export_file()
            messagebox.showerror("錯誤", f"無法建立匯出資料夾: {e}", parent=getattr(self, 'flash_win', None))

    # ------------------ 地磁偏置寫入確認（由 poll_queue 序列回呼，主執行緒） ------------------
    # ⚠ 這些在序列處理迴圈中被呼叫，絕不可跳 modal messagebox：校正視窗持有 grab_set()，
    #   對話框會被蓋在後面又搶不到輸入 → 整個 GUI 死鎖。一律只更新非阻塞標籤。
    #   原始 [CAL] 行本身已由 poll_queue 分流顯示在「重要訊息」區，不需再彈窗。
    def _on_mag_write_confirmed(self):
        ox, oy, oz = (int(round(v)) for v in self.board_mag_offsets)
        if hasattr(self, 'lbl_board_offsets') and self.lbl_board_offsets.winfo_exists():
            self.lbl_board_offsets.config(
                text=f"當前板載偏置 (Counts):\n  X={ox}, Y={oy}, Z={oz}  ✅ 已寫入 Flash",
                fg="#28a745"
            )
        self._pending_mag_write = None

    def _on_mag_write_failed(self, line):
        if hasattr(self, 'lbl_board_offsets') and self.lbl_board_offsets.winfo_exists():
            self.lbl_board_offsets.config(text="⚠ 韌體回報寫入失敗，詳見重要訊息區", fg="#dc3545")
        self._pending_mag_write = None

    def _arm_mag_write_timeout(self):
        """3 秒未收到 [CAL] 寫入確認就提示（板子韌體可能較舊/指令被拒），不再永遠卡在 ⏳。"""
        self._mag_write_token = getattr(self, '_mag_write_token', 0) + 1
        tok = self._mag_write_token
        try:
            # 8 秒逾時：逐字慢送(~3.5s) + Flash 擦除(~300ms) + CDC TX 排隊出清
            self.root.after(8000, lambda: self._check_mag_write_timeout(tok))
        except Exception:
            pass

    def _check_mag_write_timeout(self, tok):
        if tok == getattr(self, '_mag_write_token', 0) and getattr(self, '_pending_mag_write', None) is not None:
            self._pending_mag_write = None
            if hasattr(self, 'lbl_board_offsets') and self.lbl_board_offsets.winfo_exists():
                self.lbl_board_offsets.config(
                    text="⚠ 未收到韌體寫入確認\n（板子韌體可能較舊，或指令被拒，見重要訊息區）",
                    fg="#dc3545")

    def update_calib_plot(self):
        if not hasattr(self, 'calib_win') or not self.calib_win.winfo_exists():
            return
            
        if self.collecting_data and len(self.calib_x) > 0:
            self.calib_ax.clear()
            self.calib_ax.set_facecolor("#151515")
            self.calib_ax.scatter(self.calib_x, self.calib_y, self.calib_z, color="#00e5ff", s=10, alpha=0.7, label="收集數據")
            
            self.calib_ax.set_xlabel("Bx (mG)", color="#aaaaaa")
            self.calib_ax.set_ylabel("By (mG)", color="#aaaaaa")
            self.calib_ax.set_zlabel("Bz (mG)", color="#aaaaaa")
            self.calib_ax.tick_params(colors="#aaaaaa")
            self.calib_canvas.draw_idle()
            
        if self.latest_mag:
            self.lbl_live_mag.config(
                text=f"即時強度: mx={self.latest_mag['mx']:.1f}, my={self.latest_mag['my']:.1f}, mz={self.latest_mag['mz']:.1f} mG\n"
                     f"即時航向: hdg={self.latest_mag['hdg']:.1f}°"
            )
            
        self.calib_win.after(200, self.update_calib_plot)

    # ==================== 軸向對齊測試精靈 ====================
    def open_test_wizard(self):
        if not self.running:
            messagebox.showwarning("提示", "請先開啟串口連接航電板，再進行一致性測試。")
            return
        if hasattr(self, 'wizard_win') and self.wizard_win and self.wizard_win.winfo_exists():
            self.wizard_win.lift()
            return

        # 建立彈出視窗
        self.wizard_win = tk.Toplevel(self.root)
        self.wizard_win.title("🧭 航電板感測器軸向一致性測試精靈")
        self.wizard_win.geometry("580x460")
        self.wizard_win.configure(bg="#1c1c1c")
        # 不用 transient()：見 open_lora_panel 註解，子視窗要能各自獨立縮小/關閉；
        # grab_set() 仍保留，測試精靈流程需要 modal 獨佔輸入焦點。
        self.wizard_win.grab_set()
        
        self.current_step_idx = 0
        self.is_detecting = False
        
        # 建立 UI 組件
        self.setup_wizard_ui()
        
        # 啟動即時數據更新 loop
        self.update_wizard_live_feedback()

    def setup_wizard_ui(self):
        # 標題
        self.lbl_step_title = tk.Label(self.wizard_win, text="", bg="#1c1c1c", fg="#00d2ff", font=("Helvetica", 12, "bold"))
        self.lbl_step_title.pack(fill=tk.X, side=tk.TOP, pady=12)
        
        # 動作提示框 (全息邊框風格)
        prompt_frame = tk.Frame(self.wizard_win, bg="#252525", borderwidth=1, relief="solid")
        prompt_frame.pack(fill=tk.BOTH, expand=False, padx=20, pady=8, ipady=5)
        
        self.lbl_prompt = tk.Label(prompt_frame, text="", bg="#252525", fg="#ffffff", font=("Helvetica", 10), justify=tk.LEFT, wraplength=520)
        self.lbl_prompt.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)
        
        # 即時數據反饋與檢測診斷框
        self.lbl_live_val = tk.Label(self.wizard_win, text="即時數據載入中...", bg="#101010", fg="#aaaaaa", font=("Monaco", 9), justify=tk.LEFT, anchor="nw", borderwidth=1, relief="sunken", height=8)
        self.lbl_live_val.pack(fill=tk.BOTH, expand=True, padx=20, pady=8)
        
        # 貼心狀態條
        self.lbl_status = tk.Label(self.wizard_win, text="等待開始...", bg="#1c1c1c", fg="#aaaaaa", font=("Helvetica", 10, "bold"))
        self.lbl_status.pack(fill=tk.X, side=tk.TOP, pady=8)
        
        # 控制按鈕區
        btn_frame = tk.Frame(self.wizard_win, bg="#1c1c1c")
        btn_frame.pack(fill=tk.X, side=tk.BOTTOM, pady=15, padx=20)
        
        self.btn_prev = ttk.Button(btn_frame, text="◀ 上一步", width=10, command=self.prev_wizard_step)
        self.btn_prev.pack(side=tk.LEFT, padx=5)
        
        self.btn_detect = ttk.Button(btn_frame, text="⚡ 開始檢測", width=12, command=self.run_wizard_detection)
        self.btn_detect.pack(side=tk.LEFT, padx=15)
        
        ttk.Button(btn_frame, text="關閉測試", width=10, command=self.wizard_win.destroy).pack(side=tk.RIGHT, padx=5)
        
        self.btn_next = ttk.Button(btn_frame, text="下一步 ▶", width=10, command=self.next_wizard_step)
        self.btn_next.pack(side=tk.RIGHT, padx=5)
        
        self.update_wizard_step()

    def update_wizard_step(self):
        step_info = TEST_STEPS[self.current_step_idx]
        self.lbl_step_title.config(text=step_info["title"])
        self.lbl_prompt.config(text=step_info["prompt"])
        self.lbl_status.config(text="等待檢測...", fg="#aaaaaa")
        
        # 設定按鈕可用性
        self.btn_prev.config(state=tk.DISABLED if self.current_step_idx == 0 else tk.NORMAL)
        self.btn_next.config(state=tk.DISABLED if self.current_step_idx == len(TEST_STEPS) - 1 else tk.NORMAL)

    def next_wizard_step(self):
        if self.current_step_idx < len(TEST_STEPS) - 1:
            self.current_step_idx += 1
            self.update_wizard_step()

    def prev_wizard_step(self):
        if self.current_step_idx > 0:
            self.current_step_idx -= 1
            self.update_wizard_step()

    def update_wizard_live_feedback(self):
        if not hasattr(self, 'wizard_win') or not self.wizard_win.winfo_exists():
            return
            
        step_info = TEST_STEPS[self.current_step_idx]
        sensor = step_info["sensor"]
        
        text = ""
        if not self.is_detecting:
            if sensor == "IMU" and self.latest_imu:
                text = (f"即時數據 (BMI088):\n"
                        f"  ax = {self.latest_imu['ax']:.1f} mG\n"
                        f"  ay = {self.latest_imu['ay']:.1f} mG\n"
                        f"  az = {self.latest_imu['az']:.1f} mG\n"
                        f"  gx = {self.latest_imu['gx']:.1f} dps\n"
                        f"  gy = {self.latest_imu['gy']:.1f} dps\n"
                        f"  gz = {self.latest_imu['gz']:.1f} dps")
            elif sensor == "HIGHG" and self.latest_highg:
                text = (f"即時數據 (ADXL375):\n"
                        f"  ax = {self.latest_highg['ax']:.1f} mG\n"
                        f"  ay = {self.latest_highg['ay']:.1f} mG\n"
                        f"  az = {self.latest_highg['az']:.1f} mG")
            elif sensor == "MAG" and self.latest_mag:
                text = (f"即時數據 (MMC5983):\n"
                        f"  mx = {self.latest_mag['mx']:.1f} mG\n"
                        f"  my = {self.latest_mag['my']:.1f} mG\n"
                        f"  mz = {self.latest_mag['mz']:.1f} mG\n"
                        f"  hdg = {self.latest_mag['hdg']:.1f}°")
            elif sensor == "BOTH" and self.latest_imu and self.latest_highg:
                text = (f"即時數據 (低G & 高G Z軸):\n"
                        f"  BMI088  az = {self.latest_imu['az']:.1f} mG\n"
                        f"  ADXL375 az = {self.latest_highg['az']:.1f} mG")
            else:
                text = "⏳ 等待資料串流中，請確認串口已連接且正輸出遙測..."
                
            try:
                if self.lbl_live_val.winfo_exists():
                    self.lbl_live_val.config(text=text, fg="#aaaaaa")
            except tk.TclError:
                pass
            
        try:
            if self.wizard_win.winfo_exists():
                self.wizard_win.after(100, self.update_wizard_live_feedback)
        except tk.TclError:
            pass

    def run_wizard_detection(self):
        if self.is_detecting:
            return
            
        step_info = TEST_STEPS[self.current_step_idx]
        self.is_detecting = True
        self.btn_detect.config(state=tk.DISABLED)
        self.lbl_status.config(text="🔍 正在檢測，請維持動作...", fg="#ffcc00")
        
        # 收集樣本
        samples = []
        duration = 1.0 if step_info["type"] == "static" else 2.0
        start_time = time.time()
        
        def sample_loop():
            if not hasattr(self, 'wizard_win') or not self.wizard_win.winfo_exists():
                self.is_detecting = False
                return
                
            elapsed = time.time() - start_time
            if elapsed < duration:
                # 採樣
                if step_info["type"] == "static":
                    if step_info["sensor"] == "IMU" and self.latest_imu:
                        samples.append(self.latest_imu.copy())
                    elif step_info["sensor"] == "HIGHG" and self.latest_highg:
                        samples.append(self.latest_highg.copy())
                    elif step_info["sensor"] == "MAG" and self.latest_mag:
                        samples.append(self.latest_mag.copy())
                    elif step_info["sensor"] == "BOTH" and self.latest_imu and self.latest_highg:
                        samples.append({"imu": self.latest_imu.copy(), "hg": self.latest_highg.copy()})
                else: # dynamic (peak recording)
                    axis = step_info["axis"]
                    if self.latest_imu:
                        samples.append(self.latest_imu[axis])
                        
                pct = int((elapsed / duration) * 100)
                self.lbl_status.config(text=f"⏳ 採樣中... {pct}%", fg="#ffcc00")
                self.root.after(50, sample_loop)
            else:
                self.evaluate_wizard_result(step_info, samples)
                
        sample_loop()

    def evaluate_wizard_result(self, step_info, samples):
        self.is_detecting = False
        self.btn_detect.config(state=tk.NORMAL)
        
        if not hasattr(self, 'wizard_win') or not self.wizard_win.winfo_exists():
            return
            
        if not samples:
            self.lbl_status.config(text="❌ [ERROR] 未收集到有效數據！", fg="#ff3b30")
            return
            
        if step_info["type"] == "static":
            if step_info["sensor"] == "BOTH":
                avg_imu_az = sum(s["imu"]["az"] for s in samples) / len(samples)
                avg_imu_ax = sum(s["imu"]["ax"] for s in samples) / len(samples)
                avg_imu_ay = sum(s["imu"]["ay"] for s in samples) / len(samples)
                avg_hg_az = sum(s["hg"]["az"] for s in samples) / len(samples)
                avg_hg_ax = sum(s["hg"]["ax"] for s in samples) / len(samples)
                avg_hg_ay = sum(s["hg"]["ay"] for s in samples) / len(samples)
                val = {
                    "imu": {"ax": avg_imu_ax, "ay": avg_imu_ay, "az": avg_imu_az},
                    "hg": {"ax": avg_hg_ax, "ay": avg_hg_ay, "az": avg_hg_az}
                }
            else:
                keys = samples[0].keys()
                val = {}
                for k in keys:
                    val[k] = sum(s[k] for s in samples) / len(samples)
                # 對於地磁計，直接平均 hdg 度數在 0°/360° 邊界（朝北）會產生嚴重的均值環繞錯誤
                # 改為先將 Cartesian 分量 mx, my 平均，再用 np.arctan2 重新計算均值 hdg
                if "mx" in val and "my" in val:
                    hdg_deg = float(np.degrees(np.arctan2(-val["mx"], val["my"])))
                    if hdg_deg < 0:
                        hdg_deg += 360.0
                    val["hdg"] = hdg_deg
                    
            ok = step_info["run"](val)
            msg = step_info["result_msg"](val, ok)
            
        else: # dynamic
            peak = max(samples, key=abs)
            ok = step_info["run"](peak)
            msg = step_info["result_msg"](peak, ok)
            
        if ok:
            self.lbl_status.config(text="✅ [PASS] 檢測通過！", fg="#28a745")
        else:
            self.lbl_status.config(text="❌ [FAIL] 檢測未通過！", fg="#ff3b30")
            
        self.lbl_live_val.config(text=msg, fg="#ffffff" if ok else "#ff4f4f")

# ==================== 測試項目資料結構 ====================
TEST_STEPS = [
    {
        "step": 1,
        "title": "【測試 1/9】BMI088 加速度計 Z 軸 (鼻錐朝上 / 靜態)",
        "prompt": "請將航電板【水平靜止放置於桌面，正面朝上】(Z 軸朝上，即鼻錐朝上)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "IMU",
        "run": lambda val: 800.0 <= val["az"] <= 1200.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：Z 軸加速度 = {val['az']:.1f} mG\n" + (
            "✅ [PASS] Z 軸方向正確（鼻錐朝上）！" if ok else
            "❌ [FAIL] Z 軸方向異常！(預期 +800 ~ +1200 mG)\n💡 若接近 -1000 mG：代表 Z 軸被反置，需要在 main.c 中將 az 取負。"
        )
    },
    {
        "step": 2,
        "title": "【測試 2/9】BMI088 加速度計 X 軸 (右側朝上)",
        "prompt": "請將航電板【右側抬高約 45 度】(右緣朝上，body 右軸朝天)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "IMU",
        "run": lambda val: val["ax"] > 200.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：X 軸加速度 = {val['ax']:.1f} mG\n" + (
            "✅ [PASS] 右側朝上時 ax 為正，X=右 對齊正確！" if ok else
            "❌ [FAIL] 右側朝上時 ax 不為正！\n💡 建議：晶片貼裝與 sensor_axis.h 表格不符，請核對 IMU 映射 (X->Y)。"
        )
    },
    {
        "step": 3,
        "title": "【測試 3/9】BMI088 加速度計 Y 軸 (前緣朝上)",
        "prompt": "請將航電板【前緣抬高約 45 度】(前緣朝上，body 前軸朝天)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "IMU",
        "run": lambda val: val["ay"] > 200.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：Y 軸加速度 = {val['ay']:.1f} mG\n" + (
            "✅ [PASS] 前緣朝上時 ay 為正，Y=前 對齊正確！" if ok else
            "❌ [FAIL] 前緣朝上時 ay 不為正！\n💡 建議：晶片貼裝與 sensor_axis.h 表格不符，請核對 IMU 映射 (Y->X)。"
        )
    },
    {
        "step": 4,
        "title": "【測試 4/9】ADXL375 高 G 加速度計軸向對齊 (靜態)",
        "prompt": "請將航電板再次【水平靜置放於桌面，正面朝上】。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "BOTH",
        "run": lambda val: val["hg"]["az"] > 300.0,
        "result_msg": lambda val, ok: (
            f"📊 BMI088 (低G) 重力分量: [{val['imu']['ax']:.1f}, {val['imu']['ay']:.1f}, {val['imu']['az']:.1f}] mG\n"
            f"📊 ADXL375 (高G) 重力分量: [{val['hg']['ax']:.1f}, {val['hg']['ay']:.1f}, {val['hg']['az']:.1f}] mG\n"
            + ("✅ [PASS] ADXL375 Z 軸朝上，大於 +300 mG！" if ok else
               "❌ [FAIL] ADXL375 Z 軸讀值不足（< 300 mG）！\n💡 建議：在 main.c 高G替換邏輯中將 az 改為 +raw_adxl_az（去掉負號）。")
        )
    },
    {
        "step": 5,
        "title": "【測試 5/9】BMI088 陀螺儀 Z 軸 (Yaw/自旋)",
        "prompt": "請準備將航電板在水平桌面上【快速逆時針旋轉】。\n按下「開始檢測」後，請【立即開始逆時針快速旋轉約 2 秒】...",
        "type": "dynamic",
        "sensor": "IMU",
        "axis": "gz",
        "run": lambda peak: peak > 25.0,
        "result_msg": lambda peak, ok: f"📊 偵測結果：最大 Z 軸角速度 = {peak:.1f} dps\n" + (
            "✅ [PASS] 逆時針旋轉輸出為正，Yaw 軸向符合右手定則！" if ok else
            "❌ [FAIL] 逆時針旋轉輸出為負！Yaw 旋轉方向相反。\n💡 建議：您需要在 main.c 將 gz 取負。"
        )
    },
    {
        "step": 6,
        "title": "【測試 6/9】BMI088 陀螺儀 X 軸 (Pitch/俯仰，繞右軸)",
        "prompt": "請準備將航電板【快速抬頭/後仰】(前端/前緣朝上抬起)。\n按下「開始檢測」後，請【立即快速將板子前半部朝上抬起旋轉約 2 秒】...",
        "type": "dynamic",
        "sensor": "IMU",
        "axis": "gx",
        "run": lambda peak: peak > 25.0,
        "result_msg": lambda peak, ok: f"📊 偵測結果：最大 X 軸角速度 = {peak:.1f} dps\n" + (
            "✅ [PASS] 抬頭(後仰)旋轉為正，Pitch 繞 X(右)軸符合右手定則！" if ok else
            "❌ [FAIL] 抬頭旋轉為負！Pitch 旋轉方向相反。\n💡 建議：在 main.c 將 gx 來源 -raw_gy 改為 +raw_gy。"
        )
    },
    {
        "step": 7,
        "title": "【測試 7/9】BMI088 陀螺儀 Y 軸 (Roll/翻滾，繞前軸)",
        "prompt": "請準備將航電板【快速向右翻滾】(right side down)。\n按下「開始檢測」後，請【立即快速將板子向右翻滾】...",
        "type": "dynamic",
        "sensor": "IMU",
        "axis": "gy",
        "run": lambda peak: peak > 25.0,
        "result_msg": lambda peak, ok: f"📊 偵測結果：最大 Y 軸角速度 = {peak:.1f} dps\n" + (
            "✅ [PASS] 向右翻滾為正，Roll 繞 Y(前)軸符合右手定則！" if ok else
            "❌ [FAIL] 向右翻滾為負！Roll 旋轉方向相反。\n💡 建議：在 main.c 將 gy 來源 -raw_gx 改為 +raw_gx。"
        )
    },
    {
        "step": 8,
        "title": "【測試 8/9】MMC5983MA 地磁計航向角 (前緣朝北 ≈ 0°)",
        "prompt": "請將航電板水平靜置（Z軸朝天），並將【前緣/Y軸朝向正北方】。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "MAG",
        "run": lambda val: val["hdg"] <= 30.0 or val["hdg"] >= 330.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：地磁向量 B = [{val['mx']:.1f}, {val['my']:.1f}] mG, 航向角 hdg = {val['hdg']:.1f}°\n" + (
            f"✅ [PASS] 前緣朝北，航向角 {val['hdg']:.1f}° ≈ 0°，對齊正確！" if ok else
            f"❌ [FAIL] 前緣朝北，航向角 {val['hdg']:.1f}°，預期 ≈ 0°（±30°）。\n💡 建議：在 main.c 調整磁力計 mx_body/my_body 的來源軸。"
        )
    },
    {
        "step": 9,
        "title": "【測試 9/9】MMC5983MA 地磁計航向角收斂方向 (前緣朝東 ≈ 90°)",
        "prompt": "請將航電板水平靜置（Z軸朝天），並【順時針旋轉 90 度】(前緣朝向正東方)。\n置妥後，請按「開始檢測」...",
        "type": "static",
        "sensor": "MAG",
        "run": lambda val: 45.0 <= val["hdg"] <= 135.0,
        "result_msg": lambda val, ok: f"📊 偵測結果：地磁 Y 分量 my = {val['my']:.1f} mG, 航向角 hdg = {val['hdg']:.1f}°\n" + (
            f"✅ [PASS] 轉向東方航向角為 {val['hdg']:.1f}°，收斂方向正確！" if ok else
            f"❌ [FAIL] 轉向東方航向角為 {val['hdg']:.1f}°！預期應為 ~90°\n💡 建議：如果航向角往反方向收斂（例如變成 270°），您需要在 main.c 調整磁力計軸向或 heading 計算公式。"
        )
    }
]

# ==================== 5. 主入口 ====================
if __name__ == "__main__":
    # 在 macOS 上強制使用 TkAgg 後端以適應 Tkinter 視窗整合
    import matplotlib
    matplotlib.use("TkAgg")
    
    root = tk.Tk()
    app = RocketDashboardApp(root)
    # Tk 在 macOS 會自行安裝 C 層級 SIGINT/SIGTERM 處理器：收到訊號時直接在
    # 訊號情境中呼叫 Tcl_Exit() 摧毀所有視窗，此舉會觸發 <Destroy> binding
    # 回呼 Python；若訊號恰好中斷在 Python 執行到一半（例如 after() 排程的
    # 圖表重繪 callback 內），直譯器狀態尚未回到安全點就被重入，會踩中
    # PyEval_RestoreThread 的一致性檢查 → Fatal Python error → SIGABRT。
    # 此為終端機 Ctrl+C / VSCode 停止按鈕造成 GUI 監控器頻繁當機的成因。
    # 在 Tk() 建立後用 signal.signal() 覆蓋掉該 C handler，改走 Python 的
    # 安全遞延機制（於 bytecode 邊界執行），交給 on_close 做乾淨關閉。
    signal.signal(signal.SIGINT, lambda *_: root.after(0, app.on_close))
    signal.signal(signal.SIGTERM, lambda *_: root.after(0, app.on_close))
    root.mainloop()
