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
（見 firmware gs_lora_test.c: uplink_send()），不需要韌體改動。是否真的會發射 RF 由板子
自己的韌體變體決定（make flash-ground 為 RX-only、一律拒絕上行；make flash-ground-tx
才會真的發射，見 gs_lora_test.c: gs_tx_allowed()）；GUI 只顯示查詢到的 RX-only/TX-capable
徽章，不再另外用一道「已確認未安裝 LNA」的軟體鎖攔指令。
"""

import argparse
import csv
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
from matplotlib.figure import Figure   # 不用 plt.figure()：避免登記進 pyplot 全域 Gcf manager 造成漏 Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter.scrolledtext import ScrolledText

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import serial_link

import gui_theme as gt
from gui_attitude3d import Attitude3DView
from gui_map import MapView

DEFAULT_BAUD = serial_link.DEFAULT_BAUD
CHART_HIST_SEC = 60.0
LINK_HIST_LEN = 60

# ==================== 鏈路健康度：有效封包頻率／間隔健康度 ====================
# ★2026-07-30：原本用 seq gap（seq - last_seq，湊出「丟包率(%)」）已拿掉——與
# ground_station_analyzer.py 當天稍早拿掉「到達率丟失」同一個根因：兩鏈路的 seq
# 是同一顆計數器共用（main.c Telemetry_Build 每 100ms 建一次包），433 空中時間
# 限流、上行接收窗（每 3 秒靜默 800ms）都會讓 seq 出現「跳號」，但那是設計上本來
# 就不會發送、不是真的收不到；拿它當丟包率的分母（「本來該收幾包」）本身就是用
# 猜的，算出來的百分比不可信（analyzer.py 因此已整個拿掉，見該檔 gap_limits 註解）。
# 改成只依賴「實際收到什麼」的量測值：有效封包頻率（視窗內實測 Hz）＋封包間隔
# 是否出現長空窗，兩者都不需要假設火箭端排程。門檻／名目速率數值與
# ground_station_analyzer.py NOMINAL_RATE_HZ / SPEC_LIMITS 同步（估計值，非實測，
# 見該檔 LORA433_AIRTIME_S 註解），避免同一個系統的兩個工具給出不一致的判定。
LORA433_AIRTIME_S = 0.580
UPLINK_LISTEN_DUTY = 800.0 / 3000.0
NOMINAL_RATE_HZ = {
    "433": (1.0 / LORA433_AIRTIME_S) * (1.0 - UPLINK_LISTEN_DUTY),  # ≈1.26 Hz(估計)
    "920": 10.0,
}
GAP_MEAN_WARN_MULT, GAP_MEAN_FAIL_MULT = 1.67, 3.33
GAP_MAX_WARN_MULT, GAP_MAX_FAIL_MULT = 4.0, 8.0
GAP_MAX_WARN_FLOOR_MS, GAP_MAX_FAIL_FLOOR_MS = 1000.0, 2000.0
LINK_RATE_WINDOW_S = 8.0   # 有效封包頻率/間隔健康度的滑動採樣視窗

# drogue_alt_m / main_alt_m 的「尚未開傘」哨兵（與 firmware telemetry.h TELEM_DEPLOY_ALT_NA
# 及 telemetry_decoder.py 同名常數一致）。刻意不用 0：0 m 是合法的開傘高度（地面誤觸發、
# 發射台高度誤判），用 0 當「沒開」會把最該警示的事件偽裝成正常。
TELEM_DEPLOY_ALT_NA = -32768

# ==================== Flash 結構化匯出：CSV 欄位表（逐字取自 gs_log.c GsLog_CsvHeader，
# 順序/欄位須與 firmware `flash export` 輸出同步） ====================
GS_FLASH_EXPORT_COLUMNS = [
    "rx_utc", "aligned_utc", "link_mhz", "rssi_dbm", "snr_cb", "offset_ms",
    "seq", "fsm_state", "rkt_tick_ms", "ekf_alt_cm", "ekf_vel_cms", "baro_alt_cm",
    "vf_alt_cm", "vf_vel_cms", "max_alt_m", "max_vel_ms", "max_acc_cg",
    "drogue_alt_m", "main_alt_m",
    "gps_lat_1e6", "gps_lon_1e6", "gps_alt_m", "gps_sats", "gps_fix", "bat_mv",
    "flags", "health", "sensor",
    "peer_fsm", "peer_flags", "peer_baro_cm", "peer_link",
    "peer_vf_h_cm", "peer_vf_v_cms", "peer_bench_arb",
    "gs_lat_1e6", "gs_lon_1e6", "gs_alt_m", "gs_sats", "gs_fix",
    "rx_utc_ms", "aligned_utc_ms",
]
# 前兩欄是人讀時間戳 "HH:MM:SS.mmm"；其餘欄位不是十進位整數就是 0x%02X 十六進位位元組
# （flags/health/sensor/peer_flags/peer_link），故驗證用兩種 pattern 並存，不像航電那版
# 有固定的 addr 十六進位開頭欄可鎖。
GS_FLASH_EXPORT_HMS_RE = re.compile(r"^\d{2}:\d{2}:\d{2}\.\d{3}$")
GS_FLASH_EXPORT_CELL_RE = re.compile(r"^(-?\d+|0x[0-9A-Fa-f]{2})$")
GS_FLASH_SYSFLAGS_HEX_RE = re.compile(r"^[0-9A-Fa-f]{6}:(?: [0-9A-Fa-f]{2}){16}$")


class GroundStationGUI:
    # ==================== LoRa 參數設定面板：可調範圍/選項表（逐字取自 gui_monitor.py） ====================
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
• 本機是地面站（GROUND）board：E80 為接收端；另有 stats 統計/e80 init/
  e80 rxstart/airtime 估算等工具（見「統計/工具」分頁）。

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

【改完不通了怎麼辦】
• E80：對兩端各下一次「查詢」核對六個參數逐項一致；仍不通 → 地面站按
  「e80 init」重新初始化再「e80 rxstart」。
• E22：重開機兩端（firmware 開機會強制寫回預設頻道 432MHz、功率 21dBm、
  空速 2.4k），即可回到已知良好狀態。
"""

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
            # ★滾動極值：火箭端以感測器全速率追蹤、每包重複攜帶（telemetry.h max_*）。
            # 下鏈約 2Hz 抓不到真頂點/峰值 G，這三個才是可信數字；舊韌體無此 token 時維持 0。
            "max_alt_m": 0, "max_vel_ms": 0, "max_acc_g": 0.0, "hg_mag_g": 0.0,
            # 開傘高度：TELEM_DEPLOY_ALT_NA(-32768) = 未開傘（0 m 是合法開傘高度，不可混用）
            "drogue_alt_m": TELEM_DEPLOY_ALT_NA, "main_alt_m": TELEM_DEPLOY_ALT_NA,
            "q": [1.0, 0.0, 0.0, 0.0],   # 姿態四元數 [qw,qx,qy,qz]（由 [GS_ATT] 解析；舊韌體無此行則維持預設）
            "peer_fsm": 0, "peer_flags": 0,
            "peer_baro_m": 0.0, "peer_vf_h_m": 0.0, "peer_vf_v_ms": 0.0,
            "peer_link": 0, "peer_bench_arb": 0,
            "last_pkt_time": None,
        }
        self._pkt_times = {"433": deque(), "920": deque()}
        self._link_rate_hz = {"433": 0.0, "920": 0.0}
        self._link_gap_ms = {"433": (0.0, 0.0), "920": (0.0, 0.0)}  # (mean, max)

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
        self.att_dirty = False   # 姿態四元數有新資料（[GS_ATT]）；由 attitude_redraw_loop 節流消費

        # LoRa 現值（[E22]/[E80] 回報行解析結果，供 LoRa 面板「目前設定」顯示）
        self.lora_e22_state = {}
        self.lora_e80_state = {}

        # 地圖 / 3D 姿態元件（gui_map.py / gui_attitude3d.py）：在 _build_right_pane() 建構
        self.map_view = None
        self.att_view = None
        self.chart_win = None   # 圖表獨立視窗（open_chart_window()），預設不開
        self.lora_win = None    # LoRa 參數設定面板（open_lora_panel()）
        self.flash_win = None   # Flash 接收紀錄管理面板（open_flash_panel()）

        _N = 4000
        self.ts_alt_ekf = deque(maxlen=_N)
        self.ts_alt_vf = deque(maxlen=_N)
        self.ts_alt_baro = deque(maxlen=_N)
        self.ts_vz_ekf = deque(maxlen=_N)
        self.ts_vz_vf = deque(maxlen=_N)
        self.ts_peer_vf_alt = deque(maxlen=_N)
        self.ts_peer_baro = deque(maxlen=_N)
        self.ts_peer_vf_vz = deque(maxlen=_N)

        self.cmd_sender = gt.CommandSender(
            get_ser=lambda: self.ser,
            get_running=lambda: self.running,
            console_echo=self.append_console,
            event_log=self._event_log,
        )

        gt.setup_styles()
        self.build_ui()
        self.scan_ports()
        self.root.after(10, self.poll_queue)
        self.root.after(200, self.charts_redraw_loop)
        self.root.after(300, self.attitude_redraw_loop)
        self.root.after(1000, self.map_redraw_loop)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------------
    def send_command(self, cmd_str):
        ok = self.cmd_sender.send(cmd_str, parent=self.root)
        return ok

    def local_send(self, cmd_str):
        """本機 Flash 操作入口（flash erase/dump/export 等）：純粹操作本機 Flash，
        不牽涉 433/920 發射，見 send_command。"""
        return self.send_command(cmd_str)

    # ------------------------------------------------------------------ UI
    def build_ui(self):
        # 電梯測試 profile 醒目橫幅：預設隱藏，收到 [GS_PKT] prof: 位非 0 才顯示（見 parse_gs_pkt）。
        self.elevator_banner = gt.make_elevator_banner(self.root)

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

        # ── 第二排：TX 狀態徽章 + ARM + BENCH + 連線 ──
        bot_bar = tk.Frame(top_container, bg=gt.BG_ROOT)
        bot_bar.pack(fill=tk.X, side=tk.TOP, pady=(2, 4))

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
            # 這兩張就是「航電認為自己相對起點多高」——韌體端已扣過 pad_ref（發射台氣壓
            # 零點，PAD 期每 30s 重零、ARM 後凍結），下鏈 baro_alt_cm 反而是絕對海拔。
            ("EKF 高度(相對起點)", "alt", "-- m", gt.GREEN),
            ("VF 高度(相對起點)", "vf_alt", "-- m", gt.AMBER),
            ("EKF Vz", "vz", "-- m/s", gt.GREEN),
            ("VF Vz", "vf_vz", "-- m/s", gt.AMBER),
            ("加速度 |a|", "acc", "-- g", gt.GREEN),
            # ★滾動極值：火箭端以感測器全速率追蹤、每包重複攜帶（telemetry.h max_*）。
            # 下鏈只有約 2Hz，上面那幾張「當下值」卡永遠抓不到真正的頂點與峰值 G——
            # 這三張才是「到底飛多高／多快／幾 G」的可信數字，且抗丟包（頂點後任何一包
            # 穿透就拿得到），火箭無法回收時更是唯一來源。用紫色與當下值明顯區隔。
            ("★最高高度", "max_alt", "-- m", gt.PURPLE),
            ("★最大 Vz", "max_vel", "-- m/s", gt.PURPLE),
            ("★最大 |a|", "max_acc", "-- g", gt.PURPLE),
            ("★副傘開傘高度", "dalt", "-- m", gt.CYAN_HI),
            ("★主傘開傘高度", "malt", "-- m", gt.CYAN_HI),
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
        # 三個獨立視窗觸發鈕，配色比照 gui_monitor.py 的 console_tools 讓兩支 GUI 視覺一致
        gt.StyledButton(tools, text="📈 飛行圖表", command=self.open_chart_window,
                         bg="#00435a", hover_bg="#005f7f", padx=10, pady=4).pack(side=tk.LEFT, padx=3)
        gt.StyledButton(tools, text="📡 LoRa 參數設定", command=self.open_lora_panel,
                         bg="#00435a", hover_bg="#005f7f", padx=10, pady=4).pack(side=tk.LEFT, padx=3)
        gt.StyledButton(tools, text="💾 接收紀錄管理", command=self.open_flash_panel,
                         bg="#005577", hover_bg="#007799", padx=10, pady=4).pack(side=tk.LEFT, padx=3)

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
            "elevator_warn": gt.ELEVATOR_WARN_TAG_CFG,
        })

    def _build_right_pane(self, right_pane):
        # --- 上：GPS 即時地圖（左）+ 3D 姿態（右）——「目前位置/姿態」的即時視覺化，
        # 取代原本放在這裡的圖表分頁（歷史時序圖表移到獨立視窗，見 open_chart_window()，
        # 方便接第二顆螢幕；此處保留給「當下狀態」用的地圖與姿態）。 ---
        topo = tk.PanedWindow(right_pane, orient=tk.HORIZONTAL, bg=gt.BG_ROOT,
                              sashwidth=6, sashrelief="flat", bd=0)

        map_frame = tk.Frame(topo, bg=gt.BG_PANEL)
        gt.section_title(map_frame, "GPS 即時地圖")
        self.map_view = MapView(show_status_panel=False)
        self.map_view.build(map_frame).pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        topo.add(map_frame, width=380, minsize=240)

        att_frame = tk.Frame(topo, bg=gt.BG_PANEL)
        self.att_view = Attitude3DView(get_fsm_state=lambda: self.fsm_state or "--")
        self.att_view.build(att_frame).pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        topo.add(att_frame, minsize=260)

        right_pane.add(topo, height=430, minsize=280)

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
        self.chart_fig = Figure(facecolor=gt.BG_PANEL)
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
            self.ln_rssi[key], = self.ax_rssi.plot([], [], 'o', color=gt.LINK_COLORS[key], ms=3.5, label=f"{key}MHz")
            self.ln_snr[key], = self.ax_snr.plot([], [], 'o', color=gt.LINK_COLORS[key], ms=3.5, label=f"{key}MHz")
        for ax in (self.ax_rssi, self.ax_snr):
            gt.style_legend(ax, loc="upper right", ncol=1)

        self.chart_canvas = FigureCanvasTkAgg(self.chart_fig, master=parent)
        self.chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def build_alt_charts(self, parent):
        """高度（EKF/Baro/VF，主板 vs 副板）與速度（EKF Vz/VF Vz，主板 vs 副板）比對。
        線型比照 gui_monitor 慣例：實線=主板、虛線=副板。"""
        self.alt_fig = Figure(facecolor=gt.BG_PANEL)
        gs = self.alt_fig.add_gridspec(2, 1, hspace=0.4, left=0.11, right=0.97, top=0.90, bottom=0.14)
        self.ax_alt = self.alt_fig.add_subplot(gs[0])
        self.ax_vz = self.alt_fig.add_subplot(gs[1], sharex=self.ax_alt)
        for ax, ylab in ((self.ax_alt, "Alt (m)"), (self.ax_vz, "Vz (m/s)")):
            gt.style_axes(ax, ylab, facecolor=gt.BG_PANEL)
        self.ax_vz.set_xlabel("t (s)", color=gt.TXT_DIM, fontsize=9)

        # ⚠ 零點不一致：EKF/VF 是相對發射台（韌體扣過 pad_ref），Baro 兩條是**絕對海拔**
        # （telemetry.c 的 baro_alt_cm 填 baro_data.altitude，封包裡沒有 pad_ref 也沒有相對值）。
        # 兩者差一個發射台海拔的常數（場測 log 出現過 25 / 47.6 / 101 m）。USB 直連的
        # GUI_avionic / gui_monitor 能從 [FSM] pad_ref locked 文字行扣掉，走 LoRa 的本 GUI 不行，
        # 所以只在 legend 標明，不自己猜零點。
        # ★副板只剩 VF + Baro 兩條（下鏈封包 2026-07-30 起不再帶對端 EKF，見 telemetry.h）。
        self.ln_alt_ekf, = self.ax_alt.plot([], [], color=gt.CYAN_HI, lw=1.6, label="EKF 相對起點")
        self.ln_alt_baro, = self.ax_alt.plot([], [], color="#ff9800", lw=0.9, label="Baro 海拔(絕對)")
        self.ln_alt_vf, = self.ax_alt.plot([], [], color=gt.AMBER, lw=1.1, label="VF 相對起點")
        self.ln_alt_peer_baro, = self.ax_alt.plot([], [], color="#ff9800", lw=0.9, ls=":", alpha=0.7,
                                                   label="副板 Baro 海拔(絕對)")
        self.ln_alt_peer_vf, = self.ax_alt.plot([], [], color=gt.GREEN, lw=1.3, ls="--",
                                                 label="副板 VF 相對起點")

        self.ln_vz_ekf, = self.ax_vz.plot([], [], color=gt.CYAN_HI, lw=1.6, label="EKF Vz")
        self.ln_vz_vf, = self.ax_vz.plot([], [], color=gt.AMBER, lw=1.1, label="VF Vz")
        self.ln_vz_peer_vf, = self.ax_vz.plot([], [], color=gt.GREEN, lw=1.3, ls="--", label="副板 VF Vz")

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
        self._console_line_count = 0

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

            # 定期截斷終端，防記憶體/版面計算隨行數線性變慢（長時間連線卡頓的主因之一）
            self._console_line_count = getattr(self, "_console_line_count", 0) + 1
            if self._console_line_count > 2000:
                self.console.delete("1.0", "200.0")
                self._console_line_count -= 200

            try:
                # Flash CSV 導出串流捕獲（比照 gui_monitor.py：無條件每行檢查，內部依
                # _flash_export_active 早退，未匯出時幾乎零成本；CSV_START/END/資料列
                # 本身沒有 [FLASH 前綴，不能只靠下面的 "[FLASH" elif 分支攔）
                self._handle_flash_export_stream_line(line)

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
                elif "[GS_ATT]" in line:
                    self.parse_gs_att(line)
                elif "[GS_STAT]" in line:
                    self.parse_gs_stat(line)
                elif "[GS_GPS]" in line:
                    self.parse_gs_gps(line)
                elif ("[E22]" in line or "[E80]" in line
                      or "[LORA 920MHz]" in line or "[LORA433]" in line):
                    self.parse_lora_params(line)
                elif "[FLASH" in line:
                    self._handle_flash_progress(line)
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
        # ★滾動極值 + 高G 模長：整段設為可選（(?:...)?），舊韌體沒有這些 token 時仍能解析
        # 前後欄位，不會整行比對失敗（比照下面 pbarb/prof 的既有做法）。
        r"(?:max:(?P<maxalt>\d+)m mvel:(?P<maxvel>-?\d+)ms macc:(?P<maxacc>\d+)cg "
        r"dalt:(?P<dalt>-?\d+)m malt:(?P<malt>-?\d+)m )?"
        r"gps:(?P<sats>\d+)/(?P<fix>\d+) pos:(?P<lat_s>[+-])(?P<lat>\d+\.\d+),(?P<lon_s>[+-])(?P<lon>\d+\.\d+) "
        r"galt:(?P<galt>-?\d+)m accel:(?P<ax>-?\d+),(?P<ay>-?\d+),(?P<az>-?\d+)"
        r"(?:\s+hg:(?P<hg>-?\d+)cg)? "
        r"peer:(?P<peer_fsm>\d+) pflags:0x(?P<pflags>[0-9A-Fa-f]+) "
        r"plink:0x(?P<plink>[0-9A-Fa-f]+) "
        r"pbaro:(?P<pbaro>-?\d+)cm pvfh:(?P<pvfh>-?\d+)cm pvfv:(?P<pvfv>-?\d+)cms"
        r"(?:\s+pbarb:(?P<pbarb>\d+))?"
        r"(?:\s+prof:0x(?P<prof>[0-9A-Fa-f]+))?")

    # profile_flags 位（與 telemetry.h TELEM_PROFILE_* 一致）：SELF=下鏈的這片板（主航電）
    # 仍為電梯測試 profile；PEER=經板間鏈路中繼的對端（副航電）仍為電梯測試 profile。
    TELEM_PROFILE_SELF_ELEVATOR = 0x01
    TELEM_PROFILE_PEER_ELEVATOR = 0x02

    def parse_gs_pkt(self, line):
        m = self._GS_PKT_RE.search(line)
        if not m:
            return
        g = m.groupdict()
        link = g["link"]
        t = self.now_t()

        self._update_link_rate(link, t)
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
        if g["hg"] is not None:
            self.rocket_state["hg_mag_g"] = int(g["hg"]) / 100.0

        self.cards["bat"].config(text=f"{bat_v:.2f} V", foreground=gt.battery_color(bat_v))
        self.cards["alt"].config(text=f"{alt:.1f} m")
        self.cards["vf_alt"].config(text=f"{vfh:.1f} m")
        self.cards["vz"].config(text=f"{vz:+.1f} m/s")
        self.cards["vf_vz"].config(text=f"{vfv:+.1f} m/s")
        self.cards["acc"].config(text=f"{acc_g:.2f} g")

        # ★滾動極值（火箭端全速率追蹤）：舊韌體沒有這些 token → 卡片維持 "--"，不寫 0
        # 假裝有資料。max_acc 標註 BMI088 ±24g 削頂上限，讀到 ~24g 代表「至少 24g」。
        if g["maxalt"] is not None:
            max_alt = int(g["maxalt"])
            max_vel = int(g["maxvel"])
            max_acc = int(g["maxacc"]) / 100.0
            self.rocket_state.update(max_alt_m=max_alt, max_vel_ms=max_vel, max_acc_g=max_acc)
            self.cards["max_alt"].config(text=f"{max_alt} m")
            self.cards["max_vel"].config(text=f"{max_vel:+d} m/s")
            self.cards["max_acc"].config(
                text=f"{max_acc:.2f} g" + (" ⚠削頂" if max_acc >= 23.5 else ""),
                foreground=gt.RED if max_acc >= 23.5 else gt.PURPLE)

            # 開傘高度：哨兵 = 尚未開傘 → 顯示 "未開"，不顯示 0 m（0 m 是合法開傘高度，
            # 兩者混用會把「地面誤觸發開傘」這種該警示的事件偽裝成正常）。
            dalt, malt = int(g["dalt"]), int(g["malt"])
            self.rocket_state.update(drogue_alt_m=dalt, main_alt_m=malt)
            for key, val in (("dalt", dalt), ("malt", malt)):
                if val == TELEM_DEPLOY_ALT_NA:
                    self.cards[key].config(text="未開", foreground=gt.TXT_OFF)
                else:
                    self.cards[key].config(text=f"{val} m", foreground=gt.CYAN_HI)

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
            if self.map_view is not None:
                self.map_view.on_rocket_fix(lat, lon, alt_m=int(g["galt"]), sats=sats)

        pbarb = int(g["pbarb"]) if g["pbarb"] else 0
        self.update_peer_relay(int(g["peer_fsm"]), int(g["pflags"], 16), int(g["plink"], 16),
                                int(g["pbaro"]) / 100.0,
                                int(g["pvfh"]) / 100.0, int(g["pvfv"]) / 100.0, pbarb)

        order = gt.SELF_STATE_ORDER.get(full)
        self.deploy_latch.mark_by_order("self", order)
        self.chart_dirty = True

        # 電梯測試 profile 醒目提示：舊韌體沒有 prof: token 時 g["prof"] 是 None，視為 0（無警示）。
        prof = int(g["prof"], 16) if g["prof"] else 0
        self._update_elevator_warning(prof)

    def _update_elevator_warning(self, prof):
        who = []
        if prof & self.TELEM_PROFILE_SELF_ELEVATOR: who.append("本板(主航電)")
        if prof & self.TELEM_PROFILE_PEER_ELEVATOR: who.append("對端(副航電)")
        active = bool(who)
        who_text = "+".join(who)
        gt.update_elevator_banner(self.root, self.elevator_banner, active, who_text)
        if active:
            gt.spam_elevator_console_warning(self.console, who_text)

    # ------------------------------------------------------------------ [GS_ATT]（姿態四元數）
    # 獨立短行，firmware 端刻意不塞進 [GS_PKT]（那行已經 400+ bytes，見 ground_station.c
    # 對應註解）。seq 供未來需要時與 [GS_PKT] 對應，目前不使用。舊韌體沒有這行時，
    # rocket_state["q"] 維持建構子設的 identity quaternion，Attitude3DView 會顯示 NO DATA。
    _GS_ATT_RE = re.compile(
        r"\[GS_ATT\]\s+seq:(?P<seq>\d+)\s+q:(?P<q0>-?\d+),(?P<q1>-?\d+),(?P<q2>-?\d+),(?P<q3>-?\d+)")

    def parse_gs_att(self, line):
        m = self._GS_ATT_RE.search(line)
        if not m:
            return
        q_raw = [int(m.group(k)) / 10000.0 for k in ("q0", "q1", "q2", "q3")]
        norm = sum(v * v for v in q_raw) ** 0.5
        self.rocket_state["q"] = [v / norm for v in q_raw] if norm > 1e-6 else [1.0, 0.0, 0.0, 0.0]
        self.att_dirty = True

    # ------------------------------------------------------------------ LoRa 參數回報（供 LoRa 面板顯示）
    def parse_lora_params(self, line):
        """只解析 E22/E80 參數回報行（不含角色偵測、不含 [LORA]/[GS_STAT]/[GS_LORA_INIT]
        硬體狀態行——本檔已有 parse_gs_stat 管 433/920 硬體狀態，不重複）。
        逐字取自 gui_monitor.py parse_role_and_lora 的 E22/E80 部分（:3244-3311）。"""
        # E80 現值回報：[E80] freq=920000000 Hz  SF9  BW250 kHz (idx=5)  CR 4/5  pwr=+22 dBm  pre=8
        m = re.search(r"\[E80\] freq=(\d+) Hz\s+SF(\d+)\s+BW([\d\.]+) kHz \(idx=(\d+)\)\s+"
                      r"CR 4/(\d)\s+pwr=(-?\d+) dBm\s+pre=(\d+)", line)
        if m:
            self.lora_e80_state = {
                "freq_hz": int(m.group(1)), "sf": int(m.group(2)),
                "bw_khz": float(m.group(3)), "bw_idx": int(m.group(4)),
                "cr": int(m.group(5)) - 4,   # 顯示為 4/5..4/8，換算回 firmware cr 值 1..4
                "pwr": int(m.group(6)), "pre": int(m.group(7)),
            }
            self._refresh_lora_panel_values()
            return

        # E80 開機組態行 [LORA 920MHz] ... Freq: 920.000 MHz | Power: +22dBm | BW: 250kHz | SF: 9 ...
        m = re.search(r"\[LORA 920MHz\].*Freq:\s*(\d+)\.(\d+)\s*MHz.*Power:\s*\+?(-?\d+)dBm.*"
                      r"BW:\s*(\d+)kHz.*SF:\s*(\d+)", line)
        if m:
            self.lora_e80_state.update({
                "freq_hz": int(m.group(1)) * 1000000 + int(m.group(2)) * 1000,
                "pwr": int(m.group(3)), "bw_khz": float(m.group(4)), "sf": int(m.group(5)),
            })
            self._refresh_lora_panel_values()
            return

        # 地面站格式: [E22] freq=432 MHz  CH=22
        m = re.search(r"\[E22\] freq=(\d+) MHz\s+CH=(\d+)", line)
        if m:
            self.lora_e22_state.update({"freq_mhz": int(m.group(1)), "ch": int(m.group(2))})
            self._refresh_lora_panel_values()
            return

        # 設定成功回報: [E22] freq set 435 MHz (CH=25) OK
        m = re.search(r"\[E22\] freq set (\d+) MHz \(CH=(\d+)\) OK", line)
        if m:
            self.lora_e22_state.update({"freq_mhz": int(m.group(1)), "ch": int(m.group(2))})
            self._refresh_lora_panel_values()
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
            self._refresh_lora_panel_values()
            return

        # E22 功率設定成功回報: [E22] pwr set level 3 OK （韌體只回等級，換算成 dBm 顯示）
        m = re.search(r"\[E22\] pwr set level (\d+) OK", line)
        if m:
            lvl = int(m.group(1))
            if 0 <= lvl < len(self.E22_PWR_LEVELS):
                self.lora_e22_state["power"] = self.E22_PWR_LEVELS[lvl][0].split("=")[-1].strip()
            self._refresh_lora_panel_values()
            return

        # E22 空速設定成功回報: [E22] air rate set 2 OK（兩端須一致）
        m = re.search(r"\[E22\] air rate set (\d+) OK", line)
        if m:
            ar = int(m.group(1))
            if 0 <= ar < len(self.E22_AIR_RATES):
                self.lora_e22_state["air_rate"] = self.E22_AIR_RATES[ar][0].split("=")[-1].strip()
            self._refresh_lora_panel_values()

    def _update_link_rate(self, link, t):
        """記錄本鏈路實際到達時刻，換算「有效封包頻率」與間隔統計（見檔頭常數區塊
        說明：不再用 seq gap 湊丟包率，只依賴實測到達時間）。"""
        times = self._pkt_times[link]
        times.append(t)
        while len(times) > 1 and (t - times[0]) > LINK_RATE_WINDOW_S:
            times.popleft()
        if len(times) >= 2:
            span = times[-1] - times[0]
            self._link_rate_hz[link] = (len(times) - 1) / span if span > 0 else 0.0
            gaps_ms = [(times[i] - times[i - 1]) * 1000.0 for i in range(1, len(times))]
            self._link_gap_ms[link] = (sum(gaps_ms) / len(gaps_ms), max(gaps_ms))
        else:
            self._link_rate_hz[link] = 0.0
            self._link_gap_ms[link] = (0.0, 0.0)
        self._update_link_health_label()

    def _link_gap_severity(self, link):
        """0=正常 1=WARN 2=FAIL。門檻＝名目間隔(1/NOMINAL_RATE_HZ)乘上對應倍率，
        與 ground_station_analyzer.py gap_limits() 同一組常數，判定基準一致。"""
        nominal_gap_ms = 1000.0 / NOMINAL_RATE_HZ[link]
        mean_ms, max_ms = self._link_gap_ms[link]
        max_warn = max(nominal_gap_ms * GAP_MAX_WARN_MULT, GAP_MAX_WARN_FLOOR_MS)
        max_fail = max(nominal_gap_ms * GAP_MAX_FAIL_MULT, GAP_MAX_FAIL_FLOOR_MS)
        mean_warn = nominal_gap_ms * GAP_MEAN_WARN_MULT
        mean_fail = nominal_gap_ms * GAP_MEAN_FAIL_MULT
        if max_ms > max_fail or mean_ms > mean_fail:
            return 2
        if max_ms > max_warn or mean_ms > mean_warn:
            return 1
        return 0

    def _update_link_health_label(self):
        # [GS_PKT?] CRC_BAD 與 [GS_STAT] 的 crc=%lu 是同一個火韌體事件（見
        # ground_station.c gs_deliver_433 / GsLoraTest_UpdateStats）——兩者不可相加，
        # 否則會把同一次 CRC 錯誤算兩遍。取 max：crc_bad_* 逐包即時更新（GS_STAT 每 2
        # 秒才回報一次，中間會落後），GS_STAT 的 crc_* 則是開機以來的累積值，GUI 重新
        # 連線時 crc_bad_* 會歸零但 GS_STAT 仍是舊累積值，取 max 兩種情況都不會失真。
        s = self.gs_state
        crc_433 = max(s["crc_433"], s["crc_bad_433"])
        crc_920 = max(s["crc_920"], s["crc_bad_920"])
        text = (f"⚠ 433 CRC:{crc_433}/RS:{s['rsync_433']}/Hz:{self._link_rate_hz['433']:.2f}  "
                f"920 CRC:{crc_920}/RS:{s['rsync_920']}/Hz:{self._link_rate_hz['920']:.2f}")
        worst = max(self._link_gap_severity("433"), self._link_gap_severity("920"))
        color = gt.RED_ALT if worst >= 2 else (gt.YELLOW if worst >= 1 else gt.TXT_MUTED)
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

    def update_peer_relay(self, pfsm, pflags, plink,
                           pbaro_m=0.0, pvfh_m=0.0, pvfv_ms=0.0, pbarb=0):
        """副航電中繼摘要。★2026-07-30 起下鏈只帶 VF（高度/速度）+ baro + 鏈路健康位，
        EKF 高度/速度、高G、丟包率已從封包移除（見 telemetry.h 同日註解）。"""
        name = gt.PEER_FSM_NAMES[pfsm] if pfsm < len(gt.PEER_FSM_NAMES) else f"?{pfsm}"
        ever, fresh = bool(plink & gt.PEER_LINK_EVER), bool(plink & gt.PEER_LINK_FRESH)
        lost, desync = bool(plink & gt.PEER_LINK_LOST), bool(plink & gt.PEER_LINK_DESYNC)

        # 副航電已解鎖 (PAD_ARMED) 比照本板 _set_fsm_full/gt.fsm_style 的紅字警示——
        # 對端已武裝代表其點火/舵機機構也已待發，不該只有主航電 ARMED 才紅字。
        if name == "PAD_ARMED":
            self.lbl_peer_fsm.config(text=f"⚡ 副航電: {name}", fg=gt.RED if fresh else gt.TXT_OFF)
        else:
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
                gt.pill(self.peer_pill_row, "⚠逾時", "#3b2d00", gt.YELLOW)
            arb_name = gt.BENCH_ARB_NAMES.get(pbarb, f"?{pbarb}")
            if pbarb:
                gt.pill(self.peer_pill_row, f"🖥️ {arb_name}", "#2d1b4e", gt.PURPLE)

        if fresh:
            self.lbl_peer_val.config(
                # VF h 是相對起點，baro 是絕對海拔（見 build_alt_chart 註解），標明免得混看
                text=f"VF h={pvfh_m:.1f}m(相對)  VF v={pvfv_ms:+.1f}m/s  baro={pbaro_m:.1f}m(海拔)",
                fg=gt.TXT)
            self.lbl_peer_sub.config(text=f"主傘/桌測 {gt.BENCH_ARB_NAMES.get(pbarb, pbarb)}",
                                      fg=gt.TXT_MUTED)
            t = self.now_t()
            self.ts_peer_vf_alt.append((t, pvfh_m))
            self.ts_peer_baro.append((t, pbaro_m))
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
            if self.map_view is not None:
                self.map_view.on_gs_fix(lat, lon)
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
            "確定要經 433 上行發送『桌面開傘測試 (BENCH)』指令嗎？\n\n"
            "⚠️ 注意（時序與飛行邏輯 1:1 對應）：\n"
            # 副板延後量 = 韌體 DROGUE_LEAD_TIME_S，依 FLIGHT_PROFILE_ELEVATOR 為
            # 1s(電梯場測)/4s(飛行)。地面站這端收不到航電的 [PYRO-SELFTEST] 文字行，
            # 無從得知板上燒的是哪一組，故不寫死秒數。
            "1. 引傘 PD13：主板 t=0 起通電 8s；副板延後 DROGUE_LEAD_TIME_S（模擬頂點提前量，\n"
            "   依 profile 為 1s/4s）後通電 3s，\n"
            "   兩板通電窗會重疊（PD13 為 diode-OR 準位訊號，同時拉高無妨）。\n"
            "2. 主傘 PD14：★不啟動 PWM，兩板『同時』純 GPIO 拉高 1.5s（已取消互斥握手）。\n"
            "3. 全程耗時約 20 秒，測試完成後自動復位並回歸正常 FSM。\n"
            "4. 航電必須處於解鎖狀態 (STATE_PAD_ARMED)。\n\n"
            "是否立即執行？")
        if ans:
            self.send_command("bench")

    # ==================== LoRa 參數設定面板 ====================
    # 逐字取自 gui_monitor.py open_lora_panel + build_e22_tab/build_e80_tab/build_gs_tab/
    # build_doc_tab/refresh_lora_panel_values，丟掉角色 enable/disable 那套
    # （lora_widgets_common/lora_widgets_ground/refresh_lora_panel_role）——本檔永遠對
    # GROUND 板講話，所有功能永遠開著，不需要依角色動態禁用。
    def open_lora_panel(self):
        if not self.running:
            messagebox.showwarning("提示", "請先開啟串口連接地面站板，再進行 LoRa 參數設定。")
            return
        if getattr(self, "lora_win", None) is not None and self.lora_win.winfo_exists():
            self.lora_win.lift()
            return

        self.lora_win = tk.Toplevel(self.root)
        self.lora_win.title("📡 LoRa 通訊參數設定（E22 433MHz / E80 920MHz）")
        self.lora_win.geometry("980x700")
        self.lora_win.configure(bg="#1c1c1c")
        # 不用 transient(self.root)：macOS 上 transient 視窗沒有獨立 Dock 圖示，縮小/關閉
        # 主視窗或任一子視窗會把整組一起帶走，子視窗需要能各自獨立縮小/關閉。

        tk.Label(self.lora_win,
                 text="⚠ 頻率/SF/BW/CR/空速 兩端（火箭 ↔ 地面站）必須一致才能通訊；詳見「📖 參數教學」分頁",
                 bg="#1c1c1c", fg="#ffcc00", font=("Helvetica", 10), anchor="w").pack(fill=tk.X, padx=16, pady=(12, 8))

        nb = ttk.Notebook(self.lora_win)
        nb.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

        tab_e22 = tk.Frame(nb, bg="#1c1c1c")
        tab_e80 = tk.Frame(nb, bg="#1c1c1c")
        tab_gs = tk.Frame(nb, bg="#1c1c1c")
        tab_doc = tk.Frame(nb, bg="#1c1c1c")
        nb.add(tab_e22, text=" E22 433MHz ")
        nb.add(tab_e80, text=" E80 920MHz ")
        nb.add(tab_gs, text=" 統計/工具 ")
        nb.add(tab_doc, text=" 📖 參數教學 ")

        self.build_e22_tab(tab_e22)
        self.build_e80_tab(tab_e80)
        self.build_gs_tab(tab_gs)
        self.build_doc_tab(tab_doc)

        self._refresh_lora_panel_values()

        # 開啟即查詢兩鏈路現值。
        self.lora_send("e22 show")
        self.lora_win.after(700, lambda: self.lora_send("e80 show"))

    def lora_send(self, cmd):
        """LoRa 面板統一發送入口（e22/e80 頻率/功率/空速/SF/BW/CR/前導碼、stats、ver、
        e80 init/rxstart/airtime 等）。"""
        return self.cmd_sender.send(cmd, parent=self.lora_win)

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

        h = self._lora_row(grid, 0, "頻率 (MHz)",
                           f"範圍 {self.E22_FREQ_RANGE[0]}–{self.E22_FREQ_RANGE[1]} MHz（CH=頻率−410）\n"
                           "開機會被 firmware 強制寫回預設 432 MHz")
        self.e22_freq_var = tk.StringVar(value="432")
        tk.Spinbox(h, from_=self.E22_FREQ_RANGE[0], to=self.E22_FREQ_RANGE[1],
                   textvariable=self.e22_freq_var, width=8, font=("Monaco", 11),
                   bg="#252525", fg="#ffffff", insertbackground="white").pack(side=tk.LEFT)
        ttk.Button(h, text="套用頻率", width=9, command=self.apply_e22_freq).pack(side=tk.LEFT, padx=8)

        h = self._lora_row(grid, 1, "發射功率",
                           "⚠ 本板 3V3 供電，30dBm 會欠壓斷線\n建議固定 3 (21dBm)")
        self.e22_pwr_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E22_PWR_LEVELS],
                                          width=30, state="readonly", font=("Helvetica", 10))
        self.e22_pwr_combo.current(3)
        self.e22_pwr_combo.pack(side=tk.LEFT)
        ttk.Button(h, text="套用功率", width=9, command=self.apply_e22_pwr).pack(side=tk.LEFT, padx=8)

        h = self._lora_row(grid, 2, "空中速率",
                           "越低 → 射程越遠/抗干擾越強、資料率越低\n⚠ 兩端必須一致")
        self.e22_air_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E22_AIR_RATES],
                                          width=30, state="readonly", font=("Helvetica", 10))
        self.e22_air_combo.current(2)
        self.e22_air_combo.pack(side=tk.LEFT)
        ttk.Button(h, text="套用空速", width=9, command=self.apply_e22_air).pack(side=tk.LEFT, padx=8)

        tools = tk.Frame(tab, bg="#1c1c1c")
        tools.pack(fill=tk.X, padx=14, pady=12)
        ttk.Button(tools, text="🔍 查詢目前設定 (e22 show)", width=24,
                   command=lambda: self.lora_send("e22 show")).pack(side=tk.LEFT)

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
        ttk.Button(h, text="套用頻率", width=9, command=self.apply_e80_freq).pack(side=tk.LEFT, padx=8)

        h = self._lora_row(grid, 1, "展頻因子 SF",
                           "7–12；每 +1 → 靈敏度 +~2.5dB、空中時間 ×2\n預設 SF9")
        self.e80_sf_var = tk.StringVar(value="9")
        tk.Spinbox(h, from_=self.E80_SF_RANGE[0], to=self.E80_SF_RANGE[1],
                   textvariable=self.e80_sf_var, width=5, font=("Monaco", 11),
                   bg="#252525", fg="#ffffff", insertbackground="white").pack(side=tk.LEFT)
        ttk.Button(h, text="套用 SF", width=9, command=self.apply_e80_sf).pack(side=tk.LEFT, padx=8)

        h = self._lora_row(grid, 2, "頻寬 BW",
                           "加倍 → 資料率加倍、靈敏度 −~3dB\n預設 5 (250kHz)")
        self.e80_bw_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E80_BW_OPTIONS],
                                         width=28, state="readonly", font=("Helvetica", 10))
        self.e80_bw_combo.current(2)
        self.e80_bw_combo.pack(side=tk.LEFT)
        ttk.Button(h, text="套用 BW", width=9, command=self.apply_e80_bw).pack(side=tk.LEFT, padx=8)

        h = self._lora_row(grid, 3, "編碼率 CR",
                           "越大 → 糾錯越強、有效資料率越低\n預設 1 (4/5)")
        self.e80_cr_combo = ttk.Combobox(h, values=[f"{v}  {note}".rstrip() for v, note in self.E80_CR_OPTIONS],
                                         width=28, state="readonly", font=("Helvetica", 10))
        self.e80_cr_combo.current(0)
        self.e80_cr_combo.pack(side=tk.LEFT)
        ttk.Button(h, text="套用 CR", width=9, command=self.apply_e80_cr).pack(side=tk.LEFT, padx=8)

        h = self._lora_row(grid, 4, "發射功率 (dBm)",
                           f"範圍 {self.E80_PWR_RANGE[0]} ~ +{self.E80_PWR_RANGE[1]} dBm\n"
                           "22 = HP PA 上限（預設）；兩端可不同")
        self.e80_pwr_var = tk.StringVar(value="22")
        tk.Spinbox(h, from_=self.E80_PWR_RANGE[0], to=self.E80_PWR_RANGE[1],
                   textvariable=self.e80_pwr_var, width=5, font=("Monaco", 11),
                   bg="#252525", fg="#ffffff", insertbackground="white").pack(side=tk.LEFT)
        ttk.Button(h, text="套用功率", width=9, command=self.apply_e80_pwr).pack(side=tk.LEFT, padx=8)

        h = self._lora_row(grid, 5, "前導碼 (符號)",
                           f"範圍 {self.E80_PRE_RANGE[0]}–{self.E80_PRE_RANGE[1]}；一般 8 即可\n"
                           "弱訊號時加長可提升同步成功率")
        self.e80_pre_var = tk.StringVar(value="8")
        e = tk.Entry(h, textvariable=self.e80_pre_var, width=7, font=("Monaco", 11),
                     bg="#252525", fg="#ffffff", insertbackground="white")
        e.pack(side=tk.LEFT)
        ttk.Button(h, text="套用前導碼", width=9, command=self.apply_e80_pre).pack(side=tk.LEFT, padx=8)

        tools = tk.Frame(tab, bg="#1c1c1c")
        tools.pack(fill=tk.X, padx=14, pady=12)
        ttk.Button(tools, text="🔍 查詢目前參數 (e80 show)", width=24,
                   command=lambda: self.lora_send("e80 show")).pack(side=tk.LEFT)

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

    # ---- 統計/工具分頁 ----
    def build_gs_tab(self, tab):
        row1 = tk.Frame(tab, bg="#1c1c1c")
        row1.pack(fill=tk.X, padx=14, pady=(12, 4))
        for text, cmd in [("📊 顯示統計 (stats)", "stats"),
                          ("🧹 清除統計 (stats reset)", "stats reset"),
                          ("🛰 E80 版本診斷 (ver)", "ver")]:
            ttk.Button(row1, text=text, width=24, command=lambda c=cmd: self.lora_send(c)).pack(side=tk.LEFT, padx=4)

        row2 = tk.Frame(tab, bg="#1c1c1c")
        row2.pack(fill=tk.X, padx=14, pady=4)
        tk.Label(row2, text="自動列印統計間隔(秒, 0=關閉):", bg="#1c1c1c", fg="#ffffff",
                 font=("Helvetica", 10)).pack(side=tk.LEFT)
        self.gs_auto_var = tk.StringVar(value="5")
        tk.Spinbox(row2, from_=0, to=3600, textvariable=self.gs_auto_var, width=6,
                   font=("Monaco", 11), bg="#252525", fg="#ffffff",
                   insertbackground="white").pack(side=tk.LEFT, padx=6)
        ttk.Button(row2, text="套用 (stats auto N)", width=18,
                   command=lambda: self.lora_send(f"stats auto {self.gs_auto_var.get()}")).pack(side=tk.LEFT, padx=4)

        row3 = tk.Frame(tab, bg="#1c1c1c")
        row3.pack(fill=tk.X, padx=14, pady=4)
        for text, cmd in [("♻️ E80 重新初始化+收 (e80 init)", "e80 init"),
                          ("▶️ 重新進入連續接收 (e80 rxstart)", "e80 rxstart")]:
            ttk.Button(row3, text=text, width=28, command=lambda c=cmd: self.lora_send(c)).pack(side=tk.LEFT, padx=4)

        row4 = tk.Frame(tab, bg="#1c1c1c")
        row4.pack(fill=tk.X, padx=14, pady=4)
        tk.Label(row4, text="空中時間估算 payload 長度(B):", bg="#1c1c1c", fg="#ffffff",
                 font=("Helvetica", 10)).pack(side=tk.LEFT)
        self.gs_airtime_var = tk.StringVar(value="77")
        tk.Spinbox(row4, from_=1, to=255, textvariable=self.gs_airtime_var, width=5,
                   font=("Monaco", 11), bg="#252525", fg="#ffffff",
                   insertbackground="white").pack(side=tk.LEFT, padx=6)
        ttk.Button(row4, text="估算 (e80 airtime N)", width=18,
                   command=lambda: self.lora_send(f"e80 airtime {self.gs_airtime_var.get()}")).pack(side=tk.LEFT, padx=4)

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
    def _refresh_lora_panel_values(self):
        """把最近解析到的板上現值刷到面板（僅更新「目前設定」顯示，不覆寫使用者輸入框）"""
        if not (getattr(self, "lora_win", None) is not None and self.lora_win.winfo_exists()):
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

    # ==================== Flash 接收紀錄管理面板 ====================
    # 與 gui_monitor.py 對稱：raw/（sysflags hex + 結構化 CSV）+ processed/（自動報告/
    # 圖表/地圖）。firmware `flash export`（gs_lora_test.c）已補上結構化匯出，CSV 格式
    # 與 SD 卡紀錄同源（gs_log.c GsLog_CsvHeader），可直接餵給 ground_station_analyzer.py。
    def open_flash_panel(self):
        if not self.running:
            messagebox.showwarning("提示", "請先開啟串口連接地面站板，再進行 Flash 管理。")
            return
        if getattr(self, "flash_win", None) is not None and self.flash_win.winfo_exists():
            self.flash_win.lift()
            return

        self.flash_win = tk.Toplevel(self.root)
        self.flash_win.title("💾 地面站接收紀錄管理")
        self.flash_win.geometry("620x380")
        self.flash_win.configure(bg="#1c1c1c")
        # 不用 transient()：見 open_lora_panel 註解，子視窗要能各自獨立縮小/關閉。

        tk.Label(self.flash_win, text="地面站板自己的 Flash（W25Q128）接收紀錄", bg="#1c1c1c",
                 fg="#00d2ff", font=("Helvetica", 12, "bold"), anchor="w").pack(fill=tk.X, padx=16, pady=(14, 4))
        tk.Label(self.flash_win,
                 text="每筆收到的下行遙測本來就即時串流進 USB/SD，Flash 只是第三份備援副本。\n"
                      "可整段清空，也可結構化匯出（hex + CSV），並自動產生報告/圖表/地圖。",
                 bg="#1c1c1c", fg="#8a8a8a", font=("Helvetica", 9), justify=tk.LEFT,
                 anchor="w").pack(fill=tk.X, padx=16, pady=(0, 12))

        op_box = tk.LabelFrame(self.flash_win, text=" 清除操作 ", bg="#1c1c1c", fg="#ff8080",
                               font=("Helvetica", 10, "bold"), padx=10, pady=10)
        op_box.pack(fill=tk.X, padx=16, pady=(0, 10))

        erase_frame = tk.Frame(op_box, bg="#1c1c1c")
        erase_frame.pack(fill=tk.X, pady=4)
        gt.StyledButton(erase_frame, text="🔴 清空接收紀錄", command=self.erase_gs_flash,
                        bg="#b71c1c", hover_bg="#d32f2f", fg="#ffffff",
                        font=("Helvetica", 10, "bold"), padx=14, pady=7).pack(side=tk.LEFT, padx=(0, 10))
        tk.Label(erase_frame, text="整段擦淨並重設寫入頭，約 30-60 秒（期間停止接收下行遙測）",
                 bg="#1c1c1c", fg="#ff8888", font=("Helvetica", 9)).pack(side=tk.LEFT)

        export_box = tk.LabelFrame(self.flash_win, text=" 結構化匯出 ", bg="#1c1c1c", fg="#00e676",
                                   font=("Helvetica", 10, "bold"), padx=10, pady=10)
        export_box.pack(fill=tk.X, padx=16, pady=(0, 10))

        export_frame = tk.Frame(export_box, bg="#1c1c1c")
        export_frame.pack(fill=tk.X, pady=4)
        gt.StyledButton(export_frame, text="💾 匯出並分析", command=self.export_flash_csv,
                        bg="#005577", hover_bg="#007799", fg="#ffffff",
                        font=("Helvetica", 10, "bold"), padx=14, pady=7).pack(side=tk.LEFT, padx=(0, 10))
        tk.Label(export_frame, text="raw/ 存 hex+CSV 原始檔，processed/ 自動生成報告/圖表/地圖",
                 bg="#1c1c1c", fg="#88ddaa", font=("Helvetica", 9)).pack(side=tk.LEFT)

        self.lbl_flash_status = tk.Label(self.flash_win, text="就緒。", bg="#1c1c1c", fg="#888888",
                                         font=("Monaco", 9, "bold"), anchor="w")
        self.lbl_flash_status.pack(fill=tk.X, padx=16, pady=(6, 0))
        self.lbl_flash_export_status = tk.Label(self.flash_win, text="", bg="#1c1c1c", fg="#888888",
                                                font=("Monaco", 9, "bold"), anchor="w", justify=tk.LEFT,
                                                wraplength=580)
        self.lbl_flash_export_status.pack(fill=tk.X, padx=16, pady=(2, 10))

    def erase_gs_flash(self):
        if not self.running or not getattr(self, "ser", None):
            messagebox.showwarning("警告", "串口未連接！無法發送清空命令。", parent=getattr(self, "flash_win", None))
            return
        if messagebox.askyesno(
                "⚠️ 危險操作確認",
                "是否確定要清空地面站 Flash 內的接收紀錄？\n\n"
                "擦除期間（約 30-60 秒）地面站【停止接收火箭下行遙測】，之後會自動恢復。\n"
                "此操作無法復原！",
                parent=getattr(self, "flash_win", None)):
            if self.local_send("flash erase"):
                if hasattr(self, "lbl_flash_status") and self.lbl_flash_status.winfo_exists():
                    self.lbl_flash_status.config(text="⏳ 正在清空接收紀錄中 (約 30-60 秒，期間停止收包)...",
                                                 fg="#ffcc00")

    def _handle_flash_progress(self, line):
        """順手認一下 [FLASH]/[FLASH_RING] 開頭的進度/完成行，把狀態 label 從固定的
        「⏳ 60秒」換成即時進度（韌體本來就每 32 blocks 印一次，見 w25qxx.c FlashRing_EraseAll）。
        面板未開啟時直接跳過（label 還不存在）。"""
        if not (hasattr(self, "lbl_flash_status") and self.lbl_flash_status.winfo_exists()):
            return
        m = re.search(r"\[FLASH_RING\] Erasing\.\.\. (\d+)/255 blocks", line)
        if m:
            self.lbl_flash_status.config(text=f"⏳ 擦除中… {m.group(1)}/255 blocks", fg="#ffcc00")
            return
        if "[FLASH] 接收紀錄已清空並重設寫入頭 OK" in line:
            self.lbl_flash_status.config(text="✅ 清空完成，接收紀錄已重設", fg="#34d399")
        elif "[FLASH] ERROR" in line:
            self.lbl_flash_status.config(text=f"❌ {line.strip()}", fg="#ff4d4d")

    # ---- 結構化匯出：raw/（hex+CSV）+ processed/（報告/圖表/地圖）----
    # 逐字比照 gui_monitor.py 的 export_flash_csv/_handle_flash_export_stream_line/
    # _finish_flash_export 三段式流程，欄位表換成 GS_FLASH_EXPORT_COLUMNS（gs_log.c
    # GsLog_CsvHeader，非火箭 Flash Ring 的 FLASH_EXPORT_COLUMNS）。
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

    def _write_flash_export_info(self, finished_at=None):
        export_dir = getattr(self, '_flash_export_dir', None)
        if not export_dir:
            return
        try:
            info_path = os.path.join(export_dir, "export_info.txt")
            with open(info_path, "w", encoding="utf-8") as info:
                info.write("RocketCom Ground Station Flash Export\n")
                info.write(f"started_at={getattr(self, '_flash_export_started_at', '')}\n")
                if finished_at:
                    info.write(f"finished_at={finished_at}\n")
                info.write(f"total_rows={getattr(self, '_flash_export_count', 0)}\n")
                info.write(f"skipped_lines={getattr(self, '_flash_export_skipped', 0)}\n")
                info.write("raw/       = 原始資料：ground_log_all.csv, sysflags_sector0.hex\n")
                info.write("processed/ = 整理後資料：sysflags_summary.txt（Sector0 解碼，通常為空，\n")
                info.write("             除非晶片曾被燒錄成飛控角色）、ground_link_report_*.md/.json/.html\n")
                info.write("             （雙鏈路接收品質）、ground_relay_analysis_*.png（轉播飛行數據圖）、\n")
                info.write("             ground_relay_map_*.html（GPS 航跡地圖，火箭航跡+地面站定點）\n")
        except Exception:
            pass

    def export_flash_csv(self):
        """選擇資料夾並啟動 Flash 結構化匯出。"""
        if not self.running or not getattr(self, "ser", None):
            messagebox.showwarning("警告", "串口未連接！無法進行 Flash 匯出。", parent=getattr(self, "flash_win", None))
            return
        if getattr(self, '_flash_export_active', False):
            messagebox.showinfo("提示", "已有匯出正在進行中，請稍候。", parent=getattr(self, "flash_win", None))
            return

        parent_dir = filedialog.askdirectory(parent=getattr(self, "flash_win", None), title="選擇 Flash 匯出資料夾")
        if not parent_dir:
            return

        try:
            now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            export_dir = os.path.join(parent_dir, f"gs_flash_export_{now_str}")
            raw_dir = os.path.join(export_dir, "raw")
            processed_dir = os.path.join(export_dir, "processed")
            os.makedirs(export_dir, exist_ok=False)
            os.makedirs(raw_dir, exist_ok=False)
            os.makedirs(processed_dir, exist_ok=False)
            filepath = os.path.join(raw_dir, "ground_log_all.csv")

            f = open(filepath, "w", encoding="utf-8", newline="")
            self._flash_export_dir = export_dir
            self._flash_export_raw_dir = raw_dir
            self._flash_export_processed_dir = processed_dir
            self._flash_export_started_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self._flash_export_file = f
            self._flash_export_writer = csv.writer(f, lineterminator="\n")
            self._flash_export_sysflags_file = None
            self._flash_export_mode = None
            self._flash_export_active = True
            self._flash_export_writing = False
            self._flash_export_count = 0
            self._flash_export_skipped = 0
            self._write_flash_export_info()
            self._set_flash_export_status(f"📥 正在匯出到 {os.path.basename(export_dir)}...", "#00d2ff")
            if not self.local_send("flash export"):
                self._finish_flash_export(ok=False, message="⚠️ 匯出未開始：指令送出失敗")
        except Exception as e:
            self._close_flash_export_file()
            self._set_flash_export_status(f"❌ 無法建立匯出資料夾: {e}", "#ff4d4d")

    def _parse_gs_flash_export_row(self, line):
        try:
            row = next(csv.reader([line]))
        except csv.Error:
            return None
        row = [cell.strip() for cell in row]
        if row == GS_FLASH_EXPORT_COLUMNS:
            return "header"
        if len(row) != len(GS_FLASH_EXPORT_COLUMNS):
            return None
        if not GS_FLASH_EXPORT_HMS_RE.match(row[0] or "") or not GS_FLASH_EXPORT_HMS_RE.match(row[1] or ""):
            return None
        if not all(GS_FLASH_EXPORT_CELL_RE.match(cell or "") for cell in row[2:]):
            return None
        return row

    def _handle_flash_export_stream_line(self, line):
        if not getattr(self, '_flash_export_active', False):
            return

        if "--- SYSFLAGS_START ---" in line:
            self._flash_export_mode = "sysflags"
            sysflags_path = os.path.join(self._flash_export_raw_dir, "sysflags_sector0.hex")
            self._flash_export_sysflags_file = open(sysflags_path, "w", encoding="utf-8")
            self._flash_export_sysflags_file.write(f"# exported_at={datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self._set_flash_export_status("📥 正在讀取 Sector 0...", "#00d2ff")
            return

        if "--- SYSFLAGS_END ---" in line:
            self._flash_export_mode = None
            sysflags_file = getattr(self, '_flash_export_sysflags_file', None)
            if sysflags_file:
                sysflags_file.flush()
            self._set_flash_export_status("📥 Sector 0 已儲存，等待接收紀錄 CSV...", "#00d2ff")
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
                writer.writerow(GS_FLASH_EXPORT_COLUMNS)
            self._set_flash_export_status("📥 正在匯出中...", "#00d2ff")
            return

        if "--- CSV_END ---" in line or "[FLASH] Export finished" in line:
            self._finish_flash_export(ok=True)
            return

        if getattr(self, '_flash_export_mode', None) == "sysflags":
            if GS_FLASH_SYSFLAGS_HEX_RE.match(line):
                sysflags_file = getattr(self, '_flash_export_sysflags_file', None)
                if sysflags_file:
                    sysflags_file.write(line + "\n")
            else:
                self._flash_export_skipped = getattr(self, '_flash_export_skipped', 0) + 1
            return

        if not getattr(self, '_flash_export_writing', False):
            return

        parsed = self._parse_gs_flash_export_row(line)
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
        self._flash_export_count = getattr(self, '_flash_export_count', 0) + 1
        if self._flash_export_count % 10 == 0:
            self._set_flash_export_status(f"📥 已匯出 {self._flash_export_count} 筆...", "#00d2ff")

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

        if ok and raw_dir and os.path.exists(raw_dir):
            try:
                self._process_gs_flash_export(raw_dir, processed_dir)
            except Exception as e:
                print(f"[ANALYZER] 自動生成地面站報告/圖表/地圖失敗: {e}")

        if message is None:
            if ok:
                message = (f"✅ 匯出完成！{rows} 筆，原始資料於 raw/，"
                           f"已自動生成報告/圖表/地圖於 processed/（{os.path.basename(export_dir)}）")
            else:
                message = f"⚠️ 匯出中止。有效 {rows} 筆，略過 {skipped} 行"
        self._set_flash_export_status(message, "#00e676" if ok else "#ffcc00")

    def _process_gs_flash_export(self, raw_dir, processed_dir):
        """讀 raw/，寫入 processed/：
          ① sysflags 摘要——沿用 flash_analyzer.py 既有的角色無關讀取器（in-process 匯入，
             該模組固定用 Agg headless backend，安全）。
          ② 雙鏈路接收品質報告 + 轉播飛行圖表/GPS 地圖——改用 subprocess 呼叫
             ground_station_analyzer.py 的 CLI（--csv --out-dir），不 in-process import：
             該模組頂層會依平台選互動式 matplotlib backend（供其獨立執行時的即時圖表用），
             若在本 GUI 的 Tkinter mainloop 裡直接 import 可能與 Tk 的事件迴圈衝突，故隔成
             獨立行程執行，跟命令列使用方式完全一致。"""
        os.makedirs(processed_dir, exist_ok=True)
        tools_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools"))

        sysflags_hex = os.path.join(raw_dir, "sysflags_sector0.hex")
        if os.path.exists(sysflags_hex):
            if tools_dir not in sys.path:
                sys.path.append(tools_dir)
            import flash_analyzer
            flash_analyzer.generate_sysflags_report(sysflags_hex, processed_dir)

        all_csv = os.path.join(raw_dir, "ground_log_all.csv")
        if not os.path.exists(all_csv):
            return

        gsa_script = os.path.join(tools_dir, "ground_station_analyzer.py")
        try:
            import subprocess
            proc = subprocess.run(
                [sys.executable, gsa_script, "--csv", all_csv, "--out-dir", processed_dir],
                capture_output=True, text=True, timeout=120,
            )
            if proc.returncode != 0:
                print(f"[ANALYZER] ground_station_analyzer.py 執行失敗 (rc={proc.returncode}): {proc.stderr[-500:]}")
        except Exception as e:
            print(f"[ANALYZER] 無法啟動 ground_station_analyzer.py 子行程: {e}")

    # ------------------------------------------------------------------ 圖表獨立視窗
    # 歷史時序圖表（鏈路 RSSI/SNR、高度/速度）拆成獨立 Toplevel，方便拖到第二顆螢幕；
    # 不在啟動時自動開（單螢幕操作者不會被迫多開一個視窗），開啟時用既有 ts_*/link_stats
    # 歷史 deque 立刻補畫。地圖/3D 姿態留在主視窗（見 _build_right_pane），因為那是「當下
    # 狀態」而非「歷史圖表」。
    def open_chart_window(self):
        if getattr(self, "chart_win", None) is not None and self.chart_win.winfo_exists():
            self.chart_win.lift()
            return
        self.chart_win = tk.Toplevel(self.root)
        self.chart_win.title("📈 飛行圖表（鏈路 RSSI/SNR ｜ 高度・速度比對）")
        self.chart_win.geometry("900x680")
        self.chart_win.configure(bg=gt.BG_ROOT)
        # 不用 transient()：見 open_lora_panel 註解，子視窗要能各自獨立縮小/關閉。
        self.chart_win.protocol("WM_DELETE_WINDOW", self._on_chart_window_close)

        nb = ttk.Notebook(self.chart_win)
        nb.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        link_tab = tk.Frame(nb, bg=gt.BG_PANEL)
        nb.add(link_tab, text=" 鏈路 RSSI/SNR ")
        self.build_link_charts(link_tab)

        alt_tab = tk.Frame(nb, bg=gt.BG_PANEL)
        nb.add(alt_tab, text=" 高度・速度比對 ")
        self.build_alt_charts(alt_tab)

        self.chart_dirty = True   # 立刻用既有歷史補畫一幀，不必等下一筆封包

    def _on_chart_window_close(self):
        """視窗關閉：務必清空 figure/canvas 參照，否則 charts_redraw_loop 之後每次 tick
        都會對著已銷毀的 widget 呼叫 draw_idle() 拋 TclError，且重開視窗也救不回來。"""
        try:
            self.chart_win.destroy()
        finally:
            self.chart_win = None
            for attr in ("chart_fig", "chart_canvas", "ax_rssi", "ax_snr", "ln_rssi", "ln_snr",
                        "alt_fig", "alt_canvas", "ax_alt", "ax_vz",
                        "ln_alt_ekf", "ln_alt_baro", "ln_alt_vf",
                        "ln_alt_peer_baro", "ln_alt_peer_vf",
                        "ln_vz_ekf", "ln_vz_vf", "ln_vz_peer_vf"):
                setattr(self, attr, None)

    def charts_redraw_loop(self):
        chart_win = getattr(self, "chart_win", None)
        if self.chart_dirty and chart_win is not None and chart_win.winfo_exists():
            self.redraw_charts()
        self.chart_dirty = False   # 無條件清掉：視窗關閉期間也不該讓旗標一直卡 True
        try:
            if self.root.winfo_exists():
                self.root.after(200, self.charts_redraw_loop)
        except tk.TclError:
            pass

    def attitude_redraw_loop(self):
        """3D 姿態渲染：dirty-flag 節流，比照 gui_monitor.py 的手法。一幀 3D 渲染成本不低
        （matplotlib 3D，約數十 ms），故用 300ms 間隔（比 gui_monitor 的 200ms 稍寬鬆——
        本檔 poll_queue 是 10ms 一次 drain 到 60 行，比 gui_monitor 積極很多，兩者疊加需
        更保守的預算；若實測仍卡頓，可再拉長或請操作者按面板上的「暫停渲染」）。"""
        try:
            if not self.root.winfo_exists():
                return
            if self.att_dirty and self.att_view is not None and self.att_view.is_alive():
                self.att_view.update(self.rocket_state.get("q", [1.0, 0.0, 0.0, 0.0]))
                self.att_dirty = False
            self.root.after(300, self.attitude_redraw_loop)
        except tk.TclError:
            pass

    def map_redraw_loop(self):
        """GPS 地圖：1Hz 節流（tile 地圖不能每包都重畫），比照 gui_monitor.py map_redraw_loop。"""
        try:
            if not self.root.winfo_exists():
                return
            if self.map_view is not None and self.map_view.is_dirty():
                self.map_view.redraw()
            self.root.after(1000, self.map_redraw_loop)
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
        self.ln_alt_peer_baro.set_data(*clipped(self.ts_peer_baro))
        self.ln_alt_peer_vf.set_data(*clipped(self.ts_peer_vf_alt))
        self.ln_vz_ekf.set_data(*clipped(self.ts_vz_ekf))
        self.ln_vz_vf.set_data(*clipped(self.ts_vz_vf))
        self.ln_vz_peer_vf.set_data(*clipped(self.ts_peer_vf_vz))
        for ax in (self.ax_alt, self.ax_vz):
            ax.set_xlim(t0, max(now, t0 + 1))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        self.alt_canvas.draw_idle()

    def on_close(self):
        self.disconnect_serial()
        # 明確 destroy 掉還存活的 Toplevel + matplotlib 元件，再 sys.exit(0)——帶著存活的
        # matplotlib Figure 直接 exit 在 macOS 上偶發卡死，這裡先清乾淨當保險。
        for attr in ("chart_win", "lora_win", "flash_win"):
            win = getattr(self, attr, None)
            try:
                if win is not None and win.winfo_exists():
                    win.destroy()
            except Exception:
                pass
        for attr in ("att_view", "map_view"):
            view = getattr(self, attr, None)
            try:
                if view is not None:
                    view.destroy()
            except Exception:
                pass
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
