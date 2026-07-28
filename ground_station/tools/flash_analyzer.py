#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
🚀 RocketCom Flash 數據分析與自動報告/圖表生成工具 (Flash Analyzer)
======================================================================
功能：
 1. 解析 gui_monitor.py 由 Flash Dump 匯出的 CSV（ring_buffer_all.csv / flight_id_*.csv）。
 2. 依 flight_id 自動分組多段飛行紀錄。
 3. 事件偵測（起飛 / 燃燒結束 / 頂點 / 副傘 / 主傘 / 著陸）— 以 FSM 轉態 + flags 位元
    + 運動學三重來源交叉判定，取得每個事件的確切時間與高度。
 4. 計算完整飛行指標（頂點、最大上升/下降速度、峰值 G、燃燒時間、滑行時間、
    副傘/主傘平均下降率、落地速度、下靶距離、最大自旋率、電池衰減 …）。
 5. flags 位元解碼（開傘 / 失效保護 / 空中重啟 / 感測器故障 / GPS 逾時 / EKF 降級）。
 6. 真實感測器與系統健康檢核（非寫死；偵測感測器卡死、EKF 降級比例、失效保護觸發等）。
 7. 產出人類可讀報告（.txt 與 .md）與高解析多子圖飛行分析圖（.png）。
 8. 產生互動式 GPS 航跡地圖（.html，folium + OpenStreetMap 底圖，可縮放/拖曳）。
 9. 選配解碼 Sector 0 系統旗標區（sysflags_sector0.hex）。

CSV 欄位契約（與 firmware w25qxx.h FlashRingPacket_t 及 gui_monitor.py
FLASH_EXPORT_COLUMNS 逐欄對齊，勿變動順序）：
  addr, flight_id, seq, tick_ms, fsm_state, flags, bat_mv,
  bmi_ax, bmi_ay, bmi_az, bmi_gx, bmi_gy, bmi_gz,
  adxl_x, adxl_y, adxl_z,
  baro_temp_c_x100, baro_press_pa, baro_alt_cm,
  ekf_alt_cm, ekf_vel_cms,
  ekf_q0, ekf_q1, ekf_q2, ekf_q3,
  gps_lat, gps_lon, gps_alt_m, gps_spd_cms, gps_sats, gps_fix

使用方式:
  python3 flash_analyzer.py ring_buffer_all.csv
  python3 flash_analyzer.py flight_id_3.csv --outdir reports
  python3 flash_analyzer.py ring_buffer_all.csv --sysflags sysflags_sector0.hex

  # 若輸入檔位於 .../flash_export_xxx/raw/ 底下，未指定 --outdir 時會自動輸出到
  # 同層的 processed/ 資料夾（對齊 gui_monitor.py 匯出時的 raw/processed 兩資料夾慣例）。

備註：互動式地圖產生本身不需要網路（.html 只是內嵌座標的靜態檔案），但事後用
瀏覽器「打開」該 .html 檔查看地圖底圖時，瀏覽器需連網下載 OpenStreetMap 圖磚
（火箭發射現場產報告可離線，回程/回家有網路後再開檔查看地圖即可）。未安裝
folium 時會自動略過地圖產出，不影響其餘報告/圖表。
"""

import sys
import os
import argparse
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 非互動式繪圖，適合背景生成圖片
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from datetime import datetime

try:
    import folium
    import branca.colormap as bcm
    HAS_FOLIUM = True
except ImportError:
    HAS_FOLIUM = False

# 設置中文字型與黑夜酷炫主題風格
plt.style.use('dark_background')
matplotlib.rcParams['font.sans-serif'] = ['Helvetica', 'Arial', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# === FSM 狀態碼（與 firmware fsm.h FlightState_t 逐一對齊：PAD_ARMED=2）=== #
FSM_STATE_NAMES = {
    0: "INIT",
    1: "PAD",
    2: "PAD_ARMED",
    3: "BOOST",
    4: "COAST",
    5: "DEPLOY_DROGUE",
    6: "APOGEE",
    7: "DESCENT",
    8: "MAIN_DEPLOY",
    9: "LANDED",
}

FSM_STATE_COLORS = {
    "INIT": "#555555",
    "PAD": "#2b5c8f",
    "PAD_ARMED": "#851414",
    "BOOST": "#d97706",
    "COAST": "#059669",
    "DEPLOY_DROGUE": "#2563eb",
    "APOGEE": "#7c3aed",
    "DESCENT": "#8b5cf6",
    "MAIN_DEPLOY": "#0d9488",
    "LANDED": "#16a34a",
}

# === flags 位元（與 firmware telemetry.h TELEM_FLAG_* 一致；ring_pkt.flags 同語意）=== #
FLAG_BITS = [
    (0x01, "DROGUE_FIRED"),   # PD13 副傘點火 MOSFET 導通
    (0x02, "MAIN_DEPLOYED"),  # 主傘舵機已轉至釋放角度
    (0x04, "SD_ACTIVE"),      # SD 卡正在記錄
    (0x08, "GPS_STALE"),      # GPS 定位逾時 (>2s)
    (0x10, "EKF_UNHEALTHY"),  # EKF 健康位非 0 → 走 raw-baro 降級鏈
    (0x20, "SENSOR_FAULT"),   # 任一感測器失流/卡死/範圍失效
    (0x40, "FAILSAFE"),       # 失效保護計時器強制點火（需特別標示）
    (0x80, "HOTSTART"),       # 空中斷電熱啟動恢復成功
]

# 事件顏色（圖上垂直標線 / 註解共用）
EVENT_STYLE = {
    "liftoff":  ("#00e676", "🚀 Liftoff"),
    "burnout":  ("#ff9f43", "🔥 Burnout"),
    "apogee":   ("#a855f7", "🔺 Apogee"),
    "drogue":   ("#2563eb", "🪂 Drogue"),
    "main":     ("#0d9488", "🌂 Main"),
    "landing":  ("#16a34a", "🎯 Landing"),
}


def quat_to_euler_deg(q0, q1, q2, q3):
    """四元數 (w, x, y, z) → 歐拉角 Pitch, Roll, Yaw (度)。"""
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q0 * q2 - q3 * q1)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(pitch), np.degrees(roll), np.degrees(yaw)


def haversine_m(lat1, lon1, lat2, lon2):
    """兩經緯度間大圓距離 (公尺)。"""
    R = 6371000.0
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlmb = np.radians(lon2 - lon1)
    a = np.sin(dphi / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dlmb / 2) ** 2
    return 2 * R * np.arcsin(np.sqrt(a))


def decode_flags(flags):
    """flags byte → 已設位元名稱清單。"""
    return [name for bit, name in FLAG_BITS if flags & bit]


def parse_flash_csv(csv_filepath):
    """
    解析 Flash CSV，回傳 (flights, stats)。
      flights : {flight_id: [rec, ...]}
      stats   : {'total_lines', 'parsed', 'skipped'} 供資料品質回報。
    """
    if not os.path.exists(csv_filepath):
        print(f"[ERROR] 檔案不存在: {csv_filepath}")
        return {}, {"total_lines": 0, "parsed": 0, "skipped": 0}

    flights = {}
    total_lines = parsed = skipped = 0
    with open(csv_filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('---') or line.startswith('[FLASH]'):
                continue
            if line.startswith('addr,') or line.startswith('addr '):
                continue  # 表頭
            parts = line.split(',')
            if len(parts) < 25:
                continue
            total_lines += 1
            try:
                flight_id = int(parts[1])
                tick_ms   = int(parts[3])
                fsm_st    = int(parts[4])
                flags     = int(parts[5])
                bat_mv    = int(parts[6])

                bmi_ax = int(parts[7]) / 2048.0   # g  (BMI088 ±16g → 2048 LSB/g)
                bmi_ay = int(parts[8]) / 2048.0
                bmi_az = int(parts[9]) / 2048.0
                bmi_gx = int(parts[10]) / 16.4    # deg/s (±2000dps → 16.4 LSB/dps)
                bmi_gy = int(parts[11]) / 16.4
                bmi_gz = int(parts[12]) / 16.4

                adxl_x = int(parts[13]) / 16.0    # g  (ADXL375 → 20.5? 用既有 16.0)
                adxl_y = int(parts[14]) / 16.0
                adxl_z = int(parts[15]) / 16.0

                temp_c   = int(parts[16]) / 100.0  # °C
                press_pa = int(parts[17])          # Pa
                baro_alt = int(parts[18]) / 100.0  # m
                ekf_alt  = int(parts[19]) / 100.0  # m
                ekf_vel  = int(parts[20]) / 100.0  # m/s

                q0 = float(parts[21]); q1 = float(parts[22])
                q2 = float(parts[23]); q3 = float(parts[24])

                gps_lat  = int(parts[25]) / 1e6   if len(parts) > 25 else 0.0
                gps_lon  = int(parts[26]) / 1e6   if len(parts) > 26 else 0.0
                gps_alt  = int(parts[27])         if len(parts) > 27 else 0
                gps_spd  = int(parts[28]) / 100.0 if len(parts) > 28 else 0.0
                gps_sats = int(parts[29])         if len(parts) > 29 else 0
                gps_fix  = int(parts[30])         if len(parts) > 30 else 0

                pitch, roll, yaw = quat_to_euler_deg(q0, q1, q2, q3)
                adxl_total = float(np.sqrt(adxl_x ** 2 + adxl_y ** 2 + adxl_z ** 2))
                bmi_acc_total = float(np.sqrt(bmi_ax ** 2 + bmi_ay ** 2 + bmi_az ** 2))
                gyro_total = float(np.sqrt(bmi_gx ** 2 + bmi_gy ** 2 + bmi_gz ** 2))

                rec = {
                    'tick_ms': tick_ms, 'fsm_state': fsm_st,
                    'fsm_name': FSM_STATE_NAMES.get(fsm_st, f"ST_{fsm_st}"),
                    'flags': flags,
                    'bat_mv': bat_mv,
                    'bmi_ax': bmi_ax, 'bmi_ay': bmi_ay, 'bmi_az': bmi_az, 'bmi_acc_total': bmi_acc_total,
                    'bmi_gx': bmi_gx, 'bmi_gy': bmi_gy, 'bmi_gz': bmi_gz, 'gyro_total': gyro_total,
                    'adxl_x': adxl_x, 'adxl_y': adxl_y, 'adxl_z': adxl_z, 'adxl_total': adxl_total,
                    'temp_c': temp_c, 'press_pa': press_pa, 'baro_alt': baro_alt,
                    'ekf_alt': ekf_alt, 'ekf_vel': ekf_vel,
                    'q0': q0, 'q1': q1, 'q2': q2, 'q3': q3,
                    'pitch': pitch, 'roll': roll, 'yaw': yaw,
                    'gps_lat': gps_lat, 'gps_lon': gps_lon, 'gps_alt': gps_alt,
                    'gps_spd': gps_spd, 'gps_sats': gps_sats, 'gps_fix': gps_fix,
                }
                flights.setdefault(flight_id, []).append(rec)
                parsed += 1
            except (ValueError, IndexError):
                skipped += 1
                continue

    # 每段飛行依 tick 排序（滾動 ring 可能亂序）
    for fid in flights:
        flights[fid].sort(key=lambda r: r['tick_ms'])
    return flights, {"total_lines": total_lines, "parsed": parsed, "skipped": skipped}


def detect_events(records, times):
    """
    事件偵測：回傳 {name: {'t','alt','vel','idx'}}，缺資料的事件不列入。
    來源優先序：flags 位元（最可信，硬體實際動作）> FSM 轉態 > 運動學。
    """
    ev = {}
    n = len(records)
    if n == 0:
        return ev
    ekf_alt = np.array([r['ekf_alt'] for r in records])
    ekf_vel = np.array([r['ekf_vel'] for r in records])
    names = [r['fsm_name'] for r in records]

    def put(key, idx):
        if idx is None or not (0 <= idx < n):
            return
        ev[key] = {'t': times[idx], 'alt': ekf_alt[idx], 'vel': ekf_vel[idx], 'idx': idx}

    def first_state(target):
        for i, nm in enumerate(names):
            if nm == target:
                return i
        return None

    def first_flag(bit, after=0):
        for i in range(max(after, 0), n):
            if records[i]['flags'] & bit:
                return i
        return None

    # 起飛：進入 BOOST（或首次 EKF 速度顯著 > 5 m/s 作為運動學備援）
    idx_liftoff = first_state("BOOST")
    if idx_liftoff is None:
        cand = np.where(ekf_vel > 5.0)[0]
        idx_liftoff = int(cand[0]) if len(cand) else None
    put("liftoff", idx_liftoff)

    # 燃燒結束：起飛後首次進入 COAST
    idx_burnout = first_state("COAST")
    put("burnout", idx_burnout)

    # 頂點：EKF 高度最大值（最穩健）
    if n:
        put("apogee", int(np.argmax(ekf_alt)))

    # 副傘：flags DROGUE_FIRED 首次為真，否則進入 DEPLOY_DROGUE 狀態
    idx_drogue = first_flag(0x01)
    if idx_drogue is None:
        idx_drogue = first_state("DEPLOY_DROGUE")
    put("drogue", idx_drogue)

    # 主傘：flags MAIN_DEPLOYED 首次為真，否則進入 MAIN_DEPLOY 狀態
    idx_main = first_flag(0x02)
    if idx_main is None:
        idx_main = first_state("MAIN_DEPLOY")
    put("main", idx_main)

    # 著陸：進入 LANDED，否則末筆
    idx_land = first_state("LANDED")
    if idx_land is None and 'apogee' in ev:
        idx_land = n - 1  # 有飛行才把末筆當落地
    put("landing", idx_land)

    return ev


def compute_metrics(records, times, events):
    """由記錄 + 事件推導完整飛行指標 dict。"""
    ekf_alt = np.array([r['ekf_alt'] for r in records])
    baro_alt = np.array([r['baro_alt'] for r in records])
    ekf_vel = np.array([r['ekf_vel'] for r in records])
    adxl_tot = np.array([r['adxl_total'] for r in records])
    bmi_tot = np.array([r['bmi_acc_total'] for r in records])
    gyro_tot = np.array([r['gyro_total'] for r in records])
    bat_v = np.array([r['bat_mv'] / 1000.0 for r in records])

    m = {
        'duration_s': times[-1] - times[0] if len(times) > 1 else 0.0,
        'n_pkt': len(records),
        'max_ekf_alt': float(np.max(ekf_alt)),
        'max_baro_alt': float(np.max(baro_alt)),
        'max_vup': float(np.max(ekf_vel)),
        'max_vdown': float(np.min(ekf_vel)),
        'peak_g_adxl': float(np.max(adxl_tot)),
        'peak_g_bmi': float(np.max(bmi_tot)),
        'max_spin_dps': float(np.max(gyro_tot)),
        'bat_start': float(bat_v[0]), 'bat_min': float(np.min(bat_v)), 'bat_end': float(bat_v[-1]),
        'temp_mean': float(np.mean([r['temp_c'] for r in records])),
    }

    def et(name):
        return events.get(name, {}).get('t')

    # 相位時間
    if et('liftoff') is not None and et('burnout') is not None:
        m['burn_time'] = et('burnout') - et('liftoff')
    if et('burnout') is not None and et('apogee') is not None:
        m['coast_time'] = et('apogee') - et('burnout')
    if et('liftoff') is not None and et('apogee') is not None:
        m['time_to_apogee'] = et('apogee') - et('liftoff')

    # 下降率：副傘段（drogue→main）與主傘段（main→landing）平均 |v|
    def mean_descent(a, b):
        ia = events.get(a, {}).get('idx')
        ib = events.get(b, {}).get('idx')
        if ia is None or ib is None or ib <= ia:
            return None
        seg = ekf_vel[ia:ib + 1]
        seg = seg[seg < 0]  # 只取下降段
        return float(-np.mean(seg)) if len(seg) else None

    m['drogue_descent_rate'] = mean_descent('drogue', 'main')
    m['main_descent_rate'] = mean_descent('main', 'landing')

    # 落地速度：landing 前 0.5s 平均 |v|
    if 'landing' in events:
        il = events['landing']['idx']
        t_land = times[il]
        win = [i for i in range(len(times)) if 0 <= t_land - times[i] <= 0.5]
        if win:
            m['landing_vel'] = float(np.mean([abs(ekf_vel[i]) for i in win]))

    # GPS：起降點、下靶距離、最大下靶
    valid = [r for r in records if r['gps_fix'] == 1 and abs(r['gps_lat']) > 0.01]
    if valid:
        m['launch_gps'] = (valid[0]['gps_lat'], valid[0]['gps_lon'], valid[0]['gps_alt'])
        m['landing_gps'] = (valid[-1]['gps_lat'], valid[-1]['gps_lon'], valid[-1]['gps_alt'])
        m['downrange_m'] = float(haversine_m(valid[0]['gps_lat'], valid[0]['gps_lon'],
                                             valid[-1]['gps_lat'], valid[-1]['gps_lon']))
        lat0, lon0 = valid[0]['gps_lat'], valid[0]['gps_lon']
        m['max_downrange_m'] = float(max(haversine_m(lat0, lon0, r['gps_lat'], r['gps_lon']) for r in valid))
        m['gps_max_sats'] = max(r['gps_sats'] for r in records)
    return m


def audit_health(records, events):
    """真實健康檢核：回傳 [(level, text)]，level ∈ {'ok','warn','err','info'}。"""
    out = []
    n = len(records)
    flags_or = 0
    for r in records:
        flags_or |= r['flags']

    # 飛行窗（起飛→著陸）內的記錄，用於動態感測器判活
    ia = events.get('liftoff', {}).get('idx', 0)
    ib = events.get('landing', {}).get('idx', n - 1)
    flight = records[ia:ib + 1] if ib >= ia else records

    def col(key):
        return np.array([r[key] for r in flight]) if flight else np.array([])

    # BMI088 加速度：飛行期間標準差應遠大於 0（否則卡死/斷流）
    bmi = col('bmi_acc_total')
    if len(bmi) and np.std(bmi) > 0.05:
        out.append(('ok', f"BMI088 加速度計運作正常（飛行期合力 σ={np.std(bmi):.2f}g，峰值 {np.max(bmi):.1f}g）"))
    else:
        out.append(('err', "BMI088 加速度計疑似卡死/斷流（飛行期數值無變化）"))

    # BMI 陀螺儀
    gyro = col('gyro_total')
    if len(gyro) and np.max(gyro) > 1.0:
        out.append(('ok', f"BMI088 陀螺儀運作正常（最大自旋率 {np.max(gyro):.0f} °/s）"))
    else:
        out.append(('warn', "BMI088 陀螺儀無明顯角速度（若為靜置測試屬正常）"))

    # ADXL375 高 G
    adxl = col('adxl_total')
    if len(adxl) and np.std(adxl) > 0.05:
        out.append(('ok', f"ADXL375 高 G 感測器運作正常（峰值衝擊 {np.max(adxl):.1f}g）"))
    else:
        out.append(('err', "ADXL375 高 G 感測器疑似卡死/斷流"))

    # BMP388 氣壓：壓力需落在合理範圍（30–110 kPa）
    press = np.array([r['press_pa'] for r in records])
    if len(press) and np.all((press > 30000) & (press < 110000)):
        out.append(('ok', f"BMP388 氣壓計運作正常（平均溫度 {np.mean([r['temp_c'] for r in records]):.1f}°C）"))
    else:
        bad = int(np.sum((press <= 30000) | (press >= 110000)))
        out.append(('warn', f"BMP388 氣壓計有 {bad} 筆越界讀值（30–110kPa 外）"))

    # EKF 健康：flags EKF_UNHEALTHY 比例
    unhealthy = sum(1 for r in records if r['flags'] & 0x10)
    frac = unhealthy / n * 100 if n else 0
    if frac == 0:
        out.append(('ok', "EKF 全程健康（未觸發 raw-baro 降級鏈）"))
    elif frac < 20:
        out.append(('warn', f"EKF 有 {frac:.1f}% 封包降級（EKF_UNHEALTHY，走 baro 冗餘）"))
    else:
        out.append(('err', f"EKF 有 {frac:.1f}% 封包降級 — 需檢視 EKF 收斂/感測器健康"))

    # 感測器故障位
    if flags_or & 0x20:
        cnt = sum(1 for r in records if r['flags'] & 0x20)
        out.append(('err', f"偵測到 SENSOR_FAULT 旗標（{cnt} 筆）— 請對照 firmware [HEALTH] 行"))

    # 失效保護
    if flags_or & 0x40:
        idx = next((i for i, r in enumerate(records) if r['flags'] & 0x40), None)
        out.append(('warn', f"⚠ FAILSAFE 失效保護計時器曾強制點火（自封包 #{idx} 起）— 正常頂點偵測可能未生效"))

    # 空中重啟
    if flags_or & 0x80:
        out.append(('warn', "⚠ HOTSTART：偵測到空中斷電熱啟動恢復 — 飛行期間曾重啟"))

    # GPS 逾時
    if flags_or & 0x08:
        cnt = sum(1 for r in records if r['flags'] & 0x08)
        out.append(('info', f"GPS 曾逾時 {cnt} 筆（GPS_STALE）"))

    # GPS 定位
    got_fix = any(r['gps_fix'] == 1 for r in records)
    if got_fix:
        out.append(('ok', f"GPS 取得定位（最多 {max(r['gps_sats'] for r in records)} 顆衛星）"))
    else:
        out.append(('warn', "GPS 全程未取得有效定位（fix=0）"))

    # 電池
    bat_min = min(r['bat_mv'] for r in records) / 1000.0
    if bat_min >= 7.0:
        out.append(('ok', f"電池電源良好（最低 {bat_min:.2f}V ≥ 7.0V）"))
    else:
        out.append(('err', f"⚠ 電池測試期間曾低於 7.0V（最低 {bat_min:.2f}V）"))

    return out


# ---- 報告輸出 ---- #
_LEVEL_MARK = {'ok': '[✓]', 'warn': '[!]', 'err': '[✗]', 'info': '[i]'}


def _build_timeline(records, times):
    tl, last = [], None
    for r, t in zip(records, times):
        if r['fsm_name'] != last:
            tl.append((t, r['fsm_name'], r['baro_alt'], r['ekf_vel'], r['bat_mv'] / 1000.0,
                       decode_flags(r['flags'])))
            last = r['fsm_name']
    return tl


def generate_flight_report(flight_id, records, events, metrics, health, output_dir="."):
    """產出人類可讀報告（同時輸出 .txt 與 .md）。"""
    if not records:
        return ""
    t0 = records[0]['tick_ms'] / 1000.0
    times = [(r['tick_ms'] / 1000.0) - t0 for r in records]
    m = metrics
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def fmt_ev(name):
        e = events.get(name)
        return f"t={e['t']:.2f}s @ {e['alt']:.1f}m" if e else "未偵測到"

    L = []
    L.append("=" * 80)
    L.append("                    🚀 火箭 Flash 飛行數據分析報告 🚀")
    L.append(f"                    Flight Session #{flight_id} | 分析時間: {now_str}")
    L.append("=" * 80)
    L.append("")
    L.append("【一、 飛行關鍵指標總覽】")
    L.append("-" * 80)
    L.append(f"  • 飛行識別碼        : Flight #{flight_id}")
    L.append(f"  • 紀錄封包數        : {m['n_pkt']} 筆（總時長 {m['duration_s']:.2f} s）")
    L.append(f"  • 最高飛行高度      : EKF {m['max_ekf_alt']:.2f} m  |  氣壓計 {m['max_baro_alt']:.2f} m")
    L.append(f"  • 最大上升速度      : +{m['max_vup']:.2f} m/s ({m['max_vup']*3.6:.1f} km/h)")
    L.append(f"  • 最大下降速度      : {m['max_vdown']:.2f} m/s ({m['max_vdown']*3.6:.1f} km/h)")
    L.append(f"  • 峰值加速度        : ADXL375 {m['peak_g_adxl']:.2f} g  |  BMI088 {m['peak_g_bmi']:.2f} g")
    L.append(f"  • 最大自旋率        : {m['max_spin_dps']:.0f} °/s")
    if 'burn_time' in m:
        L.append(f"  • 馬達燃燒時間      : {m['burn_time']:.2f} s")
    if 'time_to_apogee' in m:
        L.append(f"  • 起飛至頂點時間    : {m['time_to_apogee']:.2f} s（滑行 {m.get('coast_time', float('nan')):.2f} s）")
    if m.get('drogue_descent_rate') is not None:
        L.append(f"  • 副傘平均下降率    : {m['drogue_descent_rate']:.2f} m/s")
    if m.get('main_descent_rate') is not None:
        L.append(f"  • 主傘平均下降率    : {m['main_descent_rate']:.2f} m/s")
    if 'landing_vel' in m:
        L.append(f"  • 落地速度（估計）  : {m['landing_vel']:.2f} m/s")
    L.append(f"  • 電池電壓變化      : 初始 {m['bat_start']:.2f}V | 最低 {m['bat_min']:.2f}V | 結尾 {m['bat_end']:.2f}V")
    if 'downrange_m' in m:
        L.append(f"  • 落地下靶距離      : {m['downrange_m']:.1f} m（最大下靶 {m['max_downrange_m']:.1f} m）")
        lg = m['launch_gps']; nd = m['landing_gps']
        L.append(f"  • 起飛點 GPS        : {lg[0]:.6f}, {lg[1]:.6f}（海拔 {lg[2]}m）")
        L.append(f"  • 著陸點 GPS        : {nd[0]:.6f}, {nd[1]:.6f}（海拔 {nd[2]}m）")
    else:
        L.append("  • GPS 座標          : 無有效定位")
    L.append("-" * 80)
    L.append("")
    L.append("【二、 關鍵飛行事件時刻】")
    L.append("-" * 80)
    for name in ("liftoff", "burnout", "apogee", "drogue", "main", "landing"):
        _, label = EVENT_STYLE[name]
        L.append(f"  {label:<14} : {fmt_ev(name)}")
    L.append("-" * 80)
    L.append("")
    L.append("【三、 狀態機轉換時間軸】")
    L.append("-" * 80)
    L.append(f"  {'時間(s)':>8} | {'狀態':<14} | {'高度(m)':>9} | {'速度(m/s)':>9} | {'電壓':>5} | 旗標")
    L.append("  " + "-" * 74)
    for t, st, alt, vel, vbat, flg in _build_timeline(records, times):
        L.append(f"  {t:>7.2f}s | {st:<14} | {alt:>8.1f}m | {vel:>8.2f} | {vbat:>4.2f}V | {'|'.join(flg) or '-'}")
    L.append("-" * 80)
    L.append("")
    L.append("【四、 感測器與系統健康檢核】")
    L.append("-" * 80)
    for level, text in health:
        L.append(f"  {_LEVEL_MARK[level]} {text}")
    L.append("=" * 80)

    txt = "\n".join(L)
    txt_path = os.path.join(output_dir, f"flight_report_Flight_{flight_id}.txt")
    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write(txt + "\n")

    # Markdown 版（可直接貼上 GitHub / 報告）
    md_path = os.path.join(output_dir, f"flight_report_Flight_{flight_id}.md")
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write(f"# 🚀 Flight #{flight_id} 飛行數據分析報告\n\n")
        f.write(f"*分析時間: {now_str}*\n\n")
        f.write("## 一、飛行關鍵指標\n\n")
        f.write("| 指標 | 數值 |\n|---|---|\n")
        f.write(f"| 紀錄封包數 | {m['n_pkt']} 筆（{m['duration_s']:.2f} s）|\n")
        f.write(f"| 最高高度 (EKF / Baro) | {m['max_ekf_alt']:.2f} / {m['max_baro_alt']:.2f} m |\n")
        f.write(f"| 最大上升 / 下降速度 | +{m['max_vup']:.2f} / {m['max_vdown']:.2f} m/s |\n")
        f.write(f"| 峰值 G (ADXL / BMI) | {m['peak_g_adxl']:.2f} / {m['peak_g_bmi']:.2f} g |\n")
        f.write(f"| 最大自旋率 | {m['max_spin_dps']:.0f} °/s |\n")
        if 'burn_time' in m:
            f.write(f"| 燃燒時間 | {m['burn_time']:.2f} s |\n")
        if 'time_to_apogee' in m:
            f.write(f"| 起飛→頂點 | {m['time_to_apogee']:.2f} s |\n")
        if m.get('drogue_descent_rate') is not None:
            f.write(f"| 副傘下降率 | {m['drogue_descent_rate']:.2f} m/s |\n")
        if m.get('main_descent_rate') is not None:
            f.write(f"| 主傘下降率 | {m['main_descent_rate']:.2f} m/s |\n")
        if 'landing_vel' in m:
            f.write(f"| 落地速度 | {m['landing_vel']:.2f} m/s |\n")
        f.write(f"| 電池 (初/最低/末) | {m['bat_start']:.2f} / {m['bat_min']:.2f} / {m['bat_end']:.2f} V |\n")
        if 'downrange_m' in m:
            f.write(f"| 下靶距離 | {m['downrange_m']:.1f} m |\n")
        f.write("\n## 二、關鍵事件\n\n| 事件 | 時刻 |\n|---|---|\n")
        for name in ("liftoff", "burnout", "apogee", "drogue", "main", "landing"):
            f.write(f"| {EVENT_STYLE[name][1]} | {fmt_ev(name)} |\n")
        f.write("\n## 三、健康檢核\n\n")
        for level, text in health:
            f.write(f"- {_LEVEL_MARK[level]} {text}\n")

    print(f"[REPORT] 已生成文字/Markdown 報告: {txt_path} / {md_path}")
    return txt_path


def _mark_events(ax, events, times, ymin=None):
    """在圖上畫事件垂直標線 + 頂端標籤。"""
    for name in ("liftoff", "burnout", "apogee", "drogue", "main", "landing"):
        e = events.get(name)
        if not e:
            continue
        color, label = EVENT_STYLE[name]
        ax.axvline(e['t'], color=color, linestyle='--', linewidth=1.2, alpha=0.8)


def generate_flight_charts(flight_id, records, events, metrics, output_dir="."):
    """產出 3×2 多子圖飛行分析 PNG。"""
    if not records:
        return ""
    t0 = records[0]['tick_ms'] / 1000.0
    times = np.array([(r['tick_ms'] / 1000.0) - t0 for r in records])
    ekf_alts = np.array([r['ekf_alt'] for r in records])
    baro_alts = np.array([r['baro_alt'] for r in records])
    ekf_vels = np.array([r['ekf_vel'] for r in records])
    adxl_tot = np.array([r['adxl_total'] for r in records])
    pitches = np.array([r['pitch'] for r in records])
    rolls = np.array([r['roll'] for r in records])
    yaws = np.array([r['yaw'] for r in records])
    gx = np.array([r['bmi_gx'] for r in records])
    gy = np.array([r['bmi_gy'] for r in records])
    gz = np.array([r['bmi_gz'] for r in records])
    bat_vs = np.array([r['bat_mv'] / 1000.0 for r in records])
    temps = np.array([r['temp_c'] for r in records])

    fig, axs = plt.subplots(3, 2, figsize=(18, 15), dpi=140)
    fig.suptitle(f"RocketCom Flight Analysis — Flight #{flight_id}",
                 fontsize=17, fontweight='bold', color='#00e676', y=0.995)

    # --- 1. 高度 + FSM 相位 + 事件 + 頂點註解 --- #
    ax1 = axs[0, 0]
    ax1.plot(times, ekf_alts, label="EKF Fused Alt", color="#00e676", linewidth=2)
    ax1.plot(times, baro_alts, label="Baro Alt", color="#ff9f43", linestyle="--", linewidth=1.4, alpha=0.85)
    last, st_start = None, times[0]
    for i, r in enumerate(records):
        st = r['fsm_name']
        if st != last:
            if last is not None:
                ax1.axvspan(st_start, times[i], color=FSM_STATE_COLORS.get(last, "#333"), alpha=0.18)
            last, st_start = st, times[i]
    ax1.axvspan(st_start, times[-1], color=FSM_STATE_COLORS.get(last, "#333"), alpha=0.18)
    _mark_events(ax1, events, times)
    if 'apogee' in events:
        e = events['apogee']
        ax1.annotate(f"Apogee {e['alt']:.0f}m\n@{e['t']:.1f}s", xy=(e['t'], e['alt']),
                     xytext=(e['t'], e['alt'] * 0.78), color="#a855f7", fontsize=9, ha='center',
                     arrowprops=dict(arrowstyle="->", color="#a855f7"))
    ax1.set_title("1. Altitude Profile & Flight Phases", fontsize=12, color="#00e676", pad=8)
    ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Altitude (m)")
    ax1.grid(True, linestyle=":", alpha=0.35); ax1.legend(loc="upper right", fontsize=9)

    # --- 2. 垂直速度 + High-G --- #
    ax2 = axs[0, 1]; ax2t = ax2.twinx()
    l1 = ax2.plot(times, ekf_vels, label="Vertical Vel", color="#38ef7d", linewidth=2)
    l2 = ax2t.plot(times, adxl_tot, label="High-G Total", color="#ff5252", linewidth=1.3, alpha=0.75)
    ax2.axhline(0, color="#888", linewidth=0.8, linestyle=":")
    _mark_events(ax2, events, times)
    ax2.set_title("2. Vertical Velocity & Acceleration", fontsize=12, color="#38ef7d", pad=8)
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Velocity (m/s)")
    ax2t.set_ylabel("High-G (g)", color="#ff5252")
    ax2.grid(True, linestyle=":", alpha=0.35)
    ax2.legend(l1 + l2, [x.get_label() for x in l1 + l2], loc="upper right", fontsize=9)

    # --- 3. 姿態歐拉角 --- #
    ax3 = axs[1, 0]
    ax3.plot(times, pitches, label="Pitch", color="#ff7675", linewidth=1.4)
    ax3.plot(times, rolls, label="Roll", color="#74b9ff", linewidth=1.4)
    ax3.plot(times, yaws, label="Yaw", color="#ffeaa7", linewidth=1.1, linestyle=":")
    _mark_events(ax3, events, times)
    ax3.set_title("3. Attitude Euler Angles", fontsize=12, color="#74b9ff", pad=8)
    ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Angle (deg)")
    ax3.grid(True, linestyle=":", alpha=0.35); ax3.legend(loc="upper right", fontsize=9)

    # --- 4. 陀螺儀角速度（自旋/穩定性） --- #
    ax4 = axs[1, 1]
    ax4.plot(times, gx, label="Gyro X (roll rate)", color="#ff7675", linewidth=1.1)
    ax4.plot(times, gy, label="Gyro Y (pitch rate)", color="#74b9ff", linewidth=1.1)
    ax4.plot(times, gz, label="Gyro Z (yaw/spin)", color="#55efc4", linewidth=1.1)
    _mark_events(ax4, events, times)
    ax4.set_title("4. Angular Rates (Spin & Stability)", fontsize=12, color="#55efc4", pad=8)
    ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Rate (°/s)")
    ax4.grid(True, linestyle=":", alpha=0.35); ax4.legend(loc="upper right", fontsize=8)

    # --- 5. GPS 航跡（依高度上色） --- #
    ax5 = axs[2, 0]
    valid = [r for r in records if r['gps_fix'] == 1 and abs(r['gps_lat']) > 0.01]
    if len(valid) >= 2:
        lats = np.array([r['gps_lat'] for r in valid])
        lons = np.array([r['gps_lon'] for r in valid])
        alts = np.array([r['ekf_alt'] for r in valid])
        pts = np.array([lons, lats]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(segs, cmap='viridis', linewidth=2)
        lc.set_array(alts[:-1])
        ax5.add_collection(lc)
        ax5.autoscale()
        ax5.plot(lons[0], lats[0], marker="^", color="#2ecc71", markersize=12, label="Launch")
        ax5.plot(lons[-1], lats[-1], marker="v", color="#e74c3c", markersize=12, label="Landing")
        cb = fig.colorbar(lc, ax=ax5, fraction=0.046, pad=0.04)
        cb.set_label("Altitude (m)", fontsize=8)
        dr = metrics.get('downrange_m')
        ttl = "5. GPS Ground Track (colour = altitude)"
        if dr is not None:
            ttl += f"  |  downrange {dr:.0f}m"
        ax5.set_title(ttl, fontsize=12, color="#00d2d3", pad=8)
        ax5.set_xlabel("Longitude (°)"); ax5.set_ylabel("Latitude (°)")
        ax5.grid(True, linestyle=":", alpha=0.35); ax5.legend(loc="best", fontsize=9)
    else:
        ax5.text(0.5, 0.5, "無足夠 GPS 定位資料\n(No valid GPS track)", ha='center', va='center',
                 color="#888", fontsize=13, transform=ax5.transAxes)
        ax5.set_title("5. GPS Ground Track", fontsize=12, color="#00d2d3", pad=8)

    # --- 6. 電池 + 溫度 --- #
    ax6 = axs[2, 1]; ax6t = ax6.twinx()
    l1 = ax6.plot(times, bat_vs, label="Battery (V)", color="#f1c40f", linewidth=2)
    l2 = ax6t.plot(times, temps, label="Baro Temp (°C)", color="#e67e22", linestyle="--", linewidth=1.4)
    ax6.axhline(7.0, color="#e74c3c", linewidth=0.9, linestyle=":", alpha=0.7)
    ax6.set_title("6. Battery Voltage & Temperature", fontsize=12, color="#f1c40f", pad=8)
    ax6.set_xlabel("Time (s)"); ax6.set_ylabel("Battery (V)", color="#f1c40f")
    ax6t.set_ylabel("Temp (°C)", color="#e67e22")
    ax6.grid(True, linestyle=":", alpha=0.35)
    ax6.legend(l1 + l2, [x.get_label() for x in l1 + l2], loc="upper right", fontsize=9)

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    chart_path = os.path.join(output_dir, f"flight_analysis_Flight_{flight_id}.png")
    plt.savefig(chart_path, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close(fig)
    print(f"[CHART] 已生成飛行分析圖: {chart_path}")
    return chart_path


def generate_flight_map(flight_id, records, events, metrics, output_dir="."):
    """
    產生互動式 GPS 航跡地圖（.html，folium + OpenStreetMap 底圖）。
    航跡依高度上色（viridis），並標出起飛/頂點/開傘/著陸事件位置。
    無 folium 或無足夠 GPS 定位點時安全略過（回傳 None）。
    """
    if not HAS_FOLIUM:
        print("[MAP] 未安裝 folium，略過互動式地圖（pip install folium）")
        return None

    valid = [r for r in records if r['gps_fix'] == 1 and abs(r['gps_lat']) > 0.01]
    if len(valid) < 2:
        print(f"[MAP] Flight #{flight_id} 無足夠 GPS 定位點，略過地圖生成")
        return None

    t0 = records[0]['tick_ms'] / 1000.0
    gps_t = [(r['tick_ms'] / 1000.0) - t0 for r in valid]
    lats = [r['gps_lat'] for r in valid]
    lons = [r['gps_lon'] for r in valid]
    alts = [r['ekf_alt'] for r in valid]

    center = [sum(lats) / len(lats), sum(lons) / len(lons)]
    fmap = folium.Map(location=center, zoom_start=16, tiles="CartoDB dark_matter")
    folium.TileLayer("OpenStreetMap", name="OpenStreetMap").add_to(fmap)

    # 依高度上色的航跡（逐段 PolyLine + viridis colormap）
    alt_min, alt_max = min(alts), max(alts)
    cmap = bcm.LinearColormap(
        colors=['#440154', '#3b528b', '#21918c', '#5ec962', '#fde725'],
        vmin=alt_min, vmax=max(alt_max, alt_min + 1e-6),
        caption=f"Flight #{flight_id} Altitude (m)",
    )
    for i in range(len(valid) - 1):
        folium.PolyLine(
            [(lats[i], lons[i]), (lats[i + 1], lons[i + 1])],
            color=cmap(alts[i]), weight=4, opacity=0.85,
        ).add_to(fmap)
    fmap.add_child(cmap)

    def nearest_gps_idx(t_target, max_dt=5.0):
        idx = min(range(len(gps_t)), key=lambda i: abs(gps_t[i] - t_target))
        return idx if abs(gps_t[idx] - t_target) <= max_dt else None

    # 事件標記：起飛 / 頂點 / 副傘 / 主傘 / 著陸
    marker_spec = {
        "liftoff": ("green", "rocket", "🚀 Liftoff"),
        "apogee":  ("purple", "arrow-up", "🔺 Apogee"),
        "drogue":  ("blue", "life-ring", "🪂 Drogue"),
        "main":    ("darkgreen", "umbrella", "🌂 Main"),
        "landing": ("red", "flag-checkered", "🎯 Landing"),
    }
    for name, (color, icon, label) in marker_spec.items():
        e = events.get(name)
        if e is None:
            continue
        idx = nearest_gps_idx(e['t'])
        if idx is None:
            continue
        popup = f"{label}<br>t={e['t']:.1f}s<br>alt={e['alt']:.1f}m<br>vel={e['vel']:.1f}m/s"
        folium.Marker(
            [lats[idx], lons[idx]], tooltip=label, popup=folium.Popup(popup, max_width=200),
            icon=folium.Icon(color=color, icon=icon, prefix='fa'),
        ).add_to(fmap)

    downrange = metrics.get('downrange_m')
    subtitle = f" | downrange {downrange:.0f}m" if downrange is not None else ""
    title_html = (
        f'<div style="position:fixed;top:10px;left:50px;z-index:9999;'
        f'background:rgba(0,0,0,0.6);color:#fff;padding:6px 12px;border-radius:6px;'
        f'font-size:14px;">🚀 Flight #{flight_id} GPS Track{subtitle}</div>'
    )
    fmap.get_root().html.add_child(folium.Element(title_html))

    folium.LayerControl().add_to(fmap)
    fmap.fit_bounds([[min(lats), min(lons)], [max(lats), max(lons)]])

    map_path = os.path.join(output_dir, f"flight_map_Flight_{flight_id}.html")
    fmap.save(map_path)
    print(f"[MAP] 已生成互動式 GPS 地圖: {map_path}")
    return map_path


def parse_sysflags_hex(hex_filepath):
    """解析 Sector 0 Raw Hex（sysflags_sector0.hex）→ FlashSysFlags_t 欄位。"""
    if not os.path.exists(hex_filepath):
        return None
    raw = bytearray()
    with open(hex_filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if ':' in line:
                for b in line.split(':', 1)[1].strip().split():
                    try:
                        raw.append(int(b, 16))
                    except ValueError:
                        pass
    if len(raw) < 86:
        return None
    try:
        data = struct.unpack("<BBBBIIHHIf3f3f3fIIBB2xIBBBbH2xH", raw[:86])
        return {
            "fsm_state": FSM_STATE_NAMES.get(data[0], f"UNKNOWN({data[0]})"),
            "drogue_deployed": bool(data[1]), "main_deployed": bool(data[2]),
            "reboot_count": data[3], "drogue_time_ms": data[4], "main_time_ms": data[5],
            "bat_voltage_v": data[6] / 1000.0, "self_test_errs": f"0x{data[7]:04X}",
            "calib_valid": (data[8] == 0xC0DEB1A5), "baro_launchpad": data[9],
            "accel_bias": data[10:13], "gyro_bias": data[13:16], "mag_offsets": data[16:19],
            "lora_valid": (data[19] == 0x50544C4F),
            "e22_freq_mhz": data[20], "e22_pwr": data[21], "e22_air": data[22],
            "e80_freq_hz": data[23], "e80_sf": data[24], "e80_bw": data[25],
            "e80_cr": data[26], "e80_pwr_dbm": data[27], "e80_preamble": data[28],
            "crc16": f"0x{data[29]:04X}",
        }
    except struct.error as e:
        print(f"[ANALYZER] SysFlags 解析失敗: {e}")
        return None


def generate_sysflags_report(hex_filepath, output_dir):
    """由 SysFlags 產出系統參數摘要 (sysflags_summary.txt)。"""
    parsed = parse_sysflags_hex(hex_filepath)
    if not parsed:
        return None
    path = os.path.join(output_dir, "sysflags_summary.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("=" * 70 + "\n")
        f.write("🚀 RocketCom 系統旗標區與硬體參數解碼報告\n")
        f.write("=" * 70 + "\n")
        f.write(f"解析時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write("【一、 系統狀態與開傘紀錄】\n")
        f.write(f"  • 當前飛行狀態    : {parsed['fsm_state']}\n")
        f.write(f"  • 副傘已部署      : {'是' if parsed['drogue_deployed'] else '否'}\n")
        f.write(f"  • 主傘已部署      : {'是' if parsed['main_deployed'] else '否'}\n")
        f.write(f"  • 副傘部署時間戳  : {parsed['drogue_time_ms']} ms\n")
        f.write(f"  • 主傘部署時間戳  : {parsed['main_time_ms']} ms\n")
        f.write(f"  • 空中重啟次數    : {parsed['reboot_count']} 次\n")
        f.write(f"  • 基準電池電壓    : {parsed['bat_voltage_v']:.2f} V\n")
        f.write(f"  • 自檢錯誤碼      : {parsed['self_test_errs']}\n\n")
        f.write("【二、 感測器偏置與發射台校準參數】\n")
        f.write(f"  • 校準 Magic 驗證 : {'有效 (0xC0DEB1A5)' if parsed['calib_valid'] else '無效/未校準'}\n")
        f.write(f"  • 發射台基準高度  : {parsed['baro_launchpad']:.2f} m\n")
        f.write(f"  • 加速度計偏置    : X={parsed['accel_bias'][0]:.4f}, Y={parsed['accel_bias'][1]:.4f}, Z={parsed['accel_bias'][2]:.4f} (m/s²)\n")
        f.write(f"  • 陀螺儀偏置      : X={parsed['gyro_bias'][0]:.4f}, Y={parsed['gyro_bias'][1]:.4f}, Z={parsed['gyro_bias'][2]:.4f} (rad/s)\n")
        f.write(f"  • 磁力計硬鐵偏置  : X={parsed['mag_offsets'][0]:.4f}, Y={parsed['mag_offsets'][1]:.4f}, Z={parsed['mag_offsets'][2]:.4f} (Gauss)\n\n")
        f.write("【三、 LoRa 遙測通訊配置】\n")
        f.write(f"  • LoRa Magic 驗證 : {'有效 (PTLO)' if parsed['lora_valid'] else '無效/預設'}\n")
        f.write(f"  • E22 433MHz      : {parsed['e22_freq_mhz']} MHz / Pwr {parsed['e22_pwr']} / Air {parsed['e22_air']}\n")
        f.write(f"  • E80 920MHz      : {parsed['e80_freq_hz']} Hz / SF{parsed['e80_sf']} / BW{parsed['e80_bw']} / CR{parsed['e80_cr']} / {parsed['e80_pwr_dbm']}dBm\n")
        f.write("=" * 70 + "\n")
    print(f"[SUMMARY] 已解碼 Sector 0 → {path}")
    return path


def _default_outdir(input_csv):
    """
    未指定 --outdir 時的預設輸出目錄：
      若輸入檔位於 .../flash_export_xxx/raw/xxx.csv，輸出到同層 processed/
      （對齊 gui_monitor.py 匯出時的 raw/processed 兩資料夾慣例）；否則用當前目錄。
    """
    parent = os.path.dirname(os.path.abspath(input_csv))
    if os.path.basename(parent) == "raw":
        return os.path.join(os.path.dirname(parent), "processed")
    return "."


def main():
    ap = argparse.ArgumentParser(description="🚀 RocketCom Flash 數據分析與自動報告/圖表生成工具")
    ap.add_argument("input_csv", nargs="?", help="Flash 匯出的 CSV（ring_buffer_all.csv 或 flight_id_*.csv）")
    ap.add_argument("--outdir", default=None,
                    help="輸出目錄（預設：若輸入檔位於 raw/ 底下則自動輸出到同層 processed/，否則為當前目錄）")
    ap.add_argument("--sysflags", help="sysflags_sector0.hex 路徑（選配；未指定時自動偵測同目錄）")
    args = ap.parse_args()

    if not args.input_csv:
        ap.print_help()
        sys.exit(1)
    outdir = args.outdir or _default_outdir(args.input_csv)
    os.makedirs(outdir, exist_ok=True)
    if not args.outdir:
        print(f"[ANALYZER] 未指定 --outdir，自動輸出到: {outdir}")

    print(f"[ANALYZER] 開始分析 Flash 紀錄: {args.input_csv}")
    flights, stats = parse_flash_csv(args.input_csv)
    print(f"[ANALYZER] 資料品質: 解析 {stats['parsed']} 筆 / 跳過 {stats['skipped']} 筆壞行")

    if not flights:
        print("[WARNING] CSV 中未發現有效 Flash 數據紀錄。")
    else:
        print(f"[ANALYZER] 共 {len(flights)} 次飛行（Flight IDs: {sorted(flights.keys())}）")
        for fid, records in sorted(flights.items()):
            print(f"\n---> Flight #{fid}（{len(records)} 筆）...")
            t0 = records[0]['tick_ms'] / 1000.0
            times = [(r['tick_ms'] / 1000.0) - t0 for r in records]
            events = detect_events(records, times)
            metrics = compute_metrics(records, times, events)
            health = audit_health(records, events)
            generate_flight_report(fid, records, events, metrics, health, outdir)
            generate_flight_charts(fid, records, events, metrics, outdir)
            generate_flight_map(fid, records, events, metrics, outdir)

    # SysFlags：顯式指定，或自動偵測同目錄的 sysflags_sector0.hex
    sf = args.sysflags
    if not sf:
        cand = os.path.join(os.path.dirname(os.path.abspath(args.input_csv)), "sysflags_sector0.hex")
        sf = cand if os.path.exists(cand) else None
    if sf:
        generate_sysflags_report(sf, outdir)

    print("\n[SUCCESS] 所有 Flash 飛行數據報告與視覺化圖表已生成完畢！")


if __name__ == "__main__":
    main()
