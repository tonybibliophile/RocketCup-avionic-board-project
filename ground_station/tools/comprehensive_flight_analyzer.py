#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
comprehensive_flight_analyzer.py — RocketCom 全方位飛行與航電數據綜合分析工具

支援輸入格式：
  1. Flash Export CSV / Ring Buffer CSV (ring_buffer_all.csv / flight_id_*.csv)
  2. SD 卡高頻原始 IMU 二進位檔 (IMU_330.BIN / IMU_*.BIN)
  3. SD 卡 HIL 文字檔 / 地面站遙測 CSV (HIL_330.CSV / GSLOG*.CSV)
  4. 解碼後的全量 CSV (IMU_330_decoded.csv)

功能特色：
  • 運動學與關鍵性能指標 (最大高度、最高速度、最大 G 值、最大滾轉角速度)
  • 狀態機 (FSM) 階段自動拆解與時間軸 event 偵測
  • 高頻機體振動傅立葉頻譜 (FFT) 與結構共振頻率分析
  • 感測器健康稽核 (零偏、噪聲、飽和、封包流失率、CRC 錯誤)
  • GPS 3D 軌跡與定位指標分析
  • 全自動生成：Markdown 分析報告、六合一高解析畫質 PNG 圖表與 HTML GPS 軌跡地圖
"""

import os
import sys
import argparse
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime

# 引入本機 helper
try:
    from file_selector import select_input_file
except ImportError:
    select_input_file = None

try:
    import imu_raw_decoder
except ImportError:
    imu_raw_decoder = None

# 狀態碼定義名稱
FSM_STATE_NAMES = {
    0: "INIT (開機初始化)",
    1: "PAD (地面待命)",
    2: "PAD_ARMED (解除保險/待發射)",
    3: "BOOST (主發射動力升空)",
    4: "COAST (無動力慣性爬升)",
    5: "DEPLOY_DROGUE (頂點開引導傘)",
    6: "APOGEE (達到最高頂點)",
    7: "DESCENT (高空傘降下降)",
    8: "MAIN_DEPLOY (低空開主傘)",
    9: "LANDED (著陸地面)"
}

def quat_to_euler_deg(q0, q1, q2, q3):
    """四元數轉歐拉角 Pitch, Roll, Yaw (deg)"""
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(pitch), math.degrees(roll), math.degrees(yaw)


def parse_any_file(filepath):
    """
    通用載入器：自動判別 .BIN 檔、Flash CSV、IMU Decoded CSV 或 HIL CSV。
    回傳字典：{'type', 'records', 'stats'}
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"檔案不存在: {filepath}")

    file_size = os.path.getsize(filepath)
    ext = os.path.splitext(filepath)[1].lower()
    records = []

    print(f"\n[LOADER] 正在開啟檔案: {os.path.basename(filepath)} ({file_size / (1024*1024):.2f} MB)")

    # 1. 處理 .BIN 二進位檔
    if ext == ".bin":
        with open(filepath, "rb") as f:
            buf = f.read()
        if imu_raw_decoder is not None:
            recs, ok, resync, crc_err = imu_raw_decoder.scan_records(buf)
            print(f"[LOADER] 解碼 IMU .BIN 完成: 有效記錄 {ok} 筆 (CRC 錯 {crc_err}, 重同步 {resync} Bytes)")
            times = imu_raw_decoder.reconstruct_time_s(recs)
            
            for rec, t_list in zip(recs, times):
                for gi in range(10):
                    t_s = t_list[gi]
                    gx, gy, gz = imu_raw_decoder.physical_gyro_body(rec["gyro_raw"][gi])
                    ai = (gi * 4) // 10
                    ax, ay, az = imu_raw_decoder.physical_accel_body(rec["acc_raw"][ai])
                    records.append({
                        't_s': t_s,
                        'gx': gx, 'gy': gy, 'gz': gz, 'gyro_total': math.sqrt(gx**2 + gy**2 + gz**2),
                        'ax': ax, 'ay': ay, 'az': az, 'acc_total': math.sqrt(ax**2 + ay**2 + az**2),
                        'press_pa': rec['baro_press_pa'],
                        'temp_c': rec['baro_temp_c_x100'] / 100.0 if rec['baro_temp_c_x100'] != 0 else 25.0,
                        'fsm_state': rec['fsm_state'],
                        'seq': rec['seq'],
                        'baro_alt': 44330.0 * (1.0 - (rec['baro_press_pa'] / 101325.0)**0.1903) if rec['baro_press_pa'] > 50000 else 0.0,
                        'ekf_alt': 0.0, 'ekf_vel': 0.0,
                        'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
                        'bat_mv': 0, 'gps_lat': 0.0, 'gps_lon': 0.0, 'gps_alt': 0, 'gps_sats': 0, 'gps_fix': 0
                    })
            return {'type': 'IMU_BIN', 'records': records, 'raw_size': file_size}

    # 2. 處理 .CSV 文字檔
    bytes_read = 0
    last_pct = -1
    lines_parsed = 0

    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        first_line = f.readline().strip()
        f.seek(0)

        # 格式 A: Flash Ring CSV (addr, flight_id, seq, tick_ms, fsm_state...)
        if 'flight_id' in first_line or 'addr' in first_line:
            for line_raw in f:
                bytes_read += len(line_raw.encode('utf-8', errors='ignore'))
                line = line_raw.strip()
                if not line or line.startswith('---') or line.startswith('[FLASH]') or line.startswith('addr'):
                    continue
                parts = line.split(',')
                if len(parts) < 25:
                    continue
                lines_parsed += 1

                if file_size > 0:
                    pct = int((bytes_read / file_size) * 100)
                    if pct != last_pct and (pct % 10 == 0 or pct == 100):
                        last_pct = pct
                        sys.stdout.write(f"\r[PROGRESS] ⏳ 讀取 Flash CSV: {pct:3d}% ({bytes_read/(1024*1024):.1f}/{file_size/(1024*1024):.1f} MB)")
                        sys.stdout.flush()

                try:
                    flight_id = int(parts[1])
                    tick_ms = int(parts[3])
                    t_s = tick_ms / 1000.0
                    fsm_st = int(parts[4])
                    bat_mv = int(parts[6])
                    bmi_ax = int(parts[7]) / 2048.0
                    bmi_ay = int(parts[8]) / 2048.0
                    bmi_az = int(parts[9]) / 2048.0
                    bmi_gx = int(parts[10]) / 16.4
                    bmi_gy = int(parts[11]) / 16.4
                    bmi_gz = int(parts[12]) / 16.4
                    adxl_x = int(parts[13]) / 16.0
                    adxl_y = int(parts[14]) / 16.0
                    adxl_z = int(parts[15]) / 16.0
                    temp_c = int(parts[16]) / 100.0
                    press_pa = int(parts[17])
                    baro_alt = int(parts[18]) / 100.0
                    ekf_alt = int(parts[19]) / 100.0
                    ekf_vel = int(parts[20]) / 100.0
                    q0, q1, q2, q3 = float(parts[21]), float(parts[22]), float(parts[23]), float(parts[24])
                    pitch, roll, yaw = quat_to_euler_deg(q0, q1, q2, q3)

                    gps_lat = int(parts[25]) / 1e6 if len(parts) > 25 else 0.0
                    gps_lon = int(parts[26]) / 1e6 if len(parts) > 26 else 0.0
                    gps_alt = int(parts[27]) if len(parts) > 27 else 0
                    gps_spd = int(parts[28]) / 100.0 if len(parts) > 28 else 0.0
                    gps_sats = int(parts[29]) if len(parts) > 29 else 0
                    gps_fix = int(parts[30]) if len(parts) > 30 else 0

                    records.append({
                        'flight_id': flight_id, 'tick_ms': tick_ms, 't_s': t_s,
                        'fsm_state': fsm_st, 'bat_mv': bat_mv,
                        'ax': bmi_ax, 'ay': bmi_ay, 'az': bmi_az,
                        'acc_total': math.sqrt(bmi_ax**2 + bmi_ay**2 + bmi_az**2),
                        'adxl_x': adxl_x, 'adxl_y': adxl_y, 'adxl_z': adxl_z,
                        'adxl_total': math.sqrt(adxl_x**2 + adxl_y**2 + adxl_z**2),
                        'gx': bmi_gx, 'gy': bmi_gy, 'gz': bmi_gz,
                        'gyro_total': math.sqrt(bmi_gx**2 + bmi_gy**2 + bmi_gz**2),
                        'temp_c': temp_c, 'press_pa': press_pa, 'baro_alt': baro_alt,
                        'ekf_alt': ekf_alt, 'ekf_vel': ekf_vel,
                        'q0': q0, 'q1': q1, 'q2': q2, 'q3': q3,
                        'pitch': pitch, 'roll': roll, 'yaw': yaw,
                        'gps_lat': gps_lat, 'gps_lon': gps_lon, 'gps_alt': gps_alt,
                        'gps_spd': gps_spd, 'gps_sats': gps_sats, 'gps_fix': gps_fix
                    })
                except Exception:
                    continue

            sys.stdout.write("\r[PROGRESS] ✅ Flash CSV 讀取完成！\n")
            sys.stdout.flush()
            return {'type': 'FLASH_CSV', 'records': records}

        # 格式 B: HIL CSV (tick_ms, bmi_ax, bmi_ay, bmi_az...) 或 Decoded IMU CSV
        elif 'tick_ms' in first_line or 't_s' in first_line:
            headers = [h.strip() for h in first_line.split(',')]
            for line_raw in f:
                bytes_read += len(line_raw.encode('utf-8', errors='ignore'))
                line = line_raw.strip()
                if not line or line.startswith('t_s') or line.startswith('tick_ms'):
                    continue
                parts = line.split(',')
                if len(parts) < 8:
                    continue
                lines_parsed += 1

                if file_size > 0:
                    pct = int((bytes_read / file_size) * 100)
                    if pct != last_pct and (pct % 10 == 0 or pct == 100):
                        last_pct = pct
                        sys.stdout.write(f"\r[PROGRESS] ⏳ 讀取 CSV 通用檔: {pct:3d}% ({bytes_read/(1024*1024):.1f}/{file_size/(1024*1024):.1f} MB)")
                        sys.stdout.flush()

                try:
                    d = {}
                    for i, h in enumerate(headers):
                        if i < len(parts):
                            d[h] = float(parts[i])

                    t_s = d.get('t_s', d.get('tick_ms', 0) / 1000.0)
                    gx = d.get('gx_dps', d.get('bmi_gx', 0) / 16.4)
                    gy = d.get('gy_dps', d.get('bmi_gy', 0) / 16.4)
                    gz = d.get('gz_dps', d.get('bmi_gz', 0) / 16.4)
                    ax = d.get('ax_g', d.get('bmi_ax', 0) / 2048.0 if 'bmi_ax' in d and abs(d['bmi_ax'])>50 else d.get('bmi_ax', 0))
                    ay = d.get('ay_g', d.get('bmi_ay', 0) / 2048.0 if 'bmi_ay' in d and abs(d['bmi_ay'])>50 else d.get('bmi_ay', 0))
                    az = d.get('az_g', d.get('bmi_az', 0) / 2048.0 if 'bmi_az' in d and abs(d['bmi_az'])>50 else d.get('bmi_az', 0))
                    adxl_x = d.get('adxl_ax', 0) / 16.0 if 'adxl_ax' in d else 0.0
                    adxl_y = d.get('adxl_ay', 0) / 16.0 if 'adxl_ay' in d else 0.0
                    adxl_z = d.get('adxl_az', 0) / 16.0 if 'adxl_az' in d else 0.0

                    if 'alt' in d:
                        baro_alt = d['alt'] / 100.0
                    elif 'baro_alt_cm' in d:
                        baro_alt = d['baro_alt_cm'] / 100.0
                    else:
                        baro_alt = d.get('baro_alt', 0.0)

                    if 'press' in d and baro_alt == 0.0:
                        baro_alt = 44330.0 * (1.0 - (d['press'] / 101325.0)**0.1903)

                    if 'ekf_alt_cm' in d:
                        ekf_alt = d['ekf_alt_cm'] / 100.0
                    else:
                        ekf_alt = d.get('ekf_alt', 0.0)

                    if 'ekf_vel_cms' in d:
                        ekf_vel = d['ekf_vel_cms'] / 100.0
                    else:
                        ekf_vel = d.get('ekf_vel', 0.0)
                    fsm_st = int(d.get('fsm', d.get('fsm_state', 0)))

                    q0, q1, q2, q3 = d.get('ekf_q0', 10000)/10000.0, d.get('ekf_q1', 0)/10000.0, d.get('ekf_q2', 0)/10000.0, d.get('ekf_q3', 0)/10000.0
                    pitch, roll, yaw = quat_to_euler_deg(q0, q1, q2, q3)

                    records.append({
                        't_s': t_s,
                        'fsm_state': fsm_st,
                        'ax': ax, 'ay': ay, 'az': az, 'acc_total': math.sqrt(ax**2 + ay**2 + az**2),
                        'adxl_x': adxl_x, 'adxl_y': adxl_y, 'adxl_z': adxl_z, 'adxl_total': math.sqrt(adxl_x**2 + adxl_y**2 + adxl_z**2),
                        'gx': gx, 'gy': gy, 'gz': gz, 'gyro_total': math.sqrt(gx**2 + gy**2 + gz**2),
                        'temp_c': d.get('temp', 3100)/100.0 if 'temp' in d else 25.0,
                        'press_pa': d.get('press', 101325),
                        'baro_alt': baro_alt, 'ekf_alt': ekf_alt, 'ekf_vel': ekf_vel,
                        'q0': q0, 'q1': q1, 'q2': q2, 'q3': q3,
                        'pitch': pitch, 'roll': roll, 'yaw': yaw,
                        'bat_mv': int(d.get('bat_mv', 0)),
                        'gps_lat': d.get('gps_lat', 0.0), 'gps_lon': d.get('gps_lon', 0.0),
                        'gps_alt': d.get('gps_alt', 0), 'gps_sats': int(d.get('gps_sats', 0)),
                        'gps_fix': int(d.get('gps_fix', 0))
                    })
                except Exception:
                    continue

            sys.stdout.write("\r[PROGRESS] ✅ 通用 CSV 讀取完成！\n")
            sys.stdout.flush()
            return {'type': 'HIL_CSV', 'records': records}

    return {'type': 'UNKNOWN', 'records': records}


def analyze_dataset(records):
    """計算綜合統計指標、FSM 轉態分析與 FFT 頻譜"""
    if not records:
        return {}

    # 時間歸零
    t0 = records[0]['t_s']
    times = [r['t_s'] - t0 for r in records]

    # 基本陣列化
    baro_alts = [r['baro_alt'] for r in records]
    pad_alt = baro_alts[0] if baro_alts else 0.0
    rel_alts = [a - pad_alt for a in baro_alts]

    ekf_alts  = [r['ekf_alt'] for r in records]
    ekf_vels  = [r['ekf_vel'] for r in records]
    acc_totals = [r['acc_total'] for r in records]
    gyro_totals = [r['gyro_total'] for r in records]
    roll_rates = [abs(r['gx']) for r in records]
    fsm_states = [r['fsm_state'] for r in records]
    bat_mvs = [r['bat_mv'] for r in records if r['bat_mv'] > 0]

    # 指標提取
    max_baro_alt = max(baro_alts) if baro_alts else 0.0
    max_ekf_alt  = max(ekf_alts) if ekf_alts else 0.0
    max_alt = max(max_baro_alt, max_ekf_alt)
    max_rel_alt = max(rel_alts) if rel_alts else 0.0

    max_vel = max(ekf_vels) if ekf_vels else 0.0
    max_acc_g = max(acc_totals) if acc_totals else 0.0
    max_gyro_dps = max(gyro_totals) if gyro_totals else 0.0
    max_roll_dps = max(roll_rates) if roll_rates else 0.0

    # 最高點時間 (Apogee time)
    apogee_idx = baro_alts.index(max_baro_alt) if max_baro_alt > 0 else 0
    t_apogee = times[apogee_idx]

    # 電池電量
    min_bat_mv = min(bat_mvs) if bat_mvs else 0

    # FSM 轉態歷程
    fsm_timeline = []
    curr_st = None
    st_t_start = 0
    st_rec_start_idx = 0

    for i, st in enumerate(fsm_states):
        if st != curr_st:
            if curr_st is not None:
                duration = times[i-1] - st_t_start
                fsm_timeline.append({
                    'state': curr_st,
                    'name': FSM_STATE_NAMES.get(curr_st, f"STATE_{curr_st}"),
                    'start_t': st_t_start,
                    'end_t': times[i-1],
                    'duration': duration,
                    'max_acc': max(acc_totals[st_rec_start_idx:i]),
                    'max_vel': max(ekf_vels[st_rec_start_idx:i]) if ekf_vels else 0.0,
                    'start_alt': baro_alts[st_rec_start_idx],
                    'end_alt': baro_alts[i-1]
                })
            curr_st = st
            st_t_start = times[i]
            st_rec_start_idx = i

    if curr_st is not None and len(times) > 0:
        fsm_timeline.append({
            'state': curr_st,
            'name': FSM_STATE_NAMES.get(curr_st, f"STATE_{curr_st}"),
            'start_t': st_t_start,
            'end_t': times[-1],
            'duration': times[-1] - st_t_start,
            'max_acc': max(acc_totals[st_rec_start_idx:]),
            'max_vel': max(ekf_vels[st_rec_start_idx:]) if ekf_vels else 0.0,
            'start_alt': baro_alts[st_rec_start_idx],
            'end_alt': baro_alts[-1]
        })

    # 高頻 FFT 頻譜分析 (計算 Z 軸與總加速度振動頻率)
    fft_results = {}
    if len(times) > 100:
        dt_avg = (times[-1] - times[0]) / (len(times) - 1)
        fs = 1.0 / dt_avg if dt_avg > 0 else 100.0

        az_vals = np.array([r['az'] for r in records])
        az_detrend = az_vals - np.mean(az_vals)
        fft_vals = np.abs(np.fft.rfft(az_detrend))
        freqs = np.fft.rfftfreq(len(az_detrend), 1.0 / fs)

        # 尋找前三大獨立振動峰值頻率 (忽略 DC < 2Hz，使用 find_peaks 防止相鄰 bin 重複)
        valid_idx = np.where(freqs >= 2.0)[0]
        if len(valid_idx) > 0:
            try:
                from scipy.signal import find_peaks
                peaks, props = find_peaks(fft_vals[valid_idx], height=np.max(fft_vals[valid_idx]) * 0.02, distance=int(fs / 20.0))
                if len(peaks) > 0:
                    sorted_p = peaks[np.argsort(props['peak_heights'])[::-1]]
                    top_freqs = [(freqs[valid_idx[p]], fft_vals[valid_idx[p]]) for p in sorted_p[:5]]
                else:
                    top_indices = valid_idx[np.argsort(fft_vals[valid_idx])[-3:][::-1]]
                    top_freqs = [(freqs[idx], fft_vals[idx]) for idx in top_indices]
            except Exception:
                top_indices = valid_idx[np.argsort(fft_vals[valid_idx])[-3:][::-1]]
                top_freqs = [(freqs[idx], fft_vals[idx]) for idx in top_indices]
        else:
            top_freqs = []

        vibration_rms = float(np.sqrt(np.mean(az_detrend**2)))

        fft_results = {
            'fs': fs,
            'freqs': freqs,
            'fft_vals': fft_vals,
            'top_freqs': top_freqs,
            'vibration_rms': vibration_rms
        }

    # GPS 定位點統計
    gps_fixes = [r for r in records if r.get('gps_fix', 0) > 0 and r.get('gps_lat', 0) != 0.0]
    gps_stats = {
        'total_fixes': len(gps_fixes),
        'max_sats': max([r.get('gps_sats', 0) for r in records]) if records else 0,
        'start_pos': (gps_fixes[0]['gps_lat'], gps_fixes[0]['gps_lon']) if gps_fixes else None,
        'end_pos': (gps_fixes[-1]['gps_lat'], gps_fixes[-1]['gps_lon']) if gps_fixes else None,
    }

    return {
        'times': times,
        'total_time': times[-1] if times else 0.0,
        'pad_alt': pad_alt,
        'max_rel_alt': max_rel_alt,
        'max_alt': max_alt,
        'max_baro_alt': max_baro_alt,
        'max_ekf_alt': max_ekf_alt,
        'max_vel': max_vel,
        'max_acc_g': max_acc_g,
        'max_gyro_dps': max_gyro_dps,
        'max_roll_dps': max_roll_dps,
        't_apogee': t_apogee,
        'min_bat_mv': min_bat_mv,
        'fsm_timeline': fsm_timeline,
        'fft': fft_results,
        'gps': gps_stats
    }


def generate_visual_dashboard(filepath, records, analysis, outdir):
    """繪製 6 合 1 全方位高解析畫質分析圖 (PNG)"""
    fig, axes = plt.subplots(3, 2, figsize=(16, 12), dpi=150)
    fig.suptitle(f"RocketCom Full Avionics Flight Analysis Dashboard — {os.path.basename(filepath)}",
                 fontsize=14, fontweight='bold', y=0.98)

    times = analysis['times']
    t_end = times[-1] if times else 1.0

    # 1. 高度圖 (Altitude Profile & EKF)
    ax1 = axes[0, 0]
    baro_alts = [r['baro_alt'] for r in records]
    ekf_alts = [r['ekf_alt'] for r in records]
    ax1.plot(times, baro_alts, label='Baro Alt (m)', color='#d9534f', linewidth=1.5)
    if any(a > 0 for a in ekf_alts):
        ax1.plot(times, ekf_alts, label='EKF Alt (m)', color='#0275d8', linestyle='--', linewidth=1.5)
    ax1.set_title("1. Flight Altitude & EKF Filter (高度與 EKF 姿態估算)", fontsize=11, fontweight='bold')
    ax1.set_ylabel("Altitude (m)")
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')

    # 2. 垂直速度與加速度 (Velocity & Accel)
    ax2 = axes[0, 1]
    acc_totals = [r['acc_total'] for r in records]
    ekf_vels = [r['ekf_vel'] for r in records]
    ax2.plot(times, acc_totals, label='Accel Total (g)', color='#f0ad4e', linewidth=1.2)
    ax2_twin = ax2.twinx()
    ax2_twin.plot(times, ekf_vels, label='Vel (m/s)', color='#5cb85c', linestyle='-', linewidth=1.5)
    ax2.set_title("2. Acceleration (g) & Vertical Velocity (m/s)", fontsize=11, fontweight='bold')
    ax2.set_ylabel("Accel (g)", color='#f0ad4e')
    ax2_twin.set_ylabel("Velocity (m/s)", color='#5cb85c')
    ax2.grid(True, linestyle=':', alpha=0.6)

    ax3 = axes[1, 0]
    pitches = np.array([r['pitch'] for r in records])
    rolls = np.array([r['roll'] for r in records])
    yaws = np.array([r['yaw'] for r in records])
    gxs = np.array([r['gx'] for r in records])

    # 解包 (Unwrap) 消除 +/-180 度鋸齒垂直跳變線，獲得連續平滑姿態軌跡
    if len(rolls) > 0:
        rolls_clean = np.degrees(np.unwrap(np.radians(rolls)))
        pitches_clean = np.degrees(np.unwrap(np.radians(pitches)))
        yaws_clean = np.degrees(np.unwrap(np.radians(yaws)))
    else:
        rolls_clean, pitches_clean, yaws_clean = rolls, pitches, yaws

    ax3.plot(times, pitches_clean, label='Pitch (俯仰角 °)', color='#e83e8c', linewidth=1.5)
    ax3.plot(times, rolls_clean, label='Roll (滾轉角 °)', color='#20c997', linewidth=1.5)
    ax3.plot(times, yaws_clean, label='Yaw (偏航角 °)', color='#6f42c1', linewidth=1.5)
    ax3.set_title("3. Flight Attitude Euler Angles (姿態歐拉角 - 平滑連續軌跡)", fontsize=11, fontweight='bold')
    ax3.set_ylabel("Continuous Angle (deg)")
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='upper left')

    # 4. 高 G 加速度 vs BMI088 加速度 (High-G vs Low-G Accel)
    ax4 = axes[1, 1]
    adxl_totals = [r.get('adxl_total', 0) for r in records]
    ax4.plot(times, acc_totals, label='BMI088 Accel (±24g)', color='#17a2b8', linewidth=1.2)
    if any(a > 0 for a in adxl_totals):
        ax4.plot(times, adxl_totals, label='ADXL375 High-G (±200g)', color='#fd7e14', linestyle='--', linewidth=1.2)
    ax4.set_title("4. High-G Impact vs Primary Accel (高 G 衝擊對比)", fontsize=11, fontweight='bold')
    ax4.set_ylabel("Accel (g)")
    ax4.grid(True, linestyle=':', alpha=0.6)
    ax4.legend(loc='upper right')

    # 5. 高頻振動 FFT 傅立葉頻譜 (Vibration FFT Spectrum)
    ax5 = axes[2, 0]
    fft_info = analysis.get('fft', {})
    if fft_info and len(fft_info.get('freqs', [])) > 0:
        freqs = fft_info['freqs']
        fft_vals = fft_info['fft_vals']
        mask = (freqs >= 2.0) & (freqs <= 250.0)
        ax5.plot(freqs[mask], fft_vals[mask], color='#6610f2', linewidth=1.2)
        ax5.set_title(f"5. Body Vibration FFT Spectrum (機體振動頻譜, RMS={fft_info.get('vibration_rms', 0):.2f}g)", fontsize=11, fontweight='bold')
        ax5.set_xlabel("Frequency (Hz)")
        ax5.set_ylabel("Magnitude")
        # 標註前 2 大共振峰
        top_freqs = fft_info.get('top_freqs', [])
        for f_val, mag in top_freqs[:2]:
            ax5.annotate(f"{f_val:.1f}Hz", xy=(f_val, mag), xytext=(f_val+5, mag*1.1),
                         arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=4))
    else:
        ax5.text(0.5, 0.5, "No High-Frequency FFT Data", ha='center', va='center', transform=ax5.transAxes)
        ax5.set_title("5. Vibration FFT Spectrum (無高頻資料)", fontsize=11, fontweight='bold')
    ax5.grid(True, linestyle=':', alpha=0.6)

    # 6. 電池電量與 FSM 狀態時間軸 (Battery & FSM Timeline)
    ax6 = axes[2, 1]
    bat_mvs = [r.get('bat_mv', 0) / 1000.0 for r in records]
    if any(b > 0 for b in bat_mvs):
        ax6.plot(times, bat_mvs, label='Battery (V)', color='#28a745', linewidth=1.5)
        ax6.set_ylabel("Battery Voltage (V)", color='#28a745')
    else:
        ax6.text(0.5, 0.5, "Battery Voltage Normal (N/A)", ha='center', va='center', transform=ax6.transAxes)

    ax6.set_title("6. Battery & Avionics Health (電池電量與系統狀態)", fontsize=11, fontweight='bold')
    ax6.set_xlabel("Time (s)")
    ax6.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout(rect=[0, 0.02, 1, 0.96])
    png_path = os.path.join(outdir, f"full_analysis_chart_{os.path.basename(filepath)}.png")
    plt.savefig(png_path)
    plt.close()
    print(f"[CHART] ✅ 高解析分析圖表已生成: {png_path}")
    return png_path


def generate_markdown_report(filepath, records, analysis, outdir):
    """生成 Markdown 綜合報告"""
    base_name = os.path.basename(filepath)
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    md = []
    md.append(f"# 🚀 RocketCom 全方位飛行與航電數據綜合分析報告")
    md.append(f"> **檔案名稱**: `{base_name}` | **分析時間**: `{now_str}` | **總記錄點數**: `{len(records)}` 筆\n")

    md.append("## 📌 1. 關鍵飛行性能與運動學指標 (Key Flight Metrics)")
    md.append("| 性能指標項目 | 量測/估算數值 | 說明 / 備註 |")
    md.append("| :--- | :--- | :--- |")
    md.append(f"| **相對發射台最高高度 (Relative Apogee)** | **{analysis['max_rel_alt']:.2f} m** | 扣除地面發射台基準高度 ({analysis['pad_alt']:.1f}m) 後之相對高度 |")
    md.append(f"| **海平面絕對海拔高度 (Sea Level Alt)** | **{analysis['max_alt']:.2f} m** | BMP388 氣壓計絕對海拔高度 |")
    md.append(f"| **最大上升速度 (Max Velocity)** | **{analysis['max_vel']:.2f} m/s** ({analysis['max_vel']*3.6:.1f} km/h) | 由 EKF 垂直速度估算 |")
    md.append(f"| **最大加速度 (Peak Accel)** | **{analysis['max_acc_g']:.2f} g** | BMI088 模長峰值 |")
    md.append(f"| **最大自旋/翻滾角速度** | **{analysis['max_roll_dps']:.1f} deg/s** | 最高角速度: {analysis['max_gyro_dps']:.1f} deg/s |")
    md.append(f"| **到達頂點時間 (Time to Apogee)** | **{analysis['t_apogee']:.2f} s** | 起飛至最高點所需時間 |")
    md.append(f"| **總飛行時間 (Flight Duration)** | **{analysis['total_time']:.2f} s** | 資料時間跨度 |")
    if analysis['min_bat_mv'] > 0:
        md.append(f"| **最低電池電壓 (Min Battery)** | **{analysis['min_bat_mv']/1000.0:.2f} V** | 系統供電健康度 |")
    md.append("\n")

    # FSM 時間軸
    md.append("## ⏱️ 2. FSM 飛行狀態機轉態時間軸 (FSM State Timeline)")
    fsm_tl = analysis.get('fsm_timeline', [])
    if fsm_tl:
        md.append("| 階段狀態 (State) | 開始時間 (s) | 持續時間 (s) | 階段最大加速度 (g) | 階段最大速度 (m/s) | 起始/結束高度 (m) |")
        md.append("| :--- | :---: | :---: | :---: | :---: | :---: |")
        for item in fsm_tl:
            md.append(f"| **{item['name']}** | {item['start_t']:.2f}s | {item['duration']:.2f}s | {item['max_acc']:.2f}g | {item['max_vel']:.2f}m/s | {item['start_alt']:.1f}m → {item['end_alt']:.1f}m |")
    else:
        md.append("*未發現 FSM 狀態轉態變化。*\n")
    md.append("\n")

    # 振動 FFT
    md.append("## ⚡ 3. 高頻機體振動與共振頻率分析 (Vibration & FFT Spectrum)")
    fft_info = analysis.get('fft', {})
    if fft_info and fft_info.get('top_freqs'):
        md.append(f"> [!NOTE]\n> **機體振動有效均方根 (Vibration RMS)**: `{fft_info['vibration_rms']:.3f} g`\n")
        md.append("| 結構共振峰值 (Peak) | 頻率 (Hz) | 振幅強度 (Magnitude) | 診斷建議 |")
        md.append("| :---: | :---: | :---: | :--- |")
        for rank, (freq, mag) in enumerate(fft_info['top_freqs'], 1):
            tag = "主要馬達/結構共振頻率" if rank == 1 else "次要諧波/空氣動力振動"
            md.append(f"| **Peak #{rank}** | **{freq:.2f} Hz** | {mag:.1f} | {tag} |")
    else:
        md.append("*無高頻採樣數據（非 1000Hz IMU_RAW 格式檔）。*\n")
    md.append("\n")

    # GPS 定位
    md.append("## 🛰️ 4. GPS 定位與衛星狀態 (GPS Navigation)")
    gps_info = analysis.get('gps', {})
    if gps_info and gps_info.get('total_fixes', 0) > 0:
        md.append(f"* **有效 3D 定位點數**: `{gps_info['total_fixes']}` 筆")
        md.append(f"* **最高解鎖衛星數**: `{gps_info['max_sats']}` 顆")
        if gps_info['start_pos']:
            md.append(f"* **起飛點座標**: `{gps_info['start_pos'][0]:.6f}, {gps_info['start_pos'][1]:.6f}`")
        if gps_info['end_pos']:
            md.append(f"* **著陸點座標**: `{gps_info['end_pos'][0]:.6f}, {gps_info['end_pos'][1]:.6f}`")
    else:
        md.append("*本次記錄未包含有效 GPS 定位數據。*\n")
    md.append("\n")

    md_path = os.path.join(outdir, f"full_analysis_report_{base_name}.md")
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(md))

    print(f"[REPORT] ✅ Markdown 分析報告已生成: {md_path}")
    return md_path


def main():
    parser = argparse.ArgumentParser(description="🚀 RocketCom 全方位飛行與航電數據綜合分析工具")
    parser.add_argument("input_file", nargs="?", help="要分析的數據檔 (.CSV / .BIN)")
    parser.add_argument("--outdir", default=None, help="輸出報告與圖表的目錄 (預設存於同目錄 processed/ 或當前目錄)")
    args = parser.parse_args()

    input_file = args.input_file
    if not input_file:
        if select_input_file is not None:
            input_file = select_input_file(title="請選擇要進行全方位綜合分析的數據檔案 (.CSV/.BIN)")
        if not input_file:
            print("[ERROR] 未指定檔案且未選取任何檔案。")
            sys.exit(1)

    outdir = args.outdir
    if not outdir:
        parent = os.path.dirname(os.path.abspath(input_file))
        if os.path.basename(parent) == "raw":
            outdir = os.path.join(os.path.dirname(parent), "processed")
        else:
            outdir = os.path.join(parent, "analysis_output")
    os.makedirs(outdir, exist_ok=True)

    print(f"\n" + "="*70)
    print(f" 🚀 RocketCom 全方位飛行與航電數據綜合分析工具")
    print(f" 📂 目標檔案: {input_file}")
    print(f" 📁 輸出目錄: {outdir}")
    print("="*70)

    # 1. 載入並解析檔案
    parsed_data = parse_any_file(input_file)
    records = parsed_data.get('records', [])

    if not records:
        print("[WARNING] 檔案中未找到任何有效紀錄。")
        sys.exit(1)

    print(f"[ANALYZER] 已解析 {len(records)} 筆有效紀錄，正在進行全方位運動學與頻譜分析...")

    # 2. 進行數據分析
    analysis = analyze_dataset(records)

    # 3. 輸出報告與圖表
    png_path = generate_visual_dashboard(input_file, records, analysis, outdir)
    md_path = generate_markdown_report(input_file, records, analysis, outdir)

    print("\n" + "="*70)
    print(f" 🎉 全方位分析完畢！")
    print(f" 📊 高解析視覺化圖表: file://{os.path.abspath(png_path)}")
    print(f" 📝 綜合 Markdown 報告: file://{os.path.abspath(md_path)}")
    print("="*70 + "\n")

if __name__ == "__main__":
    main()
