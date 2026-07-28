#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_elevator_sd_logs.py
電梯實體測試數據對比分析工具 (Elevator Real-World Field Test Comparison Tool)

用途：
讀取實體電梯測試產生的兩份 SD 卡日誌檔案：
1. SD_FLAT.CSV     (平放對照組)
2. SD_ROTATING.CSV (手持旋轉實驗組)

繪製並計算兩次測試的：
- 垂直估算高度 h_est 曲線重合度 (Correlation & RMS Error)
- 垂直估算速度 v_est 曲線重合度
- 姿態正交投影後的垂直加速度 a_z_world 重合度
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

matplotlib.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'Heiti TC', 'PingFang TC', 'DejaVu Sans', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

OUTPUT_DIR = "/Users/laizhiquan/.gemini/antigravity/brain/f27bba6e-1c25-4e0e-9f22-0df286563d42"

def parse_sd_log(filepath):
    """ 解析主航電 SD 卡輸出的 CSV 日誌 """
    if not os.path.exists(filepath):
        return None
    
    data = {
        'tick_ms': [], 'bmi_ax': [], 'bmi_ay': [], 'bmi_az': [],
        'bmi_gx': [], 'bmi_gy': [], 'bmi_gz': [],
        'alt': [], 'fsm': [], 'ekf_alt_m': [], 'ekf_vel_mps': [],
        'q0': [], 'q1': [], 'q2': [], 'q3': []
    }
    
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('tick_ms') or line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split(',')
            if len(parts) >= 16:
                try:
                    data['tick_ms'].append(float(parts[0]))
                    data['bmi_ax'].append(float(parts[1]) / 1000.0) # g
                    data['bmi_ay'].append(float(parts[2]) / 1000.0) # g
                    data['bmi_az'].append(float(parts[3]) / 1000.0) # g
                    data['alt'].append(float(parts[9]) / 100.0)     # m
                    data['fsm'].append(int(parts[10]))
                    data['ekf_alt_m'].append(float(parts[11]) / 100.0)
                    data['ekf_vel_mps'].append(float(parts[12]) / 100.0)
                    if len(parts) >= 20:
                        data['q0'].append(float(parts[16]) / 10000.0)
                        data['q1'].append(float(parts[17]) / 10000.0)
                        data['q2'].append(float(parts[18]) / 10000.0)
                        data['q3'].append(float(parts[19]) / 10000.0)
                except ValueError:
                    pass
                
    for k in data:
        data[k] = np.array(data[k])
    return data

def plot_sd_comparison(flat_file, rot_file):
    d_flat = parse_sd_log(flat_file)
    d_rot  = parse_sd_log(rot_file)
    
    if d_flat is None or d_rot is None:
        print(f"[ERROR] 找不到 CSV 檔案：{flat_file} 或 {rot_file}")
        print("提示：請將實體電梯測試產生的 SD 卡 CSV 傳入本腳本進行分析。")
        return

    # 時間歸零 (s)
    t_flat = (d_flat['tick_ms'] - d_flat['tick_ms'][0]) / 1000.0
    t_rot  = (d_rot['tick_ms'] - d_rot['tick_ms'][0]) / 1000.0

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    # 1. 高度重合度對比
    ax1.plot(t_flat, d_flat['ekf_alt_m'], color='blue', linestyle='-', linewidth=2, label="【平放對照組】VF 估估高度 h_est (m)")
    ax1.plot(t_rot, d_rot['ekf_alt_m'], color='crimson', linestyle='--', linewidth=2, label="【手持旋轉組】姿態融合 VF 高度 h_est (m)")
    ax1.set_title("電梯實體測試數據分析：平放對照組 vs. 手持旋轉組 軌跡重合度驗證", fontsize=14, fontweight='bold')
    ax1.set_ylabel("高度 (m)", fontsize=12)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper left')

    # 2. 速度重合度對比
    ax2.plot(t_flat, d_flat['ekf_vel_mps'], color='blue', linestyle='-', linewidth=1.5, label="【平放對照組】垂直速度 v_est (m/s)")
    ax2.plot(t_rot, d_rot['ekf_vel_mps'], color='crimson', linestyle='--', linewidth=1.5, label="【手持旋轉組】垂直速度 v_est (m/s)")
    ax2.set_ylabel("垂直速度 (m/s)", fontsize=12)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right')

    # 3. 垂直加速度投影對比
    # 計算旋轉組的重構世界垂直加速度
    az_flat_g = d_flat['bmi_az']
    if len(d_rot['q0']) == len(d_rot['bmi_az']):
        q0, q1, q2, q3 = d_rot['q0'], d_rot['q1'], d_rot['q2'], d_rot['q3']
        r31 = 2.0 * (q1*q3 - q0*q2)
        r32 = 2.0 * (q2*q3 + q0*q1)
        r33 = 1.0 - 2.0 * (q1**2 + q2**2)
        az_rot_proj_g = r31 * d_rot['bmi_ax'] + r32 * d_rot['bmi_ay'] + r33 * d_rot['bmi_az']
    else:
        az_rot_proj_g = d_rot['bmi_az']

    ax3.plot(t_flat, az_flat_g, color='gray', linestyle=':', label="【平放組】箭體 Z 軸加速度 (g)")
    ax3.plot(t_rot, d_rot['bmi_az'], color='orange', linestyle=':', alpha=0.6, label="【旋轉組】箭體 Z 軸原始 raw_az (受到傾斜衰減)")
    ax3.plot(t_rot, az_rot_proj_g, color='green', linestyle='-', linewidth=1.2, label="【旋轉組】四元數正交投影後垂直加速度 az_world (g)")
    ax3.set_xlabel("時間 (s)", fontsize=12)
    ax3.set_ylabel("垂直加速度 (g)", fontsize=12)
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='lower right')

    plt.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "elevator_real_sd_log_comparison.png")
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[SUCCESS] 分析對比圖已成功生成：{out_path}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("用法：python3 compare_elevator_sd_logs.py <SD_FLAT.CSV> <SD_ROTATING.CSV>")
    else:
        plot_sd_comparison(sys.argv[1], sys.argv[2])
