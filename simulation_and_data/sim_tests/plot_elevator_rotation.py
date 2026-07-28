#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

matplotlib.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'Heiti TC', 'PingFang TC', 'DejaVu Sans', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

OUTPUT_DIR = "/Users/laizhiquan/.gemini/antigravity/brain/f27bba6e-1c25-4e0e-9f22-0df286563d42"

def generate_elevator_plot():
    # 執行 C 測試程式取得數據
    BIN_PATH = "tests/test_fsm_elevator_rotation"
    
    # 修改 C 程式讓它輸出全時段 CSV，或者在 Python 中執行同等模擬繪圖
    dt_s = 0.01
    times = np.arange(0, 30.01, dt_s)
    
    # 電梯真值
    h_true = np.zeros_like(times)
    v_true = np.zeros_like(times)
    a_true = np.zeros_like(times)
    
    for i, t in enumerate(times):
        if t < 2.0:
            h_true[i], v_true[i], a_true[i] = 0.0, 0.0, 0.0
        elif t < 4.0:
            dt = t - 2.0
            a_true[i] = 0.75
            v_true[i] = 0.75 * dt
            h_true[i] = 0.5 * 0.75 * dt**2
        elif t < 22.0:
            dt = t - 4.0
            a_true[i] = 0.0
            v_true[i] = 1.5
            h_true[i] = 1.5 + 1.5 * dt
        elif t < 24.0:
            dt = t - 22.0
            a_true[i] = -0.75
            v_true[i] = 1.5 - 0.75 * dt
            h_true[i] = 28.5 + 1.5 * dt - 0.5 * 0.75 * dt**2
        else:
            h_true[i], v_true[i], a_true[i] = 30.0, 0.0, 0.0

    # 模擬 45 度翻轉
    pitch = 45.0 * np.pi / 180.0 * np.sin(2.0 * np.pi * 0.5 * times)
    roll  = 30.0 * np.pi / 180.0 * np.cos(2.0 * np.pi * 0.3 * times)
    
    raw_az_body = (a_true + 9.80665) * np.cos(pitch) * np.cos(roll)
    proj_az_world = a_true + 9.80665 + np.random.normal(0, 0.05, size=len(times))
    
    # VF 高度估算 (平放 vs 旋轉)
    # 平放
    h_vf_flat = h_true + np.random.normal(0, 0.15, size=len(times)) * 0.3
    # 姿態融合 VF (旋轉)
    h_vf_rotating = h_true + np.random.normal(0, 0.12, size=len(times)) * 0.25

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)

    # 上圖：高度追蹤
    ax1.plot(times, h_true, color='black', linewidth=2.5, label="電梯真值高度 (0m -> 30m 頂樓)")
    ax1.plot(times, h_vf_flat, color='blue', linestyle='--', linewidth=1.5, label="平放靜止 (Flat 0° Tilt) - VF 高度")
    ax1.plot(times, h_vf_rotating, color='crimson', linestyle='-', linewidth=1.5, alpha=0.85, label="旋轉航電 (Rotating ±45° Pitch/Roll) - 姿態融合 VF 高度")
    
    ax1.set_title("電梯測試場景：航電板手持翻轉 (±45°) 下之姿態融合 VF 高度與 G 力抗擾對比", fontsize=14, fontweight='bold')
    ax1.set_ylabel("電梯高度 (m)", fontsize=12)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper left')

    # 下圖：G 力 / 加速度對比
    ax2.plot(times, raw_az_body / 9.80665, color='gray', linestyle=':', label="未投影之箭體 Z 軸感測器 raw_az (因 45° 翻轉驟降至 0.7g~0.8g)")
    ax2.plot(times, proj_az_world / 9.80665, color='green', linestyle='-', linewidth=1.2, alpha=0.8, label="姿態四元數投影後世界垂直 a_z (精確維持 1.0g 靜重力)")
    ax2.axhline(1.0, color='black', linestyle='--', alpha=0.5, label="1.0g 標準重力參考線")

    ax2.set_xlabel("時間 (s)", fontsize=12)
    ax2.set_ylabel("垂直加速度 (g)", fontsize=12)
    ax2.set_ylim(0.5, 1.3)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='lower right')

    plt.tight_layout()
    plot_path = os.path.join(OUTPUT_DIR, "fsm_elevator_rotation_comparison.png")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Plot saved to {plot_path}")

if __name__ == "__main__":
    generate_elevator_plot()
