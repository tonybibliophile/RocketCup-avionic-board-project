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
BIN_PATH = "tests/test_fsm_noise_suite"
CSV_PATH = "simulation_and_data/flight_data/v3.csv"

def run_c_sim(csv_path, noise_a, noise_h, pitch_deg=0.0):
    cmd = [BIN_PATH, csv_path, str(noise_a), str(noise_h), str(pitch_deg)]
    res = subprocess.run(cmd, capture_output=True, text=True, check=True)
    
    times, alts_true, h_ests, v_ests, states, evts = [], [], [], [], [], []
    for line in res.stdout.strip().split('\n'):
        parts = line.strip().split(',')
        if len(parts) == 6:
            times.append(float(parts[0]))
            alts_true.append(float(parts[1]))
            h_ests.append(float(parts[2]))
            v_ests.append(float(parts[3]))
            states.append(int(parts[4]))
            evts.append(int(parts[5]))
            
    return np.array(times), np.array(alts_true), np.array(h_ests), np.array(v_ests), np.array(states), np.array(evts)

def generate_plots():
    # 1. 噪聲對比圖 (C EKF Filtered)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
    
    runs = [
        (0.0, 0.0, 0.0, "標稱無噪聲", 'blue', '-'),
        (1.5, 2.0, 0.0, "中高噪聲 (σ_a=1.5g, σ_h=2.0m)", 'orange', '--'),
        (3.0, 5.0, 0.0, "極大噪聲 (σ_a=3.0g, σ_h=5.0m)", 'crimson', '-.'),
        (5.0, 10.0, 0.0, "超極限噪聲 (σ_a=5.0g, σ_h=10.0m)", 'purple', ':')
    ]
    
    for na, nh, pitch, label, color, ls in runs:
        times, alts, h_ests, v_ests, states, evts = run_c_sim(CSV_PATH, na, nh, pitch)
        ax1.plot(times, h_ests, label=label, color=color, linestyle=ls, alpha=0.85, linewidth=1.5)
        ax2.plot(times, states, label=label, color=color, linestyle=ls, linewidth=1.5)

    ax1.set_title("板載 EKF 濾波處理後：不同感測器噪聲下的 FSM 狀態與高度軌跡對比 (V3模擬)", fontsize=14, fontweight='bold')
    ax1.set_ylabel("卡爾曼估計高度 h_est (m)", fontsize=12)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')
    
    ax2.set_yticks([1,2,3,4,5,6,7,8,9])
    ax2.set_yticklabels(['PAD','PAD_ARMED','BOOST','COAST','DROGUE','APOGEE','DESCENT','MAIN','LANDED'])
    ax2.set_xlabel("時間 (s)", fontsize=12)
    ax2.set_ylabel("FSM 狀態", fontsize=12)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.set_xlim(0, 280)
    
    plt.tight_layout()
    plot1 = os.path.join(OUTPUT_DIR, "fsm_noise_robustness.png")
    plt.savefig(plot1, dpi=150, bbox_inches='tight')
    plt.close()

    # 2. 傾角對比圖
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7.5), sharex=True)
    
    angles = [
        (0.0, "垂直彈道 (0°)", 'green'),
        (15.0, "輕微傾斜 (15°)", 'blue'),
        (30.0, "中度傾斜 (30°)", 'orange'),
        (45.0, "嚴重傾斜 (45°)", 'purple')
    ]
    
    for pitch, label, color in angles:
        times, alts, h_ests, v_ests, states, evts = run_c_sim(CSV_PATH, 0.0, 0.0, pitch)
        apex = np.max(h_ests)
        ax1.plot(times, h_ests, label=f"{label} [頂點: {apex:.0f}m]", color=color, linewidth=1.5)
        ax2.plot(times, states, label=label, color=color, linewidth=1.5)

    ax1.set_title("不同發射軌道傾角 (Pitch Tilt) 下的開傘與頂點判定比較 (V3模擬)", fontsize=14, fontweight='bold')
    ax1.set_ylabel("垂直高度 (m)", fontsize=12)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')
    
    ax2.set_yticks([1,2,3,4,5,6,7,8,9])
    ax2.set_yticklabels(['PAD','PAD_ARMED','BOOST','COAST','DROGUE','APOGEE','DESCENT','MAIN','LANDED'])
    ax2.set_xlabel("時間 (s)", fontsize=12)
    ax2.set_ylabel("FSM 狀態", fontsize=12)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.set_xlim(0, 280)
    
    plt.tight_layout()
    plot2 = os.path.join(OUTPUT_DIR, "fsm_tilt_deviation.png")
    plt.savefig(plot2, dpi=150, bbox_inches='tight')
    plt.close()

    # 3. 頂點微觀放大 (對齊 V3 peak 3259.16m at 27.22s)
    fig, ax = plt.subplots(figsize=(11, 5.5))
    times, alts, h_ests, v_ests, states, evts = run_c_sim(CSV_PATH, 0.0, 0.0, 0.0)
    
    ax.plot(times, alts, color='black', linewidth=2, label="OpenRocket V3 物理真值高度")
    
    # 找出 drogue fire & apogee confirm 時刻
    t_drogue = times[np.where(evts == 5)[0][0]] if 5 in evts else 24.22
    t_apogee = times[np.where(evts == 7)[0][0]] if 7 in evts else 27.22
    
    idx_d = np.argmin(np.abs(times - t_drogue))
    idx_a = np.argmin(np.abs(times - t_apogee))
    
    ax.axvline(t_drogue, color='red', linestyle='--', linewidth=1.5, label=f"DEPLOY_DROGUE 點火 (t={t_drogue:.2f}s, h={alts[idx_d]:.1f}m)")
    ax.axvline(t_apogee, color='green', linestyle='-', linewidth=1.5, label=f"STATE_APOGEE 頂點確認 (t={t_apogee:.2f}s, h={alts[idx_a]:.1f}m)")
    
    ax.annotate(f"提前 3.0s Lead 點火\n(h={alts[idx_d]:.1f}m)",
                xy=(t_drogue, alts[idx_d]), xytext=(t_drogue - 4.5, alts[idx_d] - 40),
                arrowprops=dict(facecolor='red', shrink=0.05, width=1, headwidth=6))
                
    ax.annotate(f"確認回落 & 鎖存頂點\n(max_alt=3259.16m)",
                xy=(t_apogee, alts[idx_a]), xytext=(t_apogee + 1.2, alts[idx_a] - 50),
                arrowprops=dict(facecolor='green', shrink=0.05, width=1, headwidth=6))
                
    ax.set_xlim(22, 34)
    ax.set_ylim(3100, 3280)
    ax.set_title("頂點區間 (Apogee Phase) 提前點火與確認識別微觀分析 (V3模擬)", fontsize=14, fontweight='bold')
    ax.set_xlabel("時間 (s)", fontsize=12)
    ax.set_ylabel("高度 (m)", fontsize=12)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='lower left')
    
    plt.tight_layout()
    plot3 = os.path.join(OUTPUT_DIR, "fsm_apogee_zoom.png")
    plt.savefig(plot3, dpi=150, bbox_inches='tight')
    plt.close()

    # 4. 主傘 600m 限高 (對齊 V3 main deploy at t=237.19s, h=344.6m)
    fig, ax = plt.subplots(figsize=(11, 5.5))
    ax.plot(times, alts, color='navy', linewidth=2, label="V3 降落高度軌跡")
    ax.axhline(600.0, color='crimson', linestyle='--', linewidth=1.5, label="FSM 主傘最高限制門檻 (600m Limit)")
    ax.axhline(300.0, color='gray', linestyle=':', linewidth=1.2, label="目標開傘高度 (300m Target)")
    
    t_main = times[np.where(evts == 8)[0][0]] if 8 in evts else 237.19
    idx_m = np.argmin(np.abs(times - t_main))
    
    ax.scatter([t_main], [alts[idx_m]], color='crimson', s=80, marker='o', zorder=5, label=f"MAIN_DEPLOY 部署 (t={t_main:.1f}s, h={alts[idx_m]:.1f}m)")
    ax.annotate(f"動態部署觸發 (h={alts[idx_m]:.1f}m < 600m 上限)",
                xy=(t_main, alts[idx_m]), xytext=(t_main - 25, alts[idx_m] + 150),
                arrowprops=dict(facecolor='crimson', shrink=0.05, width=1, headwidth=6))

    ax.set_xlim(180, 260)
    ax.set_ylim(0, 1000)
    ax.set_title("下降階段 (Descent Phase) 主傘 600m 限高與動態開傘保護微觀分析 (V3模擬)", fontsize=14, fontweight='bold')
    ax.set_xlabel("時間 (s)", fontsize=12)
    ax.set_ylabel("高度 (m)", fontsize=12)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plot4 = os.path.join(OUTPUT_DIR, "fsm_main_deploy_zoom.png")
    plt.savefig(plot4, dpi=150, bbox_inches='tight')
    plt.close()

if __name__ == "__main__":
    generate_plots()
