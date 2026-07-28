#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RocketCom Comprehensive Real-time Avionics Dashboard (Matplotlib HIL Visualizer)
===========================================================================
Displays:
  1. Sensor Raw Data: 3-axis gyro (dps), BMI088 Low-G, ADXL375 High-G Z-accel (g)
  2. EKF & VF State Estimation: VF Altitude vs EKF Altitude vs Baro Altitude (m), VF Vz vs EKF Vz (m/s)
  3. Dual-Board FSM States: Primary & Backup Flight State Machine transition timestamps
  4. System Health & Banner: Battery voltage, GPS status, Sensor/EKF health bits
  5. Interactive GUI Controls: ARM / DISARM Uplink Command Buttons
"""

import sys
import os
import time
import re
import queue
import threading
import math
from collections import deque

# Check dependencies
try:
    import serial
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.widgets import Button
    import numpy as np
except ImportError:
    print("[*] 偵測到缺失套件，正在自動安裝 pyserial 與 matplotlib...")
    import subprocess
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial", "matplotlib", "numpy"])
        import serial
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
        from matplotlib.widgets import Button
        import numpy as np
    except Exception as e:
        print(f"[-] 安裝失敗，請手動執行: pip install pyserial matplotlib numpy (錯誤: {e})")
        sys.exit(1)

# Add parent directory to sys.path to load serial_link
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import serial_link   # Auto-detect serial ports and default baudrate

# Global Configuration
MAX_POINTS = 200     # History window size for scrolling plot
BAUD_RATE = 460800   # Avionics default baud rate

# Upstream Serial Port reference for sending ARM/DISARM commands
global_ser = None
seq_counter = 0

def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (LE) calculation for uplink binary protocol"""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def build_uplink_frame(cmd: int, arg: int = 0) -> bytes:
    global seq_counter
    seq_counter = (seq_counter + 1) & 0xFF
    header = bytes([0x55, 0xAA, cmd, arg, seq_counter])
    crc = crc16_ccitt(header)
    return header + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def send_serial_cmd(cmd_name: str):
    """Sends both text and binary framing for ARM/DISARM commands over serial"""
    global global_ser
    if global_ser is None or not global_ser.is_open:
        print(f"⚠️ 警告: 串口未連接，無法發送 {cmd_name} 命令！")
        return
        
    try:
        if cmd_name == "ARM":
            frame = build_uplink_frame(0x10, 0x00) # UPLINK_CMD_ARM = 0x10
            text_cmd = b"ARM\r\n"
        elif cmd_name == "DISARM":
            frame = build_uplink_frame(0x11, 0x00) # UPLINK_CMD_DISARM = 0x11
            text_cmd = b"DISARM\r\n"
        else:
            return
            
        global_ser.write(text_cmd)
        global_ser.write(frame)
        global_ser.flush()
        print(f"\r\n⚡ [UPLINK SENT] 已成功傳送上行解鎖指令：{cmd_name} (文字+二進制7B框架)")
    except Exception as e:
        print(f"❌ [UPLINK ERROR] 指令發送失敗: {e}")

# Regex patterns for parsing UART data stream
tele_pattern = re.compile(
    r"\[TELE\] pos:([\d\.-]+),([\d\.-]+),([\d\.-]+) "
    r"vel:([\d\.-]+),([\d\.-]+),([\d\.-]+) "
    r"q:([\d\.-]+),([\d\.-]+),([\d\.-]+),([\d\.-]+)"
)
vf_pattern = re.compile(
    r"\[VF\] h_cm=(-?\d+) v_cms=(-?\d+)"
)
imu_pattern = re.compile(
    r"\[IMU\] a\[mG\]:(-?\d+),(-?\d+),(-?\d+) g\[dps\]:(-?\d+),(-?\d+),(-?\d+)"
)
pwr_pattern = re.compile(r"\[PWR\] bat:(\d+)mV")
gps_pattern = re.compile(r"\[GPS\] fix:(\d+) q:\d+ sat:(\d+)")
health_pattern = re.compile(r"\[HEALTH\] sens=(0x[0-9a-fA-F]+|\d+)\s+ekf=(0x[0-9a-fA-F]+|\d+)")

backup_pattern = re.compile(
    r"\[BACKUP_TELEM\] fsm:(\d+) h_cm:(-?\d+) v_cms:(-?\d+) baro_cm:(-?\d+) az_cg:(-?\d+) q:([\d\.-]+),([\d\.-]+),([\d\.-]+),([\d\.-]+) flags:0x([0-9A-Fa-f]+)"
)

# State mapping helper
FSM_NAMES = [
    "INIT", "PAD", "PAD_ARMED", "BOOST", "COAST",
    "DEPLOY_DROGUE", "APOGEE", "DESCENT", "MAIN_DEPLOY", "LANDED"
]

def get_fsm_name(code_or_str):
    if isinstance(code_or_str, int):
        if 0 <= code_or_str < len(FSM_NAMES):
            return FSM_NAMES[code_or_str]
        return f"UNK({code_or_str})"
    s = str(code_or_str).replace("STATE_", "")
    return s

# Thread-safe queue for serial reading thread -> main plotting thread
data_queue = queue.Queue()

# Deques to store history data (scrolling live view)
time_history = deque(maxlen=MAX_POINTS)
vf_alt_history = deque(maxlen=MAX_POINTS)
ekf_alt_history = deque(maxlen=MAX_POINTS)
baro_alt_history = deque(maxlen=MAX_POINTS)
backup_alt_history = deque(maxlen=MAX_POINTS)

vf_vel_z_history = deque(maxlen=MAX_POINTS)
ekf_vel_z_history = deque(maxlen=MAX_POINTS)
backup_vel_z_history = deque(maxlen=MAX_POINTS)
ekf_vel_mag_history = deque(maxlen=MAX_POINTS)

raw_accel_low_history = deque(maxlen=MAX_POINTS)
raw_accel_high_history = deque(maxlen=MAX_POINTS)
gyro_x_history = deque(maxlen=MAX_POINTS)
gyro_y_history = deque(maxlen=MAX_POINTS)
gyro_z_history = deque(maxlen=MAX_POINTS)

# Lists to store the ENTIRE run history for saving on exit
full_time = []
full_vf_alt = []
full_ekf_alt = []
full_baro_alt = []
full_backup_alt = []
full_vf_vel_z = []
full_ekf_vel_z = []
full_backup_vel_z = []
full_ekf_vel_mag = []
full_accel_low = []
full_accel_high = []
full_gyro_x = []
full_gyro_y = []
full_gyro_z = []
full_pri_fsm = []
full_bak_fsm = []

# Latest state variables
latest_vf_alt = 0.0
latest_ekf_alt = 0.0
latest_baro_alt = 0.0
latest_backup_alt = 0.0
latest_vf_vel_z = 0.0
latest_ekf_vel_z = 0.0
latest_backup_vel_z = 0.0
latest_ekf_vel_mag = 0.0

latest_accel_low = 1.0   # BMI088 Z-axis (default ~1g)
latest_accel_high = 1.0  # ADXL375 Z-axis (default ~1g)
latest_gyro_x = 0.0
latest_gyro_y = 0.0
latest_gyro_z = 0.0

# Dual Board System status
pri_fsm_state = "STATE_INIT"
bak_fsm_state = "STATE_INIT"
battery_mv = 0
gps_sats = 0
gps_fix = 0
sensor_health_bits = 0x00
ekf_health_bits = 0x00

running = True
report_saved = False
baro_baseline = None

def serial_reader_task(port):
    """Reads serial, parses data lines, and inserts snapshot to the queue"""
    global running, baro_baseline, global_ser
    global latest_vf_alt, latest_ekf_alt, latest_baro_alt, latest_backup_alt
    global latest_vf_vel_z, latest_ekf_vel_z, latest_backup_vel_z, latest_ekf_vel_mag
    global latest_accel_low, latest_accel_high, latest_gyro_x, latest_gyro_y, latest_gyro_z
    global pri_fsm_state, bak_fsm_state, battery_mv, gps_sats, gps_fix, sensor_health_bits, ekf_health_bits
    
    print(f"📡 正在開啟串口: {port} @ {BAUD_RATE} ...")
    try:
        ser = serial_link.open_serial(port, BAUD_RATE, timeout=0.1)
        global_ser = ser
    except Exception as e:
        print(f"❌ 錯誤: 無法開啟 {port}: {e}")
        running = False
        return
        
    start_time = time.time()
    print("🚀 雙板姿態與狀態圖表已連接，等待資料流... (可按 Ctrl+C 結束並自動產生高清分析報表)")
    
    while running:
        try:
            if ser.in_waiting:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                parts = line.split(',')
                is_csv = (len(parts) == 9 and all(p.lstrip('-').isdigit() for p in parts))
                if not is_csv and not line.startswith("[TELE]") and not line.startswith("[VF]"):
                    print(line)
                
                if "[BOOT]" in line:
                    print("\r\n🔄 [RESET] 偵測到航電板重置/重新開機！重設時間軸與繪圖快取...")
                    data_queue.put("RESET")
                    start_time = time.time()
                    baro_baseline = None
                    pri_fsm_state = "STATE_INIT"
                    bak_fsm_state = "STATE_INIT"
                    continue
                
                # A. Parse VF (Vertical Filter) state [VF]
                m_vf = vf_pattern.search(line)
                if m_vf:
                    latest_vf_alt = float(m_vf.group(1)) / 100.0   # cm -> m
                    latest_vf_vel_z = float(m_vf.group(2)) / 100.0 # cm/s -> m/s

                # B. Parse EKF state [TELE]
                m_tele = tele_pattern.search(line)
                if m_tele:
                    latest_ekf_alt = float(m_tele.group(3)) # pos_z
                    vx = float(m_tele.group(4))
                    vy = float(m_tele.group(5))
                    vz = float(m_tele.group(6))
                    latest_ekf_vel_z = vz
                    latest_ekf_vel_mag = math.sqrt(vx**2 + vy**2 + vz**2)
                    
                    t_elapsed = time.time() - start_time
                    data_queue.put((
                        t_elapsed, latest_vf_alt, latest_ekf_alt, latest_baro_alt, latest_backup_alt,
                        latest_vf_vel_z, latest_ekf_vel_z, latest_backup_vel_z, latest_ekf_vel_mag,
                        latest_accel_low, latest_accel_high,
                        latest_gyro_x, latest_gyro_y, latest_gyro_z,
                        pri_fsm_state, bak_fsm_state, battery_mv, gps_sats, gps_fix,
                        sensor_health_bits, ekf_health_bits
                    ))

                # C. Parse Backup Board telemetry [BACKUP_TELEM]
                m_bak = backup_pattern.search(line)
                if m_bak:
                    bak_code = int(m_bak.group(1))
                    bak_fsm_state = get_fsm_name(bak_code)
                    latest_backup_alt = float(m_bak.group(2)) / 100.0
                    latest_backup_vel_z = float(m_bak.group(3)) / 100.0

                # D. Parse raw sensor data (9 integers comma-separated line)
                if len(parts) == 9 and all(p.lstrip('-').isdigit() for p in parts):
                    latest_accel_low = int(parts[2]) / 1000.0   # bmi_az (mg -> g)
                    latest_accel_high = int(parts[5]) / 1000.0  # adxl_az (mg -> g)
                    raw_baro = int(parts[8]) / 100.0            # alt(cm -> m)
                    if baro_baseline is None:
                        baro_baseline = raw_baro
                        print(f"📍 偵測氣壓計起點基準 (Baseline): {baro_baseline:.2f} m")
                    latest_baro_alt = raw_baro - baro_baseline
                
                # E. Parse Gyro from [IMU]
                m_imu = imu_pattern.search(line)
                if m_imu:
                    latest_gyro_x = float(m_imu.group(4))
                    latest_gyro_y = float(m_imu.group(5))
                    latest_gyro_z = float(m_imu.group(6))
                
                # F. Parse battery voltage [PWR]
                m_pwr = pwr_pattern.search(line)
                if m_pwr:
                    battery_mv = int(m_pwr.group(1))
                
                # G. Parse GPS state [GPS]
                m_gps = gps_pattern.search(line)
                if m_gps:
                    gps_fix = int(m_gps.group(1))
                    gps_sats = int(m_gps.group(2))
                
                # H. Parse Health status [HEALTH]
                m_health = health_pattern.search(line)
                if m_health:
                    sensor_health_bits = int(m_health.group(1), 16) if 'x' in m_health.group(1) else int(m_health.group(1))
                    ekf_health_bits = int(m_health.group(2), 16) if 'x' in m_health.group(2) else int(m_health.group(2))
                
                # I. Parse Primary FSM state [FSM]
                if "[FSM]" in line:
                    m_state = re.search(r"(STATE_[A-Z_]+)", line)
                    if m_state:
                        pri_fsm_state = get_fsm_name(m_state.group(1))
                    elif "LIFTOFF" in line:
                        pri_fsm_state = "BOOST"
                    elif "BURNOUT" in line:
                        pri_fsm_state = "COAST"
                    elif "DEPLOY_DROGUE" in line:
                        pri_fsm_state = "DEPLOY_DROGUE"
                    elif "APOGEE" in line:
                        pri_fsm_state = "APOGEE"
                    elif "LANDED" in line:
                        pri_fsm_state = "LANDED"
            else:
                time.sleep(0.002)
        except Exception as e:
            pass
            
    try:
        ser.close()
    except:
        pass

def save_full_report():
    """Generates a print-ready report of the entire flight run without overlapping text"""
    global report_saved
    if report_saved or not full_time:
        return
        
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, "..", "..", "simulation_and_data", "flight_plots", f"flight_record_{time.strftime('%Y%m%d_%H%M%S')}.png")
    print(f"\r\n💾 正在產生雙板全感測器與 FSM 分析圖 (點數: {len(full_time)})...")
    report_saved = True
    
    plt.style.use('default')
    fig_full, axs = plt.subplots(2, 2, figsize=(16, 10))
    fig_full.suptitle(f"RocketCom Comprehensive Dual-Board Avionics Report\r\nSaved to: {filename}", fontsize=15, fontweight='bold')
    
    ax_alt, ax_vel = axs[0, 0], axs[0, 1]
    ax_acc, ax_gyro = axs[1, 0], axs[1, 1]
    
    # Subplot 1: Altitudes (VF vs EKF vs Baro vs Backup)
    ax_alt.plot(full_time, full_vf_alt, '#d90429', lw=2.0, label='VF Altitude ($h_{vf}$)')
    ax_alt.plot(full_time, full_ekf_alt, '#0077b6', lw=1.5, label='EKF Altitude ($h_{ekf}$)')
    ax_alt.plot(full_time, full_baro_alt, '#38b000', lw=1.2, ls='--', label='Baro Altitude ($h_{baro}$)')
    if any(full_backup_alt):
        ax_alt.plot(full_time, full_backup_alt, '#7209b7', lw=1.2, ls=':', label='Backup Board Alt ($h_{bak}$)')
    ax_alt.set_ylabel("Altitude (m)", fontsize=11, fontweight='bold')
    ax_alt.set_title("Altitude Estimation (VF vs EKF vs Baro vs Backup)", fontsize=12, fontweight='bold')
    ax_alt.grid(True, linestyle=':', alpha=0.6)
    ax_alt.legend(loc="upper left", fontsize=9)
    
    # Subplot 2: Velocities (VF Vz vs EKF Vz)
    ax_vel.plot(full_time, full_vf_vel_z, '#d90429', lw=2.0, label='VF Vertical Velocity ($v_{z,vf}$)')
    ax_vel.plot(full_time, full_ekf_vel_z, '#0077b6', lw=1.5, label='EKF Vertical Velocity ($v_{z,ekf}$)')
    ax_vel.plot(full_time, full_ekf_vel_mag, '#7209b7', lw=1.2, ls='--', label='Speed Magnitude ($|v|$)')
    ax_vel.set_ylabel("Velocity (m/s)", fontsize=11, fontweight='bold')
    ax_vel.set_title("Vertical Velocity Estimation (VF vs EKF)", fontsize=12, fontweight='bold')
    ax_vel.grid(True, linestyle=':', alpha=0.6)
    ax_vel.legend(loc="upper left", fontsize=9)
    
    # Subplot 3: Z-Axis Accel (BMI088 vs ADXL375)
    ax_acc.plot(full_time, full_accel_low, '#fca311', lw=1.5, label='Low-G BMI088 Z')
    ax_acc.plot(full_time, full_accel_high, '#d90429', lw=1.0, ls='--', label='High-G ADXL375 Z')
    ax_acc.set_ylabel("Acceleration (g)", fontsize=11, fontweight='bold')
    ax_acc.set_xlabel("Time (s)", fontsize=11)
    ax_acc.set_title("Z-Axis Accelerations (g)", fontsize=12, fontweight='bold')
    ax_acc.grid(True, linestyle=':', alpha=0.6)
    ax_acc.legend(loc="upper left", fontsize=9)
    
    # Subplot 4: Gyro 3-Axis
    ax_gyro.plot(full_time, full_gyro_x, '#ef233c', lw=0.9, label='Gyro X')
    ax_gyro.plot(full_time, full_gyro_y, '#38b000', lw=0.9, label='Gyro Y')
    ax_gyro.plot(full_time, full_gyro_z, '#0077b6', lw=1.2, label='Gyro Z')
    ax_gyro.set_ylabel("Angular Rate (dps)", fontsize=11, fontweight='bold')
    ax_gyro.set_xlabel("Time (s)", fontsize=11)
    ax_gyro.set_title("3-Axis Gyro Angular Rates (dps)", fontsize=12, fontweight='bold')
    ax_gyro.grid(True, linestyle=':', alpha=0.6)
    ax_gyro.legend(loc="upper left", fontsize=9)
    
    # Draw staggered, non-overlapping FSM State Transition Markers
    last_pri = None
    last_bak = None
    stagger_idx = 0
    max_alt_val = max(full_vf_alt) if full_vf_alt else 10.0
    
    for t, alt, pri, bak in zip(full_time, full_vf_alt, full_pri_fsm, full_bak_fsm):
        if pri != last_pri or bak != last_bak:
            if pri != "INIT" and pri != "STATE_INIT":
                for ax in [ax_alt, ax_vel]:
                    ax.axvline(x=t, color='#d90429', linestyle=':', alpha=0.7)
                
                offset_y = (stagger_idx % 3) * (max_alt_val * 0.08 + 1.5)
                stagger_idx += 1
                
                label_text = f"主:{pri}\n副:{bak}\n({t:.2f}s)"
                ax_alt.text(t, alt + max_alt_val * 0.05 + offset_y, label_text,
                            rotation=0, fontsize=8, fontweight='bold', color='#d90429',
                            ha='center', va='bottom',
                            bbox=dict(boxstyle='round,pad=0.2', facecolor='#ffffff', edgecolor='#d90429', alpha=0.85))
            last_pri = pri
            last_bak = bak
            
    plt.subplots_adjust(top=0.90, bottom=0.08, left=0.07, right=0.96, hspace=0.35, wspace=0.25)
    fig_full.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close(fig_full)
    print(f"✅ 飛行分析圖表已成功儲存為：{os.path.abspath(filename)}")

def main():
    global running
    
    port = serial_link.auto_port()
    if not port:
        ports = serial_link.list_candidate_ports()
        if not ports:
            print("❌ 系統未檢測到任何可用串口！請插上 USB-TTL 模組後重試。")
            sys.exit(1)
        port = serial_link.prompt_select_port()
        if not port:
            sys.exit(1)
            
    thread = threading.Thread(target=serial_reader_task, args=(port,), daemon=True)
    thread.start()
    
    # Matplotlib Dark Theme Layout
    plt.style.use('dark_background')
    fig, axs = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    fig.canvas.manager.set_window_title("RocketCom Dual-Board Live Dashboard (VF + EKF + Sensors + ARM Control)")
    fig.patch.set_facecolor('#121212')
    
    ax_alt, ax_vel = axs[0, 0], axs[0, 1]
    ax_acc, ax_gyro = axs[1, 0], axs[1, 1]
    
    for ax in [ax_alt, ax_vel, ax_acc, ax_gyro]:
        ax.set_facecolor('#1c1c1c')
        ax.grid(True, color='#444444', linestyle=':')
    
    # Subplot 1: Altitudes (VF vs EKF vs Baro vs Backup)
    line_vf_alt,  = ax_alt.plot([], [], '#ff3b30', lw=2.2, label='VF Altitude ($h_{vf}$)')
    line_ekf_alt, = ax_alt.plot([], [], '#00d2ff', lw=1.8, label='EKF Altitude ($h_{ekf}$)')
    line_baro_alt,= ax_alt.plot([], [], '#28a745', lw=1.2, ls='--', label='Baro Altitude ($h_{baro}$)')
    ax_alt.set_ylabel("Altitude (m)", fontsize=10, fontweight='bold')
    ax_alt.set_title("Altitude Estimation (VF vs EKF vs Baro)", color='white', fontsize=11, fontweight='bold')
    ax_alt.legend(loc="upper left", fontsize=8)
    
    # Subplot 2: Velocities (VF Vz vs EKF Vz)
    line_vf_vel,  = ax_vel.plot([], [], '#ff3b30', lw=2.2, label='VF Vertical Velocity ($v_{z,vf}$)')
    line_ekf_vel, = ax_vel.plot([], [], 'cyan', lw=1.8, label='EKF Vertical Velocity ($v_{z,ekf}$)')
    line_vel_mag, = ax_vel.plot([], [], '#a88beb', lw=1.2, ls='--', label='EKF Speed ($|v|$)')
    ax_vel.set_ylabel("Velocity (m/s)", fontsize=10, fontweight='bold')
    ax_vel.set_title("Vertical Velocity Estimation (VF vs EKF)", color='white', fontsize=11, fontweight='bold')
    ax_vel.legend(loc="upper left", fontsize=8)
    
    # Subplot 3: Z-axis Acceleration (BMI088 vs ADXL375)
    line_acc_low, = ax_acc.plot([], [], '#ffaa00', lw=1.8, label='Low-G BMI088 ($a_{z}$)')
    line_acc_high,= ax_acc.plot([], [], '#ff3b30', lw=1.2, ls='--', label='High-G ADXL375 ($a_{z}$)')
    ax_acc.set_ylabel("Acceleration (g)", fontsize=10, fontweight='bold')
    ax_acc.set_xlabel("Elapsed Time (s)", fontsize=10)
    ax_acc.set_title("Z-Axis Acceleration Raw Data", color='white', fontsize=11, fontweight='bold')
    ax_acc.legend(loc="upper left", fontsize=8)
    
    # Subplot 4: Gyro (3-axis angular rates)
    line_gyro_x, = ax_gyro.plot([], [], '#ff5555', lw=1.2, label='Gyro X')
    line_gyro_y, = ax_gyro.plot([], [], '#55ff55', lw=1.2, label='Gyro Y')
    line_gyro_z, = ax_gyro.plot([], [], '#5555ff', lw=1.5, label='Gyro Z')
    ax_gyro.set_ylabel("Angular Rate (dps)", fontsize=10, fontweight='bold')
    ax_gyro.set_xlabel("Elapsed Time (s)", fontsize=10)
    ax_gyro.set_title("3-Axis Gyro Raw Data", color='white', fontsize=11, fontweight='bold')
    ax_gyro.legend(loc="upper left", fontsize=8)
    
    # Status Banner
    status_text = fig.text(
        0.38, 0.96, "主板: INIT  |  副板: INIT  |  BAT: 0 mV  |  GPS: 0 Sats  |  Health: Sensor OK",
        ha='center', va='center', color='white', fontsize=10, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#222222', edgecolor='#00d2ff', alpha=0.9)
    )

    # Interactive ARM & DISARM GUI Buttons
    ax_arm = fig.add_axes([0.76, 0.94, 0.10, 0.045])
    ax_disarm = fig.add_axes([0.88, 0.94, 0.10, 0.045])

    btn_arm = Button(ax_arm, '🔒 ARM 武裝', color='#d90429', hovercolor='#ff2a42')
    btn_arm.label.set_fontsize(10)
    btn_arm.label.set_fontweight('bold')
    btn_arm.label.set_color('white')

    btn_disarm = Button(ax_disarm, '🔓 DISARM 解除', color='#2b9348', hovercolor='#38b000')
    btn_disarm.label.set_fontsize(10)
    btn_disarm.label.set_fontweight('bold')
    btn_disarm.label.set_color('white')

    def on_arm_clicked(event):
        send_serial_cmd("ARM")

    def on_disarm_clicked(event):
        send_serial_cmd("DISARM")

    btn_arm.on_clicked(on_arm_clicked)
    btn_disarm.on_clicked(on_disarm_clicked)

    fsm_state_ref = {"pri_last": "INIT", "bak_last": "INIT"}
    fsm_artists = []
    stagger_counter = 0

    def init():
        line_vf_alt.set_data([], [])
        line_ekf_alt.set_data([], [])
        line_baro_alt.set_data([], [])
        line_vf_vel.set_data([], [])
        line_ekf_vel.set_data([], [])
        line_vel_mag.set_data([], [])
        line_acc_low.set_data([], [])
        line_acc_high.set_data([], [])
        line_gyro_x.set_data([], [])
        line_gyro_y.set_data([], [])
        line_gyro_z.set_data([], [])
        return (line_vf_alt, line_ekf_alt, line_baro_alt, line_vf_vel, line_ekf_vel, line_vel_mag,
                line_acc_low, line_acc_high, line_gyro_x, line_gyro_y, line_gyro_z, status_text)

    def animate(frame):
        nonlocal stagger_counter
        new_data = False
        pri_lbl = "INIT"
        bak_lbl = "INIT"
        bat = 0
        sats = 0
        fix = 0
        sens_hb = 0
        ekf_hb = 0
        
        while not data_queue.empty():
            item = data_queue.get()
            if item == "RESET":
                time_history.clear()
                vf_alt_history.clear()
                ekf_alt_history.clear()
                baro_alt_history.clear()
                backup_alt_history.clear()
                vf_vel_z_history.clear()
                ekf_vel_z_history.clear()
                backup_vel_z_history.clear()
                ekf_vel_mag_history.clear()
                raw_accel_low_history.clear()
                raw_accel_high_history.clear()
                gyro_x_history.clear()
                gyro_y_history.clear()
                gyro_z_history.clear()
                
                full_time.clear()
                full_vf_alt.clear()
                full_ekf_alt.clear()
                full_baro_alt.clear()
                full_backup_alt.clear()
                full_vf_vel_z.clear()
                full_ekf_vel_z.clear()
                full_backup_vel_z.clear()
                full_ekf_vel_mag.clear()
                full_accel_low.clear()
                full_accel_high.clear()
                full_gyro_x.clear()
                full_gyro_y.clear()
                full_gyro_z.clear()
                full_pri_fsm.clear()
                full_bak_fsm.clear()
                
                for artist in fsm_artists:
                    try: artist.remove()
                    except: pass
                fsm_artists.clear()
                fsm_state_ref["pri_last"] = "INIT"
                fsm_state_ref["bak_last"] = "INIT"
                new_data = True
                continue
                
            (t, vfalt, ealt, balt, bakalt, vfvel, ekfvel, bakvel, vmag,
             alow, ahigh, gx, gy, gz,
             pri_lbl, bak_lbl, bat, sats, fix, sens_hb, ekf_hb) = item
            
            time_history.append(t)
            vf_alt_history.append(vfalt)
            ekf_alt_history.append(ealt)
            baro_alt_history.append(balt)
            backup_alt_history.append(bakalt)

            vf_vel_z_history.append(vfvel)
            ekf_vel_z_history.append(ekfvel)
            backup_vel_z_history.append(bakvel)
            ekf_vel_mag_history.append(vmag)

            raw_accel_low_history.append(alow)
            raw_accel_high_history.append(ahigh)
            gyro_x_history.append(gx)
            gyro_y_history.append(gy)
            gyro_z_history.append(gz)
            
            full_time.append(t)
            full_vf_alt.append(vfalt)
            full_ekf_alt.append(ealt)
            full_baro_alt.append(balt)
            full_backup_alt.append(bakalt)
            full_vf_vel_z.append(vfvel)
            full_ekf_vel_z.append(ekfvel)
            full_backup_vel_z.append(bakvel)
            full_ekf_vel_mag.append(vmag)
            full_accel_low.append(alow)
            full_accel_high.append(ahigh)
            full_gyro_x.append(gx)
            full_gyro_y.append(gy)
            full_gyro_z.append(gz)
            full_pri_fsm.append(pri_lbl)
            full_bak_fsm.append(bak_lbl)
            new_data = True
            
            # Check for Dual Board State Transitions with non-overlapping text placement
            if pri_lbl != fsm_state_ref["pri_last"] or bak_lbl != fsm_state_ref["bak_last"]:
                if pri_lbl != "INIT" and len(time_history) > 0:
                    t_trans = time_history[-1]
                    alt_trans = vf_alt_history[-1]
                    
                    v1 = ax_alt.axvline(x=t_trans, color='#ef233c', linestyle=':', alpha=0.75)
                    v2 = ax_vel.axvline(x=t_trans, color='#ef233c', linestyle=':', alpha=0.75)
                    fsm_artists.append(v1)
                    fsm_artists.append(v2)
                    
                    stagger_counter += 1
                    y_offset = (stagger_counter % 3) * 1.5 + 0.5
                    label_text = f"主:{pri_lbl}\n副:{bak_lbl}\n({t_trans:.1f}s)"
                    
                    t1 = ax_alt.text(t_trans, alt_trans + y_offset, label_text, 
                                     color='#ffcc00', fontsize=7.5, fontweight='bold',
                                     ha='center', va='bottom',
                                     bbox=dict(boxstyle='round,pad=0.2', facecolor='#1c1c1c', edgecolor='#ffcc00', alpha=0.85))
                    fsm_artists.append(t1)
                fsm_state_ref["pri_last"] = pri_lbl
                fsm_state_ref["bak_last"] = bak_lbl
            
        if new_data and len(time_history) > 0:
            t_list = list(time_history)
            
            line_vf_alt.set_data(t_list, list(vf_alt_history))
            line_ekf_alt.set_data(t_list, list(ekf_alt_history))
            line_baro_alt.set_data(t_list, list(baro_alt_history))
            
            line_vf_vel.set_data(t_list, list(vf_vel_z_history))
            line_ekf_vel.set_data(t_list, list(ekf_vel_z_history))
            line_vel_mag.set_data(t_list, list(ekf_vel_mag_history))
            
            line_acc_low.set_data(t_list, list(raw_accel_low_history))
            line_acc_high.set_data(t_list, list(raw_accel_high_history))
            
            line_gyro_x.set_data(t_list, list(gyro_x_history))
            line_gyro_y.set_data(t_list, list(gyro_y_history))
            line_gyro_z.set_data(t_list, list(gyro_z_history))
            
            x_min = t_list[0]
            x_max = t_list[-1]
            ax_acc.set_xlim(x_min, max(x_max, x_min + 5.0))
            ax_gyro.set_xlim(x_min, max(x_max, x_min + 5.0))
            
            alts_arr = np.concatenate([list(vf_alt_history), list(ekf_alt_history), list(baro_alt_history)])
            alt_min, alt_max = np.min(alts_arr), np.max(alts_arr)
            alt_pad = max(abs(alt_max - alt_min) * 0.15, 5.0)
            ax_alt.set_ylim(alt_min - alt_pad, alt_max + alt_pad + 5.0)
            
            vels_arr = np.concatenate([list(vf_vel_z_history), list(ekf_vel_z_history), list(ekf_vel_mag_history)])
            vel_min, vel_max = np.min(vels_arr), np.max(vels_arr)
            vel_pad = max(abs(vel_max - vel_min) * 0.15, 2.0)
            ax_vel.set_ylim(vel_min - vel_pad, vel_max + vel_pad)
            
            acc_arr = np.concatenate([list(raw_accel_low_history), list(raw_accel_high_history)])
            acc_min, acc_max = np.min(acc_arr), np.max(acc_arr)
            acc_pad = max(abs(acc_max - acc_min) * 0.15, 0.5)
            ax_acc.set_ylim(acc_min - acc_pad, acc_max + acc_pad)
            
            gyr_arr = np.concatenate([list(gyro_x_history), list(gyro_y_history), list(gyro_z_history)])
            gyr_min, gyr_max = np.min(gyr_arr), np.max(gyr_arr)
            gyr_pad = max(abs(gyr_max - gyr_min) * 0.15, 50.0)
            ax_gyro.set_ylim(gyr_min - gyr_pad, gyr_max + gyr_pad)
            
            gps_status = f"{sats} Sats (Fix OK)" if fix else f"{sats} Sats (No Fix)"
            health_str = []
            if sens_hb == 0: health_str.append("Sensor OK")
            else: health_str.append(f"Sensor ERR (0x{sens_hb:02X})")
            if ekf_hb == 0: health_str.append("EKF OK")
            else: health_str.append(f"EKF ERR (0x{ekf_hb:02X})")
            
            banner_msg = f"主板: {pri_lbl}  |  副板: {bak_lbl}  |  BAT: {bat} mV  |  GPS: {gps_status}  |  Health: {' / '.join(health_str)}"
            status_text.set_text(banner_msg)
            
            if pri_lbl == "PAD":
                status_text.get_bbox_patch().set_edgecolor('#00d2ff')
            elif pri_lbl == "BOOST":
                status_text.get_bbox_patch().set_edgecolor('#ff3b30')
            elif pri_lbl == "COAST":
                status_text.get_bbox_patch().set_edgecolor('#ffcc00')
            elif pri_lbl in ["DEPLOY_DROGUE", "APOGEE", "DESCENT", "MAIN_DEPLOY"]:
                status_text.get_bbox_patch().set_edgecolor('#28a745')
            elif pri_lbl == "LANDED":
                status_text.get_bbox_patch().set_edgecolor('#ffffff')

        return (line_vf_alt, line_ekf_alt, line_baro_alt, line_vf_vel, line_ekf_vel, line_vel_mag,
                line_acc_low, line_acc_high, line_gyro_x, line_gyro_y, line_gyro_z, status_text)

    ani = animation.FuncAnimation(
        fig, animate, init_func=init, interval=40, blit=False, cache_frame_data=False
    )
    
    plt.subplots_adjust(top=0.90, bottom=0.08, left=0.08, right=0.95, hspace=0.35, wspace=0.25)
    
    def on_close(event):
        global running
        running = False
        save_full_report()
        
    fig.canvas.mpl_connect('close_event', on_close)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\r\n[INFO] 偵測到 Ctrl+C 中斷，正在結束程式並儲存報告...")
    finally:
        running = False
        save_full_report()

if __name__ == '__main__':
    main()
