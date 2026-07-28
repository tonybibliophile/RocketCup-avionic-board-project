#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_live_analyzer.py — Real-time & Offline GPS TTFF & 3D Static Drift Analyzer

Usage (Live Serial Monitor):
    python3 ground_station/tools/gps_live_analyzer.py

Usage (Offline Log Analysis):
    python3 ground_station/tools/gps_live_analyzer.py --file ground_station/logs/gui_serial_continuous_20260717_000741.log
"""

import os
import re
import sys
import time
import queue
import threading
import argparse
import subprocess
import tempfile
import urllib.request
import json
import io
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec

# Add parent directory to path to import serial_link
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.append(PARENT_DIR)

try:
    import serial_link
except ImportError:
    print("[ERROR] Cannot find serial_link module. Make sure this script is in ground_station/tools/")
    sys.exit(1)

# Constants
LAT_TO_M = 111139.0

# ----------------------------------------------------
# OpenStreetMap Background Map Helper (pure python + numpy)
# ----------------------------------------------------
def fetch_map_background(lat, lon, zoom=18):
    try:
        lat_rad = math.radians(lat)
        n = 2.0 ** zoom
        cx = int((lon + 180.0) / 360.0 * n)
        cy = int((1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        
        rows = []
        for y in range(cy-1, cy+2):
            row_tiles = []
            for x in range(cx-1, cx+2):
                url = f"https://tile.openstreetmap.org/{zoom}/{x}/{y}.png"
                req = urllib.request.Request(url, headers={'User-Agent': 'RocketCom-GPS-Analyzer/1.0'})
                with urllib.request.urlopen(req, timeout=3.0) as response:
                    img = plt.imread(io.BytesIO(response.read()), format='png')
                    row_tiles.append(img)
            rows.append(np.concatenate(row_tiles, axis=1))
        
        stitched = np.concatenate(rows, axis=0)
        
        n_inv = 1.0 / n
        lon_left = (cx - 1) * n_inv * 360.0 - 180.0
        lat_rad_top = math.atan(math.sinh(math.pi * (1.0 - 2.0 * (cy - 1) * n_inv)))
        lat_top = math.degrees(lat_rad_top)
        
        lon_right = (cx + 2) * n_inv * 360.0 - 180.0
        lat_rad_bottom = math.atan(math.sinh(math.pi * (1.0 - 2.0 * (cy + 2) * n_inv)))
        lat_bottom = math.degrees(lat_rad_bottom)
        
        extent = [lon_left, lon_right, lat_bottom, lat_top]
        return stitched, extent
    except Exception as e:
        print(f"[Warning] Map download failed: {e}")
        return None, None

class LiveGpsAnalyzer:
    def __init__(self, target_source, baud, ref_loc=None):
        self.source = target_source # Serial port path OR log file path
        self.baud = baud
        self.running = False
        self.is_file_mode = False
        self.save_triggered = False
        
        # Thread-safe data queue
        self.data_queue = queue.Queue()
        
        # Data storage with thread lock
        self.lock = threading.Lock()
        self.times = []         # seconds since first packet (float)
        self.timestamps = []    # original time strings HH:MM:SS.mmm
        self.sats = []          # satellite count
        self.fixes = []         # fix valid (0/1)
        self.lats = []          # latitudes
        self.lons = []          # longitudes
        self.alts = []          # altitudes
        
        # Data for drift calculation (retained across resets for continuous accumulation)
        self.drift_times = []
        self.drift_lats = []
        self.drift_lons = []
        self.drift_alts = []
        self.drift_sats = []
        
        # Metric variables
        self.start_sys_time = None
        self.first_fix_time_str = None
        self.ttff = None
        self.prev_ok = None
        
        # Session log file (only created in live serial mode)
        self.log_file = None
        
        # Get reference GPS coordinates or use manual reference
        self.comp_loc = ref_loc
        
        # Map background variables
        self.map_img = None
        self.map_extent = None
        self.map_download_started = False
        
        # Trigger map download immediately if manual reference is provided
        if self.comp_loc:
            self.trigger_map_download(self.comp_loc[0], self.comp_loc[1])

    def trigger_map_download(self, lat, lon):
        if self.map_download_started:
            return
        self.map_download_started = True
        
        def download_worker():
            img, extent = fetch_map_background(lat, lon, zoom=18)
            if img is not None:
                with self.lock:
                    self.map_img = img
                    self.map_extent = extent
                print("[System] Map background downloaded successfully!")
            else:
                print("[System] Map download failed. Falling back to relative meters plot.")
                
        threading.Thread(target=download_worker, daemon=True).start()

    def save_session_results(self, fig):
        """Automatically saves plots, report statistics, and telemetry log to a dedicated run folder on exit"""
        with self.lock:
            if self.save_triggered:
                return
            self.save_triggered = True
            has_data = len(self.lats) > 0
            d_lats = np.array(self.drift_lats)
            d_lons = np.array(self.drift_lons)
            d_alts = np.array(self.drift_alts)
            
        if not has_data:
            print("[System] No GPS data received. Skipping result saving.")
            return

        now_str = time.strftime("%Y%m%d_%H%M%S")
        mode_prefix = "run_offline" if self.is_file_mode else "run_live"
        run_dir = os.path.join(PARENT_DIR, "runs", f"{mode_prefix}_{now_str}")
        os.makedirs(run_dir, exist_ok=True)
        
        print(f"\n[System] Saving session results to: {os.path.relpath(run_dir, PARENT_DIR)}")

        # 1. Save Figure Plot
        plot_path = os.path.join(run_dir, "gps_analysis_plot.png")
        try:
            fig.savefig(plot_path, dpi=150, bbox_inches='tight')
            print(f"• Saved final plot   : {os.path.basename(plot_path)}")
        except Exception as e:
            print(f"[Warning] Failed to save plot: {e}")

        # 2. Save Text Summary Report
        report_path = os.path.join(run_dir, "gps_accuracy_report.txt")
        try:
            with open(report_path, "w", encoding="utf-8") as rf:
                rf.write("==================================================\n")
                rf.write("       ROCKETCOM GPS ACCURACY METRICS REPORT      \n")
                rf.write("==================================================\n")
                rf.write(f"Session Mode   : {'Offline File Analysis' if self.is_file_mode else 'Live Serial Monitor'}\n")
                rf.write(f"Source         : {self.source}\n")
                rf.write(f"Report Time    : {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                
                with self.lock:
                    if self.start_sys_time and self.timestamps:
                        rf.write(f"• Start Time          : {self.timestamps[0]}\n")
                        rf.write(f"• End Time            : {self.timestamps[-1]}\n")
                    if self.first_fix_time_str:
                        rf.write(f"• First Fix Time      : {self.first_fix_time_str}\n")
                        rf.write(f"• Time to First Fix   : {self.ttff:.2f} seconds\n")
                    else:
                        rf.write("• First Fix Time      : No Fix\n")
                    if self.fixes:
                        rf.write(f"• Total Data Packets  : {len(self.fixes)}\n")
                        rf.write(f"• Fix Ratio           : {sum(self.fixes)/len(self.fixes)*100:.1f}%\n")
                        
                rf.write("\n--------------------------------------------------\n")
                rf.write("             STATIC DRIFT STATISTICS              \n")
                rf.write("--------------------------------------------------\n")
                if len(d_lats) >= 2:
                    mean_lat = np.mean(d_lats)
                    mean_lon = np.mean(d_lons)
                    y_m = (d_lats - mean_lat) * LAT_TO_M
                    x_m = (d_lons - mean_lon) * (LAT_TO_M * np.cos(np.radians(mean_lat)))
                    dist_m = np.sqrt(x_m**2 + y_m**2)
                    
                    cep_50 = np.percentile(dist_m, 50)
                    r_95 = np.percentile(dist_m, 95)
                    max_drift = np.max(dist_m)
                    
                    rf.write(f"• Average Latitude    : {mean_lat:.8f} deg\n")
                    rf.write(f"• Average Longitude   : {mean_lon:.8f} deg\n")
                    rf.write(f"• Valid GPS Fix Points: {len(d_lats)}\n")
                    rf.write(f"• CEP 50% Error Radius : {cep_50:.3f} meters\n")
                    rf.write(f"• 95% Error Circle     : {r_95:.3f} meters\n")
                    rf.write(f"• Max Drift Distance   : {max_drift:.3f} meters\n")
                    rf.write(f"• Horizontal Std (N-S) : {np.std(y_m):.3f} meters\n")
                    rf.write(f"• Horizontal Std (E-W) : {np.std(x_m):.3f} meters\n")
                    rf.write(f"• Vertical Std (Alt)   : {np.std(d_alts):.3f} meters\n")
                    rf.write(f"• Altitude Range       : {np.min(d_alts):.1f}m to {np.max(d_alts):.1f}m\n")
                else:
                    rf.write("• Insufficient GPS fix points for static drift analysis.\n")
                    
                if self.comp_loc:
                    comp_lat, comp_lon, _, comp_src = self.comp_loc
                    rf.write("\n--------------------------------------------------\n")
                    rf.write("           COMPARISON TO REFERENCE GPS            \n")
                    rf.write("--------------------------------------------------\n")
                    rf.write(f"• Ref Coordinates     : {comp_lat:.8f}, {comp_lon:.8f}\n")
                    rf.write(f"• Ref Source          : {comp_src}\n")
                    if len(d_lats) >= 1:
                        mean_lat = np.mean(d_lats)
                        mean_lon = np.mean(d_lons)
                        comp_dy = (comp_lat - mean_lat) * LAT_TO_M
                        comp_dx = (comp_lon - mean_lon) * (LAT_TO_M * np.cos(np.radians(mean_lat)))
                        comp_dist = np.sqrt(comp_dx**2 + comp_dy**2)
                        rf.write(f"• Distance to Reference: {comp_dist:.3f} meters\n")
            print(f"• Saved summary report: {os.path.basename(report_path)}")
        except Exception as e:
            print(f"[Warning] Failed to save report: {e}")

        # 3. Move/Copy Raw Serial Log File (Live Mode Only)
        if not self.is_file_mode and self.log_file:
            log_path = self.log_file.name
            try:
                self.log_file.close()
                self.log_file = None
                dest_log_path = os.path.join(run_dir, "raw_telemetry.log")
                os.rename(log_path, dest_log_path)
                print(f"• Moved raw serial log: {os.path.basename(dest_log_path)}")
            except Exception as e:
                print(f"[Warning] Failed to move log file: {e}")
                
        print(f"✨ [System] All session results saved successfully in ground_station/runs/{os.path.basename(run_dir)}/\n")

    def parse_time_str(self, ts_str):
        h, m, s = ts_str.split(":")
        return float(h) * 3600 + float(m) * 60 + float(s)

    def file_read_thread(self):
        """Background thread to read all lines from a log file offline"""
        print(f"[File] Reading log file: {self.source}...")
        try:
            with open(self.source, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    ts_match = re.match(r"^\[([\d\:\.]+)\]", line)
                    if ts_match:
                        ts = ts_match.group(1)
                        raw_line = line[ts_match.end():].strip()
                        self.data_queue.put((ts, raw_line))
            print(f"[File] Finished reading log file. Plotting all data...")
        except Exception as e:
            print(f"[ERROR] Failed to read log file {self.source}: {e}")

    def serial_read_thread(self):
        """Background thread to read serial port in real-time"""
        print(f"[Serial] Connecting to {self.source} @ {self.baud}...")
        try:
            ser = serial_link.open_serial(self.source, self.baud, timeout=1.0)
            ser.reset_input_buffer()
            
            # Setup temporary live log file
            log_dir = os.path.join(PARENT_DIR, "logs")
            os.makedirs(log_dir, exist_ok=True)
            now_str = time.strftime("%Y%m%d_%H%M%S")
            log_file_path = os.path.join(log_dir, f"gps_live_analysis_{now_str}.log")
            self.log_file = open(log_file_path, "w", encoding="utf-8")
            self.log_file.write(f"--- GPS LIVE ANALYSIS SESSION START AT {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n")
            self.log_file.flush()
            
            print(f"[Serial] Connection established! Buffering data...")
        except Exception as e:
            print(f"[ERROR] Failed to open serial port {self.source}: {e}")
            self.running = False
            return

        while self.running:
            try:
                if ser.in_waiting:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    
                    ts = time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"
                    log_line = f"[{ts}] {line}"
                    
                    if self.log_file:
                        self.log_file.write(log_line + "\n")
                        self.log_file.flush()
                    
                    self.data_queue.put((ts, line))
                else:
                    time.sleep(0.002)
            except Exception as e:
                print(f"[Warning] Serial read exception: {e}")
                time.sleep(1.0)
                
        ser.close()

    def process_queue(self):
        """Process queued logs"""
        while not self.data_queue.empty():
            ts, line = self.data_queue.get()
            
            if "[GPS]" in line or "[GS_PKT]" in line:
                if "[GPS]" in line:
                    fix_match = re.search(r"fix:(\d+)", line)
                    sat_match = re.search(r"sat:(\d+)", line)
                    coord_match = re.search(r"([\+\-]\d+\.\d+),([\+\-]\d+\.\d+)", line)
                    alt_match = re.search(r"alt:(\-?\d+)m", line)
                    ok_match = re.search(r"ok:(\d+)", line)
                    ok_val = int(ok_match.group(1)) if ok_match else 0
                else:
                    # [GS_PKT] format: gps:8/1 pos:+22.991234,+120.211234 galt:45m
                    m_gps = re.search(r"gps:(\d+)/(\d+)", line)
                    sat_match = m_gps if m_gps else None
                    fix_val = int(m_gps.group(2)) if m_gps else 0
                    sats_val = int(m_gps.group(1)) if m_gps else 0
                    fix_match = True
                    coord_match = re.search(r"pos:([\+\-]\d+\.\d+),([\+\-]\d+\.\d+)", line)
                    alt_match = re.search(r"galt:(\-?\d+)m", line)
                    ok_val = 0

                if fix_match and sat_match:
                    if "[GPS]" in line:
                        fix_val = int(fix_match.group(1))
                        sats_val = int(sat_match.group(1))

                    with self.lock:
                        # 1. Detect Reset (ok count decreases)
                        if "[GPS]" in line and self.prev_ok is not None and ok_val < self.prev_ok - 100:
                            print(f"\n🔄 [{ts}] Reset detected! Re-aligning timeline (retaining accumulated drift stats)...")
                            self.times.clear()
                            self.timestamps.clear()
                            self.sats.clear()
                            self.fixes.clear()
                            self.lats.clear()
                            self.lons.clear()
                            self.alts.clear()
                            
                            self.start_sys_time = None
                            self.first_fix_time_str = None
                            self.ttff = None
                        if "[GPS]" in line:
                            self.prev_ok = ok_val
                        
                        # 2. Reset time offset
                        t_sec = self.parse_time_str(ts)
                        if self.start_sys_time is None:
                            self.start_sys_time = t_sec
                            
                        rel_time = t_sec - self.start_sys_time
                        if rel_time < 0:
                            rel_time += 24 * 3600
                            
                        # 3. Store raw data
                        self.times.append(rel_time)
                        self.timestamps.append(ts)
                        self.sats.append(sats_val)
                        self.fixes.append(fix_val)
                        
                        # 4. Parse coordinates & altitude
                        lat, lon, alt = 0.0, 0.0, 0.0
                        if coord_match and alt_match:
                            try:
                                lat = float(coord_match.group(1))
                                lon = float(coord_match.group(2))
                                alt = float(alt_match.group(1))
                            except (ValueError, IndexError):
                                pass

                        self.lats.append(lat)
                        self.lons.append(lon)
                        self.alts.append(alt)
                        
                        # Save drift data only when GPS is fixed and coords are valid
                        if fix_val > 0 and abs(lat) > 0.01 and abs(lon) > 0.01:
                            if self.first_fix_time_str is None:
                                self.first_fix_time_str = ts
                                self.ttff = rel_time
                                if not self.is_file_mode:
                                    print(f"\n🎉 [GPS 定位成功！] 抓到位置所需時間 (TTFF): {self.ttff:.2f} 秒 (鎖定時刻: {ts})\n")
                                # Trigger map download centered at first fix location if no reference is set
                                if not self.comp_loc:
                                    self.trigger_map_download(lat, lon)
                                
                            self.drift_times.append(rel_time)
                            self.drift_lats.append(lat)
                            self.drift_lons.append(lon)
                            self.drift_alts.append(alt)
                            self.drift_sats.append(sats_val)

    def start(self, is_file_mode=False):
        self.running = True
        self.is_file_mode = is_file_mode
        
        target_thread = self.file_read_thread if is_file_mode else self.serial_read_thread
        self.thread = threading.Thread(target=target_thread, daemon=True)
        self.thread.start()

        # Initialize matplotlib plots using GridSpec
        fig = plt.figure(figsize=(16, 9))
        mode_str = "Offline File Mode" if is_file_mode else "Live Serial Mode"
        fig.suptitle(f"RocketCom GPS Live TTFF & 3D Static Drift Analyzer [{mode_str}]", fontsize=16, fontweight='bold')
        
        gs = gridspec.GridSpec(3, 4, figure=fig)
        
        # Subplot 1: Map / 2D Drift (Spans 3 rows, 3 columns on left)
        ax2 = fig.add_subplot(gs[:, :3])
        
        # Subplot 2: Satellites & Fix vs Time (Top right)
        ax1 = fig.add_subplot(gs[0, 3])
        self.ax1_twin = ax1.twinx()
        
        # Subplot 3: Altitude vs Time (Middle right)
        ax3 = fig.add_subplot(gs[1, 3], sharex=ax1)
        
        # Subplot 4: Text metrics dashboard (Bottom right)
        ax4 = fig.add_subplot(gs[2, 3])
        ax4.axis('off')
        
        def update_plots(frame):
            self.process_queue()
            
            with self.lock:
                if not self.times:
                    return (ax1, ax2, ax3, ax4)
                
                # Copy values for plotting under lock
                t_arr = np.array(self.times)
                sats_arr = np.array(self.sats)
                fixes_arr = np.array(self.fixes)
                
                d_t_arr = np.array(self.drift_times)
                d_lats = np.array(self.drift_lats)
                d_lons = np.array(self.drift_lons)
                d_alts = np.array(self.drift_alts)
                
            # Read map settings under lock
            with self.lock:
                map_img = self.map_img
                map_extent = self.map_extent
                
            # ----------------------------------------------------
            # Subplot 1: Sats & Fix Status vs Time (top-right)
            # ----------------------------------------------------
            ax1.clear()
            color = 'tab:blue'
            ax1.set_xlabel("Time (s)")
            ax1.set_ylabel("Sats", color=color)
            ax1.plot(t_arr, sats_arr, color=color, linewidth=1.5, label="Sats")
            ax1.tick_params(axis='y', labelcolor=color)
            ax1.grid(True, linestyle=":", alpha=0.6)
            
            ax1_twin = self.ax1_twin
            ax1_twin.clear()
            ax1_twin.yaxis.tick_right()
            ax1_twin.yaxis.set_label_position("right")
            color = 'tab:red'
            ax1_twin.set_ylabel("Fix", color=color)
            ax1_twin.step(t_arr, fixes_arr, color=color, where='post', linewidth=1.5, label="Fix")
            ax1_twin.tick_params(axis='y', labelcolor=color)
            ax1_twin.set_yticks([0, 1])
            ax1_twin.set_yticklabels(["No", "Fix"])
            
            ax1.set_title("Sats & Fix Status")
            
            # ----------------------------------------------------
            # Subplot 2: 2D Horizontal Position Drift & Map (Left 3/4)
            # ----------------------------------------------------
            ax2.clear()
            if len(d_lats) >= 2:
                mean_lat = np.mean(d_lats)
                mean_lon = np.mean(d_lons)
                lon_to_m = LAT_TO_M * math.cos(math.radians(mean_lat))
                
                # Projection to meters for calculations
                y_m = (d_lats - mean_lat) * LAT_TO_M
                x_m = (d_lons - mean_lon) * lon_to_m
                dist_m = np.sqrt(x_m**2 + y_m**2)
                
                cep_50 = np.percentile(dist_m, 50)
                r_95 = np.percentile(dist_m, 95)
                max_drift = np.max(dist_m)
                
                # Check if we should plot on map
                if map_img is not None and map_extent is not None:
                    # Plot in actual Latitude/Longitude coordinates
                    ax2.imshow(map_img, extent=map_extent, origin='upper', alpha=0.85)
                    
                    sc = ax2.scatter(d_lons, d_lats, c=d_t_arr, cmap="viridis", s=15, alpha=0.8, edgecolors='none')
                    ax2.plot(mean_lon, mean_lat, 'r*', markersize=12, label="Mean Center")
                    
                    # Draw CEP 50% and 95% circles as ellipses (converting meters to degrees)
                    from matplotlib.patches import Ellipse
                    r_lat_50 = cep_50 / LAT_TO_M
                    r_lon_50 = cep_50 / lon_to_m
                    ellipse_50 = Ellipse((mean_lon, mean_lat), width=2*r_lon_50, height=2*r_lat_50, 
                                         color='b', fill=False, linestyle='--', label=f"CEP 50% ({cep_50:.2f}m)")
                    
                    r_lat_95 = r_95 / LAT_TO_M
                    r_lon_95 = r_95 / lon_to_m
                    ellipse_95 = Ellipse((mean_lon, mean_lat), width=2*r_lon_95, height=2*r_lat_95, 
                                         color='r', fill=False, linestyle=':', label=f"95% Circle ({r_95:.2f}m)")
                    
                    ax2.add_patch(ellipse_50)
                    ax2.add_patch(ellipse_95)
                    
                    # Plot reference GPS
                    comp_dist = 0
                    if self.comp_loc:
                        comp_lat, comp_lon, _, comp_src = self.comp_loc
                        comp_dy = (comp_lat - mean_lat) * LAT_TO_M
                        comp_dx = (comp_lon - mean_lon) * lon_to_m
                        comp_dist = np.sqrt(comp_dx**2 + comp_dy**2)
                        ax2.plot(comp_lon, comp_lat, 'gX', markersize=10, label=f"Ref GPS ({comp_src})")
                    
                    # Adjust limits dynamically centered on mean, zoomed to drift + reference
                    max_offset_m = max(max_drift * 1.5, 15.0)
                    if self.comp_loc and comp_dist < 500: # Only zoom out to include ref if close enough
                        max_offset_m = max(max_offset_m, comp_dist * 1.2)
                        
                    offset_lat = max_offset_m / LAT_TO_M
                    offset_lon = max_offset_m / lon_to_m
                    
                    ax2.set_xlim(mean_lon - offset_lon, mean_lon + offset_lon)
                    ax2.set_ylim(mean_lat - offset_lat, mean_lat + offset_lat)
                    ax2.set_aspect(1.0 / math.cos(math.radians(mean_lat)))
                else:
                    # Fallback: Plot relative meters from center
                    sc = ax2.scatter(x_m, y_m, c=d_t_arr, cmap="viridis", s=15, alpha=0.7, edgecolors='none')
                    ax2.plot(0, 0, 'r*', markersize=12, label="Mean Center")
                    
                    circle_50 = plt.Circle((0, 0), cep_50, color='b', fill=False, linestyle='--', label=f"CEP 50% ({cep_50:.2f}m)")
                    circle_95 = plt.Circle((0, 0), r_95, color='r', fill=False, linestyle=':', label=f"95% Error Circle ({r_95:.2f}m)")
                    ax2.add_patch(circle_50)
                    ax2.add_patch(circle_95)
                    
                    comp_dx, comp_dy = None, None
                    if self.comp_loc:
                        comp_lat, comp_lon, _, comp_src = self.comp_loc
                        comp_dy = (comp_lat - mean_lat) * LAT_TO_M
                        comp_dx = (comp_lon - mean_lon) * lon_to_m
                        comp_dist = np.sqrt(comp_dx**2 + comp_dy**2)
                        if comp_dist <= 200:
                            ax2.plot(comp_dx, comp_dy, 'gX', markersize=10, label=f"PC GPS ({comp_src})")
                    
                    max_limit_val = max_drift
                    if comp_dx is not None and comp_dy is not None:
                        comp_dist = np.sqrt(comp_dx**2 + comp_dy**2)
                        if comp_dist <= 200:
                            max_limit_val = max(max_limit_val, comp_dist)
                    max_limit = max(max_limit_val * 1.2, 2.0)
                    ax2.set_xlim(-max_limit, max_limit)
                    ax2.set_ylim(-max_limit, max_limit)
                    ax2.set_aspect('equal', 'box')
                
                ax2.legend(loc="upper right")
            else:
                ax2.text(0.5, 0.5, "Waiting for valid GPS fix...", ha='center', va='center', fontsize=12)
                
            ax2.set_xlabel("East-West Deviation (m)" if map_img is None else "Longitude (deg)")
            ax2.set_ylabel("North-South Deviation (m)" if map_img is None else "Latitude (deg)")
            ax2.set_title("2D Horizontal Static Drift" if map_img is None else "2D Drift on Map Background")
            ax2.grid(True, linestyle=":", alpha=0.6)
            
            # ----------------------------------------------------
            # Subplot 3: Altitude vs Time (middle-right)
            # ----------------------------------------------------
            ax3.clear()
            with self.lock:
                valid_alt_indices = [i for i, f in enumerate(self.fixes) if f > 0 and i < len(self.alts) and i < len(self.times)]
                if valid_alt_indices:
                    alt_times = [self.times[i] for i in valid_alt_indices]
                    alt_vals = [self.alts[i] for i in valid_alt_indices]
                    ax3.plot(alt_times, alt_vals, 'g-', linewidth=1.5, label="Altitude")
                    ax3.set_ylabel("Altitude (m)")
                else:
                    ax3.text(0.5, 0.5, "Waiting for fix...", ha='center', va='center', fontsize=10)
                    
            ax3.set_xlabel("Time (s)")
            ax3.set_title("GPS Altitude")
            ax3.grid(True, linestyle=":", alpha=0.6)
            
            # ----------------------------------------------------
            # Subplot 4: Text panel with metrics (bottom-right)
            # ----------------------------------------------------
            ax4.clear()
            ax4.axis('off')
            
            stats_text = ""
            stats_text += "=== [GPS Accuracy Metrics] ===\n"
            stats_text += "=" * 32 + "\n"
            
            with self.lock:
                if self.start_sys_time:
                    stats_text += f"• Start      : {self.timestamps[0]}\n"
                else:
                    stats_text += "• Status     : Waiting...\n"
                    
                if self.first_fix_time_str:
                    stats_text += f"• First Fix  : {self.first_fix_time_str}\n"
                    stats_text += f"• TTFF       : {self.ttff:.2f} s\n"
                else:
                    stats_text += "• First Fix  : No Fix\n"
                    stats_text += "• TTFF       : Cal...\n"
                
                if len(self.fixes) > 0:
                    stats_text += f"• Fix Status : {'FIXED' if self.fixes[-1] > 0 else 'NO FIX'}\n"
                    stats_text += f"• Sats       : {self.sats[-1]} Sats\n"
                    if self.fixes[-1] > 0 and len(self.lats) > 0:
                        stats_text += f"• Coords     : {self.lats[-1]:.5f}, {self.lons[-1]:.5f}\n"
                        stats_text += f"• Altitude   : {self.alts[-1]:.1f} m\n"
                
            stats_text += "\n=== [Static Drift Stats] ===\n"
            stats_text += "-" * 32 + "\n"
            
            if len(d_lats) >= 5:
                mean_lat = np.mean(d_lats)
                mean_lon = np.mean(d_lons)
                y_m = (d_lats - mean_lat) * LAT_TO_M
                x_m = (d_lons - mean_lon) * (LAT_TO_M * np.cos(np.radians(mean_lat)))
                dist_m = np.sqrt(x_m**2 + y_m**2)
                
                cep_50 = np.percentile(dist_m, 50)
                r_95 = np.percentile(dist_m, 95)
                max_drift = np.max(dist_m)
                
                stats_text += f"• Fix Points : {len(d_lats)} pts\n"
                stats_text += f"• CEP 50%    : {cep_50:.2f} m\n"
                stats_text += f"• 95% Circle : {r_95:.2f} m\n"
                stats_text += f"• Max Drift  : {max_drift:.2f} m\n"
                stats_text += f"• Std (N-S)  : {np.std(y_m):.2f} m\n"
                stats_text += f"• Std (E-W)  : {np.std(x_m):.2f} m\n"
                stats_text += f"• Std (Alt)  : {np.std(d_alts):.2f} m\n"
                stats_text += f"• Alt Range  : {np.min(d_alts):.1f}/{np.max(d_alts):.1f}m\n"
            else:
                stats_text += "• Fix points < 5. Waiting...\n"

            # Add computer location comparison
            if self.comp_loc:
                comp_lat, comp_lon, _, comp_src = self.comp_loc
                stats_text += "\n=== [Distance to Ref] ===\n"
                stats_text += f"• Ref Coords : {comp_lat:.5f}, {comp_lon:.5f}\n"
                stats_text += f"• Ref Source : {comp_src}\n"
                if len(d_lats) >= 1:
                    mean_lat = np.mean(d_lats)
                    mean_lon = np.mean(d_lons)
                    comp_dy = (comp_lat - mean_lat) * LAT_TO_M
                    comp_dx = (comp_lon - mean_lon) * (LAT_TO_M * np.cos(np.radians(mean_lat)))
                    comp_dist = np.sqrt(comp_dx**2 + comp_dy**2)
                    if comp_dist > 1000:
                        stats_text += f"• Distance   : {comp_dist/1000.0:.3f} km\n"
                    else:
                        stats_text += f"• Distance   : {comp_dist:.2f} m\n"
                else:
                    stats_text += "• Distance   : Waiting...\n"

            # Draw a beautiful card with border, padding and nice linespacing
            bbox_props = dict(boxstyle="round,pad=0.5", fc="#F8F9FA", ec="#DEE2E6", lw=1.2)
            font_sz = 8.2 if self.comp_loc else 8.8
            line_sp = 1.3 if self.comp_loc else 1.45
            ax4.text(0.02, 0.90, stats_text, family='monospace', fontsize=font_sz, 
                     va='top', ha='left', linespacing=line_sp, bbox=bbox_props)
            ax4.set_title("Accuracy Dashboard", fontweight='bold', pad=10)
            
            fig.tight_layout()
            return (ax1, ax2, ax3, ax4)
        
        # Handle close window event to save files
        fig.canvas.mpl_connect('close_event', lambda event: self.save_session_results(fig))
        
        # Handle Ctrl+C exit in terminal cleanly
        import signal
        def signal_handler(sig, frame):
            print("\n👋 Terminating analyzer...")
            plt.close(fig)
            
        signal.signal(signal.SIGINT, signal_handler)
        
        # Refresh plots every 300ms
        ani = animation.FuncAnimation(fig, update_plots, interval=300, save_count=100)
        
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GPS Live TTFF & 3D Static Drift Analyzer")
    parser.add_argument("--port", default="AUTO", help="Serial port path (default: AUTO)")
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate (default: 460800)")
    parser.add_argument("--file", default=None, help="Path to a log file to analyze offline")
    parser.add_argument("--ref", default=None, help="Reference coordinates for comparison, formatted as 'lat,lon' (e.g. --ref 22.99,120.22)")
    args = parser.parse_args()

    ref_loc = None
    if args.ref:
        try:
            lat_str, lon_str = args.ref.split(",")
            ref_loc = (float(lat_str.strip()), float(lon_str.strip()), 0.0, "User Ref")
            print(f"[System] Using user-supplied reference coordinates: {ref_loc[0]:.6f}, {ref_loc[1]:.6f}\n")
        except Exception as e:
            print(f"[ERROR] Invalid reference format. Use '--ref lat,lon' (e.g., --ref 22.995,120.218)")
            sys.exit(1)

    if args.file:
        # Offline log analysis mode
        if not os.path.exists(args.file):
            print(f"[ERROR] Specified log file does not exist: {args.file}")
            sys.exit(1)
        analyzer = LiveGpsAnalyzer(args.file, args.baud, ref_loc=ref_loc)
        analyzer.start(is_file_mode=True)
    else:
        # Live serial mode - Incremental Hot-Plug Detection for Power-On TTFF Test
        port_to_open = args.port
        if str(args.port).upper() == "AUTO":
            # 1. 抓取啟動時系統已常駐的舊串口快照（全部忽略，避免亂抓舊裝置）
            initial_candidates = set(p for p in serial_link.list_candidate_ports() if "bluetooth" not in p.lower() and "incoming" not in p.lower())
            
            print("\n" + "=" * 62)
            print("    ⏳ RocketCom GPS Live Analyzer — 上電增量快照追蹤模式")
            print("=" * 62)
            if initial_candidates:
                print("已記錄系統當前常駐串口（連連看測試將自動忽略舊裝置）：")
                for p in sorted(initial_candidates):
                    print(f"  • {p}")
            else:
                print("系統當前無已連接之 USB 串口裝置。")
            print("-" * 62)
            print("👉 請給航電板【上電】或【插上 USB 纜線】...")
            print("系統將 100% 精準鎖定「上電後全新出現的航電串口」，絕不亂抓！")
            print("(若航電板早已上電，請直接按下 [Enter] 鍵手動選取串口)\n")

            new_found_port = None
            dots = [".  ", ".. ", "...", "   "]
            dot_idx = 0
            
            # 非阻塞微輪詢，同時監聽鍵盤 Enter 與熱插拔增量串口
            try:
                import select
                while True:
                    # 檢查是否有新出現的串口 (New Port = Current - Initial)
                    current_candidates = set(p for p in serial_link.list_candidate_ports() if "bluetooth" not in p.lower() and "incoming" not in p.lower())
                    new_ports = current_candidates - initial_candidates
                    
                    if new_ports:
                        # 優先尋找帶有 usbmodem (STM32 USB-CDC) 的新裝置
                        cdc_new = [p for p in new_ports if "usbmodem" in p.lower() or "usbserial" in p.lower()]
                        new_found_port = cdc_new[0] if cdc_new else list(new_ports)[0]
                        print(f"\n\n⚡ [偵測到航電板上電！] 增量捕獲新串口: {new_found_port}")
                        break
                    
                    # 檢查使用者是否按下 Enter 要求手動從舊串口選取
                    r, _, _ = select.select([sys.stdin], [], [], 0.3)
                    if r:
                        line = sys.stdin.readline()
                        # 按下 Enter 跳出輪詢，轉入列表選單
                        break

                    sys.stdout.write(f"\r[⏳ 等待航電板上電{dots[dot_idx]}] 正在監控串口增量... (按 Enter 可手動選擇/Ctrl+C退出)")
                    sys.stdout.flush()
                    dot_idx = (dot_idx + 1) % 4
            except (KeyboardInterrupt, EOFError):
                print("\n\n[System] 被使用者取消。")
                sys.exit(0)
            except Exception:
                # Windows / 相容性 fallback
                pass

            if new_found_port:
                port_to_open = new_found_port
                print(f"🟢 [上電秒連成功] 連接至航電板: {port_to_open} @ {args.baud} baud\n")
            else:
                # 若無新串口增量，顯示所有串口讓使用者手動挑選
                current_candidates = [p for p in serial_link.list_candidate_ports() if "bluetooth" not in p.lower()]
                if not current_candidates:
                    print("\n❌ [ERROR] 未找到任何串口。請確認 USB 纜線已插好。")
                    sys.exit(1)
                    
                print("\n" + "=" * 58)
                print("    🚀 RocketCom GPS Live Analyzer — 串口選擇")
                print("=" * 58)
                print("偵測到的可用串口裝置 (Serial Ports)：")
                for idx, p in enumerate(current_candidates, 1):
                    tag = "★ [推薦 (USB-CDC/TTL)]" if idx == 1 else ""
                    print(f"  [{idx}] {p} {tag}")
                print("-" * 58)
                
                try:
                    user_sel = input(f"請選擇串口編號 [1-{len(current_candidates)}] (按 Enter 預設選 [1]): ").strip()
                    if user_sel == "":
                        port_to_open = current_candidates[0]
                    else:
                        try:
                            idx = int(user_sel) - 1
                            if 0 <= idx < len(current_candidates):
                                port_to_open = current_candidates[idx]
                            else:
                                port_to_open = current_candidates[0]
                        except ValueError:
                            port_to_open = user_sel
                except (KeyboardInterrupt, EOFError):
                    print("\n[System] 被使用者取消。")
                    sys.exit(0)
                
                print(f"\n🟢 已選擇串口: {port_to_open} @ {args.baud} baud\n")
        else:
            port_to_open = serial_link.resolve_port(args.port)

        analyzer = LiveGpsAnalyzer(port_to_open, args.baud, ref_loc=ref_loc)
        analyzer.start(is_file_mode=False)
