#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_gps_drift.py — GPS 首次定位時間 (TTFF) 與靜態漂移即時監控工具

用法：
    python3 ground_station/monitor_gps_drift.py
"""

import os
import re
import sys
import time
import glob
import numpy as np

# 預設路徑（相對於本腳本所在位置）
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")

def parse_time_str(ts_str):
    """將 HH:MM:SS.mmm 轉為當天的秒數"""
    try:
        h, m, s = ts_str.split(":")
        return float(h) * 3600 + float(m) * 60 + float(s)
    except Exception:
        return 0.0

def format_time_diff(seconds):
    """格式化時間差"""
    if seconds < 60:
        return f"{seconds:.2f} 秒"
    minutes = int(seconds // 60)
    secs = seconds % 60
    return f"{minutes} 分 {secs:.2f} 秒"

def find_latest_log():
    """尋找 logs 資料夾中最新產生的 gui_serial_continuous_*.log"""
    pattern = os.path.join(LOG_DIR, "gui_serial_continuous_*.log")
    files = glob.glob(pattern)
    if not files:
        return None
    # 依檔名或修改時間排序，取最後一個
    return max(files, key=os.path.basename)

def scan_for_last_reset(file_path):
    """掃描檔案，找出最後一次系統 reset 的行號與時間點（GPS ok 降為 0）"""
    last_reset_line = 1
    last_reset_ts = None
    prev_ok = None
    line_num = 0
    
    with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line_num += 1
            if "[GPS]" in line:
                ok_match = re.search(r"ok:(\d+)", line)
                ts_match = re.match(r"^\[([\d\:\.]+)\]", line)
                if ok_match and ts_match:
                    ok_val = int(ok_match.group(1))
                    ts_str = ts_match.group(1)
                    
                    # 偵測 ok 計數器驟降，代表 MCU 發生重啟
                    if prev_ok is not None and ok_val < prev_ok - 100:
                        last_reset_line = line_num
                        last_reset_ts = ts_str
                    prev_ok = ok_val
                    
    # 如果沒偵測到明顯的 ok 驟降，但有第一行的 timestamp，就從頭開始
    if last_reset_ts is None and line_num > 0:
        # 重新讀取第一行取得起始時間
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                ts_match = re.match(r"^\[([\d\:\.]+)\]", line)
                if ts_match:
                    last_reset_ts = ts_match.group(1)
                    break
                    
    return last_reset_line, last_reset_ts

def main():
    print("=" * 60)
    print(" 📡 GPS 首次定位時間 (TTFF) 與靜態漂移即時監控工具")
    print("=" * 60)
    
    log_file = find_latest_log()
    if not log_file:
        print(f"[錯誤] 在 {LOG_DIR} 中找不到任何 gui_serial_continuous_*.log 檔案！")
        print("請確保您已在地面站 GUI 建立連線並勾選儲存記錄。")
        sys.exit(1)
        
    print(f"👉 偵測到最新 Log 檔案: {os.path.basename(log_file)}")
    
    # 掃描最後一次 Reset
    reset_line, reset_ts = scan_for_last_reset(log_file)
    if reset_ts:
        print(f"🔄 偵測到最後一次系統 Reset 事件:")
        print(f"   - 時間點: {reset_ts}")
        print(f"   - Log 行號: {reset_line}")
    else:
        print(f"ℹ️ 未偵測到明確的重設事件，將從 Log 起點開始監控。")
        reset_line = 1
        reset_ts = "00:00:00.000"
        
    reset_sec = parse_time_str(reset_ts)
    
    print("\n⏳ 開始即時追蹤資料流 (Tail)... 按 Ctrl+C 結束監控\n")
    sys.stdout.flush()
    
    first_fix_ts = None
    gps_records = []
    ttff_reported = False
    
    lat_to_m = 111139.0
    
    # 開檔並跳到最後一次 reset 的位置
    with open(log_file, "r", encoding="utf-8", errors="ignore") as f:
        curr_line = 0
        for _ in range(reset_line - 1):
            f.readline()
            curr_line += 1
            
        # 開始持續讀取新行
        last_update_time = time.time()
        while True:
            # 檢查檔案是否被截斷或清空
            try:
                curr_size = os.path.getsize(log_file)
            except OSError:
                curr_size = 0
                
            line = f.readline()
            if not line:
                # 沒讀到新行，稍微等待
                time.sleep(0.1)
                
                # 每 15 秒如果沒有任何 GPS 輸出，印個提示防呆
                if time.time() - last_update_time > 15:
                    print("... 等待地面站寫入新的 GPS 資料中 ...")
                    sys.stdout.flush()
                    last_update_time = time.time()
                continue
                
            curr_line += 1
            last_update_time = time.time()
            
            if "[GPS]" in line:
                ts_match = re.match(r"^\[([\d\:\.]+)\]", line)
                fix_match = re.search(r"fix:(\d+)", line)
                
                if ts_match and fix_match:
                    ts_str = ts_match.group(1)
                    fix_val = int(fix_match.group(1))
                    
                    # 定位前：每 10 句印出一次衛星增長進度
                    sat_match = re.search(r"sat:(\d+)", line)
                    if sat_match and fix_val == 0:
                        sats = int(sat_match.group(1))
                        ok_match = re.search(r"ok:(\d+)", line)
                        ok_val = int(ok_match.group(1)) if ok_match else 0
                        if ok_val > 0 and ok_val % 10 == 0:
                            print(f"[{ts_str}] ❌ 未定位 | 衛星數量: {sats} 顆 | 累計接收語句: {ok_val}")
                            sys.stdout.flush()
                            
                    # 定位成功
                    if fix_val > 0:
                        if first_fix_ts is None:
                            first_fix_ts = ts_str
                            
                        # 擷取經緯高與衛星數
                        coord_match = re.search(r"([\+\-]\d+\.\d+),([\+\-]\d+\.\d+)", line)
                        alt_match = re.search(r"alt:(\-?\d+)m", line)
                        sat_match = re.search(r"sat:(\d+)", line)
                        
                        if coord_match and alt_match and sat_match:
                            lat = float(coord_match.group(1))
                            lon = float(coord_match.group(2))
                            alt = float(alt_match.group(1))
                            sats = int(sat_match.group(1))
                            
                            # 濾除 0.0, 0.0 的無效點
                            if abs(lat) > 0.01 and abs(lon) > 0.01:
                                gps_records.append({
                                    "ts": ts_str,
                                    "lat": lat,
                                    "lon": lon,
                                    "alt": alt,
                                    "sats": sats
                                })
                                
                        # 回報 TTFF (只印一次)
                        if not ttff_reported and first_fix_ts:
                            fix_sec = parse_time_str(first_fix_ts)
                            # 處理跨午夜 rollover
                            if fix_sec < reset_sec:
                                fix_sec += 24 * 3600
                            ttff = fix_sec - reset_sec
                            print("\n" + "="*50)
                            print(f"✨  [GPS 成功定位!] TTFF 耗時: {format_time_diff(ttff)}")
                            print(f"    - 重啟時間: {reset_ts}")
                            print(f"    - 首定位時間: {first_fix_ts}")
                            print("="*50 + "\n")
                            sys.stdout.flush()
                            ttff_reported = True
                            
                        # 每累積 20 個定位點，輸出一次漂移統計
                        if len(gps_records) > 0 and len(gps_records) % 20 == 0:
                            lats = np.array([r["lat"] for r in gps_records])
                            lons = np.array([r["lon"] for r in gps_records])
                            alts = np.array([r["alt"] for r in gps_records])
                            sats_arr = np.array([r["sats"] for r in gps_records])
                            
                            mean_lat = np.mean(lats)
                            mean_lon = np.mean(lons)
                            lon_to_m = 111139.0 * np.cos(np.radians(mean_lat))
                            
                            # 計算各點相對於均值的公尺偏差
                            y_m = (lats - mean_lat) * lat_to_m
                            x_m = (lons - mean_lon) * lon_to_m
                            dist_m = np.sqrt(x_m**2 + y_m**2)
                            
                            cep_50 = np.percentile(dist_m, 50)
                            max_dist = np.max(dist_m)
                            
                            print(f"📊 [漂移統計 — 已累積 {len(gps_records)} 點]")
                            print(f"   當前位置: {lat:.6f}, {lon:.6f} | 衛星數: {sats}")
                            print(f"   CEP (50% 誤差半徑): {cep_50:.2f} 公尺")
                            print(f"   最大漂移量 (Max Drift): {max_dist:.2f} 公尺")
                            print(f"   高度平均 / 標準差: {np.mean(alts):.1f}m / {np.std(alts):.2f}m (範圍: {np.min(alts):.1f}m ~ {np.max(alts):.1f}m)")
                            print(f"   平均衛星數量: {np.mean(sats_arr):.1f} 顆")
                            print("-" * 50)
                            sys.stdout.flush()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 已終止監控。")
        sys.exit(0)
