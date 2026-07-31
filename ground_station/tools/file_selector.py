#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
file_selector.py — RocketCom 互動式檔案選擇器
當命令列未指定輸入檔案時，自動提供 GUI (Tkinter) 彈窗或 CLI 最近檔案選單。
"""
import os
import sys
from datetime import datetime

def select_input_file(title="請選擇要分析/解碼的檔案", extensions=None, file_types=None):
    """
    開啟 GUI 檔案選擇器（GUI 可用時），或於 CLI 掃描最近檔案提供選單。
    """
    if extensions is None:
        extensions = [".csv", ".bin", ".hex", ".log"]
    if file_types is None:
        file_types = [
            ("Supported RocketCom Files", "*.csv;*.bin;*.hex;*.log"),
            ("CSV files", "*.csv"),
            ("Binary files", "*.bin"),
            ("Hex files", "*.hex"),
            ("Log files", "*.log"),
            ("All files", "*.*")
        ]

    # 1. 嘗試開啟 GUI 檔案選擇彈窗 (Tkinter)
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        selected = filedialog.askopenfilename(title=title, filetypes=file_types)
        root.destroy()
        if selected and os.path.exists(selected):
            print(f"[FILE_SELECTOR] 已選擇檔案: {selected}")
            return selected
    except Exception:
        pass

    # 2. GUI 無法開啟或使用者取消/無桌面時，掃描 ~/Downloads, ground_station/logs 及當前目錄
    print(f"\n🔍 未指定輸入檔案，正為您尋找最近的可用檔案...")
    candidates = []
    search_dirs = [
        os.path.expanduser("~/Downloads"),
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "logs")),
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "reports")),
        os.getcwd()
    ]

    seen = set()
    for d in search_dirs:
        if not os.path.exists(d):
            continue
        for root_path, _, files in os.walk(d):
            rel_depth = os.path.relpath(root_path, d).count(os.sep)
            if rel_depth > 3:
                continue
            for f in files:
                ext = os.path.splitext(f)[1].lower()
                if ext in extensions:
                    full_path = os.path.abspath(os.path.join(root_path, f))
                    if full_path not in seen:
                        seen.add(full_path)
                        try:
                            mtime = os.path.getmtime(full_path)
                            size = os.path.getsize(full_path)
                            candidates.append((mtime, size, full_path))
                        except OSError:
                            pass

    candidates.sort(key=lambda x: x[0], reverse=True)
    candidates = candidates[:15]

    if not candidates:
        print("❌ 未找到任何符合格式的數據或紀錄檔案。")
        return None

    print("\n" + "=" * 70)
    print(f" 📂 找到的最近數據/記錄檔案列表 ({title})：")
    print("=" * 70)
    for idx, (mtime, size, fpath) in enumerate(candidates, 1):
        mtime_str = datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M:%S")
        if size > 1024 * 1024:
            size_str = f"{size / (1024 * 1024):6.1f} MB"
        elif size > 1024:
            size_str = f"{size / 1024:6.1f} KB"
        else:
            size_str = f"{size:6d} B"
        tag = " ★ [最新]" if idx == 1 else ""
        rel_name = os.path.basename(fpath)
        parent_name = os.path.basename(os.path.dirname(fpath))
        display_name = f"{parent_name}/{rel_name}"
        if len(display_name) > 36:
            display_name = "..." + display_name[-33:]
        print(f"  [{idx:2d}] {display_name:<36} | {size_str} | {mtime_str}{tag}")
    print("-" * 70)

    try:
        ans = input(f"請選擇檔案編號 [1-{len(candidates)}] (按 Enter 預設選 [1]，或直接輸入/貼上檔案路徑): ").strip()
        if not ans:
            chosen = candidates[0][2]
        elif ans.isdigit():
            idx = int(ans) - 1
            if 0 <= idx < len(candidates):
                chosen = candidates[idx][2]
            else:
                chosen = candidates[0][2]
        elif os.path.exists(ans):
            chosen = os.path.abspath(ans)
        else:
            chosen = candidates[0][2]
        print(f"🟢 已選擇檔案: {chosen}\n")
        return chosen
    except (KeyboardInterrupt, EOFError):
        print("\n[System] 使用者取消。")
        sys.exit(0)

if __name__ == "__main__":
    sel = select_input_file()
    print("Selected:", sel)
