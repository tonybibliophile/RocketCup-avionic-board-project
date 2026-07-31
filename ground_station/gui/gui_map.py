#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gui_map — 獨立可重用的 GPS 即時地圖元件（tkintermapview 為主，離線退回 matplotlib）。

從 gui_monitor.py 的 `build_map_tab`/`update_map`（:1162-1377）重新包裝成一個自持狀態
的類別 `MapView`，供 `ground_gui.py` 使用。★不動 gui_monitor.py 本身既有實作，理由同
`gui_attitude3d.py` 開頭註解——那支檔案已有大量未 commit 的變更，風險留在最小範圍。

跟原本內嵌版本的差異（刻意，不是疏漏）：
  - 拆成 `on_rocket_fix`/`on_gs_fix`/`redraw` 三個方法，不是單一 `update(...)`：
    火箭座標來自 [GS_PKT]（~5-10Hz），地面站座標來自 [GS_GPS]（1Hz），資料到達頻率不同，
    硬塞一個 signature 會逼其中一路一直傳舊值。`redraw()` 應由外部 1Hz timer 呼叫
    （tile 地圖不能每包都重畫，比照 gui_monitor.py 的 `map_dirty` 節流手法）。
  - 預設不建 GPS 尋星狀態側欄（`show_status_panel=False）——那個側欄餵的是直連航電板
    才有的 [GPS] 診斷行，`ground_gui.py` 收不到；它自己的 `gps_frame` 已經有 sats/fix
    數字，不重複做。
  - 座標轉換直接 import `gui_theme.latlon_to_en`，跟 `ground_gui.py` 既有文字讀數共用
    同一份換算，不會兩邊算出不同數字。

用法：
    mv = MapView()
    widget = mv.build(parent_frame)
    widget.pack(fill=tk.BOTH, expand=True)
    ...
    mv.on_rocket_fix(lat, lon, alt_m=123, sats=8)   # 收到 [GS_PKT] pos: 時呼叫
    mv.on_gs_fix(lat, lon)                          # 收到 [GS_GPS] 時呼叫
    ...
    # 外部 1Hz timer：
    if mv.is_dirty():
        mv.redraw()
    ...
    mv.destroy()   # 視窗關閉時呼叫
"""

import os
import sys

import numpy as np
import tkinter as tk
from tkinter import ttk

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gui_theme as gt   # noqa: E402  # 面板配色 + latlon_to_en/distance_bearing

# 選配：GPS 地圖元件（OpenStreetMap 圖磚，線上載入）。安裝失敗不擋主程式，
# 自動退回「相對軌跡圖」（原點=第一筆定位，離線可用）——照抄 gui_monitor.py:48-61 的手法。
try:
    import tkintermapview
    HAVE_MAPVIEW = True
except ImportError:
    try:
        import subprocess
        import sys as _sys
        subprocess.check_call([_sys.executable, "-m", "pip", "install", "tkintermapview"])
        import tkintermapview
        HAVE_MAPVIEW = True
    except Exception:
        tkintermapview = None
        HAVE_MAPVIEW = False
        print("[*] tkintermapview 未安裝（無網路？），GPS 地圖改用相對軌跡圖。")


class MapView:
    """自持狀態的 GPS 即時地圖元件。build() 一次，之後用 on_rocket_fix/on_gs_fix 餵資料，
    由外部 1Hz timer 呼叫 redraw()。"""

    def __init__(self, max_track_points=5000, show_status_panel=False):
        self._max_track_points = max_track_points
        self._show_status_panel = show_status_panel

        self._container = None
        self._lbl_info = None
        self._follow_var = None

        # tkintermapview 路徑
        self._map_widget = None
        self._map_marker = None
        self._map_path = None
        self._gs_marker = None
        self._map_zoomed = False

        # matplotlib 離線 fallback 路徑
        self._trk_fig = None
        self._trk_ax = None
        self._trk_canvas = None
        self._trk_line = None
        self._trk_pt = None
        self._trk_home = None

        # 資料狀態
        self._home = None          # (lat, lon)：第一筆火箭定位
        self._track = []           # [(lat, lon), ...]
        self._rocket_last = None   # {"lat","lon","alt","sats"}
        self._gs_pos = None        # (lat, lon)
        self._dirty = False

    # ---------------------------------------------------------- 建構
    def build(self, parent):
        self._container = tk.Frame(parent, bg=gt.BG_PANEL)

        ctrl = tk.Frame(self._container, bg=gt.BG_PANEL)
        ctrl.pack(fill=tk.X, padx=8, pady=(6, 2))
        self._follow_var = tk.BooleanVar(value=True)
        tk.Checkbutton(ctrl, text="跟隨最新位置", variable=self._follow_var,
                       bg=gt.BG_PANEL, fg=gt.CYAN, selectcolor=gt.BG_ROOT,
                       font=("Helvetica", 9)).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="清除軌跡", width=8, command=self.clear_track).pack(side=tk.LEFT, padx=6)
        mode = "OpenStreetMap 線上地圖" if HAVE_MAPVIEW else "相對軌跡圖（tkintermapview 未安裝/離線）"
        tk.Label(ctrl, text=mode, bg=gt.BG_PANEL, fg=gt.TXT_MUTED,
                 font=("Helvetica", 9)).pack(side=tk.RIGHT, padx=6)

        self._lbl_info = tk.Label(self._container, text="等待 GPS 定位…",
                                  bg=gt.BG_GPS_PANEL, fg=gt.YELLOW, font=("Monaco", 10),
                                  anchor="w", justify=tk.LEFT, padx=10, pady=6)
        self._lbl_info.pack(fill=tk.X, padx=8, pady=(0, 4))

        body = tk.Frame(self._container, bg=gt.BG_PANEL)
        body.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        map_container = tk.Frame(body, bg=gt.BG_PANEL)
        map_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        if self._show_status_panel:
            # 目前僅 gui_monitor.py 直連航電板情境用得到（需要 [GPS] 診斷行），
            # ground_gui.py 預設不開；保留參數是為了將來若有對應資料來源可以直接打開。
            status_panel = tk.Frame(body, bg=gt.BG_GPS_PANEL, width=180,
                                    highlightbackground="#2a2a30", highlightthickness=1)
            status_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(6, 0))
            status_panel.pack_propagate(False)
            tk.Label(status_panel, text="📡 GPS 尋星狀態", bg=gt.BG_GPS_PANEL, fg=gt.CYAN,
                     font=("Helvetica", 9, "bold"), pady=6).pack(fill=tk.X)

        if HAVE_MAPVIEW:
            self._map_widget = tkintermapview.TkinterMapView(map_container, corner_radius=0)
            self._map_widget.pack(fill=tk.BOTH, expand=True)
            self._map_widget.set_position(23.97, 120.97)   # 預設台灣中心，待首筆定位跳轉
            self._map_widget.set_zoom(8)
        else:
            self._trk_fig = Figure(facecolor=gt.BG_ROOT)
            self._trk_ax = self._trk_fig.add_subplot(111)
            self._trk_ax.set_facecolor(gt.BG_PANEL)
            self._trk_ax.tick_params(colors=gt.TXT_MUTED, labelsize=8)
            for sp in self._trk_ax.spines.values():
                sp.set_color(gt.SEP)
            self._trk_ax.grid(color="#2a2a2a", linewidth=0.5, alpha=0.6)
            self._trk_ax.set_xlabel("East (m)", color=gt.TXT_DIM, fontsize=9)
            self._trk_ax.set_ylabel("North (m)", color=gt.TXT_DIM, fontsize=9)
            self._trk_ax.set_aspect("equal", adjustable="datalim")
            self._trk_line, = self._trk_ax.plot([], [], color=gt.CYAN_HI, lw=1.2)
            self._trk_pt, = self._trk_ax.plot([], [], color="#ff1744", marker="o", ms=8, lw=0)
            self._trk_home, = self._trk_ax.plot([], [], color=gt.GREEN, marker="^", ms=9, lw=0)
            self._trk_canvas = FigureCanvasTkAgg(self._trk_fig, master=map_container)
            self._trk_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        return self._container

    # ---------------------------------------------------------- 資料輸入
    def on_rocket_fix(self, lat, lon, alt_m=None, sats=None):
        """收到一筆有效火箭定位（來自 [GS_PKT] pos:）。"""
        if lat == 0.0 and lon == 0.0:
            return   # 未定位時的哨兵座標，不當成真實定位
        if self._home is None:
            self._home = (lat, lon)
        if not self._track or self._track[-1] != (lat, lon):
            self._track.append((lat, lon))
            if len(self._track) > self._max_track_points:
                self._track = self._track[-(self._max_track_points - 1000):]
        g = self._rocket_last or {}
        g.update({"lat": lat, "lon": lon})
        if alt_m is not None:
            g["alt"] = alt_m
        if sats is not None:
            g["sats"] = sats
        self._rocket_last = g
        self._dirty = True

    def on_gs_fix(self, lat, lon):
        """收到地面站自身定位（來自 [GS_GPS]）。"""
        if lat == 0.0 and lon == 0.0:
            return
        self._gs_pos = (lat, lon)
        self._dirty = True

    def clear_track(self):
        self._track = []
        self._home = None
        if HAVE_MAPVIEW and self._map_path is not None:
            try:
                self._map_path.delete()
            except Exception:
                pass
            self._map_path = None
        self._dirty = True

    def is_dirty(self):
        return self._dirty

    def is_alive(self):
        if self._container is None:
            return False
        try:
            return bool(self._container.winfo_exists())
        except tk.TclError:
            return False

    def destroy(self):
        self._container = None
        self._lbl_info = None
        self._map_widget = None
        self._map_marker = None
        self._map_path = None
        self._gs_marker = None
        self._trk_fig = None
        self._trk_ax = None
        self._trk_canvas = None
        self._trk_line = None
        self._trk_pt = None
        self._trk_home = None

    # ---------------------------------------------------------- 繪製（由外部 1Hz timer 驅動）
    def redraw(self):
        if not self.is_alive():
            return
        self._dirty = False

        if self._rocket_last:
            g = self._rocket_last
            info = (f"🚀 lat={g['lat']:+.6f}  lon={g['lon']:+.6f}  alt={g.get('alt', 0)} m  "
                    f"sats={g.get('sats', '?')}")
            if self._home:
                e, n = gt.latlon_to_en(g["lat"], g["lon"], self._home[0], self._home[1])
                info += f"  距Home {float(np.hypot(e, n)):.0f} m"
            self._lbl_info.config(text=info, fg=gt.GREEN)
        elif self._gs_pos:
            self._lbl_info.config(
                text=f"📡 地面站 lat={self._gs_pos[0]:+.6f} lon={self._gs_pos[1]:+.6f}（等待火箭 GPS…）",
                fg=gt.YELLOW)

        if HAVE_MAPVIEW:
            self._redraw_mapview()
        else:
            self._redraw_fallback()

    def _redraw_mapview(self):
        if self._track:
            lat, lon = self._track[-1]
            if self._map_marker is None:
                self._map_marker = self._map_widget.set_marker(lat, lon, text="🚀")
            else:
                self._map_marker.set_position(lat, lon)
            if len(self._track) >= 2:
                pts = self._track[-600:]
                try:
                    if self._map_path is None:
                        self._map_path = self._map_widget.set_path(pts)
                    else:
                        self._map_path.set_position_list(pts)
                except Exception:   # 版本差異：退回重建路徑
                    try:
                        if self._map_path:
                            self._map_path.delete()
                    except Exception:
                        pass
                    self._map_path = self._map_widget.set_path(pts)
            if not self._map_zoomed:
                self._map_widget.set_zoom(16)
                self._map_zoomed = True
            if self._follow_var.get():
                self._map_widget.set_position(lat, lon)
        if self._gs_pos:
            if self._gs_marker is None:
                self._gs_marker = self._map_widget.set_marker(
                    self._gs_pos[0], self._gs_pos[1], text="📡GS")
            else:
                self._gs_marker.set_position(self._gs_pos[0], self._gs_pos[1])
            if not self._track and not self._map_zoomed:
                self._map_widget.set_position(self._gs_pos[0], self._gs_pos[1])
                self._map_widget.set_zoom(15)
                self._map_zoomed = True

    def _redraw_fallback(self):
        if self._track and self._home:
            lat0, lon0 = self._home
            arr = np.array(self._track, dtype=float)
            e, n = gt.latlon_to_en(arr[:, 0], arr[:, 1], lat0, lon0)
            self._trk_line.set_data(e, n)
            self._trk_pt.set_data([e[-1]], [n[-1]])
            self._trk_home.set_data([0.0], [0.0])
            self._trk_ax.relim()
            self._trk_ax.autoscale_view()
            self._trk_canvas.draw_idle()


# ==================== demo：獨立跑起來驗證，不需要 ground_gui.py ====================
if __name__ == "__main__":
    root = tk.Tk()
    root.title("MapView demo")
    root.geometry("720x560")
    root.configure(bg=gt.BG_PANEL)

    mv = MapView()
    widget = mv.build(root)
    widget.pack(fill=tk.BOTH, expand=True)

    home_lat, home_lon = 24.78, 120.99
    state = {"i": 0}

    def tick():
        state["i"] += 1
        i = state["i"]
        lat = home_lat + i * 0.00003
        lon = home_lon + i * 0.00002
        mv.on_rocket_fix(lat, lon, alt_m=100 + i, sats=8)
        mv.on_gs_fix(home_lat, home_lon)
        if mv.is_dirty():
            mv.redraw()
        root.after(1000, tick)

    root.after(500, tick)
    root.mainloop()
