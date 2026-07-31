#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gui_attitude3d — 獨立可重用的 3D 火箭姿態渲染元件（matplotlib + Tkinter）。

本模組是從 gui_monitor.py 的內嵌 3D 姿態實作（`update_3d_plot` 等）搬出來、包成一個
自持狀態的類別 `Attitude3DView`，供 `ground_gui.py` 使用。★刻意不去動 gui_monitor.py
本身的既有實作——那支檔案已有大量未 commit 的變更，風險留在最小範圍；未來若想讓
gui_monitor.py 也改用這個共用模組，是後續獨立的重構工作。

幾何生成/旋轉數學（`generate_rocket_geometry`/`rotate_points`/`quaternion_to_matrix`）
逐字取自 gui_monitor.py:101-189。歐拉角換算改成 import gui_theme 的
`quaternion_to_euler`/`tilt_angle_deg`（那邊已經是逐字抽出的同一份，不重複維護第三份）。

用法：
    view = Attitude3DView(get_fsm_state=lambda: "STATE_PAD")
    widget = view.build(parent_frame)
    widget.pack(fill=tk.BOTH, expand=True)
    ...
    view.update(q)          # q = [qw, qx, qy, qz]；由外部 dirty-flag 節流呼叫，不要每包都呼叫
    ...
    view.destroy()          # 視窗關閉時呼叫，釋放 matplotlib Figure

關鍵設計：用 `matplotlib.figure.Figure`（不是 `plt.figure()`）建圖——後者會登記進
pyplot 全域 Gcf manager 永久持有參照，視窗開關幾次就會漏 Figure、跳
`RuntimeWarning: More than 20 figures have been opened`。`Figure(...)` + `FigureCanvasTkAgg`
不會全域註冊，物件消失就會被回收。
"""

import os
import sys
import random

import numpy as np
import tkinter as tk
from tkinter import ttk

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  # projection='3d' 註冊副作用，需要 import
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gui_theme as gt   # noqa: E402  # 面板配色 + 已抽好的 quaternion_to_euler/tilt_angle_deg


# ==================== 3D 幾何生成工具（逐字取自 gui_monitor.py:101-189） ====================
def generate_rocket_geometry(r=0.25, h_body=1.8, h_nose=0.8):
    """
    在局部座標系（火箭長軸為 Z 軸）中生成圓柱體身（三分段）、圓錐體頭、黑色噴嘴、尾焰與 3D 穩定翼的頂點
    """
    theta = np.linspace(0, 2 * np.pi, 24)

    # 1. 圓柱體身分段 (Cylinder segments)
    z_b1 = np.linspace(-h_body / 2, -h_body / 6, 6)
    theta_grid, z_grid_b1 = np.meshgrid(theta, z_b1)
    x_grid_b1 = r * np.cos(theta_grid)
    y_grid_b1 = r * np.sin(theta_grid)

    z_b2 = np.linspace(-h_body / 6, h_body / 6, 6)
    theta_grid, z_grid_b2 = np.meshgrid(theta, z_b2)
    x_grid_b2 = r * np.cos(theta_grid)
    y_grid_b2 = r * np.sin(theta_grid)

    z_b3 = np.linspace(h_body / 6, h_body / 2, 6)
    theta_grid, z_grid_b3 = np.meshgrid(theta, z_b3)
    x_grid_b3 = r * np.cos(theta_grid)
    y_grid_b3 = r * np.sin(theta_grid)

    # 2. 圓錐體頭 (Nose Cone)
    z_nose = np.linspace(h_body / 2, h_body / 2 + h_nose, 8)
    theta_grid_n, z_grid_n = np.meshgrid(theta, z_nose)
    r_taper = r * ((h_body / 2 + h_nose) - z_grid_n) / h_nose
    x_grid_n = r_taper * np.cos(theta_grid_n)
    y_grid_n = r_taper * np.sin(theta_grid_n)

    # 3. 噴嘴 (Nozzle Cone)
    z_nozzle = np.linspace(-h_body / 2 - 0.2, -h_body / 2, 5)
    theta_grid_noz, z_grid_noz = np.meshgrid(theta, z_nozzle)
    r_noz_taper = r * 0.75 + (r * 0.25) * (z_grid_noz - (-h_body / 2 - 0.2)) / 0.2
    x_grid_noz = r_noz_taper * np.cos(theta_grid_noz)
    y_grid_noz = r_noz_taper * np.sin(theta_grid_noz)

    # 4. 引擎火光 (Thrust Plume Base)
    z_flame = np.linspace(-0.9, 0.0, 8)
    theta_grid_fl, z_grid_fl = np.meshgrid(theta, z_flame)
    r_fl_taper = r * 0.65 * (z_grid_fl - (-0.9)) / 0.9
    x_grid_fl = r_fl_taper * np.cos(theta_grid_fl)
    y_grid_fl = r_fl_taper * np.sin(theta_grid_fl)

    # 5. 四個穩定翼 (Fins - 3D 梯形面)
    fins = [
        np.array([[r, 0, -h_body / 2], [r + 0.4, 0, -h_body / 2], [r + 0.3, 0, -h_body / 2 + 0.45], [r, 0, -h_body / 2 + 0.45]]),
        np.array([[-r, 0, -h_body / 2], [-r - 0.4, 0, -h_body / 2], [-r - 0.3, 0, -h_body / 2 + 0.45], [-r, 0, -h_body / 2 + 0.45]]),
        np.array([[0, r, -h_body / 2], [0, r + 0.4, -h_body / 2], [0, r + 0.3, -h_body / 2 + 0.45], [0, r, -h_body / 2 + 0.45]]),
        np.array([[0, -r, -h_body / 2], [0, -r - 0.4, -h_body / 2], [0, -r - 0.3, -h_body / 2 + 0.45], [0, -r, -h_body / 2 + 0.45]]),
    ]

    return (x_grid_b1, y_grid_b1, z_grid_b1,
            x_grid_b2, y_grid_b2, z_grid_b2,
            x_grid_b3, y_grid_b3, z_grid_b3,
            x_grid_n, y_grid_n, z_grid_n,
            x_grid_noz, y_grid_noz, z_grid_noz,
            x_grid_fl, y_grid_fl, z_grid_fl,
            fins)


def rotate_points(x, y, z, R):
    """將網格點套用 3D 旋轉矩陣 R"""
    orig_shape = x.shape
    pts = np.vstack((x.flatten(), y.flatten(), z.flatten()))
    rot_pts = R @ pts
    rx = rot_pts[0].reshape(orig_shape)
    ry = rot_pts[1].reshape(orig_shape)
    rz = rot_pts[2].reshape(orig_shape)
    return rx, ry, rz


def quaternion_to_matrix(q):
    """將四元數 [qw, qx, qy, qz] 轉換為旋轉矩陣 R (航空慣用基準)"""
    w, x, y, z = q
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-6:
        return np.eye(3)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    R = np.array([
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
    ])
    return R


# 3D 模型材質色（模型本身的材質，非 UI 面板配色，故留在此模組而非 gui_theme.py）
_COLOR_SPACE_BG = "#101010"
_COLOR_BODY = "#722f37"      # 彈體酒紅
_COLOR_NOSE = "#f5f5f0"      # 鼻錐白
_COLOR_NOZZLE = "#212121"    # 噴嘴碳黑
_COLOR_FIN_FACE = "#d4af37"  # 尾翼金
_COLOR_FIN_EDGE = "#8a6f1f"
_COLOR_FLAME_OUTER = "#ff5722"
_COLOR_FLAME_INNER = "#ffeb3b"
_COLOR_PAD_NEON = "#00ffcc"
_COLOR_ZENITH = "#00e676"
_COLOR_HEADING = "#ff1744"
_COLOR_PROJ = "#00e5ff"


def _generate_static_ground_geometry():
    """發射台地平面/同心圈/方位射線/水平全息環——這些點位跟姿態四元數無關，每幀都不變。
    之前放在 _draw() 內每幀重算 np.linspace/meshgrid/cos/sin，5Hz 白白重算同一批數字；
    抽成模組層級常數只算一次，_draw() 直接吃現成陣列。"""
    z_ground = -1.2
    r_vals = np.linspace(0, 2.0, 15)
    theta_vals = np.linspace(0, 2 * np.pi, 48)
    R_grid, T_grid = np.meshgrid(r_vals, theta_vals)
    xg = R_grid * np.cos(T_grid)
    yg = R_grid * np.sin(T_grid)
    zg = np.full_like(xg, z_ground)

    cx = 2.0 * np.cos(theta_vals)
    cy = 2.0 * np.sin(theta_vals)
    cz = np.full_like(cx, z_ground)

    rings = []
    for r_c in (0.7, 1.4):
        cx_c = r_c * np.cos(theta_vals)
        cy_c = r_c * np.sin(theta_vals)
        cz_c = np.full_like(cx_c, z_ground)
        rings.append((cx_c, cy_c, cz_c))

    compass_lines = []
    for angle in (0, 45, 90, 135, 180, 225, 270, 315):
        rad = np.radians(angle)
        compass_lines.append(([0, 2.0 * np.cos(rad)], [0, 2.0 * np.sin(rad)], [z_ground, z_ground]))

    th_h = np.linspace(0, 2 * np.pi, 60)
    hx = 0.65 * np.cos(th_h)
    hy = 0.65 * np.sin(th_h)
    hz = np.zeros_like(th_h)
    horizon_surf = (np.array([np.zeros_like(th_h), hx]),
                     np.array([np.zeros_like(th_h), hy]),
                     np.array([np.zeros_like(th_h), hz]))

    return {
        "z_ground": z_ground, "xg": xg, "yg": yg, "zg": zg,
        "cx": cx, "cy": cy, "cz": cz, "rings": rings,
        "compass_lines": compass_lines,
        "hx": hx, "hy": hy, "hz": hz, "horizon_surf": horizon_surf,
    }


_STATIC_GROUND = _generate_static_ground_geometry()


class Attitude3DView:
    """自持狀態的 3D 火箭姿態顯示元件。build() 一次，之後只呼叫 update()。"""

    def __init__(self, get_fsm_state=None, title=" > 3D ROCKET ATTITUDE (EKF ESTIMATE)"):
        """get_fsm_state：無參數 callable，回傳 HUD 要顯示的狀態字串。傳 None 則顯示 '--'。"""
        self._get_fsm_state = get_fsm_state
        self._title = title
        self._geom = generate_rocket_geometry()
        self._fig = None
        self._ax = None
        self._canvas = None
        self._btn_pause = None
        self._paused = False
        self._has_data = False   # 是否收過真的 [GS_ATT]（區分「真的直立」vs「舊韌體沒回報」）

    # ---------------------------------------------------------- 建構
    def build(self, parent):
        """在 parent 裡建立標題列(暫停/重設視角) + 3D 畫布，回傳可 pack/grid 的容器 widget。"""
        container = tk.Frame(parent, bg=gt.BG_PANEL)

        title_bar = tk.Frame(container, bg=gt.BG_PANEL)
        title_bar.pack(fill=tk.X, side=tk.TOP, padx=10, pady=5)
        tk.Label(title_bar, text=self._title, bg=gt.BG_PANEL, fg=gt.CYAN,
                 font=("Monaco", 10, "bold")).pack(side=tk.LEFT)
        self._btn_pause = ttk.Button(title_bar, text="暫停渲染", width=8, command=self.toggle_paused)
        self._btn_pause.pack(side=tk.RIGHT, padx=5)
        ttk.Button(title_bar, text="重設視角", width=8, command=self.reset_view).pack(side=tk.RIGHT, padx=5)

        self._fig = Figure(facecolor=_COLOR_SPACE_BG)
        self._ax = self._fig.add_subplot(111, projection="3d")
        self._ax.set_facecolor(_COLOR_SPACE_BG)
        self._ax.view_init(elev=20, azim=45)

        self._canvas = FigureCanvasTkAgg(self._fig, master=container)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 開機先畫一次 identity quaternion（直立），標記為「尚無真資料」
        self._draw([1.0, 0.0, 0.0, 0.0], has_data=False)
        return container

    # ---------------------------------------------------------- 操作
    def toggle_paused(self):
        self.set_paused(not self._paused)

    def set_paused(self, paused):
        self._paused = bool(paused)
        if self._btn_pause is not None:
            self._btn_pause.config(text="繼續渲染" if self._paused else "暫停渲染")

    def is_paused(self):
        return self._paused

    def reset_view(self):
        if not self.is_alive():
            return
        self._ax.view_init(elev=20, azim=45)
        self._canvas.draw()

    def is_alive(self):
        """canvas widget 是否仍存在（視窗可能已被關閉銷毀）。"""
        if self._canvas is None:
            return False
        try:
            return bool(self._canvas.get_tk_widget().winfo_exists())
        except tk.TclError:
            return False

    def destroy(self):
        """視窗關閉時呼叫：釋放 Figure/canvas 參照，避免對已銷毀的 widget 繼續畫圖。"""
        self._fig = None
        self._ax = None
        self._canvas = None
        self._btn_pause = None

    def update(self, q):
        """q = [qw, qx, qy, qz]。暫停中或 widget 已銷毀時直接跳過。"""
        if self._paused or not self.is_alive():
            return
        self._draw(q, has_data=True)

    # ---------------------------------------------------------- 內部繪製
    def _draw(self, q, has_data):
        """函式體移植自 gui_monitor.py:2237-2408 的 update_3d_plot，self.ax/self.canvas/
        self.local_geom/self.fsm_state 改成本類別的 instance 屬性/建構子參數。"""
        self._has_data = self._has_data or has_data

        R = quaternion_to_matrix(q)
        zyx_roll, zyx_pitch, yaw_corr = gt.quaternion_to_euler(q)
        pitch_corr = zyx_roll
        roll_corr = zyx_pitch

        (xb1, yb1, zb1, xb2, yb2, zb2, xb3, yb3, zb3,
         xn, yn, zn, xnoz, ynoz, znoz, xfl, yfl, zfl, fins) = self._geom

        h_body = 1.8
        h_nose = 0.8
        tail_z, tip_z = -h_body / 2 - 0.2, h_body / 2 + h_nose

        rxb1, ryb1, rzb1 = rotate_points(xb1, yb1, zb1, R)
        rxb2, ryb2, rzb2 = rotate_points(xb2, yb2, zb2, R)
        rxb3, ryb3, rzb3 = rotate_points(xb3, yb3, zb3, R)
        rxn, ryn, rzn = rotate_points(xn, yn, zn, R)
        rxnoz, rynoz, rznoz = rotate_points(xnoz, ynoz, znoz, R)

        flicker_factor = random.uniform(0.75, 1.25)
        zfl_outer = zfl * flicker_factor + tail_z
        rxfl, ryfl, rzfl = rotate_points(xfl, yfl, zfl_outer, R)
        zfl_inner = zfl * 0.65 * flicker_factor + tail_z
        rxfl_inner, ryfl_inner, rzfl_inner = rotate_points(xfl * 0.55, yfl * 0.55, zfl_inner, R)

        elev, azim = self._ax.elev, self._ax.azim
        self._ax.clear()
        self._ax.set_facecolor(_COLOR_SPACE_BG)
        self._ax.view_init(elev=elev, azim=azim)

        # 尚無真實姿態資料（舊韌體沒回報 [GS_ATT]）：整體降低不透明度，避免誤讀成「真的直立」
        model_alpha = 0.95 if self._has_data else 0.35

        # 地平面/同心圈/方位射線/水平環都跟姿態無關，每幀不變，改吃模組層級快取，
        # 不再每幀重算 linspace/meshgrid/cos/sin（5Hz 下省下最大宗的重複 numpy 運算）。
        sg = _STATIC_GROUND
        z_ground = sg["z_ground"]
        self._ax.plot_surface(sg["xg"], sg["yg"], sg["zg"], color="#161616", alpha=0.6, edgecolor="#2c2c2c", linewidth=0.4, shade=False)
        self._ax.plot(sg["cx"], sg["cy"], sg["cz"], color=_COLOR_PAD_NEON, linestyle="-", linewidth=1.5, alpha=0.7)

        for cx_c, cy_c, cz_c in sg["rings"]:
            self._ax.plot(cx_c, cy_c, cz_c, color="#3e3e3e", linestyle="--", linewidth=0.8, alpha=0.5)

        for lx, ly, lz in sg["compass_lines"]:
            self._ax.plot(lx, ly, lz, color="#333333", linestyle=":", linewidth=0.6)

        self._ax.text(2.2, 0, z_ground, "E (90°)", color=_COLOR_PAD_NEON, fontsize=8, fontweight="bold", ha="left", va="center")
        self._ax.text(-2.2, 0, z_ground, "W (270°)", color=_COLOR_PAD_NEON, fontsize=8, fontweight="bold", ha="right", va="center")
        self._ax.text(0, 2.2, z_ground, "N (0°)", color=_COLOR_PAD_NEON, fontsize=8, fontweight="bold", ha="center", va="bottom")
        self._ax.text(0, -2.2, z_ground, "S (180°)", color=_COLOR_PAD_NEON, fontsize=8, fontweight="bold", ha="center", va="top")

        hx, hy, hz = sg["hx"], sg["hy"], sg["hz"]
        hsx, hsy, hsz = sg["horizon_surf"]
        self._ax.plot_surface(hsx, hsy, hsz, color=_COLOR_PROJ, alpha=0.12, shade=False)
        self._ax.plot(hx, hy, hz, color=_COLOR_PROJ, linestyle="-", linewidth=0.8, alpha=0.4)
        self._ax.plot([-0.65, 0.65], [0, 0], [0, 0], color=_COLOR_PROJ, linestyle=":", linewidth=0.5, alpha=0.4)
        self._ax.plot([0, 0], [-0.65, 0.65], [0, 0], color=_COLOR_PROJ, linestyle=":", linewidth=0.5, alpha=0.4)

        p_base = R @ np.array([0, 0, tail_z])
        p_tip = R @ np.array([0, 0, tip_z])

        self._ax.quiver(0, 0, 0, 0, 0, 1.2, color=_COLOR_ZENITH, linewidth=1.5, arrow_length_ratio=0.15, alpha=0.6)
        self._ax.text(0, 0, 1.35, "ZENITH", color=_COLOR_ZENITH, fontsize=7, fontweight="bold", ha="center")

        dir_z = R[:, 2]
        self._ax.quiver(p_tip[0], p_tip[1], p_tip[2], dir_z[0] * 0.5, dir_z[1] * 0.5, dir_z[2] * 0.5,
                        color=_COLOR_HEADING, linewidth=2.0, arrow_length_ratio=0.3, alpha=0.9)

        self._ax.plot([p_base[0], p_base[0]], [p_base[1], p_base[1]], [p_base[2], z_ground],
                      color=_COLOR_PROJ, linestyle="--", linewidth=1.0, alpha=0.6)
        self._ax.plot([p_tip[0], p_tip[0]], [p_tip[1], p_tip[1]], [p_tip[2], z_ground],
                      color=_COLOR_HEADING, linestyle="--", linewidth=1.0, alpha=0.6)
        self._ax.plot([0, 0], [0, 0], [0, z_ground], color="#ffcc00", linestyle="-.", linewidth=0.8, alpha=0.4)

        self._ax.quiver(p_base[0], p_base[1], z_ground,
                        p_tip[0] - p_base[0], p_tip[1] - p_base[1], 0,
                        color=_COLOR_PROJ, alpha=0.35, arrow_length_ratio=0.15, linewidth=2.5)

        self._ax.plot_surface(rxnoz, rynoz, rznoz, color=_COLOR_NOZZLE, alpha=model_alpha, edgecolor="none", shade=True)
        self._ax.plot_surface(rxb1, ryb1, rzb1, color=_COLOR_BODY, alpha=model_alpha, edgecolor="none", shade=True)
        self._ax.plot_surface(rxb2, ryb2, rzb2, color=_COLOR_BODY, alpha=model_alpha, edgecolor="none", shade=True)
        self._ax.plot_surface(rxb3, ryb3, rzb3, color=_COLOR_BODY, alpha=model_alpha, edgecolor="none", shade=True)
        self._ax.plot_surface(rxn, ryn, rzn, color=_COLOR_NOSE, alpha=min(0.98, model_alpha + 0.05), edgecolor="none", shade=True)

        self._ax.plot_surface(rxfl, ryfl, rzfl, color=_COLOR_FLAME_OUTER, alpha=0.4 * model_alpha, edgecolor="none", shade=True)
        self._ax.plot_surface(rxfl_inner, ryfl_inner, rzfl_inner, color=_COLOR_FLAME_INNER, alpha=0.7 * model_alpha, edgecolor="none", shade=True)

        for f in fins:
            rf = f @ R.T
            poly = Poly3DCollection([rf], facecolors=_COLOR_FIN_FACE, edgecolors=_COLOR_FIN_EDGE,
                                    linewidths=1.0, alpha=model_alpha, shade=True)
            self._ax.add_collection3d(poly)

        # 三軸跨距須維持接近相等（3.6/3.6/3.7）＋ box_aspect 同比例，否則傾角會被拉伸畫錯
        # （逐字沿用 gui_monitor.py 修過的版本：曾有 10° 傾角畫成 ~40° 的舊 bug）。
        self._ax.set_xlim([-1.8, 1.8])
        self._ax.set_ylim([-1.8, 1.8])
        self._ax.set_zlim([-1.5, 2.2])
        self._ax.set_box_aspect((3.6, 3.6, 3.7))
        self._ax.axis("off")

        tilt_cos = np.clip(dir_z[2], -1.0, 1.0)
        tilt_deg = np.degrees(np.arccos(tilt_cos))
        yaw_disp = yaw_corr % 360.0

        fsm_text = self._get_fsm_state() if self._get_fsm_state else "--"
        if not self._has_data:
            # 全部維持 ASCII：matplotlib 用 Monaco 畫這個 HUD 疊字，該字型無 CJK 字形，
            # 混進中文會缺字（tofu box），跟其餘用 Tkinter 原生渲染的中文 UI 不一樣。
            hud_text = (
                f"STATE: {fsm_text}\n"
                f"ATT:   NO DATA (firmware not sending [GS_ATT] yet)"
            )
            hud_color = gt.TXT_OFF
        else:
            hud_text = (
                f"STATE: {fsm_text}\n"
                f"TILT:  {tilt_deg:.1f}°\n"
                f"PITCH: {pitch_corr:+.1f}°\n"
                f"YAW:   {yaw_disp:.1f}°\n"
                f"ROLL:  {roll_corr:+.1f}°"
            )
            hud_color = _COLOR_PROJ
        self._ax.text2D(0.05, 0.95, hud_text, transform=self._ax.transAxes, color=hud_color,
                        fontsize=10, fontweight="bold", fontname="Monaco",
                        bbox=dict(facecolor="#0a0a0a", alpha=0.8, edgecolor=hud_color,
                                  boxstyle="round,pad=0.6", linewidth=1.2))

        self._canvas.draw_idle()


# ==================== demo：獨立跑起來驗證，不需要 ground_gui.py ====================
if __name__ == "__main__":
    import math

    root = tk.Tk()
    root.title("Attitude3DView demo")
    root.geometry("640x560")
    root.configure(bg=gt.BG_PANEL)

    view = Attitude3DView(get_fsm_state=lambda: "STATE_PAD")
    widget = view.build(root)
    widget.pack(fill=tk.BOTH, expand=True)

    t = [0.0]

    def tick():
        t[0] += 0.05
        roll = math.sin(t[0]) * 15
        pitch = math.cos(t[0] * 0.7) * 10
        # 簡易歐拉角(度)→四元數，僅供 demo 動畫用
        r, p, y = math.radians(roll), math.radians(pitch), math.radians(t[0] * 20 % 360)
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        view.update([qw, qx, qy, qz])
        root.after(200, tick)

    root.after(500, tick)
    root.mainloop()
