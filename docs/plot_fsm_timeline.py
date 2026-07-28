#!/usr/bin/env python3
"""Annotated altitude-time timeline for the FSM x OpenRocket safety review.
Reads v3.csv (the "正確"/corrected two-stage recovery sim, supersedes v2模擬.csv)
and overlays FSM event times + the current (100s) vs recommended (~255s) main
watchdog lines. English labels only (avoid CJK glyph fallback in matplotlib).

v3.csv column layout differs from v1/v2: Time, Altitude, Total velocity (not
signed vertical velocity), Total acceleration, Roll/Pitch/Yaw rate, Stability
margin. We only need Time/Altitude/Total velocity here.
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = "/Users/laizhiquan/coding/RocketCom"
CSV = os.path.join(ROOT, "simulation_and_data/flight_data/v3.csv")
OUT = os.path.join(ROOT, "docs/fsm_v3_timeline.png")

t, alt, vtot = [], [], []
with open(CSV, newline="") as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        p = line.split(",")
        try:
            tt, hh, vv = float(p[0]), float(p[1]), float(p[2])
        except (ValueError, IndexError):
            continue
        t.append(tt); alt.append(hh); vtot.append(vv)

apogee_h = max(alt); apogee_t = t[alt.index(apogee_h)]

fig, ax = plt.subplots(figsize=(12, 6.4))
ax.plot(t, alt, color="#1f77b4", lw=1.6, label="Altitude (OpenRocket v3, corrected two-stage)")

# ---- FSM event markers (flight profile) ----
def crossing(target):
    for i in range(alt.index(apogee_h), len(alt)):
        if alt[i] <= target:
            return t[i], alt[i]
    return None, None

t350, _ = crossing(350.0)
t300, _ = crossing(300.0)
drogue_fsm_t = apogee_t - 3.0          # FSM predictive drogue: 3.0s lead
land_t = t[-1]

events = [
    (0.0,        0.0,      "Liftoff  t=0",                 "#2ca02c", "bottom"),
    (6.56,       763.0,    "Burnout (a_z<0.5g)  ~6.6s",     "#2ca02c", "bottom"),
    (drogue_fsm_t, apogee_h-150, "FSM drogue (3s lead) ~%.0fs" % drogue_fsm_t, "#ff7f0e", "top"),
    (apogee_t,   apogee_h, "Apogee  %.0f m @ %.1fs" % (apogee_h, apogee_t), "#d62728", "bottom"),
    (t350,       350.0,    "Main path h<=350m  t=%.0fs" % t350, "#9467bd", "bottom"),
    (241.467,    299.5,    "OpenRocket MAIN event  t=241.5s", "#17becf", "top"),
    (land_t,     0.0,      "Landing  t=%.0fs" % land_t,     "#8c564b", "top"),
]
for et, eh, lab, col, va in events:
    ax.plot(et, eh, "o", color=col, ms=7, zorder=5)
    ax.annotate(lab, (et, eh), textcoords="offset points",
                xytext=(6, 12 if va == "bottom" else -18),
                fontsize=9, color=col, fontweight="bold")

# ---- Watchdog: current 100s (the bug) ----
alt_at_100 = next(a for tt, a in zip(t, alt) if tt >= 100.0)
ax.axvline(100.0, color="red", ls="--", lw=2)
ax.plot(100.0, alt_at_100, "X", color="red", ms=13, zorder=6)
ax.annotate("CURRENT watchdog = 100s\n-> forces MAIN at %.0f m  (BUG)" % alt_at_100,
            (100.0, alt_at_100), textcoords="offset points", xytext=(12, 4),
            fontsize=10, color="red", fontweight="bold")

# ---- Recommended watchdog window 250-260s ----
ax.axvspan(250.0, 260.0, color="green", alpha=0.15)
ax.annotate("recommended\nwatchdog window\n250-260s\n(after main %.0fs,\nbefore landing %.0fs)" % (t300, land_t),
            (255.0, 1400.0), fontsize=9, color="#187a18", ha="center", fontweight="bold")

ax.axhline(300.0, color="gray", ls=":", lw=1)
ax.annotate("TARGET_MAIN = 300 m", (5, 330), fontsize=8, color="gray")

ax.set_xlabel("Time since liftoff (s)")
ax.set_ylabel("Altitude (m)")
ax.set_title("FSM main-parachute watchdog vs real two-stage descent (OpenRocket v3, corrected, 3.26 km)")
ax.set_xlim(-5, 290); ax.set_ylim(-80, 3450)
ax.grid(True, alpha=0.3)
ax.legend(loc="upper right", fontsize=9)
fig.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.savefig(OUT, dpi=130)
print("wrote", OUT)
print("apogee=%.1fm@%.2fs  main350@%.1fs  main300@%.1fs  land@%.1fs  alt@100s=%.0fm"
      % (apogee_h, apogee_t, t350, t300, land_t, alt_at_100))
