#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磁力計硬鐵校正輔助腳本
===========================================================================
消除周遭金屬/PCB 走線造成的外部磁場干擾（hard-iron calibration）。
與板上自動執行的 SET/RESET 橋偏校正（MMC5983_Recalibrate，消除感測器晶片
內部電路偏置）是兩件不同的事——這支腳本補的是外部干擾這一塊，板子出廠/
開機都不會自動做，需要實際繞軸旋轉才能量出來。

用法：
    python3 mag_calibrate.py [--send]

流程：
    1. 開機時讀取 "[MAG] MMC5983MA online. offset[X,Y,Z]=..." 取得目前橋偏
       （MMC5983_SetOffsets 是整個取代橋偏值，不是疊加校正量，所以最終要送出
       的是「橋偏 + 硬鐵修正」的合計值，不能只送硬鐵修正量）。
       若腳本啟動時已經錯過開機那行，會提示先重啟板子。
    2. 送出 CMD_MAG_CAL_START 把 [MAG] 印出頻率從 1Hz 提到 10Hz，收集 30 秒
       即時讀值（同秒數樣本數多 10 倍），期間請把板子繞 X/Y/Z 三軸盡量各轉
       一整圈（畫 8 字最快涵蓋全部方向）；結束後送 CMD_MAG_CAL_STOP 恢復 1Hz。
    3. 各軸取 (min+max)/2 當硬鐵偏移量（mG），換算回 raw ADC counts 疊加到
       開機橋偏上，印出可以直接貼給板子的 CMD_MAG_CAL:x,y,z 指令。
    4. --send 會嘗試透過同一條序列埠直接送出指令並印出板子的回應確認。

換算依據（firmware/main_flight_code/Core/Inc/mmc5983.h）：
    MMC5983_LSB_PER_GAUSS = 16384   （18-bit，16384 counts/Gauss）
    mG -> raw counts：counts = mG_value * 16384 / 1000
"""
import sys
import os
import re
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import serial_link  # noqa: E402

LSB_PER_GAUSS = 16384.0
COLLECT_SECONDS = 30.0

# 逐字慢送間隔：板端 Poll_Serial_Commands 是 osPriorityLow 任務的 20ms 輪詢
# （timeout=0、硬體僅 1 byte 緩衝），系統忙碌時輪詢間隔會被高優先權任務拉長
# 到數十 ms——整串 burst 送或 30ms/字都會掉字元（實測），100ms/字實測可靠。
CMD_CHAR_GAP = 0.1


def slow_write(ser, data, gap=CMD_CHAR_GAP):
    """逐字慢送：每個字元間隔 gap 秒，遷就板端低優先權輪詢接收。"""
    for i in range(len(data)):
        ser.write(data[i:i + 1])
        time.sleep(gap)


def send_cmd_verified(ser, cmd_bytes, expect, tries=3, wait_s=2.5):
    """慢送指令並等待板子回應關鍵字（bytes）；回傳 (是否成功, 期間收到的資料)。
    偶發掉字元靠重試吸收（實測第 1 次可能失敗、第 2 次成功）。"""
    captured = b""
    for _ in range(tries):
        slow_write(ser, cmd_bytes)
        t0 = time.time()
        while time.time() - t0 < wait_s:
            chunk = ser.read(4096)
            if chunk:
                captured += chunk
                if expect in captured:
                    return True, captured
    return False, captured


BOOT_OFFSET_RE = re.compile(
    r"\[MAG\] MMC5983MA online\. offset\[X,Y,Z\]=(-?\d+),(-?\d+),(-?\d+)"
)
MAG_LIVE_RE = re.compile(r"\[MAG\] B\[mG\]:(-?\d+),(-?\d+),(-?\d+)")


def main():
    send_cmd = "--send" in sys.argv

    ser = serial_link.open_serial(timeout=0.5)
    print(f"[mag_cal] 已連線 {ser.port} @ {ser.baudrate}")
    print("[mag_cal] 等待開機橋偏訊息...若板子已經開機一段時間，"
          "請按板上重置鍵重啟後再試一次（Ctrl+C 取消）。")

    boot_offset = None
    t_wait_start = time.time()
    buf = b""
    # 60s 上限：涵蓋「先啟動本腳本、再用燒錄器觸發重置」的整段燒錄時間（~20s）
    while boot_offset is None and time.time() - t_wait_start < 60.0:
        chunk = ser.read(4096)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="ignore")
            m = BOOT_OFFSET_RE.search(text)
            if m:
                boot_offset = tuple(int(g) for g in m.groups())
                print(f"[mag_cal] 讀到開機橋偏: offset[X,Y,Z]={boot_offset}")

    if boot_offset is None:
        print("[mag_cal] ERROR：60 秒內沒等到開機橋偏訊息，請重啟板子後立刻執行本腳本。")
        ser.close()
        sys.exit(1)

    # 提速 [MAG] 印出頻率 1Hz->10Hz（CMD_MAG_CAL_START/STOP，main.c 對應實作），
    # 收集階段樣本數多 10 倍，同樣秒數涵蓋更完整的旋轉軌跡。
    # try/finally 保護：STOP 一定會送出，即使收集中途 Ctrl+C，避免板子卡在
    # 10Hz 快速模式回不去。
    # 先等 1.5s 讓 RTOS 排程器穩定，再用慢送+回應驗證確認指令真的送達。
    time.sleep(1.5)
    start_ok, _ = send_cmd_verified(ser, b"CMD_MAG_CAL_START\n", b"fast-print ON")
    collect_s = COLLECT_SECONDS
    if start_ok:
        print("[mag_cal] CMD_MAG_CAL_START 已確認送達（[MAG] 已提速到 10Hz）。")
    else:
        collect_s = COLLECT_SECONDS * 2
        print("[mag_cal] WARNING：CMD_MAG_CAL_START 未確認送達，維持 1Hz 收集、"
              f"時間延長為 {collect_s:.0f} 秒以補足樣本數。")
    ser.reset_input_buffer()

    mins = [None, None, None]
    maxs = [None, None, None]
    n = 0
    try:
        print(f"\n[mag_cal] 開始收集 {collect_s:.0f} 秒磁力計讀值——"
              f"現在開始把板子繞 X/Y/Z 三軸盡量各轉一整圈（畫 8 字最快涵蓋全部方向）。\n")

        t0 = time.time()
        while time.time() - t0 < collect_s:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="ignore")
                m = MAG_LIVE_RE.search(text)
                if m:
                    vals = tuple(int(g) for g in m.groups())
                    for i in range(3):
                        mins[i] = vals[i] if mins[i] is None else min(mins[i], vals[i])
                        maxs[i] = vals[i] if maxs[i] is None else max(maxs[i], vals[i])
                    n += 1
            remaining = collect_s - (time.time() - t0)
            print(f"\r[mag_cal] 收集中... 剩 {remaining:4.1f}s  樣本數={n}", end="", flush=True)
        print()
    finally:
        # 恢復 1Hz，避免板子長期跑在提速狀態（Ctrl+C 也會走到這裡）
        stop_ok, _ = send_cmd_verified(ser, b"CMD_MAG_CAL_STOP\n", b"fast-print OFF")
        if not stop_ok:
            print("[mag_cal] WARNING：CMD_MAG_CAL_STOP 未確認送達，"
                  "板子可能還在 10Hz 模式（重啟板子即恢復預設 1Hz）。")

    if mins[0] is None:
        print("[mag_cal] ERROR：沒收到任何 [MAG] 樣本，請確認板子仍在輸出、重跑一次。")
        ser.close()
        sys.exit(1)
    if n < 20:
        print("[mag_cal] WARNING：樣本數過少，讀值可能不穩定，建議重跑。")

    axis_names = ["X", "Y", "Z"]
    ranges = [maxs[i] - mins[i] for i in range(3)]
    print("\n[mag_cal] 各軸涵蓋範圍（mG）：")
    for i in range(3):
        print(f"    {axis_names[i]}: min={mins[i]:+6d}  max={maxs[i]:+6d}  range={ranges[i]:5d}")
        if ranges[i] < 200:
            print(f"       WARNING：{axis_names[i]} 軸涵蓋範圍過小，"
                  f"該軸可能沒有轉到足夠角度，校正精度會打折。")

    hard_iron_mg = [(mins[i] + maxs[i]) / 2.0 for i in range(3)]
    # [MAG] 行是 body frame，但板端 offset[] 是「晶片軸」counts。依 sensor_axis.h
    # 映射 bx=+sy, by=-sx, bz=-sz 反推：chipX←-body_cy、chipY←+body_cx、chipZ←-body_cz。
    delta_chip_mg = [-hard_iron_mg[1], +hard_iron_mg[0], -hard_iron_mg[2]]
    delta_counts = [delta_chip_mg[i] * LSB_PER_GAUSS / 1000.0 for i in range(3)]
    total_offset = [int(round(boot_offset[i] + delta_counts[i])) for i in range(3)]

    print(f"\n[mag_cal] 硬鐵偏移量（mG）：X={hard_iron_mg[0]:+.1f} "
          f"Y={hard_iron_mg[1]:+.1f} Z={hard_iron_mg[2]:+.1f}")
    print(f"[mag_cal] 開機橋偏 + 硬鐵修正 = 最終要寫入的 raw offset：")
    cmd = f"CMD_MAG_CAL:{total_offset[0]},{total_offset[1]},{total_offset[2]}"
    print(f"\n    {cmd}\n")

    if send_cmd:
        print("[mag_cal] --send：慢送指令並等待板子確認...")
        ok, resp = send_cmd_verified(ser, (cmd + "\n").encode(),
                                     b"offset saved", wait_s=3.0)
        text = resp.decode(errors="ignore")
        for ln in text.splitlines():
            if "[CAL]" in ln:
                print(f"[mag_cal] 板子回應: {ln.strip()}")
        if ok:
            print("[mag_cal] 校正值已確認寫入 Flash。")
        else:
            print("[mag_cal] WARNING：未收到寫入確認，請手動貼上上面的指令重送。")
    else:
        print("[mag_cal] 請透過序列埠終端機（如 make monitor）手動貼上以上指令，"
              "或加 --send 參數讓本腳本直接送出。")

    ser.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[mag_cal] 已取消。")
