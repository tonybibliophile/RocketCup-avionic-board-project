#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
航電板自動化硬體迴路測試 (HIL) 腳本 (Mac 版)

使用方式:
  python3 test_runner.py           # 完整流程：編譯 → 燒錄 → 監聽 → 斷言
  python3 test_runner.py --monitor # 只監聽 serial（跳過編譯燒錄，適合即時 debug）
  python3 test_runner.py --flash   # 燒錄後監聽（跳過編譯）
"""

import glob
import os
import sys
import time
import subprocess
import re
import argparse
from datetime import datetime

try:
    import serial
except ImportError:
    print("正在自動安裝所需的 pyserial 套件...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
    import serial

# Add parent directory to sys.path to load serial_link
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import serial_link   # P3：port/baud 預設與自動偵測統一於此

# ==================== 配置參數 ====================
_CUBEIDE_PLUGINS = "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins"
_PROJECT_ROOT    = os.path.dirname(os.path.abspath(__file__))

def _find_cubeide(pattern):
    matches = sorted(glob.glob(os.path.join(_CUBEIDE_PLUGINS, pattern)))
    return matches[-1] if matches else ""

COMPILER_PATH  = _find_cubeide("com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin")
PROGRAMMER_CLI = _find_cubeide("com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*/tools/bin/STM32_Programmer_CLI")
DEBUG_DIR      = os.path.join(_PROJECT_ROOT, "Main_Code", "Debug")
ELF_FILE       = os.path.join(DEBUG_DIR, "Main_AV_F407.elf")

ST_LINK_SN = ""  # 留空自動選擇唯一一條 ST-Link；同時接多條時填入 SN

SERIAL_PORT = serial_link.resolve_port()   # 自動偵測（--port 可覆蓋）
BAUD_RATE = serial_link.DEFAULT_BAUD
TEST_DURATION = 30  # 完整 HIL 測試監聽時間 (秒)
MONITOR_DURATION = 0   # --monitor 模式持續時間 (秒，0 = 無限)

# HIL 斷言門檻
# ±5% 區間、含上界（不只下界）：舊版只查下限，抓不到定時器暴走（例如某感測器
# 意外跑到 2 倍速率仍會「通過」）。這批數字對應 S4/S5 韌體改動後的實際目標速率
# （陀螺 1000Hz、加速度/ADXL 400Hz、BMP388 50Hz、MMC5983/GPS 未變）；舊值
# （1550/1950/3100/140）編碼的是已作廢的 1.6/2.0/3.2kHz 設計，早已對不上現況。
EXPECTED_BMI088_A = (380.0, 420.0)
EXPECTED_BMI088_G = (950.0, 1050.0)
EXPECTED_ADXL375  = (380.0, 420.0)
EXPECTED_BMP388   = (47.0, 53.0)
EXPECTED_MMC5983  = (95.0, 105.0)
EXPECTED_MIN_GPS  = 1.0   # GPS 未變動，維持只查下限（無精確上界可查）
# ==================================================

def run_command(cmd, cwd=None, env=None):
    process = subprocess.Popen(cmd, shell=True, cwd=cwd, env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, bufsize=1)
    output_lines = []
    while True:
        line = process.stdout.readline()
        if not line and process.poll() is not None:
            break
        if line:
            print(f"  {line.strip()}")
            output_lines.append(line)
    return process.returncode, "".join(output_lines)

def build_project():
    print("\n" + "="*60)
    print("【第一步：自動編譯專案】")
    print("="*60)
    if not COMPILER_PATH:
        print("[ERROR] STM32CubeIDE 工具鏈未找到，請確認已安裝 STM32CubeIDE")
        return False
    env = os.environ.copy()
    env["PATH"] = f"{COMPILER_PATH}:{env.get('PATH', '')}"
    ret, out = run_command("make clean && make -j4 Main_AV_F407.elf", cwd=DEBUG_DIR, env=env)
    if ret != 0:
        print("\n[ERROR] 編譯失敗！")
        return False
    print("\n[OK] 編譯成功！")
    return True

def flash_mcu(elf_path=ELF_FILE, role_name="主航電 (PRIMARY)"):
    print("\n" + "="*60)
    print(f"【自動燒錄韌體】目標角色: {role_name}")
    print("="*60)
    if not PROGRAMMER_CLI:
        print("[ERROR] STM32_Programmer_CLI 未找到，請確認已安裝 STM32CubeIDE")
        return False
    if not os.path.exists(elf_path):
        print(f"[ERROR] 找不到 ELF 檔案：{elf_path}")
        return False
    sn_arg = f" sn={ST_LINK_SN}" if ST_LINK_SN else ""
    cmd = f'"{PROGRAMMER_CLI}" -c port=SWD{sn_arg} mode=UR -d "{elf_path}" -v -rst'
    ret, out = run_command(cmd)
    if "Download verified successfully" not in out:
        print("\n[ERROR] 燒錄或驗證失敗！")
        return False
    print(f"\n[OK] {role_name} 韌體燒錄成功，MCU 已重啟！")
    return True

# ─────────────────────────────────────────────────────────────
#  顏色輸出 (ANSI)
# ─────────────────────────────────────────────────────────────
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def col(color, text): return f"{color}{text}{RESET}"

# ─────────────────────────────────────────────────────────────
#  Debug 監控模式
# ─────────────────────────────────────────────────────────────
def monitor_debug(duration_s=0):
    """
    即時監聽 serial，專注診斷 GPS 和 Mag 狀態。
    duration_s = 0 → 無限迴圈直到 Ctrl-C
    """
    print("\n" + "="*60)
    print(col(BOLD, f"【Debug 監聽模式】{SERIAL_PORT} @ {BAUD_RATE} baud"))
    print(f"  持續時間: {'無限 (Ctrl-C 停止)' if duration_s == 0 else f'{duration_s} 秒'}")
    print("="*60)

    log_filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs", "serial_continuous.log")
    print(col(GREEN, f"  [INFO] 所有讀取數據將同步追加寫入 {os.path.basename(log_filename)}\n"))

    # ── 狀態追蹤 ──
    state = {
        # Mag
        "mag_init": None,          # None / "online" / "not_detected"
        "mag_offset": None,
        "mag_reads_ok": 0,
        "mag_reads_err": 0,
        "mag_last_B": None,        # (Bx, By, Bz) mG
        "mag_last_hdg": None,

        # GPS
        "gps_fix": 0,
        "gps_sats": 0,
        "gps_sentences_ok": 0,
        "gps_sentences_err": 0,
        "gps_last_lat": None,
        "gps_last_lon": None,
        "gps_seen_any": False,

        # IMU/Baro rates
        "bmi_a": 0, "bmi_g": 0, "adxl": 0, "bmp": 0,

        # EKF
        "ekf_calib": False,
        "ekf_drop": 0,

        # System
        "flash_ring_ready": False,
        "flash_pkt": 0,
    }

    # LED 預測
    def predict_leds():
        stat1 = state["gps_sentences_ok"] > 0
        stat2 = state["mag_init"] == "online"
        fix   = state["gps_fix"] == 1

        stat1_str = (col(GREEN, "● 常亮 (有fix)") if (stat1 and fix) else
                     col(YELLOW, "◎ 閃爍 (有NMEA無fix)") if stat1 else
                     col(RED, "○ 熄滅"))
        stat2_str = col(GREEN, "● 常亮") if stat2 else col(RED, "○ 熄滅")
        print(f"\n  {col(BOLD,'LED 預測')}  STAT1(GPS)={stat1_str}  STAT2(MAG)={stat2_str}")

    def print_separator():
        print(col(CYAN, "  " + "─"*56))

    try:
        ser = serial_link.open_serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        print(f"  串口已開啟。等待 MCU 資料...\n")
        start = time.time()
        last_summary = time.time()

        while True:
            if duration_s > 0 and (time.time() - start) > duration_s:
                break

            try:
                raw = ser.readline()
            except serial.SerialException:
                time.sleep(0.1)
                continue
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').strip()
            if not line:
                continue

            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            # 寫入持續日誌檔
            with open(log_filename, "a", encoding="utf-8") as f_log:
                f_log.write(f"[{ts}] {line}\n")

            # ── Mag 初始化 ──
            if "[MAG] MMC5983MA online" in line:
                state["mag_init"] = "online"
                m = re.search(r"offset\[X,Y,Z\]=(-?\d+),(-?\d+),(-?\d+)", line)
                if m:
                    state["mag_offset"] = tuple(int(m.group(i)) for i in range(1,4))
                print(col(GREEN, f"  [{ts}] [OK] MAG ONLINE  offset={state['mag_offset']}"))

            elif "[MAG] MMC5983MA NOT detected" in line:
                state["mag_init"] = "not_detected"
                print(col(RED, f"  [{ts}] [ERROR] MAG NOT DETECTED — 請確認 I2C1 接線 (SDA=PB8, SCL=PB7) 及 3V3 供電"))

            # ── Mag 週期遙測  [MAG] B[mG]:x,y,z hdg:d ok:n err:n ──
            elif line.startswith("[MAG]") and "B[mG]" in line:
                m = re.search(r"B\[mG\]:(-?\d+),(-?\d+),(-?\d+).*hdg:(\d+).*ok:(\d+).*err:(\d+)", line)
                if m:
                    bx,by,bz = int(m.group(1)),int(m.group(2)),int(m.group(3))
                    hdg = int(m.group(4))
                    state["mag_last_B"]    = (bx,by,bz)
                    state["mag_last_hdg"]  = hdg
                    state["mag_reads_ok"]  = int(m.group(5))
                    state["mag_reads_err"] = int(m.group(6))
                    print(col(GREEN,
                        f"  [{ts}] MAG  B=({bx:+5d},{by:+5d},{bz:+5d}) mG  "
                        f"hdg={hdg}°  ok={state['mag_reads_ok']}  err={state['mag_reads_err']}"))

            # ── GPS 週期遙測  [GPS] fix:x q:x sat:x ... ok:n err:n ──
            elif line.startswith("[GPS]"):
                state["gps_seen_any"] = True
                mf  = re.search(r"fix:(\d)", line)
                ms  = re.search(r"sat:(\d+)", line)
                mok = re.search(r"ok:(\d+)", line)
                mer = re.search(r"err:(\d+)", line)
                mla = re.search(r"([NS])(\d+)\.(\d+)", line)
                mlo = re.search(r"([EW])(\d+)\.(\d+)", line)

                if mf:  state["gps_fix"]  = int(mf.group(1))
                if ms:  state["gps_sats"] = int(ms.group(1))
                if mok: state["gps_sentences_ok"]  = int(mok.group(1))
                if mer: state["gps_sentences_err"] = int(mer.group(1))

                fix_col = col(GREEN,"[OK] FIX") if state["gps_fix"] else col(YELLOW,"[WAIT] 搜星中")
                sat_col = col(GREEN if state["gps_sats"]>=4 else YELLOW, f"sats={state['gps_sats']}")
                ok_col  = col(GREEN if state["gps_sentences_ok"]>0 else RED,
                              f"ok={state['gps_sentences_ok']}")
                err_col = col(RED if state["gps_sentences_err"]>0 else RESET,
                              f"err={state['gps_sentences_err']}")

                print(f"  [{ts}] GPS  {fix_col}  {sat_col}  {ok_col}  {err_col}")

                if state["gps_sentences_ok"] == 0 and state["gps_sentences_err"] > 10:
                    print(col(RED,
                        "  [WARNING]  GPS 有資料進來但 checksum 全失敗 → "
                        "可能 baud rate 不符（NEO-M9N 預設 38400，板設 115200）"))

            # ── RATE 報告 ──
            elif line.startswith("[RATE]"):
                m = re.search(r"BMI088_A:([\d\.]+)Hz.*BMI088_G:([\d\.]+)Hz.*ADXL375:([\d\.]+)Hz.*BMP388:([\d\.]+)Hz", line)
                if m:
                    state["bmi_a"],state["bmi_g"],state["adxl"],state["bmp"] = \
                        float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))
                mmc_rate = re.search(r"MMC5983:([\d\.]+)Hz", line)
                gps_rate = re.search(r"GPS:([\d\.]+)Hz", line)
                mag_rate_val = float(mmc_rate.group(1)) if mmc_rate else 0.0
                gps_rate_val = float(gps_rate.group(1)) if gps_rate else 0.0

                ekf_drop = re.search(r"EKF_DROP:(\d+)", line)
                if ekf_drop: state["ekf_drop"] = int(ekf_drop.group(1))
                print(f"  [{ts}] RATE  "
                      f"BMI-A={state['bmi_a']:.2f}Hz  BMI-G={state['bmi_g']:.2f}Hz  "
                      f"ADXL={state['adxl']:.2f}Hz  BMP={state['bmp']:.2f}Hz  "
                      f"MAG={mag_rate_val:.2f}Hz  GPS={gps_rate_val:.2f}Hz  "
                      f"EKF_DROP={state['ekf_drop']}")

            # ── EKF 校準完成 ──
            elif "Calibration Done" in line or "EKF_calibrated" in line:
                state["ekf_calib"] = True
                print(col(GREEN, f"  [{ts}] [OK] EKF 校準完成"))

            # ── Flash Ring ──
            elif "[FLASH_RING] Ready" in line:
                state["flash_ring_ready"] = True
                print(col(GREEN, f"  [{ts}] [OK] Flash Ring Buffer 就緒"))
            else:
                m_pkt = re.search(r"PKT_TOTAL:(\d+)", line)
                if m_pkt: state["flash_pkt"] = int(m_pkt.group(1))
                # 其他行原樣完整印出
                print(f"  [{ts}] {line}")

            # ── 每 10 秒打印一次摘要 ──
            if time.time() - last_summary >= 10:
                last_summary = time.time()
                print_separator()
                print(f"  {col(BOLD,'─── 10s 診斷摘要 ───')}")

                # Mag
                if state["mag_init"] is None:
                    print(col(YELLOW,"  MAG: 尚未收到初始化訊息（MCU 可能還在啟動中）"))
                elif state["mag_init"] == "not_detected":
                    print(col(RED,"  MAG: [ERROR] init 失敗 — 硬體未連接或 I2C 接線錯誤"))
                    print(col(RED,"       ↳ 確認 SDA=PB8 / SCL=PB7 / 3V3 / 4.7kΩ pull-up"))
                else:
                    print(col(GREEN,f"  MAG: [OK] online  最新 hdg={state['mag_last_hdg']}°  "
                               f"reads_ok={state['mag_reads_ok']}  err={state['mag_reads_err']}"))

                # GPS
                if not state["gps_seen_any"]:
                    print(col(RED,"  GPS: [ERROR] 完全沒收到 [GPS] 行"))
                    print(col(RED,"       ↳ 可能原因: GPS TX 未接 PC7, 或 baud rate 錯誤"))
                    print(col(RED,"       ↳ NEO-M9N 預設 baud=38400，板設 115200，若不一致需用 U-Center 設定"))
                elif state["gps_sentences_ok"] == 0:
                    print(col(YELLOW,f"  GPS: [WARNING]  有收到遙測但 sentences_ok=0  err={state['gps_sentences_err']}"))
                    print(col(YELLOW,"       ↳ 可能 baud rate 不符 → NMEA checksum 全失敗"))
                else:
                    fix_s = "有 fix [OK]" if state["gps_fix"] else "搜星中 [WAIT]"
                    print(col(GREEN,f"  GPS: [OK] sentences_ok={state['gps_sentences_ok']}  "
                               f"sats={state['gps_sats']}  {fix_s}"))

                predict_leds()
                print_separator()

        ser.close()

    except serial.SerialException as e:
        print(col(RED, f"\n[ERROR] 串口連線失敗: {e}"))
        print(col(YELLOW, f"   請確認 {SERIAL_PORT} 是否正確，或改用 --port /dev/cu.xxx"))
        return False
    except KeyboardInterrupt:
        print("\n\n  [使用者中斷]")

    # 最終報告
    print("\n" + "="*60)
    print(col(BOLD, "【最終診斷報告】"))
    print("="*60)

    ok = True

    mag_result = (state["mag_init"] == "online")
    print(f"  MAG  : {col(GREEN,'[OK] online') if mag_result else col(RED,'[ERROR] NOT detected')}")
    if not mag_result:
        ok = False
        print(col(RED,"         → 確認 I2C1 (PB8=SDA, PB7=SCL), 3V3, 4.7kΩ pull-up"))

    gps_alive = state["gps_sentences_ok"] > 0
    gps_fix   = state["gps_fix"] == 1
    if gps_alive:
        print(f"  GPS  : {col(GREEN,'[OK] NMEA ok')}  sats={state['gps_sats']}  "
              f"fix={'yes [OK]' if gps_fix else 'no (需戶外)'}")
    else:
        ok = False
        if state["gps_seen_any"]:
            print(col(YELLOW,f"  GPS  : [WARNING]  有遙測但 sentences_ok=0 err={state['gps_sentences_err']}"))
            print(col(YELLOW,"         → baud rate 不符? NEO-M9N 預設 38400，需 U-Center 改 115200"))
        else:
            print(col(RED,"  GPS  : [ERROR] 完全沒收到 [GPS] 行"))
            print(col(RED,"         → GPS TX 未接 PC7? 或模組未供電?"))

    print(f"\n  STAT1 預測: {col(GREEN,'● 閃爍/常亮') if gps_alive else col(RED,'○ 熄滅')}")
    print(f"  STAT2 預測: {col(GREEN,'● 常亮') if mag_result else col(RED,'○ 熄滅')}")

    return ok

# ─────────────────────────────────────────────────────────────
#  完整 HIL 測試模式（原有功能保留）
# ─────────────────────────────────────────────────────────────
def monitor_serial_and_verify():
    print("\n" + "="*60)
    print(f"【HIL 監聽 & 自動斷言】| 時長: {TEST_DURATION}s")
    print("="*60)

    bmi_acc_rates, bmi_gyro_rates, adxl_rates, bmp_rates = [], [], [], []
    mmc_rates, gps_rates = [], []
    flash_ring_ready = False
    flash_ring_final_pkt = 0

    # P1：新觀測斷言（P0 加入的 1Hz 診斷行）
    health_bad_lines = []     # 開機 5s 寬限後出現的非零 [HEALTH] 行
    loop_max_us_seen = 0      # [LOOP] max_us 全程最大值
    flash_pool_first = None   # [FLASH] pool 首見值
    flash_pool_target = None  # [FLASH] pool=N/target 的 target（動態解析，不寫死；
                               # FLASH_RING_PREERASE_TARGET 本身已改過好幾次，硬編數字
                               # 只會再度過時，直接讀韌體印出的分母才是唯一真相來源）
    flash_pool_last  = None   # [FLASH] pool 最後值
    stack_min = {}            # [STACK] 各任務歷史最低剩餘 bytes
    t0_boot = time.time()

    try:
        ser = serial_link.open_serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        start_time = time.time()
        print("  接收遙測中...")

        while time.time() - start_time < TEST_DURATION:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                print(f"  [Tele]: {line}")

                m = re.search(r"\[RATE\]\s*BMI088_A:([\d\.]+)Hz,\s*BMI088_G:([\d\.]+)Hz,\s*ADXL375:([\d\.]+)Hz,\s*BMP388:([\d\.]+)Hz,\s*MMC5983:([\d\.]+)Hz,\s*GPS:([\d\.]+)Hz", line)
                if m:
                    a,g,x,b = float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))
                    m_rate, g_rate = float(m.group(5)), float(m.group(6))
                    if 0.0 in (a,g,x,b):
                        print("  [SKIP] 暖機期，跳過")
                        continue
                    bmi_acc_rates.append(a); bmi_gyro_rates.append(g)
                    adxl_rates.append(x);    bmp_rates.append(b)
                    mmc_rates.append(m_rate); gps_rates.append(g_rate)
                    print(f"  RATE BMI-A={a:.2f}Hz G={g:.2f}Hz ADXL={x:.2f}Hz BMP={b:.2f}Hz MAG={m_rate:.2f}Hz GPS={g_rate:.2f}Hz")

                if "[FLASH_RING] Ready." in line:
                    flash_ring_ready = True
                    start_time = time.time()
                    print("  [OK] Flash Ring 就緒，重置計時")

                m2 = re.search(r"\[FLASH_RING\] PKT_TOTAL:(\d+)", line)
                if m2: flash_ring_final_pkt = int(m2.group(1))

                # ── P1：[HEALTH] sens=0xXX ekf=0xXX（開機 5s 寬限：首批批次未進、
                #         monitors 未餵入前 STALE 屬預期） ──
                mh = re.search(r"\[HEALTH\]\s*sens=0x([0-9A-Fa-f]+)\s*ekf=0x([0-9A-Fa-f]+)", line)
                if mh and (time.time() - t0_boot) > 5.0:
                    sens_b, ekf_b = int(mh.group(1), 16), int(mh.group(2), 16)
                    if sens_b != 0 or ekf_b != 0:
                        health_bad_lines.append(line)
                        print(col(RED, f"  [WARNING] 健康位非零: {line}"))

                # ── P1：[LOOP] max_us=N 水位 ──
                ml = re.search(r"\[LOOP\]\s*max_us=(\d+)", line)
                if ml:
                    loop_max_us_seen = max(loop_max_us_seen, int(ml.group(1)))

                # ── P1：[FLASH] pool=N/target 背景預擦進度（target 動態解析，見上方註解） ──
                mp = re.search(r"\[FLASH\]\s*pool=(\d+)/(\d+)", line)
                if mp:
                    val = int(mp.group(1))
                    flash_pool_target = int(mp.group(2))
                    if flash_pool_first is None: flash_pool_first = val
                    flash_pool_last = val

                # ── P1：[STACK] main=N ekf=N lora=N（0.2Hz，歷史最低剩餘 bytes） ──
                ms = re.search(r"\[STACK\]\s*main=(\d+)\s*ekf=(\d+)\s*lora=(\d+)", line)
                if ms:
                    for name, v in zip(("main", "ekf", "lora"), map(int, ms.groups())):
                        stack_min[name] = min(stack_min.get(name, 1 << 30), v)

                # ── LoRa E80 參數驗證 ──
                mlora = re.search(r"\[LORA 920MHz\].*Freq:\s*([\d\.]+)\s*MHz.*BW:\s*(\d+)kHz", line)
                if mlora:
                    freq_str, bw_str = mlora.group(1), mlora.group(2)
                    print(col(GREEN, f"  [OK] 驗證 E80 LoRa 實時參數配置: Freq={freq_str} MHz, BW={bw_str} kHz"))

        ser.close()
    except Exception as e:
        print(f"\n[ERROR] 串口錯誤: {e}")
        return False

    print("\n" + "="*60)
    print("【HIL 斷言結果】")
    print("="*60)

    if not bmi_acc_rates:
        print("[ERROR] 未收到 [RATE] 資料")
        return False

    avg = lambda lst: sum(lst)/len(lst)
    range_results = {
        "BMI088 Accel": (avg(bmi_acc_rates),  EXPECTED_BMI088_A),
        "BMI088 Gyro" : (avg(bmi_gyro_rates), EXPECTED_BMI088_G),
        "ADXL375"     : (avg(adxl_rates),     EXPECTED_ADXL375),
        "BMP388"      : (avg(bmp_rates),      EXPECTED_BMP388),
        "MMC5983MA"   : (avg(mmc_rates),      EXPECTED_MMC5983),
    }
    failures = []
    for name, (val, (lo, hi)) in range_results.items():
        ok = lo <= val <= hi
        sym = "[OK]" if ok else "[ERROR]"
        print(f"  {sym} {name}: {val:.1f} Hz (應在 [{lo}, {hi}] 區間)")
        if not ok: failures.append(f"{name} 超出預期區間 ({val:.1f}Hz，應為 [{lo},{hi}])")

    gps_val = avg(gps_rates)
    gps_ok = gps_val >= EXPECTED_MIN_GPS
    print(f"  {'[OK]' if gps_ok else '[ERROR]'} GPS: {gps_val:.1f} Hz (>= {EXPECTED_MIN_GPS})")
    if not gps_ok: failures.append(f"GPS 過低 ({gps_val:.1f}Hz)")

    # Flash Ring：S3.1（PAD/PAD_ARMED 完全不寫入，整池保留給飛行）落地後，本 HIL 腳本
    # 是純被動監聽、不主動送 ARM/模擬起飛，全程停在 PAD → 飛行封包數本該恆為 0，這是
    # 新的預期行為（正面驗證），不是待補的舊 EXPECTED_MIN_RING_PACKETS 下限失效。
    # flash_ring_ready（開機 bulk 預擦是否完成，見 main.c FlashRing_InitEx）才是這裡
    # 唯一還有意義的硬性檢查——若要驗證飛行中真的會寫入，需改走能驅動 FSM 進 BOOST
    # 的測試路徑（例如上行測試指令模擬起飛），不在本腳本範圍內。
    print(f"\n  Flash Ring: {'[OK]' if flash_ring_ready else '[ERROR]'} ready")
    if not flash_ring_ready: failures.append("Flash Ring 未就緒（開機預擦未完成或逾時，見 TEST_DURATION 是否 > ~48s）")
    print(f"  Flash pkt（PAD 態應恆為 0，S3.1）: {flash_ring_final_pkt}")
    if flash_ring_final_pkt != 0:
        failures.append(f"Flash pkt 非 0 ({flash_ring_final_pkt})——PAD/PAD_ARMED 不應寫入，檢查是否已進入 BOOST 或 S3.1 邏輯回歸")

    # ── P1：健康位斷言（開機 5s 寬限後必須全程 sens=0 ekf=0） ──
    if health_bad_lines:
        print(f"  [ERROR] [HEALTH] 出現 {len(health_bad_lines)} 行非零健康位（誤報率檢查失敗）")
        failures.append(f"[HEALTH] 非零 ×{len(health_bad_lines)}（首行: {health_bad_lines[0]}）")
    else:
        print("  [OK] [HEALTH] 全程 sens=0x00 ekf=0x00")

    # ── P1：[LOOP] 水位（PAD 期背景預擦單次最壞 ~400ms 屬預期，>450ms 視為異常；
    #        飛行態 <20ms 斷言需配合 HIL_FSM_BENCH 場景另行驗證） ──
    if loop_max_us_seen > 0:
        loop_ok = loop_max_us_seen <= 450000
        sym = "[OK]" if loop_ok else "[ERROR]"
        note = ""
        if 20000 < loop_max_us_seen <= 450000:
            note = "（PAD 期背景預擦造成的尖峰屬預期；飛行態斷言 <20ms 需 HIL_FSM_BENCH）"
        print(f"  {sym} [LOOP] max_us={loop_max_us_seen} {note}")
        if not loop_ok: failures.append(f"[LOOP] 單迴圈耗時異常 ({loop_max_us_seen}us > 450ms)")
    else:
        print("  [WARNING] 未收到 [LOOP] 行（韌體版本過舊?）")

    # ── P1：[FLASH] pool 背景預擦進度 ──
    # 開機 bulk 預擦（main.c FlashRing_InitEx，~target×50ms，例如 960 sectors≈48s）已在
    # 主迴圈啟動前把池子一次擦滿；主迴圈裡殘留的背景 PreEraseOne 只在 INIT/PAD 且池未達標
    # 時才印 [FLASH] pool 行（P0-E 安全網，見 main.c）。故本監聽視窗完全收不到這行是新
    # 設計下的正常情況（代表 bulk 預擦已完成、沒有缺口要補），不再代表韌體版本過舊。
    if flash_pool_first is not None:
        target = flash_pool_target or flash_pool_last  # 保底：萬一沒解析到分母，退化用最後觀測值當基準
        grew = flash_pool_last >= flash_pool_first + 5 or flash_pool_last >= target
        sym = "[OK]" if grew else "[WARNING]"
        print(f"  {sym} [FLASH] pool {flash_pool_first} → {flash_pool_last}/{target}"
              f"{'' if grew else '（增長停滯，檢查背景預擦）'}")
        if not grew: failures.append(f"[FLASH] pool 增長停滯 ({flash_pool_first}→{flash_pool_last})")
    else:
        print("  [INFO] 監聽期間未收到 [FLASH] pool 行——開機 bulk 預擦通常已在主迴圈啟動前"
              "把池子一次擦滿，背景安全網無缺口可補、不會印這行，屬正常（非韌體版本過舊）")

    # ── P1：[STACK] 任務堆疊餘裕（歷史最低剩餘 ≥256 bytes，逼近 0 = 溢位前兆） ──
    if stack_min:
        worst = min(stack_min.values())
        stack_ok = worst >= 256
        sym = "[OK]" if stack_ok else "[ERROR]"
        detail = " ".join(f"{k}={v}" for k, v in stack_min.items())
        print(f"  {sym} [STACK] 最低剩餘 {detail} bytes")
        if not stack_ok: failures.append(f"[STACK] 堆疊餘裕不足 ({detail})")
    else:
        print("  [WARNING] 未收到 [STACK] 行（韌體版本過舊?）")

    if failures:
        print(col(RED, "\n[ERROR] FAILED"))
        for f in failures: print(f"  [WARNING] {f}")
        return False
    print(col(GREEN, f"\n[PASS] PASSED — 所有感測器正常，Flash pool ready={flash_ring_ready}"
                      f"（PAD 態飛行封包=0 為預期值，S3.1）"))
    return True

# ─────────────────────────────────────────────────────────────
#  入口
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="航電板 HIL 測試工具")
    parser.add_argument("--monitor", action="store_true", help="只監聽 serial（跳過編譯燒錄）")
    parser.add_argument("--flash",   action="store_true", help="燒錄後監聽")
    parser.add_argument("--role",    choices=["primary", "backup", "ground"], default="primary", help="選擇燒錄目標角色 (primary:主航電, backup:備援航電, ground:地面站)")
    parser.add_argument("--port",    default=SERIAL_PORT,  help=f"串口路徑 (預設 {SERIAL_PORT})")
    parser.add_argument("--select",  action="store_true", help="互動式手動選擇串口選單")
    parser.add_argument("--duration",type=int, default=MONITOR_DURATION, help="監聽秒數 (0=無限)")
    args = parser.parse_args()

    if args.select:
        sel = serial_link.prompt_select_port()
        if sel:
            SERIAL_PORT = sel
    else:
        SERIAL_PORT = args.port  # 允許命令列覆蓋

    if args.monitor:
        print("Debug 監聽模式（不編譯不燒錄）")
        monitor_debug(args.duration)
        sys.exit(0)

    # 決定 ELF 檔案與角色名稱
    role_map = {
        "primary": ("Main_Code/Debug/Main_AV_F407.elf", "主航電 (PRIMARY)"),
        "backup":  ("Main_Code/Debug/Main_AV_F407_backup.elf", "備援航電 (BACKUP)"),
        "ground":  ("Main_Code/Debug/Main_AV_F407_ground.elf", "地面站 (GROUND)")
    }
    target_elf, target_role_name = role_map[args.role]

    if args.flash:
        print(f"燒錄 [{target_role_name}] + 監聽模式")
        if not flash_mcu(target_elf, target_role_name): sys.exit(1)
        time.sleep(2)
        monitor_debug(args.duration)
        sys.exit(0)

    # 完整 HIL 流程
    print("完整 HIL 測試流程（編譯 → 燒錄 → 斷言）")
    if not build_project(): sys.exit(1)
    if not flash_mcu(target_elf, target_role_name): sys.exit(1)
    if not monitor_serial_and_verify(): sys.exit(2)
    sys.exit(0)
