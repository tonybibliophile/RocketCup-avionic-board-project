#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
telemetry_decoder.py — RocketCom 下行遙測二進制解碼器（地面站）

封包契約（與 Main_Code/Core/Inc/telemetry.h 同步,由 tests/test_telemetry.c 機器鎖定）:
    99 bytes packed little-endian, sync 0xA5,0x5A,
    CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF) 覆蓋前 97 bytes。
    （★2026-07-30 瘦身：拿掉 baro_press_pa（與 baro_alt_cm 冗餘）、mag 三軸、
      高G 三軸縮成 hg_mag_cg 模長；換上飛行滾動極值 max_alt_m/max_vel_ms/max_acc_cg
      ——火箭端全速率追蹤、每包重複攜帶，433 只有 ~2Hz 抓不到真峰值，見 telemetry.h。
      另加 drogue_alt_m/main_alt_m 實際開傘高度（哨兵 -32768 = 未開傘），三者皆跨熱重啟保存。）
    （含本板垂直濾波器 (VF) 摘要：vf_pos_z_cm/vf_vel_z_cms；主/副協同「對端(副板)
      摘要」尾段：peer_fsm_state/peer_flags/peer_baro_cm/peer_link/
      peer_vf_h_cm/peer_vf_v_cms/peer_bench_arb——★2026-07-30 起對端只中繼 VF，
      不再帶 EKF 高度/速度、高G、丟包率（見 telemetry.h 同日註解）；
      arm_flags——ARM 被 flash pool 擋下時的原因位元，見 TELEM_ARM_* / arm_flags；
      profile_flags——電梯測試 profile 醒目標示（本板/對端），見 TELEM_PROFILE_*。）

用法:
    python3 telemetry_decoder.py --port /dev/cu.usbserial-110 --baud 460800
    python3 telemetry_decoder.py --file dump.bin
    python3 telemetry_decoder.py --file dump.bin --csv out.csv
    python3 telemetry_decoder.py --selftest

來源可以是 E22 433MHz 透傳接收端 UART、E80 920MHz 接收端,或任何含封包流的檔案。
逐位元組掃描 sync 對齊,CRC 失敗自動滑動重同步;統計丟包(seq gap)與 CRC 錯誤。
"""
import argparse
import struct
import sys
import time

PACKET_SIZE = 99
SYNC0, SYNC1 = 0xA5, 0x5A

# 與 test_telemetry.c offset 表一一對應(little-endian)
_STRUCT_FMT = "<4BI2i4hi7h2ih2B2H3B2iHhH2h2BiB2i3BH"
_FIELDS = [
    "sync0", "sync1", "seq", "fsm_state", "tick_ms",
    "ekf_pos_z_cm", "ekf_vel_z_cms",
    "ekf_q0", "ekf_q1", "ekf_q2", "ekf_q3",
    "baro_alt_cm",
    "imu_ax_mg", "imu_ay_mg", "imu_az_mg",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "hg_mag_cg",
    "gps_lat_1e6", "gps_lon_1e6", "gps_alt_m", "gps_sats", "gps_fix",
    "bat_mv", "cpu_main_x10",
    "flags", "health_bits", "sensor_bits",
    "vf_pos_z_cm", "vf_vel_z_cms",
    "max_alt_m", "max_vel_ms", "max_acc_cg",
    "drogue_alt_m", "main_alt_m",
    "peer_fsm_state", "peer_flags", "peer_baro_cm", "peer_link",
    "peer_vf_h_cm", "peer_vf_v_cms",
    "arm_flags", "peer_bench_arb", "profile_flags",
    "crc16",
]
assert struct.calcsize(_STRUCT_FMT) == PACKET_SIZE, struct.calcsize(_STRUCT_FMT)


def _fmt_field_count(fmt: str) -> int:
    """展開 struct 格式字串（如 "4B" -> 4 個欄位）算總欄位數，驗證與 _FIELDS 對齊。"""
    n, digits = 0, ""
    for ch in fmt:
        if ch == "<":
            continue
        if ch.isdigit():
            digits += ch
        else:
            n += int(digits) if digits else 1
            digits = ""
    return n


assert _fmt_field_count(_STRUCT_FMT) == len(_FIELDS), \
    (_fmt_field_count(_STRUCT_FMT), len(_FIELDS))

FSM_NAMES = ["INIT", "PAD", "PAD_ARMED", "BOOST", "COAST", "DEPLOY_DROGUE", "APOGEE", "DESCENT", "MAIN", "LANDED"]
FLAG_NAMES = [
    (0x01, "DROGUE"), (0x02, "MAIN"), (0x04, "SD"), (0x08, "GPS_STALE"),
    (0x10, "EKF_UNHEALTHY"), (0x20, "SENSOR_FAULT"), (0x40, "FAILSAFE"), (0x80, "HOTSTART"),
]
# peer_link 位（主/副協同：主板中繼的副板鏈路健康）
PEER_LINK_EVER, PEER_LINK_FRESH, PEER_LINK_LOST, PEER_LINK_DESYNC = 0x01, 0x02, 0x04, 0x08

# arm_flags 位（ARM 被擋下的原因，與 telemetry.h TELEM_ARM_* 對應）
TELEM_ARM_BLOCKED_FLASH_POOL = 0x01
# ★2026-07-31：航電開機不再自動擦除 flash，池未達標時 ARM 會被擋。此位在「使用者按 ARM
# 之前」就先亮，讓地面站能提早提醒下 `flash erase`（整環，飛前正規流程）或 `flash pool`
# （只補不足部分）。bit0 = 已按 ARM 但被擋；bit1 = 還沒按就先警告。
TELEM_ARM_NEED_ERASE = 0x02

# drogue_alt_m / main_alt_m 的「尚未開傘」哨兵（與 telemetry.h TELEM_DEPLOY_ALT_NA 一致）。
# 刻意不用 0：0 m 是合法開傘高度（地面誤觸發），用 0 當「沒開」會把該警示的事件藏起來。
TELEM_DEPLOY_ALT_NA = -32768

# profile_flags 位（電梯測試 profile 醒目標示，與 telemetry.h TELEM_PROFILE_* 對應）
TELEM_PROFILE_SELF_ELEVATOR = 0x01   # 本板（發下鏈的這片板）仍以電梯測試 profile 編譯
TELEM_PROFILE_PEER_ELEVATOR = 0x02   # 對端（副板，經板間鏈路中繼）仍以電梯測試 profile 編譯


def elevator_test_active(p: dict) -> bool:
    """只要本板或對端仍是電梯測試 profile 就回傳 True——GUI/log 據此觸發醒目警告。"""
    prof = p.get("profile_flags", 0)
    return bool(prof & (TELEM_PROFILE_SELF_ELEVATOR | TELEM_PROFILE_PEER_ELEVATOR))


def elevator_test_who(p: dict) -> str:
    prof = p.get("profile_flags", 0)
    who = []
    if prof & TELEM_PROFILE_SELF_ELEVATOR: who.append("本板")
    if prof & TELEM_PROFILE_PEER_ELEVATOR: who.append("對端(副板)")
    return "+".join(who) if who else ""


def fmt_peer(p: dict) -> str:
    link = p.get("peer_link", 0)
    if not (link & PEER_LINK_EVER):
        return "peer:--"                       # 從未收過對端（無副板 / 鏈路未起）
    pstate = (FSM_NAMES[p["peer_fsm_state"]] if p["peer_fsm_state"] < len(FSM_NAMES)
              else f"?{p['peer_fsm_state']}")
    pflags = "|".join(name for bit, name in FLAG_NAMES if p["peer_flags"] & bit) or "-"
    tags = ["FRESH" if (link & PEER_LINK_FRESH) else "STALE"]
    if link & PEER_LINK_LOST:   tags.append("LOST")
    if link & PEER_LINK_DESYNC: tags.append("DESYNC")
    return (f"peer:{pstate} pbaro={p.get('peer_baro_cm', 0)/100.0:.1f}m "
            f"pvf=({p.get('peer_vf_h_cm', 0)/100.0:.1f}m,{p.get('peer_vf_v_cms', 0)/100.0:+.1f}m/s) "
            f"[{pflags}] {'|'.join(tags)}")


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


class Stats:
    def __init__(self):
        self.ok = 0
        self.crc_err = 0
        self.lost = 0
        self.last_seq = None

    def feed_seq(self, seq: int):
        if self.last_seq is not None:
            gap = (seq - self.last_seq - 1) & 0xFF
            self.lost += gap
        self.last_seq = seq

    def summary(self) -> str:
        total = self.ok + self.lost
        rate = (100.0 * self.lost / total) if total else 0.0
        return f"封包 OK={self.ok}  丟包={self.lost} ({rate:.1f}%)  CRC錯誤={self.crc_err}"


def decode_packet(raw: bytes) -> dict:
    vals = struct.unpack(_STRUCT_FMT, raw)
    return dict(zip(_FIELDS, vals))


def fmt_human(p: dict) -> str:
    fsm = FSM_NAMES[p["fsm_state"]] if p["fsm_state"] < len(FSM_NAMES) else f"?{p['fsm_state']}"
    flags = "|".join(name for bit, name in FLAG_NAMES if p["flags"] & bit) or "-"
    gps = (f"{p['gps_lat_1e6']/1e6:.6f},{p['gps_lon_1e6']/1e6:.6f} alt={p['gps_alt_m']}m "
           f"sats={p['gps_sats']}" if p["gps_fix"] else "no-fix")
    health = ""
    if p["health_bits"] or p["sensor_bits"]:
        health = f" !ekf=0x{p['health_bits']:02X} !sens=0x{p['sensor_bits']:02X}"
    arm_block = ""
    _af = p.get("arm_flags", 0)
    if _af & TELEM_ARM_BLOCKED_FLASH_POOL:
        arm_block = " [ARM BLOCKED: flash pool not ready]"
    elif _af & TELEM_ARM_NEED_ERASE:
        arm_block = " [FLASH NOT ERASED: run `flash erase` before ARM]"
    vf = f"vf=({p.get('vf_pos_z_cm', 0)/100.0:.2f}m,{p.get('vf_vel_z_cms', 0)/100.0:+.2f}m/s)"
    # 滾動極值：火箭端全速率追蹤，下鏈 ~2Hz 抓不到真峰值，這三個才是可信數字
    mx = (f"MAX=({p.get('max_alt_m', 0)}m,{p.get('max_vel_ms', 0):+d}m/s,"
          f"{p.get('max_acc_cg', 0)/100.0:.1f}g)")
    # 實際開傘高度；-32768 是「未開傘」哨兵（telemetry.h TELEM_DEPLOY_ALT_NA），不是 -32768 m
    def _dalt(v):
        return "--" if v == TELEM_DEPLOY_ALT_NA else f"{v}m"
    dep = (f"deploy=(D:{_dalt(p.get('drogue_alt_m', TELEM_DEPLOY_ALT_NA))},"
           f"M:{_dalt(p.get('main_alt_m', TELEM_DEPLOY_ALT_NA))})")
    line = (f"#{p['seq']:3d} t={p['tick_ms']/1000.0:8.2f}s {fsm:7s} "
            f"h={p['ekf_pos_z_cm']/100.0:7.2f}m v={p['ekf_vel_z_cms']/100.0:7.2f}m/s "
            f"baro={p['baro_alt_cm']/100.0:8.2f}m {vf} {mx} {dep} bat={p['bat_mv']/1000.0:.2f}V "
            f"[{flags}]{health}{arm_block} gps={gps} | {fmt_peer(p)}")
    if elevator_test_active(p):
        line += f"  🚨🚨🚨 ELEVATOR TEST PROFILE ACTIVE ({elevator_test_who(p)}) — DO NOT LAUNCH FOR REAL FLIGHT 🚨🚨🚨"
    return line


def csv_header() -> str:
    return ",".join(_FIELDS[2:])  # 跳過 sync bytes


def fmt_csv(p: dict) -> str:
    return ",".join(str(p[f]) for f in _FIELDS[2:])


def scan_stream(read_chunk, stats: Stats, on_packet):
    """read_chunk() → bytes(可空)；逐位元組掃 sync,CRC 過才算封包,失敗滑動 1 byte 重同步。"""
    buf = bytearray()
    while True:
        chunk = read_chunk()
        if chunk is None:
            break
        buf.extend(chunk)
        while len(buf) >= PACKET_SIZE:
            if buf[0] != SYNC0 or buf[1] != SYNC1:
                del buf[0]
                continue
            raw = bytes(buf[:PACKET_SIZE])
            crc_recv = raw[PACKET_SIZE - 2] | (raw[PACKET_SIZE - 1] << 8)
            if crc16_ccitt_false(raw[:PACKET_SIZE - 2]) != crc_recv:
                stats.crc_err += 1
                del buf[0]      # 滑動重同步(可能是假 sync 或封包損毀)
                continue
            p = decode_packet(raw)
            stats.ok += 1
            stats.feed_seq(p["seq"])
            on_packet(p)
            del buf[:PACKET_SIZE]


def make_selftest_packet(seq=7) -> bytes:
    vals = {f: 0 for f in _FIELDS}
    vals.update(sync0=SYNC0, sync1=SYNC1, seq=seq, fsm_state=3, tick_ms=123456,
                ekf_pos_z_cm=25032, ekf_vel_z_cms=-1500, baro_alt_cm=24890,
                bat_mv=11850, flags=0x41,
                health_bits=0x01, sensor_bits=0x04, gps_fix=0,
                vf_pos_z_cm=24950, vf_vel_z_cms=-1480,
                hg_mag_cg=980,
                # 滾動極值 + 開傘高度：副傘已在 251m 開、主傘尚未開（哨兵），
                # 這樣一筆就同時驗到「有值」與「NA 哨兵顯示成 --」兩條路徑。
                max_alt_m=251, max_vel_ms=118, max_acc_cg=1430,
                drogue_alt_m=251, main_alt_m=TELEM_DEPLOY_ALT_NA,
                peer_fsm_state=3, peer_flags=0x01,
                peer_baro_cm=24750, peer_link=0x03,
                peer_vf_h_cm=24760, peer_vf_v_cms=-1470)
    raw = struct.pack(_STRUCT_FMT, *[vals[f] for f in _FIELDS])
    raw = raw[:PACKET_SIZE - 2] + struct.pack("<H", crc16_ccitt_false(raw[:PACKET_SIZE - 2]))
    return raw


def main():
    ap = argparse.ArgumentParser(description="RocketCom 遙測解碼器")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial 埠(如 /dev/cu.usbserial-110)")
    src.add_argument("--file", help="二進制 dump 檔")
    src.add_argument("--selftest", action="store_true", help="合成封包自我測試")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--csv", help="同時輸出 CSV 檔")
    args = ap.parse_args()

    stats = Stats()
    csv_fh = open(args.csv, "w", encoding="utf-8") if args.csv else None
    if csv_fh:
        csv_fh.write(csv_header() + "\n")

    def on_packet(p):
        print(fmt_human(p))
        if csv_fh:
            csv_fh.write(fmt_csv(p) + "\n")

    try:
        if args.selftest:
            noise = b"\x00\xa5z31"                       # 假 sync / 垃圾前綴
            stream = noise + make_selftest_packet(7) + b"\xff" * 3 + make_selftest_packet(9)
            it = iter([stream])
            scan_stream(lambda: next(it, None), stats, on_packet)
            ok = stats.ok == 2 and stats.lost == 1 and stats.crc_err == 0
            print(stats.summary())
            print("SELFTEST", "PASS（含 1 個刻意 seq gap）" if ok else "FAIL")
            sys.exit(0 if ok else 1)
        elif args.file:
            with open(args.file, "rb") as fh:
                chunks = iter(lambda: fh.read(4096) or None, None)
                scan_stream(lambda: next(chunks, None), stats, on_packet)
        else:
            import serial  # pyserial,僅 --port 時需要
            with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
                print(f"# listening {args.port} @ {args.baud} …Ctrl-C 結束")
                while True:
                    yield_chunk = ser.read(4096)
                    if yield_chunk:
                        buf_iter = iter([yield_chunk])
                        scan_stream(lambda: next(buf_iter, None), stats, on_packet)
                    else:
                        time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_fh:
            csv_fh.close()
        print("\n" + stats.summary())


if __name__ == "__main__":
    main()
