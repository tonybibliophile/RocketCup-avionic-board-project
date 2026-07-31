#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_raw_decoder.py — RocketCom SD 卡原始 IMU 二進位記錄解碼器（S6）

封包契約（與 firmware/main_flight_code/Core/Inc/imu_raw_log.h 同步，手動維護，
不像下行遙測有 tests/test_telemetry.c 機器鎖定——若韌體端改動 ImuRawRecord_t
版面，務必同步更新本檔的 _STRUCT_FMT/_FIELDS，並跑一次 --selftest 驗證往返一致）：
    128 bytes packed little-endian，sync 'R','I'，CRC-16/CCITT-FALSE
    （poly=0x1021, init=0xFFFF）覆蓋前 126 bytes。內容為 remap 前、sensor frame
    的原始 LSB（陀螺 ±2000dps→32768、加速度 ±24g→32768，同 bmi088.c 的刻度），
    供離線重跑 EKF_AttitudeUpdate/EKF_Predict/vf_* 忠實重現飛行當下的積分輸入
    （firmware 端只存每批次最後一顆到 flash/CSV，此檔才是逐顆全量）。

用法：
    python3 imu_raw_decoder.py --file IMU_007.BIN --csv out.csv
    python3 imu_raw_decoder.py --file IMU_007.BIN --stats
    python3 imu_raw_decoder.py --selftest
"""
import argparse
import struct
import sys

RECORD_SIZE = 128
MAGIC0, MAGIC1 = ord('R'), ord('I')

# 與 imu_raw_log.h 的 ImuRawRecord_t 欄位順序一一對應（little-endian）
_STRUCT_FMT = "<2sBBBBHIBBhI30h12h10HhH"
_FIELDS = [
    "magic", "ver", "n_gyro", "n_acc", "flags", "seq", "t_cyc",
    "fsm_state", "_rsv", "baro_temp_c_x100", "baro_press_pa",
] + [f"gyro_{i}_{ax}" for i in range(10) for ax in ("x", "y", "z")] \
  + [f"acc_{i}_{ax}" for i in range(4) for ax in ("x", "y", "z")] \
  + [f"gyro_dt_q_{i}" for i in range(10)] \
  + ["acc_t_off_q", "crc16"]
assert struct.calcsize(_STRUCT_FMT) == RECORD_SIZE, struct.calcsize(_STRUCT_FMT)

# --- 刻度常數（須與 bmi088.c 同步；BMI088 現行組態 gyro ±2000dps / accel ±24g） ---
GYRO_FULLSCALE_DPS = 2000.0
ACCEL_FULLSCALE_G = 24.0
INT16_SPAN = 32768.0

# CPU 時脈（STM32F407 PLL 組態，main.c MX_RCC_Init）：t_cyc/gyro_dt_q 皆為此時脈下的
# DWT->CYCCNT 週期數，換算秒數需要此值；若韌體時脈設定改變，此處須同步更新。
SYSTEM_CORE_CLOCK_HZ = 168_000_000


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def sensor_imu_to_body(sx, sy, sz):
    """對應 sensor_axis.h 的 sensor_imu_to_body()：bx=sy, by=sx, bz=-sz。"""
    return sy, sx, -sz


def decode_record(raw: bytes) -> dict:
    vals = struct.unpack(_STRUCT_FMT, raw)
    d = dict(zip(_FIELDS, vals))
    d["gyro_raw"] = [(d[f"gyro_{i}_x"], d[f"gyro_{i}_y"], d[f"gyro_{i}_z"]) for i in range(10)]
    d["acc_raw"] = [(d[f"acc_{i}_x"], d[f"acc_{i}_y"], d[f"acc_{i}_z"]) for i in range(4)]
    d["gyro_dt_q"] = [d[f"gyro_dt_q_{i}"] for i in range(10)]
    return d


def physical_gyro_body(raw_xyz):
    """原始 LSB(sensor frame) → 物理量 dps(body frame)，套用同一刻度+remap。"""
    sx, sy, sz = (v * GYRO_FULLSCALE_DPS / INT16_SPAN for v in raw_xyz)
    return sensor_imu_to_body(sx, sy, sz)


def physical_accel_body(raw_xyz):
    """原始 LSB(sensor frame) → 物理量 g(body frame)。"""
    sx, sy, sz = (v * ACCEL_FULLSCALE_G / INT16_SPAN for v in raw_xyz)
    return sensor_imu_to_body(sx, sy, sz)


def scan_records(buf: bytes):
    """逐 128B 掃描：magic+CRC 過才算一筆，失敗則逐 byte 滑動重新同步。"""
    records = []
    ok = resync = crc_err = 0
    i = 0
    n = len(buf)
    last_pct = -1
    while i + RECORD_SIZE <= n:
        if n > 0:
            pct = int((i / n) * 100)
            if pct != last_pct and (pct % 5 == 0 or pct == 100):
                last_pct = pct
                sys.stdout.write(f"\r[PROGRESS] ⏳ 掃描 IMU 二進位檔進度: {pct:3d}% ({i / (1024*1024):.1f} / {n / (1024*1024):.1f} MB)")
                sys.stdout.flush()

        chunk = buf[i:i + RECORD_SIZE]
        if chunk[0] == MAGIC0 and chunk[1] == MAGIC1:
            crc_calc = crc16_ccitt_false(chunk[:RECORD_SIZE - 2])
            crc_recv = chunk[RECORD_SIZE - 2] | (chunk[RECORD_SIZE - 1] << 8)
            if crc_calc == crc_recv:
                records.append(decode_record(chunk))
                ok += 1
                i += RECORD_SIZE
                continue
            crc_err += 1
        resync += 1
        i += 1  # 網格未對齊或 CRC 錯：逐 byte 滑動找下一個 'RI'

    if n > 0:
        sys.stdout.write("\r[PROGRESS] ✅ 二進位檔掃描解碼完成 (100%)\n")
        sys.stdout.flush()
    return records, ok, resync, crc_err


def reconstruct_time_s(records):
    """用 gyro_dt_q 逐顆累加重建絕對時間（秒），wrap-safe。"""
    t_s = 0.0
    out = []
    first = True
    for rec in records:
        per_sample = []
        for dq in rec["gyro_dt_q"]:
            if first:
                dt_s = 1.0 / 1000.0
                first = False
            else:
                dt_s = (dq * 8) / SYSTEM_CORE_CLOCK_HZ
            t_s += dt_s
            per_sample.append(t_s)
        out.append(per_sample)
    return out


def to_csv_rows(records):
    times = reconstruct_time_s(records)
    rows = ["t_s,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,has_new_baro,baro_press_pa,fsm_state,seq"]
    total = len(records)
    last_pct = -1
    for idx, (rec, t_list) in enumerate(zip(records, times)):
        if total > 0:
            pct = int((idx / total) * 100)
            if pct != last_pct and (pct % 10 == 0 or pct == 100):
                last_pct = pct
                sys.stdout.write(f"\r[PROGRESS] ⏳ 轉換 CSV 數據進度: {pct:3d}% ({idx} / {total} 筆)")
                sys.stdout.flush()

        for gi in range(10):
            t_s = t_list[gi]
            gx, gy, gz = physical_gyro_body(rec["gyro_raw"][gi])
            ai = (gi * 4) // 10  # 對應韌體 EKF 組裝迴圈的 ai = (j*EKF_ACC_PER_FRAME)/EKF_GYRO_PER_FRAME
            ax, ay, az = physical_accel_body(rec["acc_raw"][ai])
            has_baro = 1 if (gi == 0 and rec["baro_press_pa"] != 0) else 0
            rows.append(f"{t_s:.6f},{gx:.4f},{gy:.4f},{gz:.4f},{ax:.5f},{ay:.5f},{az:.5f},"
                        f"{has_baro},{rec['baro_press_pa']},{rec['fsm_state']},{rec['seq']}")

    if total > 0:
        sys.stdout.write("\r[PROGRESS] ✅ CSV 數據轉換完成 (100%)\n")
        sys.stdout.flush()
    return rows


def print_stats(records, resync, crc_err):
    if not records:
        print("[STATS] 無有效記錄")
        return
    seqs = [r["seq"] for r in records]
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFFFF) != 1)
    all_dt = [dq * 8 / SYSTEM_CORE_CLOCK_HZ * 1000.0
              for rec in records[1:] for dq in rec["gyro_dt_q"]]  # 跳過第一筆，避免名目值污染統計
    mean_dt = sum(all_dt) / len(all_dt) if all_dt else 0.0
    var = sum((x - mean_dt) ** 2 for x in all_dt) / len(all_dt) if all_dt else 0.0
    print(f"[STATS] 記錄數={len(records)} 丟包/重同步位元組={resync} CRC錯誤={crc_err} "
          f"seq不連續次數={gaps}")
    print(f"[STATS] 陀螺樣本間隔：均值={mean_dt:.4f}ms std={var ** 0.5:.4f}ms "
          f"（理論值 1.000ms @1000Hz；顯著偏離代表 TIM7 未如預期跑在 1000Hz）")


def make_selftest_record(seq: int) -> bytes:
    """建一筆已知輸入的 record，供 --selftest 驗證 pack/decode/CRC 往返一致。"""
    magic = bytes([MAGIC0, MAGIC1])
    gyro = [(100 * i, -50 * i, 25 * i) for i in range(10)]
    acc = [(1000, 2000, -3000 + i) for i in range(4)]
    gyro_dt_q = [21000 + i for i in range(10)]  # ~1ms @168MHz /8 附近
    flat_gyro = [v for xyz in gyro for v in xyz]
    flat_acc = [v for xyz in acc for v in xyz]
    body = struct.pack(
        _STRUCT_FMT,
        magic, 1, 10, 4, 0x08, seq, 123456789, 3, 0, 2512, 98412,
        *flat_gyro, *flat_acc, *gyro_dt_q, -17, 0  # 最後 0 為 crc16 佔位值，下面重算後覆蓋
    )
    crc = crc16_ccitt_false(body[:-2])
    return body[:-2] + struct.pack("<H", crc)


def main():
    ap = argparse.ArgumentParser(description="RocketCom SD 原始 IMU 二進位記錄解碼器")
    ap.add_argument("--file", help="IMU_xxx.BIN 檔案路徑")
    ap.add_argument("--csv", help="輸出 CSV 路徑（省略則印到 stdout 前幾行）")
    ap.add_argument("--stats", action="store_true", help="只印統計，不輸出 CSV")
    ap.add_argument("--selftest", action="store_true", help="Python 端 pack/decode/CRC 往返自我測試")
    args = ap.parse_args()

    if args.selftest:
        raw = make_selftest_record(7)
        assert len(raw) == RECORD_SIZE, f"selftest record size={len(raw)}"
        assert raw[0] == MAGIC0 and raw[1] == MAGIC1
        crc_calc = crc16_ccitt_false(raw[:RECORD_SIZE - 2])
        crc_recv = raw[RECORD_SIZE - 2] | (raw[RECORD_SIZE - 1] << 8)
        assert crc_calc == crc_recv, "CRC round-trip mismatch"
        rec = decode_record(raw)
        assert rec["seq"] == 7
        assert rec["gyro_raw"][1] == (100, -50, 25)
        assert rec["acc_raw"][2] == (1000, 2000, -2998)
        gx, gy, gz = physical_gyro_body(rec["gyro_raw"][1])
        assert abs(gx - (-50 * GYRO_FULLSCALE_DPS / INT16_SPAN)) < 1e-9
        records, ok, resync, crc_err = scan_records(raw)
        assert ok == 1 and resync == 0 and crc_err == 0
        print("SELFTEST PASS")
        return

    if not args.file:
        try:
            from file_selector import select_input_file
            args.file = select_input_file(title="請選擇 IMU 原始二進位檔 (.BIN)", extensions=[".bin"])
        except Exception as e:
            print(f"[WARNING] 無法啟動互動式檔案選擇器: {e}")

    if not args.file:
        ap.error("需要 --file（或使用 --selftest）")

    with open(args.file, "rb") as f:
        buf = f.read()

    records, ok, resync, crc_err = scan_records(buf)
    print(f"[DECODE] {args.file}: {len(buf)} bytes → {ok} 筆有效記錄 "
          f"（重同步 {resync} bytes，CRC 錯誤 {crc_err} 筆）")

    print_stats(records, resync, crc_err)

    if not args.stats:
        rows = to_csv_rows(records)
        if args.csv:
            with open(args.csv, "w") as f:
                f.write("\n".join(rows) + "\n")
            print(f"[CSV] 已寫入 {args.csv}（{len(rows) - 1} 列）")
        else:
            for row in rows[:11]:
                print(row)
            if len(rows) > 11:
                print(f"... 共 {len(rows) - 1} 列，用 --csv 輸出完整檔案")


if __name__ == "__main__":
    sys.exit(main())
