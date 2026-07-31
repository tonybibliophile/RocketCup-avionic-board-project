#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
raw_log_decoder.py — RocketCom 全資料來源二進位原始記錄解碼器
===========================================================================
一支工具涵蓋主/副航電、地面站兩塊板子「所有會落地成檔案」的原始記錄格式，
逐格式標注目前是否為「完整原始資料」（含 firmware header 內明確寫出的 byte
offset 註解為準，並以實際編譯 sizeof 驗證過）。

┌─────────────┬────────────────────┬──────────┬────────────────────────────┐
│ 來源          │ 檔案 / 存放位置        │ 格式        │ 是否完整                       │
├─────────────┼────────────────────┼──────────┼────────────────────────────┤
│ 主/副航電 Flash │ 板載 W25Q128（僅能經    │ FlashRing   │ 完整（除 reserved[28] 未定義  │
│              │ `flash export` 印成    │ Packet_t    │ 欄位外）；但此格式本身「從沒   │
│              │ CSV 文字流，或用外部     │ (128B)      │ 存過磁力計」——不是漏印，是     │
│              │ SPI 燒錄器整顆拉一份     │             │ 結構本來就沒這 3 個欄位。      │
│              │ raw .bin）           │             │                            │
│ 主/副航電 SD   │ HIL_xxx.CSV        │ 純文字 CSV  │ 100Hz 節流版；同樣沒有磁力計、  │
│              │                    │             │ 電池以外的健康位元。            │
│ 主/副航電 SD   │ IMU_xxx.BIN        │ ImuRaw      │ 完整（逐顆 1000Hz 陀螺/400Hz   │
│              │                    │ Record_t    │ 加速度原始 LSB，remap 前）；    │
│              │                    │ (128B)      │ 同樣不含磁力計（MMC5983 不在   │
│              │                    │             │ 高速 IMU 取樣管線內）。         │
│ 地面站 Flash   │ 板載 W25Q128（僅能經    │ GsLogRecord │ 「下鏈收到的」全部欄位逐一保留， │
│              │ `flash export` 印成    │ _t (137B)   │ 但下鏈本身自 2026-07-30 起就   │
│              │ CSV，或整顆 raw dump）  │             │ 不再送 mag 三軸/原始氣壓 Pa，   │
│              │                    │             │ 高G 也只送模長（見下方說明）。   │
│ 地面站 SD    │ GSLOG*.CSV           │ 純文字 CSV  │ 不完整：GsLog_FormatCsvRow()   │
│              │                    │             │ 這個轉換函式沒收錄 gyro；       │
│              │                    │             │ 完整欄位請解 GSRAW*.BIN。       │
│ 地面站 SD    │ GSRAW*.BIN（本次新增）  │ GsLogRecord │ 完整，與 Flash 內容逐 byte 相同 │
│              │                    │ _t (137B)   │（同一個 struct 直接落地）。      │
└─────────────┴────────────────────┴──────────┴────────────────────────────┘

★封包格式是「固定長度」契約，差 1 byte 就 100% 解不出來（接收端對到 sync 後硬數 N
  bytes 才算 CRC）。本專案有四份重複的 TelemetryPacket_t 定義必須同時同步：
    firmware telemetry.h（唯一真相）／telemetry_decoder.py／本檔／sensor_error_analyzer.py
  由 tests/test_telemetry.c 的 offset 表機器鎖定 firmware 側；Python 三份靠 assert
  calcsize 自我檢查，但「欄位順序寫錯而長度剛好相同」assert 抓不到，改動時務必逐欄比對。
  2026-07-30 起：TelemetryPacket_t = 99 bytes、GsLogRecord_t = 137 bytes。

用法：
    python3 raw_log_decoder.py --file GSRAW003.BIN                    # 自動判斷格式
    python3 raw_log_decoder.py --file GSRAW003.BIN --csv out.csv      # 解出「所有」欄位（含 CSV 沒有的 gyro/hg 模長）
    python3 raw_log_decoder.py --file IMU_007.BIN --type imu_raw --stats
    python3 raw_log_decoder.py --file avionics_flash_dump.bin --type flash_ring --csv out.csv
    python3 raw_log_decoder.py --report                              # 只印上面這張表，不解檔案
    python3 raw_log_decoder.py --selftest
"""
import argparse
import struct
import sys

REPORT_TEXT = __doc__.split("=========================================================================\n", 1)[1] \
    .split("用法：", 1)[0].strip()


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF，與 firmware crc16.h 同參數。
    黃金向量 "123456789" -> 0x29B1（與 tests/test_telemetry.c 同一組）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def _fmt_field_count(fmt: str) -> int:
    n, digits = 0, ""
    for ch in fmt:
        if ch == "<":
            continue
        if ch == "s":
            n += 1
            digits = ""
            continue
        if ch.isdigit():
            digits += ch
        else:
            n += int(digits) if digits else 1
            digits = ""
    return n


# ===========================================================================
# GS_LOG — 地面站 GsLogRecord_t（gs_log.h）。133 bytes packed little-endian。
# SD 上為 GSRAW*.BIN（見 ground_station.c gs_sd_write_raw()）；
# 若日後用外部 SPI 燒錄器整顆讀出地面站板 Flash，其 Ring 區同一格式可直接沿用
# 本解碼器（gs_flash_append() 寫入的 bytes 與 gs_sd_write_raw() 完全相同）。
# 欄位順序/大小逐一比對 gs_log.h GsLogRecord_t + telemetry.h TelemetryPacket_t，
# 由 tests/test_gs_log.c + tests/test_telemetry.c 機器鎖定（sizeof(TelemetryPacket_t)=99）。
# ===========================================================================
GS_LOG_MAGIC0, GS_LOG_MAGIC1 = ord('G'), ord('S')
GS_LOG_RECORD_SIZE = 137   # 36B 表頭 + 99B TelemetryPacket_t + 2B 紀錄 CRC

_GS_LOG_HEADER_FIELDS = [
    "magic0", "magic1", "link_source", "_rsv", "rssi_dbm", "snr_cb",
    "rx_tick_ms", "rx_utc_ms", "aligned_utc_ms", "offset_ms",
    "gs_lat_1e6", "gs_lon_1e6", "gs_alt_m", "gs_sats", "gs_fix",
]
_GS_LOG_HEADER_FMT = "BBBBhhIIIiiihBB"

# 內嵌的完整下行遙測封包（telemetry.h TelemetryPacket_t，99 bytes）。
# gyro_* 是 [GS_PKT] 文字行與 GSLOG*.CSV 都沒印出來的部分，只有這裡（以及地面站
# Flash 本身）保留。mag 三軸/原始氣壓 Pa/高G 三軸自 2026-07-30 起已不在下鏈裡，
# 想要那些只能拿回航電板本身的 Flash ring（FlashRingPacket_t）。
_GS_LOG_PKT_FIELDS = [
    "sync0", "sync1", "seq", "fsm_state", "tick_ms",
    "ekf_pos_z_cm", "ekf_vel_z_cms", "ekf_q0", "ekf_q1", "ekf_q2", "ekf_q3",
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
    "pkt_crc16",   # TelemetryPacket_t 自己的 CRC（下行時已驗過），與外層 rec_crc16 是兩回事
]
_GS_LOG_PKT_FMT = "4BI2i4hi7h2ih2B2H3B2iHhH2h2BiB2i3BH"

GS_LOG_FIELDS = _GS_LOG_HEADER_FIELDS + _GS_LOG_PKT_FIELDS + ["rec_crc16"]
GS_LOG_FMT = "<" + _GS_LOG_HEADER_FMT + _GS_LOG_PKT_FMT + "H"
assert struct.calcsize(GS_LOG_FMT) == GS_LOG_RECORD_SIZE, struct.calcsize(GS_LOG_FMT)
assert _fmt_field_count(GS_LOG_FMT) == len(GS_LOG_FIELDS), \
    (_fmt_field_count(GS_LOG_FMT), len(GS_LOG_FIELDS))


def decode_gs_log_record(raw: bytes) -> dict:
    vals = struct.unpack(GS_LOG_FMT, raw)
    return dict(zip(GS_LOG_FIELDS, vals))


def gs_log_csv_header() -> str:
    return ",".join(f for f in GS_LOG_FIELDS if f != "_rsv")


def gs_log_csv_row(r: dict) -> str:
    return ",".join(str(r[f]) for f in GS_LOG_FIELDS if f != "_rsv")


# ===========================================================================
# FLASH_RING — 主/副航電飛行資料環形緩衝區封包（w25qxx.h FlashRingPacket_t）。
# 128 bytes packed little-endian，magic 0xAA,0x55。目前韌體只有 `flash export`
# 把它轉成 CSV 文字印出（main.c ~line 3559），沒有現成的 raw .bin 匯出路徑；
# 若之後用外部 SPI 燒錄器整顆拉一份原始 dump，或另外加一個仿照 gs_sd_write_raw
# 的逐筆二進位落地，可直接用本解碼器解。
# ★ 這個格式本身沒有 mag_x/y/z 欄位——MMC5983 磁力計 raw 從未進過飛行記錄，
#   不是本工具漏解，是 firmware 端這個 struct 從一開始就沒留這三個欄位。
# ===========================================================================
FLASH_RING_MAGIC0, FLASH_RING_MAGIC1 = 0xAA, 0x55
FLASH_RING_RECORD_SIZE = 128

FLASH_RING_FIELDS = [
    "magic0", "magic1", "seq", "flight_id", "tick_ms", "fsm_state", "flags", "bat_voltage_mv",
    "bmi_ax", "bmi_ay", "bmi_az", "bmi_gx", "bmi_gy", "bmi_gz",
    "adxl_x", "adxl_y", "adxl_z",
    "baro_temp_c_x100", "baro_press_pa", "baro_alt_cm",
    "ekf_pos_z_cm", "ekf_vel_z_cms",
    "ekf_q0", "ekf_q1", "ekf_q2", "ekf_q3",
    "gps_lat", "gps_lon", "gps_alt_m", "gps_spd_cms", "gps_sats", "gps_fix",
    "reserved0", "flight_tick_ms",
    "max_alt_m", "max_vel_ms", "max_acc_cg", "drogue_alt_m", "main_alt_m",
    "reserved", "crc16",
]
# flight_tick_ms（起飛後經過時間 ms）自 reserved 區切出，封包總長仍是 128B——
# tick_ms 是「開機以來」的絕對 tick，重啟後歸零，跨重啟的時間軸只能看 flight_tick_ms。
FLASH_RING_FMT = ("<2BHIIBBH" + "hhhhhh" + "hhh" + "h" + "I" + "iii" + "ffff" + "ii" + "hh" + "BB"
                  + "2s" + "I" + "HhHhh" + "28s" + "H")
assert struct.calcsize(FLASH_RING_FMT) == FLASH_RING_RECORD_SIZE, struct.calcsize(FLASH_RING_FMT)
assert _fmt_field_count(FLASH_RING_FMT) == len(FLASH_RING_FIELDS), \
    (_fmt_field_count(FLASH_RING_FMT), len(FLASH_RING_FIELDS))


def decode_flash_ring_record(raw: bytes) -> dict:
    vals = struct.unpack(FLASH_RING_FMT, raw)
    d = dict(zip(FLASH_RING_FIELDS, vals))
    d["reserved0"] = d["reserved0"].hex()
    d["reserved"] = d["reserved"].hex()
    return d


def flash_ring_csv_header() -> str:
    return ",".join(FLASH_RING_FIELDS)


def flash_ring_csv_row(r: dict) -> str:
    return ",".join(str(r[f]) for f in FLASH_RING_FIELDS)


# ===========================================================================
# IMU_RAW — 主/副航電 SD 卡逐顆原始 IMU 記錄（imu_raw_log.h ImuRawRecord_t）。
# 128 bytes packed little-endian，magic 'R','I'。與既有 tools/imu_raw_decoder.py
# 為同一份格式定義（該檔已用 --selftest 驗證過，這裡為求本檔獨立可用而複製一份
# 同款格式字串，兩邊都源自 imu_raw_log.h 的欄位 offset 註解，非各自猜測）。
# 完整——逐顆 1000Hz 陀螺 / 400Hz 加速度原始 LSB，sensor frame，remap 前。
# ===========================================================================
IMU_RAW_MAGIC0, IMU_RAW_MAGIC1 = ord('R'), ord('I')
IMU_RAW_RECORD_SIZE = 128

IMU_RAW_FIELDS = [
    "magic0", "magic1", "ver", "n_gyro", "n_acc", "flags", "seq", "t_cyc",
    "fsm_state", "_rsv", "baro_temp_c_x100", "baro_press_pa",
] + [f"gyro_{i}_{ax}" for i in range(10) for ax in ("x", "y", "z")] \
  + [f"acc_{i}_{ax}" for i in range(4) for ax in ("x", "y", "z")] \
  + [f"gyro_dt_q_{i}" for i in range(10)] \
  + ["acc_t_off_q", "crc16"]
IMU_RAW_FMT = "<BBBBBBHIBBhI30h12h10HhH"
assert struct.calcsize(IMU_RAW_FMT) == IMU_RAW_RECORD_SIZE, struct.calcsize(IMU_RAW_FMT)
assert _fmt_field_count(IMU_RAW_FMT) == len(IMU_RAW_FIELDS), \
    (_fmt_field_count(IMU_RAW_FMT), len(IMU_RAW_FIELDS))


def decode_imu_raw_record(raw: bytes) -> dict:
    vals = struct.unpack(IMU_RAW_FMT, raw)
    return dict(zip(IMU_RAW_FIELDS, vals))


def imu_raw_csv_header() -> str:
    return ",".join(f for f in IMU_RAW_FIELDS if f != "_rsv")


def imu_raw_csv_row(r: dict) -> str:
    return ",".join(str(r[f]) for f in IMU_RAW_FIELDS if f != "_rsv")


# ===========================================================================
# 共用：固定長度網格掃描 + CRC 驗證 + 逐 byte 重同步（三種格式共用同一套邏輯，
# 差別只在 record_size / magic / crc 覆蓋範圍 / decode 函式）。
# ===========================================================================
_FORMATS = {
    "gs_log": dict(size=GS_LOG_RECORD_SIZE, magic=(GS_LOG_MAGIC0, GS_LOG_MAGIC1),
                    decode=decode_gs_log_record, crc_field="rec_crc16",
                    csv_header=gs_log_csv_header, csv_row=gs_log_csv_row,
                    label="地面站 GsLogRecord_t（SD GSRAW*.BIN / Flash raw dump）"),
    "flash_ring": dict(size=FLASH_RING_RECORD_SIZE, magic=(FLASH_RING_MAGIC0, FLASH_RING_MAGIC1),
                        decode=decode_flash_ring_record, crc_field="crc16",
                        csv_header=flash_ring_csv_header, csv_row=flash_ring_csv_row,
                        label="主/副航電 FlashRingPacket_t（Flash raw dump）"),
    "imu_raw": dict(size=IMU_RAW_RECORD_SIZE, magic=(IMU_RAW_MAGIC0, IMU_RAW_MAGIC1),
                     decode=decode_imu_raw_record, crc_field="crc16",
                     csv_header=imu_raw_csv_header, csv_row=imu_raw_csv_row,
                     label="主/副航電 ImuRawRecord_t（SD IMU_xxx.BIN）"),
}


def scan_records(buf: bytes, kind: str):
    """回傳 (records, ok, resync, crc_err)。逐 record_size 網格掃描，magic/CRC 過才收。"""
    spec = _FORMATS[kind]
    size, (m0, m1), decode, crc_field = spec["size"], spec["magic"], spec["decode"], spec["crc_field"]
    records = []
    ok = resync = crc_err = 0
    i, n = 0, len(buf)
    last_pct = -1
    while i + size <= n:
        if n > 0:
            pct = int((i / n) * 100)
            if pct != last_pct and (pct % 5 == 0 or pct == 100):
                last_pct = pct
                sys.stdout.write(f"\r[PROGRESS] ⏳ 掃描原始紀錄進度: {pct:3d}% ({i / (1024*1024):.1f} / {n / (1024*1024):.1f} MB)")
                sys.stdout.flush()

        chunk = buf[i:i + size]
        if chunk[0] == m0 and chunk[1] == m1:
            crc_calc = crc16_ccitt_false(chunk[:size - 2])
            crc_recv = chunk[size - 2] | (chunk[size - 1] << 8)
            if crc_calc == crc_recv:
                rec = decode(chunk)
                assert rec[crc_field] == crc_recv
                records.append(rec)
                ok += 1
                i += size
                continue
            crc_err += 1
        resync += 1
        i += 1

    if n > 0:
        sys.stdout.write("\r[PROGRESS] ✅ 原始紀錄掃描解碼完成 (100%)\n")
        sys.stdout.flush()
    return records, ok, resync, crc_err


def sniff_type(buf: bytes) -> str:
    """依前 2 bytes magic 猜格式；三種格式的 magic 彼此不重疊，可唯一判斷。"""
    if len(buf) < 2:
        raise ValueError("檔案太短，無法判斷格式")
    m0, m1 = buf[0], buf[1]
    for kind, spec in _FORMATS.items():
        if (m0, m1) == spec["magic"]:
            return kind
    raise ValueError(
        f"前 2 bytes = 0x{m0:02X},0x{m1:02X}，不符合任何已知格式"
        f"（gs_log=0x47,0x53 'GS'；flash_ring=0xAA,0x55；imu_raw=0x52,0x49 'RI'）。"
        f" 請用 --type 明確指定，或確認檔案是否被截斷/非本工具支援的格式。"
    )


def print_stats(kind: str, records, resync: int, crc_err: int):
    spec = _FORMATS[kind]
    print(f"[STATS] 格式={kind}（{spec['label']}） 記錄數={len(records)} "
          f"重同步位元組={resync} CRC錯誤={crc_err}")
    if not records:
        return
    if kind == "gs_log":
        fsm_counts = {}
        for r in records:
            fsm_counts[r["fsm_state"]] = fsm_counts.get(r["fsm_state"], 0) + 1
        print(f"[STATS] fsm_state 分布：{fsm_counts}")
        print(f"[STATS] 第一筆 rx_tick_ms={records[0]['rx_tick_ms']} "
              f"最後一筆={records[-1]['rx_tick_ms']}")
    elif kind == "flash_ring":
        seqs = [r["seq"] for r in records]
        gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFFFF) != 1)
        print(f"[STATS] seq 不連續次數={gaps}（flight_id 變化代表跨飛行批次）")
    elif kind == "imu_raw":
        seqs = [r["seq"] for r in records]
        gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFFFF) != 1)
        print(f"[STATS] seq 不連續次數={gaps}")


def make_selftest_records():
    """三種格式各造一筆已知輸入，驗證 pack/decode/CRC 往返一致與欄位數對齊。"""
    ok_all = True

    # gs_log
    vals = {f: 0 for f in GS_LOG_FIELDS}
    vals.update(magic0=GS_LOG_MAGIC0, magic1=GS_LOG_MAGIC1, link_source=1,
                rssi_dbm=-72, snr_cb=45, rx_tick_ms=123456, seq=7, fsm_state=3,
                ekf_pos_z_cm=25000, gyro_x_dps=150, hg_mag_cg=980,
                max_alt_m=1240, max_vel_ms=215, max_acc_cg=1450,
                drogue_alt_m=1238, main_alt_m=305)
    body = struct.pack(GS_LOG_FMT, *[vals[f] for f in GS_LOG_FIELDS])
    body = body[:-2] + struct.pack("<H", crc16_ccitt_false(body[:-2]))
    records, ok, resync, crc_err = scan_records(body, "gs_log")
    ok_all &= (ok == 1 and resync == 0 and crc_err == 0 and records[0]["gyro_x_dps"] == 150
               and records[0]["hg_mag_cg"] == 980
               and records[0]["max_alt_m"] == 1240
               and records[0]["max_vel_ms"] == 215
               and records[0]["max_acc_cg"] == 1450
               and records[0]["drogue_alt_m"] == 1238
               and records[0]["main_alt_m"] == 305)
    print("gs_log selftest:", "PASS" if ok_all else "FAIL")

    # flash_ring
    vals2 = {f: 0 for f in FLASH_RING_FIELDS}
    # reserved0/reserved 是 "2s"/"38s" 位元組欄位（不是 int），且長度必須精確吻合，
    # 否則 struct.pack 直接拋 error —— 這兩個先前填成 int 0 / 44 bytes，--selftest 從沒跑過綠。
    vals2.update(magic0=FLASH_RING_MAGIC0, magic1=FLASH_RING_MAGIC1, seq=9, flight_id=2,
                 bmi_ax=1000, ekf_q0=1.0, gps_lat=250427000,
                 reserved0=b"\x00" * 2, reserved=b"\x00" * 28)
    body2 = struct.pack(FLASH_RING_FMT, *[vals2[f] for f in FLASH_RING_FIELDS])
    body2 = body2[:-2] + struct.pack("<H", crc16_ccitt_false(body2[:-2]))
    records2, ok2, resync2, crc_err2 = scan_records(body2, "flash_ring")
    ok2_pass = (ok2 == 1 and resync2 == 0 and crc_err2 == 0 and records2[0]["bmi_ax"] == 1000)
    print("flash_ring selftest:", "PASS" if ok2_pass else "FAIL")

    # imu_raw
    vals3 = {f: 0 for f in IMU_RAW_FIELDS}
    vals3.update(magic0=IMU_RAW_MAGIC0, magic1=IMU_RAW_MAGIC1, ver=1, n_gyro=10, n_acc=4,
                 seq=3, t_cyc=999, gyro_1_y=-321)
    body3 = struct.pack(IMU_RAW_FMT, *[vals3[f] for f in IMU_RAW_FIELDS])
    body3 = body3[:-2] + struct.pack("<H", crc16_ccitt_false(body3[:-2]))
    records3, ok3, resync3, crc_err3 = scan_records(body3, "imu_raw")
    ok3_pass = (ok3 == 1 and resync3 == 0 and crc_err3 == 0 and records3[0]["gyro_1_y"] == -321)
    print("imu_raw selftest:", "PASS" if ok3_pass else "FAIL")

    return ok_all and ok2_pass and ok3_pass


def main():
    ap = argparse.ArgumentParser(description="RocketCom 全資料來源二進位原始記錄解碼器")
    ap.add_argument("--file", help="要解碼的二進位檔（GSRAW*.BIN / IMU_*.BIN / flash raw dump）")
    ap.add_argument("--type", choices=["auto", "gs_log", "flash_ring", "imu_raw"], default="auto")
    ap.add_argument("--csv", help="輸出 CSV 路徑（省略則印前幾列到終端機）")
    ap.add_argument("--stats", action="store_true", help="只印統計，不輸出 CSV")
    ap.add_argument("--report", action="store_true", help="印出各資料來源格式/完整性對照表，不解檔案")
    ap.add_argument("--selftest", action="store_true", help="三種格式的 pack/decode/CRC 往返自我測試")
    args = ap.parse_args()

    if args.report:
        print(REPORT_TEXT)
        return

    if args.selftest:
        sys.exit(0 if make_selftest_records() else 1)

    if not args.file:
        try:
            from file_selector import select_input_file
            args.file = select_input_file(title="請選擇原始記錄檔 (.BIN)", extensions=[".bin", ".hex"])
        except Exception as e:
            print(f"[WARNING] 無法啟動互動式檔案選擇器: {e}")

    if not args.file:
        ap.error("需要 --file（或用 --report / --selftest）")

    with open(args.file, "rb") as f:
        buf = f.read()

    kind = args.type
    if kind == "auto":
        kind = sniff_type(buf)
        print(f"[AUTO] 判斷格式為 {kind}（{_FORMATS[kind]['label']}）")

    records, ok, resync, crc_err = scan_records(buf, kind)
    print(f"[DECODE] {args.file}: {len(buf)} bytes -> {ok} 筆有效記錄 "
          f"（重同步 {resync} bytes，CRC 錯誤 {crc_err} 筆）")
    print_stats(kind, records, resync, crc_err)

    if not args.stats:
        spec = _FORMATS[kind]
        header = spec["csv_header"]()
        rows = [header] + [spec["csv_row"](r) for r in records]
        if args.csv:
            with open(args.csv, "w") as f:
                f.write("\n".join(rows) + "\n")
            print(f"[CSV] 已寫入 {args.csv}（{len(rows) - 1} 列，{len(header.split(','))} 欄）")
        else:
            for row in rows[:11]:
                print(row)
            if len(rows) > 11:
                print(f"... 共 {len(rows) - 1} 列，用 --csv 輸出完整檔案")


if __name__ == "__main__":
    main()
