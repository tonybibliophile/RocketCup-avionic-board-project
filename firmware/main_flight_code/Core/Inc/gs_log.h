/*
 * gs_log.h — 地面站紀錄單元 + CSV 格式化（純邏輯，host 可測）
 * ===========================================================================
 * 地面站每收到一筆有效下行遙測（TelemetryPacket_t），就組一筆 GsLogRecord_t：
 *   原始火箭封包（完整保留，後處理可解所有欄位）
 *   + 鏈路來源/品質（E22 433：開 REG3 bit7 才有逐包 RSSI、恆無 SNR；E80 920 有 RSSI/SNR）
 *   + 地面接收/對齊時間（rx tick、GPS 紀律當日 UTC、對齊後 UTC、rocket↔ground 偏移）
 *   + 地面站自身 GPS。
 *
 * 同一筆 GsLogRecord_t 三路落地：Flash（二進位 append，附 CRC）、SD（CSV 一列）、
 * USB-CDC（CSV 一列）。CSV 欄位即解碼契約，皆以縮放整數輸出（遵專案黃金法則：不用
 * %f；倍率見各欄註解），時間另附 HH:MM:SS.mmm 方便人讀。
 *
 * 本檔 + gs_log.c 不依賴 HAL / RTOS（僅 telemetry.h / crc16.h header-only），
 * 可由 tests/test_gs_log.c 驗證。SD/Flash/USB 的實體寫入在 HAL 端（ground_station.c /
 * gs_flash_log.c），不在本檔。
 */
#ifndef GS_LOG_H
#define GS_LOG_H

#include <stdint.h>
#include <stddef.h>
#include "telemetry.h"   /* TelemetryPacket_t */

#ifdef __cplusplus
extern "C" {
#endif

/* 鏈路來源 */
#define GS_LINK_433   0U   /* E22 433MHz（UART3 透傳） */
#define GS_LINK_920   1U   /* E80 920MHz（SX126x SPI3） */

/* 沒有該項資訊時填哨兵。433 恆無 SNR；433 的 RSSI 只在 E22 開啟 REG3 bit7
 * （lora_e22.c E22_RSSI_BYTE_EN）且成功剝到那個附加位元組時才有值。 */
#define GS_RSSI_NA   ((int16_t)-32768)
#define GS_SNR_NA    ((int16_t)-32768)

/* 地面站紀錄單元（packed，固定長度）。欄位順序即 Flash/解碼契約。 */
typedef struct __attribute__((packed)) {
    uint8_t  magic0;          /* 0x47 'G'（Flash 掃描標記） */
    uint8_t  magic1;          /* 0x53 'S' */
    uint8_t  link_source;     /* GS_LINK_433 / GS_LINK_920 */
    uint8_t  _rsv;            /* 對齊保留 */

    int16_t  rssi_dbm;        /* 920: SX126x RSSI(dBm)；433: GS_RSSI_NA */
    int16_t  snr_cb;          /* 920: SNR (0.25dB 單位之 SX126x 原值)；433: GS_SNR_NA */

    uint32_t rx_tick_ms;      /* 本機收包 tick (ms) */
    uint32_t rx_utc_ms;       /* 地面 GPS 紀律「當日 UTC 毫秒」（無 fix=0） */
    uint32_t aligned_utc_ms;  /* 火箭事件對齊後「當日 UTC 毫秒」（無對齊=0） */
    int32_t  offset_ms;       /* rocket↔ground 偏移（本機 tick − 火箭 tick_ms） */

    int32_t  gs_lat_1e6;      /* 地面站自身緯度 ×1e6 */
    int32_t  gs_lon_1e6;      /* 地面站自身經度 ×1e6 */
    int16_t  gs_alt_m;        /* 地面站自身海拔 (m) */
    uint8_t  gs_sats;         /* 地面站可見衛星數 */
    uint8_t  gs_fix;          /* 地面站定位有效 (0/1) */

    TelemetryPacket_t pkt;    /* 原始火箭遙測封包（79 bytes，完整保留） */

    uint16_t crc16;           /* CRC-16/CCITT-FALSE，覆蓋本紀錄前面所有位元組 */
} GsLogRecord_t;

#define GS_LOG_RECORD_SIZE  ((uint16_t)sizeof(GsLogRecord_t))
#define GS_LOG_MAGIC0  0x47U
#define GS_LOG_MAGIC1  0x53U

/* CSV 一行建議緩衝大小（含結尾 \r\n\0 餘量）。含本板 VF + 主/副協同 peer 欄
 * （baro/accel/VF 全量）後再放寬（表頭實測 411 bytes，512 留餘裕）。 */
#define GS_LOG_CSV_MAX  512U

/**
 * @brief 組一筆紀錄：填 magic、各欄位、原始封包，並算 CRC16。
 * @note  呼叫端先填好 link/rssi/snr/時間/地面 GPS 與 pkt（或用本函式一次帶入）。
 */
void GsLog_BuildRecord(GsLogRecord_t *rec,
                       uint8_t link_source, int16_t rssi_dbm, int16_t snr_cb,
                       uint32_t rx_tick_ms, uint32_t rx_utc_ms,
                       uint32_t aligned_utc_ms, int32_t offset_ms,
                       int32_t gs_lat_1e6, int32_t gs_lon_1e6,
                       int16_t gs_alt_m, uint8_t gs_sats, uint8_t gs_fix,
                       const TelemetryPacket_t *pkt);

/** @brief 驗證一筆紀錄的 magic 與 CRC。回傳 1=有效。 */
uint8_t GsLog_RecordValid(const GsLogRecord_t *rec);

/** @brief 寫 CSV 表頭到 out（含 \r\n）。回傳寫入字元數（不含 NUL）。 */
int GsLog_CsvHeader(char *out, size_t cap);

/** @brief 把一筆紀錄格式化為 CSV 一列（含 \r\n）。回傳寫入字元數（不含 NUL）。
 *  全部以縮放整數輸出（無 %f）；時間另附 HH:MM:SS.mmm 字串。 */
int GsLog_FormatCsvRow(char *out, size_t cap, const GsLogRecord_t *rec);

#ifdef __cplusplus
}
#endif

#endif /* GS_LOG_H */
