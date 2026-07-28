/*
 * telem_rx.h — 下行遙測「接收端」逐位元組解析（純邏輯，host 可測）
 * ===========================================================================
 * 地面站（ROLE_GROUND）收火箭下行的 TelemetryPacket_t（telemetry.h，sync 0xA5/0x5A，
 * 116 bytes，結尾 CRC-16/CCITT-FALSE 覆蓋前 114 bytes）。本檔為其解碼端，與
 * link_proto.c::LinkRx_Feed() 同一套「sync 對齊 + CRC 校驗 + 自動重新同步」狀態機，
 * 只是封包型別換成 TelemetryPacket_t、sync 換成下行遙測的 0xA5/0x5A。
 *
 * 等同 GroundStation/telemetry_decoder.py::scan_stream() 的 C 版（同 sync / 同 CRC 參數）。
 * 純 header-only（仿 gps_parse.h / crc16.h），不依賴 HAL / RTOS，可由
 * tests/test_telem_rx.c 在 host 上以情境鎖定行為。E22(433) 透傳 byte 流與 E80(920)
 * 讀出的 FIFO payload 皆可餵入同一解析器。位元組順序假設 little-endian（STM32 與 host 皆是）。
 */
#ifndef TELEM_RX_H
#define TELEM_RX_H

#include <stdint.h>
#include <string.h>
#include "telemetry.h"   /* TelemetryPacket_t, TELEM_SYNC0/1, TELEM_PACKET_SIZE */
#include "crc16.h"       /* crc16_ccitt_false（與 TX/Flash ring 同一實作） */

#ifdef __cplusplus
extern "C" {
#endif

/* 逐位元組接收狀態機 + 診斷統計 */
typedef struct {
    uint8_t  buf[sizeof(TelemetryPacket_t)];
    uint8_t  idx;       /* 已存入 buf 的位元組數（0 = 等待 sync0） */
    uint32_t ok;        /* CRC 通過、成功交出的封包數 */
    uint32_t crc_err;   /* 湊滿一筆但 CRC 不符（疑似空中誤碼） */
    uint32_t resync;    /* sync1 階段遇到非預期位元組、退回重找 sync 的次數 */
} TelemRx_t;

static inline void TelemRx_Init(TelemRx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
}

/**
 * @brief CRC 失敗後的重新對齊：在已緩衝的這 SIZE bytes 內回頭找下一個 A5 5A。
 *
 * 舊行為是「CRC 失敗 → idx 歸零、整段 SIZE bytes 丟掉、從下一個位元組重找 sync」，
 * 代價是一次對齊失誤會連帶吃掉下一包：若真正的封包起點就落在這段緩衝中間
 * （例如位元組流中間被插入/漏掉一個位元組、或雜訊裡湊巧出現假的 A5 5A），
 * 那個起點會被整段丟棄，下一包也跟著報銷。
 *
 * 改成回溯掃描：找到的話把殘料搬到緩衝前端接續累積，等於只賠掉一包。
 * 找不到才歸零。搬移後 idx = SIZE - k ≤ SIZE-1，不會湊出假的「已滿」。
 */
static inline void TelemRx_Realign(TelemRx_t *rx)
{
    const uint8_t SIZE = (uint8_t)sizeof(TelemetryPacket_t);

    /* 從 buf[1] 起找（buf[0] 是這次失敗的起點，必須跳過才會有進展）。 */
    for (uint8_t k = 1U; (uint16_t)(k + 1U) < SIZE; k++) {
        if (rx->buf[k] == TELEM_SYNC0 && rx->buf[k + 1U] == TELEM_SYNC1) {
            uint8_t n = (uint8_t)(SIZE - k);
            memmove(rx->buf, &rx->buf[k], n);
            rx->idx = n;
            rx->resync++;
            return;
        }
    }
    /* 最後一個位元組是 sync0：要再看下一個位元組才知道是不是真的，先停在 idx=1。 */
    if (rx->buf[SIZE - 1U] == TELEM_SYNC0) {
        rx->buf[0] = TELEM_SYNC0;
        rx->idx    = 1U;
        rx->resync++;
        return;
    }
    rx->idx = 0U;
}

/**
 * @brief 餵入一個位元組。湊滿一筆且 CRC 正確時，複製到 *out 並回傳 1；否則 0。
 *        內建 sync 對齊與 CRC 校驗；CRC 失敗或框架破損會自動重新尋找 sync。
 * @note  與 LinkRx_Feed() 行為一致：不論 CRC 成敗，湊滿一筆後 idx 歸零、下一筆重找 sync。
 */
static inline uint8_t TelemRx_Feed(TelemRx_t *rx, uint8_t b, TelemetryPacket_t *out)
{
    const uint8_t SIZE = (uint8_t)sizeof(TelemetryPacket_t);  /* 116 < 256，uint8_t 足夠 */

    switch (rx->idx) {
        case 0U:                       /* 等待 sync0 */
            if (b == TELEM_SYNC0) {
                rx->buf[0] = b;
                rx->idx = 1U;
            }
            return 0U;

        case 1U:                       /* 等待 sync1 */
            if (b == TELEM_SYNC1) {
                rx->buf[1] = b;
                rx->idx = 2U;
            } else if (b == TELEM_SYNC0) {
                rx->buf[0] = b;        /* 連續 sync0：停在 idx=1 重新對齊 */
                rx->idx = 1U;
            } else {
                rx->idx = 0U;          /* 非 sync1 → 退回重找 sync0 */
                rx->resync++;
            }
            return 0U;

        default:                       /* idx >= 2：累積酬載 + CRC */
            rx->buf[rx->idx++] = b;
            if (rx->idx >= SIZE) {
                uint16_t crc_calc = crc16_ccitt_false(rx->buf, (uint16_t)(SIZE - 2));
                uint16_t crc_recv = (uint16_t)(rx->buf[SIZE - 2] |
                                    ((uint16_t)rx->buf[SIZE - 1] << 8));
                if (crc_calc == crc_recv) {
                    memcpy(out, rx->buf, SIZE);
                    rx->idx = 0U;      /* CRC 過：邊界確定，下一筆從頭找 sync */
                    rx->ok++;
                    return 1U;
                }
                rx->crc_err++;
                TelemRx_Realign(rx);   /* CRC 失敗：回頭找緩衝內的下一個 A5 5A */
            }
            return 0U;
    }
}

/**
 * @brief 除錯版：與 TelemRx_Feed 相同的 sync 對齊/CRC 校驗狀態機，但「湊滿一筆就交出」——
 *        不論 CRC 成敗都 memcpy 到 *out 並回傳 1，透過 *crc_ok 回報 CRC 是否通過（1/0）。
 *        供地面站「不過濾、全部先印出來」除錯用（看 CRC 失敗的封包內容判斷訊號品質）。
 *        統計欄位（ok/crc_err/resync）與 TelemRx_Feed 累加方式一致。
 */
static inline uint8_t TelemRx_FeedAny(TelemRx_t *rx, uint8_t b, TelemetryPacket_t *out, uint8_t *crc_ok)
{
    const uint8_t SIZE = (uint8_t)sizeof(TelemetryPacket_t);

    switch (rx->idx) {
        case 0U:
            if (b == TELEM_SYNC0) { rx->buf[0] = b; rx->idx = 1U; }
            return 0U;

        case 1U:
            if (b == TELEM_SYNC1)      { rx->buf[1] = b; rx->idx = 2U; }
            else if (b == TELEM_SYNC0) { rx->buf[0] = b; rx->idx = 1U; }
            else                       { rx->idx = 0U;   rx->resync++; }
            return 0U;

        default:
            rx->buf[rx->idx++] = b;
            if (rx->idx >= SIZE) {
                uint16_t crc_calc = crc16_ccitt_false(rx->buf, (uint16_t)(SIZE - 2));
                uint16_t crc_recv = (uint16_t)(rx->buf[SIZE - 2] |
                                    ((uint16_t)rx->buf[SIZE - 1] << 8));
                memcpy(out, rx->buf, SIZE);          /* 不論成敗都交出 */
                if (crc_calc == crc_recv) {
                    rx->idx = 0U;
                    rx->ok++;
                    if (crc_ok) *crc_ok = 1U;
                } else {
                    rx->crc_err++;
                    if (crc_ok) *crc_ok = 0U;
                    TelemRx_Realign(rx);   /* 回頭找緩衝內的下一個 A5 5A，見該函式註解 */
                }
                return 1U;
            }
            return 0U;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* TELEM_RX_H */
