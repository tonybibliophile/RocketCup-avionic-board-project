/*
 * uplink_text_proto.h — 文字上行命令幀（地面站 → 火箭，433MHz）協定（純邏輯，host 可測）
 * ===========================================================================
 * 與 uplink_proto.h 的「固定 2-byte cmd/arg」幀並存但分開：本幀承載一段變長 ASCII
 * 文字命令（≤ UPLINK_TEXT_MAX bytes），火箭端解出後直接餵進 main.c 的
 * Parse_Serial_Command() —— 一條路徑即涵蓋所有既有文字指令（磁力計校正、e22/e80
 * RF 設定、role/help…），不需為每個指令新增 opcode。
 *
 * 為與二進制上行幀（0x55/0xAA）區隔，本幀 sync 用 0x55/0xBB（sync1 不同）。火箭端在
 * 同一 433 位元組流上同時餵二進制解析器（UplinkRx_Feed）與本文字解析器
 * （UplinkTextRx_Feed），兩者各自忽略不符自身 sync 的位元組，互不干擾。
 *
 * 框架（變長）：
 *   [0]=SYNC0 0x55  [1]=SYNC1 0xBB  [2]=seq  [3]=len(1..UPLINK_TEXT_MAX)
 *   [4..4+len-1]=text  [.. +2]=crc16(LE)
 *   crc16 = CRC-16/CCITT-FALSE，覆蓋 sync0..text（即 crc 前面全部位元組）。
 *
 * 純 header-only（仿 uplink_proto.h / telem_rx.h），不依賴 HAL / RTOS，
 * 由 tests/test_uplink_proto.c 驗證。位元組順序 little-endian（STM32 與 host 皆是）。
 */
#ifndef UPLINK_TEXT_PROTO_H
#define UPLINK_TEXT_PROTO_H

#include <stdint.h>
#include <string.h>
#include "crc16.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UPLINK_TEXT_SYNC0  0x55U
#define UPLINK_TEXT_SYNC1  0xBBU
#define UPLINK_TEXT_MAX    32U   /* 文字酬載上限（不含結尾 NUL；涵蓋現有最長指令） */

/* 幀最大長度：sync(2)+seq(1)+len(1)+text(UPLINK_TEXT_MAX)+crc(2) */
#define UPLINK_TEXT_FRAME_MAX  (2U + 1U + 1U + UPLINK_TEXT_MAX + 2U)

/**
 * @brief 建構一筆文字上行幀到 out（須 >= UPLINK_TEXT_FRAME_MAX）。
 * @param text  ASCII 文字（不需結尾 NUL；由 len 界定）。len 會夾到 UPLINK_TEXT_MAX。
 * @return 寫入位元組數；text 為空（len==0）時回傳 0（不建構空命令幀）。
 */
static inline uint8_t UplinkTextProto_Build(uint8_t *out, const char *text,
                                            uint8_t len, uint8_t seq)
{
    if (len == 0U) return 0U;
    if (len > UPLINK_TEXT_MAX) len = UPLINK_TEXT_MAX;
    out[0] = UPLINK_TEXT_SYNC0;
    out[1] = UPLINK_TEXT_SYNC1;
    out[2] = seq;
    out[3] = len;
    memcpy(&out[4], text, len);
    uint16_t crc = crc16_ccitt_false(out, (uint16_t)(4U + len));
    out[4 + len]     = (uint8_t)(crc & 0xFFU);
    out[4 + len + 1] = (uint8_t)(crc >> 8);
    return (uint8_t)(4U + len + 2U);
}

/* 逐位元組接收狀態機 + 診斷統計（仿 UplinkRx_t / TelemRx_t） */
typedef struct {
    uint8_t  buf[UPLINK_TEXT_FRAME_MAX];
    uint8_t  idx;      /* 已存入 buf 的位元組數（0 = 等待 sync0） */
    uint8_t  len;      /* 本幀宣告的 text 長度（idx>=4 後有效） */
    uint32_t ok;       /* CRC 通過交出的命令數 */
    uint32_t crc_err;  /* 湊滿一筆但 CRC 不符 */
    uint32_t resync;   /* sync1 階段遇非預期位元組退回的次數 */
} UplinkTextRx_t;

static inline void UplinkTextRx_Init(UplinkTextRx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
}

/**
 * @brief 餵入一個位元組。湊滿一筆且 CRC 正確時，把文字複製到 out_text（NUL 結尾）、
 *        填 *out_len / *out_seq 並回傳 1；否則 0。
 * @param out_text  緩衝區，須 >= UPLINK_TEXT_MAX+1（含結尾 NUL）。可為 NULL。
 *        內建 sync 對齊、len 界定與 CRC 校驗；破損自動重新尋找 sync。
 */
static inline uint8_t UplinkTextRx_Feed(UplinkTextRx_t *rx, uint8_t b,
                                        char *out_text, uint8_t *out_len, uint8_t *out_seq)
{
    switch (rx->idx) {
        case 0U:
            if (b == UPLINK_TEXT_SYNC0) { rx->buf[0] = b; rx->idx = 1U; }
            return 0U;

        case 1U:
            if (b == UPLINK_TEXT_SYNC1) {
                rx->buf[1] = b; rx->idx = 2U;
            } else if (b == UPLINK_TEXT_SYNC0) {
                rx->buf[0] = b;            /* 連續 sync0：停在 idx=1 重新對齊 */
                rx->idx = 1U;
            } else {
                rx->idx = 0U; rx->resync++;
            }
            return 0U;

        case 2U:                           /* seq */
            rx->buf[2] = b; rx->idx = 3U;
            return 0U;

        case 3U:                           /* len */
            if (b == 0U || b > UPLINK_TEXT_MAX) {
                rx->idx = 0U; rx->resync++;  /* 非法長度 → 重找 sync */
                return 0U;
            }
            rx->buf[3] = b; rx->len = b; rx->idx = 4U;
            return 0U;

        default: {                         /* idx>=4：text + crc */
            rx->buf[rx->idx++] = b;
            uint8_t frame_len = (uint8_t)(4U + rx->len + 2U);
            if (rx->idx >= frame_len) {
                rx->idx = 0U;              /* 不論成敗，下一筆從頭找 sync */
                uint16_t crc_calc = crc16_ccitt_false(rx->buf, (uint16_t)(4U + rx->len));
                uint16_t crc_recv = (uint16_t)(rx->buf[4 + rx->len] |
                                    ((uint16_t)rx->buf[4 + rx->len + 1] << 8));
                if (crc_calc == crc_recv) {
                    if (out_text) {
                        memcpy(out_text, &rx->buf[4], rx->len);
                        out_text[rx->len] = '\0';
                    }
                    if (out_len) *out_len = rx->len;
                    if (out_seq) *out_seq = rx->buf[2];
                    rx->ok++;
                    return 1U;
                }
                rx->crc_err++;
            }
            return 0U;
        }
    }
}

#ifdef __cplusplus
}
#endif

#endif /* UPLINK_TEXT_PROTO_H */
