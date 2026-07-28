/*
 * ack_proto.h — 火箭 → 地面站 命令 ACK 幀（下行，433/920）協定（純邏輯，host 可測）
 * ===========================================================================
 * 火箭收到並執行地面站上行命令（uplink_text_proto / uplink_proto）後，經下行回傳一筆
 * 文字 ACK，讓地面站確認「命令有收到且已執行/被拒」。ACK 與二進制遙測共用同一條下行
 * （E22 433 透傳 byte 流 / E80 920 封包），故用不同 sync（0xAC/0xCA）與遙測（0xA5/0x5A）
 * 及上行（0x55/xx）區隔；地面站在同一位元組流上同時餵 TelemRx_Feed 與 AckRx_Feed，
 * 各自忽略不符自身 sync 的位元組，互不干擾。
 *
 * 框架（變長）：
 *   [0]=SYNC0 0xAC  [1]=SYNC1 0xCA  [2]=seq(回應對應上行命令的 seq)  [3]=status
 *   [4]=len(0..ACK_TEXT_MAX)  [5..5+len-1]=text  [.. +2]=crc16(LE)
 *   crc16 = CRC-16/CCITT-FALSE，覆蓋 crc 前面全部位元組。
 *   text = 回 echo 的命令字串（供操作員核對是哪一筆），可為空（len==0）。
 *
 * 純 header-only（仿 uplink_text_proto.h / telem_rx.h），不依賴 HAL / RTOS，
 * 由 tests/test_ack_proto.c 驗證。位元組順序 little-endian（STM32 與 host 皆是）。
 */
#ifndef ACK_PROTO_H
#define ACK_PROTO_H

#include <stdint.h>
#include <string.h>
#include "crc16.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ACK_SYNC0     0xACU
#define ACK_SYNC1     0xCAU
#define ACK_TEXT_MAX  32U   /* echo 文字上限（不含結尾 NUL） */

/* 幀最大長度：sync(2)+seq(1)+status(1)+len(1)+text(ACK_TEXT_MAX)+crc(2) */
#define ACK_FRAME_MAX  (2U + 1U + 1U + 1U + ACK_TEXT_MAX + 2U)

/* status 碼 */
#define ACK_OK        0x00U  /* 命令已接受/執行 */
#define ACK_UNKNOWN   0x01U  /* 未知命令 */
#define ACK_BADARG    0x02U  /* 參數格式錯誤 */
#define ACK_UNARMED   0x03U  /* 需先 ARM（bench/開傘類被拒） */
#define ACK_REJECTED  0x04U  /* 條件不符被拒（如 bench 僅限未起飛） */

/** @brief status 碼 → 短字串（地面站/GUI 顯示用）。 */
static inline const char *ack_status_str(uint8_t status)
{
    switch (status) {
        case ACK_OK:       return "OK";
        case ACK_UNKNOWN:  return "UNKNOWN";
        case ACK_BADARG:   return "BADARG";
        case ACK_UNARMED:  return "UNARMED";
        case ACK_REJECTED: return "REJECTED";
        default:           return "?";
    }
}

/**
 * @brief 建構一筆 ACK 幀到 out（須 >= ACK_FRAME_MAX）。
 * @param text 可為 NULL（視為空）。len 會夾到 ACK_TEXT_MAX。
 * @return 寫入位元組數（>= 7）。
 */
static inline uint8_t AckProto_Build(uint8_t *out, uint8_t seq, uint8_t status,
                                     const char *text, uint8_t len)
{
    if (text == NULL) len = 0U;
    if (len > ACK_TEXT_MAX) len = ACK_TEXT_MAX;
    out[0] = ACK_SYNC0;
    out[1] = ACK_SYNC1;
    out[2] = seq;
    out[3] = status;
    out[4] = len;
    if (len) memcpy(&out[5], text, len);
    uint16_t crc = crc16_ccitt_false(out, (uint16_t)(5U + len));
    out[5 + len]     = (uint8_t)(crc & 0xFFU);
    out[5 + len + 1] = (uint8_t)(crc >> 8);
    return (uint8_t)(5U + len + 2U);
}

/* 逐位元組接收狀態機 + 診斷統計 */
typedef struct {
    uint8_t  buf[ACK_FRAME_MAX];
    uint8_t  idx;
    uint8_t  len;
    uint32_t ok;
    uint32_t crc_err;
    uint32_t resync;
} AckRx_t;

static inline void AckRx_Init(AckRx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
}

/**
 * @brief 餵入一個位元組。湊滿一筆且 CRC 正確時，填 out_seq/out_status，把 echo 文字
 *        複製到 out_text（NUL 結尾）並回傳 1；否則 0。
 * @param out_text 緩衝區，須 >= ACK_TEXT_MAX+1。可為 NULL。
 */
static inline uint8_t AckRx_Feed(AckRx_t *rx, uint8_t b,
                                 uint8_t *out_seq, uint8_t *out_status,
                                 char *out_text, uint8_t *out_len)
{
    switch (rx->idx) {
        case 0U:
            if (b == ACK_SYNC0) { rx->buf[0] = b; rx->idx = 1U; }
            return 0U;

        case 1U:
            if (b == ACK_SYNC1) {
                rx->buf[1] = b; rx->idx = 2U;
            } else if (b == ACK_SYNC0) {
                rx->buf[0] = b; rx->idx = 1U;
            } else {
                rx->idx = 0U; rx->resync++;
            }
            return 0U;

        case 2U:                           /* seq */
            rx->buf[2] = b; rx->idx = 3U;
            return 0U;

        case 3U:                           /* status */
            rx->buf[3] = b; rx->idx = 4U;
            return 0U;

        case 4U:                           /* len */
            if (b > ACK_TEXT_MAX) {
                rx->idx = 0U; rx->resync++;  /* 非法長度 → 重找 sync */
                return 0U;
            }
            rx->buf[4] = b; rx->len = b; rx->idx = 5U;
            /* len==0 的合法空 ACK：下個位元組進 default 分支收 crc */
            return 0U;

        default: {                         /* idx>=5：text + crc */
            rx->buf[rx->idx++] = b;
            uint8_t frame_len = (uint8_t)(5U + rx->len + 2U);
            if (rx->idx >= frame_len) {
                rx->idx = 0U;
                uint16_t crc_calc = crc16_ccitt_false(rx->buf, (uint16_t)(5U + rx->len));
                uint16_t crc_recv = (uint16_t)(rx->buf[5 + rx->len] |
                                    ((uint16_t)rx->buf[5 + rx->len + 1] << 8));
                if (crc_calc == crc_recv) {
                    if (out_seq)    *out_seq    = rx->buf[2];
                    if (out_status) *out_status = rx->buf[3];
                    if (out_text) {
                        if (rx->len) memcpy(out_text, &rx->buf[5], rx->len);
                        out_text[rx->len] = '\0';
                    }
                    if (out_len) *out_len = rx->len;
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

#endif /* ACK_PROTO_H */
