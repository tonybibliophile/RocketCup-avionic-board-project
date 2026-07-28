/*
 * uplink_proto.h — 上行命令封包（地面站 → 火箭，433MHz）協定（純邏輯，host 可測）
 * ===========================================================================
 * 地面站手動觸發開傘等命令，經 E22 433 透傳「反向」打給火箭。火箭在下行遙測
 * 之外，每 10 個 433 時槽空出 1 槽（不發射）讓地面站有空檔可上行；火箭 E22
 * 透傳模式在不發送時即在接收，空檔內收到的位元組流餵入本解析器。
 *
 * 框架（7 bytes，與下行遙測 0xA5/0x5A 不同 sync，避免互相誤判）：
 *   [0]=SYNC0 0x55  [1]=SYNC1 0xAA  [2]=cmd  [3]=arg  [4]=seq  [5..6]=crc16(LE)
 *   crc16 = CRC-16/CCITT-FALSE，覆蓋前 5 bytes（sync+cmd+arg+seq）。
 *
 * 安全：開傘為兩段式 —— 須先收 ARM，DEPLOY 才生效（火箭端 uplink_cmd.c 實作 +
 * ARM 逾時自動解除）。本檔僅負責「框架 + CRC」的建構與解析，不含任何點火動作。
 *
 * 純 header-only（仿 telem_rx.h / crc16.h），不依賴 HAL / RTOS，
 * 由 tests/test_uplink_proto.c 驗證。位元組順序 little-endian（STM32 與 host 皆是）。
 */
#ifndef UPLINK_PROTO_H
#define UPLINK_PROTO_H

#include <stdint.h>
#include <string.h>
#include "crc16.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UPLINK_SYNC0  0x55U
#define UPLINK_SYNC1  0xAAU
#define UPLINK_FRAME_SIZE  7U

/* 命令碼 */
#define UPLINK_CMD_PING           0x01U  /* 連線測試（火箭印出，不動作） */
#define UPLINK_CMD_ARM            0x10U  /* 武裝：開啟開傘允許窗（逾時自動解除） */
#define UPLINK_CMD_DISARM         0x11U  /* 解除武裝 */
#define UPLINK_CMD_DEPLOY_DROGUE  0x20U  /* 手動開副傘（須先 ARM） */
#define UPLINK_CMD_DEPLOY_MAIN    0x21U  /* 手動開主傘（須先 ARM） */
#define UPLINK_CMD_DEPLOY_BOTH    0x22U  /* 手動同時開副傘 + 主傘（須先 ARM） */
#define UPLINK_CMD_BENCH          0x30U  /* 桌面測試：跑一次 pyro/servo 自測後回歸（須先 ARM
                                          * ＋僅限未起飛 STATE_PAD/INIT，見 uplink_cmd.c） */
#define UPLINK_CMD_RECALIBRATE    0x40U  /* EKF 重新校準（須先 ARM＋僅限未起飛 STATE_PAD/INIT） */
#define UPLINK_CMD_RECOVERY       0x50U  /* 尋回指令：落地後停止蜂鳴器與數據記錄（SD/Flash） */

/** @brief 該命令是否為開傘類（需武裝才生效）。 */
static inline int uplink_cmd_is_deploy(uint8_t cmd)
{
    return (cmd == UPLINK_CMD_DEPLOY_DROGUE ||
            cmd == UPLINK_CMD_DEPLOY_MAIN   ||
            cmd == UPLINK_CMD_DEPLOY_BOTH);
}

/**
 * @brief 建構一筆上行命令框架到 out（須 >= UPLINK_FRAME_SIZE）。
 * @return 寫入位元組數（UPLINK_FRAME_SIZE）。
 */
static inline uint8_t UplinkProto_Build(uint8_t *out, uint8_t cmd, uint8_t arg, uint8_t seq)
{
    out[0] = UPLINK_SYNC0;
    out[1] = UPLINK_SYNC1;
    out[2] = cmd;
    out[3] = arg;
    out[4] = seq;
    uint16_t crc = crc16_ccitt_false(out, 5);
    out[5] = (uint8_t)(crc & 0xFF);
    out[6] = (uint8_t)(crc >> 8);
    return UPLINK_FRAME_SIZE;
}

/* 逐位元組接收狀態機 + 診斷統計（仿 TelemRx_t） */
typedef struct {
    uint8_t  buf[UPLINK_FRAME_SIZE];
    uint8_t  idx;
    uint32_t ok;        /* CRC 通過交出的命令數 */
    uint32_t crc_err;   /* 湊滿一筆但 CRC 不符 */
    uint32_t resync;    /* sync1 階段遇非預期位元組退回的次數 */
} UplinkRx_t;

static inline void UplinkRx_Init(UplinkRx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
}

/**
 * @brief 餵入一個位元組。湊滿一筆且 CRC 正確時填 *cmd/*arg/*seq 並回傳 1；否則 0。
 *        內建 sync 對齊與 CRC 校驗；破損會自動重新尋找 sync（與 TelemRx_Feed 一致）。
 */
static inline uint8_t UplinkRx_Feed(UplinkRx_t *rx, uint8_t b,
                                    uint8_t *cmd, uint8_t *arg, uint8_t *seq)
{
    switch (rx->idx) {
        case 0U:
            if (b == UPLINK_SYNC0) { rx->buf[0] = b; rx->idx = 1U; }
            return 0U;

        case 1U:
            if (b == UPLINK_SYNC1) {
                rx->buf[1] = b; rx->idx = 2U;
            } else if (b == UPLINK_SYNC0) {
                rx->buf[0] = b;            /* 連續 sync0：停在 idx=1 重新對齊 */
                rx->idx = 1U;
            } else {
                rx->idx = 0U; rx->resync++;
            }
            return 0U;

        default:
            rx->buf[rx->idx++] = b;
            if (rx->idx >= UPLINK_FRAME_SIZE) {
                rx->idx = 0U;              /* 不論成敗，下一筆從頭找 sync */
                uint16_t crc_calc = crc16_ccitt_false(rx->buf, 5);
                uint16_t crc_recv = (uint16_t)(rx->buf[5] | ((uint16_t)rx->buf[6] << 8));
                if (crc_calc == crc_recv) {
                    if (cmd) *cmd = rx->buf[2];
                    if (arg) *arg = rx->buf[3];
                    if (seq) *seq = rx->buf[4];
                    rx->ok++;
                    return 1U;
                }
                rx->crc_err++;
            }
            return 0U;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* UPLINK_PROTO_H */
