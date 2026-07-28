/*
 * link_proto.h — 主/備航電板間鏈路二進制封包（binary + CRC16，純邏輯）
 * ===========================================================================
 * 沿用下行遙測（telemetry.h）的框架慣例：sync word + seq + 縮放整數 + 結尾
 * CRC-16/CCITT-FALSE。但採用「不同 sync」(0xC3,0x3C) 以與下行遙測 (0xA5,0x5A)
 * 區隔，避免地面站 / 轉發器把兩種封包混淆。flags 直接重用 telemetry.h 的
 * TELEM_FLAG_* 位元值（單一真相來源），備板據 DROGUE_FIRED / MAIN_DEPLOYED 判
 * 主板是否已開傘。
 *
 * 本檔與 link_proto.c 不依賴 HAL / RTOS，僅 <stdint.h>（+ crc16.h / telemetry.h
 * 皆 header-only 純邏輯），可由 tests/test_link_proto.c 在 host 上驗證。
 * 位元組順序假設 little-endian（STM32 與 x86 host 皆是，與 telemetry 同策略）。
 */
#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>
#include "telemetry.h"   /* 重用 TELEM_FLAG_*（DROGUE_FIRED / MAIN_DEPLOYED / ...） */

#ifdef __cplusplus
extern "C" {
#endif

/* 同步字（板間鏈路專用，區別於下行遙測 0xA5/0x5A） */
#define LINK_SYNC0  0xC3U
#define LINK_SYNC1  0x3CU

/* board_id 欄位值 */
#define LINK_BOARD_PRIMARY  0U
#define LINK_BOARD_BACKUP   1U

/* 板間狀態封包（packed，固定長度）。欄位順序即解碼契約。 */
typedef struct __attribute__((packed)) {
    uint8_t  sync0;        /* 0xC3 */
    uint8_t  sync1;        /* 0x3C */
    uint8_t  board_id;     /* LINK_BOARD_PRIMARY / LINK_BOARD_BACKUP */
    uint8_t  seq;          /* 遞增序號（自動 wrap），對端偵測丟包 */
    uint8_t  fsm_state;    /* FlightState_t 飛行狀態碼 */
    uint8_t  flags;        /* TELEM_FLAG_* 子集（DROGUE_FIRED/MAIN_DEPLOYED/FAILSAFE/...） */
    uint32_t tick_ms;      /* 發送端飛行 tick；對端判 freshness / 飛行時間一致性 */
    int32_t  h_est_cm;     /* EKF 高度 (cm)：交叉檢查與事後判讀 */
    int32_t  v_est_cms;    /* EKF 垂直速度 (cm/s) */
    int32_t  baro_alt_cm;  /* baro 相對高度 (cm) */
    int16_t  a_z_cg;       /* 高G 垂直加速度 (cg = 0.01g) */
    uint8_t  ack_state;    /* echo-ACK：回送「我最近採納的對端 FSM 狀態」＝對 peer 的確認 */
    int16_t  q_w;          /* 四元數 qw * 10000 [-10000, 10000] */
    int16_t  q_x;          /* 四元數 qx * 10000 [-10000, 10000] */
    int16_t  q_y;          /* 四元數 qy * 10000 [-10000, 10000] */
    int16_t  q_z;          /* 四元數 qz * 10000 [-10000, 10000] */
    uint8_t  main_arb;     /* D2 主傘舵機互斥握手狀態（SERVO_ARB_MSG_*，servo_arb.h） */
    uint8_t  flash_ready;  /* 1 = 本板 Flash 預擦池已達目標 (960 sectors) */
    uint8_t  erase_pct;    /* 本板 Flash 預擦進度 0..100% */
    int32_t  vf_h_cm;      /* 垂直濾波器 (VF) 高度 (cm)：供對端中繼下鏈，與 EKF 對照 */
    int32_t  vf_v_cms;     /* 垂直濾波器 (VF) 垂直速度 (cm/s) */
    uint16_t crc16;        /* CRC-16/CCITT-FALSE，覆蓋本封包前面所有位元組 */
} LinkPacket_t;

#define LINK_PACKET_SIZE  ((uint16_t)sizeof(LinkPacket_t))

/* 組封包輸入（呼叫端填好交給 LinkProto_Build；sync/crc 由 Build 補上） */
typedef struct {
    uint8_t  board_id;
    uint8_t  seq;
    uint8_t  fsm_state;
    uint8_t  flags;
    uint32_t tick_ms;
    int32_t  h_est_cm;
    int32_t  v_est_cms;
    int32_t  baro_alt_cm;
    int16_t  a_z_cg;
    uint8_t  ack_state;    /* echo-ACK：呼叫端填「我最近採納的對端 FSM 狀態」 */
    int16_t  q_w;
    int16_t  q_x;
    int16_t  q_y;
    int16_t  q_z;
    uint8_t  main_arb;     /* D2 主傘舵機互斥握手狀態（呼叫端填 ServoArb 廣播值） */
    uint8_t  flash_ready;  /* 1 = 本板 Flash 預擦池已達目標 (960 sectors) */
    uint8_t  erase_pct;    /* 本板 Flash 預擦進度 0..100% */
    int32_t  vf_h_cm;      /* 垂直濾波器 (VF) 高度 (cm) */
    int32_t  vf_v_cms;     /* 垂直濾波器 (VF) 垂直速度 (cm/s) */
} LinkStatus_t;

/**
 * @brief 由狀態快照組一筆封包到 out（須至少 LINK_PACKET_SIZE bytes）。
 *        自動填入 sync0/sync1 與結尾 CRC16。
 * @return 寫入長度（= LINK_PACKET_SIZE）。
 */
uint16_t LinkProto_Build(uint8_t *out, const LinkStatus_t *st);

/* 逐位元組接收狀態機 + 鏈路品質統計（照 ack_proto.h 的 AckRx_t 慣例） */
typedef struct {
    uint8_t  buf[sizeof(LinkPacket_t)];
    uint8_t  idx;      /* 已存入 buf 的位元組數（0 = 等待 sync0） */
    uint32_t ok;       /* 成功解出（CRC 通過）的封包數 */
    uint32_t crc_err;  /* 湊滿一筆但 CRC 不符 */
    uint32_t resync;   /* sync0 後遇非法 sync1 → 重新對齊 */
} LinkRx_t;

void LinkRx_Init(LinkRx_t *rx);

/**
 * @brief 餵入一個位元組。湊滿一筆且 CRC 正確時，複製到 *out 並回傳 1；否則 0。
 *        內建 sync 對齊與 CRC 校驗；CRC 失敗或框架破損會自動重新尋找 sync。
 */
uint8_t LinkRx_Feed(LinkRx_t *rx, uint8_t b, LinkPacket_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LINK_PROTO_H */
