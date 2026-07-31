/*
 * link.h — 板間鏈路對端狀態追蹤（純邏輯，host 可測）
 * ===========================================================================
 * 對稱獨立冗餘：兩板各跑自己的 FSM、獨立開傘；本檔只追蹤「對端最近回報了什麼」，
 * 供地面監看與加法協同（Phase D：副傘 peer_drogue_cmd 加法 OR、主傘 servo_arb
 * 錯開握手）取用。刻意保持純函式（不依賴 HAL / RTOS），由 tests/test_link.c 以
 * 情境鎖定行為。HAL 端（USART2 DMA / 週期廣播）由 main.c 在 FEATURE_LINK 下接線。
 *
 *   對端狀態：LinkPeer_OnPacket() 餵入解析成功的封包；開傘旗標一旦收到即「鎖存」
 *             （drogue_latched / main_latched，供加法協同判斷對端是否已開傘）。
 */
#ifndef LINK_H
#define LINK_H

#include <stdint.h>
#include "link_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* === 對端（peer）狀態 === */
typedef struct {
    uint8_t  valid;          /* 曾收過至少一筆有效封包 */
    uint8_t  board_id;       /* 對端 board_id */
    uint8_t  fsm_state;      /* 對端最近回報的 FSM 狀態 */
    uint8_t  flags;          /* 對端最近一筆 flags */
    uint32_t peer_tick_ms;   /* 對端封包內的飛行 tick */
    uint32_t last_rx_ms;     /* 本機收到該封包時的 tick（freshness 基準） */
    uint8_t  drogue_latched; /* 對端曾通報 DROGUE_FIRED（鎖存，不清除） */
    uint8_t  main_latched;   /* 對端曾通報 MAIN_DEPLOYED（鎖存，不清除） */
    uint8_t  peer_ack_state; /* 對端封包 ack_state：對端回送「它所認知的『我方』狀態」 */
    uint8_t  peer_main_arb;  /* 對端主傘共開 / BENCH 狀態（SERVO_ARB_MSG_*，servo_arb.h） */
    uint8_t  peer_flash_ready; /* 對端 Flash 預擦池是否已達目標 (1=已就緒) */
    uint8_t  peer_erase_pct;   /* 對端 Flash 預擦進度 0..100% */
    int32_t  h_est_cm;       /* 對端最近回報的 EKF 高度 (cm)（供下鏈中繼/監看） */
    int32_t  v_est_cms;      /* 對端最近回報的 EKF 垂直速度 (cm/s) */
    int32_t  baro_alt_cm;    /* 對端最近回報的 baro 相對高度 (cm) */
    int16_t  a_z_cg;         /* 對端最近回報的加速度 (0.01g) */
    int16_t  q_w, q_x, q_y, q_z; /* 對端最近回報的姿態四元數 * 10000 */
    int32_t  vf_h_cm;        /* 對端最近回報的 VF 高度 (cm) */
    int32_t  vf_v_cms;       /* 對端最近回報的 VF 垂直速度 (cm/s) */
    int16_t  bmi_mag_cg;     /* 對端最近回報的 BMI088 加速度模長 (cg = 0.01g) */
    int16_t  adxl_mag_cg;    /* 對端最近回報的 ADXL375 加速度模長 (cg = 0.01g) */
    uint8_t  profile_flags;  /* 對端最近回報的 TELEM_PROFILE_SELF_ELEVATOR（電梯測試 profile 標示） */
    /* --- 鏈路品質（QoS）：以 seq 差偵測丟包 --- */
    uint8_t  last_seq;       /* 上一筆封包的 seq（seq_valid 後才有效） */
    uint8_t  seq_valid;      /* 已收過至少一筆、last_seq 可用 */
    uint32_t rx_count;       /* 已接受（CRC 通過）的封包數 */
    uint32_t lost_count;     /* 依 seq gap 估計的累計丟包數 */
} LinkPeer_t;

void    LinkPeer_Init(LinkPeer_t *p);
void    LinkPeer_OnPacket(LinkPeer_t *p, const LinkPacket_t *pkt, uint32_t now_ms);
/* 對端是否仍在線（valid 且距上次收包 < timeout_ms） */
uint8_t LinkPeer_Fresh(const LinkPeer_t *p, uint32_t now_ms, uint32_t timeout_ms);

/* 對端是否已「回音確認」我方目前狀態（peer_ack_state == my_state）。
 * 用於失同步偵測：我方狀態改變後，對端在 LINK_SYNC_TIMEOUT_MS 內未回同一狀態 → DESYNC。
 * 純觀測，不影響開傘決策。 */
uint8_t LinkPeer_Synced(const LinkPeer_t *p, uint8_t my_state);

#ifdef __cplusplus
}
#endif

#endif /* LINK_H */
