/*
 * link_hw.h — 板間鏈路硬體層（USART2 全雙工：DMA/IDLE 收 + IT 送）
 * ===========================================================================
 * 把純邏輯（link_proto / link）接到 HAL：主備兩板程式相同，差別只在角色與接線
 * （板間排線 TX/RX 交叉一次）。收到的封包於 ISR 內更新 LinkPeer_t；main.c 讀
 * Link_GetPeer() 的狀態/鎖存旗標/QoS 供地面監看與加法協同（不抑制自身開傘）。
 *
 * 僅在 FEATURE_LINK 編入。
 */
#ifndef LINK_HW_H
#define LINK_HW_H

#include "board_config.h"

#if FEATURE_LINK

#include "link.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 啟動 USART2 板間鏈路（重設 baud 為 LINK_BAUD + 啟動循環 DMA/IDLE 接收）。 */
void Link_Init(void);

/* 非阻塞送出一筆自身狀態（IT 傳輸；自動補遞增 seq；上一筆未送完則略過本次）。 */
void Link_SendStatus(const LinkStatus_t *st);

/* 組裝「本板目前狀態」封包（實作在 main.c，需要 FSM/EKF/感測器等全域）。
 * 凡是要送 LinkStatus 的地方都走這支，確保各路廣播內容同源——bench 序列與飛控迴圈
 * 是兩個 task、會同時各送各的，內容若不同源，對端欄位就會在兩套值之間跳動。 */
void Link_BuildOwnStatus(LinkStatus_t *ls);

/* fsm_state 數值 → 簡短字串（"PAD_ARMED" 等），供 [LINK] 診斷行輸出（實作在 main.c）。 */
const char *link_fsm_state_name(uint8_t s);

/* 取得對端狀態（含鎖存的 drogue/main 開傘旗標 + QoS：rx_count/lost_count），
 * 供地面監看與加法協同讀取。 */
const LinkPeer_t *Link_GetPeer(void);

/* 取得接收端框架統計（ok/crc_err/resync），供 QoS 下鏈/診斷。 */
const LinkRx_t *Link_GetRx(void);

/* 對端是否仍在線（valid 且距上次收包 < LINK_PEER_TIMEOUT_MS）。 */
uint8_t Link_PeerFresh(uint32_t now_ms);

/* DISARM 專用：清掉對端的四個開傘鎖存（見 LinkPeer_ClearDeployLatches）。 */
void Link_ClearPeerDeployLatches(void);

/* 板間鏈路健康狀態位（純觀測；不影響開傘決策）。 */
#define LINK_STATUS_LOST    0x01U  /* 對端失聯（距上次有效封包 > LINK_PEER_TIMEOUT_MS） */
#define LINK_STATUS_DESYNC  0x02U  /* 我方狀態改變後對端未於 LINK_SYNC_TIMEOUT_MS 內 echo-ACK */

/* 依「我方目前 FSM 狀態」更新鏈路健康狀態（每 publish tick 呼叫）。 */
void    Link_UpdateStatus(uint8_t my_fsm_state, uint32_t now_ms);
/* 取得最近一次 Link_UpdateStatus 計算的 LINK_STATUS_* 位。 */
uint8_t Link_GetStatus(void);

/* --- 以下由 main.c 的 HAL 回呼依 instance 轉接（ISR context） --- */
void Link_OnRxEvent(uint16_t Size);   /* HAL_UARTEx_RxEventCallback(USART2) */
void Link_OnError(void);              /* HAL_UART_ErrorCallback(USART2) */
void Link_OnTxComplete(void);         /* HAL_UART_TxCpltCallback(USART2) */

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_LINK */
#endif /* LINK_HW_H */
