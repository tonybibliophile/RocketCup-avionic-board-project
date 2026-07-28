/*
 * link_hw.c — 板間鏈路硬體層實作（USART2 全雙工：循環 DMA/IDLE 收 + IT 送）
 * ===========================================================================
 * RX 環形差分沿用 gps.c 的成熟模式（ReceiveToIdle_DMA + RxEventCallback）。
 * TX 用 HAL_UART_Transmit_IT 非阻塞，避免 26 bytes @38400≈6.8ms 阻塞 100Hz 飛控迴圈。
 * 解析與對端狀態更新（LinkRx_Feed / LinkPeer_OnPacket）皆於 ISR context 執行，
 * 單位元組鎖存旗標的讀寫容忍極輕微 tearing（與遙測同策略）。
 */
#include "link_hw.h"

#if FEATURE_LINK

#include "main.h"      /* HAL 型別 + huart2 */
#include <string.h>

extern UART_HandleTypeDef huart2;

/* 循環 DMA 緩衝（DMA1 可存取主 SRAM；落於 .bss）。封包 26B，64B 可緩衝 2+ 筆，
 * IDLE/HT/TC 事件於每筆後即排空。 */
#define LINK_DMA_BUF_SIZE  64U
static uint8_t   s_dma_buf[LINK_DMA_BUF_SIZE];
static uint16_t  s_dma_old_pos = 0;

static LinkRx_t   s_rx;
static LinkPeer_t s_peer;

static uint8_t          s_tx_buf[LINK_PACKET_SIZE];
static uint8_t          s_tx_seq  = 0;
static volatile uint8_t s_tx_busy = 0;

/* ISR：餵一個位元組進解析；湊滿一筆有效封包即更新對端狀態 */
static void link_feed(uint8_t b, uint32_t now)
{
    LinkPacket_t pkt;
    if (LinkRx_Feed(&s_rx, b, &pkt)) {
        LinkPeer_OnPacket(&s_peer, &pkt, now);
    }
}

void Link_Init(void)
{
    LinkRx_Init(&s_rx);
    LinkPeer_Init(&s_peer);
    s_dma_old_pos = 0;
    s_tx_seq      = 0;
    s_tx_busy     = 0;

    HAL_UART_DeInit(&huart2);

    /* USART2 由 MX_USART2_UART_Init 設為 460800；板間鏈路改用 LINK_BAUD。 */
    huart2.Init.BaudRate = LINK_BAUD;
    HAL_UART_Init(&huart2);

    /* IDLE-line + 循環 DMA 接收（hdma_usart2_rx 已於 MSP 連結） */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_dma_buf, LINK_DMA_BUF_SIZE);
}

void Link_SendStatus(const LinkStatus_t *st)
{
    if (s_tx_busy) return;             /* 上一筆 IT 未送完 → 略過（20Hz，下次再送） */

    LinkStatus_t s = *st;
    s.seq = s_tx_seq++;
    LinkProto_Build(s_tx_buf, &s);

    if (HAL_UART_Transmit_IT(&huart2, s_tx_buf, LINK_PACKET_SIZE) == HAL_OK) {
        s_tx_busy = 1;
    }
}

const LinkPeer_t *Link_GetPeer(void) { return &s_peer; }
const LinkRx_t   *Link_GetRx(void)   { return &s_rx;   }

uint8_t Link_PeerFresh(uint32_t now_ms)
{
    return LinkPeer_Fresh(&s_peer, now_ms, LINK_PEER_TIMEOUT_MS);
}

/* 失同步/失聯偵測（純觀測）：追蹤我方狀態變更時刻，若對端未於 LINK_SYNC_TIMEOUT_MS
 * 內 echo-ACK 回同一狀態 → DESYNC；對端逾時無封包 → LOST。 */
static uint8_t  s_link_status         = 0U;
static uint8_t  s_last_my_state       = 0xFFU;
static uint32_t s_my_state_changed_ms = 0U;

void Link_UpdateStatus(uint8_t my_fsm_state, uint32_t now_ms)
{
    if (my_fsm_state != s_last_my_state) {
        s_last_my_state       = my_fsm_state;
        s_my_state_changed_ms = now_ms;
    }
    uint8_t st = 0U;
    if (!LinkPeer_Fresh(&s_peer, now_ms, LINK_PEER_TIMEOUT_MS)) {
        st |= LINK_STATUS_LOST;
    } else if (!LinkPeer_Synced(&s_peer, my_fsm_state) &&
               (now_ms - s_my_state_changed_ms) >= LINK_SYNC_TIMEOUT_MS) {
        st |= LINK_STATUS_DESYNC;
    }
    s_link_status = st;
}

uint8_t Link_GetStatus(void) { return s_link_status; }

/* 循環 DMA 接收事件：Size = 自緩衝起點至目前寫入位置的累計位元組數。
 * 以 s_dma_old_pos 環形差分取出新位元組（與 gps.c 同法）。 */
void Link_OnRxEvent(uint16_t Size)
{
    uint32_t now = HAL_GetTick();
    if (Size != s_dma_old_pos) {
        if (Size > s_dma_old_pos) {
            for (uint16_t i = s_dma_old_pos; i < Size; i++) link_feed(s_dma_buf[i], now);
        } else {
            for (uint16_t i = s_dma_old_pos; i < LINK_DMA_BUF_SIZE; i++) link_feed(s_dma_buf[i], now);
            for (uint16_t i = 0; i < Size; i++) link_feed(s_dma_buf[i], now);
        }
        s_dma_old_pos = Size;
        if (s_dma_old_pos >= LINK_DMA_BUF_SIZE) s_dma_old_pos = 0;
    }
}

void Link_OnError(void)
{
    /* overrun 會中止 DMA → 清旗標、重置解析器並重啟接收，避免鏈路 RX 死掉 */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    s_dma_old_pos = 0;
    LinkRx_Init(&s_rx);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_dma_buf, LINK_DMA_BUF_SIZE);
}

void Link_OnTxComplete(void) { s_tx_busy = 0; }

#endif /* FEATURE_LINK */
