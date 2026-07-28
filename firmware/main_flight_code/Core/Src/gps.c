/**
  ******************************************************************************
  * @file           : gps.c
  * @brief          : NMEA-0183 GPS driver (USART6, DMA-to-idle RX) — 實作
  *
  * GPS（NEO-M9N）實體掛載於 USART6（PC6=TX/PC7=RX），該埠已於 CubeMX 配置
  * 循環 DMA RX（DMA2_Stream1, ch5）＋ USART6 IDLE 中斷，故採 ReceiveToIdle_DMA：
  * DMA 於背景把位元組搬進 gps_dma_buf，IDLE/半傳輸/傳輸完成事件觸發
  * HAL_UARTEx_RxEventCallback，於其中把新位元組餵入 GPS_FeedByte 組句。
  ******************************************************************************
  */
#include "gps.h"
#include "rate_monitor.h"
#include "main.h"
#include <string.h>

extern IWDG_HandleTypeDef hiwdg;

/* ── GPS bring-up 隔離測試開關 ─────────────────────────────────────────────
 * 0 = 正常：460800 動態協商 + 25Hz + CFG-RST。★上板實測此路徑 GPS RAW 有資料，
 *     模組實際跑在 460800（推測由 BBR/備援電池保留上次設定），為正式組態。
 * 1 = 最小收訊模式：跳過所有 UBX 協商，USART6 固定於 GPS_MINIMAL_BAUD 直接收。
 *     ★上板實測 9600 完全收不到（模組不在 9600），僅保留供日後鮑率隔離除錯。 */
#ifndef GPS_BRINGUP_MINIMAL
#define GPS_BRINGUP_MINIMAL 0
#endif
/* 最小模式鮑率：當初測試成功用 9600；NEO-M9N 冷開機出廠預設為 38400，收不到時可改試。 */
#ifndef GPS_MINIMAL_BAUD
#define GPS_MINIMAL_BAUD    9600U
#endif

/* 純解析邏輯（組句狀態機 / checksum / GGA / RMC）已抽至 gps_parse.h（host 可測，
 * tests/test_gps.c），本檔僅保留不純的部分：DMA 環形差分、ISR↔task 交接、UBX 初始化。 */
#define GPS_LINE_MAX   GPS_PARSE_LINE_MAX

/* --- 模組狀態 --- */
static UART_HandleTypeDef *gps_huart = NULL;

/* USART6_RX 循環 DMA 緩衝：DMA2 僅能存取主 SRAM（非 CCM），此 static 落於 .bss 主 SRAM。
 * 512 bytes 供 25 Hz 高速 UBX 串流，IDLE/HT/TC 事件可即時排空。 */
#define GPS_DMA_BUF_SIZE  512U
static uint8_t  gps_dma_buf[GPS_DMA_BUF_SIZE];
static uint16_t gps_dma_old_pos = 0;               /* 上次已處理到的 DMA 寫入位置（環形） */

static GpsUbxAsm_t gps_ubx_asm;                    /* ISR 內 UBX 二進位組包狀態機 */
static UbxNavPvt_t gps_pvt_ready;                  /* 已就緒、待 task 解析的整包 PVT */
static volatile uint8_t gps_pvt_ready_flag = 0;    /* 1 = gps_pvt_ready 有一包待解析 */
static volatile uint32_t gps_overrun_drops = 0;    /* task 還沒解析就被新包覆蓋的次數 */

static GPS_Data_t gps_data;                        /* 對外解析結果 */

/* ------------------------------------------------------------------ */
/* 低階：RX 中斷組裝整包 UBX                                          */
/* ------------------------------------------------------------------ */

/* 由 HAL_UARTEx_RxEventCallback（ISR context）對每個新收到的位元組呼叫。 */
static void GPS_FeedByte(uint8_t b)
{
    if (gps_ubx_feed(&gps_ubx_asm, b)) {
        if (gps_pvt_ready_flag) {
            gps_overrun_drops++;
        }
        memcpy(&gps_pvt_ready, gps_ubx_asm.payload_buf, sizeof(UbxNavPvt_t));
        gps_pvt_ready_flag = 1;
    }
}

/* 發送 UBX 指令助手：自動計算 8-bit Fletcher Checksum (CK_A, CK_B) 避開人為填寫錯誤 */
static void send_ubx_cmd(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    if (!gps_huart) return;
    uint8_t header[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) {
        ck_a = (uint8_t)(ck_a + header[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    for (int i = 0; i < len; i++) {
        ck_a = (uint8_t)(ck_a + payload[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    uint8_t footer[2] = {ck_a, ck_b};

    HAL_UART_Transmit(gps_huart, header, 6, 50);
    if (len > 0 && payload) {
        HAL_UART_Transmit(gps_huart, (uint8_t*)payload, len, 50);
    }
    HAL_UART_Transmit(gps_huart, footer, 2, 50);
}

/* ------------------------------------------------------------------ */
/* 公開 API                                                            */
/* ------------------------------------------------------------------ */

void GPS_Init(UART_HandleTypeDef *huart)
{
    gps_huart = huart;
    memset(&gps_data, 0, sizeof(gps_data));
    gps_ubx_asm_init(&gps_ubx_asm);
    gps_pvt_ready_flag = 0;
    gps_dma_old_pos = 0;

    if (gps_huart) {
#if GPS_BRINGUP_MINIMAL
        /* ── 最小收訊模式：不送任何 UBX，USART6 固定鮑率直接收 NMEA ──
         * 對照「當初測試成功」的組態，隔離 460800 協商是否為收不到主因。 */
        HAL_UART_DeInit(gps_huart);
        gps_huart->Init.BaudRate = GPS_MINIMAL_BAUD;
        HAL_UART_Init(gps_huart);
        HAL_Delay(20);
        if (hiwdg.Instance != NULL) {
            HAL_IWDG_Refresh(&hiwdg);
        }
        __HAL_UART_CLEAR_OREFLAG(gps_huart);
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
#else
        if (hiwdg.Instance != NULL) {
            HAL_IWDG_Refresh(&hiwdg);
        }

        /* 1. 定義 UBX 設置指令 Payload */
        /* UBX-CFG-PRT Payload: UART1 鮑率 460800, 8N1, 輸入 UBX+NMEA, 輸出 UBX only (0x0001) */
        const uint8_t UBX_CFG_PRT_460800_PAYLOAD[20] = {
            0x01, 0x00, 0x00, 0x00, 
            0xD0, 0x08, 0x00, 0x00, 
            0x00, 0x08, 0x07, 0x00, 
            0x01, 0x00, 0x01, 0x00, 
            0x00, 0x00, 0x00, 0x00
        };
        
        /* UBX-CFG-RATE Payload: 定位頻率 25 Hz (40ms 測量週期) */
        const uint8_t UBX_CFG_RATE_25HZ_PAYLOAD[6] = {
            0x28, 0x00, 0x01, 0x00, 0x01, 0x00
        };

        /* UBX-CFG-MSG Payload: 開啟 UBX-NAV-PVT (Class 0x01, ID 0x07) 於 UART1 輸出率 1 */
        const uint8_t UBX_CFG_MSG_PVT_PAYLOAD[3] = {
            0x01, 0x07, 0x01
        };

        /* 2. 動態鮑率協商序列 (嘗試以多種鮑率發送設置命令，以相容各類初始狀態；加入 460800 保障熱重啟) */
        const uint32_t try_bauds[] = {38400, 115200, 9600, 460800};
        for (int i = 0; i < 4; i++) {
            /* 切換 MCU UART 至嘗試的鮑率 */
            HAL_UART_DeInit(gps_huart);
            gps_huart->Init.BaudRate = try_bauds[i];
            HAL_UART_Init(gps_huart);
            
            /* 發送變更鮑率與訊息設置命令 */
            send_ubx_cmd(0x06, 0x00, UBX_CFG_PRT_460800_PAYLOAD, sizeof(UBX_CFG_PRT_460800_PAYLOAD));
            HAL_Delay(30);
            if (hiwdg.Instance != NULL) {
                HAL_IWDG_Refresh(&hiwdg);
            }
        }

        /* 3. 將 MCU UART 固定於最終目標鮑率 460800 */
        HAL_UART_DeInit(gps_huart);
        gps_huart->Init.BaudRate = 460800;
        HAL_UART_Init(gps_huart);
        HAL_Delay(20);
        if (hiwdg.Instance != NULL) {
            HAL_IWDG_Refresh(&hiwdg);
        }

        /* 4. 在 460800 鮑率下，發送設置定位更新率為 25Hz 與啟用 UBX-NAV-PVT 輸出命令 */
        send_ubx_cmd(0x06, 0x08, UBX_CFG_RATE_25HZ_PAYLOAD, sizeof(UBX_CFG_RATE_25HZ_PAYLOAD));
        HAL_Delay(20);
        send_ubx_cmd(0x06, 0x01, UBX_CFG_MSG_PVT_PAYLOAD, sizeof(UBX_CFG_MSG_PVT_PAYLOAD));
        HAL_Delay(20);

        /* 5. 啟動 IDLE-line + 循環 DMA 接收 (啟動前清空 ORE/FE 旗標，防止 DMA 因殘留錯誤拒絕啟動) */
        __HAL_UART_CLEAR_OREFLAG(gps_huart);
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
#endif /* GPS_BRINGUP_MINIMAL */
    }
}

uint8_t GPS_Update(void)
{
    if (!gps_pvt_ready_flag) return 0;

    /* 取出就緒 PVT 包（以 NVIC 關閉 UART 中斷做局部臨界區保護） */
    UbxNavPvt_t pvt;
    HAL_NVIC_DisableIRQ(USART6_IRQn);
    memcpy(&pvt, &gps_pvt_ready, sizeof(UbxNavPvt_t));
    gps_pvt_ready_flag = 0;
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    gps_parse_ubx_pvt(&gps_data, &pvt, HAL_GetTick());
    RATE_TICK_GPS();
    return 1;
}

const GPS_Data_t* GPS_GetData(void)
{
    return &gps_data;
}

uint8_t GPS_IsStale(uint32_t timeout_ms)
{
    if (!gps_data.fix_valid) return 1;
    return (HAL_GetTick() - gps_data.last_fix_tick) > timeout_ms;
}

/* ------------------------------------------------------------------ */
/* HAL UART 回呼處理（以 instance 過濾僅處理 USART6）                   */
/* ------------------------------------------------------------------ */

void GPS_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (!gps_huart || huart->Instance != gps_huart->Instance) return;

    if (Size != gps_dma_old_pos) {
        if (Size > gps_dma_old_pos) {
            for (uint16_t i = gps_dma_old_pos; i < Size; i++) GPS_FeedByte(gps_dma_buf[i]);
        } else {
            /* 環形回繞：先處理 old_pos..尾端，再 0..Size */
            for (uint16_t i = gps_dma_old_pos; i < GPS_DMA_BUF_SIZE; i++) GPS_FeedByte(gps_dma_buf[i]);
            for (uint16_t i = 0; i < Size; i++) GPS_FeedByte(gps_dma_buf[i]);
        }
        gps_dma_old_pos = Size;
        if (gps_dma_old_pos >= GPS_DMA_BUF_SIZE) gps_dma_old_pos = 0;
    }
}

void GPS_HandleUartError(UART_HandleTypeDef *huart)
{
    if (gps_huart && huart->Instance == gps_huart->Instance) {
        /* UART 錯誤（常見為 overrun）會中止 DMA 接收 → 清狀態並重啟，避免 GPS RX 死掉 */
        __HAL_UART_CLEAR_OREFLAG(gps_huart);
        gps_ubx_asm_init(&gps_ubx_asm);
        gps_dma_old_pos = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
    }
}

