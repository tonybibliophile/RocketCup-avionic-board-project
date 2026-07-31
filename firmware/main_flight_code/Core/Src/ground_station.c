/*
 * ground_station.c — 地面站接收器主流程（ROLE_GROUND）
 * ===========================================================================
 * 整檔以 #if IS_GROUND 包住：主/備航電編譯為空，零影響。
 *
 * 雙鏈路接收火箭下行 TelemetryPacket_t：
 *   E22 433（USART3 透傳）：中斷接收 → 位元組環形緩衝 → telem_rx 同步FSM
 *   E80 920（SX126x SPI3）：輪詢 DIO1 → LoRaE80_ReadPacket → 同 telem_rx 解析（含 RSSI/SNR）
 * + 讀自身 GPS（USART6），用 gs_timesync 把火箭 tick 對齊到 GPS 紀律的牆鐘。
 * 每筆有效封包組成 GsLogRecord → 三路落地：USB-CDC(CSV) / SD(CSV) / Flash(二進位 append)。
 *
 * GS_USB_SELFTEST=1：略過 LoRa，產生模擬遙測以「相同輸出格式」串流 USB（並寫 SD/Flash），
 * 用於在沒有任何 LoRa 流量下單獨確認 USB 列舉與資料管線正常。
 */
#include "board_config.h"
#if IS_GROUND

#include "ground_station.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

#include "telem_rx.h"
#include "ack_proto.h"     /* 主航電對上行命令的下行 ACK 幀（與遙測同流並排解析） */
#include "gs_timesync.h"
#include "gs_log.h"
#include "gs_lora_test.h"
#include "gps.h"
#include "lora_e22.h"
#include "lora_e80.h"
#include "w25qxx.h"
#include "flash_ring_math.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#ifndef GS_USB_SELFTEST
#define GS_USB_SELFTEST 0          /* 1 = USB 自測模式（不靠 LoRa） */
#endif
#define GS_TIMESYNC_EMA_SHIFT  3   /* rocket↔ground 偏移 EMA：alpha = 1/8 */
#define GS_POLL_DELAY_MS       2   /* 主迴圈輪詢週期（~500Hz） */

extern UART_HandleTypeDef huart3;  /* E22 433 透傳（main.c 定義） */
extern IWDG_HandleTypeDef hiwdg;   /* 看門狗（main.c 定義） */
extern uint8_t lora433_ok;         /* E22 433MHz 模組就緒狀態（main.c 定義） */
extern uint8_t lora920_ok;         /* E80 920MHz 模組就緒狀態（main.c 定義） */

/* ---- 狀態 ---- */
static TelemRx_t     s_rx433;
static TelemRx_t     s_rx920;
static AckRx_t       s_ack433;   /* 命令 ACK 解析（與遙測同一位元組流並排，sync 0xAC/0xCA） */
static AckRx_t       s_ack920;
static GsTimeSync_t  s_ts;
static uint32_t      s_last_fix_tick = 0;
static uint32_t      s_stat_433_cnt = 0;
static uint32_t      s_stat_920_cnt = 0;

/* ---- 920 靜默看門狗（見 lora_e80.c LoRaE80_ReadPacket 的 DIO1 latch 說明）----
 * 修好 IRQ 清除與 RxReady() 電平查詢後，理論上 920 不會再永久卡死；這裡再加一層
 * 「超過 N 毫秒沒收到 920 封包就自己重新 StartRx」，防的是任何未來又漏清 IRQ、
 * 或 LR1121 本身進入某種需要重新武裝才能脫離的狀態。
 * s_920_rearm_cnt 一定要印出來（見下方 [GS_STAT]）：火箭沒開機時本來就會一直
 * 靜默，這個機制會固定每 GS_E80_REARM_MS 觸發一次——不印出來，事後看 log 會分不清
 * 「鏈路本來就沒訊號」跟「真的卡死被自動救回」，等於用自動復原把真實問題蓋掉。 */
#define GS_E80_REARM_MS  5000U
static uint32_t      s_last_920_tick  = 0;   /* 上次成功交付 920 封包的時刻 */
static uint32_t      s_920_rearm_cnt  = 0;   /* 累計自動重新武裝次數 */

/* 大緩衝置於檔案範圍，降低任務堆疊壓力（地面站單一任務，安全） */
static char    s_row[GS_LOG_CSV_MAX];
static uint8_t s_e80buf[255];

/* ---- 指示燈（板上三顆，地面站用途）----
 *   PE2 LED_SYS    : 心跳閃爍（1Hz）= 韌體存活、主迴圈在跑
 *   PE3 LED_State1 : 收到 433 (E22) 封包 = 逐包翻轉，隨收包速率閃爍
 *   PE4 LED_State2 : 收到 920 (E80) 封包 = 逐包翻轉，隨收包速率閃爍
 * 規格表標「低/高電平控制」；此處採 active-high（SET=亮，GPIO init 為 RESET=滅）。
 * 若實際硬體為 active-low，將 GS_LED_ON/GS_LED_OFF 對調即可。
 *
 * ★收包燈採「逐包翻轉」而非「收到後亮 N ms」：920 實測可達 8–10Hz（週期
 *   100–125ms），若採固定亮燈時長，只要亮燈時長 >= 封包週期，下一包就會在
 *   前一次亮燈熄滅前又把燈重新點亮，脈衝彼此重疊、肉眼看起來像恆亮，感覺
 *   不出速率、甚至像「跟不上」。逐包翻轉不管封包多快都一定有可見邊沿。 */
#define GS_LED_ON            GPIO_PIN_SET
#define GS_LED_OFF           GPIO_PIN_RESET
#define GS_LED1_Pin          GPIO_PIN_3   /* PE3 LED_State1（main.h 未命名，直接用腳號） */
#define GS_RX_LED_TIMEOUT_MS 300U         /* 超過此時間沒收到新封包，燈直接熄滅（判定鏈路中斷） */
#define GS_HEARTBEAT_MS      500U         /* LED_SYS 心跳半週期（1Hz 閃） */

static volatile uint32_t s_last_pkt_tick_433 = 0;   /* 最後一筆 433 (E22) 有效封包 tick */
static volatile uint32_t s_last_pkt_tick_920 = 0;   /* 最後一筆 920 (E80) 有效封包 tick */
static volatile uint8_t  s_led_433_on = 0;          /* 每收一包 433 翻轉一次 */
static volatile uint8_t  s_led_920_on = 0;          /* 每收一包 920 翻轉一次 */

/* 依現況驅動三顆 LED。每個主迴圈週期呼叫一次。 */
static void gs_leds_update(uint32_t now)
{
    static uint32_t hb_tick = 0;
    static uint8_t  hb_on   = 0;
    if ((now - hb_tick) >= GS_HEARTBEAT_MS) {       /* PE2：心跳 */
        hb_tick = now;
        hb_on ^= 1U;
        HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, hb_on ? GS_LED_ON : GS_LED_OFF);
    }
    uint8_t rx_433 = s_led_433_on &&
        (s_last_pkt_tick_433 != 0) && ((now - s_last_pkt_tick_433) < GS_RX_LED_TIMEOUT_MS);
    HAL_GPIO_WritePin(GPIOE, GS_LED1_Pin,            /* PE3：433 收包活動 */
                      rx_433 ? GS_LED_ON : GS_LED_OFF);
    uint8_t rx_920 = s_led_920_on &&
        (s_last_pkt_tick_920 != 0) && ((now - s_last_pkt_tick_920) < GS_RX_LED_TIMEOUT_MS);
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin,  /* PE4：920 收包活動 */
                      rx_920 ? GS_LED_ON : GS_LED_OFF);
}

/* ---- USART3（E22）位元組環形緩衝：ISR 推入、任務取出 ---- */
#define U3_RING_SZ 1024U
static volatile uint8_t  s_u3_ring[U3_RING_SZ];
static volatile uint16_t s_u3_head = 0, s_u3_tail = 0;
/* USART3 循環 DMA 接收緩衝。
 * ★舊版是 ReceiveToIdle_IT：每次事件都得在 callback 內重新掛載才能繼續收，重掛期間
 *   RX 未武裝，該窗內到達的位元組會漏接或觸發 ORE；且緩衝一度小於 TELEM_PACKET_SIZE，
 *   封包本體會被 HAL 自己攔腰打斷。改成 circular DMA + 位置差分（比照 link_hw.c 的
 *   Link_OnRxEvent 與 gps.c，專案內已驗證的作法）：DMA 全程武裝、無重掛空檔，
 *   IDLE 事件只是「來取新位元組」的通知，不再具有任何 framing 意義。
 * 大小取 2 包餘裕，讓單次事件之間即使延遲數十毫秒也不會被 DMA 追過（覆寫）。 */
#define U3_RXBUF_SZ (2U * (TELEM_PACKET_SIZE) + 32U)
static uint8_t           s_u3_rxbuf[U3_RXBUF_SZ];   /* circular DMA 目的緩衝 */
static volatile uint16_t s_u3_dma_old_pos = 0;      /* 上次已取到的 DMA 寫入位置 */
static volatile uint32_t s_u3_rx_bytes = 0;  /* USART3(E22 433) 累計收到的原始位元組數（含雜訊） */

static void u3_push(uint8_t b)
{
    uint16_t nh = (uint16_t)((s_u3_head + 1U) % U3_RING_SZ);
    if (nh != s_u3_tail) { s_u3_ring[s_u3_head] = b; s_u3_head = nh; }  /* 滿則丟 */
}
static int u3_pop(uint8_t *b)
{
    if (s_u3_tail == s_u3_head) return 0;
    *b = s_u3_ring[s_u3_tail];
    s_u3_tail = (uint16_t)((s_u3_tail + 1U) % U3_RING_SZ);
    return 1;
}

/* 循環 DMA 接收事件：Size = 自緩衝起點至目前 DMA 寫入位置的累計位元組數。
 * 以 s_u3_dma_old_pos 環形差分取出新位元組推進 ring（與 link_hw.c / gps.c 同法）。 */
void GroundStation_OnUart3RxEvent(uint16_t Size)
{
    uint16_t old = s_u3_dma_old_pos;
    if (Size == old) return;

    if (Size > old) {
        for (uint16_t i = old; i < Size; i++) u3_push(s_u3_rxbuf[i]);
        s_u3_rx_bytes += (uint32_t)(Size - old);
    } else {
        for (uint16_t i = old; i < U3_RXBUF_SZ; i++) u3_push(s_u3_rxbuf[i]);
        for (uint16_t i = 0; i < Size; i++)          u3_push(s_u3_rxbuf[i]);
        s_u3_rx_bytes += (uint32_t)(U3_RXBUF_SZ - old) + Size;
    }
    s_u3_dma_old_pos = (Size >= U3_RXBUF_SZ) ? 0U : Size;
}

/* 位元組流不連續旗標：ISR/設定模式設、任務端消化。位元組流中間缺了一段時，
 * 解析器極可能卡在半包狀態（idx 停在中途），若不重置會沿著錯誤邊界一直錯下去。
 * 不在 ISR 內直接 TelemRx_Init：任務可能正在 TelemRx_FeedAny 中途，會 race。 */
static volatile uint8_t s_u3_rx_desync = 0;

/* 重新掛載 USART3 循環 DMA 接收。兩個呼叫來源：
 *   1. UART 錯誤復原（ORE）——HAL 會中止 RX 且不會自己重掛。
 *   2. lora_e22.c 每次離開設定模式後（見 LoRaE22_SetRxRearmCallback）——設定模式
 *      要改 baud 而呼叫 HAL_UART_Init，那會把 RxState 打回 READY，進行中的
 *      ReceiveToIdle 就此失效；不重掛的話 `e22 freq/pwr/air` 之後 433 再也收不到。
 * 兩種情況位元組流都斷過，故一律標記 desync 讓任務端重置解析器。 */
static void gs_u3_rx_rearm(void)
{
    HAL_UART_AbortReceive(&huart3);   /* 確保舊的 DMA 接收確實停掉，否則重掛會回 HAL_BUSY */
    s_u3_dma_old_pos = 0;
    s_u3_rx_desync   = 1;             /* 比照 link_hw.c Link_OnError 的 LinkRx_Init */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_u3_rxbuf, sizeof(s_u3_rxbuf));
}

/* USART3(E22 433 RX) 錯誤復原：清 ORE/雜訊旗標並重啟循環 DMA 接收。
 * 由 main.c 的 HAL_UART_ErrorCallback 在 USART3 出錯時轉接。
 * 若不做：收包時 printf/SD 寫入很慢，E22 以 115200 繼續吐下一包 → USART3 溢位(ORE)
 * → HAL 中止 RX 且不會自己重掛 → raw 從此凍結、之後完全收不到。此為地面站
 * 「收一包就死」的根因修正（清旗標做法比照 main.c USART2 既有寫法）。 */
void GroundStation_OnUart3Error(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    (void)huart3.Instance->SR;
    (void)huart3.Instance->DR;
    s_u3_rx_desync = 1;   /* 不論哪一種錯誤，位元組流都缺了一段 → 解析器要重置 */

    /* ★DMA 接收下 ORE 屬「非阻斷錯誤」：HAL 只發錯誤回呼，DMA 接收其實還活著。
     * 這種情況不可重掛（會回 HAL_BUSY），更不可把 s_u3_dma_old_pos 歸零——DMA 的
     * 寫入位置並沒有跟著回到 0，歸零會讓下一次事件把一整段舊資料當新的重讀一遍。
     * 只有 HAL 真的把接收停掉（RxState 已非 BUSY_RX）時才需要、也才能重掛。
     * ★但「RxState != BUSY_RX」不等於「現在可以重掛」——lora_e22.c 的設定模式一開始
     *   就用 HAL_UART_AbortReceive() 把 RxState 打回 READY，一路到離開設定模式前都
     *   是這個狀態；若這個 ISR（不受任何任務優先權節制）在這段窗口內被 M1 切換/baud
     *   改變觸發的雜訊 ORE 打進來，重掛的 DMA 會硬生生把 lora_e22.c 正在阻塞輪詢等待
     *   的模組回應位元組搶走（DMA 永遠比軟體輪詢快），造成 `e22 freq/pwr/air` 每次
     *   都 st=3(HAL_TIMEOUT)、回讀全 0。故先查 LoRaE22_IsInConfigMode()：正在設定
     *   模式就不搶，離開時 lora_e22.c 自己會呼叫這支重掛。 */
    if (huart3.RxState != HAL_UART_STATE_BUSY_RX) {
        if (!LoRaE22_IsInConfigMode()) {
            gs_u3_rx_rearm();
        }
    }
}

/* ---- SD（FatFS：CSV 人讀 log + 完整二進位原始封包） ----
 * 兩個檔案同一 session 配對（同一個索引 i）：
 *   GSLOG%03d.CSV — 人讀摘要，欄位為 GsLog_FormatCsvRow() 子集（不含 gyro/mag/hg raw）。
 *   GSRAW%03d.BIN — 逐筆 append 完整 GsLogRecord_t（含 79-byte 原始 TelemetryPacket_t，
 *                   與 Flash 上寫入的格式一模一樣，含 magic/CRC），供事後解出 gyro/mag/
 *                   高G/原始氣壓等 CSV 沒收錄的欄位。 */
static FIL      s_sd_file;
static uint8_t  s_sd_ok = 0;
static uint32_t s_sd_rows = 0;

static FIL      s_sd_raw_file;
static uint8_t  s_sd_raw_ok = 0;
static uint32_t s_sd_raw_rows = 0;

static void gs_sd_open(void)
{
    char name[16], raw_name[16];
    FILINFO fno;
    HAL_IWDG_Refresh(&hiwdg);
    if (f_mount(&SDFatFS, SDPath, 1) != FR_OK) {
        HAL_IWDG_Refresh(&hiwdg);
        return;
    }
    HAL_IWDG_Refresh(&hiwdg);
    for (int i = 0; i < 1000; i++) {
        snprintf(name, sizeof(name), "GSLOG%03d.CSV", i);
        snprintf(raw_name, sizeof(raw_name), "GSRAW%03d.BIN", i);
        /* 兩個都要未使用才取這個索引，確保 CSV/BIN 是同一 session 配對 */
        if (f_stat(name, &fno) == FR_NO_FILE && f_stat(raw_name, &fno) == FR_NO_FILE) break;
    }
    HAL_IWDG_Refresh(&hiwdg);
    if (f_open(&s_sd_file, name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
        char hdr[GS_LOG_CSV_MAX];
        int n = GsLog_CsvHeader(hdr, sizeof(hdr));
        UINT bw;
        if (n > 0) f_write(&s_sd_file, hdr, (UINT)n, &bw);
        f_sync(&s_sd_file);
        s_sd_ok = 1;
    }
    HAL_IWDG_Refresh(&hiwdg);
    if (f_open(&s_sd_raw_file, raw_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
        s_sd_raw_ok = 1;
    }
    HAL_IWDG_Refresh(&hiwdg);
}
static void gs_sd_write(const char *row, uint16_t n)
{
    if (!s_sd_ok) return;
    UINT bw;
    f_write(&s_sd_file, row, (UINT)n, &bw);
    if ((++s_sd_rows % 16U) == 0U) f_sync(&s_sd_file);   /* 定期 flush 降低掉電損失 */
}
static void gs_sd_write_raw(const GsLogRecord_t *rec)
{
    if (!s_sd_raw_ok) return;
    UINT bw;
    f_write(&s_sd_raw_file, rec, (UINT)GS_LOG_RECORD_SIZE, &bw);
    if ((++s_sd_raw_rows % 16U) == 0U) f_sync(&s_sd_raw_file);   /* 定期 flush 降低掉電損失 */
}

/* ---- Flash（W25Q128 ring 區順序 append；用前才擦的 erase-ahead） ---- */
static uint32_t s_fl_addr;        /* 寫入頭 */
static uint32_t s_fl_erased_end;  /* 已擦至此位址（exclusive） */

static void gs_flash_init(void)
{
    s_fl_addr = FLASH_RINGBUF_ADDR;
    s_fl_erased_end = FLASH_RINGBUF_ADDR;   /* 尚未擦任何 sector */
}

/* 手動 `flash erase`（gs_lora_test.c）在呼叫 FlashRing_EraseAll() 整段擦淨後呼叫本函式，
 * 重設寫入頭。★注意 s_fl_erased_end 設為 END+1（已擦到底），不是 gs_flash_init() 那個
 * ADDR（尚未擦任何 sector）——後者會讓 gs_flash_append() 在收包當下逐個 sector 現場擦除
 * （每次 ~數十~數百 ms），把剛用 FlashRing_EraseAll() 省下的一次性停頓又分散成每包卡頓。 */
void GroundStation_FlashResetAfterErase(void)
{
    s_fl_addr = FLASH_RINGBUF_ADDR;
    s_fl_erased_end = FLASH_RINGBUF_END + 1UL;
}
static void gs_flash_ensure_erased(uint32_t addr, uint32_t n)
{
    while (addr + n > s_fl_erased_end) {
        if (W25QXX_EraseSector(s_fl_erased_end) != W25QXX_OK) break;
        s_fl_erased_end += FLASH_RING_SECTOR_SIZE;
    }
}
static void gs_flash_append(const GsLogRecord_t *rec)
{
    if (s_fl_addr + GS_LOG_RECORD_SIZE > FLASH_RINGBUF_END + 1UL) {
        s_fl_addr = FLASH_RINGBUF_ADDR;          /* 回繞，重新從頭擦 */
        s_fl_erased_end = FLASH_RINGBUF_ADDR;
    }
    gs_flash_ensure_erased(s_fl_addr, GS_LOG_RECORD_SIZE);
    if (W25QXX_WriteData(s_fl_addr, (const uint8_t *)rec, GS_LOG_RECORD_SIZE) == W25QXX_OK) {
        s_fl_addr += GS_LOG_RECORD_SIZE;
    }
}

/* ---- USB-CDC（best-effort，PC 沒讀就丟） ---- */
static void gs_usb_send(const uint8_t *buf, uint16_t n)
{
    for (int i = 0; i < 8; i++) {
        if (CDC_Transmit_FS((uint8_t *)buf, n) == USBD_OK) return;
        osDelay(1);   /* 前一筆未送完，稍等再試 */
    }
}

/* ---- 共用：一筆封包 → 對齊時間 → 組紀錄 → 三路落地 ---- */
static void gs_handle_packet(uint8_t link, const TelemetryPacket_t *pkt,
                             int16_t rssi, int16_t snr)
{
    uint32_t rx_tick = HAL_GetTick();
    if (link == GS_LINK_920) {
        s_last_pkt_tick_920 = rx_tick;  /* 接收活動指示燈（PE4）用 */
        s_led_920_on ^= 1U;
    } else {
        s_last_pkt_tick_433 = rx_tick;  /* 接收活動指示燈（PE3）用 */
        s_led_433_on ^= 1U;
    }
    GsTimeSync_OnPacket(&s_ts, pkt->tick_ms, rx_tick, GS_TIMESYNC_EMA_SHIFT);
    uint32_t rx_utc = GsTimeSync_GroundUtcMs(&s_ts, rx_tick);
    uint32_t al_utc = GsTimeSync_RocketAlignedUtcMs(&s_ts, pkt->tick_ms);
    int32_t  off    = GsTimeSync_Offset(&s_ts);

    const GPS_Data_t *g = GPS_GetData();
    GsLogRecord_t rec;
    GsLog_BuildRecord(&rec, link, rssi, snr, rx_tick, rx_utc, al_utc, off,
                      g->lat_1e6, g->lon_1e6, (int16_t)g->altitude_m,
                      g->satellites, g->fix_valid, pkt);

    int n = GsLog_FormatCsvRow(s_row, sizeof(s_row), &rec);
    if (n > (int)sizeof(s_row) - 1) n = (int)sizeof(s_row) - 1;  /* 防 snprintf 截斷回傳值溢位 */
    if (n > 0) {
        gs_usb_send((const uint8_t *)s_row, (uint16_t)n);
        gs_sd_write(s_row, (uint16_t)n);
    }
    gs_sd_write_raw(&rec);
    gs_flash_append(&rec);

    /* 更新通訊測試統計 */
    GsLoraTest_UpdateStats(link, rssi, snr, 1 /* crc_ok */);
    if (link == GS_LINK_920) {
        s_stat_920_cnt++;
        s_last_920_tick = rx_tick;   /* 920 靜默看門狗計時基準 */
    } else {
        s_stat_433_cnt++;
    }

    /* 姿態四元數（EKF 估測，×10000 int16）：封包本就攜帶（telemetry.h ekf_q0..q3），
     * 但過去從未印出，GUI 端因此完全沒有姿態資料。★不要塞進下面的 [GS_PKT]——那行已經
     * 400+ bytes（USB_LOG_LINE_MAX 從 256 加大到 512 前還會被硬截斷，見 main.c _write()
     * 註解），加大後也所剩無幾、以後還會有人繼續加欄位。改印成獨立短行（~40 bytes），
     * 完全不影響 [GS_PKT] 本身，舊 GUI/舊解析器也不會受影響（純新增一行）。seq 供 GUI 端
     * 與 [GS_PKT] 對應/去重用。 */
    printf("[GS_ATT] seq:%u q:%d,%d,%d,%d\r\n",
           (unsigned)pkt->seq,
           (int)pkt->ekf_q0, (int)pkt->ekf_q1, (int)pkt->ekf_q2, (int)pkt->ekf_q3);

    /* 及時（實時）控制台印出收到的下行遙測封包摘要。
     * vz / pos / galt：GUI 圖表（EKF 垂直速度）與 GPS 地圖（火箭經緯度）需要，
     * 封包本就攜帶，此處補印出（pos 格式與航電 [GPS] 行一致：±d.6f）。 */
    {
        char lat_sign = (pkt->gps_lat_1e6 < 0) ? '-' : '+';
        char lon_sign = (pkt->gps_lon_1e6 < 0) ? '-' : '+';
        uint32_t lat_abs = (pkt->gps_lat_1e6 < 0) ? (uint32_t)(-pkt->gps_lat_1e6) : (uint32_t)pkt->gps_lat_1e6;
        uint32_t lon_abs = (pkt->gps_lon_1e6 < 0) ? (uint32_t)(-pkt->gps_lon_1e6) : (uint32_t)pkt->gps_lon_1e6;
        /* ★2026-07-30：peer 段拿掉 EKF 高度/速度(ph/pv)、丟包率(ploss)、高G(paz)——
         * 對端狀態一律改看 VF（開傘決策實際採用的估計器），見 telemetry.h peer_vf_h_cm
         * 註解。EKF/丟包率/高G 仍在板間鏈路本身可查（USB 直連該板的 [LINK] 行）。 */
        /* ★max/mvel/macc＝火箭端全速率追蹤的滾動極值（見 telemetry.h）。務必印出來——
         * 下鏈只有約 2Hz，抓不到真正的頂點與峰值 G，這三個數字才是「飛多高/多快/幾 G」
         * 的唯一可信來源，火箭無法回收時尤其如此。 */
        printf("[GS_PKT] link:%uMHz rssi:%d snr:%d seq:%u fsm:%u alt:%dcm vz:%dcms baro:%dcm bat:%dmV "
               "vfh:%dcm vfv:%dcms max:%um mvel:%dms macc:%ucg dalt:%dm malt:%dm "
               "gps:%u/%u pos:%c%lu.%06lu,%c%lu.%06lu galt:%dm accel:%d,%d,%d hg:%dcg "
               "peer:%u pflags:0x%02X plink:0x%02X "
               "pbaro:%dcm pvfh:%dcm pvfv:%dcms pbarb:%u prof:0x%02X armf:0x%02X\r\n",
               (unsigned)((link == GS_LINK_920) ? 920U : 433U),
               (int)rssi, (int)snr, (unsigned)pkt->seq, (unsigned)pkt->fsm_state,
               (int)pkt->ekf_pos_z_cm, (int)pkt->ekf_vel_z_cms, (int)pkt->baro_alt_cm,
               (unsigned)pkt->bat_mv,
               (int)pkt->vf_pos_z_cm, (int)pkt->vf_vel_z_cms,
               (unsigned)pkt->max_alt_m, (int)pkt->max_vel_ms, (unsigned)pkt->max_acc_cg,
               (int)pkt->drogue_alt_m, (int)pkt->main_alt_m,   /* -32768 = 未開傘 */
               (unsigned)pkt->gps_sats, (unsigned)pkt->gps_fix,
               lat_sign, (unsigned long)(lat_abs / 1000000U), (unsigned long)(lat_abs % 1000000U),
               lon_sign, (unsigned long)(lon_abs / 1000000U), (unsigned long)(lon_abs % 1000000U),
               (int)pkt->gps_alt_m,
               (int)pkt->imu_ax_mg, (int)pkt->imu_ay_mg, (int)pkt->imu_az_mg,
               (int)pkt->hg_mag_cg,
               (unsigned)pkt->peer_fsm_state, (unsigned)pkt->peer_flags,
               (unsigned)pkt->peer_link,
               (int)pkt->peer_baro_cm,
               (int)pkt->peer_vf_h_cm, (int)pkt->peer_vf_v_cms,
               /* ★2026-07-31 新增 armf：航電開機不再自動擦 flash，池未達標時 ARM 會被擋。
                * 這一位（TELEM_ARM_NEED_ERASE）讓地面站 GUI 在使用者按 ARM「之前」就能跳
                * 橫幅提醒先擦除。純字串尾端新增欄位，舊解析器不受影響（封包格式未動，
                * 不需三板同燒）。 */
               (unsigned)pkt->peer_bench_arb, (unsigned)pkt->profile_flags,
               (unsigned)pkt->arm_flags);
    }
}

/* ---- 433 收包延後交付（等模組附加的 RSSI 位元組）----------------------------
 * E22 開啟 REG3 bit7 後，每收一包會在酬載「之後」再吐 1 個 RSSI 位元組。該位元組
 * 在 UART 上比封包最後一個位元組晚整整一個字元時間才到：9600bps 下約 1.04ms。
 * 而主迴圈是 tight-drain（while(u3_pop)），封包湊滿的當下環形緩衝通常還沒有它。
 * ★ 舊寫法在封包湊滿當下「立刻 u3_pop()，拿不到就放棄」——在 9600bps 幾乎必定
 *   拿不到，只有舊韌體時代 UART=115200bps（模組整包 burst 吐出、87µs/byte，
 *   湊滿時 RSSI 早已在緩衝裡）才碰巧會成功。實測 log 完全吻合：有效 RSSI 只在
 *   UART=115200 那段出現過，UART=9600 那段一次都沒有。
 * 改法：湊滿封包後先暫存不交付，等下一個位元組到達才當 RSSI 取用並一起交付；
 * 逾時則以 N/A 交付避免卡住。如此與 UART baud 無關，任何速率都正確。 */
static TelemetryPacket_t s_pend_pkt;        /* 等待 RSSI 位元組的暫存封包 */
static uint8_t           s_pend_crc_ok = 0; /* 該封包的 CRC 結果 */
static uint32_t          s_pend_tick   = 0; /* 進入等待的時刻（逾時用） */
static uint8_t           s_pend_valid  = 0; /* 1 = 有封包正在等 RSSI */

/* ★2026-07-28 修正 #3：只有 CRC 過的封包才進延後交付。
 * datasheet bit7 是「每一次無線接收」都附加 RSSI byte，不論我們自己的應用層
 * CRC-16 過不過——CRC 失敗有兩種可能成因：(a) 真的是一筆完整、對齊正確的封包，
 * 只是空中誤碼幾個位元；(b) sync 對齊本身就是巧合湊出來的假封包（雜訊或壞包
 * 裡剛好出現 A5 5A），這種情況下「湊滿 116 bytes」跟模組實際的無線接收事件
 * 邊界根本對不上，此時去吃下一個位元組當 RSSI，吃到的常常是下一筆封包的
 * sync0，讓一次誤判擴大成連環錯位（正回饋失步）。CRC 失敗一律當場以 N/A 交付、
 * 不消耗下一個位元組，情願少一筆 RSSI，不要放大既有的錯位。 */

/* ★2026-07-28 修正 #2：命令 ACK 幀也是一次無線接收，bit7 開啟時模組同樣會在
 * ACK 之後附加一個 RSSI byte，先前沒有任何人消化它，導致下一個位元組（通常是
 * 下一包遙測的 sync0）被平白吃掉。用一個獨立旗標處理（ACK 不需要用到 RSSI 值，
 * 純粹丟棄即可），同樣有逾時保護避免卡死。 */
static uint32_t          s_pend_ack_tick  = 0;
static uint8_t           s_pend_ack_skip  = 0; /* 1 = 下一個位元組是 ACK 的 RSSI byte，丟棄 */

/* 交付一筆 433 封包：CRC 過 → 正常落地並印 [GS_PKT]；CRC 失敗 → 只印供人工判讀。 */
static void gs_deliver_433(const TelemetryPacket_t *pkt, uint8_t crc_ok, int16_t rssi)
{
    if (crc_ok) {
        gs_handle_packet(GS_LINK_433, pkt, rssi, GS_SNR_NA);
    } else {
        /* ★433 的 CRC 失敗以前從來沒進 [STATS]（gs_handle_packet 永遠傳 crc_ok=1），
         * 所以 [STATS] 的 433 crc_err 結構上恆為 0、完全看不出鏈路品質。此處補上。 */
        GsLoraTest_UpdateStats(GS_LINK_433, GS_RSSI_NA, GS_SNR_NA, 0 /* crc_err */);
        /* CRC 失敗也印（不落地 SD/Flash、不動時間同步），供人工看收到什麼 */
        printf("[GS_PKT?] link:433MHz CRC_BAD rssi:%d seq:%u fsm:%u alt:%dcm baro:%dcm bat:%dmV\r\n",
               (int)rssi, (unsigned)pkt->seq, (unsigned)pkt->fsm_state,
               (int)pkt->ekf_pos_z_cm, (int)pkt->baro_alt_cm, (unsigned)pkt->bat_mv);
    }
}

/* ---- 433 原始位元組 hex dump（`e22 dump on`，見 gs_lora_test.c）--------------
 * 印出 u3_pop() 吐出的每一個原始位元組，不論後續被解析成什麼——用來直接肉眼
 *核對 RSSI byte 到底是不是「固定一個、在封包尾端」，不必再靠推論或統計反推。
 * 每 16 個位元組換行，行首標流水序號方便對齊封包邊界。 */
static void gs_raw_dump_byte(uint8_t b)
{
    static uint32_t s_dump_idx = 0;
    if ((s_dump_idx % 16U) == 0U) {
        printf("\r\n[E22RAW] %6lu: ", (unsigned long)s_dump_idx);
    }
    printf("%02X ", (unsigned)b);
    s_dump_idx++;
}

#if GS_USB_SELFTEST
/* 模擬器：以 ~10Hz 產生決定性「走動」遙測，串流相同格式驗證 USB（不碰 LoRa）。 */
static void gs_usb_selftest_loop(void)
{
    uint8_t  seq = 0;
    uint32_t tick = 0;
    int32_t  alt_m = 0;
    int      dir = 1;

    GsTimeSync_OnGpsFix(&s_ts, 120000U, HAL_GetTick());  /* 假錨點 12:00:00 */

    for (;;) {
        TelemetryPacket_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.sync0 = TELEM_SYNC0;
        pkt.sync1 = TELEM_SYNC1;
        pkt.seq = seq++;
        pkt.tick_ms = tick;
        pkt.fsm_state = (alt_m > 1000) ? 3 : 1;
        pkt.ekf_pos_z_cm = alt_m * 100;
        pkt.baro_alt_cm = alt_m * 100;
        pkt.gps_lat_1e6 = 25033000;
        pkt.gps_lon_1e6 = 121564000;
        pkt.gps_alt_m = (int16_t)alt_m;
        pkt.gps_sats = 9;
        pkt.gps_fix = 1;
        pkt.bat_mv = 11800;
        pkt.crc16 = crc16_ccitt_false((const uint8_t *)&pkt, (uint16_t)(TELEM_PACKET_SIZE - 2));

        uint8_t link = (seq & 1U) ? GS_LINK_920 : GS_LINK_433;     /* 兩鏈路交替 */
        int16_t rssi = (link == GS_LINK_920) ? (int16_t)-85 : GS_RSSI_NA;
        int16_t snr  = (link == GS_LINK_920) ? (int16_t)40  : GS_SNR_NA;
        gs_handle_packet(link, &pkt, rssi, snr);

        alt_m += dir * 20;                          /* 高度拋物線升降 */
        if (alt_m >= 3000) dir = -1;
        if (alt_m <= 0) { alt_m = 0; dir = 1; }
        tick += 100;

        gs_leds_update(HAL_GetTick());      /* 心跳 + 433/920 接收活動仍會動 */
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(100);   /* 10 Hz */
    }
}
#endif /* GS_USB_SELFTEST */

void GroundStation_Run(void)
{
    printf("\r\n[ROLE] GROUND_STATION  RX=E22(433)+E80(920)  GPS+SD+Flash+USB-CDC%s\r\n",
           GS_USB_SELFTEST ? "  [USB SELFTEST]" : "");
    printf("[LED] PE2 心跳(存活) / PE3 GPS定位 / PE4 接收活動\r\n");
    printf("[GS_LORA_INIT] 433MHz(E22):%s | 920MHz(E80):%s\r\n",
           lora433_ok ? "OK (Ready)" : "FAIL/OFF",
           lora920_ok ? "OK (Ready)" : "FAIL/OFF");
    if (lora433_ok) {
        LoRaE22_PrintConfig();
    }
    if (lora920_ok) {
        LoRaE80_PrintConfig();
    }

    TelemRx_Init(&s_rx433);
    TelemRx_Init(&s_rx920);
    AckRx_Init(&s_ack433);
    AckRx_Init(&s_ack920);
    s_last_920_tick = HAL_GetTick();   /* 從現在起算靜默時間，避免開機瞬間誤觸發重新武裝 */
    GsTimeSync_Init(&s_ts);
    gs_sd_open();
    gs_flash_init();
    GsLoraTest_Init();   /* 啟動 UART2 命令介面 + 統計 */

#if GS_USB_SELFTEST
    gs_usb_selftest_loop();   /* 不返回 */
#else
    /* 設定模式（`e22 freq/pwr/air`）會呼叫 HAL_UART_Init 而讓接收失效，
     * 註冊重掛回呼讓驅動離開設定模式後自動救回來。必須在啟動接收之前註冊。 */
    LoRaE22_SetRxRearmCallback(gs_u3_rx_rearm);

    /* 啟動 E22 USART3 循環 DMA 接收（E80 已於 main 進入連續 RX） */
    gs_u3_rx_rearm();

    for (;;) {
        /* GPS：新 fix 時更新地面牆鐘錨點 */
        GPS_Update();
        const GPS_Data_t *g = GPS_GetData();
        if (g->fix_valid && g->last_fix_tick != s_last_fix_tick) {
            GsTimeSync_OnGpsFix(&s_ts, g->utc_hhmmss, g->last_fix_tick);
            s_last_fix_tick = g->last_fix_tick;
        }

        /* E22 433：取出環形緩衝餵入同步FSM。
         * 用 TelemRx_FeedAny「不過濾」：湊滿一筆就交出，CRC 失敗也印出來供除錯（看訊號品質）。 */
        uint8_t b;
        TelemetryPacket_t pkt;
        uint8_t dump_on = GsLoraTest_RawDumpEnabled();

        /* UART 溢位後位元組流缺了一段：解析器可能卡在半包，重置回「重找 sync」狀態，
         * 並丟掉正在等 RSSI 的暫存（那個 RSSI 位元組多半已隨溢位一起掉了）。 */
        if (s_u3_rx_desync) {
            s_u3_rx_desync = 0U;
            TelemRx_Init(&s_rx433);
            AckRx_Init(&s_ack433);
            s_pend_valid    = 0U;
            s_pend_ack_skip = 0U;
        }

        while (u3_pop(&b)) {
            if (dump_on) gs_raw_dump_byte(b);

            /* 上一輪湊滿的封包正在等它的 RSSI 位元組：這個位元組就是了。
             * 取用後 continue —— 它是模組附加的頻外資料，不可餵進任何解析器。 */
            if (s_pend_valid) {
                s_pend_valid = 0U;
                gs_deliver_433(&s_pend_pkt, s_pend_crc_ok,
                               (int16_t)(-(256 - (int)b)));
                continue;
            }
            /* 上一輪的 ACK 幀正在等它的 RSSI 位元組：丟棄即可，ACK 不需要 RSSI 值。 */
            if (s_pend_ack_skip) {
                s_pend_ack_skip = 0U;
                continue;
            }
            /* 命令 ACK（sync 0xAC/0xCA）與遙測（0xA5/0x5A）並排解析：AckRx 忽略非自身 sync
             * 位元組，互不干擾。解出即印 [ACK]（走地面 printf→USART2，GUI 看得到）。 */
            {
                uint8_t aseq = 0, ast = 0, alen = 0;
                char    atext[ACK_TEXT_MAX + 1];
                if (AckRx_Feed(&s_ack433, b, &aseq, &ast, atext, &alen)) {
                    printf("[ACK] link:433 seq:%u status:%s cmd:\"%s\"\r\n",
                           (unsigned)aseq, ack_status_str(ast), atext);
                    if (LoRaE22_RssiByteEnabled()) {
                        s_pend_ack_tick  = HAL_GetTick();
                        s_pend_ack_skip  = 1U;
                    }
                }
            }
            uint8_t crc_ok = 0;
            if (TelemRx_FeedAny(&s_rx433, b, &pkt, &crc_ok)) {
                if (crc_ok && LoRaE22_RssiByteEnabled()) {
                    /* ★ 延後交付：RSSI 位元組還沒到，先把封包暫存起來（見 s_pend_pkt 註解）。
                     * 只對 CRC 過的封包延後——CRC 失敗的湊滿事件不保證是真的無線接收
                     * 邊界，見上方修正 #3 註解。 */
                    s_pend_pkt    = pkt;
                    s_pend_crc_ok = crc_ok;
                    s_pend_tick   = HAL_GetTick();
                    s_pend_valid  = 1U;
                } else {
                    gs_deliver_433(&pkt, crc_ok, GS_RSSI_NA);   /* CRC 失敗，或模組未附加 RSSI：直接交付 */
                }
            }
        }
        /* 逾時保護：模組該吐的 RSSI 位元組沒來（訊號中斷/模組狀態異常）時，
         * 不能讓暫存封包/ACK 永遠卡著不交付 —— 逾時就以 N/A 交付或直接放棄等待。
         * 50ms 遠大於任何 baud 下的單一字元時間（9600bps 約 1.04ms），不會誤觸發。 */
        if (s_pend_valid && (HAL_GetTick() - s_pend_tick) > 50U) {
            s_pend_valid = 0U;
            gs_deliver_433(&s_pend_pkt, s_pend_crc_ok, GS_RSSI_NA);
        }
        if (s_pend_ack_skip && (HAL_GetTick() - s_pend_ack_tick) > 50U) {
            s_pend_ack_skip = 0U;
        }

        /* E80 920：DIO1 觸發則讀封包，payload 餵入同步FSM（含 RSSI/SNR） */
        if (LoRaE80_RxReady()) {
            uint8_t el = 0;
            int16_t rssi = GS_RSSI_NA, snr = GS_SNR_NA;
            HAL_StatusTypeDef rx_st = LoRaE80_ReadPacket(s_e80buf, &el, &rssi, &snr);
            if (rx_st == HAL_OK) {
                for (uint8_t i = 0; i < el; i++) {
                    {
                        uint8_t aseq = 0, ast = 0, alen = 0;
                        char    atext[ACK_TEXT_MAX + 1];
                        if (AckRx_Feed(&s_ack920, s_e80buf[i], &aseq, &ast, atext, &alen)) {
                            printf("[ACK] link:920 seq:%u status:%s cmd:\"%s\"\r\n",
                                   (unsigned)aseq, ack_status_str(ast), atext);
                        }
                    }
                    uint8_t crc_ok = 0;
                    if (TelemRx_FeedAny(&s_rx920, s_e80buf[i], &pkt, &crc_ok)) {
                        if (crc_ok) {
                            gs_handle_packet(GS_LINK_920, &pkt, rssi, snr);
                        } else {
                            printf("[GS_PKT?] link:920MHz CRC_BAD rssi:%d snr:%d seq:%u fsm:%u alt:%dcm baro:%dcm bat:%dmV\r\n",
                                   (int)rssi, (int)snr, (unsigned)pkt.seq, (unsigned)pkt.fsm_state,
                                   (int)pkt.ekf_pos_z_cm, (int)pkt.baro_alt_cm, (unsigned)pkt.bat_mv);
                        }
                    }
                }
            } else if (rx_st == HAL_ERROR) {
                /* CRC/header 錯誤：記入統計 */
                GsLoraTest_UpdateStats(GS_LINK_920, GS_RSSI_NA, GS_SNR_NA, 0 /* crc_err */);
            }
        }

        /* UART2 命令處理（地面站通訊測試） */
        GsLoraTest_Tick();

        /* 每 1 秒 (1Hz) 即時回報地面站自身 GPS 定位、座標與海拔 */
        static uint32_t s_last_gps_report_tick = 0;
        uint32_t now_tick = HAL_GetTick();
        if (now_tick - s_last_gps_report_tick >= 1000U) {
            s_last_gps_report_tick = now_tick;
            char lat_sign = (g->lat_1e6 < 0) ? '-' : '+';
            char lon_sign = (g->lon_1e6 < 0) ? '-' : '+';
            uint32_t lat_abs = (g->lat_1e6 < 0) ? (uint32_t)(-g->lat_1e6) : (uint32_t)g->lat_1e6;
            uint32_t lon_abs = (g->lon_1e6 < 0) ? (uint32_t)(-g->lon_1e6) : (uint32_t)g->lon_1e6;
            if (g->fix_valid) {
                printf("[GS_GPS] FIX sats=%u Pos:%c%lu.%06lu,%c%lu.%06lu Alt:%dm\r\n",
                       (unsigned)g->satellites,
                       lat_sign, (unsigned long)(lat_abs / 1000000U), (unsigned long)(lat_abs % 1000000U),
                       lon_sign, (unsigned long)(lon_abs / 1000000U), (unsigned long)(lon_abs % 1000000U),
                       (int)g->altitude_m);
            } else {
                printf("[GS_GPS] SEARCHING sats=%u q=%u ok=%lu err=%lu\r\n",
                       (unsigned)g->satellites, (unsigned)g->fix_quality, (unsigned long)g->sentences_ok, (unsigned long)g->sentences_err);
            }
        }

        /* 每 2 秒回報雙鏈路通訊狀態（bring-up 診斷用）：
         *   433: raw=原始位元組數 / ok=完整封包 / crc=湊滿但CRC錯 / rsync=找sync退回次數
         *   解讀：raw=0 → 完全沒收到(通道不符/接線/未進RX)；raw↑ 但 ok=0 且 rsync↑ → 空中速率不符(雜訊)；
         *         crc↑ → 速率對但有誤碼(訊號弱/部分)；ok↑ → 正常。 */
        static uint32_t s_last_stat_report_tick = 0;
        if (now_tick - s_last_stat_report_tick >= 2000U) {
            s_last_stat_report_tick = now_tick;
            printf("[GS_STAT] HW:433=%s 920=%s | 433 raw=%lu ok=%lu crc=%lu rsync=%lu | 920 ok=%lu crc=%lu rsync=%lu | pkts 433=%lu 920=%lu | 920rearm=%lu\r\n",
                   lora433_ok ? "OK" : "OFF", lora920_ok ? "OK" : "OFF",
                   (unsigned long)s_u3_rx_bytes,
                   (unsigned long)s_rx433.ok, (unsigned long)s_rx433.crc_err, (unsigned long)s_rx433.resync,
                   (unsigned long)s_rx920.ok, (unsigned long)s_rx920.crc_err, (unsigned long)s_rx920.resync,
                   (unsigned long)s_stat_433_cnt, (unsigned long)s_stat_920_cnt,
                   (unsigned long)s_920_rearm_cnt);
        }

        /* 920 靜默看門狗：見上方 s_920_rearm_cnt 宣告處註解。
         * 不用等 2 秒節流的 [GS_STAT] 區塊，每輪都檢查（成本只是一次 tick 比較），
         * 但重新武裝本身每 GS_E80_REARM_MS 只會真的觸發一次（見下方對
         * s_last_920_tick 的更新）。 */
        if (lora920_ok && (now_tick - s_last_920_tick) >= GS_E80_REARM_MS) {
            s_last_920_tick = now_tick;   /* 先重置計時，避免 StartRx 本身耗時導致連續觸發 */
            s_920_rearm_cnt++;
            printf("[GS_E80] 920 靜默 >=%lums，重新 StartRx（第 %lu 次）\r\n",
                   (unsigned long)GS_E80_REARM_MS, (unsigned long)s_920_rearm_cnt);
            (void)LoRaE80_StartRx();
        }

        /* 指示燈：心跳 / 433 收包 / 920 收包 */
        gs_leds_update(now_tick);

        HAL_IWDG_Refresh(&hiwdg);
        osDelay(GS_POLL_DELAY_MS);
    }
#endif /* GS_USB_SELFTEST */
}

#endif /* IS_GROUND */
