/**
 ******************************************************************************
 * @file    lora_e22.c
 * @brief   E22-400T30S 433MHz LoRa 透傳模式驅動 (UART3)
 ******************************************************************************
 */
#include "lora_e22.h"
#include "main.h"   /* LORA433_* 腳位巨集 */
#include "cmsis_os2.h"   /* e22_hw_lock() 逾時輪詢用 osDelay */
#include <stdio.h>
#include <string.h>

#define LORA_E22_AUX_BOOT_TIMEOUT_MS  1000U  /* 開機/重置後等 AUX 拉高上限 */
#define LORA_E22_TX_TIMEOUT_MS        100U   /* UART 阻塞傳輸逾時 */

/* 頻率設定：E22-400T30S 頻率 = 410 + CH (MHz)，CH 寫入 EEPROM 暫存器 0x05 */
#define E22_FREQ_BASE_MHZ  410U
#define E22_TX_FREQ_MHZ    432U
#define E22_CH             ((uint8_t)(E22_TX_FREQ_MHZ - E22_FREQ_BASE_MHZ))

/* 發射功率等級（REG1(0x04) bit[1:0]）：0=30dBm 1=27dBm 2=24dBm 3=21dBm。
 * 本板 3V3 供電無法穩定驅動 30dBm（1W，突波電流 ~600mA 會把 3V3 拉垮→模組欠壓，
 * 表現為「發幾秒就斷」）。降到最低 21dBm 大幅減少突波電流。
 * 註：E22-400T30S 無 22dBm 檔位，21dBm 為最接近且最省電者。 */
#define E22_TX_POWER_LEVEL 3U   /* 21dBm */

/* 空中速率等級（REG0(0x03) bit[2:0]）：0=0.3k 1=1.2k 2=2.4k 3=4.8k 4=9.6k 5=19.2k 6=38.4k 7=62.5k。
 * 空中速率越低 → 靈敏度/鏈路餘裕越好、射程越遠、抗雜訊越強（代價是資料率低）。
 * 模組先前被(舊版)設成 62.5k → 台面測試就大量 CRC 錯（raw↑ 但 crc↑↑、ok 極少）。
 * 降到 2.4k 增加 ~15-20dB 餘裕；配合遙測降速 divider 不會塞爆緩衝。
 * ★兩端(火箭/地面站)必須相同才能通訊——此值兩板共用同一韌體巨集，保證一致。 */
#define E22_AIR_RATE       2U   /* 2.4k */

/* RSSI 相關兩個功能位（★注意是兩個不同暫存器的不同位元，不要混淆）：
 *   REG1(0x04) bit5 = 「環境噪聲 RSSI 致能」。開啟後可在**透傳/WOR 模式**下發
 *       C0 C1 C2 C3 這個保留字首讀回底噪/上次 RSSI —— 這代表模組會持續監看
 *       一般透傳的 UART 資料流找這個字首，不是只在設定模式才生效的純命令式
 *       讀取（這是我先前想錯的地方，見下方 E22_RSSI_NOISE_EN 實測+datasheet 引註）。
 *   REG3(0x06) bit7 = 「RSSI 位元組致能」。開啟後模組每收到一包，會在 UART 吐完
 *       酬載後**多附加一個 RSSI 位元組**（值 v 對應 −(256−v) dBm）。這個會改變
 *       透傳位元組流的長度，接收端必須跟著多讀一個位元組。
 * ★ 先前韌體兩個位都沒主動寫過，是開是關全看模組 EEPROM 殘留；而地面站
 *   ground_station.c 卻「無條件」多讀一個位元組當 RSSI。模組若沒開 bit7，那一個
 *   位元組讀到的其實是下一包的 sync0(0xA5) → 換算恆為 −91dBm（實測 log 130 筆有
 *   129 筆剛好 −91），看起來像「訊號突然變小」，實際是讀到假值。
 *   現在改為兩個位都由韌體明確寫入，行為不再取決於 EEPROM 殘留。 */
/* ★★★★ 2026-07-28 根因定案：bit7 開啟後 ok/crc 長期卡在 ~8%（歷輪實測 ok=2/crc=31、
 * ok=46/crc=493、ok=1/crc=12），而 bit7=0 同鏈路 0 CRC 錯。歷輪修正全在接收端，全無效。
 * 真正的病灶在「空中子封包邊界」，datasheet 兩處合起來看就清楚：
 *   p.12 §5.6.2：模組把內部 1000-byte 緩衝寫入 RFIC 時是 "Auto sub-packaging"
 *                —— 空中封包邊界由模組自行決定，與我們 116-byte 應用封包無關。
 *   p.19 REG3 bit7："the module receives wireless data and it will follow an RSSI
 *                strength byte after output via the serial port TXD"
 *                —— 每一次「無線接收」附加一個 RSSI 位元組，不是每個應用封包一個。
 * 兩者相乘：116 bytes 一旦被拆成 2 個以上子封包，RSSI 位元組就會插進封包「中間」，
 * 而接收端是用位置剝它（湊滿 116 後的下一個位元組）。拆不拆是時序決定、非決定性
 * → 就是那個固定 ~8% 的成功率。bit7=0 時拆包完全不可見（位元組直接接續），故 0 錯。
 * 兩個造成拆包的具體條件，本版都已堵住：
 *   (1) 子封包長度 REG1 bit[7:6] 以前韌體從不寫、全看 EEPROM 殘留；殘留成
 *       128B/64B/32B 就「每包必拆」。→ 現由 E22_SUBPKT_CODE 明確釘死 240B。
 *   (2) 發射端 UART 位元組流有空隙：LoRaE22_Send 舊版是阻塞輪詢傳輸，呼叫者
 *       LoRaTelemetry_Task 是 osPriorityLow，115200 下送 116 bytes 要 10ms，
 *       期間被搶佔 → UART idle → 模組提前收尾送出子封包。→ 現改 DMA 發送，
 *       由硬體連續餵位元組，不受任務搶佔影響。 */
#define E22_RSSI_BYTE_EN   1U   /* REG3 bit7：每包附加 RSSI 位元組 */

/* 子封包長度（REG1(0x04) bit[7:6]）：0=240B(原廠預設) 1=128B 2=64B 3=32B。
 * ★必須 240B：datasheet p.18「When the data is smaller than the sub packet length,
 *   the serial output of the receiving end is an uninterrupted continuous output.」
 *   TELEM_PACKET_SIZE=116 < 240 才能保證「一個應用封包 = 一個空中子封包 = 一個
 *   RSSI 位元組（且在尾端）」，接收端的位置式剝除才成立。 */
#define E22_SUBPKT_CODE    0U   /* 240 bytes */

/* UART baud 等級（REG0(0x03) bit[7:5]）：0=1200 1=2400 2=4800 3=9600(原廠預設)
 * 4=19200 5=38400 6=57600 7=115200。這是「MCU↔模組」的有線速率，與空中速率
 * (air rate)無關，兩端不需要一致。
 * ★ 用 115200 是為了「整包快速灌進模組緩衝」：116 bytes 在 115200 下只要 10ms，
 *   模組還來不及做 auto sub-packaging 決策就已收齊，最有利於「一包 = 一個空中
 *   子封包 = 一個尾端 RSSI 位元組」。9600 下同一包要 121ms，模組有大把時間在
 *   中途決定先送一段出去。搭配 LoRaE22_Send 的 DMA 發送（位元組間無空隙），
 *   這個組合才是 RSSI 位元組能正確落在尾端的前提。
 * ★ 安全性附註：datasheet p.15 明載設定模式(M1=1,M0=0) 恆為 9600 8N1、與 REG0
 *   無關，故 baud 寫壞永遠可救援、不會變磚。 */
#define E22_UART_BAUD_CODE 7U   /* 115200bps */
/* ★★ 2026-07-28 補充：REG1 bit5 也必須維持 0，原因見官方 datasheet ★★
 * Datasheet(E22-400T30S_UserManual_EN_v1.8.pdf, p.17-18, REG1 bit5) 原文：
 *   "5: RSSI Ambient noise enable. 1: Enable / 0: Disable (default).
 *    When enabled, the C0 C1 C2 C3 command can be sent in the transmitting
 *    mode or WOR mode to read the register."
 * 這代表開啟後，模組在「一般透傳模式下」會持續監看 UART 進來的資料流，一旦
 * 看到 C0 C1 C2 C3 這個保留字首就攔截當成讀暫存器命令，不會當成資料發射。
 * 我們的遙測封包是幾乎隨機的二進位內容，送多了封包裡任何位置出現這 4 個
 * 位元組只是時間問題——一旦命中，那幾個位元組會被模組吃掉、不上空，後面
 * 欄位整個位移，正是稍早實測看到「前段正常、中段後全亂」的災情特徵。
 * 原廠 factory default 本來就是 0（Disable），不要再開。 */
#define E22_RSSI_NOISE_EN  0U   /* REG1 bit5：★保持 0（原廠預設），開啟會讓模組攔截 C0C1C2C3 位元組序列 */

/* REG0(0x03)/REG1(0x04)/REG3(0x06) 暫存器位元定義（EByte E22-400T30S User Manual）：
 *   REG0 bit[7:5]=UART baud, bit[4:3]=parity, bit[2:0]=空中速率(air data rate)
 *   REG1 bit[7:6]=子封包長度, bit[5]=RSSI雜訊致能, bit[1:0]=發射功率
 *   REG3 bit[7]=RSSI byte致能, bit[6]=傳輸模式(0=透傳/1=定點)
 * 僅供 LoRaE22_PrintConfig() 解碼開機讀回的暫存器供人工比對雙端（地面站/主航電）
 * 是否一致；韌體不主動改寫這些暫存器（寫錯 UART baud 位會讓模組與 MCU 直接
 * 失聯、無法再讀回救援），一致性靠 bring-up 時比對雙端 log 人工確認。 */

static const char *const s_air_rate_str[8] = {
    "0.3k", "1.2k", "2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"
};
static const char *const s_uart_baud_str[8] = {
    "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"
};
static const uint32_t s_uart_baud_val[8] = {
    1200U, 2400U, 4800U, 9600U, 19200U, 38400U, 57600U, 115200U
};
static const char *const s_parity_str[4] = { "8N1", "8O1", "8E1", "8N1" };
static const char *const s_tx_power_str[4] = { "30dBm", "27dBm", "24dBm", "21dBm" };
static const char *const s_subpkt_str[4] = { "240B", "128B", "64B", "32B" };

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t             s_inited = 0;

/* AUX(PE11)：HIGH=空閒。等待拉高，含逾時保護。回傳 1=就緒, 0=逾時。 */
static uint8_t e22_wait_aux_high(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LORA433_BUSY_GPIO_Port, LORA433_BUSY_Pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return 0;
        }
    }
    return 1;
}

static uint8_t s_e22_cfg[7] = {0};
static uint8_t s_e22_cfg_valid = 0;

/* 透傳模式下 MCU↔E22 的 UART baud 必須等於模組 REG0(0x03) 設定值（只有「設定模式」才固定
 * 9600、與 REG0 無關）。開機 e22_probe() 已把 REG0 讀進 s_e22_cfg[3]；此處解出模組實際 baud，
 * 讓 MCU 跟著模組走，不再寫死 115200。
 *   —— 先前寫死 115200 對上模組實際 9600（開機 log 的 UART=9600bps），透傳位元框全錯：
 *      發射端灌 115200 → E22 以 9600 取樣得亂碼再上空；接收端 E22 吐 9600 → MCU 以 115200
 *      取樣得亂碼。偵測走設定模式固定 9600 故 log 報 OK，實際卻整條收不到。
 * 尚未 probe 到（模組未偵測）時回落 9600（E22 出廠預設）。 */
static uint32_t e22_transparent_baud(void)
{
    if (s_e22_cfg_valid) {
        return s_uart_baud_val[(s_e22_cfg[3] >> 5) & 0x07];
    }
    return 9600U;
}

/* 接收端重新掛載回呼（見 LoRaE22_SetRxRearmCallback 的說明）。 */
static void (*s_rx_rearm_cb)(void) = NULL;

void LoRaE22_SetRxRearmCallback(void (*cb)(void))
{
    s_rx_rearm_cb = cb;
}

/* ★★ 真正根因（比先前「命令台被 EKF_Task 搶佔」的猜測更根本）★★
 * UplinkCmd_OnUart3Error()/GroundStation_OnUart3Error() 是掛在 HAL_UART_ErrorCallback
 * 上、由 USART3 錯誤中斷直接呼叫的 —— 是 ISR context，不受任何 RTOS 任務優先權節制
 * （幫命令台任務拉 osPriorityRealtime 完全擋不住中斷）。它們的邏輯是
 * 「RxState != BUSY_RX 就重掛 DMA 接收」；而 e22_enter_config_baud() 一開始就呼叫
 * HAL_UART_AbortReceive() 把 RxState 打回 READY，之後一路到 e22_exit_config_mode()
 * 呼叫 s_rx_rearm_cb() 前，RxState 都不是 BUSY_RX。M1 腳位切換 / baud 改變在這段
 * 期間很容易在 RX 線上產生雜訊觸發 ORE/FE，一旦 USART3 錯誤中斷在這個窗口內觸發，
 * 就會呼叫 uplink_u3_rx_rearm()/gs_u3_rx_rearm() 在本函式的阻塞輪詢
 * HAL_UART_Transmit/Receive 「中途」重新掛上 DMA 接收——DMA 是硬體直接搶著把 DR
 * 讀空，永遠比軟體輪詢快，模組的回應位元組於是被 DMA 吃掉，輪詢端讀到的就是全 0、
 * 逾時回 HAL_TIMEOUT(st=3)。這就是「開機 probe 會成功、飛行/測試中下 e22 指令卻
 * 每次 st=3」的真正根因：開機時通常還沒有 ORE 中斷在恰好的時間點打進來，測試中
 * 反覆進出設定模式、M1 反覆切換，遇到的機會大增。
 * 對策：用 s_e22_in_cfg_mode 旗標告訴那兩支 ISR 回呼「目前在設定模式輪詢中，先別
 * 重掛」，離開設定模式時才由本檔自己呼叫 s_rx_rearm_cb() 重新掛好。 */
static volatile uint8_t s_e22_in_cfg_mode = 0;

uint8_t LoRaE22_IsInConfigMode(void)
{
    return s_e22_in_cfg_mode;
}

/* ★另一個獨立的競爭條件：LoRaE22_Init() 不是只有開機呼叫一次——LoRaTelemetry_Task
 * 每 10s 會在 `!lora433_ok` 時重試呼叫它，而這個重試判斷只在該任務 for(;;) 迴圈「最
 * 頂端」檢查 g_lora_cfg_pause；LoRaE22_Init() 本身要跑好幾輪進出設定模式，可能耗時
 * 數百 ms。若命令台這時候呼叫 LoRaE22_SetFreqMHz()/SetAirRate()/SetPowerLevel()，
 * cmd_lora_pause() 設旗標時 LoRaTelemetry_Task 可能已經在 LoRaE22_Init() 執行「中
 * 途」，不會回頭看旗標——兩個任務同時搶 UART3/M1/AUX，比 ISR 那個 race 更直接，
 * 結果就是「時好時壞」：只有兩者真的撞在一起的那幾次才會失敗。
 * 用這個簡易忙線鎖讓 LoRaE22_Init() 與三個 Set* 函式互斥：誰先進來誰就把硬體鎖住，
 * 另一邊改成有限時間輪詢等待，逾時就回 HAL_BUSY 而不是硬闖（Init 由呼叫端 10s 後
 * 自動再試；Set* 由命令台印出 FAIL 讓使用者重下一次，皆是可接受的 best-effort）。 */
static volatile uint8_t s_e22_hw_busy = 0;

static uint8_t e22_hw_lock(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        if (!s_e22_hw_busy) { s_e22_hw_busy = 1; return 1; }
        if ((HAL_GetTick() - t0) > timeout_ms) return 0;
        osDelay(5);
    }
}

static void e22_hw_unlock(void)
{
    s_e22_hw_busy = 0;
}

/* 進入設定模式的 UART 準備：停掉進行中的接收，再切到設定模式固定的 9600 8N1。
 * ★停接收是 DMA 化之後的硬性要求：透傳期間 USART3 掛的是循環 DMA 接收，CR3.DMAR
 *   一旦致能，DMA 控制器會在每次 RXNE 由硬體把 DR 讀走 —— 底下那些設定模式的
 *   阻塞式 HAL_UART_Receive 就永遠等不到位元組，回讀全逾時（probe 失敗、
 *   `e22 freq/pwr/air` 靜默失效）。HAL_UART_AbortReceive 會清掉 DMAR 並停下 DMA。
 *   （中斷式接收沒這問題，因為 RXNE 中斷在 RxState != BUSY_RX 時不會取走資料，
 *    所以舊版沒踩到；改 DMA 後必須明確處理。）
 * 離開設定模式時由 e22_exit_config_mode() 通知擁有者重新掛載接收。 */
static void e22_enter_config_baud(void)
{
    s_e22_in_cfg_mode = 1;   /* 見上方說明：告訴 ISR 端的 rearm 先別搶著重掛 */
    HAL_UART_AbortReceive(s_huart);
    s_huart->Init.BaudRate = 9600;
    HAL_UART_Init(s_huart);
}

/* 離開設定模式：還原透傳 baud → 回透傳模式(M1=0) → 通知接收端重新掛載。
 * ★最後那一步是必要的：本檔進出設定模式都會呼叫 HAL_UART_Init 改 baud，而
 *   HAL_UART_Init 會把 huart->RxState 打回 READY —— 進行中的 ReceiveToIdle
 *   （IT 或 DMA 皆然）就此失效，HAL_UART_IRQHandler 的 IDLE 分支要求
 *   RxState == BUSY_RX 才會回呼，於是 433 接收從此靜默、直到重開機。
 *   以前沒處理，所以只要在執行中下 `e22 freq/pwr/air`，之後就再也收不到封包。
 *   驅動不知道誰在收 UART3（地面站 / 主航電上行各一份），故由擁有者註冊回呼。 */
static void e22_exit_config_mode(void)
{
    s_huart->Init.BaudRate = e22_transparent_baud();
    HAL_UART_Init(s_huart);
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    if (s_rx_rearm_cb != NULL) s_rx_rearm_cb();
    s_e22_in_cfg_mode = 0;   /* 本檔已自行重掛好接收，ISR 端的 rearm 現在可以恢復正常運作 */
}

/* 進入設定模式，寫入指定頻道後回透傳模式（帶 ch 參數版，供外部呼叫） */
static HAL_StatusTypeDef e22_write_channel(uint8_t ch)
{
    /* 切設定模式（M1=1, M0=0），等 AUX HIGH（模式切換完成） */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    if (!e22_wait_aux_high(200)) {
        HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
        return HAL_TIMEOUT;
    }

    e22_enter_config_baud();   /* 停接收(清 DMAR) + 切設定模式固定 9600 */

    /* 讀回 CH 暫存器，若已一致則跳過寫入 */
    uint8_t rd[3] = {0xC1, 0x05, 0x01};
    uint8_t rd_resp[4] = {0};
    HAL_UART_Transmit(s_huart, rd, sizeof(rd), 50);
    HAL_UART_Receive(s_huart, rd_resp, sizeof(rd_resp), 100);
    printf("[LORA433] CH read-back: %02X %02X %02X %02X (want C1 05 01 %02X)\r\n",
           rd_resp[0], rd_resp[1], rd_resp[2], rd_resp[3], ch);

    HAL_StatusTypeDef ret = HAL_OK;
    if (rd_resp[0] == 0xC1 && rd_resp[3] == ch) {
        printf("[LORA433] CH already %d (%u MHz), skip write\r\n",
               ch, (unsigned)(E22_FREQ_BASE_MHZ + ch));
    } else {
        uint8_t wr[4] = {0xC0, 0x05, 0x01, ch};
        uint8_t wr_resp[4] = {0};
        HAL_UART_Transmit(s_huart, wr, sizeof(wr), 50);
        HAL_UART_Receive(s_huart, wr_resp, sizeof(wr_resp), 100);
        printf("[LORA433] CH write resp: %02X %02X %02X %02X\r\n",
               wr_resp[0], wr_resp[1], wr_resp[2], wr_resp[3]);
        if (!e22_wait_aux_high(300)) ret = HAL_TIMEOUT;
        else {
            if (s_e22_cfg_valid) s_e22_cfg[5] = ch; // update cache
        }
    }

    e22_exit_config_mode();   /* 還原透傳 baud + M1=0 + 通知接收端重掛 */
    return ret;
}

/* 寫入 REG1(0x04)：子封包長度位元[7:6] + 環境噪聲 RSSI 致能位元[5] + 發射功率位元[1:0]。
 * 三者同屬 REG1，合併成一次寫入（少進一次設定模式，開機較快）。
 * REG1 不含 UART baud（那在 REG0），故不影響 MCU↔E22 baud 一致性；
 * 也不含 RSSI 位元組致能（那在 REG3 bit7，見 e22_write_reg3_rssi）。
 * ★子封包長度以前刻意不寫、全看 EEPROM 殘留，是 RSSI byte 開啟後 framing 崩掉的
 *   根因之一（殘留 <116 就每包必拆、RSSI 位元組插進封包中間）——現在明確釘死，
 *   行為不再取決於模組出廠/前人設定。REG1 不含 baud 位，寫錯不會失聯。
 * 需先 probe 到 REG1（保留其他位元）才能寫；已是目標值則跳過（省一次寫入、加快後續開機）。 */
static HAL_StatusTypeDef e22_write_reg1(uint8_t pwr_level, uint8_t noise_en, uint8_t subpkt)
{
    if (!s_e22_cfg_valid) return HAL_ERROR;

    /* 目標值：動 bit[7:6](子封包長度)、bit5(噪聲致能)、bit[1:0](功率)，保留 bit[4:2] */
    uint8_t reg1 = (uint8_t)((s_e22_cfg[4] & (uint8_t)~0xE3) |
                             (uint8_t)((subpkt & 0x03) << 6) |
                             (pwr_level & 0x03) |
                             (uint8_t)((noise_en ? 1U : 0U) << 5));
    if (s_e22_cfg[4] == reg1) {
        printf("[LORA433] REG1 already pwr=%s noise=%u subpkt=%s, skip write\r\n",
               s_tx_power_str[pwr_level & 0x03], (unsigned)(noise_en ? 1U : 0U),
               s_subpkt_str[subpkt & 0x03]);
        return HAL_OK;
    }

    /* 切設定模式（M1=1, M0=0），等 AUX HIGH */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    if (!e22_wait_aux_high(200)) {
        HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
        return HAL_TIMEOUT;
    }

    e22_enter_config_baud();   /* 停接收(清 DMAR) + 切設定模式固定 9600 */

    uint8_t wr[4] = {0xC0, 0x04, 0x01, reg1};
    uint8_t wr_resp[4] = {0};
    HAL_UART_Transmit(s_huart, wr, sizeof(wr), 50);
    HAL_UART_Receive(s_huart, wr_resp, sizeof(wr_resp), 100);
    printf("[LORA433] REG1 write resp: %02X %02X %02X %02X (set pwr=%s noise=%u subpkt=%s)\r\n",
           wr_resp[0], wr_resp[1], wr_resp[2], wr_resp[3],
           s_tx_power_str[pwr_level & 0x03], (unsigned)(noise_en ? 1U : 0U),
           s_subpkt_str[subpkt & 0x03]);

    HAL_StatusTypeDef ret = HAL_OK;
    if (!e22_wait_aux_high(300)) ret = HAL_TIMEOUT;
    else s_e22_cfg[4] = reg1;   /* 更新快取，使 LoRaE22_PrintConfig 顯示新值 */

    e22_exit_config_mode();
    return ret;
}

/* 寫入 REG3(0x06) bit7「RSSI 位元組致能」，保留傳輸模式(bit6)/LBT/WOR 等其他位元。
 * ★這個位元會改變透傳位元組流：開啟後模組每收一包會在酬載後多吐一個 RSSI 位元組，
 *   接收端（ground_station.c）必須跟著多讀一個位元組，兩者要嘛都開、要嘛都關，
 *   否則 framing 會差一個位元組。故接收端不自己假設，改查 LoRaE22_RssiByteEnabled()。
 * 不動 REG0（UART baud）故無失聯風險；已是目標值則跳過。 */
static HAL_StatusTypeDef e22_write_reg3_rssi(uint8_t en)
{
    if (!s_e22_cfg_valid) return HAL_ERROR;

    uint8_t reg3 = (uint8_t)((s_e22_cfg[6] & (uint8_t)~0x80) |
                             (uint8_t)((en ? 1U : 0U) << 7));
    if (s_e22_cfg[6] == reg3) {
        printf("[LORA433] RSSI byte already %s, skip write\r\n", en ? "ON" : "OFF");
        return HAL_OK;
    }

    /* 切設定模式（M1=1, M0=0），等 AUX HIGH */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    if (!e22_wait_aux_high(200)) {
        HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
        return HAL_TIMEOUT;
    }

    e22_enter_config_baud();   /* 停接收(清 DMAR) + 切設定模式固定 9600 */

    uint8_t wr[4] = {0xC0, 0x06, 0x01, reg3};
    uint8_t wr_resp[4] = {0};
    HAL_UART_Transmit(s_huart, wr, sizeof(wr), 50);
    HAL_UART_Receive(s_huart, wr_resp, sizeof(wr_resp), 100);
    printf("[LORA433] REG3 write resp: %02X %02X %02X %02X (set RSSI byte=%s)\r\n",
           wr_resp[0], wr_resp[1], wr_resp[2], wr_resp[3], en ? "ON" : "OFF");

    HAL_StatusTypeDef ret = HAL_OK;
    if (!e22_wait_aux_high(300)) ret = HAL_TIMEOUT;
    else s_e22_cfg[6] = reg3;   /* 更新快取，使 LoRaE22_RssiByteEnabled() 立即反映 */

    e22_exit_config_mode();
    return ret;
}

/* 寫入 REG0(0x03)：空中速率位元[2:0] + UART baud 位元[7:5]，保留 parity[4:3]。
 * 兩者同屬 REG0，合併成一次寫入（少進一次設定模式）。
 *   air rate：兩端(火箭/地面站)必須相同才能通訊。
 *   UART baud：只是 MCU↔模組的有線速率，兩端不需一致；但接收端邏輯對它敏感，
 *              見 E22_UART_BAUD_CODE 註解（RSSI 位元組到達時序）。
 * ★ 不會變磚：datasheet p.15 明載設定模式恆為 9600 8N1、與 REG0 無關，故 baud
 *   寫壞仍可再進設定模式改回。寫入後更新快取，e22_transparent_baud() 隨即回傳
 *   新值，函式結尾便把 MCU 端 UART 切到新速率，兩邊同步。 */
static HAL_StatusTypeDef e22_write_reg0(uint8_t air_rate, uint8_t baud_code)
{
    if (!s_e22_cfg_valid) return HAL_ERROR;

    /* 目標值：只動 baud[7:5] 與 air rate[2:0]，parity[4:3] 原樣保留 */
    uint8_t reg0 = (uint8_t)((s_e22_cfg[3] & 0x18) |
                             (uint8_t)((baud_code & 0x07) << 5) |
                             (air_rate & 0x07));
    if (s_e22_cfg[3] == reg0) {
        printf("[LORA433] REG0 already AirRate=%s UART=%sbps, skip write\r\n",
               s_air_rate_str[air_rate & 0x07], s_uart_baud_str[baud_code & 0x07]);
        return HAL_OK;
    }

    /* 切設定模式（M1=1, M0=0），等 AUX HIGH */
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    if (!e22_wait_aux_high(200)) {
        HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);
        return HAL_TIMEOUT;
    }

    e22_enter_config_baud();   /* 停接收(清 DMAR) + 切設定模式固定 9600 */

    uint8_t wr[4] = {0xC0, 0x03, 0x01, reg0};
    uint8_t wr_resp[4] = {0};
    HAL_UART_Transmit(s_huart, wr, sizeof(wr), 50);
    HAL_UART_Receive(s_huart, wr_resp, sizeof(wr_resp), 100);
    printf("[LORA433] REG0 write resp: %02X %02X %02X %02X (set AirRate=%s UART=%sbps)\r\n",
           wr_resp[0], wr_resp[1], wr_resp[2], wr_resp[3],
           s_air_rate_str[air_rate & 0x07], s_uart_baud_str[baud_code & 0x07]);

    HAL_StatusTypeDef ret = HAL_OK;
    if (!e22_wait_aux_high(300)) ret = HAL_TIMEOUT;
    else s_e22_cfg[3] = reg0;   /* 更新快取，下方 e22_transparent_baud() 即回傳新 baud */

    e22_exit_config_mode();   /* baud 跟隨剛寫入的新值，MCU 端 UART 同步切換 */
    return ret;
}

/* 模組在線偵測：透傳模式無握手，故進設定模式回讀全部 7 個設定暫存器。
 * 在線 → 回 `C1 00 07 <7位元組>`；未接/接線錯/故障 → UART 無回應。回傳 1=在線, 0=無回應。 */
static uint8_t e22_probe(void)
{
    if (s_huart == NULL) return 0;

    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_SET);  /* 進設定模式 */
    HAL_Delay(20);
    (void)e22_wait_aux_high(100);

    e22_enter_config_baud();   /* 停接收(清 DMAR) + 切設定模式固定 9600 */

    uint8_t present = 0;
    for (int attempt = 0; attempt < 2 && !present; attempt++) {
        uint8_t rd[3] = {0xC1, 0x00, 0x07};         /* 讀全部 7 個設定暫存器 */
        uint8_t resp[10] = {0};
        HAL_UART_Transmit(s_huart, rd, sizeof(rd), 50);
        HAL_UART_Receive(s_huart, resp, sizeof(resp), 200);
        if (resp[0] == 0xC1 && resp[1] == 0x00 && resp[2] == 0x07) {
            present = 1;
            memcpy(s_e22_cfg, &resp[3], 7);
            s_e22_cfg_valid = 1;
        }
    }

    e22_exit_config_mode();   /* 還原透傳 baud（跟隨模組回讀值）+ 回透傳模式 */
    return present;
}

/* 是否已經對模組做過一次「強制寫死基準值」。★只在這個旗標還是 0 的時候（開機後第一次
 * 真正偵測到模組在線）才寫入 E22_CH/E22_TX_POWER_LEVEL/E22_AIR_RATE/E22_RSSI_BYTE_EN
 * 這組固定基準值，寫過一次後就再也不覆寫。
 * 原本每次呼叫 LoRaE22_Init()（含 LoRaTelemetry_Task 每 10s 的離線重試）都會無條件
 * 覆寫回這組固定值，代表使用者用 `e22 freq/pwr/air` 手動調的參數，只要模組之後又觸發
 * 一次重試（例如飛行中短暫斷訊又復原），就會被無聲蓋掉、退回韌體寫死的頻率/功率/
 * 空速——這正是「參數改了、過一陣子又跳回去」的根因之一。改成只在真正第一次上線時
 * 建立基準（確保雙板不依賴各自 EEPROM 殘留、起手式一致），之後的每次重試只重新
 * probe 確認模組還活著，不再動使用者已經設定過的值。 */
static uint8_t s_e22_synced_once = 0;

HAL_StatusTypeDef LoRaE22_Init(UART_HandleTypeDef *huart)
{
    if (!e22_hw_lock(500)) return HAL_BUSY;   /* 見 e22_hw_lock 註解：避免與命令台的 Set* 撞車 */

    s_huart = huart;

    /* 透傳模式 M1=0, M0=0 */
    HAL_GPIO_WritePin(LORA433_M0_GPIO_Port, LORA433_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA433_M1_GPIO_Port, LORA433_M1_Pin, GPIO_PIN_RESET);

    /* 硬體重置脈衝：RST 拉低 ~10ms 再釋放（RST 低電平有效） */
    HAL_GPIO_WritePin(LORA433_RST_GPIO_Port, LORA433_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA433_RST_GPIO_Port, LORA433_RST_Pin, GPIO_PIN_SET);
    /* 等待 E22 模組開機完成（等待 AUX 腳位拉高，防開機太快回讀失敗） */
    (void)e22_wait_aux_high(LORA_E22_AUX_BOOT_TIMEOUT_MS);
    HAL_Delay(200);   /* 增加 200ms 穩定延時，給 E22 內部 MCU 充足開機時間 */

    s_inited = 1;   /* 鏈路恆標記可用（透傳模式發送靠 AUX 背壓）；偵測結果另由回傳值表示 */

    /* 設定模式回讀偵測模組是否真的在線（誠實回報；呼叫端據此印訊息 / 主航電每 10s 重試）。 */
    if (!e22_probe()) {
        e22_hw_unlock();
        return HAL_TIMEOUT;
    }

    if (!s_e22_synced_once) {
        /* probe 已把 MCU UART baud 對齊到模組實際 REG0 值（見 e22_transparent_baud）；
         * 此處只再「強制寫入固定頻道 E22_CH」，保證主航電/地面站兩端一定落在同一頻率，
         * 不依賴各模組 EEPROM 殘留的舊頻道。只寫 CH(REG2)，刻意不動 baud/air rate
         * （空中速率維持模組現值以保留射程；避免改寫 REG0 UART 位的失聯風險）。
         * 頻道寫入為 best-effort：即使逾時，模組仍在線、鏈路可用，故仍回 HAL_OK；
         * 實際生效頻道由呼叫端隨後的 LoRaE22_PrintConfig() 印出供人工核對。 */
        (void)e22_write_channel(E22_CH);
        /* 3V3 供電：把發射功率降到 21dBm（見 E22_TX_POWER_LEVEL 註解），並一併寫入環境噪聲
         * RSSI 致能與子封包長度 240B（三者同屬 REG1，合併一次寫入）。
         * ★子封包長度必須 240B，否則 116-byte 封包會被拆成多個空中子封包、每個都附加一個
         *   RSSI 位元組插進封包中間，framing 必壞（見 E22_SUBPKT_CODE 註解）。
         * 已是目標值時內部會跳過寫入，故穩態開機不增加時間。best-effort，同上仍回 HAL_OK。 */
        (void)e22_write_reg1(E22_TX_POWER_LEVEL, E22_RSSI_NOISE_EN, E22_SUBPKT_CODE);
        /* REG0：空中速率 2.4k（見 E22_AIR_RATE 註解，增加鏈路餘裕）+ UART 115200bps
         * （見 E22_UART_BAUD_CODE 註解，還原舊 log 中 RSSI 正常運作的已知能動組態）。
         * 已是目標值時跳過。best-effort，仍回 HAL_OK。 */
        (void)e22_write_reg0(E22_AIR_RATE, E22_UART_BAUD_CODE);
        /* REG3 bit7「每包附加 RSSI 位元組」：明確寫入而非沿用 EEPROM 殘留 —— 先前韌體
         * 從不寫此位，模組 EEPROM 殘留什麼就是什麼，接收端卻寫死假設，framing 才會
         * 莫名其妙壞掉。現在兩端都由韌體釘死，接收端以 LoRaE22_RssiByteEnabled() 對齊。 */
        (void)e22_write_reg3_rssi(E22_RSSI_BYTE_EN);
        s_e22_synced_once = 1;
    }
    /* 之後每次重試（模組曾離線又復原）只到這裡為止：probe 已確認在線、s_e22_cfg[]
     * 也已回讀最新值，不再覆寫使用者可能已手動調整過的頻率/功率/空速。 */

    e22_hw_unlock();
    return HAL_OK;
}

uint8_t LoRaE22_IsReady(void)
{
    return s_inited;
}

HAL_StatusTypeDef LoRaE22_Send(const uint8_t *data, uint16_t len)
{
    if (!s_inited || s_huart == NULL || data == NULL || len == 0) {
        return HAL_ERROR;
    }
    /* 逾時需覆蓋整包在「目前實際 baud」(跟隨模組回讀值，見 e22_transparent_baud
     * 註解；可能是 9600 而非寫死的 115200)下的傳輸時間，否則 baud 較低時封包還沒
     * 送完就先逾時 → 100% ERR。以 bit 數/baud + 安全餘裕動態算，下限仍是原本的
     * LORA_E22_TX_TIMEOUT_MS。 */
    uint32_t baud       = e22_transparent_baud();
    uint32_t timeout_ms = (10UL * len * 1000UL) / baud + 50U;
    if (timeout_ms < LORA_E22_TX_TIMEOUT_MS) timeout_ms = LORA_E22_TX_TIMEOUT_MS;

    /* ★發送前查一次 AUX，忙就跳過——不可改成「等到 AUX 拉高再送」的阻塞式等待。
     * datasheet p.13 §5.6.2：AUX=0（上一包還在把 UART 輸入資料寫進 RFIC/送空中）
     * 時送下一包，"may cause ... transmitting wireless sub package"——這正是我們
     * 剛修掉的那個 bug 的成因之一，所以必須查 AUX 才能放心把發射頻率逼近上限。
     * ★但查詢本身必須是「即時讀一次、忙就回 HAL_BUSY」，不能真的等待：
     *   LoRaTelemetry_Task 同一輪迴圈裡，433 送完才輪到 920 送（main.c）。若這裡
     *   阻塞等待 AUX 拉高（116B@2.4k 空中約需 ~580ms），等於每輪迴圈都讓 920 陪
     *   著卡住半秒以上——上一版就是這樣把 920 的實際發送頻率從 ~10Hz 拖垮到
     *   ~2.6Hz（量測報告 ground_station_report_20260728_041445 抓到的）。改成
     *   不等待、忙即跳過後，即使 LORA433_TX_EVERY=1（每槽都嘗試），大部分槽會在
     *   微秒等級判定「AUX 還低→跳過」返回，不影響同一輪的 920 發送與其餘工作。
     *   呼叫端本來就假設 LoRaE22_Send 會「忙線跳過」（見 main.c/gs_lora_test.c
     *   既有的「忙線跳過」註解），語意一致。 */
    if (HAL_GPIO_ReadPin(LORA433_BUSY_GPIO_Port, LORA433_BUSY_Pin) == GPIO_PIN_RESET) {
        return HAL_BUSY;
    }

    /* ★★ 必須用 DMA 發送，不能用阻塞輪詢 HAL_UART_Transmit ★★
     * E22 透傳模式是 "Auto sub-packaging"（datasheet p.12 §5.6.2）：模組自己決定空中
     * 子封包邊界，一旦 UART 位元組流中間出現 idle 空隙，模組就把已收到的部分當成一個
     * 子封包收尾送出，剩下的變第二個子封包。開啟 RSSI 位元組(REG3 bit7)後，模組是
     * 「每一次無線接收」各附加一個 RSSI 位元組 → 第二個子封包前面就多了一個位元組，
     * 等於把 RSSI 位元組插進應用封包中間，接收端的位置式剝除必然失敗（CRC 全錯）。
     * 舊版阻塞輪詢寫法：呼叫者 LoRaTelemetry_Task 是 osPriorityLow，115200 下送
     * 116 bytes 要 10ms，這段期間被 defaultTask/EKF_Task 搶佔幾乎必然 → 空隙必然出現。
     * DMA 由硬體直接餵 TDR，位元組間隔恆為 0，完全不受任務搶佔影響。
     * 仍等到傳完才返回，維持「呼叫者回來時來源緩衝已可重用」的既有語意。 */
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(s_huart, (uint8_t *)data, len);
    if (st != HAL_OK) return st;

    uint32_t t0 = HAL_GetTick();
    while (s_huart->gState != HAL_UART_STATE_READY) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            HAL_UART_DMAStop(s_huart);
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef LoRaE22_SetFreqMHz(uint32_t freq_mhz)
{
    if (!s_inited || s_huart == NULL) return HAL_ERROR;
    if (freq_mhz < E22_FREQ_BASE_MHZ || freq_mhz > (E22_FREQ_BASE_MHZ + 83U)) {
        return HAL_ERROR;
    }
    uint8_t ch = (uint8_t)(freq_mhz - E22_FREQ_BASE_MHZ);
    if (!e22_hw_lock(1000)) return HAL_BUSY;   /* 見 e22_hw_lock 註解：避免與 LoRaE22_Init 撞車 */
    HAL_StatusTypeDef ret = e22_write_channel(ch);   /* 只改頻道(REG2)，不動 baud/air rate */
    e22_hw_unlock();
    return ret;
}

HAL_StatusTypeDef LoRaE22_SetPowerLevel(uint8_t pwr_level)
{
    if (!s_inited || s_huart == NULL) return HAL_ERROR;
    if (pwr_level > 3U) return HAL_ERROR;
    if (!s_e22_cfg_valid) return HAL_ERROR;
    if (!e22_hw_lock(1000)) return HAL_BUSY;
    /* 只改 REG1 功率位；噪聲致能位沿用模組現值（不因調功率而被關掉）。
     * 子封包長度一律帶 E22_SUBPKT_CODE：那是 framing 正確性的前提，不可因調功率而被
     * 沿用成 EEPROM 殘留值（見 E22_SUBPKT_CODE 註解）。 */
    HAL_StatusTypeDef ret = e22_write_reg1(pwr_level, (uint8_t)((s_e22_cfg[4] >> 5) & 0x01), E22_SUBPKT_CODE);
    e22_hw_unlock();
    return ret;
}

uint8_t LoRaE22_RssiByteEnabled(void)
{
    /* 未 probe 到暫存器時保守回 0：接收端就不會多讀位元組，寧可沒有 RSSI
     * 也不要把下一包的 sync0 吃掉而破壞 framing。 */
    return (s_e22_cfg_valid && (s_e22_cfg[6] & 0x80)) ? 1U : 0U;
}

HAL_StatusTypeDef LoRaE22_SetAirRate(uint8_t air_rate)
{
    if (!s_inited || s_huart == NULL) return HAL_ERROR;
    if (air_rate > 7U) return HAL_ERROR;
    if (!s_e22_cfg_valid) return HAL_ERROR;
    if (!e22_hw_lock(1000)) return HAL_BUSY;
    /* 只改 REG0 空速位；UART baud 位沿用模組現值（不因調空速而被改動） */
    HAL_StatusTypeDef ret = e22_write_reg0(air_rate, (uint8_t)((s_e22_cfg[3] >> 5) & 0x07));
    e22_hw_unlock();
    return ret;
}

void LoRaE22_PrintConfig(void)
{
    /* s_e22_cfg[] 是 e22_probe() 讀回位址 0x00~0x06 共 7 個暫存器的快取：
     *   [0]=ADDH [1]=ADDL [2]=NETID [3]=REG0 [4]=REG1 [5]=REG2(CH) [6]=REG3
     * 注意：發射功率/傳輸模式在 REG1([4])/REG3([6])，不是同一個位元組，
     * 曾經誤把兩者都讀成 [6]（把 REG3 當 REG1）算出來的功率是錯的，已修正。 */
    if (s_e22_cfg_valid) {
        uint32_t freq_mhz  = E22_FREQ_BASE_MHZ + s_e22_cfg[5];
        uint8_t  reg0      = s_e22_cfg[3];
        uint8_t  reg1      = s_e22_cfg[4];
        uint8_t  reg3      = s_e22_cfg[6];
        const char *air_rate = s_air_rate_str[reg0 & 0x07];
        const char *parity   = s_parity_str[(reg0 >> 3) & 0x03];
        const char *baud     = s_uart_baud_str[(reg0 >> 5) & 0x07];
        const char *tx_power = s_tx_power_str[reg1 & 0x03];
        const char *subpkt   = s_subpkt_str[(reg1 >> 6) & 0x03];
        uint8_t      rssi_noise_en = (reg1 >> 5) & 0x01;   /* REG1 bit5：環境噪聲讀取 */
        uint8_t      rssi_byte_en  = (reg3 >> 7) & 0x01;   /* REG3 bit7：每包附加 RSSI 位元組 */
        const char  *mode    = (reg3 & 0x40) ? "Fixed" : "Transparent";

        /* ★RSSIbyte 與 RSSInoise 是兩個不同暫存器的不同功能，必須分開顯示：
         * 只有 RSSIbyte=1 時位元組流才會每包多一個 RSSI 位元組（影響接收端 framing）。
         * 舊版只印了 noise 那個並簡稱「RSSIen」，害人誤以為附加功能是關的。 */
        printf("[LORA433] E22-400T30S | Freq=%lu.000MHz(CH=%u) | Power=%s | AirRate=%s | "
               "UART=%sbps/%s | SubPkt=%s | RSSIbyte=%u | RSSInoise=%u | Mode=%s\r\n",
               (unsigned long)freq_mhz, (unsigned)s_e22_cfg[5], tx_power, air_rate,
               baud, parity, subpkt, (unsigned)rssi_byte_en, (unsigned)rssi_noise_en, mode);
    } else {
        printf("[LORA433] E22-400T30S | Freq=%u.000MHz(CH=%u) | (未回讀到暫存器，顯示韌體預設值，非模組實際值)\r\n",
               (unsigned)E22_TX_FREQ_MHZ, (unsigned)E22_CH);
    }
}

void LoRaE22_GetParams(uint32_t *freq_mhz, uint8_t *pwr_level, uint8_t *air_rate)
{
    if (freq_mhz) {
        *freq_mhz = s_e22_cfg_valid ? (410U + s_e22_cfg[5]) : 432U;
    }
    if (pwr_level) {
        *pwr_level = s_e22_cfg_valid ? (s_e22_cfg[4] & 0x03) : 3U;
    }
    if (air_rate) {
        *air_rate = s_e22_cfg_valid ? (s_e22_cfg[3] & 0x07) : 2U;
    }
}

