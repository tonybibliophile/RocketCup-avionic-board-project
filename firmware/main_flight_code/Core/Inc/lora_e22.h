/**
 ******************************************************************************
 * @file    lora_e22.h
 * @brief   E22-400T30S 433MHz LoRa 透傳模式驅動 (UART3)
 *
 * E22 為 SX1268 為核心之 UART 透傳模組。模式由 M1/M0 決定（見 main.h LORA433_*）：
 *     M1=0 M0=0 → 透傳(Normal)：寫入 UART 的位元組即經空中發送（本驅動使用）
 *     M1=0 M0=1 → WOR
 *     M1=1 M0=0 → 設定(暫存器)
 *     M1=1 M0=1 → 深度睡眠
 * AUX(PE11)：HIGH=空閒可發送，LOW=忙線(發送中/模式切換/開機)。
 * 接線（連線和基本硬體規格表.md）：UART3 TX=PD8/RX=PD9, M0=PD11, M1=PD10, RST=PD12, AUX=PE11。
 * 腳位與 UART 已由 CubeMX 初始化（M0=M1=0 透傳、RST 釋放、AUX 為輸入）。
 ******************************************************************************
 */
#ifndef __LORA_E22_H
#define __LORA_E22_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "board_config.h"   /* IS_GROUND（地面站 433 接收不受本開關影響） */

/* ============================================================
 *  模組啟用開關
 * ============================================================
 * LORA433_ENABLE = 0：主航電開機不初始化 E22，遙測任務不重試亦不發送 433 下行。
 *   E22 走獨立 UART3（不與 Flash 共用匯流排，無 E80 那種匯流排污染風險），停用僅
 *   代表不啟用該鏈路；模組維持 CubeMX 預設腳位（M0=M1=0 透傳、RST 釋放、AUX 輸入），
 *   未被餵入資料故不發射。★ 目前已啟用（=1）；如需停用（例如排查 3V3 短路、或不掛 433）
 *   改回 0。
 *   註：地面站（ROLE_GROUND）的 433「接收」固定啟用、不受本開關影響。
 * LORA433_ENABLE = 1：正常初始化；AUX 逾時（模組未回應）時由遙測任務每 10s 週期性重試。 */
#ifndef LORA433_ENABLE
#define LORA433_ENABLE 1
#endif

/**
 * @brief 綁定 UART、設透傳模式（M0=M1=0）+ 硬體重置，並以「設定模式回讀 CH 暫存器」
 *        偵測模組是否真的在線（透傳模式無握手；AUX 為 PULLUP 無法判在線，故用回讀）。
 *        於 main() 初始化區呼叫；偵測失敗時可由低優先任務週期性重試（P1）。
 * @param huart E22 所掛的 UART（本專案為 &huart3）。
 * @return HAL_OK = 模組有回應（在線）；HAL_TIMEOUT = 無回應（未接 / 接線錯 / 故障）。
 *         注意：不論回傳值，鏈路皆標記為可用（s_inited=1）；回傳值僅供誠實回報 / 重試判斷。
 */
HAL_StatusTypeDef LoRaE22_Init(UART_HandleTypeDef *huart);

/**
 * @brief 透傳發送一筆位元組。
 * @return HAL_OK 已送出；HAL_BUSY 模組忙線(AUX low)本次跳過；HAL_ERROR 參數錯誤/未初始化。
 * @note  AUX 背壓：忙線即跳過不阻塞，由呼叫端（遙測任務）自我限流以對齊空中速率。
 */
HAL_StatusTypeDef LoRaE22_Send(const uint8_t *data, uint16_t len);

/** @brief 模組是否空閒可發送（AUX=HIGH 且已初始化）。 */
uint8_t LoRaE22_IsReady(void);
void LoRaE22_PrintConfig(void);

/**
 * @brief 動態修改 E22 433 透傳頻率（進入設定模式寫 CH 暫存器後回透傳模式）。
 *        CH = freq_mhz - 410；合法範圍 410~493 MHz（CH 0~83）。
 *        設定存入 EEPROM，掉電不遺失。僅供地面站通訊測試使用。
 * @param freq_mhz 目標頻率 MHz (410~493)
 * @return HAL_OK 成功；HAL_ERROR 範圍錯誤或未初始化；HAL_TIMEOUT AUX 等待逾時；
 *         HAL_BUSY LoRaE22_Init() 的離線重試正在進行中，稍後重下即可。
 */
HAL_StatusTypeDef LoRaE22_SetFreqMHz(uint32_t freq_mhz);

/**
 * @brief 動態修改 E22 發射功率等級（寫 REG1 bit[1:0]，保留其餘位元）。
 *        0=30dBm 1=27dBm 2=24dBm 3=21dBm。設定存入 EEPROM，掉電不遺失。
 *        ★本板 3V3 供電無法穩定驅動 30dBm（突波電流會拉垮 3V3），建議維持 3(21dBm)。
 * @return HAL_OK 成功；HAL_ERROR 未初始化/未回讀到暫存器；HAL_TIMEOUT AUX 逾時；
 *         HAL_BUSY LoRaE22_Init() 的離線重試正在進行中，稍後重下即可。
 */
HAL_StatusTypeDef LoRaE22_SetPowerLevel(uint8_t pwr_level);

/**
 * @brief 動態修改 E22 空中速率（寫 REG0 bit[2:0]，保留 UART baud/parity 位元）。
 *        0=0.3k 1=1.2k 2=2.4k 3=4.8k 4=9.6k 5=19.2k 6=38.4k 7=62.5k。
 *        ★兩端（火箭/地面站）必須相同才能通訊；速率越低射程/餘裕越好。
 * @return HAL_OK 成功；HAL_ERROR 未初始化/未回讀到暫存器；HAL_TIMEOUT AUX 逾時；
 *         HAL_BUSY LoRaE22_Init() 的離線重試正在進行中，稍後重下即可。
 */
HAL_StatusTypeDef LoRaE22_SetAirRate(uint8_t air_rate);
void LoRaE22_GetParams(uint32_t *freq_mhz, uint8_t *pwr_level, uint8_t *air_rate);

/**
 * @brief 模組是否已開啟「每包附加 RSSI 位元組」（REG3 bit7）。
 *        開啟時模組每收一包會在酬載後多吐 1 個位元組（值 v → −(256−v) dBm）；
 *        接收端必須據此決定要不要多讀那個位元組，否則 framing 會差一個位元組
 *        （關閉卻硬讀 → 吃掉下一包的 sync0，RSSI 恆為假值 −91dBm）。
 * @return 1 = 已開啟；0 = 未開啟或尚未 probe 到暫存器（保守值，接收端不應多讀）。
 */
uint8_t LoRaE22_RssiByteEnabled(void);

/**
 * @brief 註冊「重新掛載 UART3 接收」的回呼，於本驅動每次離開設定模式後呼叫。
 *
 * 本驅動進出設定模式都要改 UART baud（設定模式恆 9600），而 HAL_UART_Init 會把
 * huart->RxState 打回 READY —— 進行中的 HAL_UARTEx_ReceiveToIdle_IT/DMA 就此失效
 * （IDLE 分支要求 RxState == BUSY_RX 才回呼），433 接收會靜默到重開機。
 * 驅動不知道誰在收 UART3（地面站遙測 / 主航電上行各一份），故由擁有者註冊。
 *
 * 呼叫時機必須在啟動接收「之前」（否則第一次設定模式的重掛會漏掉）。
 * 傳 NULL 可取消註冊。回呼在呼叫者的執行緒情境下執行，不是 ISR。
 */
void LoRaE22_SetRxRearmCallback(void (*cb)(void));

/**
 * @brief 是否正處於設定模式的阻塞輪詢收發窗口中（M1=1，UART3 暫時無 DMA 接收掛載）。
 *        供 USART3 錯誤中斷回呼（UplinkCmd_OnUart3Error/GroundStation_OnUart3Error）
 *        判斷：此時絕不可搶著重掛 DMA 接收，否則會把本驅動輪詢等待中的模組回應
 *        位元組吃掉，造成 `e22 freq/pwr/air` 每次都 st=3(HAL_TIMEOUT)、回讀全 0
 *        （中斷不受任何 RTOS 任務優先權節制，只能用此旗標主動避讓）。離開設定模式
 *        時本驅動會自行呼叫已註冊的 rearm 回呼，之後此旗標歸零、ISR 端才恢復重掛。
 * @return 1 = 設定模式輪詢中（ISR 端應跳過重掛）；0 = 一般透傳模式（照常重掛）。
 */
uint8_t LoRaE22_IsInConfigMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_E22_H */
