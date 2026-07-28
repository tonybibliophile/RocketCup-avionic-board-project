/**
 ******************************************************************************
 * @file    lora_e80.h
 * @brief   E80-900M2213S 920MHz LoRa 驅動 (Semtech LR1121, SPI3)
 *
 * ★ E80-900M2213S 的核心是 Semtech **LR1121**（非 SX126x）。命令協定差異大：
 *   16-bit opcode、頻率以 Hz 直接帶入、讀回應走獨立第二次 SPI 交易、LoRa 封包
 *   型態=0x02、單一位元組 sync word、32-bit IRQ、HP PA + DIO5/DIO6 RF 開關。
 *   詳見 lora_e80.c 檔頭。
 * 接線（連線和基本硬體規格表.md）：SPI3(SCK=PB3/MISO=PB4/MOSI=PB5),
 *   CS=PD7(CSB_LORA920), RST=PD5, IRQ(LR1121 DIO9)→PD4(EXTI4 rising), BUSY=PD6。
 * SPI3 與 W25Q128 Flash 共用 → 經 spi3_bus.h 的 SPI3_Bus_Lock/Unlock 互斥。
 *
 * ★ 板級參數（頻率/SF/BW/CR/PA/TCXO/RF-switch）集中於 lora_e80.c 頂部 #define。
 *   **務必於上板 bring-up 對照 E80-900M2213S 規格書逐項驗證**，尤其：
 *     - RF 開關真值表（DIO5=RFSW0 / DIO6=RFSW1）—— 設錯則收發 RF 完全不通；
 *     - TCXO 供電方式；IRQ 對應之 LR1121 DIO。見 docs LoRa 測試操作手冊。
 ******************************************************************************
 */
#ifndef __LORA_E80_H
#define __LORA_E80_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ============================================================
 *  模組啟用開關
 * ============================================================
 * LORA920_ENABLE = 0：開機不初始化 E80，改呼叫 LoRaE80_Shutdown() 把模組
 *   按在 reset（RST 拉低、CS 拉高）—— SX126x 於 NRST 拉低時所有腳位呈高阻，
 *   E80 因此「物理斷開」於 SPI3 共用的 SCK/MISO/MOSI，不會污染與其同匯流排
 *   的 W25Q128 Flash 讀寫（飛行資料記錄優先）。
 *   ★ E80 920MHz 鏈路目前已啟用（=1）。若 bring-up／共用匯流排異常需隔離，改回 0
 *     即停用並按住 reset 釋放 SPI3（保護同匯流排的 Flash 飛行記錄）。
 * LORA920_ENABLE = 1：正常初始化；若偵測失敗（含 MISO 接地/浮空）亦會自動
 *   走 Shutdown 隔離，故障的 E80 絕不被放任掛在 SPI3 上。 */
#ifndef LORA920_ENABLE
#define LORA920_ENABLE 1
#endif

/**
 * @brief 初始化 E80（重置 → standby → LoRa 參數 → DIO IRQ），並讀回 sync word 驗活。
 * @param hspi E80 所掛之 SPI（本專案為 &hspi3）。
 * @return HAL_OK 晶片在線且設定完成；HAL_ERROR/HAL_TIMEOUT 未偵測到模組
 *         （含早期 MISO 健康檢查失敗：GetStatus=0x00 接地 / 0xFF 浮空）。
 * @note  於 main() 初始化區（scheduler 啟動前）呼叫一次。
 */
HAL_StatusTypeDef LoRaE80_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief 安全停用 E80 並從 SPI3 匯流排隔離：CS 拉高 + RST 拉低並保持。
 *        SX126x NRST 拉低時全腳高阻 → E80 不再驅動共用 MISO，保護同匯流排
 *        的 W25Q128 Flash。停用後 LoRaE80_Send/IsReady 一律不觸碰 SPI3。
 *        僅操作 GPIO（不碰 SPI/mutex），可於 scheduler 啟動前安全呼叫。
 *        用於：①LORA920_ENABLE=0 主動停用；②Init 偵測失敗的故障隔離。
 */
void LoRaE80_Shutdown(void);

/** @brief 列印 E80 目前設定之 RF 參數 (頻率、功率、頻寬、SF 等) */
void LoRaE80_PrintConfig(void);

/**
 * @brief 非阻塞發送一筆 LoRa 封包。
 *        若上一筆 TX 尚未完成（DIO1 TxDone 未到且未逾時）則跳過本次回傳 HAL_BUSY，
 *        否則載入緩衝區並啟動 TX 後立即返回（不空等空中傳輸完成）。
 * @param len 封包長度 (1..255 bytes)。
 * @return HAL_OK 已啟動；HAL_BUSY 前次未完成本次跳過；HAL_ERROR 參數/未初始化。
 */
HAL_StatusTypeDef LoRaE80_Send(const uint8_t *data, uint8_t len);

/** @brief 是否可開始新的一筆發送（已初始化、BUSY 低、且無 TX 進行中）。 */
uint8_t LoRaE80_IsReady(void);

/** @brief DIO1(EXTI4) 中斷服務：設定 TxDone / RxDone 事件旗標。由 HAL_GPIO_EXTI_Callback 呼叫。 */
void LoRaE80_OnDio1IRQ(void);

/* ============================================================
 *  地面站接收（ROLE_GROUND）：E80 改作 LoRa 連續接收
 * ============================================================ */

/** @brief 設定並進入 LoRa 連續接收模式（RxDone/CrcErr/Timeout → DIO1）。
 *  須先 LoRaE80_Init() 設好 RF（頻率/SF/BW/CR/sync）。 */
HAL_StatusTypeDef LoRaE80_StartRx(void);

/** @brief DIO1 是否曾觸發（有封包待讀的輪詢提示）。 */
uint8_t LoRaE80_RxReady(void);

/**
 * @brief 讀出一筆已接收的 LoRa 封包。
 *        流程：GetIrqStatus → (CRC/表頭錯則丟) → GetRxBufferStatus → ReadBuffer →
 *        GetPacketStatus → 清 IRQ。連續接收模式下晶片維持 RX，無需重新 SetRx。
 * @param buf      輸出緩衝（須 >= 255 bytes）。
 * @param len      輸出 payload 長度。
 * @param rssi_dbm 輸出 RSSI (dBm)；可為 NULL。
 * @param snr_q    輸出 SNR（SX126x 原始有號值，dB = snr_q/4）；可為 NULL。
 * @return HAL_OK 取得有效封包；HAL_BUSY 尚無 RxDone；HAL_ERROR CRC/表頭錯或參數錯；
 *         HAL_TIMEOUT SPI/BUSY 逾時。
 */
HAL_StatusTypeDef LoRaE80_ReadPacket(uint8_t *buf, uint8_t *len, int16_t *rssi_dbm, int16_t *snr_q);

/** @brief 填入初始化診斷結果供週期性遙測輸出。gs = GetStatus 的 Stat1。 */
void LoRaE80_GetInitDiag(int *rd_st, uint8_t *busy, uint8_t *rb0, uint8_t *rb1, uint8_t *gs);

/**
 * @brief 開機校準後由 GetErrors 讀回的 16-bit 裝置錯誤旗標（0x0000 = 全正常）。
 *
 * 判讀 TCXO/射頻時鐘是否真的起來的唯一直接證據：設定類命令跑在 HF RC 振盪器上，
 * XOSC 全掛也照樣回 OK。bit5=HF_XOSC_START、bit7=PLL_LOCK 亮起即代表 TCXO 供電或
 * 電壓設定仍不正確，射頻不會work（表現為 SetTx 回 OK 但 TxDone 永遠不來）。
 */
uint16_t LoRaE80_GetErrors(void);

/**
 * @brief 開機掃描後實際採用的 TCXO 供電檔位（RegTcxoTune）；0xFF = 全部候選都起振失敗。
 *
 * 0x00=1.6V 0x01=1.7V 0x02=1.8V 0x03=2.2V 0x04=2.4V 0x05=2.7V 0x06=3.0V 0x07=3.3V。
 */
uint8_t LoRaE80_GetTcxoTune(void);

/* ============================================================
 *  地面站通訊測試：動態修改 RF 參數
 * ============================================================ */

/**
 * @brief 動態重新配置 E80 RF 參數並重啟連續接收。
 *        進入 Standby → 更新頻率/調變/功率/前導碼 → StartRx。
 *        LDRO 依 SF 與 BW 自動計算（符號時間 ≥ 16ms 時啟用）。
 * @param freq_hz  中心頻率 (Hz)，例如 920000000
 * @param sf       展頻因子 7~12（SX126x 值 = sf）
 * @param bw       頻寬 index（SX126x 定義：0x04=125kHz, 0x05=250kHz, 0x06=500kHz 等）
 * @param cr       編碼率 1~4（SX126x 定義：1=4/5, 2=4/6, 3=4/7, 4=4/8）
 * @param pwr_dbm  發射功率 dBm (-3~22)
 * @param preamble 前導碼符號數 (6~65535)
 * @return HAL_OK 已完成並重啟 RX；HAL_ERROR 未初始化；HAL_TIMEOUT SPI 逾時。
 * @note  僅在地面站 (IS_GROUND) 建置下有意義，其他角色仍可呼叫但通常無作用。
 */
HAL_StatusTypeDef LoRaE80_Reconfig(uint32_t freq_hz, uint8_t sf, uint8_t bw,
                                    uint8_t cr, int8_t pwr_dbm, uint16_t preamble);

/**
 * @brief 讀回目前套用中的 RF 參數（Init/Reconfig 成功套用時記錄）。
 *        供命令台 `e80 show` 與 GUI 查詢；任何指標可為 NULL。
 */
void LoRaE80_GetParams(uint32_t *freq_hz, uint8_t *sf, uint8_t *bw,
                       uint8_t *cr, int8_t *pwr_dbm, uint16_t *preamble);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_E80_H */
