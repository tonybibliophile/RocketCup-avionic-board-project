/*
 * ground_station.h — 地面站接收器主流程（ROLE_GROUND）
 * ===========================================================================
 * 雙鏈路接收火箭下行遙測（E22 433 透傳 / E80 920 SX126x）+ 讀自身 GPS，
 * 對齊時間戳後記錄到 SD（CSV）+ Flash（二進位 append）並經 USB-CDC 串流給 PC。
 * 由 main.c 的 defaultTask 在 IS_GROUND 時呼叫 GroundStation_Run()（不返回）。
 *
 * 本檔內容整體以 #if IS_GROUND 包住：主/備航電編譯為空，零影響。
 */
#ifndef GROUND_STATION_H
#define GROUND_STATION_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 地面站主迴圈（不返回）。由 StartDefaultTask 在 IS_GROUND 時呼叫。 */
void GroundStation_Run(void);

/** @brief USART3（E22 433 透傳）RxEvent 回呼：把收到的 Size bytes 推入位元組環形緩衝。
 *  由 main.c 的 HAL_UARTEx_RxEventCallback 在 huart->Instance==USART3 時呼叫。 */
void GroundStation_OnUart3RxEvent(uint16_t Size);

/** @brief USART3 錯誤(ORE/雜訊)復原：清旗標並重新掛載 ReceiveToIdle。
 *  由 main.c 的 HAL_UART_ErrorCallback 在 huart->Instance==USART3 時呼叫。
 *  不做則一次溢位就讓 433 RX 永久停擺（收一包後 raw 凍結）。 */
void GroundStation_OnUart3Error(void);

/** @brief 手動抹除後重設 Flash 寫入頭（gs_flash_append 的 append 指標）。
 *  僅供 gs_lora_test.c 的 `flash erase` 命令在 FlashRing_EraseAll() 之後呼叫——
 *  該函式已把整段 Flash 實體擦淨，這裡只是把地面站自己的 write/erase 指標重設
 *  回起點，讓下一筆收到的封包從頭開始 append。 */
void GroundStation_FlashResetAfterErase(void);

#ifdef __cplusplus
}
#endif

#endif /* GROUND_STATION_H */
