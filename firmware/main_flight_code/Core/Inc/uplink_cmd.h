/*
 * uplink_cmd.h — 火箭端上行命令處理（433 反向鏈路 → 手動開傘）
 * ===========================================================================
 * 主航電（FEATURE_UPLINK_DEPLOY）在下行遙測之外，於 USART3（E22 433 透傳）持續
 * 接收地面站上行命令（uplink_proto.h 框架）。命令解析、兩段式武裝（ARM→DEPLOY）、
 * ARM 逾時自動解除皆在本模組；實際點火動作（PD13 / TIM4）留在 main.c 飛控迴圈，
 * 經 UplinkCmd_TakeDeploy() 取出待辦旗標後執行（與既有 FSM 點火路徑並存）。
 *
 * 整檔以 #if FEATURE_UPLINK_DEPLOY 包住：備援/地面站編譯為空。
 */
#ifndef UPLINK_CMD_H
#define UPLINK_CMD_H

#include "stm32f4xx_hal.h"
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_UPLINK_DEPLOY

/** @brief 啟動 USART3 上行接收（ReceiveToIdle 中斷）。於 main() E22 init 成功後呼叫。 */
void UplinkCmd_Init(void);

/** @brief USART3 RxEvent 回呼（由 main.c HAL_UARTEx_RxEventCallback 轉接）。 */
void UplinkCmd_OnUart3RxEvent(uint16_t size);

/** @brief USART3 錯誤(ORE/雜訊)復原：清旗標並重新掛載 ReceiveToIdle。
 *  由 main.c 的 HAL_UART_ErrorCallback 在 huart->Instance==USART3 時呼叫。 */
void UplinkCmd_OnUart3Error(void);

/** @brief 處理已收位元組：解析框架、執行 ARM/DISARM、設定待辦開傘旗標、ARM 逾時解除。
 *  由低優先遙測任務每週期呼叫一次（解析不可放 1kHz 飛控迴圈）。 */
void UplinkCmd_Poll(uint32_t now_ms);

/**
 * @brief 取出並清除待辦的手動開傘請求（一次性消費）。由飛控迴圈呼叫。
 * @param want_drogue 輸出：1 = 本次有手動副傘請求
 * @param want_main   輸出：1 = 本次有手動主傘請求
 * @return 1 = 至少一項待辦（呼叫端應執行點火）；0 = 無。
 */
uint8_t UplinkCmd_TakeDeploy(uint8_t *want_drogue, uint8_t *want_main);

/**
 * @brief 直接排入一筆待辦手動開傘請求，繞過 433 幀解析（供本機 USB-CDC 文字命令
 *  "deploy drogue|main|both" 使用）。與 UPLINK_CMD_DEPLOY_* 走同一組 pending 旗標，
 *  確保「地面站 LoRa 觸發」與「主航電本機直連觸發」執行路徑完全一致。
 *  呼叫端須自行做 ARM/in_flight 閘（比照 uplink_cmd.c 對應 case）。
 * @param want_drogue 1 = 本次要副傘
 * @param want_main   1 = 本次要主傘
 */
void UplinkCmd_ForceDeploy(uint8_t want_drogue, uint8_t want_main);

/**
 * @brief 作廢尚未被取走的 pending 開傘請求。DISARM 專用（見 main.c Deploy_ResetLatches）：
 *  不清的話，DISARM 前那一瞬間收到的 DEPLOY 會在下次 ARM 後才被 TakeDeploy 取走並點火。
 */
void UplinkCmd_ClearPendingDeploy(void);

/**
 * @brief 取出並清除待辦的文字命令（一次性消費）。由診斷任務呼叫 → 餵 Parse_Serial_Command。
 * @param out  緩衝區，須 >= UPLINK_TEXT_MAX+1（含結尾 NUL）。
 * @param sz   out 大小。
 * @param seq  輸出：對應上行幀 seq（供 ACK 回填）。
 * @return 1 = 有一筆待辦文字命令；0 = 無。
 */
uint8_t UplinkCmd_TakeTextCmd(char *out, uint16_t sz, uint8_t *seq);

/**
 * @brief 取出並清除待辦的 bench（桌面測試）請求（一次性消費，已過 ARM 閘）。
 * @param seq 輸出：對應上行幀 seq（供 ACK 回填）。
 * @return 1 = 有待辦 bench；0 = 無。
 */
uint8_t UplinkCmd_TakeBench(uint8_t *seq);

/**
 * @brief 取出並清除待辦的 recovery（尋回指令）請求（一次性消費，停止蜂鳴器與紀錄）。
 * @param seq 輸出：對應上行幀 seq（供 ACK 回填）。
 * @return 1 = 有待辦 recovery；0 = 無。
 */
uint8_t UplinkCmd_TakeRecovery(uint8_t *seq);

/**
 * @brief 設定一筆待送 ACK（覆蓋前一筆未取走者）。由執行端（診斷任務）於命令執行後呼叫。
 * @param seq    對應命令 seq
 * @param status ACK_OK / ACK_UNKNOWN / ACK_BADARG / ACK_UNARMED / ACK_REJECTED（ack_proto.h）
 * @param text   echo 文字（可 NULL）；過長自動截斷至 ACK_TEXT_MAX。
 */
void UplinkCmd_SetAck(uint8_t seq, uint8_t status, const char *text);

/**
 * @brief 取出並清除待送 ACK（一次性消費）。由遙測任務呼叫後經無線送出。
 * @param seq/status 輸出。out_text 緩衝須 >= ACK_TEXT_MAX+1。out_len 輸出文字長度。
 * @return 1 = 有待送 ACK；0 = 無。
 */
uint8_t UplinkCmd_TakePendingAck(uint8_t *seq, uint8_t *status, char *out_text, uint8_t *out_len);

/** @brief 目前是否武裝（ARM 窗內）。 */
uint8_t UplinkCmd_IsArmed(void);

/** @brief 手動設定武裝狀態（供本地串口指令 arm/disarm 同步）。 */
void UplinkCmd_SetArmedState(uint8_t armed);

/** 上行接收診斷統計（由 main.c 的 1Hz [UPLINK_STAT] 行印出，供地面站/GUI 判讀）。 */
typedef struct {
    uint32_t raw_bytes;      /* USART3 原始位元組總數（含雜訊）：0 = 射頻層完全沒東西進來 */
    uint32_t bin_ok;         /* 二進制幀（ARM/DEPLOY/...）CRC 通過數 */
    uint32_t bin_crc_err;    /* 二進制幀湊滿但 CRC 不符 */
    uint32_t bin_resync;     /* 二進制幀 sync 對齊退回次數 */
    uint32_t text_ok;        /* 文字幀（tx 中繼）CRC 通過數 */
    uint32_t text_crc_err;   /* 文字幀湊滿但 CRC 不符 */
    uint8_t  last_cmd;       /* 最後一筆二進制命令碼 */
} UplinkCmdStats_t;

/** @brief 診斷統計快照。out 為 NULL 時直接返回。 */
void UplinkCmd_GetStats(UplinkCmdStats_t *out);

#else
static inline uint8_t UplinkCmd_IsArmed(void) { return 0U; }
#endif /* FEATURE_UPLINK_DEPLOY */

#ifdef __cplusplus
}
#endif

#endif /* UPLINK_CMD_H */
