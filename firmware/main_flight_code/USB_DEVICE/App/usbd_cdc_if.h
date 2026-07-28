/**
  ******************************************************************************
  * @file    usbd_cdc_if.h
  * @brief   CDC 介面層標頭（手動移植）。CDC_Transmit_FS 為地面站串流 USB 的入口。
  ******************************************************************************
  */
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

#define APP_RX_DATA_SIZE  2048U
#define APP_TX_DATA_SIZE  2048U

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/** @brief 經 CDC 傳一段資料給 PC。回傳 USBD_OK / USBD_BUSY（前一筆未送完）。 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/** @brief 前一筆 CDC 傳輸是否仍在進行中（未連線視為不忙，回 0）。 */
uint8_t CDC_TxBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
