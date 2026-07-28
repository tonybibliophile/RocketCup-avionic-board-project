/**
  ******************************************************************************
  * @file    usbd_cdc_if.c
  * @brief   CDC 介面層（手動移植，F407 FS）。整檔 #if FEATURE_USB_CDC。
  *          地面站只用 TX（CDC_Transmit_FS）；RX 端僅重新掛載接收避免端點卡死。
  ******************************************************************************
  */
#include "board_config.h"
#if FEATURE_USB_CDC

#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "gs_lora_test.h"

/* 收/送使用者緩衝 */
static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* 線路編碼（115200 8N1 預設；host 端設定會覆寫） */
static USBD_CDC_LineCodingTypeDef LineCoding = {
    115200U, /* baud */
    0x00U,   /* stop bits: 1 */
    0x00U,   /* parity: none */
    0x08U    /* data bits: 8 */
};

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    return (int8_t)USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    return (int8_t)USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;
    switch (cmd) {
        case CDC_SET_LINE_CODING:
            LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |
                                    (pbuf[2] << 16) | (pbuf[3] << 24));
            LineCoding.format     = pbuf[4];
            LineCoding.paritytype = pbuf[5];
            LineCoding.datatype   = pbuf[6];
            break;
        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t)(LineCoding.bitrate);
            pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
            pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
            pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
            pbuf[4] = LineCoding.format;
            pbuf[5] = LineCoding.paritytype;
            pbuf[6] = LineCoding.datatype;
            break;
        default:
            break;
    }
    return (int8_t)USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    /* 將 PC 傳入的 USB CDC 資料餵入指令緩衝區（地面站與航電板皆支援 USB CDC CLI） */
    GsLoraTest_FeedRxBuffer(Buf, *Len);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (int8_t)USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    (void)Buf; (void)Len; (void)epnum;
    return (int8_t)USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) {
        return (uint8_t)USBD_FAIL;
    }
    if (hcdc->TxState != 0) {
        return (uint8_t)USBD_BUSY;        /* 前一筆尚未送完 */
    }
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

uint8_t CDC_TxBusy(void)
{
    /* 前一筆 CDC 傳輸是否仍在進行（非同步，經 IN 端點中斷清除 TxState）。
     * 未列舉/未連線（pClassData 為 NULL）視為「不忙」，讓呼叫端據此直接放行/交由
     * CDC_Transmit_FS 回 USBD_FAIL。printf 直通 USB（見 main.c _write）用它判斷共用
     * 靜態緩衝是否可安全覆寫。 */
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) {
        return 0U;
    }
    return (hcdc->TxState != 0U) ? 1U : 0U;
}

#endif /* FEATURE_USB_CDC */
