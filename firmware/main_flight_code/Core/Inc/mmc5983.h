/**
  ******************************************************************************
  * @file           : mmc5983.h
  * @brief          : MMC5983MA 三軸地磁計驅動 (I2C1, 18-bit)
  *
  * MMC5983MA 實體掛載於 I2C1（SCL=PB7 / SDA=PB8，I2C 位址 0x30 7-bit）。
  * 採「一次性量測 (one-shot TM_M)」+「SET/RESET 橋偏校準」架構：
  *   - MMC5983_Recalibrate() 透過 SET 與 RESET 兩次量測估計各軸橋路 DC 偏移 (bridge offset)，
  *     結束時保持 SET 極性，供後續快速讀取使用。
  *   - MMC5983_Read() 僅觸發一次 TM_M 量測（快速），扣除已存的 offset 後輸出磁場 (Gauss)。
  * 主迴圈在 task context 呼叫（含 HAL_Delay 輪詢量測完成旗標），不在 ISR 內操作 I2C。
  *
  * 注意：driver 內 heading_deg 為「假設水平」的粗略診斷航向；真正的航向/yaw 修正
  * 由 EKF 以四元數做傾斜補償後融合（見 EKF_SubmitMag）。
  ******************************************************************************
  */
#ifndef __MMC5983_H
#define __MMC5983_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* --- I2C 位址 (HAL 需要 8-bit 左移後位址) --- */
#define MMC5983_I2C_ADDR_7BIT   (0x30U)
#define MMC5983_I2C_ADDR        (MMC5983_I2C_ADDR_7BIT << 1U)   /* 0x60 */

/* --- 暫存器位址 --- */
#define MMC5983_REG_XOUT0       (0x00U)   /* X[17:10] */
#define MMC5983_REG_XOUT1       (0x01U)   /* X[9:2]   */
#define MMC5983_REG_YOUT0       (0x02U)
#define MMC5983_REG_YOUT1       (0x03U)
#define MMC5983_REG_ZOUT0       (0x04U)
#define MMC5983_REG_ZOUT1       (0x05U)
#define MMC5983_REG_XYZOUT2     (0x06U)   /* 各軸最低 2 bits */
#define MMC5983_REG_TOUT        (0x07U)   /* 溫度 */
#define MMC5983_REG_STATUS      (0x08U)
#define MMC5983_REG_CTRL0       (0x09U)
#define MMC5983_REG_CTRL1       (0x0AU)
#define MMC5983_REG_CTRL2       (0x0BU)
#define MMC5983_REG_CTRL3       (0x0CU)
#define MMC5983_REG_PRODUCT_ID  (0x2FU)
#define MMC5983_PRODUCT_ID      (0x30U)   /* 預期 Product ID */

/* --- Control 0 (0x09) 位元 --- */
#define MMC5983_CTRL0_TM_M      (0x01U)   /* 觸發磁場量測（自動清除） */
#define MMC5983_CTRL0_TM_T      (0x02U)   /* 觸發溫度量測 */
#define MMC5983_CTRL0_SET       (0x08U)   /* SET 消磁脈衝（自動清除） */
#define MMC5983_CTRL0_RESET     (0x10U)   /* RESET 消磁脈衝（自動清除） */

/* --- Control 1 (0x0A) 位元 --- */
#define MMC5983_CTRL1_BW_100HZ  (0x00U)   /* 頻寬 100Hz：量測 ~8ms */
#define MMC5983_CTRL1_BW_200HZ  (0x01U)   /* 200Hz：~4ms */
#define MMC5983_CTRL1_BW_400HZ  (0x02U)   /* 400Hz：~2ms（本驅動採用） */
#define MMC5983_CTRL1_BW_800HZ  (0x03U)   /* 800Hz：~0.5ms */
#define MMC5983_CTRL1_SW_RST    (0x80U)   /* 軟體重置 */

/* --- Status (0x08) 位元 --- */
#define MMC5983_STATUS_MEAS_M_DONE  (0x01U)
#define MMC5983_STATUS_MEAS_T_DONE  (0x02U)

/* --- 18-bit 換算常數 --- */
#define MMC5983_ZERO_COUNT      (131072.0f)   /* 2^17 = 0 Gauss 中點 */
#define MMC5983_LSB_PER_GAUSS   (16384.0f)    /* 18-bit：16384 counts/Gauss */

/* --- 解析後的磁力計資料 --- */
typedef struct {
    uint8_t  ok;            /* 1 = Init/通訊正常 */
    int32_t  raw[3];        /* 最近一次 18-bit 原始計數 X/Y/Z */
    int32_t  offset[3];     /* SET/RESET 估計之橋路偏移 (counts) */
    int32_t  hard_iron_offset[3]; /* Flash 載入的環境硬鐵偏移 (counts) */
    float    gauss[3];      /* 校準後磁場 (Gauss) X/Y/Z */
    float    heading_deg;   /* 水平假設下的粗略磁航向 (deg, 0..360)；僅供診斷 */
    uint32_t reads_ok;      /* 成功量測次數 */
    uint32_t reads_err;     /* 通訊/逾時失敗次數 */
} MMC5983_Data_t;

/* --- API --- */

/* 初始化軟體 I2C 並配置 MMC5983MA：檢查 Product ID、軟體重置、設定頻寬、做一次 SET/RESET 橋偏校準。
 * 回傳 HAL_OK 表示晶片在線且校準完成。 */
HAL_StatusTypeDef MMC5983_Init(void);

/* SET/RESET 兩次量測，更新各軸橋路偏移 offset[]，結束時保持 SET 極性。
 * 建議每數秒呼叫一次以補償溫漂。 */
HAL_StatusTypeDef MMC5983_Recalibrate(void);

/* 觸發一次 TM_M 量測並讀取 18-bit X/Y/Z，扣除 offset[] 後更新 gauss[] 與 heading_deg（阻塞式）。 */
HAL_StatusTypeDef MMC5983_Read(void);

/* 取得最近一次量測結果（指向內部 static，唯讀）。 */
const MMC5983_Data_t* MMC5983_GetData(void);

/* 透過軟體 I2C 讀取最新的 7 位元組暫存器資料（適用於 100Hz 連續讀取，非阻塞）。 */
HAL_StatusTypeDef MMC5983_Read_Continuous(void);

/* 取得軟體低通濾波後的三軸磁力值 (Gauss)。 */
void MMC5983_GetFilteredGauss(float *x, float *y, float *z);
void MMC5983_SetOffsets(const float *offsets);

#ifdef __cplusplus
}
#endif

#endif /* __MMC5983_H */
