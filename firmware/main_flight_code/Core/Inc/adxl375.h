/**
  ******************************************************************************
  * @file           : adxl375.h
  * @brief          : Header for adxl375.c file.
  *                   This file contains the common defines of the ADXL375 High-G sensor.
  ******************************************************************************
  */
#ifndef __ADXL375_H
#define __ADXL375_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* --- ADXL375 SPI Chip Select Port/Pin --- */
#define HIGHG_CS_PORT    GPIOC
#define HIGHG_CS_PIN     GPIO_PIN_4

/* --- ADXL375 Registers --- */
#define ADXL375_DEVID_REG          0x00
#define ADXL375_DEVID_VAL          0xE5
#define ADXL375_OFSX_REG           0x1E   /* 硬體 offset，8-bit 二補數，1.56 g/LSB，斷電不保留 */
#define ADXL375_OFSY_REG           0x1F
#define ADXL375_OFSZ_REG           0x20
#define ADXL375_BW_RATE_REG        0x2C
#define ADXL375_POWER_CTL_REG      0x2D
#define ADXL375_DATA_FORMAT_REG    0x31
#define ADXL375_DATAX0_REG         0x32
#define ADXL375_OFS_SCALE_G_PER_LSB 1.56f

/* --- High-G Data Struct --- */
typedef struct {
    // 原始 16-bit 數據
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;

    // 物理量數據 (單位: g)
    float ax;
    float ay;
    float az;
} ADXL375_Data_t;

/* --- Function Prototypes --- */
HAL_StatusTypeDef ADXL375_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef ADXL375_ReadData(SPI_HandleTypeDef *hspi, ADXL375_Data_t *data);
void ADXL375_DumpDiag(SPI_HandleTypeDef *hspi);   /* 開機一次性：配置回讀 + raw hex 傾印 */
void ADXL375_CalibrateAgainstRef(SPI_HandleTypeDef *hspi, float ref_ax, float ref_ay, float ref_az);
    /* 以外部參考重力向量（來自 BMI088，同一原生軸慣例，見檔頭軸向備註）校正
     * OFSX/OFSY/OFSZ，補償封裝應力零點偏移（實測平放 z 僅 20mG，應 ≈1000mG）。 */

#ifdef __cplusplus
}
#endif

#endif /* __ADXL375_H */
