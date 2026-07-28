/**
  ******************************************************************************
  * @file           : mmc5983.c
  * @brief          : MMC5983MA 三軸地磁計驅動 (軟體 I2C, 18-bit, 100Hz 輪詢) 實作
  ******************************************************************************
  */
#include "mmc5983.h"
#include "soft_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rate_monitor.h"
#include <math.h>
#include <stddef.h>

/* --- 內部組態 --- */
#define MMC5983_I2C_TIMEOUT_MS      (50U)    /* 單次 I2C 傳輸逾時 */
#define MMC5983_READY_TRIALS        (3U)     /* IsDeviceReady 重試次數 */
#define MMC5983_MEAS_TIMEOUT_MS     (10U)    /* 等待量測完成上限（400Hz BW ~2ms） */
#define MMC5983_SETRESET_SETTLE_MS  (1U)     /* SET/RESET 線圈脈衝後沉降時間 */
#define MMC5983_RAW_BYTES           (7U)     /* XOUT0..XYZOUT2 共 7 bytes */

/* CONTROL_2 (0x0B) 位元配置（datasheet p.16 / SparkFun·kriswiner 驅動同）：
 *   bits2:0 = CM_Freq（101 = 100Hz）
 *   bit3    = Cmm_en（連續量測模式致能）★ 不是 bit4！
 *   bits6:4 = Prd_set, bit7 = En_prd_set（本驅動不用，週期 SET 由軟體 Recalibrate 做）
 * 舊值 0x15 誤把 Cmm_en 當 bit4 → 連續模式根本沒開，輸出暫存器凍結，
 * 只剩每 10s Recalibrate 的 TM_M 更新一筆（且為 RESET 負极性）→
 * 「不同時間戳讀值全一樣」的根因。 */
#define MMC5983_CTRL2_CMM_100HZ     (0x0DU)  /* Cmm_en(0x08) | CM_Freq=101(0x05) */
#define MMC5983_CTRL2_CMM_OFF       (0x00U)

static MMC5983_Data_t     mmc_data;

/* --- 軟體低通濾波相關變數 --- */
static float filtered_gauss[3] = {0.0f, 0.0f, 0.0f};
static const float alpha = 0.1f;             /* 軟體一階 IIR 係數（截止頻率 ~1.76Hz） */

/* ------------------------------------------------------------------ */
/* 低階暫存器存取                                                       */
/* ------------------------------------------------------------------ */
static HAL_StatusTypeDef mmc_write_reg(uint8_t reg, uint8_t val)
{
    return SoftI2C_Mem_Write(MMC5983_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1U);
}

static HAL_StatusTypeDef mmc_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return SoftI2C_Mem_Read(MMC5983_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len);
}

/* 觸發一次 TM_M 量測、輪詢 STATUS 完成旗標、讀回 18-bit X/Y/Z 原始計數。
 * 用於初始化與 SET/RESET 校準階端。 */
static HAL_StatusTypeDef mmc_measure_raw(int32_t *x, int32_t *y, int32_t *z)
{
    uint8_t  status = 0U;
    uint8_t  raw[MMC5983_RAW_BYTES] = {0};
    uint32_t t0;

    if (mmc_write_reg(MMC5983_REG_CTRL0, MMC5983_CTRL0_TM_M) != HAL_OK) {
        return HAL_ERROR;
    }

    t0 = HAL_GetTick();
    do {
        if (mmc_read_regs(MMC5983_REG_STATUS, &status, 1U) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((status & MMC5983_STATUS_MEAS_M_DONE) != 0U) {
            break;
        }
    } while ((HAL_GetTick() - t0) < MMC5983_MEAS_TIMEOUT_MS);

    if ((status & MMC5983_STATUS_MEAS_M_DONE) == 0U) {
        return HAL_TIMEOUT;
    }

    if (mmc_read_regs(MMC5983_REG_XOUT0, raw, MMC5983_RAW_BYTES) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 18-bit 組裝：raw[0]=hi8, raw[1]=mid8, raw[6] 最低 2 bits */
    *x = (int32_t)(((uint32_t)raw[0] << 10) | ((uint32_t)raw[1] << 2) |
                   (((uint32_t)raw[6] >> 6) & 0x03U));
    *y = (int32_t)(((uint32_t)raw[2] << 10) | ((uint32_t)raw[3] << 2) |
                   (((uint32_t)raw[6] >> 4) & 0x03U));
    *z = (int32_t)(((uint32_t)raw[4] << 10) | ((uint32_t)raw[5] << 2) |
                   (((uint32_t)raw[6] >> 2) & 0x03U));
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* 公開 API                                                            */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef MMC5983_Init(void)
{
    uint8_t id = 0U;

    SoftI2C_Init();
    mmc_data.ok = 0U;
    mmc_data.reads_ok = 0U;
    mmc_data.reads_err = 0U;
    for (int i = 0; i < 3; i++) {
        mmc_data.raw[i] = 0;
        mmc_data.offset[i] = (int32_t)MMC5983_ZERO_COUNT;  /* 預設中點，待校準覆蓋 */
        mmc_data.hard_iron_offset[i] = 0;                 /* 預設無環境硬鐵偏置 */
        mmc_data.gauss[i] = 0.0f;
    }
    mmc_data.heading_deg = 0.0f;

    if (SoftI2C_IsDeviceReady(MMC5983_I2C_ADDR,
                              MMC5983_READY_TRIALS,
                              MMC5983_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    if (mmc_read_regs(MMC5983_REG_PRODUCT_ID, &id, 1U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (id != MMC5983_PRODUCT_ID) {
        return HAL_ERROR;
    }

    /* 軟體重置 → 等待開機 (datasheet 建議 ~10ms) */
    if (mmc_write_reg(MMC5983_REG_CTRL1, MMC5983_CTRL1_SW_RST) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(15);

    /* 設定頻寬 400Hz（內部量測 ~2ms，雜訊適中） */
    if (mmc_write_reg(MMC5983_REG_CTRL1, MMC5983_CTRL1_BW_400HZ) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 初次 SET/RESET 橋偏校準（同時把感測器留在 SET 極性） */
    if (MMC5983_Recalibrate() != HAL_OK) {
        return HAL_ERROR;
    }

    /* 初始化軟體濾波器基底值，防止開機時有階躍抖動 */
    filtered_gauss[0] = mmc_data.gauss[0];
    filtered_gauss[1] = mmc_data.gauss[1];
    filtered_gauss[2] = mmc_data.gauss[2];

    /* 啟用 100Hz 硬體連續量測模式（Cmm_en=bit3；定義與勘誤見檔頭 MMC5983_CTRL2_CMM_100HZ） */
    if (mmc_write_reg(MMC5983_REG_CTRL2, MMC5983_CTRL2_CMM_100HZ) != HAL_OK) {
        return HAL_ERROR;
    }

    mmc_data.ok = 1U;
    return HAL_OK;
}

/* SET/RESET 校準本體（呼叫前必須已暫停 CMM——CMM 下 TM_M 行為未定義） */
static HAL_StatusTypeDef mmc_recalibrate_body(void)
{
    int32_t s[3] = {0};
    int32_t r[3] = {0};

    /* 1) SET 脈衝 → +極性量測 */
    if (mmc_write_reg(MMC5983_REG_CTRL0, MMC5983_CTRL0_SET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(MMC5983_SETRESET_SETTLE_MS);
    if (mmc_measure_raw(&s[0], &s[1], &s[2]) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 2) RESET 脈衝 → −極性量測 */
    if (mmc_write_reg(MMC5983_REG_CTRL0, MMC5983_CTRL0_RESET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(MMC5983_SETRESET_SETTLE_MS);
    if (mmc_measure_raw(&r[0], &r[1], &r[2]) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 3) 橋偏 = (SET + RESET) / 2；磁場分量 = (SET − RESET) / 2 */
    for (int i = 0; i < 3; i++) {
        mmc_data.offset[i] = (s[i] + r[i]) / 2;
    }

    /* 4) 收尾再做一次 SET，讓感測器停在 +極性供後續連續量測 */
    if (mmc_write_reg(MMC5983_REG_CTRL0, MMC5983_CTRL0_SET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(MMC5983_SETRESET_SETTLE_MS);

    return HAL_OK;
}

HAL_StatusTypeDef MMC5983_Recalibrate(void)
{
    /* 暫停連續量測（初始化階段 CMM 尚未開，寫 0 無害），校準完不論成敗都恢復：
     * 若失敗仍把 CMM 關著，讀值會再度凍結——寧可帶著舊橋偏繼續量。 */
    (void)mmc_write_reg(MMC5983_REG_CTRL2, MMC5983_CTRL2_CMM_OFF);

    HAL_StatusTypeDef st = mmc_recalibrate_body();

    if (mmc_data.ok) {
        (void)mmc_write_reg(MMC5983_REG_CTRL2, MMC5983_CTRL2_CMM_100HZ);
    }
    return st;
}

/* 本函數為相容性保留，做阻塞式單次 one-shot 量測。 */
HAL_StatusTypeDef MMC5983_Read(void)
{
    int32_t x = 0, y = 0, z = 0;

    if (mmc_measure_raw(&x, &y, &z) != HAL_OK) {
        mmc_data.reads_err++;
        return HAL_ERROR;
    }

    mmc_data.raw[0] = x;
    mmc_data.raw[1] = y;
    mmc_data.raw[2] = z;

    /* 扣除橋偏與環境硬鐵偏移 → Gauss */
    mmc_data.gauss[0] = (float)(x - mmc_data.offset[0] - mmc_data.hard_iron_offset[0]) / MMC5983_LSB_PER_GAUSS;
    mmc_data.gauss[1] = (float)(y - mmc_data.offset[1] - mmc_data.hard_iron_offset[1]) / MMC5983_LSB_PER_GAUSS;
    mmc_data.gauss[2] = (float)(z - mmc_data.offset[2] - mmc_data.hard_iron_offset[2]) / MMC5983_LSB_PER_GAUSS;

    float h = atan2f(mmc_data.gauss[1], mmc_data.gauss[0]) * (180.0f / 3.14159265f);
    if (h < 0.0f) {
        h += 360.0f;
    }
    mmc_data.heading_deg = h;

    mmc_data.reads_ok++;
    return HAL_OK;
}

const MMC5983_Data_t* MMC5983_GetData(void)
{
    return &mmc_data;
}

/* ------------------------------------------------------------------ */
/* 軟體 I2C 連續讀取與濾波實作                                           */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef MMC5983_Read_Continuous(void)
{
    uint8_t raw[MMC5983_RAW_BYTES] = {0};

    if (!mmc_data.ok) {
        return HAL_ERROR;
    }

    /* 透過軟體 I2C 直接讀取最新的 7 位元組暫存器資料 (XOUT0 起頭) */
    if (mmc_read_regs(MMC5983_REG_XOUT0, raw, MMC5983_RAW_BYTES) != HAL_OK) {
        mmc_data.reads_err++;
        return HAL_ERROR;
    }

    /* 18-bit 組裝：raw[0]=hi8, raw[1]=mid8, raw[6] 最低 bits */
    int32_t x = (int32_t)(((uint32_t)raw[0] << 10) | ((uint32_t)raw[1] << 2) |
                         (((uint32_t)raw[6] >> 6) & 0x03U));
    int32_t y = (int32_t)(((uint32_t)raw[2] << 10) | ((uint32_t)raw[3] << 2) |
                         (((uint32_t)raw[6] >> 4) & 0x03U));
    int32_t z = (int32_t)(((uint32_t)raw[4] << 10) | ((uint32_t)raw[5] << 2) |
                         (((uint32_t)raw[6] >> 2) & 0x03U));

    mmc_data.raw[0] = x;
    mmc_data.raw[1] = y;
    mmc_data.raw[2] = z;

    /* 扣除動態橋偏與靜態硬鐵偏移後，換算為高斯 (Gauss) */
    float raw_gx = (float)(x - mmc_data.offset[0] - mmc_data.hard_iron_offset[0]) / MMC5983_LSB_PER_GAUSS;
    float raw_gy = (float)(y - mmc_data.offset[1] - mmc_data.hard_iron_offset[1]) / MMC5983_LSB_PER_GAUSS;
    float raw_gz = (float)(z - mmc_data.offset[2] - mmc_data.hard_iron_offset[2]) / MMC5983_LSB_PER_GAUSS;

    mmc_data.gauss[0] = raw_gx;
    mmc_data.gauss[1] = raw_gy;
    mmc_data.gauss[2] = raw_gz;

    /* 軟體一階 IIR 濾波器：y[k] = y[k-1] * 0.9 + x[k] * 0.1 */
    taskENTER_CRITICAL();
    filtered_gauss[0] = filtered_gauss[0] * (1.0f - alpha) + raw_gx * alpha;
    filtered_gauss[1] = filtered_gauss[1] * (1.0f - alpha) + raw_gy * alpha;
    filtered_gauss[2] = filtered_gauss[2] * (1.0f - alpha) + raw_gz * alpha;
    taskEXIT_CRITICAL();

    /* 更新航向角 (heading) */
    float h = atan2f(filtered_gauss[1], filtered_gauss[0]) * (180.0f / 3.14159265f);
    if (h < 0.0f) {
        h += 360.0f;
    }
    mmc_data.heading_deg = h;

    mmc_data.reads_ok++;

    /* 統計實際物理採樣率 (100 Hz) */
    RATE_TICK_MMC5983();

    return HAL_OK;
}

void MMC5983_GetFilteredGauss(float *x, float *y, float *z)
{
    taskENTER_CRITICAL();
    *x = filtered_gauss[0];
    *y = filtered_gauss[1];
    *z = filtered_gauss[2];
    taskEXIT_CRITICAL();
}

void MMC5983_SetOffsets(const float *offsets)
{
    if (offsets != NULL) {
        /* offsets[i] 為開機橋偏與硬鐵偏移的合計值。
         * 我們在此藉由減去已測得之橋偏，分離並還原出純粹的環境硬鐵偏移。 */
        mmc_data.hard_iron_offset[0] = (int32_t)offsets[0] - mmc_data.offset[0];
        mmc_data.hard_iron_offset[1] = (int32_t)offsets[1] - mmc_data.offset[1];
        mmc_data.hard_iron_offset[2] = (int32_t)offsets[2] - mmc_data.offset[2];
    }
}
