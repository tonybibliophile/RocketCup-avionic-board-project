/**
  ******************************************************************************
  * @file           : bmp388.c
  * @brief          : BMP388 Driver implementation using official Bosch BMP3 API
  ******************************************************************************
  */
#include "bmp388.h"
#include "bmp3.h"
#include <string.h>
#include <math.h>

/* Bosch API 最大單次 SPI 傳輸長度 (len + 1 dummy byte + 1 addr):
 * 校正係數讀取最多 21 bytes + overhead 。定義 64 保留充足餲量。 */
#define BMP388_MAX_SPI_BUF  64U

/* --- 官方 API 所需的 device 實體 --- */
static struct bmp3_dev bmp_dev;

/* === 高度換算物理常數 (Item H) === */
#define BMP388_SEA_LEVEL_PA   101325.0f   /* 參考氣壓；相對高度計算時會完全抵銷，僅作為絕對值零點 */
#define BMP388_R_SPECIFIC     287.052874f /* 乾空氣比氣體常數 J/(kg·K) */
#define BMP388_GRAVITY        9.80665f    /* 標準重力加速度 m/s² */

/* === 高度換算參考溫度 (Item H) ===
 * 預設 ISA 標準海平面溫度 288.15 K (15°C)；開機自檢時由 BMP388_SetReferenceTemp()
 * 以實測發射台溫度鎖定，鎖定後全程固定不變。確保 EKF 的 launchpad 參考與飛行讀數
 * 共用同一個 T0 → 相對高度 (baro_alt - launchpad) 在發射台恆為 0 且全程連續，
 * 不會在校準完成瞬間產生階躍，故不影響 FSM 頂點偵測。 */
static float bmp388_ref_temp_K = 288.15f;

/* --- SPI 低階通訊與延時回調函式實作 --- */

static BMP3_INTF_RET_TYPE BMP388_SPI_Read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)intf_ptr;

    // Bosch bmp3.c 內部已將 len 加上 dummy_byte(=1) 後才呼叫 callback。
    // 尤此，callback 收到的 len = 真實資料長度 + 1 (dummy)。
    // HAL 傳輸總長: 1 (addr) + len (dummy + 資料) = len + 1
    uint32_t transfer_len = len + 1U;
    if (transfer_len > BMP388_MAX_SPI_BUF) return -1; // 防止溢位

    uint8_t tx_buf[BMP388_MAX_SPI_BUF] = {0};
    uint8_t rx_buf[BMP388_MAX_SPI_BUF] = {0};

    tx_buf[0] = reg_addr | 0x80; // Read mode: MSB = 1 (已由 bmp3.c 設定)

    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, transfer_len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_SET);

    if (status == HAL_OK) {
        // rx_buf[0]: addr 回波垃圾，跳過
        // rx_buf[1..len]: dummy byte + 資料 — 全部回傳給 API，API 自己跳 dummy
        memcpy(read_data, &rx_buf[1], len);
        return BMP3_INTF_RET_SUCCESS;
    }
    return -1; // 失敗
}

static BMP3_INTF_RET_TYPE BMP388_SPI_Write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)intf_ptr;

    // SPI Write: 1 addr + len data
    uint32_t transfer_len = len + 1U;
    if (transfer_len > BMP388_MAX_SPI_BUF) return -1; // 防止溢位

    uint8_t tx_buf[BMP388_MAX_SPI_BUF] = {0};
    uint8_t rx_buf[BMP388_MAX_SPI_BUF] = {0};

    tx_buf[0] = reg_addr & 0x7F; // Write mode: MSB = 0
    memcpy(&tx_buf[1], write_data, len);

    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, transfer_len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_SET);

    if (status == HAL_OK) {
        return BMP3_INTF_RET_SUCCESS;
    }
    return -1; // 失敗
}

static void BMP388_Delay_Us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    // HAL_Delay 是毫秒級別延遲，向上取整以防不足
    uint32_t ms = (period + 999) / 1000;
    HAL_Delay(ms);
}

/* --- 公開驅動對接介面 --- */

HAL_StatusTypeDef BMP388_Init(SPI_HandleTypeDef *hspi, BMP388_Data_t *data)
{
    int8_t rslt = BMP3_E_DEV_NOT_FOUND;
    
    // 確保 Chip Select 引腳處於高電平空閒狀態
    HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
    
    // 配置設備結構體
    bmp_dev.intf_ptr = hspi;
    bmp_dev.intf = BMP3_SPI_INTF;
    bmp_dev.read = BMP388_SPI_Read;
    bmp_dev.write = BMP388_SPI_Write;
    bmp_dev.delay_us = BMP388_Delay_Us;
    
    // 嘗試初始化晶片 (連續嘗試最多 5 次以防模式切換未穩)
    for (int retry = 0; retry < 5; retry++) {
        rslt = bmp3_init(&bmp_dev);
        if (rslt == BMP3_OK) {
            break;
        }
        HAL_Delay(10);
    }
    
    if (rslt != BMP3_OK) {
        return HAL_ERROR;
    }
    
    // 配置感測器參數結構
    struct bmp3_settings settings = { 0 };
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    /* 50Hz ODR + 8x 壓力過採樣：高度是低頻資訊，200Hz/1x 純屬浪費且雜訊大，換成
     * 低速高精度直接改善頂點/開傘判定品質。合法性驗算（bmp3.c:verify_meas_time_and_odr_duration）：
     * meas_t = 234 + (392+8×2000) + (313+1×2000) = 18939µs < 20000µs(50Hz 週期)，餘裕 1.06ms。
     * ⚠️ temp_os 必須維持 1x：若提到 2x，meas_t=20939µs > 20000µs，驅動會回傳
     * BMP3_E_INVALID_ODR_OSR_SETTINGS，BMP388_Init 整個回 HAL_ERROR，bmp388_ok=0，
     * 整條氣壓鏈無聲失效（無 log 提示原因）。 */
    settings.odr_filter.press_os   = BMP3_OVERSAMPLING_8X;    // 氣壓 8x 過採樣
    settings.odr_filter.temp_os    = BMP3_NO_OVERSAMPLING;    // 溫度必須維持 1x，見上方註解
    settings.odr_filter.odr        = BMP3_ODR_50_HZ;          // 50Hz ODR
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    
    uint16_t settings_sel = BMP3_SEL_PRESS_EN | 
                            BMP3_SEL_TEMP_EN | 
                            BMP3_SEL_PRESS_OS | 
                            BMP3_SEL_TEMP_OS | 
                            BMP3_SEL_ODR | 
                            BMP3_SEL_IIR_FILTER;
    
    // 寫入配置到晶片
    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &bmp_dev);
    if (rslt != BMP3_OK) {
        return HAL_ERROR;
    }
    
    // 將晶片設為 Normal Mode
    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &bmp_dev);
    if (rslt != BMP3_OK) {
        return HAL_ERROR;
    }
    
    HAL_Delay(20);

    return HAL_OK;
}

/* 讀取率(main.c 100Hz 輪詢) > ODR(50Hz) 時，晶片內部時鐘與 MCU 各自獨立計時，若不看
 * drdy 直接讀，會偶爾在同一顆轉換結果上重複讀到兩次（拍頻），人為壓低量測噪聲、
 * 也讓 EKF/VF 誤以為餵入了新資訊。呼叫端應在 drdy=1 時才呼叫 BMP388_ReadData()——
 * 該呼叫走 bmp3_get_sensor_data() 讀資料暫存器，會一併清除本旗標。 */
uint8_t BMP388_IsDataReady(void)
{
    struct bmp3_status st = {0};
    if (bmp3_get_status(&st, &bmp_dev) != BMP3_OK) {
        return 0U;
    }
    return st.intr.drdy ? 1U : 0U;
}

HAL_StatusTypeDef BMP388_ReadData(SPI_HandleTypeDef *hspi, BMP388_Data_t *data)
{
    (void)hspi;
    int8_t rslt;
    struct bmp3_data sensor_data;
    
    // 調用官方 API 讀取並自動進行高精度補償計算
    rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &sensor_data, &bmp_dev);
    if (rslt != BMP3_OK) {
        return HAL_ERROR;
    }
    
    // 存入公開數據結構體
    data->pressure = (float)sensor_data.pressure;
    data->temperature = (float)sensor_data.temperature;
    
    // 換算為高度：採用 hypsometric（等溫層）公式，以鎖定的發射台溫度 T0 修正高度尺度。
    //   h = (R / g) · T0 · ln(P0 / P)
    // P0 (BMP388_SEA_LEVEL_PA) 僅為絕對值零點，相對高度 (baro_alt - launchpad) 計算時會抵銷；
    // 真正影響相對高度尺度的是 T0：高溫環境 T0 較大 → 同一壓降對應更大的實際高度。
    // 原 ISA 定溫公式 (44330·(1-(P/P0)^0.190295)) 固定假設 15°C，於高/低溫環境會有數 % 的尺度誤差 (Item H)。
    data->altitude = (BMP388_R_SPECIFIC / BMP388_GRAVITY) * bmp388_ref_temp_K
                     * logf(BMP388_SEA_LEVEL_PA / data->pressure);

    // 診斷暫存器讀取在正常運行中省略，以避免不必要的 SPI 傳輸 overhead
    // 如需診斷，可手動呼叫 bmp3_get_regs(BMP3_REG_ERR, ...) 等

    return HAL_OK;
}

/* 以實測發射台溫度 (°C) 鎖定高度換算參考溫度 T0。
 * 僅接受 BMP388 合理工作範圍 (-40~85°C) 的讀數以過濾開機異常值；超出範圍則維持
 * 預設 288.15 K（行為等同未修正）。必須在 EKF 校準前呼叫一次，之後不應再變更 (Item H)。 */
void BMP388_SetReferenceTemp(float temp_c)
{
    if (temp_c >= -40.0f && temp_c <= 85.0f) {
        bmp388_ref_temp_K = temp_c + 273.15f;
    }
}
