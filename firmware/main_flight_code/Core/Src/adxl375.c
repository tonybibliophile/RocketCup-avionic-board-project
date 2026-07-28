/**
  ******************************************************************************
  * @file           : adxl375.c
  * @brief          : ADXL375 High-G Accelerometer Driver Implementation
  ******************************************************************************
  */
#include "adxl375.h"
#include <stdio.h>   /* ADXL375_DumpDiag 開機診斷列印 */

/* 本檔的診斷/校正迴圈總阻塞 ~680ms，且全部落在開機序列「IWDG 已啟動但主迴圈尚未
 * 進入」的視窗內（main.c：MX_IWDG_Init 之後、GPS_Init 的第一次餵狗之前，全段無人餵狗）。
 * 必須自行餵狗，否則與其他開機延遲累加即撐爆 IWDG(~2.05s) → 開機無限重啟。
 * 慣例同 gps.c（同樣位於該視窗內的驅動）。 */
extern IWDG_HandleTypeDef hiwdg;

/* 軟體零點偏移補償（g）：硬體 OFS 暫存器 1.56 g/LSB 太粗，修不了百 mG 等級的
 * 封裝應力偏移（見 ADXL375_CalibrateAgainstRef）。ReadData 每筆疊加，全解析度
 * 不受暫存器格數限制。開機預設 0（未校正時原樣輸出，行為不變）。 */
static float s_sw_bias_ax = 0.0f, s_sw_bias_ay = 0.0f, s_sw_bias_az = 0.0f;

/* --- SPI Low-Level Helper Functions (使用 TransmitReceive 確保時鐘與清除標誌) --- */

static HAL_StatusTypeDef ADXL375_Reg_Write(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    tx_data[0] = reg & 0x7F; // Write mode: MSB = 0
    tx_data[1] = val;

    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_SET);

    return status;
}

static HAL_StatusTypeDef ADXL375_Reg_Read(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};

    tx_data[0] = reg | 0x80; // Read mode: MSB = 1

    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, HAL_MAX_DELAY); // 1 addr + 1 data = 2 bytes
    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_SET);

    if (status == HAL_OK) {
        *val = rx_data[1]; // Data is returned in the 2nd byte
    }
    return status;
}

/* --- Public Driver Functions --- */

HAL_StatusTypeDef ADXL375_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t chip_id = 0;
    HAL_StatusTypeDef status;

    // 確保 Chip Select 引腳處於高電平空閒狀態
    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    // 讀取 Chip ID (應為 0xE5)
    status = ADXL375_Reg_Read(hspi, ADXL375_DEVID_REG, &chip_id);
    if (status != HAL_OK || chip_id != ADXL375_DEVID_VAL) {
        return HAL_ERROR;
    }

    // 配置數據格式：全解析度 (Full Resolution)、±200g 量程、4線 SPI
    status = ADXL375_Reg_Write(hspi, ADXL375_DATA_FORMAT_REG, 0x0B);
    HAL_Delay(10);

    /* 強制歸零 OFS 暫存器：datasheet「斷電才清零」指的是感測器晶片本身斷電，
     * MCU 端 SWD/軟體重置不會斷 ADXL375 的 VDD——上一輪測試殘留的 OFS 值會
     * 一直留著（實測踩過：OFSZ 殘留 -1 LSB 疊加新校正，結果對不上預期）。
     * 現行校正全走軟體 bias（見 ADXL375_CalibrateAgainstRef），OFS 硬體暫存器
     * 本就不使用，開機強制清零避免任何殘留污染。 */
    ADXL375_Reg_Write(hspi, ADXL375_OFSX_REG, 0x00);
    ADXL375_Reg_Write(hspi, ADXL375_OFSY_REG, 0x00);
    ADXL375_Reg_Write(hspi, ADXL375_OFSZ_REG, 0x00);

    /* ODR 800Hz（0x0D）。★官方規格，非實測猜測：datasheet Rev.B p.28「DATA
     * FORMATTING AT OUTPUT DATA RATES OF 3200 HZ AND 1600 HZ」明文：
     * "When using the 3200 Hz or 1600 Hz output data rate, the LSB of the
     * output data-word is always 0." ——這兩檔最高速率官方就是少 1-bit 解析度
     * 換速度，不是雜訊/接線/驅動問題（實測 1600/3200 皆 LSB 恆 0 與此完全吻合，
     * self-test Δ+105~110 LSB 正常已排除 die 損壞）。800Hz 為全解析度最高速率。
     * 對 100Hz 的 FSM/診斷消費端仍 8× 過採樣，頻寬足夠。 */
    status = ADXL375_Reg_Write(hspi, ADXL375_BW_RATE_REG, 0x0D);
    HAL_Delay(10);

    // 啟動測量模式：將 POWER_CTL 暫存器設為 Measure (0x08)
    status = ADXL375_Reg_Write(hspi, ADXL375_POWER_CTL_REG, 0x08);
    HAL_Delay(20);

    return HAL_OK;
}

/* 開機一次性診斷：回讀配置暫存器 + 傾印原始 16-bit 資料（hex）。
 * 用途：區分「配置寫入未生效 / 偏移暫存器髒值 / 晶片本體輸出異常」——
 * 實測資料 raw 恆偶數且無重力，DEVID(0xE5, bit0=1) 卻讀取正常，需以
 * 回讀+hex 傾印定位。預期值：1E/1F/20(OFS)=00, 2C(BW_RATE)=0F,
 * 2D(POWER_CTL)=08, 31(DATA_FORMAT)=0B, 38(FIFO_CTL)=00。 */
void ADXL375_DumpDiag(SPI_HandleTypeDef *hspi)
{
    static const uint8_t regs[] = { ADXL375_DEVID_REG, ADXL375_OFSX_REG, ADXL375_OFSY_REG,
                                     ADXL375_OFSZ_REG, ADXL375_BW_RATE_REG, ADXL375_POWER_CTL_REG,
                                     ADXL375_DATA_FORMAT_REG, 0x38 };
    printf("[HIGHG_DIAG] regs:");
    for (unsigned i = 0; i < sizeof(regs); i++) {
        uint8_t v = 0xAA;
        (void)ADXL375_Reg_Read(hspi, regs[i], &v);
        printf(" %02X=%02X", regs[i], v);
    }
    printf("\r\n");
    for (int s = 0; s < 4; s++) {
        HAL_IWDG_Refresh(&hiwdg);
        ADXL375_Data_t d;
        if (ADXL375_ReadData(hspi, &d) == HAL_OK) {
            printf("[HIGHG_DIAG] raw hex x=%04X y=%04X z=%04X\r\n",
                   (uint16_t)d.x_raw, (uint16_t)d.y_raw, (uint16_t)d.z_raw);
        }
        HAL_Delay(2);
    }

    /* Self-test（DATA_FORMAT bit7）：對 Z 軸施加靜電力，正品典型 Δz ≈ +6.4g
     * ≈ +130 LSB（datasheet ST output change, 3.3V）。Δ≈0 或亂跳 → MEMS/die
     * 異常（含仿冒品），類比端問題再多也擋不住靜電力響應。結束後還原 0x0B。
     * 取樣數 80 筆＝datasheet p.28 建議「0.1 秒份量資料」（800Hz×0.1s=80），
     * 原 8 筆平均效果打折、判定窗才需放寬到 ±25%；80 筆後雜訊平均更確實。 */
    {
        ADXL375_Data_t d;
        int32_t z_off = 0, z_on = 0;
        for (int s = 0; s < 80; s++) {
            HAL_IWDG_Refresh(&hiwdg);
            if (ADXL375_ReadData(hspi, &d) == HAL_OK) z_off += d.z_raw;
            HAL_Delay(2);
        }
        ADXL375_Reg_Write(hspi, ADXL375_DATA_FORMAT_REG, 0x8B);
        HAL_Delay(50);
        for (int s = 0; s < 80; s++) {
            HAL_IWDG_Refresh(&hiwdg);
            if (ADXL375_ReadData(hspi, &d) == HAL_OK) z_on += d.z_raw;
            HAL_Delay(2);
        }
        ADXL375_Reg_Write(hspi, ADXL375_DATA_FORMAT_REG, 0x0B);
        HAL_Delay(20);
        HAL_IWDG_Refresh(&hiwdg);
        printf("[HIGHG_DIAG] selftest z_off=%ld z_on=%ld delta=%ld LSB (正品期望 +100~+160)\r\n",
               (long)(z_off / 80), (long)(z_on / 80), (long)((z_on - z_off) / 80));
    }
}

/* 以 BMI088 平均值為參考，校正 ADXL375 零點偏移（封裝應力，非 die 損壞）。
 * 平放實測：BMI088 乾淨（z σ=5mG），ADXL375 三軸全偏（z mean=20mG，應≈1000mG）。
 * self-test 已確認 MEMS/die 正常。★純軟體補償，不寫硬體 OFS 暫存器：
 *   1) 1.56 g/LSB 對百 mG 級偏移太粗，多數情況四捨五入成 0，形同無效；
 *   2) 實測寫入 OFSZ=-1 LSB 後，讀值與 datasheet「自動疊加」描述的預期差了
 *      >1g（可能是暫存器內部縮放與 DATA_FORMAT 其他位元有未查明的交互作用），
 *      沒有把握的黑盒行為不值得依賴——ADXL 已非飛控關鍵路徑，軟體加法是
 *      100% 可控、可驗證、無格數限制的替代方案。OFS 暫存器維持預設 0x00 不動。
 * 參考向量非假設理想擺放，而是 BMI088 當下實測的真實姿態，經呼叫端（main.c）
 * 依 sensor_axis.h 轉換為「ADXL375 原生座標系」下的目標值——★BMI088 與 ADXL375
 * 兩顆晶片實體貼裝方向不同（BMI088 晶片 Z 朝下、X/Y 對調；ADXL375 晶片 Z 朝上、
 * X/Y 直接取負，見 tests/test_sensor_axis.c 的貼裝反推），原生讀值不可直接逐軸
 * 相減比較，呼叫端已處理好座標轉換，本函式只單純做「原生對原生」的差值。 */
void ADXL375_CalibrateAgainstRef(SPI_HandleTypeDef *hspi, float ref_ax, float ref_ay, float ref_az)
{
    ADXL375_Data_t d;
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    int n = 0;
    for (int s = 0; s < 40; s++) {
        HAL_IWDG_Refresh(&hiwdg);
        if (ADXL375_ReadData(hspi, &d) == HAL_OK) {
            sum_ax += d.ax; sum_ay += d.ay; sum_az += d.az;
            n++;
        }
        HAL_Delay(2);
    }
    if (n == 0) {
        printf("[HIGHG_CAL] SKIP：讀取全部失敗\r\n");
        return;
    }
    float meas_ax = sum_ax / (float)n;
    float meas_ay = sum_ay / (float)n;
    float meas_az = sum_az / (float)n;

    /* ReadData 此刻的 bias 仍是上次（或預設 0）的值，尚未套用本次校正，
     * meas_* 即「零校正前」的原始讀值，減出來的差額即完整所需補償量。 */
    s_sw_bias_ax += ref_ax - meas_ax;
    s_sw_bias_ay += ref_ay - meas_ay;
    s_sw_bias_az += ref_az - meas_az;

    printf("[HIGHG_CAL] BMI_ref=(%ld,%ld,%ld)mg meas=(%ld,%ld,%ld)mg -> SW_bias=(%ld,%ld,%ld)mg\r\n",
           (long)(ref_ax * 1000.0f), (long)(ref_ay * 1000.0f), (long)(ref_az * 1000.0f),
           (long)(meas_ax * 1000.0f), (long)(meas_ay * 1000.0f), (long)(meas_az * 1000.0f),
           (long)(s_sw_bias_ax * 1000.0f), (long)(s_sw_bias_ay * 1000.0f), (long)(s_sw_bias_az * 1000.0f));
}

HAL_StatusTypeDef ADXL375_ReadData(SPI_HandleTypeDef *hspi, ADXL375_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[7] = {0};
    uint8_t rx_data[7] = {0}; // 1 addr + 6 bytes data

    // 讀取 6 位元組連續數據 (DATAX0 至 DATAZ1)
    // Read Mode (Bit 7 = 1) + Multi-Byte Mode (Bit 6 = 1)
    tx_data[0] = ADXL375_DATAX0_REG | 0x80 | 0x40; // 0xF2

    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 7, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(HIGHG_CS_PORT, HIGHG_CS_PIN, GPIO_PIN_SET);

    if (status != HAL_OK) return status;

    // 解析原始 16-bit 補碼數據 (LSB + MSB，跳過 rx_data[0] 的 address echo)
    //   rx_data[0] = dummy/addr echo
    //   rx_data[1] = DATAX0 (LSB), rx_data[2] = DATAX1 (MSB)
    //   rx_data[3] = DATAY0 (LSB), rx_data[4] = DATAY1 (MSB)
    //   rx_data[5] = DATAZ0 (LSB), rx_data[6] = DATAZ1 (MSB)
    data->x_raw = (int16_t)((rx_data[2] << 8) | rx_data[1]);
    data->y_raw = (int16_t)((rx_data[4] << 8) | rx_data[3]);
    data->z_raw = (int16_t)((rx_data[6] << 8) | rx_data[5]);

    // 根據說明書，ADXL375 的解析度固定為 49mg/LSB = 0.049g/LSB
    // 軸向備註：PCB 標示與感測器晶片本體 X/Y 相反。為與 BMI088 一致，本驅動「不」在此
    // 做軸向取負——BMI088 同樣於驅動層輸出感測器原生軸，軸反向修正集中於 EKF 處理
    // （比照 sensor_axis.h 統一映射）。★這只代表「兩者都是未經 body-frame 映射的
    // 原生輸出」，不代表兩者原生數值可直接逐軸比較/相減——BMI088 與 ADXL375 實體
    // 貼裝方向不同（BMI088 晶片 Z 朝下、X/Y 對調；ADXL375 晶片 Z 朝上、X/Y 直接
    // 取負，見 tests/test_sensor_axis.c），跨晶片比對前必須先各自轉 body frame
    // 再換算，見 main.c 呼叫 ADXL375_CalibrateAgainstRef() 前的轉換（此前一版
    // 誤解此註解、直接原生相減校正，Z 軸符號因而算反，已修正）。若日後 ADXL 融入
    // EKF（改善項目 A），於 EKF 饋入處比照 BMI088 套用相同的軸向映射即可。
    data->ax = (float)data->x_raw * 0.049f + s_sw_bias_ax;
    data->ay = (float)data->y_raw * 0.049f + s_sw_bias_ay;
    data->az = (float)data->z_raw * 0.049f + s_sw_bias_az;

    return HAL_OK;
}
