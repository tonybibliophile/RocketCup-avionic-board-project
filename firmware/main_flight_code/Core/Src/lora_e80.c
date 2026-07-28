/**
 ******************************************************************************
 * @file    lora_e80.c
 * @brief   E80-900M2213S 920MHz LoRa 驅動 (Semtech LR1121, SPI3)
 *
 * ★★ 重要：E80-900M2213S 的核心是 Semtech **LR1121**（非 SX126x）。LR1121 的
 *    SPI 命令協定與 SX126x 截然不同：
 *      - opcode 為 16-bit（2 bytes），如 SetRfFrequency = 0x020B
 *      - 頻率直接以 Hz（4 bytes 大端）帶入，不用 SX126x 的 Frf=f·2^25/32e6 公式
 *      - 讀取命令回應走「獨立的第二次 SPI 交易」：先送 opcode，等 BUSY 拉低，
 *        再拉 CS 讀 [Stat1][資料…]
 *      - LoRa 封包型態 = 0x02、LoRa sync word 為「單一位元組」命令 0x022B
 *      - IRQ 遮罩為 32-bit；IRQ 狀態由 GetStatus(0x0100) 回應內含
 *      - PA 為 HP PA；模組內建 **RF 開關接在 LR1121 DIO5/DIO6**，必須以
 *        SetDioAsRfSwitch(0x0112) 設定，否則收發 RF 完全不通（最關鍵的上板項）
 *
 * 接線（連線和基本硬體規格表.md）：SPI3(SCK=PB3/MISO=PB4/MOSI=PB5),
 *   CS=PD7(CSB_LORA920), RST=PD5, IRQ(LR1121 DIO9)→PD4(EXTI4 rising), BUSY=PD6。
 * SPI3 與 W25Q128 Flash 共用 → 經 spi3_bus.h 的 SPI3_Bus_Lock/Unlock 互斥。
 *
 * ★ 下列 #define 為「板級」參數，務必於上板 bring-up 對照 E80-900M2213S 規格書
 *   逐項驗證（見 docs 操作手冊「上板驗證清單」）：
 *     - E80_RFSW_*：RF 開關真值表（DIO5=RFSW0 / DIO6=RFSW1），預設採 Semtech
 *       LR1121 參考設計，E80 接線若不同則收不到封包。
 *     - E80_USE_TCXO：E80 用 32MHz 主動式振盪器；若為自供電（clipped-sine 進 XTA）
 *       則保持 0；若 bring-up 顯示時鐘異常再開。
 *     - IRQ 對應之 LR1121 DIO：預設假設模組 INT 腳接 LR1121 DIO9（IRQ 走 dio1 遮罩）。
 ******************************************************************************
 */
#include "lora_e80.h"
#include "main.h"        /* LORA920_* / CSB_LORA920 腳位巨集 */
#include "spi3_bus.h"    /* SPI3 共用匯流排互斥鎖 */
#include "lora_calc.h"   /* 純換算：頻率位元組 / LDRO（與 host 測試共用） */
#include <stdio.h>
#include <string.h>

/* 看門狗（main.c 定義）：BUSY 卡死時（模組未接/未上電/故障）本檔內部逐一 lr_cmd/
 * e80_wait_busy 逾時雖各自有界（20ms 級），但 TCXO 電壓掃描 4 候選 × reset+多道
 * 指令加總可達 2~3 秒，超過 main.c 開機序列的 IWDG(~2.05s) 視窗且本檔原本全程無人
 * 餵狗 → 看門狗重置，卡在 LoRaE80_Init() 內、永遠到不了 RTOS/GroundStation_Run
 * （症狀與「開機重開機」、USB 完全無輸出一致）。 */
extern IWDG_HandleTypeDef hiwdg;

/* ============================================================
 *  ★ RF 組態（上板 bring-up 須對照 E80-900M2213S 規格書與地面站逐項驗證）
 * ============================================================ */
#define E80_RF_FREQ_HZ        920000000UL /* 載波頻率 (Hz)，回復為 920MHz 以匹配 E80 天線與硬體帶通濾波器 */
#define E80_TX_POWER_DBM      22          /* 發射功率 (dBm)，E80-2213S HP PA 上限 +22 */
/* ★ 10Hz 下行需求：原 SF9/BW250 單包空中時間 307.7ms（116B payload，見 lora_calc.h
 * 的 lora_time_on_air_us，host test 已驗證），理論上限僅 3.25Hz，離 10Hz 差 3 倍。
 * 改 SF8/BW500 → 87.2ms/包，理論上限 11.5Hz，100ms 時槽下僅 ~13ms(13%) 排程餘裕
 * ——已與使用者確認接受此取捨（比 SF7/BW500 少掉 2.5dB 但更省，用戶選定）。
 * ⚠ 換來的代價：靈敏度 −5.5dB（SF9→8 約 −2.5dB + BW250→500 約 −3dB），
 * 自由空間距離粗估縮至現在的 ~53%。若實測搆不到穩定 10Hz 或距離不夠，
 * 這是第一個該調回的地方。 */
#define E80_LORA_SF           0x08U       /* 展頻因子 SF8（原 SF9，換 10Hz 下行速率） */
#define E80_LORA_BW           0x06U       /* 頻寬 500kHz（0x06，原 250kHz） */
#define E80_LORA_CR           0x01U       /* 編碼率 4/5（0x01） */
#define E80_PREAMBLE_LEN      8U          /* 前導碼符號數 */
#define E80_LORA_SYNCWORD     0x12U       /* LoRa sync word（單一位元組）：0x12 私有 / 0x34 公有，須與對端一致 */

/* ★ E80 必須由 LR1121 供電給 TCXO —— 這是「晶片 rdy、SetTx 回 OK，但 TxDone 永遠不來、
 *   從未真正發射成功」的根因。兩份規格書交叉確認：
 *     · E80-xxxM2213S 使用手冊 v1.1 p.註2：「模組內部 LR1121 的 XTA、XTB 和【VTCXO】
 *       引腳已連接 32M【有源溫補晶振】」→ TCXO 的電源來自 LR1121 的 REG_TCXO(VTCXO 腳)，
 *       不是模組 VCC 直供。不下 SetTcxoMode 就等於整顆 32MHz 參考時鐘沒有供電。
 *     · LR1121 datasheet §1.2.4：「The circuit is able to boot when a TCXO is connected
 *       instead of a 32MHz crystal, however all start-up (POR) calibrations are skipped.
 *       The host processor should program the TCXO configuration and re-launch the
 *       calibrations before further usage of the chip.」
 *   —— 正好解釋觀察到的症狀：設定類命令跑在 32MHz HF【RC】振盪器上，所以 GetVersion/
 *   GetStatus/SetStandby/WriteBuffer/SetTx 全部「成功」；但真正的射頻收發需要 HFXOSC，
 *   TCXO 沒供電 → PLL 永遠鎖不上 → TxDone 永遠不觸發 → 每包都走 1.5s 逾時放行(0.6Hz)。
 *
 *   ⚠ 先前把本旗標設 1 曾讓晶片在 SPI 上完全不回應(gs 0x98→0x00)，因而被誤判為
 *   「E80 是自供電振盪器」。真正原因是【電壓選錯】：當時用 0x07=3.3V，但 datasheet
 *   Table 3-13 明訂 REG_TCXO 的條件是「VDDop > VTCXO + 200mV」，而 E80 工作電壓就是
 *   3.3V（手冊 p. 工作電壓 typ 3.3V）→ 3.3V 供不出 3.3V，穩壓器失效、TCXO 起振失敗、
 *   晶片卡死。改用 1.8V 即滿足 3.3 > 1.8+0.2 的條件。 */
#define E80_USE_TCXO          1           /* 1=由 LR1121 REG_TCXO 供電給模組上的 32M TCXO（E80 屬此） */
#define E80_TCXO_VOLTAGE      0x02U       /* RegTcxoTune：0x00=1.6V 0x01=1.7V 0x02=1.8V 0x03=2.2V
                                             0x04=2.4V 0x05=2.7V 0x06=3.0V 0x07=3.3V。
                                             ★須滿足 VDDop(3.3V) > VTCXO + 200mV → 上限 3.0V；
                                             取 1.8V（此類模組常規值，餘裕最大）。 */
#define E80_TCXO_DELAY        0x000140UL  /* TCXO 啟動延遲（×30.52us，0x140=320≈9.8ms） */
#define E80_USE_DCDC          0           /* 0=LDO（保守）；1=DC-DC（須外部電感，E80 多含） */

/* ---- RF 開關組態 ★對齊 Ebyte E80-xxxM2213S 使用手冊 v1.1「第 8 頁億佰特 E80 專用
 *   SDK 程式碼」（smtc_shield_lr11xx_common_rf_switch_cfg，只適用於 E80 系列模組）----
 * ⚠ 手冊第 5 頁的 DIO5/DIO6 真值表與第 8 頁的億佰特自訂 SDK 程式碼「互相矛盾」：
 *   兩者僅 TX-HP 一致（DIO5=1,DIO6=0），RX 與 TX-LP 相反。手冊 p.5 note3 本身即
 *   指明開關狀態「與 SEMTECH 官方 SDK 預設不同，請參考…億佰特自訂 SDK」——即 p.8。
 *   故以 p.8 億佰特 SDK 為權威來源（原本照 p.5 表寫的 RX=0x00 會讓天線未接到 LNA、
 *   地面站完全收不到）。各欄位 = 該模式下 RFSW0(DIO5,bit0)/RFSW1(DIO6,bit1)/RFSW2(DIO7,bit2)
 *   的高電平組合，取自 p.8：
 *     .enable=RFSW0|RFSW1|RFSW2  .standby=0
 *     .rx=RFSW1        .tx=RFSW0|RFSW1   .tx_hp=RFSW0   .tx_hf=0   .gnss=RFSW2  .wifi=0 */
#define E80_RFSW_ENABLE       0x07U       /* RFSW0|RFSW1|RFSW2 皆作 RF 開關 */
#define E80_RFSW_STBY         0x00U       /* 待機：全低 */
#define E80_RFSW_RX           0x02U       /* 接收：RFSW1_HIGH（DIO6=1）★地面站主用（原 0x00 為 p.5 誤表） */
#define E80_RFSW_TX           0x03U       /* TX 低功率 LP：RFSW0|RFSW1（DIO5=1,DIO6=1） */
#define E80_RFSW_TX_HP        0x01U       /* TX 高功率 HP（+22dBm，下行主用）：RFSW0_HIGH（DIO5=1,DIO6=0） */
#define E80_RFSW_TX_HF        0x00U       /* TX 2.4GHz：本專案不用（億佰特 SDK tx_hf=0） */
#define E80_RFSW_GNSS         0x04U       /* RFSW2_HIGH（不影響 sub-G 收發） */
#define E80_RFSW_WIFI         0x00U

/* ============================================================
 *  LR1121 opcode（16-bit）
 * ============================================================ */
/* System */
#define LR_GET_STATUS         0x0100U
#define LR_GET_VERSION        0x0101U
#define LR_GET_ERRORS         0x010DU
#define LR_CLEAR_ERRORS       0x010EU
#define LR_CALIBRATE          0x010FU
#define LR_SET_REGMODE        0x0110U
#define LR_CALIB_IMAGE        0x0111U
#define LR_SET_DIO_RFSW       0x0112U
#define LR_SET_DIO_IRQ        0x0113U
#define LR_CLEAR_IRQ          0x0114U
#define LR_SET_TCXO           0x0117U
#define LR_SET_STANDBY        0x011CU
/* RegMem（FIFO 緩衝） */
#define LR_WRITE_BUFFER       0x0109U
#define LR_READ_BUFFER        0x010AU
/* Radio */
#define LR_GET_RXBUF_STATUS   0x0203U
#define LR_GET_PKT_STATUS     0x0204U
#define LR_SET_TX_CW          0x0208U
#define LR_SET_RX             0x0209U
#define LR_SET_TX             0x020AU
#define LR_SET_RF_FREQ        0x020BU
#define LR_SET_PKT_TYPE       0x020EU
#define LR_SET_MOD_PARAMS     0x020FU
#define LR_SET_PKT_PARAMS     0x0210U
#define LR_SET_TX_PARAMS      0x0211U
#define LR_SET_PA_CFG         0x0215U
#define LR_SET_RX_BOOSTED     0x0227U
#define LR_SET_LORA_SYNCWORD  0x022BU

#define LR_STANDBY_RC         0x00U
#define LR_PKT_TYPE_LORA      0x02U

/* GetErrors(0x010D) 回報的錯誤位元（Semtech LR11xx 驅動慣例；本倉庫的硬體 datasheet
 * PDF 不含命令集）。HF_XOSC_START 是判斷「TCXO 是否真的起振」的關鍵旗標。 */
#define E80_ERR_LF_RC_CALIB     0x0001U
#define E80_ERR_HF_RC_CALIB     0x0002U
#define E80_ERR_ADC_CALIB       0x0004U
#define E80_ERR_PLL_CALIB       0x0008U
#define E80_ERR_IMG_CALIB       0x0010U
#define E80_ERR_HF_XOSC_START   0x0020U
#define E80_ERR_LF_XOSC_START   0x0040U
#define E80_ERR_PLL_LOCK        0x0080U

/* IRQ 位元（32-bit） */
#define LR_IRQ_TX_DONE        0x00000004UL
#define LR_IRQ_RX_DONE        0x00000008UL
#define LR_IRQ_PREAMBLE       0x00000010UL
#define LR_IRQ_HEADER_ERR     0x00000040UL
#define LR_IRQ_CRC_ERR        0x00000080UL
#define LR_IRQ_TIMEOUT        0x00000400UL

#define E80_BUSY_TIMEOUT_MS   20U     /* 等 BUSY 拉低逾時（命令處理一般 <ms） */
#define E80_SPI_TIMEOUT_MS    20U     /* SPI 傳輸逾時 */
#define E80_TX_TIMEOUT_MS     1500U   /* 一筆封包空中傳輸 + TxDone 上限（保守） */

/* ============================================================
 *  狀態
 * ============================================================ */
static SPI_HandleTypeDef *s_hspi = NULL;
static volatile uint8_t   s_tx_done = 0;
static volatile uint8_t   s_rx_event = 0;   /* DIO IRQ 觸發（地面站 RX：RxDone/CrcErr/Timeout） */
static uint8_t            s_tx_in_progress = 0;
static uint32_t           s_tx_start_tick = 0;
static uint8_t            s_inited = 0;
static uint8_t            s_disabled = 0;    /* 1 = 已 Shutdown 隔離：拒絕一切 SPI3 觸碰 */
static uint16_t           s_preamble = E80_PREAMBLE_LEN;  /* 目前前導碼（Reconfig 可改） */
static uint8_t            s_stat1 = 0xFF;    /* 最近一次讀回的 LR1121 Stat1 */

/* 目前套用中的 RF 參數（e80_apply_rf 成功時記錄；供 LoRaE80_GetParams 查詢） */
static uint32_t           s_cur_freq_hz  = E80_RF_FREQ_HZ;
static uint8_t            s_cur_sf       = E80_LORA_SF;
static uint8_t            s_cur_bw       = E80_LORA_BW;
static uint8_t            s_cur_cr       = E80_LORA_CR;
static int8_t             s_cur_pwr_dbm  = E80_TX_POWER_DBM;

/* 初始化診斷：存起來供週期性遙測輸出（開機太早、序列埠來不及接） */
/* GetErrors(0x010D) 回報的 16-bit 錯誤旗標，開機校準後讀一次。
 * ★這是判斷「TCXO/XOSC 是否真的起振、PLL 是否鎖上」的唯一直接證據——設定類命令跑在 HF RC
 * 振盪器上，即使 XOSC 全掛也一樣回 OK，只有這裡看得出來。位元定義（Semtech LR11xx 驅動慣例，
 * 非本倉庫硬體 datasheet PDF 內容，該 PDF 不含命令集）：
 *   bit0 LF_RC_CALIB  bit1 HF_RC_CALIB  bit2 ADC_CALIB  bit3 PLL_CALIB
 *   bit4 IMG_CALIB    bit5 HF_XOSC_START  bit6 LF_XOSC_START  bit7 PLL_LOCK
 * 正常應為 0x0000；若 bit5(HF_XOSC_START) 或 bit7(PLL_LOCK) 亮 → TCXO 供電/電壓仍不對。 */
static uint16_t s_dev_errors     = 0xFFFF;
static uint8_t  s_tcxo_tune_used = 0xFFU;   /* 掃描後實際採用的 RegTcxoTune；0xFF=全部失敗/未啟用 */
static int     s_init_rd_st      = -1;
static uint8_t s_init_busy       = 0xFF;
static uint8_t s_init_ver[2]     = {0, 0};  /* GetVersion: [0]=HW, [1]=Type(0x03=LR1121) */
static uint8_t s_get_status_byte = 0xFF;    /* GetStatus 的 Stat1 */

/* --- CS 控制 --- */
#define E80_CS_LOW()   HAL_GPIO_WritePin(CSB_LORA920_GPIO_Port, CSB_LORA920_Pin, GPIO_PIN_RESET)
#define E80_CS_HIGH()  HAL_GPIO_WritePin(CSB_LORA920_GPIO_Port, CSB_LORA920_Pin, GPIO_PIN_SET)

/* BUSY(PD6) HIGH=忙線；等待拉低，含逾時。不觸 SPI，故不需持鎖。 */
static HAL_StatusTypeDef e80_wait_busy(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LORA920_BUSY_GPIO_Port, LORA920_BUSY_Pin) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/* ============================================================
 *  LR1121 SPI 基本交易
 * ============================================================ */

/* 寫入型命令：CS 低 → [opcode_hi, opcode_lo, params…] → CS 高。整段持 SPI3 鎖。 */
static HAL_StatusTypeDef lr_cmd(uint16_t opcode, const uint8_t *params, uint16_t n)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[2] = { (uint8_t)(opcode >> 8), (uint8_t)(opcode & 0xFF) };
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 2, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) st = HAL_SPI_Transmit(s_hspi, (uint8_t *)params, n, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    if (st == HAL_OK) {
        st = e80_wait_busy(E80_BUSY_TIMEOUT_MS);
    }
    SPI3_Bus_Unlock();
    return st;
}

/* 讀取型命令（LR1121 兩段式）：
 *   交易1：CS 低 → 送 opcode(+params) → CS 高；
 *   等 BUSY 拉低（晶片備妥回應）；
 *   交易2：CS 低 → 第一位元組為 Stat1（存 s_stat1），續讀 rn 位元組資料 → CS 高。 */
static HAL_StatusTypeDef lr_read(uint16_t opcode, const uint8_t *params, uint16_t pn,
                                 uint8_t *rdata, uint16_t rn)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[2] = { (uint8_t)(opcode >> 8), (uint8_t)(opcode & 0xFF) };
    uint8_t stat1 = 0xFF;
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    /* 交易1：送命令 */
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 2, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && pn > 0) st = HAL_SPI_Transmit(s_hspi, (uint8_t *)params, pn, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    if (st != HAL_OK) { SPI3_Bus_Unlock(); return st; }
    /* 等晶片備妥回應 */
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    /* 交易2：讀 Stat1 + 資料 */
    E80_CS_LOW();
    st = HAL_SPI_Receive(s_hspi, &stat1, 1, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && rn > 0) st = HAL_SPI_Receive(s_hspi, rdata, rn, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    s_stat1 = stat1;
    return st;
}

/* 寫 TX FIFO（LR1121 WriteBuffer8：opcode + data，無 offset，從緩衝起點寫整筆）。 */
static HAL_StatusTypeDef lr_write_buffer(const uint8_t *data, uint8_t n)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[2] = { (uint8_t)(LR_WRITE_BUFFER >> 8), (uint8_t)(LR_WRITE_BUFFER & 0xFF) };
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 2, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) st = HAL_SPI_Transmit(s_hspi, (uint8_t *)data, n, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    return st;
}

/* 讀 RX FIFO（LR1121 ReadBuffer8：opcode + offset + len，等 BUSY 後讀 Stat1 + data）。 */
static HAL_StatusTypeDef lr_read_buffer(uint8_t offset, uint8_t *data, uint8_t n)
{
    HAL_StatusTypeDef st;
    uint8_t hdr[4] = { (uint8_t)(LR_READ_BUFFER >> 8), (uint8_t)(LR_READ_BUFFER & 0xFF), offset, n };
    uint8_t stat1 = 0xFF;
    SPI3_Bus_Lock();
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Transmit(s_hspi, hdr, 4, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    if (st != HAL_OK) { SPI3_Bus_Unlock(); return st; }
    if (e80_wait_busy(E80_BUSY_TIMEOUT_MS) != HAL_OK) { SPI3_Bus_Unlock(); return HAL_TIMEOUT; }
    E80_CS_LOW();
    st = HAL_SPI_Receive(s_hspi, &stat1, 1, E80_SPI_TIMEOUT_MS);
    if (st == HAL_OK && n > 0) st = HAL_SPI_Receive(s_hspi, data, n, E80_SPI_TIMEOUT_MS);
    E80_CS_HIGH();
    SPI3_Bus_Unlock();
    s_stat1 = stat1;
    return st;
}

/* ============================================================
 *  共用設定片段
 * ============================================================ */

/* 硬體重置：RST(PD5) 拉低脈衝後等 BUSY 就緒。 */
static void e80_reset(void)
{
    HAL_GPIO_WritePin(LORA920_RST_GPIO_Port, LORA920_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(35);
    HAL_GPIO_WritePin(LORA920_RST_GPIO_Port, LORA920_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    (void)e80_wait_busy(100U);
}

/* 設定 LoRa 封包參數（前導碼 s_preamble、顯式表頭、CRC on、標準 IQ、本次 payload 長度）。 */
static HAL_StatusTypeDef e80_set_packet_params(uint8_t payload_len)
{
    uint8_t pp[6] = {
        (uint8_t)(s_preamble >> 8), (uint8_t)(s_preamble & 0xFF),
        0x00,          /* 顯式表頭（variable length） */
        payload_len,   /* payload 長度 */
        0x01,          /* CRC on */
        0x00           /* 標準 IQ */
    };
    return lr_cmd(LR_SET_PKT_PARAMS, pp, 6);
}

/* 設定 RF 開關（★板級真值表）。LR1121 必須設定，否則收發 RF 不通。 */
static HAL_StatusTypeDef e80_set_rf_switch(void)
{
    uint8_t rfsw[8] = {
        E80_RFSW_ENABLE, E80_RFSW_STBY, E80_RFSW_RX, E80_RFSW_TX,
        E80_RFSW_TX_HP,  E80_RFSW_TX_HF, E80_RFSW_GNSS, E80_RFSW_WIFI
    };
    return lr_cmd(LR_SET_DIO_RFSW, rfsw, 8);
}

/* 影像校準（CalibImage 0x0111）：換到目標頻段後校準影像抑制，確保 RX 靈敏度。
 * 與 TCXO 供電方式解耦——E80 自供電振盪器亦需此步。須於 STANDBY_RC 執行。
 * 頻段位元組由 lora_calc.h 的純函式計算（與 host 測試共用同一份）。 */
static HAL_StatusTypeDef e80_calib_image(uint32_t freq_hz)
{
    uint8_t cb[2];
    lr1121_calib_image_bytes(freq_hz, cb);
    HAL_StatusTypeDef st = lr_cmd(LR_CALIB_IMAGE, cb, 2);
    (void)e80_wait_busy(50U);   /* 校準需時（數 ms），等 BUSY 確實結束 */
    return st;
}

/* 設定 DIO IRQ 遮罩（32-bit ×2：dio1 / dio2）。事件放 dio1（假設模組 INT=LR1121 DIO9）。 */
static HAL_StatusTypeDef e80_set_dio_irq(uint32_t irq)
{
    uint8_t dio[8] = {
        (uint8_t)(irq >> 24), (uint8_t)(irq >> 16), (uint8_t)(irq >> 8), (uint8_t)irq, /* dio1 */
        0x00, 0x00, 0x00, 0x00                                                          /* dio2 */
    };
    return lr_cmd(LR_SET_DIO_IRQ, dio, 8);
}

/* 清除指定的 IRQ 位（LR_CLEAR_IRQ 本來就吃 32-bit 遮罩，寫 1 的位才被清）。
 * 只清「這次實際讀到」的位，可避免把 e80_get_irq() 之後才到達的新 RX_DONE 一併抹掉
 * （全清版 e80_clear_irq() 存在這個小 race，會白白丟一包）。 */
static HAL_StatusTypeDef e80_clear_irq_mask(uint32_t mask)
{
    uint8_t clr[4] = {
        (uint8_t)(mask >> 24), (uint8_t)(mask >> 16), (uint8_t)(mask >> 8), (uint8_t)mask
    };
    return lr_cmd(LR_CLEAR_IRQ, clr, 4);
}

/* 全清版：武裝接收(StartRx)時用，把殘留狀態一次歸零。 */
static HAL_StatusTypeDef e80_clear_irq(void)
{
    return e80_clear_irq_mask(0xFFFFFFFFUL);
}

static HAL_StatusTypeDef e80_set_standby_rc(void)
{
    uint8_t sb = LR_STANDBY_RC;
    return lr_cmd(LR_SET_STANDBY, &sb, 1);
}

/* 套用調變參數（SF/BW/CR + 自動 LDRO）+ 載波頻率 + 發射功率。 */
static HAL_StatusTypeDef e80_apply_rf(uint32_t freq_hz, uint8_t sf, uint8_t bw,
                                      uint8_t cr, int8_t pwr_dbm)
{
    HAL_StatusTypeDef st;

    uint8_t fb[4];
    lr1121_freq_to_bytes(freq_hz, fb);
    st = lr_cmd(LR_SET_RF_FREQ, fb, 4);
    if (st != HAL_OK) return st;

    uint8_t mod[4] = { sf, bw, cr, lora_ldro_required(sf, bw) };
    st = lr_cmd(LR_SET_MOD_PARAMS, mod, 4);
    if (st != HAL_OK) return st;

    uint8_t txp[2] = { (uint8_t)pwr_dbm, 0x04 /* ramp 80us */ };
    st = lr_cmd(LR_SET_TX_PARAMS, txp, 2);
    if (st != HAL_OK) return st;

    /* 三段命令皆成功才記錄為「目前套用中」參數 */
    s_cur_freq_hz = freq_hz;
    s_cur_sf      = sf;
    s_cur_bw      = bw;
    s_cur_cr      = cr;
    s_cur_pwr_dbm = pwr_dbm;
    return HAL_OK;
}

/* 讀 GetStatus 的 32-bit IRQ 狀態（回應 = Stat1 + Stat2 + Irq[31:0]）。 */
static HAL_StatusTypeDef e80_get_irq(uint32_t *irq_out)
{
    uint8_t b[5] = {0};   /* [0]=Stat2, [1..4]=Irq 大端 */
    HAL_StatusTypeDef st = lr_read(LR_GET_STATUS, NULL, 0, b, 5);
    if (st != HAL_OK) return st;
    if (irq_out) {
        *irq_out = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) |
                   ((uint32_t)b[3] << 8)  | (uint32_t)b[4];
    }
    s_get_status_byte = s_stat1;
    return HAL_OK;
}

/* ============================================================
 *  初始化
 * ============================================================ */
HAL_StatusTypeDef LoRaE80_Init(SPI_HandleTypeDef *hspi)
{
    s_hspi           = hspi;
    s_tx_done        = 0;
    s_tx_in_progress = 0;
    s_inited         = 1;
    s_disabled       = 0;
    s_preamble       = E80_PREAMBLE_LEN;

    HAL_IWDG_Refresh(&hiwdg);   /* 進入 Init 前先餵一次，蓋掉呼叫端已耗用的時間 */
    E80_CS_HIGH();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  /* 強制拉高 W25Q128 CS，防止 SPI3 匯流排干擾 */
    e80_reset();
    s_init_busy = HAL_GPIO_ReadPin(LORA920_BUSY_GPIO_Port, LORA920_BUSY_Pin);

    e80_set_standby_rc();

#if E80_USE_DCDC
    uint8_t reg = 0x01; lr_cmd(LR_SET_REGMODE, &reg, 1);   /* DC-DC */
#endif

#if E80_USE_TCXO
    /* === TCXO 供電電壓自動掃描 ==========================================================
     * 模組上那顆 32M TCXO 的實際工作電壓，E80 手冊並未載明，只能實測。與其寫死猜值、
     * 猜錯就整條射頻無聲失效（SetTx 照樣回 OK、只有 TxDone 永不觸發，極難察覺），
     * 這裡逐一試候選電壓，每次用 GetErrors 的 HF_XOSC_START 位元驗證晶振「真的起振」，
     * 取第一個成功者。開機成本：每次失敗約 +100ms，全掃不中最壞 ~0.4s。
     *
     * 候選順序與 datasheet Table 3-13 的硬性條件 VDDop > VTCXO + 200mV 對齊：
     * 板上 VDD=3.3V ⇒ VTCXO 上限 3.1V ⇒ 合法最高檔為 0x06(3.0V)。0x07(3.3V) 供不出來
     * （穩壓器失去落差、晶振起不來，實測會讓晶片整個卡死），故僅列為最後保底、不優先。
     * 3.3V 標稱的 TCXO 一般容許 2.7~3.6V，因此 3.0V 是最合理的首選。 */
    static const uint8_t k_tcxo_tune_cands[] = { 0x06U, 0x05U, 0x02U, 0x07U };  /* 3.0/2.7/1.8/3.3V */
    s_tcxo_tune_used = 0xFFU;
    for (unsigned ti = 0; ti < sizeof(k_tcxo_tune_cands); ti++) {
        uint8_t tune = k_tcxo_tune_cands[ti];
        HAL_IWDG_Refresh(&hiwdg);   /* BUSY 卡死時每候選 reset+多道指令逼近 IWDG 視窗，逐輪餵狗 */

        /* 每次重試都從硬體重置起步：前一輪若晶振沒起來，晶片可能停在半死狀態。 */
        if (ti > 0U) {
            e80_reset();
            e80_set_standby_rc();
#if E80_USE_DCDC
            uint8_t reg_r = 0x01; lr_cmd(LR_SET_REGMODE, &reg_r, 1);
#endif
        }

        uint8_t tcxo[4] = { tune,
                            (uint8_t)((E80_TCXO_DELAY >> 16) & 0xFF),
                            (uint8_t)((E80_TCXO_DELAY >> 8) & 0xFF),
                            (uint8_t)(E80_TCXO_DELAY & 0xFF) };
        lr_cmd(LR_SET_TCXO, tcxo, 4);
        (void)e80_wait_busy(20U);

        /* 先清舊錯誤，否則讀到的可能是上一輪（或 POR）殘留，判斷失準。ClearErrors 無參數。 */
        lr_cmd(LR_CLEAR_ERRORS, NULL, 0);

        uint8_t calib = 0x3F;   /* 重跑 POR 被跳過的各區塊校準（datasheet §1.2.4 要求） */
        lr_cmd(LR_CALIBRATE, &calib, 1);
        (void)e80_wait_busy(200U);

        uint8_t eb[2] = {0xFF, 0xFF};
        if (lr_read(LR_GET_ERRORS, NULL, 0, eb, 2) == HAL_OK) {
            uint16_t errs = (uint16_t)(((uint16_t)eb[0] << 8) | eb[1]);
            if ((errs & E80_ERR_HF_XOSC_START) == 0U) {
                s_tcxo_tune_used = tune;   /* 晶振確實起振，採用此電壓 */
                break;
            }
        }
    }
#endif

    /* ★ RF 開關（板級；不設則收發不通） */
    e80_set_rf_switch();

    /* LoRa 封包型態 */
    uint8_t ptype = LR_PKT_TYPE_LORA;
    lr_cmd(LR_SET_PKT_TYPE, &ptype, 1);

    /* PA 設定（HP PA → +22dBm）。
     * ★ LR1121 datasheet SetPaConfig：HP PA(PaSel=0x01) 的 regPaSupply 必須為
     *   0x01（VBAT）；用 0x00（內部 LDO）餵 HP PA 規格不支援 → 輸出極弱/近乎無，
     *   正是「晶片 rdy 但 SDR 收不到」的典型主因。Semtech +22dBm 參考組合：
     *   PaSel=0x01, regPaSupply=0x01(VBAT), paDutyCycle=0x04, paHpSel=0x07。 */
    uint8_t pa[4] = { 0x01, 0x01, 0x04, 0x07 };
    lr_cmd(LR_SET_PA_CFG, pa, 4);

    /* ★ 影像校準：LR1121 開機預設為 sub-G 低頻，跳到 920MHz 頻段須校準一次，
     *   否則 RX 影像抑制未最佳化、靈敏度打折（rdy 但收得弱）。standby 中執行。 */
    e80_calib_image(E80_RF_FREQ_HZ);

    /* 頻率 / 調變 / 發射功率 */
    e80_apply_rf(E80_RF_FREQ_HZ, E80_LORA_SF, E80_LORA_BW, E80_LORA_CR, E80_TX_POWER_DBM);

    /* 封包參數（payload 先給 1，發送/接收時再依實際長度重設） */
    e80_set_packet_params(1);

    /* LoRa sync word（單一位元組，須與對端一致） */
    uint8_t sw = E80_LORA_SYNCWORD;
    lr_cmd(LR_SET_LORA_SYNCWORD, &sw, 1);

    /* 預設 IRQ：TxDone | Timeout（主航電 TX 用；地面站 StartRx 會改為 RX 事件） */
    e80_set_dio_irq(LR_IRQ_TX_DONE | LR_IRQ_TIMEOUT);

    /* 驗活：GetVersion（Type=0x03 為 LR1121）+ MISO 健康檢查（全 0x00 接地 / 0xFF 浮空） */
    uint8_t ver[4] = {0, 0, 0, 0};   /* [0]=HW, [1]=Type, [2]=FW major, [3]=FW minor */
    s_init_rd_st  = (int)lr_read(LR_GET_VERSION, NULL, 0, ver, 4);
    s_init_ver[0] = ver[0];
    s_init_ver[1] = ver[1];

    /* GetStatus 的 Stat1（順帶清初始 IRQ） */
    uint32_t irq0 = 0;
    e80_get_irq(&irq0);
    e80_clear_irq();

    /* 校準結果自查：TCXO 起振/PLL 鎖定失敗只有這裡看得到（見 s_dev_errors 註解）。 */
    {
        uint8_t eb[2] = {0, 0};
        if (lr_read(LR_GET_ERRORS, NULL, 0, eb, 2) == HAL_OK) {
            s_dev_errors = (uint16_t)(((uint16_t)eb[0] << 8) | eb[1]);
        }
    }

    /* 在線判定（強化）：
     *  - GetVersion 全 0x00 / 全 0xFF → MISO 接地/浮空，晶片沒回應
     *  - GetStatus 的 Stat1（gs）為 0x00 / 0xFF → 同樣是 MISO 沒被晶片驅動
     *  - GetVersion 的 Type 必須為 0x03（LR1121 簽章；0x02=LR1120, 0x01=LR1110）
     * 任一不符即視為未偵測到 → 隔離 SPI3、回 HAL_ERROR（誠實回報，不謊報 rdy）。
     * 真正在線的 LR1121：Type=0x03、Stat1 為有效模式位元，皆會通過。 */
    uint8_t all_zero  = (ver[0] == 0x00 && ver[1] == 0x00 && ver[2] == 0x00 && ver[3] == 0x00);
    uint8_t all_ff    = (ver[0] == 0xFF && ver[1] == 0xFF && ver[2] == 0xFF && ver[3] == 0xFF);
    uint8_t gs_dead   = (s_get_status_byte == 0x00 || s_get_status_byte == 0xFF);
    uint8_t bad_type  = (ver[1] != 0x03);   /* Type 非 LR1121 */
    if (s_init_rd_st != (int)HAL_OK || all_zero || all_ff || gs_dead || bad_type) {
        LoRaE80_Shutdown();
        return HAL_ERROR;
    }

    s_inited   = 1;
    s_disabled = 0;
    return HAL_OK;
}

void LoRaE80_Shutdown(void)
{
    /* CS 釋放 + RST 拉低保持：LR1121 NRST=低時全腳高阻，自 SPI3 共用線斷開，
     * 不再驅動/污染 Flash 的讀寫交易。純 GPIO（不碰 SPI/mutex），可早於 scheduler 呼叫。 */
    E80_CS_HIGH();
    HAL_GPIO_WritePin(LORA920_RST_GPIO_Port, LORA920_RST_Pin, GPIO_PIN_RESET);

    s_inited         = 0;
    s_disabled       = 1;
    s_tx_in_progress = 0;
    s_tx_done        = 0;
}

uint8_t LoRaE80_IsReady(void)
{
    if (s_disabled || !s_inited) return 0U;
    if (s_tx_in_progress) return 0U;
    return (e80_wait_busy(E80_BUSY_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

/* ============================================================
 *  發送（主航電下行）
 * ============================================================ */
HAL_StatusTypeDef LoRaE80_Send(const uint8_t *data, uint8_t len)
{
    if (s_disabled || !s_inited || s_hspi == NULL || data == NULL || len == 0) {
        return HAL_ERROR;
    }

    /* 背壓：上一筆 TX 是否完成？
     * 原本只信任 DIO IRQ 腳(s_tx_done)；若該腳沒觸發（接線/DIO 對應問題），
     * 就只能死等 E80_TX_TIMEOUT_MS 逾時才放行，把吞吐量節流到 1/1.5s
     * （實際單包空中時間僅 ~百 ms 等級）。改為同時用 GetStatus 直接輪詢
     * TxDone/Timeout bit（作法同 LoRaE80_ReadPacket 對 RxDone 的處理）當備援，
     * 不需要中斷腳也能在下個 200ms tick 內偵測到真正完成。 */
    if (s_tx_in_progress) {
        uint8_t  done = s_tx_done;
        uint32_t irq  = 0;
        if (!done && e80_get_irq(&irq) == HAL_OK && (irq & (LR_IRQ_TX_DONE | LR_IRQ_TIMEOUT))) {
            done = 1;
        }
        if (done) {
            s_tx_in_progress = 0;
            e80_clear_irq();
        } else if ((HAL_GetTick() - s_tx_start_tick) > E80_TX_TIMEOUT_MS) {
            s_tx_in_progress = 0;   /* 真的沒收到完成事件時的最後防線 */
        } else {
            return HAL_BUSY;        /* 仍在空中傳輸，本次跳過 */
        }
    }

    s_tx_done = 0;

    e80_set_standby_rc();
    e80_set_rf_switch();   /* 強制切換天線開關至 TX 模式 */
    e80_set_dio_irq(LR_IRQ_TX_DONE | LR_IRQ_TIMEOUT);
    if (e80_set_packet_params(len) != HAL_OK)       return HAL_ERROR;
    e80_clear_irq();
    if (lr_write_buffer(data, len) != HAL_OK)       return HAL_ERROR;

    /* SetTx，timeout=0 → 單次發送，TxDone 後自動回 standby */
    uint8_t tx[3] = { 0x00, 0x00, 0x00 };
    if (lr_cmd(LR_SET_TX, tx, 3) != HAL_OK)         return HAL_ERROR;

    s_tx_in_progress = 1;
    s_tx_start_tick  = HAL_GetTick();
    return HAL_OK;
}

/* ============================================================
 *  接收（地面站）
 * ============================================================ */
HAL_StatusTypeDef LoRaE80_StartRx(void)
{
    if (s_disabled || !s_inited || s_hspi == NULL) return HAL_ERROR;

    e80_set_standby_rc();
    if (e80_set_packet_params(255) != HAL_OK) return HAL_ERROR;   /* RX payload 上限 */

    uint8_t boost = 0x01;
    lr_cmd(LR_SET_RX_BOOSTED, &boost, 1);   /* 提升接收靈敏度 */

    e80_set_dio_irq(LR_IRQ_RX_DONE | LR_IRQ_CRC_ERR | LR_IRQ_HEADER_ERR | LR_IRQ_TIMEOUT);
    e80_clear_irq();
    s_rx_event = 0;

    /* 連續接收：timeout = 0xFFFFFF（RxContinuous） */
    uint8_t rx[3] = { 0xFF, 0xFF, 0xFF };
    return lr_cmd(LR_SET_RX, rx, 3);
}

/* ★DIO1(PD4/EXTI4) 是 rising-edge only（main.c GPIO_MODE_IT_RISING）：只要
 * LoRaE80_ReadPacket() 曾經有一條 return 路徑忘了清 IRQ，DIO1 就會被 LR1121
 * 持續拉在高電平，之後永遠不會再有上升緣、s_rx_event 恆為 0、920 永久收不到
 * 東西（433 完全不受影響，因為那是獨立的 USART3 位元組流）。這正是舊版的病灶：
 * HEADER_ERR 單獨發生（雜訊造成假 preamble，晶片沒拿到有效長度）時不帶
 * RX_DONE，走到 ReadPacket 的 HAL_BUSY 分支卻沒清 IRQ，DIO1 就此卡死。
 * 這裡額外直接查一次腳位電平當第二道防線：即使哪天又漏清、或開機當下線本來
 * 就是高的，只要 DIO1 實際是高，主迴圈就會被叫去呼叫 ReadPacket 把它清掉，
 * 不必依賴「上升緣沒被漏接」這個前提。 */
uint8_t LoRaE80_RxReady(void)
{
    if (s_disabled || !s_inited) return 0U;
    if (s_rx_event) return 1U;
    return (HAL_GPIO_ReadPin(LORA920_INT_GPIO_Port, LORA920_INT_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

HAL_StatusTypeDef LoRaE80_ReadPacket(uint8_t *buf, uint8_t *len, int16_t *rssi_dbm, int16_t *snr_q)
{
    if (s_disabled || !s_inited || s_hspi == NULL || buf == NULL || len == NULL) {
        return HAL_ERROR;
    }
    s_rx_event = 0;

    /* IRQ 狀態（LR1121 含於 GetStatus 回應） */
    uint32_t irq = 0;
    if (e80_get_irq(&irq) != HAL_OK) {
        /* irq 內容不可信（BUSY 逾時/SPI 逾時），但 DIO1 可能仍卡著未知的舊狀態；
         * 盡力全清一次，好過完全不清、放任 DIO1 continue 卡在高電平。 */
        (void)e80_clear_irq();
        return HAL_TIMEOUT;
    }
    if (!(irq & LR_IRQ_RX_DONE)) {
        /* ★關鍵修正：以前這裡直接 return HAL_BUSY、完全不清 IRQ。單獨的
         * HEADER_ERR/CRC_ERR/TIMEOUT（無 RX_DONE）就會讓 DIO1 永久卡在高電平，
         * 見本函式上方註解。只清「這次讀到的位」，不用全清版，避免把
         * get_irq()之後才真正到達的 RX_DONE 一併抹掉、白白丟一包完整封包。 */
        (void)e80_clear_irq_mask(irq);
        return HAL_BUSY;      /* 尚無完整封包 */
    }

    e80_clear_irq();                                   /* 連續 RX 維持 */

    if (irq & (LR_IRQ_CRC_ERR | LR_IRQ_HEADER_ERR)) return HAL_ERROR;   /* 壞包丟棄 */

    /* RX 緩衝狀態：[0]=payload 長度, [1]=起始指標 */
    uint8_t rbs[2] = {0, 0};
    if (lr_read(LR_GET_RXBUF_STATUS, NULL, 0, rbs, 2) != HAL_OK) return HAL_TIMEOUT;
    uint8_t plen = rbs[0];
    uint8_t pptr = rbs[1];
    if (plen == 0) return HAL_ERROR;

    if (lr_read_buffer(pptr, buf, plen) != HAL_OK) return HAL_TIMEOUT;
    *len = plen;

    /* 封包品質：[0]=rssi_pkt, [1]=snr_pkt, [2]=signal_rssi_pkt */
    uint8_t ps[3] = {0, 0, 0};
    if (lr_read(LR_GET_PKT_STATUS, NULL, 0, ps, 3) == HAL_OK) {
        if (rssi_dbm) *rssi_dbm = (int16_t)(-(int)ps[0] / 2);   /* RSSI(dBm) = -rssi_pkt/2 */
        if (snr_q)    *snr_q    = (int16_t)((int8_t)ps[1]);     /* SNR 原始值（dB = ÷4，沿用 gs_log 契約） */
    }
    return HAL_OK;
}

/* ============================================================
 *  動態重配置（地面站通訊測試）
 * ============================================================ */
HAL_StatusTypeDef LoRaE80_Reconfig(uint32_t freq_hz, uint8_t sf, uint8_t bw,
                                    uint8_t cr, int8_t pwr_dbm, uint16_t preamble)
{
    if (s_disabled || !s_inited || s_hspi == NULL) return HAL_ERROR;

    HAL_StatusTypeDef st = e80_set_standby_rc();
    if (st != HAL_OK) return st;

    s_preamble = preamble;

    e80_calib_image(freq_hz);   /* 換頻段須重新影像校準（standby 中） */

    st = e80_apply_rf(freq_hz, sf, bw, cr, pwr_dbm);
    if (st != HAL_OK) return st;

    st = e80_set_packet_params(255);   /* RX 模式上限 */
    if (st != HAL_OK) return st;

    /* 重新進入連續接收（保持地面站 RX 模式） */
    uint8_t boost = 0x01;
    lr_cmd(LR_SET_RX_BOOSTED, &boost, 1);
    e80_set_dio_irq(LR_IRQ_RX_DONE | LR_IRQ_CRC_ERR | LR_IRQ_HEADER_ERR | LR_IRQ_TIMEOUT);
    e80_clear_irq();
    s_rx_event = 0;

    uint8_t rx[3] = { 0xFF, 0xFF, 0xFF };
    return lr_cmd(LR_SET_RX, rx, 3);
}

/* ============================================================
 *  IRQ / 診斷
 * ============================================================ */
void LoRaE80_OnDio1IRQ(void)
{
    s_tx_done  = 1;   /* TX 路徑（主航電）：TxDone */
    s_rx_event = 1;   /* RX 路徑（地面站）：DIO 觸發，實際類型由 GetStatus IRQ 判 */
}

void LoRaE80_GetParams(uint32_t *freq_hz, uint8_t *sf, uint8_t *bw,
                       uint8_t *cr, int8_t *pwr_dbm, uint16_t *preamble)
{
    if (freq_hz)  *freq_hz  = s_cur_freq_hz;
    if (sf)       *sf       = s_cur_sf;
    if (bw)       *bw       = s_cur_bw;
    if (cr)       *cr       = s_cur_cr;
    if (pwr_dbm)  *pwr_dbm  = s_cur_pwr_dbm;
    if (preamble) *preamble = s_preamble;
}

uint16_t LoRaE80_GetErrors(void)
{
    return s_dev_errors;
}

uint8_t LoRaE80_GetTcxoTune(void)
{
    return s_tcxo_tune_used;
}

void LoRaE80_GetInitDiag(int *rd_st, uint8_t *busy, uint8_t *rb0, uint8_t *rb1, uint8_t *gs)
{
    if (rd_st) *rd_st = s_init_rd_st;
    if (busy)  *busy  = s_init_busy;
    if (rb0)   *rb0   = s_init_ver[0];   /* GetVersion HW */
    if (rb1)   *rb1   = s_init_ver[1];   /* GetVersion Type（0x03=LR1121） */
    if (gs)    *gs    = s_get_status_byte;
}

/* DIO IRQ（PD4/EXTI4）上升緣中斷 → 設定事件旗標。覆寫 HAL 弱定義。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LORA920_INT_Pin) {
        LoRaE80_OnDio1IRQ();
    }
}

void LoRaE80_PrintConfig(void)
{
    printf("[LORA 920MHz] E80-2213S   | Freq: %lu.%03lu MHz | Power: +%ddBm | BW: %ukHz | SF: %u | CR: 4/5 | SyncWord: 0x%02X\r\n",
           E80_RF_FREQ_HZ / 1000000UL,
           (E80_RF_FREQ_HZ % 1000000UL) / 1000UL,
           E80_TX_POWER_DBM,
           (unsigned)lora_bw_to_khz(E80_LORA_BW),
           (unsigned)E80_LORA_SF,
           (unsigned)E80_LORA_SYNCWORD);
}


