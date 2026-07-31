/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi088.h"
#include "adxl375.h"
#include "bmp388.h"
#include "w25qxx.h"
#include "crc16.h"         /* S6：SD 原始 IMU 記錄封包 CRC-16/CCITT-FALSE */
#include "imu_raw_log.h"   /* S6：SD 原始 IMU 二進位記錄封包格式（IMU_xxx.BIN） */
#include "rate_monitor.h"  /* 採樣率監測模組，要停用請在 rate_monitor.h 註解 #define RATE_MONITOR_ENABLE */
#include "ekf.h"
#include "gps.h"
#include "mmc5983.h"
#include "sensor_axis.h"   /* 感測器->body 軸向映射（唯一真相來源，見 sensor_axis.h） */
#include "telemetry.h"     /* 共用二進制+CRC16 下行遙測封包 */
#include "sensor_health.h" /* P0-D：感測器失流/卡死/範圍偵測（host 可測純邏輯） */
#include "lora_e22.h"      /* E22-400T30S 433MHz LoRa 透傳 (UART3) */
#include "lora_e80.h"      /* E80-900M2213S 920MHz LoRa (SX126x SPI3) */
#include "spi3_bus.h"      /* SPI3 共用匯流排互斥鎖 (Flash + E80) */
#include "board_config.h"  /* 主/備角色與功能旗標（單一程式碼雙板） */
#include "link_hw.h"       /* 板間鏈路（USART2 全雙工）；FEATURE_LINK 下編入 */
#include "servo_arb.h"     /* 主傘 PD14 共開時序（互斥握手已取消） */
#include "ground_station.h" /* 地面站主流程（IS_GROUND 下編入；其餘角色為空） */
#include "pyro_selftest.h" /* 開傘地面自測（FEATURE_PYRO_SELFTEST 下編入；平時為空） */
#include "gs_lora_test.h"   /* 地面站 LoRa 通訊測試模組（UART2 命令 + 雙鏈路統計） */
#include "uplink_cmd.h"     /* 上行手動開傘（FEATURE_UPLINK_DEPLOY 下編入；其餘為空） */
#include "usb_device.h"     /* USB-CDC 初始化（FEATURE_USB_CDC 下啟用） */
#include "usbd_cdc_if.h"    /* CDC_Transmit_FS（FEATURE_USB_DEBUG_LOG 下 printf 直通 USB 用） */
#include "ack_proto.h"      /* 命令 ACK 幀 + ACK_OK/UNKNOWN/... 狀態碼（遠端指令回覆） */
#include "uplink_text_proto.h" /* UPLINK_TEXT_MAX（遠端文字命令緩衝大小） */
#include "vertical_filter.h" /* 垂直通道 3 狀態 Kalman（FEATURE_VFILTER 下於 FSM_Update 接線） */
/* 版本資訊：版號為 board_config.h 的 FIRMWARE_VERSION（手動語意化版本）。
 * Makefile 建置前自動產生 build_version.h，補上 git 版次(FW_GIT) 與建置時間(FW_BUILD)；
 * 未經 Makefile（如直接用 CubeIDE）建置時該檔不存在 → 退回預設，不影響編譯。 */
#if defined(__has_include)
#  if __has_include("build_version.h")
#    include "build_version.h"
#  endif
#endif
#ifndef FW_GIT
#define FW_GIT   "nogit"
#endif
#ifndef FW_BUILD
#define FW_BUILD (__DATE__ " " __TIME__)
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   /* strtoul/atoi：命令台參數解析 */
#include <ctype.h>    /* tolower：命令台小寫化 */
#include <math.h>
#include "lora_calc.h" /* LoRa 純換算（BW/LDRO/E22 CH）：命令台參數檢查與顯示 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_tx;   /* E22 433 發送：必須 DMA，見 LoRaE22_Send 註解 */
DMA_HandleTypeDef hdma_usart3_rx;   /* E22 433 接收：circular DMA，無重掛空檔 */
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_i2c1_rx; // Dummy handle to prevent linker/compiler errors in generated code

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  /* 4096 bytes。原 2048 已見底：[STACK] 實測 main=220 bytes 歷史最低餘裕（低於本檔
   * 0.2Hz 診斷行自訂的 ≥256 bytes 門檻），而 StartDefaultTask 單一 frame 就佔 992
   * bytes（-O0，csv_buf[256]/ImuRawRecord_t 等區域變數），再疊 printf 浮點路徑
   * （_vfprintf_r→_dtoa_r）與 FatFs f_write 鏈即溢位。FreeRTOSConfig.h 未開
   * configCHECK_FOR_STACK_OVERFLOW，溢位不會被攔截，只會踩壞相鄰 heap → HardFault
   * → stm32f4xx_it.c 的 while(1) → IWDG 重置，症狀與開機重啟無法區分。
   * 成本：FreeRTOS heap（configTOTAL_HEAP_SIZE=96000）多用 2048 bytes，餘裕充足。 */
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
#define ENABLE_DIAGNOSTICS 1  /* 註解此行即可完全關閉診斷任務以節省資源 */

volatile float g_main_task_cpu_usage = 0.0f;
FlightState_t current_fsm_state = STATE_INIT;   /* 由 FSM 包裝層自 g_fsm_ctx 鏡射（P0-A） */
uint32_t flight_start_tick = 0;                 /* 同上；速度歷史 last_vel_z 已收入 FSM_Context_t */
extern void FSM_SetState(FSM_Context_t *ctx, FlightState_t target_state);
uint8_t sd_logging_active = 0;
/* S6：原始 IMU 二進位記錄（IMU_xxx.BIN）狀態。檔案層級（非 StartDefaultTask 區域變數）
 * 是因為 App_ExecuteRecovery() 是獨立函式（雖與 StartDefaultTask 同一 task context
 * 同步呼叫，見其呼叫點皆在 FSM_Update()/主迴圈內，無需鎖），關檔時也要能沖掉緩衝，
 * 比照 sd_logging_active 同樣的檔案層級模式。s_imu_buf 刻意留在一般 SRAM（非 CCM）：
 * SD 磁碟層走 BSP_SD_WriteBlocks_DMA，CCM 不可 DMA 存取。 */
uint8_t  sd_imu_bin_active = 0;
static uint8_t  s_imu_buf[4096] __attribute__((aligned(4)));
static uint16_t s_imu_len = 0;
static uint16_t s_imu_seq = 0;
static uint32_t s_imu_bin_drop_count = 0;
static uint32_t s_ekf_queue_drops_seen = 0;
uint8_t g_flash_logging_active = 1;
uint32_t g_flash_current_flight_id = 0;         /* Flash ring flight/session ID；由最後封包延續 */
static uint8_t g_flash_flight_id_active = 0;    /* 1 = 本次 ARM/飛行已分配 flight_id */
uint32_t g_flash_write_fail_count = 0;          /* Flash 寫入失敗累計（先前完全靜默，現在可觀測） */
volatile uint8_t g_flash_erase_in_progress = 0; /* 1 = 正在執行 flash erase（阻塞式）；此時 ARM 一律拒絕 */
/* ★2026-07-31：1 = 已擦池未達 FLASH_RING_PREERASE_TARGET，需要使用者下 `flash erase`／
 * `flash pool`。開機不再自動擦除（見 StartDefaultTask 開機序列），故此旗標是使用者唯一
 * 的提醒來源：主迴圈 1Hz 橫幅（USB/LoRa 文字）＋下鏈 arm_flags 的 TELEM_ARM_NEED_ERASE
 * （地面站 GUI 顯示）。ARM 本身由 fsm.c 既有的 flash_pool_ready 閘擋下，非靠此旗標。 */
volatile uint8_t g_flash_need_erase = 0;

static void FlashFlight_SetLastId(uint32_t last_id, uint8_t active)
{
  g_flash_current_flight_id = last_id;
  g_flash_flight_id_active = active ? 1U : 0U;
}

static void FlashFlight_EnsureActive(void)
{
  if (g_flash_flight_id_active == 0U) {
    g_flash_current_flight_id++;
    if (g_flash_current_flight_id == 0U) {
      g_flash_current_flight_id = 1U;
    }
    g_flash_flight_id_active = 1U;
    printf("[FLASH] New flight_id=%lu\r\n", (unsigned long)g_flash_current_flight_id);
  }
}

static void FlashFlight_ClearIfPrelaunch(void)
{
  g_flash_flight_id_active = 0U;
}

/* P1：任務堆疊水位觀測用 handle（defaultTask 用 CubeMX 既有的 defaultTaskHandle） */
static osThreadId_t g_ekf_task_handle  = NULL;
static osThreadId_t g_lora_task_handle = NULL;

/* P0-D：感測器健康監測（失流/卡死/範圍）。彙整位 SH_BIT_* 供 FSM/遙測/[HEALTH] 行。 */
/* 硬鐵校正輔助：CMD_MAG_CAL_START/STOP 切換 [MAG] 印出頻率 1Hz<->10Hz。
 * 改用倒數計時（tick 數）取代簡單 0/1 flag：START 設 3000（60s × 50ticks/s），
 * 每 tick 遞減，歸零自動回 1Hz，避免 STOP 命令被 CDC TX 洪水吃掉後永遠卡在高頻。
 * 10Hz（tick%5）而非 50Hz（tick%1）：留出 CDC TX 餘裕給命令 ACK 回覆。 */
volatile uint32_t g_mag_cal_fast_ticks = 0U;
static SensorMon_t g_mon_bmi088;
static SensorMon_t g_mon_adxl375;
static SensorMon_t g_mon_bmp388;
volatile uint8_t g_sensor_fault_bits = 0;
/* 落地時刻 tick（0 = 尚未落地）。SD 全程記錄據此由 100Hz 降為 10Hz，並於逾時後自動關檔 (Item E)。 */
volatile uint32_t g_touchdown_tick = 0;

BMI088_Data_t imu_data;
ADXL375_Data_t highg_data;
BMP388_Data_t baro_data;
/* BMP388 drdy 閘只在真的讀到新樣本時遞增：只有主任務寫、EKF 組裝(同任務)讀，
 * 供其判斷本 10ms frame 是否有新 baro（取代舊有 (i==0||i==2) 硬編位置的假設）。 */
volatile uint32_t g_baro_seq = 0;

uint8_t bmi088_ok = 0;
uint8_t adxl375_ok = 0;
uint8_t bmp388_ok = 0;
uint8_t mag_ok = 0;       /* MMC5983MA 地磁計（I2C1）初始化狀態 */
uint8_t lora433_ok = 0;   /* E22-400T30S 433MHz LoRa (UART3 透傳) 初始化狀態 */
uint8_t lora920_ok = 0;   /* E80-900M2213S 920MHz LoRa (SX126x SPI3) 初始化狀態 */
uint8_t flash_ring_hw_ok = 0;   /* W25Q128 (Flash) 初始化狀態：ARM flash-pool 閘的 fail-open 判斷依據 */
volatile uint8_t g_arm_blocked_flash = 0;   /* 1 = 本週期 ARM 被 flash pool 未達標擋下（見 FSM_Update） */

/* 命令台改 LoRa 參數期間 = 1：遙測任務跳過該槽不發射。保護兩件事：
 * ① E22 設定模式（M1=1）期間透傳資料衝進 UART3 會被模組當成設定命令（可能誤寫暫存器）；
 * ② E80 Reconfig 為多段 SPI 命令序列，不可與 LoRaE80_Send 交錯。 */
volatile uint8_t g_lora_cfg_pause = 0;



/* 最新電池電壓 (mV)，由飛控迴圈讀 ADC 後更新；遙測與 [PWR] 行唯讀此值（避免 ADC 並發） */
volatile uint16_t g_bat_voltage_mv = 0;

/* SPI3 共用匯流排互斥鎖（W25Q128 Flash + E80 920MHz LoRa）。於 RTOS init 建立。 */
osMutexId_t g_spi3_mutex = NULL;
osMutexId_t g_uart_printf_mutex = NULL;

/* 採樣率監測器：在 IDE Watch / Live Expressions 加入《g_sampling_rate》即可一次觀察所有感測器 */
SAMPLING_RATE_DECL();   /* 展開為: SamplingRateAll_t g_sampling_rate */

/* --- EKF 1000 Hz Double Buffers in SRAM --- */
EKF_Buffer_t g_ekf_buffers[2];
volatile uint8_t g_ekf_active_idx = 0;
volatile uint8_t g_ekf_sample_count = 0;
/* EKF queue put 失敗計數：消費端 (EKF_Task) 落後造成 buffer 來不及入列時遞增。
 * 由 [RATE] 行輸出供觀測；若長期 >0 才需考慮擴大 buffer pool + queue 深度 (Item I)。 */
volatile uint32_t g_ekf_queue_drops = 0;

/* --- TIM3 (400 Hz) ADXL375 Ping-Pong 雙緩衝區宣告 ---
 * 移入 CCM：純「ISR 寫滿→主任務讀」的滾動緩衝，讀取皆由 new_batch_ready 旗標
 * （一般 .bss，開機自動歸零）閘控，消費端不會在首次填滿前讀到內容，故 CCM
 * 開機未歸零（startup 僅清 .bss、不清 .ccmram）對此三組緩衝無影響，不需額外
 * 執行期 memset（相對於 EKF 的累加器類 CCM 變數，見 EKF_Init 的顯式歸零）。 */
#define ADXL_BATCH_SIZE 4
typedef struct {
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    float ax;
    float ay;
    float az;
} ADXL_Sample_t;

CCMRAM ADXL_Sample_t g_adxl_batches[2][ADXL_BATCH_SIZE];
volatile uint8_t g_adxl_active_batch = 0;   // 當前寫入的半區 (0 or 1)
volatile uint8_t g_adxl_sample_count = 0;   // 當前半區已累積的樣本數 (0..3)
volatile uint8_t g_adxl_new_batch_ready = 0;// 1 代表有一批 (4筆) 數據集滿
volatile uint8_t g_adxl_ready_batch_idx = 0;// 已集滿可讀取的半區索引

/* --- TIM6 (400 Hz) BMI088 Accel Ping-Pong 雙緩衝區宣告（CCM，理由同上） --- */
#define BMI_ACC_BATCH_SIZE 4
typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    float ax, ay, az;
    uint32_t t_cyc;   // DWT->CYCCNT@讀取完成瞬間；S6 SD 原始 IMU 記錄的 acc_t_off_q 來源
} BMI_Accel_Sample_t;

CCMRAM BMI_Accel_Sample_t g_bmi_acc_batches[2][BMI_ACC_BATCH_SIZE];
volatile uint8_t g_bmi_acc_active_batch = 0;   // 當前寫入的半區 (0 or 1)
volatile uint8_t g_bmi_acc_sample_count = 0;   // 當前半區已累積的樣本數 (0..3)
volatile uint8_t g_bmi_acc_new_batch_ready = 0;// 1 代表有一批 (4筆) 數據集滿
volatile uint8_t g_bmi_acc_ready_batch_idx = 0;// 已集滿可讀取的半區索引

/* --- TIM7 (1000 Hz) BMI088 Gyro Ping-Pong 雙緩衝區宣告（CCM，理由同上） ---
 * BATCH_SIZE 4→10：1000Hz 下每 10ms frame 對應 10 顆陀螺樣本（EKF 逐顆積分姿態，
 * 見 EKF_GYRO_PER_FRAME）。 */
#define BMI_GYRO_BATCH_SIZE 10
typedef struct {
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    float gx, gy, gz;
    uint32_t t_cyc;   // DWT->CYCCNT@讀取完成瞬間；EKF playback dt 與 S6 記錄時戳的真實來源
} BMI_Gyro_Sample_t;

CCMRAM BMI_Gyro_Sample_t g_bmi_gyro_batches[2][BMI_GYRO_BATCH_SIZE];
volatile uint8_t g_bmi_gyro_active_batch = 0;   // 當前寫入的半區 (0 or 1)
volatile uint8_t g_bmi_gyro_sample_count = 0;   // 當前半區已累積的樣本數 (0..9)
volatile uint8_t g_bmi_gyro_new_batch_ready = 0;// 1 代表有一批 (10筆) 數據集滿
volatile uint8_t g_bmi_gyro_ready_batch_idx = 0;// 已集滿可讀取的半區索引
static ServoArb_t g_servo_arb;   /* 主傘 PD14 共開時序（單板/雙板皆用，見 servo_arb.h） */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
static void Servo_HoldLow(void);   /* 主傘 PD14 開機拉低（無訊號），部署時才純 GPIO 拉高 */
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
#if !FEATURE_USB_CDC
static void MX_USB_OTG_FS_PCD_Init(void);
#endif
static void MX_CRC_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
uint16_t ADC_Read_Battery_mv(void);
void LoRaTelemetry_Task(void *argument);   /* 5Hz 下行遙測：E22(433) + E80(920) */
#ifdef ENABLE_DIAGNOSTICS
void StartDiagnosticTask(void *argument);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* 擷取上一次重置原因（診斷「拔掉 UART 就一直重啟」）。RCC CSR 旗標跨重置保留至清除。 */
  uint32_t reset_csr = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
#if !FEATURE_USB_CDC
  MX_USB_OTG_FS_PCD_Init();   /* 飛行版：低階 PCD；地面站改由 USB-CDC 的 usbd_conf 帶起 */
#endif
  MX_CRC_Init();
  MX_RNG_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  /* 主傘 PD14 立即拉低：MX_TIM4_Init 的 MspPostInit 已把 PD14 設為 TIM4 AF，此處覆蓋為
   * GPIO 輸出並拉低，確保開機到部署前腳位「完全無訊號」（部署時 Servo_MainHigh 純 GPIO 拉高）。 */
  Servo_HoldLow();
  setvbuf(stdout, NULL, _IONBF, 0);   /* 關閉 stdout 緩衝，使所有 FreeRTOS 任務的 printf 立即送往 UART，防慢速任務因獨立 reent 結構積壓不印 */
  HAL_Delay(500);   /* 延遲 500ms 讓 USB-TTL 串口驅動在 MCU 重置後有時間穩定，防止 PC 端漏字 */
  /* 開機橫幅：版本 + 角色，重複三次（剛接上序列埠/雜訊環境也至少看得到一次）。 */
  {
      const char *role_str =
          IS_GROUND  ? (GS_LORA_TX_ENABLE ? "GROUND(地面站,TX可發射)" : "GROUND(地面站,RX-only)") :
          IS_BACKUP  ? "BACKUP(備援)"   : "PRIMARY(主航電)";
      for (int v = 0; v < 3; v++) {
          printf("============================================================\r\n");
          printf("[BOOT] RocketCom FW=%s  ROLE=%s  (git=%s build=%s)\r\n",
                 FIRMWARE_VERSION, role_str, FW_GIT, FW_BUILD);
      }
      printf("============================================================\r\n");
  }
  /* 開機印出重置原因：POR/BOR=電源/欠壓(拔 UART 掉電/LoRa 發射突波最常見)；
   * IWDG=看門狗逾時(任務卡住)；SOFT=軟體重置；PIN=外部復位腳。 */
  printf("[RESET] %s%s%s%s%s%s%s(CSR=0x%08lX)\r\n",
         (reset_csr & RCC_CSR_BORRSTF)  ? "BOR(欠壓) "          : "",
         (reset_csr & RCC_CSR_PORRSTF)  ? "POR/PDR(上電/欠壓) " : "",
         (reset_csr & RCC_CSR_IWDGRSTF) ? "IWDG(看門狗) "       : "",
         (reset_csr & RCC_CSR_WWDGRSTF) ? "WWDG "               : "",
         (reset_csr & RCC_CSR_SFTRSTF)  ? "SOFT(軟體) "         : "",
         (reset_csr & RCC_CSR_LPWRRSTF) ? "LPWR "               : "",
         (reset_csr & RCC_CSR_PINRSTF)  ? "PIN(外部腳) "        : "",
         (unsigned long)reset_csr);
  fflush(stdout);
  /* 前面的 HAL_Delay(500)（USB-TTL 穩定）與開機橫幅已吃掉 ~0.6s 的 IWDG 視窗，
   * 進感測器初始化前先清一次；之後各驅動的長迴圈自行餵狗（見 BMP388 T0 迴圈註解）。 */
  HAL_IWDG_Refresh(&hiwdg);
#if FEATURE_PYRO_SELFTEST
  /* === PYRO SELF-TEST（開傘台面自測）===
   * 每次重啟自動跑一次開傘輸出序列後停在原地（不進飛控 FSM）。
   * 飛行前務必在 board_config.h 把 FEATURE_PYRO_SELFTEST 設回 0。 */
  PyroSelfTest_RunOnce();   /* 不返回：停在無窮迴圈持續餵 IWDG */
#endif
#if FEATURE_USB_CDC
  MX_USB_DEVICE_Init();   /* 地面站：USB 虛擬序列埠 (CDC)。飛行版不啟用（不列舉、不佔 CPU）。 */
#endif
#if FEATURE_FLIGHT   /* 主/備：飛控感測器 (IMU/高G/氣壓)；地面站 (ROLE_GROUND) 不啟用以省資源 */
  float bmi_boot_ref_ax = 0.0f, bmi_boot_ref_ay = 0.0f, bmi_boot_ref_az = 1.0f;
  uint8_t bmi_boot_ref_valid = 0U;
  if (BMI088_Init(&hspi2) == HAL_OK) {
      bmi088_ok = 1;
      /* 先同步取樣 40 筆當 ADXL375 校正參考（須在 TIM6 中斷啟動「前」做，否則
       * 1.6kHz ISR 與本處主執行緒對同一顆 SPI2/BMI088 並發存取，無鎖保護）。 */
      {
          BMI088_Accel_t a;
          float sx = 0.0f, sy = 0.0f, sz = 0.0f;
          int n = 0;
          for (int k = 0; k < 40; k++) {
              HAL_IWDG_Refresh(&hiwdg);   /* 開機無人餵狗視窗，見下方 BMP388 T0 迴圈註解 */
              if (BMI088_ReadAccel(&hspi2, &a) == HAL_OK) { sx += a.ax; sy += a.ay; sz += a.az; n++; }
              HAL_Delay(2);
          }
          if (n > 0) {
              bmi_boot_ref_ax = sx / (float)n;
              bmi_boot_ref_ay = sy / (float)n;
              bmi_boot_ref_az = sz / (float)n;
              bmi_boot_ref_valid = 1U;
          }
      }
      HAL_TIM_Base_Start_IT(&htim6);                           // 啟動 TIM6 400 Hz 中斷採樣 (Accel)
      HAL_TIM_Base_Start_IT(&htim7);                           // 啟動 TIM7 1000 Hz 中斷採樣 (Gyro)
  }
  if (ADXL375_Init(&hspi1) == HAL_OK) {
      adxl375_ok = 1;
      ADXL375_DumpDiag(&hspi1);                                // 開機診斷：配置回讀 + raw hex（查偶數量化/無重力異常）
      if (bmi_boot_ref_valid) {
          /* 用 BMI088（乾淨、平放 σ=5mG）當參考，校正 ADXL375 零點偏移（封裝應力，
           * 實測平放 z 僅 20mG 應≈1000mG）。★兩顆晶片原生座標系不同（BMI088 晶片
           * Z 朝下、X/Y 對調；ADXL375 晶片 Z 朝上、X/Y 直接取負——見 sensor_axis.h
           * 與 tests/test_sensor_axis.c 的實體貼裝反推），不可直接拿 BMI 原生值當
           * ADXL 原生目標。由兩者 body-frame 映射消去 body 分量反推：
           *   adxl_target_x = -bmi_y, adxl_target_y = -bmi_x, adxl_target_z = -bmi_z */
          ADXL375_CalibrateAgainstRef(&hspi1, -bmi_boot_ref_ay, -bmi_boot_ref_ax, -bmi_boot_ref_az);
      }
      HAL_TIM_Base_Start_IT(&htim3);                           // 啟動 TIM3 400 Hz 中斷採樣
  }
  if (BMP388_Init(&hspi1, &baro_data) == HAL_OK) {
      bmp388_ok = 1;
      /* 以發射台環境溫度鎖定高度換算參考溫度 T0 (Item H)。
       * 取 16 筆讀數平均濾除單筆雜訊；必須在 EKF 校準（RTOS 啟動後）之前完成，
       * 使 launchpad 參考與飛行讀數共用同一 T0，相對高度於發射台恆為 0 且全程連續。
       * BMP388 與 ADXL375 共用 SPI1，讀取時暫關 TIM3 中斷以避免匯流排競爭（同主迴圈做法）。 */
      float t0_sum = 0.0f;
      int   t0_n   = 0;
      for (int i = 0; i < 16; i++) {
          /* ★必須餵狗：MX_IWDG_Init() 之後到 GPS_Init()（gps.c 內第一次 Refresh）之間，
           * 整段開機序列無人餵狗，而 IWDG 只有 ~2.05s（Reload=4095, /16）。實測帳：
           * HAL_Delay(500) + BMI088_Init 179 + BMI 參考 120 + ADXL375_Init 54 +
           * DumpDiag 564 + CalibrateAgainstRef 120 + BMP388_Init ~50 + 本迴圈 416
           * ≈ 2.0s（HAL_Delay 內部各 +1 tick），已貼齊超時；LSI 偏快的板子（規格
           * 17–47kHz）更是必炸 → 開機無限重啟、USB 反覆列舉且看不到任何 log
           * （_write 在排程器啟動前直接丟棄）。本迴圈自 200Hz ODR 改 50Hz 後由
           * 16×5ms 變 16×25ms，是吃掉最後餘裕的那一筆。 */
          HAL_IWDG_Refresh(&hiwdg);
          __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
          HAL_StatusTypeDef t0_res = BMP388_ReadData(&hspi1, &baro_data);
          __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
          if (t0_res == HAL_OK) {
              t0_sum += baro_data.temperature;
              t0_n++;
          }
          HAL_Delay(25);  /* BMP388 50Hz ODR → 20ms 週期 + 5ms 餘裕，確保取得新樣本 */
      }
      if (t0_n > 0) {
          float t0_avg = t0_sum / (float)t0_n;
          BMP388_SetReferenceTemp(t0_avg);
          printf("[BMP388] 高度換算參考溫度 T0 鎖定為 %d.%02d °C\r\n",
                 (int)t0_avg,
                 (int)(fabsf(t0_avg - (int)t0_avg) * 100.0f));
      }
  }
#else
  bmi088_ok = 0;
  adxl375_ok = 0;
  bmp388_ok = 0;   /* 地面站：不啟用 IMU/高G/氣壓感測器 */
#endif /* FEATURE_FLIGHT */

  /* GPS 驅動啟動 (USART6, NMEA-0183, 循環 DMA + IDLE 接收)。
   * NEO-M9N 實體掛載於 USART6（PC6/PC7）；解析於 task context（主迴圈 GPS_Update()）進行，
   * ISR/DMA 事件回呼僅組句設旗標。 */
#if FEATURE_GPS
  GPS_Init(&huart6);
#endif

  /* MMC5983MA 三軸地磁計啟動 (I2C1, SCL=PB7/SDA=PB8, 位址 0x30)。
   * 初始化含 Product ID 驗證、軟體重置、400Hz 頻寬與一次 SET/RESET 橋偏校準。
   * 失敗（晶片未上線）僅記錄旗標，不阻擋系統啟動。 */
#if FEATURE_MAG
  if (MMC5983_Init() == HAL_OK) {
      mag_ok = 1;
      printf("[MAG] MMC5983MA online. offset[X,Y,Z]=%ld,%ld,%ld\r\n",
             (long)MMC5983_GetData()->offset[0],
             (long)MMC5983_GetData()->offset[1],
             (long)MMC5983_GetData()->offset[2]);
  } else {
      mag_ok = 0;
      printf("[MAG] MMC5983MA NOT detected (I2C1).\r\n");
  }
#else
  mag_ok = 0;   /* 備板：磁力計 (I2C1) 關閉 */
#endif

#if FEATURE_LORA
  /* E22-400T30S 433MHz LoRa 啟動（UART3 透傳模式 M0=M1=0）。
   * 重置 + 等 AUX 拉高；未接模組也不阻擋啟動（發送由 AUX 背壓守護）。
   * 地面站固定啟用（收 433 下行）；主航電由 LORA433_ENABLE 決定是否啟用 433。 */
#if IS_GROUND || LORA433_ENABLE
  lora433_ok = 0;
  for (int retry = 0; retry < 5; retry++) {
      HAL_IWDG_Refresh(&hiwdg);
      if (LoRaE22_Init(&huart3) == HAL_OK) {
          lora433_ok = 1;
          break;
      }
      HAL_Delay(150);
  }
  if (lora433_ok) {
      printf("[LORA433] E22 偵測成功（config 回讀 OK），透傳就緒 (UART3)。\r\n");
      LoRaE22_PrintConfig();
  } else {
      printf("[LORA433] ⚠ E22 未偵測到（config 無回應）—— 確認模組/接線/供電；主航電遙測任務每 10s 重試。\r\n");
  }
#if FEATURE_UPLINK_DEPLOY
  /* 主航電：在下行之外開啟 USART3 上行接收（地面站手動開傘命令）。
   * E22 透傳模式不發送時即在接收；遙測任務每 10 槽空出 1 槽不發 433 留給上行。 */
  UplinkCmd_Init();
#if LORA433_RX_ONLY
  /* 純接收診斷模式：開機就講清楚，避免事後看 log 分不出燒進去的是哪個版本
   * （這個模式下 433 下行遙測會完全消失，看起來很像 433 壞掉）。 */
  printf("[LORA433] ★純接收模式 (LORA433_RX_ONLY=1)：433 完全不發射，整段時間都在收上行。\r\n");
  printf("[LORA433] ⚠ 433 下行遙測會消失（920 不受影響）—— 上行 bring-up 專用，勿帶此設定飛行。\r\n");
#endif
#elif !IS_GROUND && LORA433_TX_ONLY
  /* 純發送實驗模式（board_config.h::LORA433_TX_ONLY=1）：開機就把狀態講清楚，
   * 避免看 log 時搞不清楚燒進去的是哪個版本、以及為何遠端指令沒反應。 */
  printf("[LORA433] ★純發送模式 (LORA433_TX_ONLY=1)：USART3 不掛 RX、不收上行、不發 ACK。\r\n");
  printf("[LORA433] ⚠ 地面站 ARM/DEPLOY/RECOVERY 遠端指令全部失效 —— 台面測試專用，勿帶此設定飛行。\r\n");
#endif
#else
  lora433_ok = 0;
  printf("[LORA433] disabled (LORA433_ENABLE=0) — 433 下行停用，UART3 不發送。\r\n");
#endif

  HAL_IWDG_Refresh(&hiwdg);

  /* E80-900M2213S 920MHz LoRa 啟動（SX126x/LLCC68, SPI3，與 Flash 共用匯流排）。 */
#if IS_GROUND
  lora920_ok = 0;
  for (int retry = 0; retry < 5; retry++) {
      HAL_IWDG_Refresh(&hiwdg);
      if (LoRaE80_Init(&hspi3) == HAL_OK && LoRaE80_StartRx() == HAL_OK) {
          lora920_ok = 1;
          break;
      }
      HAL_Delay(150);
  }
  if (lora920_ok) {
      printf("[LORA920] E80 RX online (SX126x SPI3, continuous RX).\r\n");
  } else {
      LoRaE80_Shutdown();
      printf("[LORA920] E80 RX init failed — held in reset (SPI3 freed for Flash).\r\n");
  }
#elif LORA920_ENABLE
  lora920_ok = 0;
  for (int retry = 0; retry < 5; retry++) {
      HAL_IWDG_Refresh(&hiwdg);
      if (LoRaE80_Init(&hspi3) == HAL_OK) {
          lora920_ok = 1;
          break;
      }
      HAL_Delay(150);
  }
  if (lora920_ok) {
      /* Init 已完成全部 RF 設定（standby_rc → RF 開關 → LoRa 封包型態 → HP PA/VBAT →
       * 920MHz/SF9/BW250/CR4-5/+22dBm → 封包參數 → sync word → TxDone IRQ）。
       * 不再啟動連續載波(CW)；E80 留在 standby，由 LoRaTelemetry_Task 每包脈衝 SetTx 發送下行。 */
      printf("[LORA920] E80 online (LR1121 SPI3) — standby，下行由遙測任務每包脈衝發送。\r\n");
  } else {
      LoRaE80_Shutdown();   /* 偵測失敗（含 MISO 接地/浮空）→ RST 拉低隔離，釋放 SPI3 */
      printf("[LORA920] E80 NOT detected — held in reset to free SPI3 (protect Flash).\r\n");
  }
#else
  lora920_ok = 0;
  LoRaE80_Shutdown();       /* 主動停用：RST 拉低使 E80 不驅動共用 SPI3 MISO */
  printf("[LORA920] disabled (LORA920_ENABLE=0) — held in reset, SPI3 freed for Flash.\r\n");
#endif
#else  /* !FEATURE_LORA（備援航電）：不啟用任何 LoRa；仍按住 E80 RST 保護 SPI3 上的 Flash */
  lora433_ok = 0;
  lora920_ok = 0;
  LoRaE80_Shutdown();       /* RST 拉低使 E80 不驅動共用 SPI3 MISO（飛行資料記錄保護） */
#endif
  /* E22(最壞 900ms) + E80(最壞 560ms) 合計可達 ~1.5s；補餵一次防 IWDG 2.05s 到期。
   * Flash_Test() 含 sector erase(400ms) + write(500ms) 再補一次。 */
  HAL_IWDG_Refresh(&hiwdg);

#if FEATURE_FLASH
  /* W25Qxx SPI Flash 啟動自檢 (SPI3, CS=PA15) */
  Flash_Test();
  HAL_IWDG_Refresh(&hiwdg);
#endif

  /* Buzzer 開機提示音：依角色不同節奏，耳朵即可確認「燒對角色」（呼應 board_config 角色易混）。
   * TIM2 時基 1MHz → 頻率 = 1e6/(ARR+1)。所有角色都會響（TIM2 與此段皆無條件執行）。 */
#if FEATURE_BUZZER
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
#if IS_GROUND
  for (int b = 0; b < 3; b++) {                 /* 地面站：三短聲 2kHz */
      htim2.Instance->ARR = 499; htim2.Instance->CCR1 = 250; htim2.Instance->EGR = TIM_EGR_UG;
      HAL_Delay(90);
      htim2.Instance->CCR1 = 0;
      HAL_Delay(90);
  }
#elif IS_BACKUP
  htim2.Instance->ARR = 499; htim2.Instance->CCR1 = 250; htim2.Instance->EGR = TIM_EGR_UG;  /* 備援：一長聲 2kHz */
  HAL_Delay(500);
  htim2.Instance->CCR1 = 0;
#else                                            /* 主航電：兩聲漸高 2kHz→4kHz */
  htim2.Instance->ARR = 499; htim2.Instance->CCR1 = 250; htim2.Instance->EGR = TIM_EGR_UG;
  HAL_Delay(100);
  htim2.Instance->CCR1 = 0;
  HAL_Delay(100);
  htim2.Instance->ARR = 249; htim2.Instance->CCR1 = 125; htim2.Instance->EGR = TIM_EGR_UG;
  HAL_Delay(100);
  htim2.Instance->CCR1 = 0;
#endif
  HAL_IWDG_Refresh(&hiwdg);   /* 提示音占用 ~0.4–0.5s，補餵一次防 IWDG */
#endif

#if FEATURE_LINK
  /* 板間鏈路（USART2 全雙工）：重設 baud 為 LINK_BAUD 並啟動循環 DMA/IDLE 接收。
   * 主備兩板程式相同，靠板間排線把 TX/RX 交叉對接。啟用後 USART2 不再作 printf 除錯橋。 */
  Link_Init();
#endif
  /* 主傘 PD14 共開時序（主/副對稱，無優先序；見 servo_arb.h）。 */
  ServoArb_Init(&g_servo_arb);
  /* === SYSTEM INITIALIZATION DIAGNOSTICS SUMMARY === */
  printf("\r\n============================================================\r\n");
  printf("[SYS_DIAG] 開機自檢診斷報告 (Boot Diagnostics Summary):\r\n");
  
  if (IS_GROUND) {
      printf("  - 運作模式: 地面站接收端 (GROUND STATION)\r\n");
      printf("  - E22 433MHz (LoRa):    %s\r\n", lora433_ok ? "OK" : "FAIL (未檢測到/已停用)");
      printf("  - E80 920MHz (LoRa):    %s\r\n", lora920_ok ? "OK" : "FAIL (未檢測到/已停用)");
  } else {
      const char *mode_str = IS_PRIMARY ? "主航電 (PRIMARY)" : "備援航電 (BACKUP)";
      printf("  - 運作模式: %s\r\n", mode_str);
      printf("  - BMI088 (低G感測器):   %s\r\n", bmi088_ok ? "OK" : "FAIL (未檢測到/損壞)");
      printf("  - ADXL375 (高G感測器):  %s\r\n", adxl375_ok ? "OK" : "FAIL (未檢測到/損壞)");
      printf("  - BMP388 (氣壓計):      %s\r\n", bmp388_ok ? "OK" : "FAIL (未檢測到/損壞)");
      printf("  - MMC5983MA (磁力計):   %s\r\n", mag_ok ? "OK" : "FAIL (未檢測到/已停用)");
      printf("  - E22 433MHz (LoRa):    %s\r\n", lora433_ok ? "OK" : "FAIL (未檢測到/已停用)");
      printf("  - E80 920MHz (LoRa):    %s\r\n", lora920_ok ? "OK" : "FAIL (未檢測到/已停用)");
#if FEATURE_FLASH
      W25QXX_InfoTypeDef flash_info;
      uint8_t flash_ok = (W25QXX_Init(&flash_info) == W25QXX_OK) ? 1U : 0U;
      flash_ring_hw_ok = flash_ok;
      printf("  - W25Q128 (Flash):      %s\r\n", flash_ok ? "OK" : "FAIL (未檢測到/已停用)");
#else
      printf("  - W25Q128 (Flash):      DISABLED (未啟用)\r\n");
#endif
  }
  printf("============================================================\r\n\r\n");
  fflush(stdout);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* SPI3 匯流排互斥鎖：W25Q128 Flash 與 E80 920MHz LoRa 共用 SPI3，須互斥存取。
   * 遞迴 + 優先級繼承：容忍 flash 驅動 CS 巨集的防禦性釋放，並避免高優先飛控任務
   * 等待 mutex 時被低優先 LoRa 任務造成優先級反轉。 */
  static const osMutexAttr_t spi3_mutex_attr = {
    .name = "spi3Bus",
    .attr_bits = osMutexRecursive | osMutexPrioInherit,
  };
  g_spi3_mutex = osMutexNew(&spi3_mutex_attr);

  /* UART printf 互斥鎖：防止多任務並發 printf 列印衝突導致序列埠輸出交錯或毀損。 */
  static const osMutexAttr_t uart_printf_mutex_attr = {
    .name = "uartPrintf",
    .attr_bits = osMutexRecursive | osMutexPrioInherit,
  };
  g_uart_printf_mutex = osMutexNew(&uart_printf_mutex_attr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
#if FEATURE_FLIGHT
  xEKFQueue = osMessageQueueNew(2, sizeof(EKF_Buffer_t*), NULL);
#endif
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#if FEATURE_FLIGHT
  g_ekf_task_handle = osThreadNew(EKF_Task, NULL, &EKF_Task_attributes);
#endif
#if defined(ENABLE_DIAGNOSTICS) && !IS_GROUND
  static const osThreadAttr_t diagnosticTask_attributes = {
    .name = "diagTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };
  osThreadNew(StartDiagnosticTask, NULL, &diagnosticTask_attributes);
#endif
#if FEATURE_LORA_TX
  /* LoRa 下行遙測任務（低優先，不影響 1kHz 飛控迴圈時序）。地面站 (FEATURE_LORA_TX=0) 只收不發。
   * 即使兩個 LoRa 模組都未初始化也建立——任務內自會跳過未就緒的鏈路。
   * 備援航電（FEATURE_LORA=0）不建立此任務：無 LoRa 下行，省去 E22 週期重試。 */
  static const osThreadAttr_t loraTelemTask_attributes = {
    .name = "loraTxTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };
  g_lora_task_handle = osThreadNew(LoRaTelemetry_Task, NULL, &loraTelemTask_attributes);
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;                   // 16 分頻 -> 32kHz / 16 = 2000Hz (每秒計數 2000 下)
  hiwdg.Init.Reload = 4095;                                   // 4095 載入值 -> 4095 / 2000 ≈ 2.05 秒超時 (飛行中快速復位)
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  /* 2.625MHz：ADXL375 SPI 上限 5MHz（datasheet
                                              p.10-11），原 ÷16=5.25MHz 超規，獨立修正、與 raw 恆偶數無關
                                              （該症狀是 3200/1600Hz ODR 官方規格少 1-bit，見 adxl375.c 註解）。 */
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5;                                   // 5 + 1 = 6 分頻 -> 84MHz / 6 = 14MHz
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 34999;                                  // 34999 + 1 = 35000 週期 -> 14MHz / 35000 = 400 Hz
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{
  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 41;                                  // 41 + 1 = 42 分頻 -> 84MHz / 42 = 2.0MHz
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1999;                                   // 1999 + 1 = 2000 週期 -> 2.0MHz / 2000 = 1000 Hz
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 41;                                  // 41 + 1 = 42 分頻 -> 84MHz / 42 = 2.0MHz
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;                                   // 4999 + 1 = 5000 週期 -> 2.0MHz / 5000 = 400 Hz
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
#if !FEATURE_USB_CDC   /* 地面站不需低階 PCD：USB-CDC 由 usbd_conf 的 USBD_LL_Init 帶起 */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}
#endif /* !FEATURE_USB_CDC */

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_SYS_Pin|GPIO_PIN_3|LED_STAT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSB_BARO_Pin|RST_GPS_Pin|FLASH_CSB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSB_HIGHG_GPIO_Port, CSB_HIGHG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSB_GYRO_Pin|CSB_ACCEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LORA433_M1_Pin|LORA433_M0_Pin|FIRE_Pin|LORA920_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LORA433_RST_Pin|CSB_LORA920_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_SYS_Pin PE3 LED_STAT2_Pin */
  GPIO_InitStruct.Pin = LED_SYS_Pin|GPIO_PIN_3|LED_STAT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BT2_Pin */
  GPIO_InitStruct.Pin = USER_BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BT1_Pin */
  GPIO_InitStruct.Pin = USER_BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSB_BARO_Pin */
  GPIO_InitStruct.Pin = CSB_BARO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSB_BARO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSB_HIGHG_Pin */
  GPIO_InitStruct.Pin = CSB_HIGHG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSB_HIGHG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_HIGHG2_Pin */
  GPIO_InitStruct.Pin = INT_HIGHG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(INT_HIGHG2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_HIGHG1_Pin INT_BARO_Pin */
  GPIO_InitStruct.Pin = INT_HIGHG1_Pin|INT_BARO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_ACCEL1_Pin INT_ACCEL2_Pin INT_GYRO1_Pin INT_GYRO2_Pin
                           INT_MAGN_Pin */
  GPIO_InitStruct.Pin = INT_ACCEL1_Pin|INT_ACCEL2_Pin|INT_GYRO1_Pin|INT_GYRO2_Pin
                          |INT_MAGN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA433_BUSY_Pin */
  GPIO_InitStruct.Pin = LORA433_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LORA433_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSB_GYRO_Pin */
  GPIO_InitStruct.Pin = CSB_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CSB_GYRO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CSB_ACCEL_Pin */
  GPIO_InitStruct.Pin = CSB_ACCEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CSB_ACCEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA433_M1_Pin LORA433_M0_Pin */
  GPIO_InitStruct.Pin = LORA433_M1_Pin|LORA433_M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA433_RST_Pin */
  GPIO_InitStruct.Pin = LORA433_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA433_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FIRE_Pin LORA920_RST_Pin CSB_LORA920_Pin */
  GPIO_InitStruct.Pin = FIRE_Pin|LORA920_RST_Pin|CSB_LORA920_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_GPS_Pin */
  GPIO_InitStruct.Pin = RST_GPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CSB_Pin */
  GPIO_InitStruct.Pin = FLASH_CSB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FLASH_CSB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_DET_Pin */
  GPIO_InitStruct.Pin = SDIO_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDIO_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA920_INT_Pin */
  GPIO_InitStruct.Pin = LORA920_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LORA920_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA920_BUSY_Pin */
  GPIO_InitStruct.Pin = LORA920_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA920_BUSY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* P1：LORA920_BUSY (PD6) 由 CubeMX 的 IT_RISING_FALLING 覆寫為純輸入。
   * lora_e80.c 只輪詢 BUSY、從不用中斷；E80 SPI 交易期間 BUSY 高頻 toggle
   * 會狂觸 EXTI9_5 ISR（無動作純開銷，與 BMI/ADXL 中斷搶 CPU）。
   * DeInit 清除 EXTI line 6 映射（PD6 為該 line 唯一使用者）再重設為輸入；
   * 放在 USER CODE 區塊，CubeMX 重新產生程式碼後仍保留本覆寫。 */
  HAL_GPIO_DeInit(LORA920_BUSY_GPIO_Port, LORA920_BUSY_Pin);
  GPIO_InitStruct.Pin  = LORA920_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA920_BUSY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
#if FEATURE_USB_DEBUG_LOG
  /* 台面除錯開關（board_config.h）：printf 改走 USB-CDC 虛擬序列埠，免接 SWD 除錯器、
   * 免佔用被板間鏈路用掉的 USART2。
   * 註：把 FEATURE_USB_DEBUG_LOG 改回 0 的理由是「關掉 USB 指令通道」，不是省 CPU——
   * CDC 傳輸本身約 0.01% CPU（SOF 中斷已關、飛行拔線後為 0），而底下這段 printf 的
   * 字串格式化成本走 ITM 分支也一樣付。詳見 board_config.h 該旗標的說明。
   *
   * 串行化：與 UART 分支共用 g_uart_printf_mutex。缺此鎖時多任務會同時呼叫 CDC_Transmit_FS
   * 覆寫同一顆 TX 緩衝，輸出交錯、字元層級毀損。
   * 非同步安全：CDC 傳輸經 IN 端點中斷背景完成，傳輸期間讀取的是我們給的緩衝，故
   *   (1) 先等前一筆傳完（CDC_TxBusy 清）才可覆寫共用靜態緩衝；
   *   (2) 複製 ptr 到靜態緩衝再送，避免 newlib 於傳輸中覆寫呼叫端 ptr。
   * best-effort：PC 未讀取（TxState 卡住）時逾 5ms 放棄該段，不阻塞飛控路徑。
   * RTOS 未啟動前（USB 尚未列舉、osDelay 不可用）直接略過。 */
  /* 行組裝：stdout 無緩衝 → 一次 printf 拆成多次 _write 片段。若逐片段送 USB，USB-FS 每
   * ~1ms 只能推一個 64B 封包，片段撞 TxBusy 被丟 → 半行掉字。改為在此累積到換行才整行送出：
   *   送 → 送整行；丟 → 丟整行（PC 未讀時），輸出乾淨可讀。
   * 完全不阻塞（絕不 osDelay/忙等）：本函式亦由「餵看門狗」的 StartDefaultTask 呼叫，任何忙等
   * 都會在 PC 未讀（TxState 卡住）時逐行累積延遲 → 撐爆 IWDG(2.05s) → MCU 重置（先前症狀）。
   * s_usb_line 累積中的行、s_usb_tx 送出用（CDC 非同步傳輸期間讀取的是它，須與累積緩衝分開，
   * 且僅在 !TxBusy＝前一筆已送完時覆寫）。g_uart_printf_mutex 串行化多任務對這組共用緩衝的存取。
   *
   * ★7/28：原本 256 bytes 對地面站的 [GS_PKT] 診斷行（實際約 400+ bytes）太小——超過就在沒有
   * \n 的情況下於此強制切斷（送出前半段或若 CDC 忙碌直接整段丟棄），後半段接著下一行內容一起
   * 印出，肉眼看是「斷在奇怪位置、接上下一行」。實測 ground_station/logs 資料夾下的既有紀錄
   * 檔驗證多個檔案有 10%~70%+ 的 [GS_PKT] 行因此缺尾巴（在 pvfh:.. pvf 處硬斷）。加大到 512
   * bytes 涵蓋現有最長診斷行，非本次姿態四元數功能專屬——所有角色的長診斷行都受益。 */
  #define USB_LOG_LINE_MAX 512U
  static uint8_t  s_usb_line[USB_LOG_LINE_MAX];
  static uint16_t s_usb_line_len = 0;
  static uint8_t  s_usb_tx[USB_LOG_LINE_MAX];
  if (g_uart_printf_mutex != NULL &&
      osKernelGetState() == osKernelRunning &&
      __get_IPSR() == 0U) {
    osMutexAcquire(g_uart_printf_mutex, osWaitForever);
    for (int i = 0; i < len; i++) {
      uint8_t c = (uint8_t)ptr[i];
      if (s_usb_line_len < USB_LOG_LINE_MAX) {
        s_usb_line[s_usb_line_len++] = c;
      }
      if (c == (uint8_t)'\n' || s_usb_line_len >= USB_LOG_LINE_MAX) {
        if (!CDC_TxBusy()) {                       /* 前一筆已送完才送（否則丟整行，不等待） */
          memcpy(s_usb_tx, s_usb_line, s_usb_line_len);
          CDC_Transmit_FS(s_usb_tx, s_usb_line_len);
        }
        s_usb_line_len = 0;                        /* 送出或丟棄後都清空，開始下一行 */
      }
    }
    osMutexRelease(g_uart_printf_mutex);
  }
#elif FEATURE_LINK
  /* USART2 已作板間鏈路；printf 改走 SWO/ITM（需 SWD/SWV 觀看；無除錯器時 ITM_SendChar
   * 自動略過、不阻塞）。如需回復 USART2 文字除錯橋，建置時設 -DFEATURE_LINK=0。 */
  for (int i = 0; i < len; i++) {
    ITM_SendChar((uint32_t)(uint8_t)ptr[i]);
  }
#else
  uint8_t use_mutex = (g_uart_printf_mutex != NULL &&
                       osKernelGetState() == osKernelRunning &&
                       __get_IPSR() == 0U);
  if (use_mutex) {
    osMutexAcquire(g_uart_printf_mutex, osWaitForever);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  if (use_mutex) {
    osMutexRelease(g_uart_printf_mutex);
  }
#endif
  return len;
}

/* === 主傘 PD14：開傘前硬體拉低、無任何訊號；部署時純 GPIO 拉高 SERVO_MAIN_HIGH_MS ===
 * ★ 已取消 PWM 舵機驅動（TIM4_CH3 不再 start，也不再切 AF）：主傘機構改為吃準位訊號，
 *   部署 = PD14 拉高 1.5s 後回低（見 board_config.h SERVO_MAIN_HIGH_MS / servo_arb.h）。
 * 安全需求：MspPostInit 開機把 PD14 設為 TIM4 AF，腳位狀態不保證為低，故開機即以
 * Servo_HoldLow() 覆蓋為 GPIO 輸出並拉低。 */
static void Servo_HoldLow(void)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = PWM_Servo_Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PWM_Servo_GPIO_Port, &gi);
    HAL_GPIO_WritePin(PWM_Servo_GPIO_Port, PWM_Servo_Pin, GPIO_PIN_RESET);   /* 硬體低、無訊號 */
}

/* 主傘部署：PD14 純 GPIO 輸出高（不切 AF、不啟 PWM）。與 Servo_HoldLow 對稱，只是準位相反。
 * 兩板可同時拉高（diode-OR 準位訊號，無 PWM 合併脈衝問題），故不需互斥握手。 */
static void Servo_MainHigh(void)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = PWM_Servo_Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PWM_Servo_GPIO_Port, &gi);
    HAL_GPIO_WritePin(PWM_Servo_GPIO_Port, PWM_Servo_Pin, GPIO_PIN_SET);   /* 純 GPIO 拉高（無 PWM） */
}

void App_ExecuteRecovery(void)
{
    printf("[RECOVERY] 執行尋回指令：停止蜂鳴器並安全關閉記錄...\r\n");
#if FEATURE_BUZZER
    htim2.Instance->CCR1 = 0;
#endif
    if (sd_imu_bin_active) {
        extern FIL SDImuFile;
        if (s_imu_len > 0U) {
            UINT imu_bw;
            f_write(&SDImuFile, s_imu_buf, s_imu_len, &imu_bw);
            s_imu_len = 0U;
        }
        f_close(&SDImuFile);
        sd_imu_bin_active = 0;
        printf("[SD] 尋回指令：原始 IMU 記錄檔已沖刷並安全關閉。\r\n");
    }
    if (sd_logging_active) {
        extern FIL SDFile;
        extern char SDPath[4];
        f_close(&SDFile);
        f_mount(NULL, SDPath, 1);
        sd_logging_active = 0;
        printf("[SD] 尋回指令：SD 卡檔案已安全關閉並停止記錄。\r\n");
    }
    if (g_flash_logging_active) {
        g_flash_logging_active = 0;
        printf("[FLASH] 尋回指令：Flash 數據記錄已停止。\r\n");
    }
}

/* === FSM 包裝層（P0-A）：純邏輯已抽離至 fsm.c/h，由 tests/test_fsm.c 驗證 ===
 * 此處職責：組輸入快照 → FSM_Step() → 執行硬體動作 → 鏡射全域 → 事件列印。
 * 硬體動作（PD13 點火/PD14 主傘/蜂鳴器）嚴格先於 printf：避免 UART 阻塞（~9ms/行）延遲開傘。 */
FSM_Context_t g_fsm_ctx;
volatile uint8_t g_fsm_failsafe_fired = 0;   /* 失效保護點火鎖存（telemetry TELEM_FLAG_FAILSAFE 讀取） */
/* 主傘 PD14 共開狀態（見 servo_arb.h；已無互斥握手，兩板同時拉高 1.5s）。 */
static uint8_t         g_servo_arb_bcast    = SERVO_ARB_MSG_NONE; /* 供 Link_BuildOwnStatus 廣播 */
static uint8_t         g_servo_want_latched = 0U;                 /* want 一旦成立即鎖存（開傘不撤回） */
static uint8_t         g_servo_high         = 0U;                 /* 邊緣偵測：目前 PD14 是否拉高中 */
volatile uint8_t g_main_deployed = 0U;       /* 主傘已部署鎖存（PD14 曾拉高）：PD14 只高 1.5s，
                                              * 不能像舊 PWM 那樣由腳位/CCR 現況推導，否則
                                              * 下鏈 TELEM_FLAG_MAIN_DEPLOYED 只會亮 1.5s，
                                              * 2Hz 下鏈幾乎必漏。telemetry.c / Link_BuildOwnStatus 讀取。 */
volatile uint8_t g_hotstart_restored  = 0;   /* P0-F：熱啟動恢復鎖存（telemetry TELEM_FLAG_HOTSTART） */
volatile float g_vf_h_m  = 0.0f;   /* 垂直濾波器 (VF) 高度 (m)：供 telemetry.c 下鏈 + Link_BuildOwnStatus 中繼給對端 */
volatile float g_vf_v_ms = 0.0f;   /* 垂直濾波器 (VF) 垂直速度 (m/s) */

/* === 飛行滾動極值（供下鏈 max_alt_m / max_vel_ms / max_acc_cg，見 telemetry.h 說明）===
 * 存在理由：433 下鏈實測只有約 2Hz，而峰值 G 不到 1 秒、頂點是一個瞬間——2Hz 取樣抓不到。
 * 這三個值由飛控迴圈以感測器全速率更新，每一包都重複下鏈，因此抗丟包：頂點之後任何
 * 一包穿透就拿得到真峰值，火箭無法回收時這是唯一的來源。
 * 高度/速度於 FSM_Update(100Hz) 更新（取 FSM 決策實際採用的 h_est/v_est，與開傘判斷同源）；
 * 加速度於 BMI088 批次就緒時「逐筆掃描整批」更新（不是只看批次最後一筆，否則 400Hz 取樣
 * 會被抽成 100Hz、峰值一樣漏掉）。ARM 時歸零，見 ExtremaTrack_Reset()。 */
volatile float g_max_alt_m  = 0.0f;   /* 最大相對高度 (m) */
volatile float g_max_vel_ms = 0.0f;   /* 最大垂直速度 (m/s) */
volatile float g_max_acc_g  = 0.0f;   /* 最大合加速度 |a| (g)，BMI088（±24g 會削頂） */

/* float 公尺 → int16 飽和轉換，供開傘高度鎖存用。
 * ★下限刻意夾在 -32767 而非 -32768：-32768 是 TELEM_DEPLOY_ALT_NA「未開傘」哨兵，
 * 若某個離譜的負高度剛好飽和成 -32768，就會被下游誤讀成「傘沒開」——把真正該警示的
 * 事件藏起來，正好是相反的效果。 */
static int16_t sat_alt_i16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32767.0f) return -32767;
    return (int16_t)v;
}

/* 實際開傘高度（m，相對發射台）。於開傘動作發生的那個飛控週期就地鎖存一次，之後不再改；
 * 未開傘維持 TELEM_DEPLOY_ALT_NA 哨兵（0 是合法開傘高度，不能拿來當「沒開」）。
 * 與上面三個極值同樣跨熱重啟保存（見 FlashRingPacket_t 同名欄位）。 */
volatile int16_t g_drogue_alt_m = TELEM_DEPLOY_ALT_NA;
volatile int16_t g_main_alt_m   = TELEM_DEPLOY_ALT_NA;

/* ARM 時呼叫：把上一輪地面測試/搬動累積的極值與開傘高度清掉，讓下鏈數字只涵蓋本次飛行。
 * ★熱重啟時「絕不」呼叫本函式——那條路徑要的是還原，不是歸零（見 ExtremaTrack_Restore）。 */
void ExtremaTrack_Reset(void)
{
    g_max_alt_m    = 0.0f;
    g_max_vel_ms   = 0.0f;
    g_max_acc_g    = 0.0f;
    g_drogue_alt_m = TELEM_DEPLOY_ALT_NA;
    g_main_alt_m   = TELEM_DEPLOY_ALT_NA;
}

/* 空中熱重啟還原：由 Flash ring 最後一筆封包把飛行摘要接回來。
 * 沒有這支，飛行中一次 brownout/IWDG 重置就會讓最大高度/速度/G 與開傘高度全部歸零，
 * 而這些正是火箭無法回收時唯一的資料來源（下鏈 ~2.3Hz 事後反推不出峰值）。
 * 只在 FSM_HotStartDecide 判定 restore 成立時呼叫（該判定已含封包有效性 + CRC + 高度
 * 連續性檢查，故此處不再重複驗證）。 */
void ExtremaTrack_Restore(uint16_t max_alt_m, int16_t max_vel_ms, uint16_t max_acc_cg,
                          int16_t drogue_alt_m, int16_t main_alt_m)
{
    g_max_alt_m    = (float)max_alt_m;
    g_max_vel_ms   = (float)max_vel_ms;
    g_max_acc_g    = (float)max_acc_cg / 100.0f;
    g_drogue_alt_m = drogue_alt_m;
    g_main_alt_m   = main_alt_m;
}

static void FSM_Update(void)
{
    EKF_State_t ekf_state = EKF_GetState();
    uint32_t now = HAL_GetTick();

    /* === P0-B：baro 發射台基準（pad_ref） ===
     * PAD/INIT 期每 30s 重零（抗氣象漂移），起飛後凍結；
     * 首筆有效氣壓樣本（pressure > 1000 Pa）才建立基準，避免開機初期髒值。 */
    static float    pad_ref       = 0.0f;
    static uint8_t  pad_ref_valid = 0U;
    static uint32_t pad_ref_tick  = 0U;
    if (g_fsm_ctx.state <= STATE_PAD && baro_data.pressure > 1000.0f) {
        if (!pad_ref_valid || (now - pad_ref_tick) >= 30000U) {
            /* 診斷：單筆快照鎖定 pad_ref，若這筆剛好是瞬態壞值（例如握持板子
             * 造成氣壓瞬變），之後全部 baro_alt_rel 會持續偏移直到下次 30s
             * 重零——印出來才能確認是否為 LIFTOFF 誤觸發的根因。 */
            printf("[FSM] pad_ref locked: %d cm (was %s)\r\n",
                   (int)(baro_data.altitude * 100.0f), pad_ref_valid ? "refresh" : "first");
            pad_ref       = baro_data.altitude;
            pad_ref_valid = 1U;
            pad_ref_tick  = now;
        }
    }

    /* P0-D：彙整感測器健康位（100Hz；初始化失敗從未餵入 → STALE 自動涵蓋） */
    uint8_t sens_bits = 0U;
    if (sensor_mon_status(&g_mon_bmi088,  now) != 0U) sens_bits |= SH_BIT_BMI088;
    if (sensor_mon_status(&g_mon_adxl375, now) != 0U) sens_bits |= SH_BIT_ADXL375;
    if (sensor_mon_status(&g_mon_bmp388,  now) != 0U) sens_bits |= SH_BIT_BMP388;
    g_sensor_fault_bits = sens_bits;

    FSM_Input_t in;
    in.now_ms         = now;
    in.h_est          = ekf_state.pos_z;   // 卡爾曼(EKF)估計高度 (m)：僅為初始值，FEATURE_VFILTER_FSM=1
                                            // （現行）時下方會被 VF 輸出覆寫，見該區塊
    in.v_est          = ekf_state.vel_z;   // 卡爾曼(EKF)估計垂直速度 (m/s)：同上，會被 VF 覆寫
    /* 改吃 BMI088 低-G（原 highg_data.az / ADXL375）：平放實測 ADXL375 三軸雜訊
     * 爆表（Z 軸 mean=20mG σ=645mG，離 1g 差 50 倍），BMI088 同軸乾淨
     * （mean=1014mG σ=5mG）。BMI088 設定 ±24g（bmi088.c），量程覆蓋 3g 起飛
     * 門檻與任何合理燒完前推力，門檻常數(3.0g/0.5g)不需重調。 */
    in.a_z_g          = imu_data.az;       // 低 G 垂直加速度 (g)
    in.baro_alt_rel   = pad_ref_valid ? (baro_data.altitude - pad_ref) : 0.0f;
    in.est_calibrated = EKF_calibrated;    // 初始值＝真 EKF 校準旗標；FEATURE_VFILTER_FSM=1 時下方會
                                            // 覆寫成 pad_ref_valid（VF 無獨立校準階段，見 fsm.h 欄位註解）
#if IS_BACKUP
    /* 備援航電（IS_BACKUP）：ARM 武裝狀態經由板間 UART 鏈路（USART2）跟隨主航電同步 */
#if FEATURE_LINK
    {
        const LinkPeer_t *peer = Link_GetPeer();
        if (Link_PeerFresh(now) && peer->board_id == LINK_BOARD_PRIMARY) {
            in.uplink_armed = (peer->fsm_state == STATE_PAD_ARMED) ? 1U : 0U;
        } else {
            in.uplink_armed = (g_fsm_ctx.state == STATE_PAD_ARMED) ? 1U : 0U;
        }
    }
#else
    in.uplink_armed = (g_fsm_ctx.state == STATE_PAD_ARMED) ? 1U : 0U;
#endif
#else
    in.uplink_armed   = UplinkCmd_IsArmed();
#endif
#if FEATURE_LINK
    in.peer_drogue_cmd = Link_GetPeer()->drogue_latched;  /* D1 加法互救：對端已開副傘（鎖存旗標） */
#else
    in.peer_drogue_cmd = 0U;
#endif
    /* ARM flash-pool 閘（選項 A：fail-open）——flash 停用/未偵測到/未在記錄時恆視為就緒，
     * 不擋 ARM；只有「flash 確實啟用中且正在記錄」才要求池達 FLASH_RING_PREERASE_TARGET。 */
#if FEATURE_FLASH
    in.flash_pool_ready = (!flash_ring_hw_ok || !g_flash_logging_active)
                          ? 1U
                          : (FlashRing_GetPoolSectors() >= FLASH_RING_PREERASE_TARGET) ? 1U : 0U;
#else
    in.flash_pool_ready = 1U;
#endif
#if defined(FEATURE_FORCE_BARO_ONLY) && FEATURE_FORCE_BARO_ONLY
    in.est_healthy    = 0U;   /* [TEST ONLY] 強制降級使用純氣壓開傘鏈（忽略估計器的高度/速度判定） */
#else
    /* P0-C：初始值＝真 EKF 健康位（健康位全 0 且 EKF_Task 300ms 內有更新才視為 healthy，餓死防護）。
     * FEATURE_VFILTER_FSM=1（現行）時下方會覆寫成 VF 健康位，見該區塊。 */
    in.est_healthy    = (EKF_GetHealthBits() == 0U &&
                         (now - EKF_GetLastUpdateTick()) <= 300U) ? 1U : 0U;
#endif
    /* P0-D：baro 路徑閘控 = sensor_health 的 BMP388 位 + pad 基準已建立 */
    in.sensor_bits    = (((sens_bits & SH_BIT_BMP388) == 0U) && pad_ref_valid)
                        ? 0U : FSM_SB_BARO_FAULT;

#if FEATURE_VFILTER
    /* === 垂直通道 3 狀態 Kalman（vertical_filter.h，Schultz 架構）===
     * FEATURE_VFILTER_FSM=1（現行）：本濾波器為開傘主估計器，下方直接把 h_est/v_est
     * 接進 FSM 輸入（EKF 僅供姿態/遙測/記錄）。若設 0 則退回記錄模式：與 EKF 並行
     * 估計、僅 1Hz 對照列印、EKF 不健康時才 fallback 頂上。 */
    static VFilter_t s_vf;
    static uint8_t   s_vf_started      = 0U;
    static float     s_vf_last_baro    = 0.0f;   /* 上次推入的 baro 相對高度 */
    static uint8_t   s_vf_baro_any     = 0U;     /* 已推過至少一筆 baro */
    static uint32_t  s_vf_baro_ok_tick = 0U;     /* 最近一筆「被接受」的 baro tick */
    if (!s_vf_started) { vf_init(&s_vf); s_vf_started = 1U; }

    vf_predict(&s_vf, 0.01f);   /* 100Hz 呼叫頻率契約（同 FSM_STEP_PERIOD_S） */

    /* baro 有效（壓力合理 + pad 基準已建立）且 altitude 與上次推入值不同時才
     * 更新：BMP388 實際更新率 < 100Hz，重複推同一筆會人為壓低量測噪聲。 */
    if (baro_data.pressure > 1000.0f && pad_ref_valid) {
        float vf_baro_rel = baro_data.altitude - pad_ref;
        if (!s_vf_baro_any || vf_baro_rel != s_vf_last_baro) {
            if (vf_update_baro(&s_vf, vf_baro_rel)) {
                s_vf_baro_ok_tick = now;   /* 維護最近接受 tick（VFILTER_FSM 健康判定用） */
            }
            s_vf_last_baro = vf_baro_rel;
            s_vf_baro_any  = 1U;
        }
    }

    /* 姿態融合垂直加速度投影（Attitude-Fused Vertical Acceleration Projection）：
     * 利用 EKF 解出之姿態四元數 q = [q0, q1, q2, q3]，將箭體三軸 IMU 加速度 (ax, ay, az)
     * 正交投影至慣性世界座標 Z 軸 (垂直方向)，消除大仰角/斜拋軌跡下的加速度投影偏差。
     * R_31 = 2(q1*q3 - q0*q2), R_32 = 2(q2*q3 + q0*q1), R_33 = 1 - 2(q1^2 + q2^2) */
    EKF_State_t vf_ekf_st = EKF_GetState();
    float fq0 = vf_ekf_st.q[0], fq1 = vf_ekf_st.q[1], fq2 = vf_ekf_st.q[2], fq3 = vf_ekf_st.q[3];
    float R31 = 2.0f * (fq1 * fq3 - fq0 * fq2);
    float R32 = 2.0f * (fq2 * fq3 + fq0 * fq1);
    float R33 = 1.0f - 2.0f * (fq1 * fq1 + fq2 * fq2);
    float az_world_g = R31 * imu_data.ax + R32 * imu_data.ay + R33 * imu_data.az;
    vf_update_accel(&s_vf, (az_world_g - 1.0f) * 9.80665f);

    /* 鏡射至全域，供 telemetry.c 下鏈自身摘要 + Link_BuildOwnStatus 中繼給對端
     * （與 EKF 並列輸出，供地面站同屏比對）。 */
    g_vf_h_m  = vf_h(&s_vf);
    g_vf_v_ms = vf_v(&s_vf);

#if FEATURE_VFILTER_FSM
    /* 場驗後開啟：開傘決策輸入改接垂直濾波器（EKF 僅供姿態/遙測/記錄）。本區塊覆寫
     * FSM_Input_t 的 est_calibrated/est_healthy/h_est/v_est——此後這幾個欄位代表的是
     * VF 的狀態，不是 EKF 的（上方初始賦值的真 EKF 值到此已作廢，見 fsm.h 欄位註解）。
     * vf 無靜態校準階段 → calibrated 即 pad 基準就緒；健康 = 300ms 內有 baro
     * 被創新值閘接受（baro 斷流/持續拒收即降級走純氣壓鏈，與 P0-C 同語意）。 */
    in.h_est          = vf_h(&s_vf);
    in.v_est          = vf_v(&s_vf);
    in.est_calibrated = pad_ref_valid;
    in.est_healthy    = (s_vf_baro_ok_tick != 0U &&
                         (now - s_vf_baro_ok_tick) <= 300U) ? 1U : 0U;
#else
    /* EKF 為主估計器；當 EKF 不健康時，若垂直通道濾波器 (Vertical Filter) 健康，
     * 則降級使用 Vertical Filter 的高度與速度，以實現『不健康時也能在速度降到0前3s預測開傘』。
     * （此分支未編譯於現行設定，FEATURE_VFILTER_FSM=1；est_healthy 在此仍是上方賦的真 EKF 值。） */
    {
        uint8_t ekf_ok = in.est_healthy;
        uint8_t vf_ok = (s_vf_baro_ok_tick != 0U && (now - s_vf_baro_ok_tick) <= 300U) ? 1U : 0U;
        if (!ekf_ok && vf_ok) {
            in.h_est = vf_h(&s_vf);
            in.v_est = vf_v(&s_vf);
            in.est_healthy = 1U;
            /* 印出降級預警，以利日誌排查 */
            static uint32_t last_warn_tick = 0;
            if (now - last_warn_tick >= 1000) {
                printf("[FSM] [WARNING] EKF unhealthy! Falling back to Vertical Filter for apogee prediction.\r\n");
                last_warn_tick = now;
            }
        }
    }
#endif
#endif /* FEATURE_VFILTER */

#if defined(FEATURE_FORCE_BARO_ONLY) && FEATURE_FORCE_BARO_ONLY
    in.est_healthy    = 0U;   /* 強制覆蓋：電梯 profile 全程強制走純氣壓降級鏈，禁止估計器觸發動態頂點與主傘 */
#endif

    /* --- 飛行滾動極值：高度/速度（100Hz）---
     * 取 in.h_est/in.v_est —— 也就是 FSM 開傘判斷「實際採用」的那組估計值（現行設定為 VF，
     * 見上方 FEATURE_VFILTER_FSM 區塊），下鏈的 max 才會跟飛控的決策依據同源、可互相印證。
     * 放在 FSM_Step 之前：本輪的 in 已完全定案（含降級改接），且不受 FSM_Step 內部影響。
     * 只在真的離開發射台後才累積（PAD/PAD_ARMED 期間 h_est 在 0 附近抖動，且搬動板子會
     * 灌進假的最大速度）。 */
    {
        FlightState_t st_now = (FlightState_t)current_fsm_state;
        if (st_now != STATE_INIT && st_now != STATE_PAD && st_now != STATE_PAD_ARMED) {
            if (in.h_est > g_max_alt_m)  g_max_alt_m  = in.h_est;
            if (in.v_est > g_max_vel_ms) g_max_vel_ms = in.v_est;
        }
    }

    FSM_Action_t act = FSM_Step(&g_fsm_ctx, &in);

    /* --- 1. 硬體動作（先於任何列印） ---
     * 對稱獨立：主/副兩板跑同一份 FSM、同一組輸出，各自依自身感測器獨立開傘。
     * 副傘 DC 馬達＝PD13（簡單 GPIO，導通 FSM_DROGUE_MOTOR_RUN_MS：主 8s / 副 3s）；
     * 主傘＝PD14，平時硬體拉低、無任何訊號（見 Servo_HoldLow），部署時純 GPIO 拉高
     * SERVO_MAIN_HIGH_MS(1.5s) 後回低（Servo_MainHigh，★不再輸出 PWM）。 */
    if (act.fire_drogue) {
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);    // 啟動副傘 DC 馬達（PD13）
        /* 就地鎖存副傘開傘高度（只記第一次）：下鏈 ~2.3Hz，開傘那一瞬間的封包很可能
         * 剛好丟掉，事後從稀疏取樣反推不出真正的開傘高度。用 in.h_est 與 max_alt_m 同源。 */
        if (g_drogue_alt_m == TELEM_DEPLOY_ALT_NA) {
            g_drogue_alt_m = sat_alt_i16(in.h_est);
        }
    }
    if (act.release_drogue) {
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);  // 馬達限時導通保護：斷開
    }
    /* --- 主傘 PD14 共開（原 D2 互斥握手已取消，見 servo_arb.h / board_config.h）---
     * 主傘不再輸出 PWM：判到就把 PD14 純 GPIO 拉高 SERVO_MAIN_HIGH_MS(1.5s) 後回低。
     * 兩板 PD14 走 diode-OR 準位訊號，同時拉高無害，故「一板判到 → 呼叫對端一起開」，
     * 不做讓位/guard/隔離。want 來源：
     *   (1) 本板 FSM 判到主傘（act.deploy_main，鎖存，開傘不撤回）；
     *   (2) 對端已在拉高/已拉完（peer_arb = MAIN_HIGH / DONE）→ 本板跟著一起開。
     * (2) 的台上誤觸防護 = in-flight 閘：本板須已進入 STATE_COAST 之後（必經 BOOST 起飛
     * 與燒完）才接受對端呼叫。刻意不用舊的「峰值 baro > 300m」閘——那條在本板 baro 失效
     * 時會把冗餘一起擋掉，而 baro 失效正是最需要對端拉一把的情境。 */
    if (act.deploy_main) g_servo_want_latched = 1U;
    {
#if FEATURE_LINK
        const LinkPeer_t *sa_peer = Link_GetPeer();
        uint8_t peer_arb = Link_PeerFresh(now) ? sa_peer->peer_main_arb : SERVO_ARB_MSG_NONE;
        if ((g_fsm_ctx.state >= STATE_COAST) &&
            (peer_arb == SERVO_ARB_MSG_MAIN_HIGH || peer_arb == SERVO_ARB_MSG_DONE)) {
            g_servo_want_latched = 1U;   /* 對端呼叫 → 一起開（本板已在飛行中） */
        }
#endif
        ServoArbAction_t sa = ServoArb_Step(&g_servo_arb, g_servo_want_latched, now,
                                            SERVO_MAIN_HIGH_MS);
        g_servo_arb_bcast = sa.broadcast_arb;
        if (sa.high_servo && !g_servo_high) {
            Servo_MainHigh();            // 上升邊：PD14 純 GPIO 拉高（無 PWM）
            g_servo_high    = 1U;
            g_main_deployed = 1U;        // 下鏈 MAIN_DEPLOYED 鎖存（PD14 只高 1.5s，需鎖存）
            /* 鎖存主傘開傘高度：取「PD14 真的被拉高」那一刻，而不是 act.deploy_main
             * 判定成立那一刻——本板也可能是被對端呼叫才開，記判定時刻會失真。 */
            if (g_main_alt_m == TELEM_DEPLOY_ALT_NA) {
                g_main_alt_m = sat_alt_i16(in.h_est);
            }
        } else if (sa.release_servo && g_servo_high) {
            Servo_HoldLow();             // 1.5s 窗結束：PD14 回低
            g_servo_high = 0U;
        }
    }

#if FEATURE_UPLINK_DEPLOY
    /* 上行手動開傘（地面站 433 命令，uplink_cmd 已經 ARM→DEPLOY 兩段式驗證）。
     * 與 FSM 自動點火並存：副傘直接驅動點火輸出並鎖存 drogue_fired（防 FSM 二次點火）；
     * 主傘改走上方共開路徑（g_servo_want_latched）。下行 DROGUE_FIRED（PD13 現況）/
     * MAIN_DEPLOYED（g_main_deployed 鎖存）旗標即為地面站確認。 */
    {
        static uint8_t  s_man_drogue_active = 0U;
        static uint32_t s_man_drogue_tick   = 0U;
        uint8_t man_drogue = 0U, man_main = 0U;
        if (UplinkCmd_TakeDeploy(&man_drogue, &man_main)) {
            if (man_drogue) {
                HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);   // 手動啟動副傘 DC 馬達
                g_fsm_ctx.drogue_fired = 1U;
                s_man_drogue_active = 1U;
                s_man_drogue_tick   = now;
            }
            if (man_main) {
                /* 手動主傘走同一條共開路徑：PD14 拉高 1.5s 後自動回低，並廣播 MAIN_HIGH
                 * 呼叫對端一起開（單板無鏈路時就只是本板拉高，行為不變）。 */
                g_servo_want_latched = 1U;
            }
        }
        /* 手動副傘導通限時保護：FSM_DROGUE_MOTOR_RUN_MS 後斷開馬達（同 FSM 自身馬達限時）。 */
        if (s_man_drogue_active && (now - s_man_drogue_tick) >= FSM_DROGUE_MOTOR_RUN_MS) {
            HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
            s_man_drogue_active = 0U;
        }

        uint8_t rec_seq = 0;
        if (UplinkCmd_TakeRecovery(&rec_seq)) {
            App_ExecuteRecovery();
        }
    }
#endif

#if FEATURE_BUZZER
    if (act.start_buzzer) {
        // 開啟板載尋標蜂鳴器（持續鳴叫，利於落點尋標）
        htim2.Instance->ARR  = 999;
        htim2.Instance->CCR1 = 500;
        htim2.Instance->EGR  = TIM_EGR_UG;
    }
#endif

    /* --- 2. 鏡射回全域（telemetry / flash ring / SD / EKF 既有讀取點不變） --- */
    current_fsm_state = g_fsm_ctx.state;
    flight_start_tick = g_fsm_ctx.flight_start_ms;

    /* ARM 被 flash pool 擋下：鏡射全域供 telemetry.c 下鏈，並限流印出原因（USB/console）。
     * fail-open（選項 A）已在 in.flash_pool_ready 組裝時處理，這裡只是回報。 */
    g_arm_blocked_flash = act.arm_blocked_flash;
    if (act.arm_blocked_flash) {
        static uint32_t s_last_warn_tick = 0;
        if (now - s_last_warn_tick >= 1000) {
            printf("[FSM] [ARM BLOCKED] Flash pool 未達標 (%lu/%u sectors)，ARM 被擋下，等待背景預擦完成。\r\n",
                   (unsigned long)FlashRing_GetPoolSectors(), (unsigned)FLASH_RING_PREERASE_TARGET);
            s_last_warn_tick = now;
        }
    }
    if (act.event == FSM_EVT_TOUCHDOWN) {
        g_touchdown_tick = now;   // 記錄落地時刻：SD 記錄降為 10Hz，逾時（SD_LANDED_LOG_TIMEOUT_MS）後關檔
    }

    /* P0-E：武裝待發起禁止 Flash 同步滾動擦除（~50~400ms 阻塞）。武裝後隨時可能起飛，
     * 擦除不得落在該瞬間；PAD/PAD_ARMED 已不寫入 flash（見下方 ring_enabled），池子在此
     * 窗口內不被消耗，本限制零代價。 */
    FlashRing_SetEraseAllowed((g_fsm_ctx.state >= STATE_PAD_ARMED &&
                               g_fsm_ctx.state <= STATE_MAIN_DEPLOY) ? 0U : 1U);

    /* --- 3. 事件列印（訊息逐字保留自原 FSM_Update） --- */
    switch ((FSM_Event_t)act.event) {
        case FSM_EVT_ARMED:
            /* PAD/PAD_ARMED 已不寫入 flash（見下方 ring_enabled），原本靠飛行寫入路徑
             * 順帶配置 flight_id 的作法不再觸發，改在 ARM 事件當下直接配置。 */
            FlashFlight_EnsureActive();
            printf("[FSM] [ARMED] Launch pad ARMED! Liftoff detection ENABLED. Entering STATE_PAD_ARMED.\r\n");
            break;
        case FSM_EVT_DISARMED:
            printf("[FSM] [DISARMED] Launch pad DISARMED! Liftoff detection DISABLED. Entering STATE_PAD.\r\n");
            break;
        case FSM_EVT_LIFTOFF:
            /* 診斷：印出三條起飛判定路徑的當下數值，直接鎖定是哪條觸發（別再用
             * CSV 行反推）。pad_ref 為 FSM_Update 內 static，本行同函式作用域可讀。 */
            printf("[FSM] [LIFTOFF] LIFTOFF DETECTED! a_z=%d mg h_est=%d cm baro_rel=%d cm "
                   "pad_ref=%d cm cal=%u healthy=%u sb=0x%02X\r\n",
                   (int)(in.a_z_g * 1000.0f), (int)(in.h_est * 100.0f),
                   (int)(in.baro_alt_rel * 100.0f), (int)(pad_ref * 100.0f),
                   (unsigned)in.est_calibrated, (unsigned)in.est_healthy,
                   (unsigned)in.sensor_bits);
            break;
        case FSM_EVT_BURNOUT:
            printf("[FSM] [BURNOUT] MOTOR BURNOUT! Entering STATE_COAST.\r\n");
            break;
        case FSM_EVT_DEPLOY_DROGUE:
            printf("[FSM] [DEPLOY_DROGUE] PREDICTIVE TRIGGER! Expected apogee in %d ms (h_est: %d cm). Starting drogue motor (%lu ms run).\r\n",
                   (int)(act.apogee_t_pred * 1000.0f), (int)(in.h_est * 100.0f),
                   (unsigned long)FSM_DROGUE_MOTOR_RUN_MS);
            break;
        case FSM_EVT_APOGEE_FAILSAFE:
            g_fsm_failsafe_fired = 1U;
            printf("[FSM] [FAILSAFE] FAILSAFE APOGEE TIMEOUT (%lu ms)! Forcing drogue motor start. Entering "
                   "STATE_DEPLOY_DROGUE (h_est: %d cm, baro_rel: %d cm).\r\n",
                   (unsigned long)(IS_PRIMARY ? FSM_FAILSAFE_APOGEE_MS : FSM_FAILSAFE_APOGEE_BACKUP_MS),
                   (int)(in.h_est * 100.0f), (int)(in.baro_alt_rel * 100.0f));
            break;
        case FSM_EVT_DROGUE_DONE:
            printf("[FSM] [DROGUE] Drogue motor run complete (%lu ms). Entering STATE_APOGEE (record) -> STATE_DESCENT.\r\n",
                   (unsigned long)FSM_DROGUE_MOTOR_RUN_MS);
            break;
        case FSM_EVT_MAIN_DEPLOY:
            printf("[FSM] [MAIN] DYNAMIC LOW ALTITUDE REACHED! Trigger H: %d cm (Target H: %d cm, Fall V: %d cm/s). "
                   "Deploying MAIN. Entering STATE_MAIN_DEPLOY.\r\n",
                   (int)(in.h_est * 100.0f), (int)(TARGET_MAIN_ALTITUDE * 100.0f), (int)(in.v_est * 100.0f));
            break;
        case FSM_EVT_MAIN_OPEN:
            printf("[FSM] Main deployed. Entering STATE_LANDED (landing detection).\r\n");
            break;
        case FSM_EVT_TOUCHDOWN:
            printf("[FSM] [TOUCHDOWN] TOUCHDOWN! 持續記錄中（降頻 10Hz）。逾時後自動關檔。\r\n");
            break;
        default:
            break;
    }

#if FEATURE_VFILTER
    /* 1Hz A/B 對照列印（vf vs EKF，供場測記錄與後續分析）。位於硬體動作與
     * 事件列印之後 —— 不插入動作執行路徑，維持「硬體動作先於 printf」紀律。 */
    {
        static uint32_t s_vf_print_tick = 0U;
        if ((now - s_vf_print_tick) >= 1000U) {
            s_vf_print_tick = now;
            /* 縮放整數輸出（nano.specs 無 %f，遵 gs_log.c 黃金法則）。
             * braw_cm：pad_ref 相對地面高度原始值（未經 VF 卡爾曼融合），
             * 直接反映 in.baro_alt_rel = baro_data.altitude - pad_ref（見本函式開頭
             * pad_ref 每 30s 於 PAD 重零區塊）——地面站 bench 圖表要看「地面 0 點
             * 有沒有持續校正」需要這條未濾波的原始線，VF/EKF 都已經是融合後結果。 */
            printf("[VF] h_cm=%d v_cms=%d | EKF h_cm=%d v_cms=%d | braw_cm=%d\r\n",
                   (int)(vf_h(&s_vf) * 100.0f), (int)(vf_v(&s_vf) * 100.0f),
                   (int)(ekf_state.pos_z * 100.0f), (int)(ekf_state.vel_z * 100.0f),
                   (int)(in.baro_alt_rel * 100.0f));
        }
    }
#endif
}

#if FEATURE_LINK
/* 對端 fsm_state（FlightState_t 數值）→ 簡短字串，供 [LINK] 診斷行輸出（GUI 監控用）。 */
static const char *link_fsm_state_name(uint8_t s)
{
    static const char *const names[] = {
        "INIT", "PAD", "PAD_ARMED", "BOOST", "COAST", "DEP_DROGUE",
        "APOGEE", "DESCENT", "MAIN_DEPLOY", "LANDED"
    };
    return (s < (sizeof(names) / sizeof(names[0]))) ? names[s] : "UNK";
}

/* === 開機訊息與進度同步廣播至 USB 與 LoRa (433/920) === */
static void Boot_BroadcastPrompt(const char *msg)
{
    if (!msg) return;
    uint16_t len = (uint16_t)strlen(msg);
    printf("%s\r\n", msg);
    fflush(stdout);

#if FEATURE_LORA
    if (lora433_ok) {
        LoRaE22_Send((const uint8_t *)msg, len);
    }
    if (lora920_ok) {
        uint8_t e80_len = (len > 255U) ? 255U : (uint8_t)len;
        LoRaE80_Send((const uint8_t *)msg, e80_len);
    }
#endif
    HAL_IWDG_Refresh(&hiwdg);
}

/* === 板間鏈路：自身狀態組裝 + 週期廣播 ===
 * 注意：Flash 開機預擦期間主迴圈尚未進入，Link_PublishTick 不會跑；
 * 必須在預擦 callback 內主動 Link_SendStatus，否則對端 peer_erase_pct 永遠是 0。 */
static void Link_BuildOwnStatus(LinkStatus_t *ls)
{
    EKF_State_t e = EKF_GetState();
    ls->board_id  = IS_BACKUP ? LINK_BOARD_BACKUP : LINK_BOARD_PRIMARY;
    ls->seq       = 0U;   /* 由 Link_SendStatus 補遞增序號 */
    ls->fsm_state = (uint8_t)current_fsm_state;

    /* flags 與下行遙測同語意（TELEM_FLAG_*）；備板據主板 DROGUE_FIRED/MAIN_DEPLOYED 抑制 */
    uint8_t flags = 0U;
    if (HAL_GPIO_ReadPin(FIRE_GPIO_Port, FIRE_Pin) == GPIO_PIN_SET)   flags |= TELEM_FLAG_DROGUE_FIRED;
    if (g_main_deployed)                                              flags |= TELEM_FLAG_MAIN_DEPLOYED;
    if (g_fsm_failsafe_fired)                                         flags |= TELEM_FLAG_FAILSAFE;
    if (EKF_GetHealthBits() != 0U)                                    flags |= TELEM_FLAG_EKF_UNHEALTHY;
    if (g_sensor_fault_bits != 0U)                                    flags |= TELEM_FLAG_SENSOR_FAULT;
    ls->flags = flags;

    ls->tick_ms     = HAL_GetTick();
    ls->h_est_cm    = (int32_t)(e.pos_z * 100.0f);
    ls->v_est_cms   = (int32_t)(e.vel_z * 100.0f);
    ls->baro_alt_cm = (int32_t)(baro_data.altitude * 100.0f);
    ls->a_z_cg      = (int16_t)(highg_data.az * 100.0f);
    ls->q_w         = (int16_t)(e.q[0] * 10000.0f);
    ls->q_x         = (int16_t)(e.q[1] * 10000.0f);
    ls->q_y         = (int16_t)(e.q[2] * 10000.0f);
    ls->q_z         = (int16_t)(e.q[3] * 10000.0f);
    /* echo-ACK：回送「我最近採納的對端 FSM 狀態」，供對端偵測失同步（純觀測） */
    ls->ack_state   = Link_GetPeer()->fsm_state;
    /* 廣播本板主傘 PD14 共開狀態（MAIN_HIGH/DONE：對端據此「一起開」，見 servo_arb.h） */
    ls->main_arb    = g_servo_arb_bcast;
#if FEATURE_FLASH
    ls->flash_ready = (FlashRing_GetPoolSectors() >= FLASH_RING_PREERASE_TARGET) ? 1U : 0U;
    ls->erase_pct   = FlashRing_GetErasePct();
#else
    ls->flash_ready = 1U;
    ls->erase_pct   = 100U;
#endif
    /* 垂直濾波器 (VF)：與 EKF h/v 並列中繼給對端，供對端下鏈時一併轉發給地面站。 */
    ls->vf_h_cm  = (int32_t)(g_vf_h_m * 100.0f);
    ls->vf_v_cms = (int32_t)(g_vf_v_ms * 100.0f);
    /* BMI088/ADXL375 加速度模長（|a|，三軸向量長度）：與本板原始加速度圖對照用，
     * 讓對端（含 USB 直連單板的 bench 圖表）也能看到「本板」原始感測器過載程度，
     * 不只是融合後的 EKF/VF 估計值。 */
    ls->bmi_mag_cg  = (int16_t)(sqrtf(imu_data.ax * imu_data.ax +
                                       imu_data.ay * imu_data.ay +
                                       imu_data.az * imu_data.az) * 100.0f);
    ls->adxl_mag_cg = (int16_t)(sqrtf(highg_data.ax * highg_data.ax +
                                        highg_data.ay * highg_data.ay +
                                        highg_data.az * highg_data.az) * 100.0f);
    /* 電梯測試 profile 醒目標示：讓對端（及對端下鏈給地面站）知道「本板」是否仍以
     * 電梯測試門檻編譯——雙板換板/重燒時最容易漏掉其中一片，靠此位互相中繼揭穿。 */
#if FLIGHT_PROFILE_ELEVATOR
    ls->profile_flags = TELEM_PROFILE_SELF_ELEVATOR;
#else
    ls->profile_flags = 0U;
#endif
}

/* 預擦期間輕量廣播：只關心 erase_pct / flash_ready 被對端收到。
 * 每 sector 擦完呼叫；上一筆 IT 未送完則 Link_SendStatus 自動略過。 */
static void Link_PublishEraseProgress(void)
{
    LinkStatus_t ls;
    Link_BuildOwnStatus(&ls);
    Link_SendStatus(&ls);
}

/* 組「主/副絕對角色」進度字串，讓主航電 USB-CDC 一眼看兩板（不靠 self/peer 相對語意）。 */
static void FlashErase_FormatDualProgress(char *buf, size_t buflen,
                                          uint8_t self_pct, uint32_t current, uint32_t total)
{
    const LinkPeer_t *peer = Link_GetPeer();
    uint8_t peer_valid = peer->valid;
    uint8_t peer_pct   = peer_valid ? peer->peer_erase_pct : 0U;
    uint8_t peer_ready = peer_valid ? peer->peer_flash_ready : 0U;
    uint8_t peer_fresh = peer_valid ? Link_PeerFresh(HAL_GetTick()) : 0U;

    uint8_t primary_pct, backup_pct;
    uint8_t primary_ready, backup_ready;
    if (IS_BACKUP) {
        primary_pct   = peer_valid ? peer_pct : 0U;
        primary_ready = peer_valid ? peer_ready : 0U;
        backup_pct    = self_pct;
        backup_ready  = (self_pct >= 100U) ? 1U : 0U;
    } else {
        primary_pct   = self_pct;
        primary_ready = (self_pct >= 100U) ? 1U : 0U;
        backup_pct    = peer_valid ? peer_pct : 0U;
        backup_ready  = peer_valid ? peer_ready : 0U;
    }

    if (total > 0U) {
        snprintf(buf, buflen,
                 "[FLASH_RING] primary=%u%%%s backup=%u%%%s  "
                 "self=%s %lu/%lu  peer=%s%s",
                 (unsigned)primary_pct, primary_ready ? "(rdy)" : "",
                 (unsigned)backup_pct,  backup_ready  ? "(rdy)" : "",
                 IS_BACKUP ? "BACKUP" : "PRIMARY",
                 (unsigned long)current, (unsigned long)total,
                 !peer_valid ? "NONE" : (peer_fresh ? "OK" : "STALE"),
                 peer_valid ? "" : " (waiting link)");
    } else {
        /* total=0：非 sector 進度列（例如等對端收尾） */
        snprintf(buf, buflen,
                 "[FLASH_RING] primary=%u%%%s backup=%u%%%s  "
                 "self=%s peer=%s%s",
                 (unsigned)primary_pct, primary_ready ? "(rdy)" : "",
                 (unsigned)backup_pct,  backup_ready  ? "(rdy)" : "",
                 IS_BACKUP ? "BACKUP" : "PRIMARY",
                 !peer_valid ? "NONE" : (peer_fresh ? "OK" : "STALE"),
                 peer_valid ? "" : " (waiting link)");
    }
}

static void FlashPreErase_ProgressCallback(uint32_t current, uint32_t total)
{
    /* 1) 每 sector 廣播自身 erase_pct → 對端 DMA RX 可更新 peer_erase_pct。
     *    主迴圈 Link_PublishTick 此時尚未跑，這是雙板進度互通的唯一路徑。 */
    Link_PublishEraseProgress();
    HAL_IWDG_Refresh(&hiwdg);

    /* 2) USB-CDC / LoRa：每 10% 或對端進度有明顯變化時印雙板進度。
     *    主航電插 USB 時應同時看到 primary=..% 與 backup=..%。 */
    uint32_t step = total / 10U;  /* 960 → 96 */
    if (step == 0U) step = 1U;

    static uint8_t s_last_printed_peer_pct = 0xFFU;
    const LinkPeer_t *peer = Link_GetPeer();
    uint8_t peer_pct = peer->valid ? peer->peer_erase_pct : 0U;
    uint8_t peer_step_changed =
        peer->valid &&
        (s_last_printed_peer_pct == 0xFFU ||
         (uint8_t)(peer_pct / 10U) != (uint8_t)(s_last_printed_peer_pct / 10U));

    if ((current % step == 0U) || (current == total) || peer_step_changed) {
        char buf[192];
        uint8_t self_pct = (uint8_t)((current * 100U) / total);
        FlashErase_FormatDualProgress(buf, sizeof(buf), self_pct, current, total);
        Boot_BroadcastPrompt(buf);
        if (peer->valid) s_last_printed_peer_pct = peer_pct;
    }
}

/* 本板預擦完成後：若對端仍在擦，持續廣播 + 印雙板進度，直到對端 ready / 100%
 * 或逾時。主航電 USB-CDC 因此可在「主先擦完、副還在擦」時繼續看到副板進度。
 * 若開機期間從未見過對端，短暫等一下就放棄（單板台面測試不卡住）。 */
static void Boot_WaitPeerEraseProgress(void)
{
    const uint32_t kNoPeerGiveUpMs   = 3000U;    /* 從未見過對端：3s 後放棄 */
    const uint32_t kPeerTimeoutMs    = 180000U;  /* 見過對端：最多再等 3 min */
    const uint32_t kPrintPeriodMs    = 1000U;

    uint32_t t0 = HAL_GetTick();
    uint32_t last_print = 0U;
    uint8_t  saw_peer = Link_GetPeer()->valid;

    for (;;) {
        uint32_t now = HAL_GetTick();
        Link_PublishEraseProgress();
        HAL_IWDG_Refresh(&hiwdg);

        const LinkPeer_t *peer = Link_GetPeer();
        if (peer->valid) saw_peer = 1U;

        /* 對端已 ready / 已 100% → 完成 */
        if (peer->valid && (peer->peer_flash_ready || peer->peer_erase_pct >= 100U)) {
            char buf[192];
            FlashErase_FormatDualProgress(buf, sizeof(buf), 100U, 0U, 0U);
            Boot_BroadcastPrompt(buf);
            Boot_BroadcastPrompt("[FLASH_RING] Dual-board pre-erase COMPLETE (peer ready).");
            return;
        }

        /* 從未見過對端且逾時 → 單板模式，不卡住開機 */
        if (!saw_peer && (now - t0) >= kNoPeerGiveUpMs) {
            Boot_BroadcastPrompt("[FLASH_RING] No peer on link during pre-erase (single-board OK).");
            return;
        }

        /* 見過對端但太久沒 ready → 放棄等待，進主迴圈後 [LINK] 1Hz 仍會更新 */
        if (saw_peer && (now - t0) >= kPeerTimeoutMs) {
            Boot_BroadcastPrompt("[FLASH_RING] Peer erase wait TIMEOUT — continue boot, watch [LINK] logs.");
            return;
        }

        if ((now - last_print) >= kPrintPeriodMs) {
            last_print = now;
            char buf[192];
            FlashErase_FormatDualProgress(buf, sizeof(buf), 100U, 0U, 0U);
            Boot_BroadcastPrompt(buf);
        }

        /* 預擦阻塞於 defaultTask：用 osDelay 讓出 CPU，UART DMA/IT 與 USB CDC 可跑 */
        osDelay(50U);
    }
}

/* 飛控迴圈每 LINK_TX_PERIOD_MS 呼叫一次：廣播自身狀態給對端板 + 更新鏈路健康。 */
static void Link_PublishTick(void)
{
    uint32_t now_ms = HAL_GetTick();
    Link_UpdateStatus((uint8_t)current_fsm_state, now_ms);
    LinkStatus_t ls;
    Link_BuildOwnStatus(&ls);
    Link_SendStatus(&ls);

#if IS_BACKUP
    /* 副航電同步主航電的 ARM / DISARM 狀態：
     * 不論主航電是收到 433MHz LoRa 地面站遙控 ARM，或是 USB CDC 直連 arm，
     * 副航電皆經由板間鏈路 50ms 內自動連帶切換 ARM / DISARM 狀態。 */
    if (Link_PeerFresh(now_ms)) {
        const LinkPeer_t *peer = Link_GetPeer();
        if (peer->board_id == LINK_BOARD_PRIMARY) {
            if (peer->fsm_state == STATE_PAD_ARMED &&
                (current_fsm_state == STATE_INIT || current_fsm_state == STATE_PAD)) {
                FSM_SetState(&g_fsm_ctx, STATE_PAD_ARMED);
                printf("[LINK_SYNC] 接收主航電 ARM 命令 -> 副航電同步進入 STATE_PAD_ARMED\r\n");
            } else if (peer->fsm_state == STATE_PAD && current_fsm_state == STATE_PAD_ARMED) {
                FSM_SetState(&g_fsm_ctx, STATE_PAD);
                printf("[LINK_SYNC] 接收主航電 DISARM 命令 -> 副航電同步回到 STATE_PAD\r\n");
            }
        }
    }
#endif
}
#endif /* FEATURE_LINK */

/* === HAL UART 回呼統一分派（單一定義；gps.c 已改為 GPS_Handle* 由此轉接） ===
 * USART6 → GPS（FEATURE_GPS）；USART2 → 板間鏈路（FEATURE_LINK）。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
#if FEATURE_GPS
    GPS_HandleRxEvent(huart, Size);
#endif
#if FEATURE_LINK
    if (huart->Instance == USART2) Link_OnRxEvent(Size);
#endif
#if IS_GROUND
    if (huart->Instance == USART3) GroundStation_OnUart3RxEvent(Size);  /* E22 433 透傳 RX */
    if (huart->Instance == USART2) GsLoraTest_OnUart2RxEvent(Size);     /* 通訊測試命令介面 */
#endif
#if FEATURE_UPLINK_DEPLOY
    if (huart->Instance == USART3) UplinkCmd_OnUart3RxEvent(Size);      /* 主航電：433 上行命令 RX */
#endif
#if (!FEATURE_GPS && !FEATURE_LINK && !IS_GROUND)
    (void)huart; (void)Size;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if FEATURE_GPS
    GPS_HandleUartError(huart);
#endif
#if FEATURE_LINK
    if (huart->Instance == USART2) Link_OnError();
#endif
#if IS_GROUND
    if (huart->Instance == USART3) GroundStation_OnUart3Error();   /* 433 RX 溢位(ORE)復原：不做則收一包就死 */
#endif
#if FEATURE_UPLINK_DEPLOY
    if (huart->Instance == USART3) UplinkCmd_OnUart3Error();        /* 主航電 433 上行 RX 溢位復原 */
#endif
#if (!FEATURE_GPS && !FEATURE_LINK && !IS_GROUND && !FEATURE_UPLINK_DEPLOY)
    (void)huart;
#endif
}

#if FEATURE_LINK
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) Link_OnTxComplete();
}
#endif

uint16_t ADC_Read_Battery_mv(void) {
    uint32_t val = 0;
    HAL_ADC_Start(&hadc1);
    /* P0-E：逾時 10ms→2ms。ADC1 12-bit 轉換僅 ~µs 級，2ms 已極寬裕；
     * 10ms 逾時在 ADC 故障時會凍結 1kHz 飛控迴圈 10 個週期。 */
    if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK) {
        val = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return (uint16_t)((val * 3300UL * 3UL) / 4095UL);
}

/* ===== SPI3 共用匯流排互斥鎖包裝（W25Q128 Flash + E80 920MHz LoRa，見 spi3_bus.h） =====
 * NULL-guard：mutex 建立前（scheduler 啟動前的單執行緒初始化階段）為 no-op。
 * 非持有者釋放：遞迴 mutex 於非持有者呼叫 osMutexRelease 會安全回傳錯誤（忽略），
 * 故 flash 驅動 CS 巨集中偶發的防禦性 CS_HIGH 不會誤釋放他人持有的鎖。 */
void SPI3_Bus_Lock(void)
{
    if (g_spi3_mutex != NULL) {
        osMutexAcquire(g_spi3_mutex, osWaitForever);
    }
}

void SPI3_Bus_Unlock(void)
{
    if (g_spi3_mutex != NULL) {
        osMutexRelease(g_spi3_mutex);
    }
}

/* ===== LoRa 下行遙測任務 =====
 * 每 LORA_TELEM_PERIOD_MS 打包一筆 TelemetryPacket_t，分別經 E22(433) 與 E80(920) 發送。
 * 兩條鏈路各自靠 AUX/BUSY + TxDone 背壓自我限流（忙線即跳過），互不阻塞。
 * 低優先任務：blocking UART 傳輸會被高優先飛控任務搶佔，完全不影響 1kHz 迴圈時序。 */
/* ★ 920/E80 目標 10Hz：改 SF8/BW500 後單包空中時間 87.2ms（見 lora_e80.c 的
 * E80_LORA_SF/BW 註解），100ms 時槽只留 ~13ms 排程餘裕，是刻意接受的取捨
 * （已與使用者確認；比 SF7/BW500 少了 2.5dB 但更省）。原 200ms(5Hz) 太慢，
 * 就算射頻時鐘正常、TxDone 準時，排程本身也追不上 10Hz，必須一併調快。 */
#define LORA_TELEM_PERIOD_MS  100U   /* 基礎時槽 10Hz（920/E80 用；實際仍受空中速率限制） */
/* 433/E22 每 N 個時槽「嘗試」發一次：實際發不發、發多快，現在交給 LoRaE22_Send
 * 內建的 AUX 背壓自己決定（見 lora_e22.c 的 e22_wait_aux_high 呼叫）——上一包還
 * 沒真的送上空中，這次呼叫就直接回 HAL_BUSY、本槽跳過，不會撞上正在飛的封包。
 * ★2026-07-28：既然背壓已經是真的（不再是只靠「N 選得夠保守」硬撐的隱性假設），
 *   就把 N 調到 1（每槽都嘗試），讓發送頻率自然逼近目前空中速率(2.4k)+封包大小
 *   (116B)算出來的物理上限（~386ms/包，116*8bit/2400bps）——多送的嘗試會被 AUX
 *   擋掉，不會真的疊加發射。副作用（刻意保留、對這輪頻率測試有利）：
 *   `Telemetry_Build()` 的 seq 计数器現在「每一次嘗試」都遞增一次，N=1 時等於
 *   「每一次實際送出」都對應唯一遞增的 seq——地面站收到的 433 封包 seq 因此
 *   會連續（除了下面 UPLINK_LISTEN_PERIOD_MS/HOLD_MS 那個刻意保留的上行接收窗
 *   ——那是已知、規律的空缺，不是遺失，分析時可直接排除）。 */
#define LORA433_TX_EVERY      1U
/* === 上行接收窗（地面站 → 火箭）=============================================
 * ★2026-07-30 修正：舊版是「每 10 個時槽空出 1 槽不發射」（UPLINK_LISTEN_EVERY），
 *   那個實作在目前的參數下等於沒有窗口，地面站的 ARM/DEPLOY 因此永遠打不進來：
 *     時槽 = LORA_TELEM_PERIOD_MS = 100ms，
 *     但一包 116B 在空中速率 2.4k 下要 116*8/2400 ≈ 387ms（不含 E22 前導/同步開銷）。
 *   也就是說「跳過 1 個排程時槽」時，模組多半還在把前 1~2 槽排進去的封包送上空中——
 *   空中完全沒有靜默，E22 半雙工在發射時不可能收到任何東西。實測 log 佐證：
 *     [LORA_TX_LOG] 433MHz tx ok=597/try=3826（84% 因 AUX 忙線被跳過），
 *     地面站連送 10 次 ARM（seq 6~15）全無 [ACK]、fsm 一直停在 1(PAD)。
 *   ★窗口必須用「時間」定義，而且長度要 > 一包的空中時間，模組才會真的把手上那包
 *     送完、進入靜默、聽得到上行幀。
 *
 * ★★2026-07-30 第二輪修正：只有「窗內不餵資料」還是不夠，上行仍然打不進來。實測
 *   佐證：把 LORA433_RX_ONLY 設 1（433 完全不發）之後 arm 立刻就通，設回 0 就不通
 *   ——RF 層、模組參數、天線、地面站發射全部無罪，問題 100% 在窗。
 *   根因是「窗的名目長度 != 真正的靜默長度」：listen_slot 只在迴圈開頭判斷一次，
 *   一包在 phase 2999 才起飛的下行封包，會讓模組一路發射到 phase ≈3300~3500，
 *   把窗的前半段整個吃掉：
 *     真正靜默 = HOLD - 在途封包剩餘空中時間 = 800 - 330~500 ≈ 300~470ms（最壞更短）
 *   而上行 7-byte 幀在 2.4k 下前導碼+表頭才是大頭，實際空中時間 ~80~150ms，且模組
 *   必須在「前導碼開始之前」就已經進 RX。地面站 UPLINK_TX_GAP_MS=150、burst 4500ms
 *   只橫跨 1.5 個窗 —— 能完整塞進那段真靜默的機會本來就趨近 0，實測就是 0。
 *
 *   ★解法：窗開始「之前」一個最大封包空中時間就停止餵 433（發射守衛帶），讓在途
 *     封包在窗開始前送完。這樣 HOLD 才是「保證」的靜默，而不是名目值：
 *       保證靜默 = GUARD - 最大空中時間 + HOLD = 600 - 500 + 400 = 500ms
 *     500ms 可穩穩容納一筆 80~150ms 的上行幀，且地面站每 150ms 一次的重送在窗內
 *     有 ~3 次機會。
 * 週期/長度取捨：不發射時間 = (GUARD + HOLD)/PERIOD = 1000/3000 ≈ 33%（舊版名目 27%
 * 但實際上行完全不通）。920 完全不受影響、維持全速率。
 * ⚠ 改動 LORA_TELEM_PERIOD_MS / LORA433_TX_EVERY / 封包大小 / 空中速率時，必須重算
 *   最大空中時間並確認 GUARD 仍然大於它，否則守衛帶失效、退回這次的失敗模式。 */
#define UPLINK_LISTEN_PERIOD_MS  3000U  /* 每 N ms 開一次上行接收窗 */
#define UPLINK_LISTEN_HOLD_MS     400U  /* 窗長（保證靜默的下限，見上方 GUARD 說明） */
/* 窗前發射守衛帶：須 > 一包下行的最大空中時間。
 * 目前 TELEM_PACKET_SIZE=99B @2.4k：99*8/2400 = 330ms 純資料，加 LoRa 前導/表頭/CRC
 * 實測上限約 500ms → 取 600ms 留餘裕。 */
#define UPLINK_TX_GUARD_MS        600U
void LoRaTelemetry_Task(void *argument)
{
    (void)argument;
    static uint8_t tx_buf[TELEM_PACKET_SIZE];
    uint32_t slot = 0;   /* 433 時槽計數（用於空出上行接收窗） */

    /* 讓開機自檢 / SD 掛載 / FlashRing 初始化先跑一段再開始發送（非必要，SPI3 mutex 已保護）。 */
    osDelay(2000);

    uint32_t wake = osKernelGetTickCount();
#if LORA433_ENABLE
    uint32_t e22_retry_tick = 0;
#endif
#if LORA920_ENABLE
    uint32_t e80_retry_tick = 0;
#endif
    for (;;) {
        if (g_lora_cfg_pause) {   /* 命令台正在改 LoRa 參數：本槽不發射（見旗標宣告處註解） */
            slot++;
            wake += LORA_TELEM_PERIOD_MS;
            osDelayUntil(wake);
            continue;
        }
        uint16_t len = Telemetry_Build(tx_buf);

#if LORA433_ENABLE
        /* P1：E22 初始化失敗時每 10s 重試一次（本任務最低優先，~305ms 阻塞無妨）。
         * LORA433_ENABLE=0 時整段編譯移除——停用後不重試、不復活 E22。 */
        if (!lora433_ok && (HAL_GetTick() - e22_retry_tick) >= 10000U) {
            e22_retry_tick = HAL_GetTick();
            if (LoRaE22_Init(&huart3) == HAL_OK) {
                lora433_ok = 1;
                printf("[LORA] E22 433MHz 重試成功，恢復下行。\r\n");
                LoRaE22_PrintConfig();
            }
        }
#endif

#if LORA920_ENABLE
        /* E80 920MHz (LR1121) 初始化失敗時每 10s 重試一次。
         * 若因快速 Reset 或餘電未放光導致開機瞬間檢測失敗，10s 後餘電放光即可重試復活。 */
        if (!lora920_ok && (HAL_GetTick() - e80_retry_tick) >= 10000U) {
            e80_retry_tick = HAL_GetTick();
#if IS_GROUND
            if (LoRaE80_Init(&hspi3) == HAL_OK && LoRaE80_StartRx() == HAL_OK) {
                lora920_ok = 1;
                printf("[LORA] E80 920MHz 重試成功，恢復連續接收。\r\n");
                LoRaE80_PrintConfig();
            }
#else
            if (LoRaE80_Init(&hspi3) == HAL_OK) {
                lora920_ok = 1;
                printf("[LORA] E80 920MHz 重試成功，恢復下行與連線。\r\n");
                LoRaE80_PrintConfig();
            }
#endif
        }
#endif

        HAL_StatusTypeDef st433 = HAL_BUSY;
        uint8_t tried433 = 0;
#if FEATURE_UPLINK_DEPLOY
        /* 上行接收窗：每 UPLINK_LISTEN_PERIOD_MS 有一段連續 UPLINK_TX_GUARD_MS +
         * UPLINK_LISTEN_HOLD_MS 完全不發 433（E22 透傳模式不發射時硬體上即在接收），
         * 讓地面站的上行命令有真正無干擾的空中時段可被收到。920 不受影響、維持全速率。
         * ★用 HAL_GetTick() 直接算相位而非數時槽：時槽長度會被 E80 發送/排程延遲拉長
         *   （實測 100ms 名目值實際約 180ms），數槽算出來的窗長會跟著漂，而窗長是否
         *   大於一包空中時間正是這個機制能不能成立的關鍵，不能讓它漂。
         * ★守衛帶（phase 落在窗「之前」GUARD 毫秒內也不發）是這個機制成立的關鍵：
         *   沒有它的話，一包在窗開始前一瞬間才起飛的下行封包會吃掉窗的前半段，名目
         *   800ms 的窗真正靜默只剩 ~300ms，上行實測完全打不進來。詳見 UPLINK_TX_GUARD_MS
         *   宣告處的完整記錄。 */
#if LORA433_RX_ONLY
        /* 純接收診斷模式（board_config.h::LORA433_RX_ONLY=1）：整段時間都當成接收窗，
         * 433 完全不發（下行遙測與下方的 ACK 都跳過），把「半雙工/窗開得夠不夠」這個
         * 變數整個拿掉，用來判斷上行收不到到底是窗的問題還是 RF 層的問題。 */
        uint8_t listen_slot = 1U;
#else
        uint32_t uplink_phase = HAL_GetTick() % UPLINK_LISTEN_PERIOD_MS;
        /* 窗內：phase < HOLD。守衛帶：phase 距離下一個窗起點不到 GUARD。
         * 兩段在時間軸上相連（… GUARD | 窗起點 | HOLD …），合計連續靜默 GUARD+HOLD。 */
        uint8_t listen_slot = (uplink_phase < UPLINK_LISTEN_HOLD_MS) ||
                              ((UPLINK_LISTEN_PERIOD_MS - uplink_phase) <= UPLINK_TX_GUARD_MS);
#endif
        uint8_t tx433_slot  = ((slot % LORA433_TX_EVERY) == 0U);   /* 降速：每 N 槽才發 433 */
        if (lora433_ok && !listen_slot && tx433_slot) {
            st433 = LoRaE22_Send(tx_buf, len);
            tried433 = 1;
        }
        UplinkCmd_Poll(HAL_GetTick());            /* 解析上行、處理 ARM/DEPLOY、ARM 逾時 */
#else
        if (lora433_ok && ((slot % LORA433_TX_EVERY) == 0U)) {   /* 降速：每 N 槽才發 433 */
            st433 = LoRaE22_Send(tx_buf, len);    /* 透傳：HAL_OK=已推入 UART→E22 */
            tried433 = 1;
        }
#endif
        /* 433 下行發送狀態 log（確認 MCU 確實把封包推給 E22 + AUX 腳狀態）。
         * AUX：HIGH=E22 空閒可收發；持續 LOW=E22 忙/異常。每 25 槽(~5s)印一次。 */
        {
            static uint32_t s_433_try = 0, s_433_ok = 0;
            if (tried433) { s_433_try++; if (st433 == HAL_OK) s_433_ok++; }
            if (slot % 25U == 0U) {
                printf("[LORA_TX_LOG] 433MHz tx ok=%lu/try=%lu len=%u AUX=%d last=%s\r\n",
                       (unsigned long)s_433_ok, (unsigned long)s_433_try, (unsigned)len,
                       (int)HAL_GPIO_ReadPin(LORA433_BUSY_GPIO_Port, LORA433_BUSY_Pin),
                       tried433 ? ((st433 == HAL_OK) ? "OK" : ((st433 == HAL_BUSY) ? "BUSY" : "ERR"))
                                : "skip(listen/off)");
            }
        }
        if (lora920_ok) {
            HAL_StatusTypeDef st920 = LoRaE80_Send(tx_buf, (uint8_t)len);
            static uint32_t s_920_tx_cnt = 0;
            if (st920 == HAL_OK) s_920_tx_cnt++;
            if (slot % 25 == 0) {
                printf("[LORA_TX_LOG] 920MHz Sent Pkts: %lu (Last Status: %s)\r\n",
                       (unsigned long)s_920_tx_cnt,
                       (st920 == HAL_OK) ? "OK" : ((st920 == HAL_BUSY) ? "BUSY" : "ERR"));
            }
        }

#if FEATURE_UPLINK_DEPLOY
        /* === 命令 ACK 下行（主航電）====================================================
         * 診斷任務執行完上行命令後經 UplinkCmd_SetAck 暫存一筆 ACK；此處取出並組 ack_proto 幀，
         * 於接下來數個時槽經 433(E22)+920(E80) 各重送 ACK_TX_REPEAT 次（滿足「多傳幾次」，
         * 對抗空中誤碼/漏收）。ACK 幀為單一連續位元組序列、與遙測封包在本任務內序列化送出，
         * 不會 byte 交錯（地面站以 AckRx 於同一位元組流並排解析，sync 0xAC/0xCA 與遙測區隔）。 */
        {
            #define ACK_TX_REPEAT  4U
            static uint8_t  s_ack_frame[ACK_FRAME_MAX];
            static uint8_t  s_ack_frame_len = 0;
            static uint8_t  s_ack_repeat = 0;
            if (s_ack_repeat == 0U) {
                uint8_t aseq = 0, astatus = 0, alen = 0;
                char    atext[ACK_TEXT_MAX + 1];
                if (UplinkCmd_TakePendingAck(&aseq, &astatus, atext, &alen)) {
                    s_ack_frame_len = AckProto_Build(s_ack_frame, aseq, astatus, atext, alen);
                    s_ack_repeat    = ACK_TX_REPEAT;
                    printf("[ACK_TX] seq=%u status=%s cmd=\"%s\" ×%u\r\n",
                           (unsigned)aseq, ack_status_str(astatus), atext, (unsigned)ACK_TX_REPEAT);
                }
            }
            if (s_ack_repeat > 0U && s_ack_frame_len > 0U) {
                /* ★重送次數只在「真的送出去」時才遞減。舊版無條件 s_ack_repeat--，但
                 * LoRaE22_Send 在 AUX 忙線時是直接回 HAL_BUSY 完全不發（實測 84% 的
                 * 呼叫都是這樣），等於 4 次重送常常一次都沒上空中，地面站自然收不到
                 * [ACK]；「靠重送命中」的前提從一開始就不成立。
                 * 433 同時避開上行接收窗：ACK 也是下行發射，在窗內送會把剛開好的窗吃掉。
                 * s_ack_tries 是防呆上限——兩條鏈路都持續忙線時不至於無限重試卡住。 */
                static uint16_t s_ack_tries = 0;
                HAL_StatusTypeDef ack433 = HAL_BUSY, ack920 = HAL_BUSY;
                if (lora433_ok && !listen_slot) ack433 = LoRaE22_Send(s_ack_frame, s_ack_frame_len);
                if (lora920_ok)                 ack920 = LoRaE80_Send(s_ack_frame, s_ack_frame_len);
                s_ack_tries++;
                if (ack433 == HAL_OK || ack920 == HAL_OK) {
                    s_ack_repeat--;
                    s_ack_tries = 0;
                } else if (s_ack_tries >= 100U) {   /* ~10s 都推不出去：放棄本筆，別卡住下一筆 */
                    printf("[ACK_TX] ⚠ 兩條鏈路持續忙線，放棄重送（剩 %u 次）\r\n",
                           (unsigned)s_ack_repeat);
                    s_ack_repeat = 0;
                    s_ack_tries  = 0;
                }
            }
        }
#endif /* FEATURE_UPLINK_DEPLOY */

        slot++;
        wake += LORA_TELEM_PERIOD_MS;
        osDelayUntil(wake);
    }
}

#if defined(ENABLE_DIAGNOSTICS) && !IS_GROUND
void Poll_Serial_Commands(void);
void Parse_Serial_Command(const char* cmd);
uint8_t Parse_GetLastStatus(void);   /* Parse_Serial_Command 最後結果（ACK_*，見定義處） */

void StartDiagnosticTask(void *argument)
{
  (void)argument;
  uint32_t tick = 0;
  uint32_t wake_tick = osKernelGetTickCount();
  for (;;)
  {
#if !FEATURE_LINK
      Poll_Serial_Commands();   /* USART2 文字命令台；FEATURE_LINK 時 USART2 改作板間鏈路 */
#elif !IS_GROUND && FEATURE_USB_CDC
      /* FEATURE_LINK 開啟時 UART2 作板間鏈路，但 USB CDC 是獨立通道，仍可接受命令。
       * ⚠ 這裡「不分飛行狀態」一律受理：Parse_Serial_Command 內含 disarm、手動開傘等
       *   危險指令，目前只有 bench 點火自帶 STATE_PAD/INIT 閘，其餘沒有飛行態防護。
       *   待補：對危險指令加同樣的飛行態閘（見 board_config.h FEATURE_USB_DEBUG_LOG）。 */
      {
          static char s_usb_cmd_buf[128];
          static uint8_t s_usb_cmd_idx = 0;
          uint8_t b;
          for (int g = 0; g < 128; g++) {
              if (!GsLoraTest_PopUsbByte(&b)) break;
              if (b == '\n' || b == '\r') {
                  if (s_usb_cmd_idx > 0) {
                      s_usb_cmd_buf[s_usb_cmd_idx] = '\0';
                      Parse_Serial_Command(s_usb_cmd_buf);
                      s_usb_cmd_idx = 0;
                  }
              } else if (s_usb_cmd_idx < sizeof(s_usb_cmd_buf) - 1U) {
                  s_usb_cmd_buf[s_usb_cmd_idx++] = (char)b;
              } else {
                  s_usb_cmd_idx = 0;
              }
          }
      }
#endif

#if FEATURE_UPLINK_DEPLOY
      /* === 遠端 433 上行命令執行（主航電）===================================
       * 文字命令：餵 Parse_Serial_Command（複用整組本地命令：mag 校正/e22/e80/role…），
       *   結果經 Parse_GetLastStatus 取回 → SetAck 回地面站。指令執行的 printf 同時走火箭
       *   自身 USB CDC（FEATURE_USB_DEBUG_LOG 開啟時），即「ack 給 usb」。
       * bench（桌面測試）：已過 uplink_cmd 的 ARM 閘；此處再加「僅限未起飛」閘
       *   （STATE_PAD/INIT）——序列含 PD13 點火頭導通 8s + PD14 拉高，飛行中觸發會災難性。
       *   通過 → 跑一次 PyroSelfTest_RunSequence()（阻塞 ~25s，自身餵狗；飛控 1kHz 任務照常
       *   搶佔），返回後 Servo_HoldLow() 復位 PD14 為部署前安全低態，回歸正常 FSM，回 ACK。 */
      {
          char     ucmd[UPLINK_TEXT_MAX + 1];
          uint8_t  useq = 0;
          if (UplinkCmd_TakeTextCmd(ucmd, sizeof(ucmd), &useq)) {
              Parse_Serial_Command(ucmd);
              UplinkCmd_SetAck(useq, Parse_GetLastStatus(), ucmd);
          }
          uint8_t bseq = 0;
          if (UplinkCmd_TakeBench(&bseq)) {
              FlightState_t st = (FlightState_t)current_fsm_state;
              if (st == STATE_PAD_ARMED) {
                  printf("[BENCH] 遠端桌面測試觸發（已過 ARM 閘）→ 跑一次 pyro/servo 序列\r\n");
                  PyroSelfTest_RunSequence();
                  Servo_HoldLow();   /* 復位 PD14 為部署前安全低態（無訊號） */
                  printf("[BENCH] 序列完成，已回歸正常 FSM。\r\n");
                  UplinkCmd_SetAck(bseq, ACK_OK, "bench");
              } else {
                  printf("[BENCH] 拒絕：未解鎖（fsm=%u），BENCH 測試僅限在 ARM (STATE_PAD_ARMED) 狀態下執行。\r\n",
                         (unsigned)current_fsm_state);
                  UplinkCmd_SetAck(bseq, ACK_UNARMED, "bench");
              }
          }
      }
#endif

#if FEATURE_LINK
      /* 副板：當處於 ARM 解鎖狀態且感測到主板發起 BENCH 測試 (BENCH_START / BENCH_PRI_FIRE) 時自動跟隨進入自測。
       * 注意：此區塊不可掛在 FEATURE_UPLINK_DEPLOY(=IS_PRIMARY) 底下——那樣副板編譯時會被前處理器整段
       * 拿掉，IS_BACKUP 判斷式永遠不存在，副板就完全不會自動跟隨（曾發生過的 bug：peer_arb 卡在 NONE、
       * 副板從未進入 bench 序列）。獨立掛在 FEATURE_LINK 底下，主/備兩板都會編譯進去，主板上 IS_BACKUP
       * 恆為假、純粹跳過，無副作用。 */
      if (IS_BACKUP && (FlightState_t)current_fsm_state == STATE_PAD_ARMED) {
          const LinkPeer_t *peer = Link_GetPeer();
          if (peer->valid && Link_PeerFresh(HAL_GetTick())) {
              if (peer->peer_main_arb == SERVO_ARB_MSG_BENCH_PRI_FIRE ||
                  peer->peer_main_arb == SERVO_ARB_MSG_BENCH_START) {
                  printf("[BENCH] 板間鏈路偵測到主板 BENCH 測試觸發 → 副板自動跟隨啟動自測序列\r\n");
                  PyroSelfTest_RunSequence_Ex(1);
                  Servo_HoldLow();
                  printf("[BENCH] 協同自測完成，已回歸正常 FSM。\r\n");
              }
          }
      }
#endif

      if (tick % 50 == 0) {
          // 0. 本板 FSM 狀態廣播（1Hz）：供 GUI 頂部 STATE 標籤即時更新
          printf("[FSM] state=%s role=%s\r\n",
                 link_fsm_state_name((uint8_t)current_fsm_state),
                 IS_BACKUP ? "BACKUP" : "PRIMARY");

          // 1. GPS 遙測
          const GPS_Data_t *g = GPS_GetData();
          char     lat_sign = (g->lat_1e6 < 0) ? '-' : '+';
          char     lon_sign = (g->lon_1e6 < 0) ? '-' : '+';
          uint32_t lat_abs  = (g->lat_1e6 < 0) ? (uint32_t)(-g->lat_1e6) : (uint32_t)g->lat_1e6;
          uint32_t lon_abs  = (g->lon_1e6 < 0) ? (uint32_t)(-g->lon_1e6) : (uint32_t)g->lon_1e6;
          printf("[GPS] fix:%d q:%d sat:%d %c%lu.%06lu,%c%lu.%06lu alt:%dm spd:%dcm/s stale:%d ok:%lu err:%lu\r\n",
                 (int)g->fix_valid,
                 (int)g->fix_quality,
                 (int)g->satellites,
                 lat_sign, (unsigned long)(lat_abs / 1000000U), (unsigned long)(lat_abs % 1000000U),
                 lon_sign, (unsigned long)(lon_abs / 1000000U), (unsigned long)(lon_abs % 1000000U),
                 (int)g->altitude_m,
                 (int)(g->speed_mps * 100.0f),
                 (int)GPS_IsStale(2000),
                 (unsigned long)g->sentences_ok,
                 (unsigned long)g->sentences_err);

#if FEATURE_LINK
          /* 2. 板間鏈路溝通狀態（1Hz）：自身角色/對端角色/鏈路是否新鮮/對端最近回報的
           * FSM 狀態與旗標/距上次收包時間。GUI 監控（USB）據此顯示主備兩板的溝通狀態；
           * 對稱獨立冗餘設計下鏈路不再仲裁點火（各板獨立判斷），純供人工監看/事後判讀。 */
          {
              uint32_t now_link = HAL_GetTick();
              const LinkPeer_t *peer = Link_GetPeer();
              uint8_t fresh = Link_PeerFresh(now_link);
              const char *link_state = !peer->valid ? "NONE" : (fresh ? "OK" : "STALE");
              uint8_t lstat = Link_GetStatus();
              /* primary/backup 用絕對角色標籤，主航電 USB-CDC 一眼對上兩板 erase 進度 */
              {
                  uint8_t self_pct = FlashRing_GetErasePct();
                  uint8_t peer_pct = peer->valid ? peer->peer_erase_pct : 0U;
                  uint8_t primary_pct = IS_BACKUP ? peer_pct : self_pct;
                  uint8_t backup_pct  = IS_BACKUP ? self_pct : peer_pct;
                  printf("[LINK] self=%s peer=%s link=%s state=%s flags=0x%02X age=%lums "
                         "sync=%s lost=%u desync=%u self_arb=%s peer_arb=%s peer_flash=%s "
                         "primary_erase=%u%% backup_erase=%u%% ph_cm=%ld pv_cms=%ld "
                         "pbaro_cm=%ld paz_cg=%d pvfh_cm=%ld pvfv_cms=%ld "
                         "pbmi_cg=%d padxl_cg=%d\r\n",
                         IS_BACKUP ? "BACKUP" : "PRIMARY",
                         !peer->valid ? "NONE" : ((peer->board_id == LINK_BOARD_BACKUP) ? "BACKUP" : "PRIMARY"),
                         link_state,
                         link_fsm_state_name(peer->fsm_state),
                         peer->flags,
                         (unsigned long)(peer->valid ? (now_link - peer->last_rx_ms) : 0U),
                         LinkPeer_Synced(peer, (uint8_t)current_fsm_state) ? "OK" : "NO",
                         (unsigned)((lstat & LINK_STATUS_LOST) != 0U),
                         (unsigned)((lstat & LINK_STATUS_DESYNC) != 0U),
                         ServoArb_MsgName(g_servo_arb_bcast),
                         ServoArb_MsgName(peer->peer_main_arb),
                         peer->peer_flash_ready ? "READY" : "WAIT",
                         (unsigned)primary_pct,
                         (unsigned)backup_pct,
                         /* 對端最近回報的 EKF 高度/垂直速度/baro 相對高度/加速度/VF 高度速度：
                          * LinkPeer_t 本來就經板間鏈路 20Hz(LINK_TX_PERIOD_MS) 持續更新這些欄位
                          * （見 link_proto.c LinkStatus 封包），這裡只是把已經在收的資料印出來，
                          * 讓直連單板序列埠的 bench 圖表不必透過地面站 LoRa 轉發也能畫出完整
                          * 副航電高度/速度/加速度/VF 曲線；peer 無效時維持 0（與其餘 peer 欄位
                          * 一致的「無資料」慣例）。 */
                         (long)peer->h_est_cm,
                         (long)peer->v_est_cms,
                         (long)peer->baro_alt_cm,
                         (int)peer->a_z_cg,
                         (long)peer->vf_h_cm,
                         (long)peer->vf_v_cms,
                         (int)peer->bmi_mag_cg,
                         (int)peer->adxl_mag_cg);
              }
          }
#endif

#ifdef RATE_MONITOR_ENABLE
          // 3. 採樣率遙測
          float r_bmi_a = g_sampling_rate.bmi088_acc.rate_hz;
          float r_bmi_g = g_sampling_rate.bmi088_gyro.rate_hz;
          float r_adxl  = g_sampling_rate.adxl375.rate_hz;
          float r_bmp   = g_sampling_rate.bmp388.rate_hz;
          float r_mmc   = g_sampling_rate.mmc5983.rate_hz;
          float r_gps   = g_sampling_rate.gps.rate_hz;

          printf("[RATE] BMI088_A:%d.%02uHz, BMI088_G:%d.%02uHz, ADXL375:%d.%02uHz, BMP388:%d.%02uHz, MMC5983:%d.%02uHz, GPS:%d.%02uHz, SD_DET:%d, EKF_DROP:%lu\r\n",
                 (int)r_bmi_a, (unsigned int)((r_bmi_a - (int)r_bmi_a) * 100.0f),
                 (int)r_bmi_g, (unsigned int)((r_bmi_g - (int)r_bmi_g) * 100.0f),
                 (int)r_adxl,  (unsigned int)((r_adxl  - (int)r_adxl)  * 100.0f),
                 (int)r_bmp,   (unsigned int)((r_bmp   - (int)r_bmp)   * 100.0f),
                 (int)r_mmc,   (unsigned int)((r_mmc   - (int)r_mmc)   * 100.0f),
                 (int)r_gps,   (unsigned int)((r_gps   - (int)r_gps)   * 100.0f),
                 (int)HAL_GPIO_ReadPin(SDIO_DET_GPIO_Port, SDIO_DET_Pin),
                 (unsigned long)g_ekf_queue_drops);

          // 4. W25Q128 Flash 記錄遙測
          printf("[FLASH_RING] PKT_TOTAL:%lu ADDR:0x%06lX\r\n",
                 FlashRing_GetPacketCount(), FlashRing_GetWriteAddr());

          // 5. CPU 佔用率遙測
          float r_ekf_cpu = EKF_GetCPUUsage();
          printf("[CPU] MainTask+ISR:%d.%02u%%, EKFTask:%d.%02u%%\r\n",
                 (int)g_main_task_cpu_usage, (unsigned int)((g_main_task_cpu_usage - (int)g_main_task_cpu_usage) * 100.0f),
                 (int)r_ekf_cpu, (unsigned int)((r_ekf_cpu - (int)r_ekf_cpu) * 100.0f));
#endif

          // 6. IMU & High-G 原始數據用於方向對齊驗證
          printf("[IMU] a[mG]:%d,%d,%d g[dps]:%d,%d,%d\r\n",
                 (int)(imu_data.ax * 1000.0f),
                 (int)(imu_data.ay * 1000.0f),
                 (int)(imu_data.az * 1000.0f),
                 (int)(imu_data.gx),
                 (int)(imu_data.gy),
                 (int)(imu_data.gz));
          if (adxl375_ok) {
              printf("[HIGHG] a[mG]:%d,%d,%d\r\n",
                     (int)(highg_data.ax * 1000.0f),
                     (int)(highg_data.ay * 1000.0f),
                     (int)(highg_data.az * 1000.0f));
          }

          // 7. 電源監測遙測（REQ-SYS-01：板載 ADC 電池電壓採樣 + 遙測顯示）
          printf("[PWR] bat:%dmV\r\n", (int)g_bat_voltage_mv);

          // 8. LoRa 下行鏈路就緒狀態（433 透傳 / 920 SX126x）+ E80 init 診斷
          {
              int    e80_rd; uint8_t e80_busy, e80_rb0, e80_rb1, e80_gs;
              LoRaE80_GetInitDiag(&e80_rd, &e80_busy, &e80_rb0, &e80_rb1, &e80_gs);
              /* gs：LR1121 Stat1 = (CmdStatus<<1)|IrqPending，CmdStatus 2=OK（故 0x04/0x05 正常）;
               * gs=0x00 → MISO接地; gs=0xFF → MISO浮空。
               * err：GetErrors 旗標，0x0000=射頻時鐘/校準全正常；bit5(0x20)=HF_XOSC 起振失敗、
               * bit7(0x80)=PLL 鎖不上 → TCXO 供電/電壓不對，射頻不會動（見 lora_e80.h）。 */
              printf("[LORA] 433:%s 920:%s | gs=0x%02X busy=%d rd=%d sw=0x%02X,0x%02X err=0x%04X tcxo=0x%02X\r\n",
                     lora433_ok ? "rdy" : "off",
                     lora920_ok ? "rdy" : "off",
                     e80_gs, (int)e80_busy, e80_rd, e80_rb0, e80_rb1,
                     (unsigned)LoRaE80_GetErrors(),
                     (unsigned)LoRaE80_GetTcxoTune());
          }

#if FEATURE_UPLINK_DEPLOY
          /* 9. 上行（地面站 → 火箭）接收狀態。★先前完全沒有這行：地面站按 ARM 沒反應時
           * 無從判斷是「沒發出來」「沒飛到」還是「收到但被閘擋掉」，只能盲猜。
           * 判讀：raw=0 → 射頻層什麼都沒進來（地面站沒真的發射／頻道空速不符／本機一直在發射
           *              沒讓出接收窗）；raw↑ ok=0 crc↑ → 有訊號但幀壞（誤碼或被自己的發射截斷）；
           *              ok↑ → 命令確實收到，接著看 [UPLINK] 那幾行的 ARM/狀態閘結果。 */
          {
              UplinkCmdStats_t us;
              UplinkCmd_GetStats(&us);
              printf("[UPLINK_STAT] raw=%lu bin_ok=%lu bin_crc=%lu bin_rsync=%lu txt_ok=%lu txt_crc=%lu last=0x%02X armed=%u\r\n",
                     (unsigned long)us.raw_bytes, (unsigned long)us.bin_ok,
                     (unsigned long)us.bin_crc_err, (unsigned long)us.bin_resync,
                     (unsigned long)us.text_ok, (unsigned long)us.text_crc_err,
                     (unsigned)us.last_cmd, (unsigned)UplinkCmd_IsArmed());
          }
#endif
      }

      /* 地磁計遙測（獨立頻率，跟外層 GPS 等診斷脫鉤）：平常 1Hz（tick%50==0，
       * 20ms/次任務 tick），CMD_MAG_CAL_START 期間提速到 10Hz（tick%5，每 100ms
       * 印一行）供硬鐵校正快速收集樣本；MMC5983 讀取本就 100Hz，故 10Hz 印出
       * 不會取到重複值，且留出 CDC TX 餘裕給命令 ACK 回覆。
       * auto-stop：倒數歸零自動回 1Hz，避免 STOP 被吃掉而永遠卡在高頻。 */
      if (g_mag_cal_fast_ticks > 0U) g_mag_cal_fast_ticks--;
      if (mag_ok && (tick % (g_mag_cal_fast_ticks > 0U ? 5U : 50U) == 0)) {
          const MMC5983_Data_t *m = MMC5983_GetData();
          // 重映射至 body frame (sensor_axis.h)：mag X->-Y, Y->X, Z->-Z
          float mx_body, my_body, mz_body;
          sensor_mag_to_body(m->gauss[0], m->gauss[1], m->gauss[2], &mx_body, &my_body, &mz_body);
          float hdg_body = atan2f(-mx_body, my_body) * (180.0f / 3.14159265f);
          if (hdg_body < 0.0f) hdg_body += 360.0f;
          printf("[MAG] B[mG]:%d,%d,%d hdg:%d ok:%lu err:%lu\r\n",
                 (int)(mx_body * 1000.0f),
                 (int)(my_body * 1000.0f),
                 (int)(mz_body * 1000.0f),
                 (int)hdg_body,
                 (unsigned long)m->reads_ok,
                 (unsigned long)m->reads_err);
      }

      tick++;
      wake_tick += 20U;
      osDelayUntil(wake_tick);
  }
}
#endif

#define CMD_BUFFER_SIZE 64
static char g_cmd_buf[CMD_BUFFER_SIZE];
static uint8_t g_cmd_idx = 0;

/* ============================================================
 *  航電命令台（USART2）：LoRa 執行期參數調整
 *  命令語法/輸出格式刻意與地面站 gs_lora_test.c 一致，GUI 對三角色同一介面。
 * ============================================================ */

#if FEATURE_LORA
/* 暫停遙測發送 → 確認模組真的閒下來（AUX HIGH）→ 執行 LoRa 設定 → 恢復。
 * ★先前只 osDelay(300)（> E22_Send 最大阻塞 100ms + E80 單筆 SPI 序列），這只保證
 *   「MCU 端已經把封包灌完 UART」，不保證「模組已經把上一包真的發射出空中」——LoRa
 *   實際 airtime（含 preamble/同步字）可能明顯長於 MCU→模組的 UART 交遞時間，尤其
 *   air rate 較低時。若這裡沒等到，`e22_write_channel()` 一開始的 M1=SET 就可能撞上
 *   模組仍在忙線(AUX=LOW)發射中，M1 腳位這時候被切換，datasheet 沒定義行為，
 *   結果就是「e22 set 有時可以有時不行」——取決於下指令當下剛好落在模組忙線窗口
 *   內外的時序運氣。改成等 AUX 真正拉高(模組閒置)才放行；逾時仍放行，避免模組真的
 *   故障/沒接時命令台永久卡死。 */
static void cmd_lora_pause(void)
{
    g_lora_cfg_pause = 1;
    osDelay(300);
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LORA433_BUSY_GPIO_Port, LORA433_BUSY_Pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t0) > 1000U) break;
        osDelay(5);
    }
}
static void cmd_lora_resume(void) { g_lora_cfg_pause = 0; }

#if FEATURE_FLASH
static void Save_Lora_Params_To_Flash(void)
{
    FlashSysFlags_t sys_flags;
    if (Flash_ReadSysFlags(&sys_flags) != W25QXX_OK) {
        memset(&sys_flags, 0, sizeof(FlashSysFlags_t));
    }
    
    uint32_t e22_freq = 0;
    uint8_t e22_pwr = 0, e22_air = 0;
    LoRaE22_GetParams(&e22_freq, &e22_pwr, &e22_air);
    
    uint32_t e80_freq = 0;
    uint8_t e80_sf = 0, e80_bw = 0, e80_cr = 0;
    int8_t e80_pwr = 0;
    uint16_t e80_pre = 0;
    LoRaE80_GetParams(&e80_freq, &e80_sf, &e80_bw, &e80_cr, &e80_pwr, &e80_pre);
    
    sys_flags.calib.magic = 0xC0DEB1A5;
    sys_flags.lora.magic = 0x50544c4f; // 'PTLO'
    sys_flags.lora.e22_freq_mhz = e22_freq;
    sys_flags.lora.e22_pwr = e22_pwr;
    sys_flags.lora.e22_air = e22_air;
    sys_flags.lora.e80_freq_hz = e80_freq;
    sys_flags.lora.e80_sf = e80_sf;
    sys_flags.lora.e80_bw = e80_bw;
    sys_flags.lora.e80_cr = e80_cr;
    sys_flags.lora.e80_pwr_dbm = e80_pwr;
    sys_flags.lora.e80_preamble = e80_pre;
    
    if (Flash_WriteSysFlags(&sys_flags) == W25QXX_OK) {
        printf("[LORA] Parameters Saved to Flash!\r\n");
    } else {
        printf("[LORA] ERROR: Failed to save LoRa parameters to Flash!\r\n");
    }
}
#endif


static void cmd_print_e80_params(void)
{
    uint32_t f = 0; uint8_t sf = 0, bw = 0, cr = 0; int8_t pwr = 0; uint16_t pre = 0;
    LoRaE80_GetParams(&f, &sf, &bw, &cr, &pwr, &pre);
    /* 與 gs_lora_test.c print_e80_params 同格式（GUI 解析共用） */
    printf("[E80] freq=%lu Hz  SF%u  BW%lu kHz (idx=%u)  CR 4/%u  pwr=%d dBm  pre=%u  ldro=%u\r\n",
           (unsigned long)f, (unsigned)sf,
           (unsigned long)lora_bw_to_khz(bw), (unsigned)bw,
           (unsigned)(cr + 4U), (int)pwr,
           (unsigned)pre, (unsigned)lora_ldro_required(sf, bw));
}

/* 讀現值 → 換掉一個欄位 → 整組重配置（LoRaE80_Reconfig 一次收全部參數） */
static void cmd_e80_reconfig(uint32_t f, uint8_t sf, uint8_t bw,
                             uint8_t cr, int8_t pwr, uint16_t pre)
{
    cmd_lora_pause();
    HAL_StatusTypeDef st = LoRaE80_Reconfig(f, sf, bw, cr, pwr, pre);
    cmd_lora_resume();
    cmd_print_e80_params();
    if (st == HAL_OK) {
        printf("[E80] reconfig OK\r\n");
#if FEATURE_FLASH
        Save_Lora_Params_To_Flash();
#endif
    }
    else              printf("[E80] reconfig FAIL (st=%d)\r\n", (int)st);
}
#endif /* FEATURE_LORA */

static void cmd_print_help(void)
{
    printf("\r\n=== 航電命令台（USART2 460800, 換行結尾）===\r\n"
           "  role                 回報角色/版本（GUI 自動偵測用）\r\n"
           "  help                 顯示此說明\r\n"
#if FEATURE_LORA
           "  e22 show             顯示 E22 433 模組設定\r\n"
           "  e22 freq <mhz>       設 E22 頻率 410-493 MHz\r\n"
           "  e22 pwr  <0-3>       設 E22 功率 0=30 1=27 2=24 3=21dBm（3V3 供電建議 3）\r\n"
           "  e22 air  <0-7>       設 E22 空速 0=0.3k 1=1.2k 2=2.4k ... 7=62.5k（兩端須一致）\r\n"
           "  e80 show             顯示 E80 920 RF 參數\r\n"
           "  e80 freq <hz>        設 E80 頻率 Hz（建議 862-928MHz，天線/濾波器調 920）\r\n"
           "  e80 sf   <7-12>      設 E80 展頻因子\r\n"
           "  e80 bw   <idx>       設 E80 頻寬 3=62.5k 4=125k 5=250k 6=500k\r\n"
           "  e80 cr   <1-4>       設 E80 編碼率 1=4/5 2=4/6 3=4/7 4=4/8\r\n"
           "  e80 pwr  <-9..22>    設 E80 發射功率 dBm\r\n"
           "  e80 pre  <6-65535>   設 E80 前導碼符號數\r\n"
#else
           "  （BACKUP 無 LoRa 硬體，無 e22/e80 命令）\r\n"
#endif
           "  flash erase [ring]   只清 Ring Buffer 飛行紀錄（保留校準/mag/LoRa/總結）★飛前正規流程，~3min\r\n"
           "  flash erase all      清整顆 Flash（含校準/總結/紀錄，需重新校正）\r\n"
           "  flash pool           快速填池：只補到 ARM 門檻，不動整環（bench 省時用）\r\n"
           "  flash export         格式化匯出 Flash 數據為 CSV 串流\r\n"
           "  CMD_MAG_CAL:x,y,z（整數 raw ADC counts，非 mG/Gauss）/ CMD_MAG_YAW_LOCK:0|1 / CMD_RESET_CAL（磁力計校正）\r\n"
           "  CMD_MAG_CAL_START / CMD_MAG_CAL_STOP（[MAG] 1Hz<->50Hz，供硬鐵校正腳本快速收集樣本）\r\n"
           "  arm / disarm         武裝解鎖/解除（PAD<->PAD_ARMED）\r\n"
           "  bench                桌面開傘自測（引傘 PD13 + 主傘 PD14；須先 arm，僅限 STATE_PAD_ARMED）\r\n"
           "  deploy drogue|main|both  手動開傘（須先 arm，或已在飛行中）\r\n"
           "  recalib              EKF 重新校準（須先 arm，僅限未起飛）\r\n"
           "  recovery             尋回指令：停蜂鳴器+安全關閉 SD/Flash 記錄（僅限 STATE_LANDED）\r\n"
           "===========================================\r\n");
}

/* 最後一筆命令解析結果（ACK_OK/ACK_BADARG/ACK_UNKNOWN，見 ack_proto.h）：Parse_Serial_Command
 * 進入時預設 OK，僅失敗分支覆寫；供遠端 433 文字命令的 ACK 判斷（診斷任務讀取）。 */
static uint8_t g_parse_status = ACK_OK;
uint8_t Parse_GetLastStatus(void) { return g_parse_status; }

void Parse_Serial_Command(const char* cmd) {
    g_parse_status = ACK_OK;
    if (strncmp(cmd, "CMD_MAG_CAL:", 12) == 0) {
        /* 整數 raw ADC counts（非浮點）：nano.specs 連結不支援浮點 sscanf/printf，
         * %f 這裡會靜默解析失敗（同一根因見昨天修的 8 處 printf("%f")）。硬鐵
         * 校正的 offset 本就是 ADC 原始計數量級（~131072，2^17 中點），整數精度
         * 已足夠，不需要小數。 */
        long cx = 0, cy = 0, cz = 0;
        if (sscanf(cmd + 12, "%ld,%ld,%ld", &cx, &cy, &cz) == 3) {
            EKF_SaveMagCalibration((float)cx, (float)cy, (float)cz);
        } else {
            printf("[CAL] ERROR: Invalid mag cal command format (expect integer raw ADC counts)\r\n");
            g_parse_status = ACK_BADARG;
        }
        printf("[ACK] status:%s cmd:\"CMD_MAG_CAL\"\r\n", ack_status_str(g_parse_status));
        return;
    }
    else if (strncmp(cmd, "CMD_MAG_YAW_LOCK:", 17) == 0) {
        int val = 0;
        if (sscanf(cmd + 17, "%d", &val) == 1) {
            g_mag_yaw_lock = (val != 0) ? 1 : 0;
            printf("[CAL] EKF Mag Yaw Lock set to: %d\r\n", g_mag_yaw_lock);
        } else {
            g_parse_status = ACK_BADARG;
        }
        printf("[ACK] status:%s cmd:\"CMD_MAG_YAW_LOCK\"\r\n", ack_status_str(g_parse_status));
        return;
    }
    else if (strcmp(cmd, "CMD_RESET_CAL") == 0) {
        EKF_ResetCalibration();
        printf("[ACK] status:OK cmd:\"CMD_RESET_CAL\"\r\n");
        return;
    }
    else if (strcmp(cmd, "CMD_MAG_CAL_START") == 0) {
        g_mag_cal_fast_ticks = 3000U;  /* 60s × 50ticks/s；到期自動回 1Hz */
        printf("[CAL] Mag calibration fast-print ON (10Hz, 60s guard)\r\n");
        printf("[ACK] status:OK cmd:\"CMD_MAG_CAL_START\"\r\n");
        return;
    }
    else if (strcmp(cmd, "CMD_MAG_CAL_STOP") == 0) {
        g_mag_cal_fast_ticks = 0U;     /* 立即恢復 1Hz */
        printf("[CAL] Mag calibration fast-print OFF (1Hz)\r\n");
        printf("[ACK] status:OK cmd:\"CMD_MAG_CAL_STOP\"\r\n");
        return;
    }

    /* --- 小寫 token 命令（與地面站 gs_lora_test.c 同語法） --- */
    char buf[CMD_BUFFER_SIZE];
    strncpy(buf, cmd, sizeof(buf) - 1U);
    buf[sizeof(buf) - 1U] = '\0';
    for (char *q = buf; *q; q++) *q = (char)tolower((unsigned char)*q);

    char *tok[3] = { NULL, NULL, NULL };
    int   n = 0;
    char *p = buf;
    while (*p && n < 3) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }
    if (n == 0) return;

    if (strcmp(tok[0], "role") == 0) {
        printf("[ROLE_ID] role=%s fw=%s git=%s build=%s\r\n",
               IS_BACKUP ? "BACKUP" : "PRIMARY", FIRMWARE_VERSION, FW_GIT, FW_BUILD);

    } else if (strcmp(tok[0], "arm") == 0) {
        FlightState_t st = (FlightState_t)current_fsm_state;
        if (g_flash_erase_in_progress) {
            printf("[ARM] REJECTED: Flash erase in progress, try again after it finishes\r\n");
            g_parse_status = ACK_REJECTED;
        } else if (st == STATE_INIT || st == STATE_PAD) {
#if FEATURE_UPLINK_DEPLOY
            UplinkCmd_SetArmedState(1);
#endif
            FlashFlight_EnsureActive();
            FSM_SetState(&g_fsm_ctx, STATE_PAD_ARMED);
            /* 下鏈滾動極值歸零：ARM 即本次飛行的起算點（見 ExtremaTrack_Reset） */
            ExtremaTrack_Reset();
            printf("[ARM] SUCCESS: Switched to STATE_PAD_ARMED (Ready for Liftoff)\r\n");
        } else {
            printf("[ARM] WARNING: Cannot ARM from current state (fsm=%u)\r\n", (unsigned)st);
            g_parse_status = ACK_REJECTED;
        }
    } else if (strcmp(tok[0], "disarm") == 0) {
        FlightState_t st = (FlightState_t)current_fsm_state;
        if (st == STATE_PAD_ARMED || st == STATE_INIT) {
#if FEATURE_UPLINK_DEPLOY
            UplinkCmd_SetArmedState(0);
#endif
            FlashFlight_ClearIfPrelaunch();
            FSM_SetState(&g_fsm_ctx, STATE_PAD);
            printf("[ARM] SUCCESS: Disarmed to STATE_PAD\r\n");
        } else {
            printf("[ARM] WARNING: Cannot DISARM from flight state (fsm=%u)\r\n", (unsigned)st);
            g_parse_status = ACK_REJECTED;
        }
    } else if (strcmp(tok[0], "bench") == 0) {
#if PYRO_SELFTEST_AVAILABLE
        FlightState_t st = (FlightState_t)current_fsm_state;
        if (st != STATE_PAD_ARMED) {
            printf("[BENCH] 拒絕：未解鎖（fsm=%u），BENCH 測試僅限在 ARM (STATE_PAD_ARMED) 狀態下執行，請先送出 ARM 命令！\r\n", (unsigned)st);
            g_parse_status = ACK_UNARMED;
        } else {
            printf("[BENCH] 桌面測試觸發 (ARMED) -> 跑一次 pyro/servo 自測序列 (~25s)\r\n");
            PyroSelfTest_RunSequence();
            Servo_HoldLow();   /* 復位 PD14 為部署前安全低態（無訊號） */
            printf("[BENCH] 桌面測試完成，已回歸正常 FSM。\r\n");
            g_parse_status = ACK_OK;
        }
#else
        printf("[BENCH] 本板未編入桌面測試模組（PYRO_SELFTEST_AVAILABLE=0）\r\n");
        g_parse_status = ACK_REJECTED;
#endif
    } else if (strcmp(tok[0], "deploy") == 0 && n >= 2) {
        /* 本機 USB-CDC 觸發手動開傘：與地面站 433 上行 UPLINK_CMD_DEPLOY_* 走同一組
         * pending 旗標（UplinkCmd_ForceDeploy），確保兩種觸發來源執行路徑完全一致。 */
#if FEATURE_UPLINK_DEPLOY
        FlightState_t st = (FlightState_t)current_fsm_state;
        uint8_t in_flight = (st >= STATE_BOOST && st <= STATE_MAIN_DEPLOY) ? 1U : 0U;
        if (!in_flight && !UplinkCmd_IsArmed()) {
            printf("[CMD] 開傘命令忽略：未武裝且在地面（先送 arm）\r\n");
            g_parse_status = ACK_UNARMED;
        } else if (strcmp(tok[1], "drogue") == 0) {
            UplinkCmd_ForceDeploy(1, 0);
            printf("[CMD] *** 手動開傘 *** drogue=1 main=0 (in_flight=%u)\r\n", (unsigned)in_flight);
            g_parse_status = ACK_OK;
        } else if (strcmp(tok[1], "main") == 0) {
            UplinkCmd_ForceDeploy(0, 1);
            printf("[CMD] *** 手動開傘 *** drogue=0 main=1 (in_flight=%u)\r\n", (unsigned)in_flight);
            g_parse_status = ACK_OK;
        } else if (strcmp(tok[1], "both") == 0) {
            UplinkCmd_ForceDeploy(1, 1);
            printf("[CMD] *** 手動開傘 *** drogue=1 main=1 (in_flight=%u)\r\n", (unsigned)in_flight);
            g_parse_status = ACK_OK;
        } else {
            printf("[CMD] 用法：deploy drogue|main|both（須先 arm，或已在飛行中）\r\n");
            g_parse_status = ACK_UNKNOWN;
        }
#else
        printf("[CMD] 本板未編入 FEATURE_UPLINK_DEPLOY，無法本地觸發開傘\r\n");
        g_parse_status = ACK_REJECTED;
#endif
    } else if (strcmp(tok[0], "recalib") == 0) {
        /* 本機 USB-CDC 觸發 EKF 重新校準：與地面站 433 上行 UPLINK_CMD_RECALIBRATE
         * 走同一組閘（須先 arm ＋僅限未起飛 STATE_PAD/INIT/STATE_PAD_ARMED）。 */
#if FEATURE_UPLINK_DEPLOY
        if (!UplinkCmd_IsArmed()) {
            printf("[CMD] RECALIBRATE 忽略：未武裝（先送 arm）\r\n");
            g_parse_status = ACK_UNARMED;
        } else {
            FlightState_t st = (FlightState_t)current_fsm_state;
            if (st <= STATE_PAD || st == STATE_PAD_ARMED) {
                EKF_ResetCalibration();
                printf("[CMD] *** EKF 重新校準已觸發 ***\r\n");
                g_parse_status = ACK_OK;
            } else {
                printf("[CMD] RECALIBRATE 拒絕：非地面狀態（state=%u）\r\n", (unsigned)st);
                g_parse_status = ACK_REJECTED;
            }
        }
#else
        printf("[CMD] 本板未編入 FEATURE_UPLINK_DEPLOY\r\n");
        g_parse_status = ACK_REJECTED;
#endif
    } else if (strcmp(tok[0], "help") == 0) {
        cmd_print_help();

#if FEATURE_LORA
    } else if (strcmp(tok[0], "e22") == 0 && n >= 2) {
        if (strcmp(tok[1], "show") == 0) {
            LoRaE22_PrintConfig();
        } else if (strcmp(tok[1], "freq") == 0 && n >= 3) {
            uint32_t mhz = (uint32_t)strtoul(tok[2], NULL, 10);
            uint8_t  ch  = 0;
            if (!e22_mhz_to_ch(mhz, &ch)) {
                printf("[E22] freq 範圍 410-493 MHz\r\n");
                g_parse_status = ACK_BADARG;
                return;
            }
            cmd_lora_pause();
            HAL_StatusTypeDef st = LoRaE22_SetFreqMHz(mhz);
            cmd_lora_resume();
            if (st == HAL_OK) {
                printf("[E22] freq set %lu MHz (CH=%u) OK\r\n",
                                     (unsigned long)mhz, (unsigned)ch);
#if FEATURE_FLASH
                Save_Lora_Params_To_Flash();
#endif
            }
            else              printf("[E22] freq set FAIL (st=%d)\r\n", (int)st);
        } else if (strcmp(tok[1], "pwr") == 0 && n >= 3) {
            uint32_t lvl = (uint32_t)strtoul(tok[2], NULL, 10);
            if (lvl > 3U) {
                printf("[E22] pwr 等級 0=30dBm 1=27dBm 2=24dBm 3=21dBm（3V3 供電建議 3）\r\n");
                g_parse_status = ACK_BADARG;
                return;
            }
            cmd_lora_pause();
            HAL_StatusTypeDef st = LoRaE22_SetPowerLevel((uint8_t)lvl);
            cmd_lora_resume();
            if (st == HAL_OK) {
                printf("[E22] pwr set level %lu OK\r\n", (unsigned long)lvl);
#if FEATURE_FLASH
                Save_Lora_Params_To_Flash();
#endif
            }
            else              printf("[E22] pwr set FAIL (st=%d)\r\n", (int)st);
        } else if (strcmp(tok[1], "air") == 0 && n >= 3) {
            uint32_t ar = (uint32_t)strtoul(tok[2], NULL, 10);
            if (ar > 7U) {
                printf("[E22] air 速率 0=0.3k 1=1.2k 2=2.4k 3=4.8k 4=9.6k 5=19.2k 6=38.4k 7=62.5k\r\n");
                g_parse_status = ACK_BADARG;
                return;
            }
            cmd_lora_pause();
            HAL_StatusTypeDef st = LoRaE22_SetAirRate((uint8_t)ar);
            cmd_lora_resume();
            if (st == HAL_OK) {
                printf("[E22] air rate set %lu OK（兩端須一致）\r\n", (unsigned long)ar);
#if FEATURE_FLASH
                Save_Lora_Params_To_Flash();
#endif
            }
            else              printf("[E22] air rate set FAIL (st=%d)\r\n", (int)st);
        } else {
            printf("[E22] 未知子命令，輸入 help\r\n");
            g_parse_status = ACK_UNKNOWN;
        }

    } else if (strcmp(tok[0], "e80") == 0 && n >= 2) {
        uint32_t f = 0; uint8_t sf = 0, bw = 0, cr = 0; int8_t pwr = 0; uint16_t pre = 0;
        LoRaE80_GetParams(&f, &sf, &bw, &cr, &pwr, &pre);

        if (strcmp(tok[1], "show") == 0) {
            cmd_print_e80_params();
        } else if (strcmp(tok[1], "freq") == 0 && n >= 3) {
            uint32_t hz = (uint32_t)strtoul(tok[2], NULL, 10);
            if (hz < 862000000UL || hz > 928000000UL)
                printf("[E80] 注意：頻率建議 862-928 MHz，仍套用\r\n");
            cmd_e80_reconfig(hz, sf, bw, cr, pwr, pre);
        } else if (strcmp(tok[1], "sf") == 0 && n >= 3) {
            uint32_t v = (uint32_t)strtoul(tok[2], NULL, 10);
            if (v < 7U || v > 12U) { printf("[E80] SF 範圍 7-12\r\n"); g_parse_status = ACK_BADARG; return; }
            cmd_e80_reconfig(f, (uint8_t)v, bw, cr, pwr, pre);
        } else if (strcmp(tok[1], "bw") == 0 && n >= 3) {
            uint32_t v = (uint32_t)strtoul(tok[2], NULL, 10);
            if (v > 0xFFU || !lora_bw_valid((uint8_t)v)) {
                printf("[E80] BW idx 合法值: 0 1 2 3 4(125k) 5(250k) 6(500k) 8 9 10\r\n");
                g_parse_status = ACK_BADARG;
                return;
            }
            cmd_e80_reconfig(f, sf, (uint8_t)v, cr, pwr, pre);
        } else if (strcmp(tok[1], "cr") == 0 && n >= 3) {
            uint32_t v = (uint32_t)strtoul(tok[2], NULL, 10);
            if (v < 1U || v > 4U) { printf("[E80] CR 範圍 1-4\r\n"); g_parse_status = ACK_BADARG; return; }
            cmd_e80_reconfig(f, sf, bw, (uint8_t)v, pwr, pre);
        } else if (strcmp(tok[1], "pwr") == 0 && n >= 3) {
            int v = atoi(tok[2]);
            if (v < -9 || v > 22) { printf("[E80] pwr 範圍 -9~22 dBm\r\n"); g_parse_status = ACK_BADARG; return; }
            cmd_e80_reconfig(f, sf, bw, cr, (int8_t)v, pre);
        } else if (strcmp(tok[1], "pre") == 0 && n >= 3) {
            long v = atol(tok[2]);
            if (v < 6 || v > 65535) { printf("[E80] preamble 6~65535\r\n"); g_parse_status = ACK_BADARG; return; }
            cmd_e80_reconfig(f, sf, bw, cr, pwr, (uint16_t)v);
        } else {
            printf("[E80] 未知子命令，輸入 help\r\n");
            g_parse_status = ACK_UNKNOWN;
        }
#else
    } else if (strcmp(tok[0], "e22") == 0 || strcmp(tok[0], "e80") == 0) {
        printf("[LORA] BACKUP 無 LoRa 硬體（FEATURE_LORA=0），無參數可調\r\n");
#endif /* FEATURE_LORA */

    } else if (strcmp(tok[0], "recovery") == 0 || strcmp(cmd, "CMD_RECOVERY") == 0) {
        extern FlightState_t current_fsm_state;
        /* 僅允許已著陸 (STATE_LANDED) 才執行：避免發射前在 PAD/PAD_ARMED 誤觸就
         * 停掉蜂鳴器與 SD/Flash 記錄（先前只擋飛行中 BOOST..MAIN_DEPLOY，PAD/PAD_ARMED
         * 會被放行，非預期）。 */
        if (current_fsm_state != STATE_LANDED) {
            printf("[CMD] 尋回指令拒絕：僅限已著陸 (STATE_LANDED) 才可執行（state=%u）\r\n", (unsigned)current_fsm_state);
            g_parse_status = ACK_REJECTED;
        } else {
            App_ExecuteRecovery();
        }

    } else if (strcmp(tok[0], "flash") == 0 && n >= 2) {
        FlightState_t current_st = (FlightState_t)current_fsm_state;
        if (current_st != STATE_INIT && current_st != STATE_PAD && 
            current_st != STATE_PAD_ARMED && current_st != STATE_LANDED) {
            printf("[FLASH] REJECTED: Flash erase/export only allowed in PAD or LANDED states! (fsm=%u)\r\n", (unsigned)current_st);
            g_parse_status = ACK_REJECTED;
            return;
        }
        if (strcmp(tok[1], "erase") == 0) {
            /* 子命令：flash erase ring | flash erase all
             *   ring（或省略，相容舊 GUI 按鈕）：只清環形緩衝區，保留 Sector 0
             *                                    校準/mag/LoRa 參數與任務總結區
             *   all                          ：連同校準/總結整顆清空 */
            int erase_all  = (n >= 3 && strcmp(tok[2], "all")  == 0);
            int erase_ring = (n <  3) || (strcmp(tok[2], "ring") == 0);
            if (!erase_all && !erase_ring) {
                printf("[FLASH] 未知擦除範圍 '%s'（用 ring 或 all）\r\n", tok[2]);
                g_parse_status = ACK_UNKNOWN;
            } else {
                /* 不在此包外層 SPI3 鎖：W25QXX_EraseBlock64K/EraseSector 每次 SPI
                 * 交易已各自透過 CS_LOW/CS_HIGH 自行鎖/解鎖（見 w25qxx.c）。外層長時間
                 * 持鎖會讓 E80 920MHz LoRa 遙測（含擦除進度/副板中繼）整段擦除期間
                 * 完全發不出去，GUI 因此在 erase 階段看不到任何更新——並非必要。 */
                /* 逐塊 64KB 擦除單次最壞 ~2s；先放寬 IWDG 到 ~10s（比照 SD 掛載），
                 * 每塊擦完餵狗，全程不觸發看門狗重啟；完成後改回飛行用 2.05s 視窗。 */
                {
                    IWDG_HandleTypeDef iwdg_wide = hiwdg;
                    iwdg_wide.Init.Prescaler = IWDG_PRESCALER_256;
                    iwdg_wide.Init.Reload    = 1250;   /* 32kHz/256=125Hz -> 1250/125 = 10s */
                    HAL_IWDG_Init(&iwdg_wide);
                }

                /* erase 全程（跨多個 SPI 交易、可達數秒）不可讓 ARM 生效：LoRa 上行的
                 * UPLINK_CMD_ARM 在另一個 FreeRTOS 任務(UplinkCmd_Poll)非同步處理，
                 * 會與這裡尚未擦完的 flash 同時把 FSM 切到 STATE_PAD_ARMED、開始記錄，
                 * 寫入位置可能落在還沒擦淨的區塊。★此旗標由 uplink_cmd.c 的 ARM/text
                 * "arm" 兩處共查。 */
                g_flash_erase_in_progress = 1U;

                W25QXX_StatusTypeDef st = W25QXX_OK;
                if (erase_all) {
                    printf("[FLASH] Erasing WHOLE chip (含校準/總結/紀錄)...\r\n");
                    st = W25QXX_EraseBlock64K(FLASH_SYSFLAGS_ADDR);  /* Block 0：SysFlags + Summary */
                    HAL_IWDG_Refresh(&hiwdg);
                    if (st == W25QXX_OK) st = FlashRing_EraseAll();  /* Block 1..255：Ring Buffer */
                } else {
                    printf("[FLASH] Erasing Ring Buffer only (保留校準/總結)...\r\n");
                    st = FlashRing_EraseAll();
                }

                g_flash_erase_in_progress = 0U;

                /* 還原飛行用 ~2.05s IWDG 視窗 */
                HAL_IWDG_Init(&hiwdg);
                HAL_IWDG_Refresh(&hiwdg);

                /* 實體擦除無論 st 成敗都已執行 → 一律重掃寫入頭。
                 * 若只在 st==OK 才 Init，失敗時會留下 erase 前的「環中間」舊指標，
                 * 之後寫入落到 export（固定自 base 掃描）看不到的位置 —— 病徵正是
                 * 「不重啟就抓不到資料、重啟後才有」。同時把記錄開關拉回開啟，
                 * 避免先前 recovery 關閉後未復位（recovery 只在開機才會被重設）。
                 * ★2026-07-31：改用「重掃 + 唯讀認領」取代 FlashRing_Init()——整環剛全擦完，
                 * 再跑一次 bulk 預擦等於對已擦淨區域重擦 FLASH_RING_PREERASE_TARGET 次
                 * （1500 × ~50ms ≈ 75s）純浪費；認領是毫秒級且結果相同。 */
                FlashRing_ScanWriteHeadOnly();
                uint32_t pool_after = FlashRing_ProbePoolNoErase();
                g_flash_logging_active = 1;
                g_flash_need_erase = (pool_after < FLASH_RING_PREERASE_TARGET) ? 1U : 0U;
                printf("[FLASH] 擦除後池：%lu/%u sectors（%s）\r\n",
                       (unsigned long)pool_after, (unsigned)FLASH_RING_PREERASE_TARGET,
                       g_flash_need_erase ? "★仍未達標，ARM 仍被擋" : "已達標，可 ARM");
                if (st == W25QXX_OK) {
                    printf("[FLASH] %s erased and re-initialized OK!\r\n",
                           erase_all ? "Whole chip" : "Ring Buffer");
                } else {
                    printf("[FLASH] ERROR: Erase reported failure (st=%d)，已強制重掃寫入頭\r\n", (int)st);
                    g_parse_status = ACK_REJECTED;
                }
            }
        } else if (strcmp(tok[1], "pool") == 0) {
            /* ★2026-07-31：快速填池。只把已擦池補到 FLASH_RING_PREERASE_TARGET 達標，
             * 不動整環（已擦部分不重擦）。用途：bench 反覆測試、或飛前已整環全擦過但
             * 池被寫掉一部分時的省時補救。飛前正規流程仍是 `flash erase`。
             * 與 `flash erase` 共用同一組保護：擦除期間禁止 ARM 生效（見上方說明）。 */
            printf("[FLASH] 快速填池：目標 %u sectors（只補不足部分，不動整環）...\r\n",
                   (unsigned)FLASH_RING_PREERASE_TARGET);
            g_flash_erase_in_progress = 1U;
#if FEATURE_LINK
            uint32_t pool_now = FlashRing_TopUpPool(FlashPreErase_ProgressCallback);
#else
            uint32_t pool_now = FlashRing_TopUpPool(NULL);
#endif
            g_flash_erase_in_progress = 0U;
            HAL_IWDG_Refresh(&hiwdg);
            g_flash_logging_active = 1;
            g_flash_need_erase = (pool_now < FLASH_RING_PREERASE_TARGET) ? 1U : 0U;
            if (g_flash_need_erase) {
                printf("[FLASH] ★填池未達標（%lu/%u）：環內可能有殘留舊資料擋住，"
                       "請改下 `flash erase` 整環全擦。ARM 仍被擋。\r\n",
                       (unsigned long)pool_now, (unsigned)FLASH_RING_PREERASE_TARGET);
                g_parse_status = ACK_REJECTED;
            } else {
                printf("[FLASH] 填池完成（%lu/%u），可以 ARM。\r\n",
                       (unsigned long)pool_now, (unsigned)FLASH_RING_PREERASE_TARGET);
            }
        } else if (strcmp(tok[1], "dump") == 0) {
            SPI3_Bus_Lock();
            Flash_DumpAll();
            SPI3_Bus_Unlock();
        } else if (strcmp(tok[1], "export") == 0) {
            SPI3_Bus_Lock();
            printf("--- SYSFLAGS_START ---\r\n");
            for (uint32_t dump_addr = FLASH_SYSFLAGS_ADDR; dump_addr < FLASH_SYSFLAGS_ADDR + FLASH_SYSFLAGS_SIZE; dump_addr += 16U) {
                uint8_t dump_buf[16];
                if (W25QXX_ReadData(dump_addr, dump_buf, sizeof(dump_buf)) != W25QXX_OK) break;
                printf("%06lX:", (unsigned long)dump_addr);
                for (uint32_t i = 0; i < sizeof(dump_buf); i++) {
                    printf(" %02X", dump_buf[i]);
                }
                printf("\r\n");
                if (((dump_addr - FLASH_SYSFLAGS_ADDR) & 0x3FFU) == 0U) HAL_IWDG_Refresh(&hiwdg);
            }
            printf("--- SYSFLAGS_END ---\r\n");

            printf("--- CSV_START ---\r\n");
            printf("addr,flight_id,seq,tick_ms,fsm_state,flags,bat_mv,bmi_ax,bmi_ay,bmi_az,bmi_gx,bmi_gy,bmi_gz,adxl_x,adxl_y,adxl_z,baro_temp_c_x100,baro_press_pa,baro_alt_cm,ekf_alt_cm,ekf_vel_cms,ekf_q0,ekf_q1,ekf_q2,ekf_q3,gps_lat,gps_lon,gps_alt_m,gps_spd_cms,gps_sats,gps_fix\r\n");

            FlashRingPacket_t pkt;
            uint32_t addr = FLASH_RINGBUF_ADDR;
            uint32_t read_count = 0;

            while (addr + FLASH_RING_PACKET_SIZE <= FLASH_RINGBUF_END + 1UL) {
                if (W25QXX_ReadData(addr, (uint8_t*)&pkt, sizeof(FlashRingPacket_t)) != W25QXX_OK) break;
                if (pkt.magic[0] == 0xFF && pkt.magic[1] == 0xFF) break;

                if (pkt.magic[0] == 0xAA && pkt.magic[1] == 0x55 && ring_crc16((const uint8_t*)&pkt, FLASH_RING_PACKET_SIZE - 2) == pkt.crc16) {
                    /* 免 float printf 格式化：拆成整數與小數 4 位，完美相容 nano.specs */
                    int q0_i = (int)pkt.ekf_q0, q0_f = (int)fabsf((pkt.ekf_q0 - (float)q0_i) * 10000.0f);
                    int q1_i = (int)pkt.ekf_q1, q1_f = (int)fabsf((pkt.ekf_q1 - (float)q1_i) * 10000.0f);
                    int q2_i = (int)pkt.ekf_q2, q2_f = (int)fabsf((pkt.ekf_q2 - (float)q2_i) * 10000.0f);
                    int q3_i = (int)pkt.ekf_q3, q3_f = (int)fabsf((pkt.ekf_q3 - (float)q3_i) * 10000.0f);

                    printf("0x%06lX,%lu,%u,%lu,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%ld,%ld,%ld,%d.%04d,%d.%04d,%d.%04d,%d.%04d,%ld,%ld,%d,%d,%u,%u\r\n",
                           (unsigned long)addr, (unsigned long)pkt.flight_id,
                           (unsigned)pkt.seq, (unsigned long)pkt.tick_ms, (unsigned)pkt.fsm_state, (unsigned)pkt.flags,
                           (unsigned)pkt.bat_voltage_mv, (int)pkt.bmi_ax, (int)pkt.bmi_ay, (int)pkt.bmi_az,
                           (int)pkt.bmi_gx, (int)pkt.bmi_gy, (int)pkt.bmi_gz,
                           (int)pkt.adxl_x, (int)pkt.adxl_y, (int)pkt.adxl_z,
                           (int)pkt.baro_temp_c_x100, (unsigned long)pkt.baro_press_pa,
                           (long)pkt.baro_alt_cm, (long)pkt.ekf_pos_z_cm, (long)pkt.ekf_vel_z_cms,
                           q0_i, q0_f, q1_i, q1_f, q2_i, q2_f, q3_i, q3_f,
                           (long)pkt.gps_lat, (long)pkt.gps_lon,
                           (int)pkt.gps_alt_m, (int)pkt.gps_spd_cms,
                           (unsigned)pkt.gps_sats, (unsigned)pkt.gps_fix);

                    read_count++;
                }
                addr += FLASH_RING_PACKET_SIZE;
                if ((read_count & 0x3F) == 0) HAL_IWDG_Refresh(&hiwdg);
            }
            printf("--- CSV_END ---\r\n");
            if (read_count == 0) {
                printf("[FLASH] Notice: Flash ring buffer has 0 valid packets (unwritten/erased or no active flight session).\r\n");
            }
            printf("[FLASH] Export finished. Total %lu packets.\r\n", (unsigned long)read_count);
            SPI3_Bus_Unlock();
        } else {
            printf("[FLASH] 未知指令 (格式: flash dump / flash erase [ring|all] / flash pool / flash export)\r\n");
            g_parse_status = ACK_UNKNOWN;
        }

    } else if ((strcmp(tok[0], "cmd") == 0 && n >= 2 && strcmp(tok[1], "hello") == 0) ||
               strcmp(tok[0], "hello") == 0) {
        for (int i = 0; i < 10; i++) {
            printf("[CMD] hello\r\n");
        }
    } else {
        printf("[CMD] 未知命令 '%s'，輸入 help\r\n", tok[0]);
        g_parse_status = ACK_UNKNOWN;
    }

    /* 統一輸出標準 [ACK] 回覆確認，供 USB CDC 串口與 GUI 控制台捕捉與狀態同步 */
    printf("[ACK] status:%s cmd:\"%s\"\r\n", ack_status_str(g_parse_status), tok[0]);
}

void Poll_Serial_Commands(void) {
    uint8_t rx_byte;
    /* 每次最多處理 CMD_BUFFER_SIZE 個 byte（上限保護）：UART 轉接頭拔掉時 RX 即使有殘餘
     * 雜訊，也不會在此 while 無限狂讀、霸佔 CPU 而餓死餵狗任務（PA3 已改上拉為主要防線，
     * 此為第二道防線）。 */
    for (int guard = 0; guard < CMD_BUFFER_SIZE; guard++) {
        if (HAL_UART_Receive(&huart2, &rx_byte, 1, 0) != HAL_OK) break;
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (g_cmd_idx > 0) {
                g_cmd_buf[g_cmd_idx] = '\0';
                Parse_Serial_Command(g_cmd_buf);
                g_cmd_idx = 0;
            }
        } else if (g_cmd_idx < CMD_BUFFER_SIZE - 1) {
            g_cmd_buf[g_cmd_idx++] = (char)rx_byte;
        } else {
            g_cmd_idx = 0; // overflow reset
        }
    }
    /* 清 USART2 浮動/雜訊造成的 ORE/FE/NE/PE（讀 SR 後讀 DR），避免錯誤旗標卡住接收 */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    (void)huart2.Instance->SR;
    (void)huart2.Instance->DR;

#if !IS_GROUND && FEATURE_USB_CDC
    /* 同時消費 USB CDC 環形緩衝（USB 直連航電板時 arm/disarm/help 等命令） */
    for (int guard = 0; guard < CMD_BUFFER_SIZE; guard++) {
        if (!GsLoraTest_PopUsbByte(&rx_byte)) break;
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (g_cmd_idx > 0) {
                g_cmd_buf[g_cmd_idx] = '\0';
                Parse_Serial_Command(g_cmd_buf);
                g_cmd_idx = 0;
            }
        } else if (g_cmd_idx < CMD_BUFFER_SIZE - 1) {
            g_cmd_buf[g_cmd_idx++] = (char)rx_byte;
        } else {
            g_cmd_idx = 0;
        }
    }
#endif
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
#if IS_GROUND
  GroundStation_Run();   /* 地面站主迴圈（接收 LoRa 433+920 + GPS → 對齊時間戳 → SD/Flash/USB），不返回 */
#endif
  uint32_t tick = 0;

  /* ★2026-07-30：熱重啟狀態恢復移到 StartDefaultTask 最前面、搶在 LED 開機自檢(2s)與
   * SD 卡掛載(正常 <1s，卡壞/接觸不良最壞 ~10s，見下方 SD 區塊註解)之前執行。
   * 原本這段接在 SD 卡掛載「之後」——代表即使旗標/tick_ms 語意都修好了，飛行中重啟
   * 到 current_fsm_state 真正變回 BOOST 之間，仍會經過「2s LED + SD 掛載」這段完全
   * 空窗：期間 current_fsm_state 停在全域預設值 STATE_INIT 未變，遙測（LoRaTelemetry_Task
   * 是獨立 RTOS task，開機即跑）會照樣送出 STATE_INIT，且沒有任何 FSM_Step/失效保護/
   * 開傘邏輯在跑——這正是使用者回報「還是從 init 開始跑，沒有立刻切換到 boost」的根因：
   * 旗標/tick_ms 修好後熱重啟「最終」仍會恢復正確狀態，只是被這段開機序列拖慢，不是
   * 立刻生效。搬到最前面後，開機到狀態恢復之間只剩幾次 Flash SPI 讀取（W25QXX_Init 已在
   * main() 由 HAL 初始化完成，可直接呼叫），量級是毫秒而非秒。
   * 依賴確認：FlashRing 用的 SPI3/W25QXX_Init、baro 用的 SPI1/BMP388_Init 都在 main()
   * osKernelStart() 之前完成；TIM3 中斷（BMP388_ReadData 前後的遮蔽保護）也已在 main()
   * 啟動（HAL_TIM_Base_Start_IT），此處搬到最前面不影響其正確性。 */
#if FEATURE_FLASH
  /* === Flash Ring Buffer：先只掃描寫入頭（不擦除），確定「是否為空中熱重啟」後才決定
   * 要不要跑開機 bulk 預擦 ===
   * ★P0 修正：舊版一開機就無條件 bulk 預擦 960 sector（阻塞 defaultTask ~48s，最壞數分鐘），
   * 且預擦範圍從寫入頭所在 Sector 開始，涵蓋了該 Sector 內「寫入頭之前」的既有資料——等於
   * 在 FlashRing_GetLastPacket() 讀上一筆飛行封包之前就先把它擦了。若重啟發生在飛行中
   * （IWDG/BOR/PIN 等熱重啟），這會同時導致：
   *   1) 熱重啟判斷讀到已擦除(0xFF)的資料、誤判為地面開機，飛控狀態被打回 STATE_PAD；
   *   2) 主迴圈卡在預擦迴圈裡最長數分鐘，期間火箭仍在飛、沒有 FSM/開傘邏輯在跑。
   * 修正後：一律先掃描寫入頭（不擦除）→ 讀上一筆封包 → 判斷熱重啟 → 只有「確定不是熱
   * 重啟」才執行 bulk 預擦；熱重啟則整段跳過（改由唯讀探測還原已擦池，見
   * FlashRing_SkipPreErase()），池耗盡時仍交給既有的
   * s_ring_erase_allowed=0「池滿即丟包」保護（見下方 FlashRing_SetEraseAllowed 呼叫）。 */
  FlashRing_ScanWriteHeadOnly();

  FlashRingPacket_t last_pkt;
  uint8_t pkt_valid = (FlashRing_GetLastPacket(&last_pkt) == W25QXX_OK) ? 1U : 0U;
  FlashFlight_SetLastId(pkt_valid ? last_pkt.flight_id : 0U, 0U);
#else
  uint8_t pkt_valid = 0;
  FlashRingPacket_t last_pkt = {0};
  FlashFlight_SetLastId(0U, 0U);
#endif

  /* 取當下首筆 baro 供高度連續性檢查（與主迴圈相同的 TIM3 遮蔽保護） */
  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
  BMP388_ReadData(&hspi1, &baro_data);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

  /* ★ 第三個參數必須是「起飛後經過時間」而非封包的 tick_ms（開機以來的絕對 tick）：
   * 真實流程是通電 → 校準 → ARM → 在發射台等待，起飛時 tick_ms 早就遠超
   * FSM_HOTSTART_MAX_TICK_MS(60s)，用它比對會把每一次空中熱重啟都判成「陳舊資料」
   * 而打回 PAD（症狀＝重啟後重跑開機預擦、狀態歸零）。 */
  FSM_HotStartDecision_t hs = FSM_HotStartDecide(
      pkt_valid,
      pkt_valid ? last_pkt.fsm_state : 0U,
      pkt_valid ? last_pkt.flight_tick_ms : 0U,
      pkt_valid ? ((float)last_pkt.baro_alt_cm / 100.0f) : 0.0f,
      baro_data.altitude,
      (pkt_valid && (last_pkt.flags & 0x01U)) ? 1U : 0U);

#if FEATURE_FLASH
  if (hs.restore) {
      /* 空中熱重啟：絕不執行 bulk 預擦（見上方說明）。立刻關閉 erase_allowed——不等主
       * 迴圈第一輪才關，避免開機到第一輪迴圈之間這段空窗仍有機會被誤觸發同步擦除卡死。 */
      FlashRing_SkipPreErase();
      FlashRing_SetEraseAllowed(0U);
      printf("[FLASH_RING] HOT-RESTART：全程零擦除，改用唯讀探測還原已擦池，重啟後繼續記錄"
             "（池耗盡才丟包，飛行中永不擦除）\r\n");
  } else {
      /* 正常地面開機（或熱啟動驗證未過回 PAD 重校準）：
       * ★2026-07-31 起「開機一律不擦除」（使用者決策）。舊版每次開機無條件整環全擦
       * （~3 min）＋960-sector 預擦（~48s），代價太大且有兩個實務問題：飛完落地重開機
       * 就把黑盒子擦光（BT1 dump 排在擦除之後，救不回來），以及擦除全程主迴圈停擺。
       * 新流程：開機只做「唯讀認領」——把上一次使用者觸發擦除留在 flash 上的已擦區
       * 找回來（FlashRing_ProbePoolNoErase，毫秒級、零擦除）。池未達標就：
       *   ① 1Hz 醒目橫幅提醒（見主迴圈 g_flash_need_erase 區塊）
       *   ② 下鏈 arm_flags 帶 TELEM_ARM_NEED_ERASE 給地面站 GUI 顯示
       *   ③ ARM 一律被擋（既有 fsm.c flash_pool_ready 閘，非新增機制）
       * 使用者以 `flash erase`（整環全擦，飛前正規流程）或 `flash pool`（快速填池，
       * bench 用）建立池，USB console 與 LoRa 上行文字幀共用同一條路徑。
       * FEATURE_FLASH_BOOT_FULL_ERASE 保留為緊急退路（預設 0）：設 1 才恢復舊的
       * 開機自動整環全擦 + 填池行為。 */
#if FEATURE_FLASH_BOOT_FULL_ERASE
      {
          IWDG_HandleTypeDef iwdg_wide = hiwdg;
          iwdg_wide.Init.Prescaler = IWDG_PRESCALER_256;
          iwdg_wide.Init.Reload    = 1250;   /* 32kHz/256=125Hz -> 1250/125 = 10s，
                                               * 單次 64KB block erase 最壞 ~2s，比照
                                               * 「flash erase」指令同款放寬幅度。 */
          HAL_IWDG_Init(&iwdg_wide);
      }
#if FEATURE_LINK
      Boot_BroadcastPrompt("[BOOT] FEATURE_FLASH_BOOT_FULL_ERASE=1：開機整環全擦 "
                            "(4080 sectors / ~16MB, ~3 min)...");
#else
      printf("[BOOT] FEATURE_FLASH_BOOT_FULL_ERASE=1: Erasing ENTIRE Flash Ring (~3 min)...\r\n");
#endif
      FlashRing_EraseAll();   /* Block 1..255：只清環（保留 Sector 0 校準/mag/LoRa/總結） */
      HAL_IWDG_Init(&hiwdg);
      HAL_IWDG_Refresh(&hiwdg);
      FlashRing_ScanWriteHeadOnly();
#if FEATURE_LINK
      FlashRing_RunPreErase(FlashPreErase_ProgressCallback);
      Boot_WaitPeerEraseProgress();
#else
      FlashRing_RunPreErase(NULL);
#endif
#else /* 預設路徑：零擦除，只認領既有已擦區 */
      uint32_t pool_sectors = FlashRing_ProbePoolNoErase();
      printf("[FLASH_RING] 開機不擦除，認領既有已擦池：%lu/%u sectors（write=0x%06lX）\r\n",
             (unsigned long)pool_sectors, (unsigned)FLASH_RING_PREERASE_TARGET,
             (unsigned long)FlashRing_GetWriteAddr());
      if (pool_sectors < FLASH_RING_PREERASE_TARGET) {
          g_flash_need_erase = 1U;   /* 主迴圈 1Hz 橫幅 + 下鏈 arm_flags；ARM 已被 pool 閘擋下 */
#if FEATURE_LINK
          {
              char boot_msg[128];
              snprintf(boot_msg, sizeof(boot_msg),
                       "[FLASH_RING] ★需要擦除：池 %lu/%u sectors，ARM 已被擋下。"
                       "請下 `flash erase`（整環，~3min）或 `flash pool`（快速填池）。",
                       (unsigned long)pool_sectors, (unsigned)FLASH_RING_PREERASE_TARGET);
              Boot_BroadcastPrompt(boot_msg);
          }
#endif
      }
#endif /* FEATURE_FLASH_BOOT_FULL_ERASE */
  }
#endif

  if (hs.restore) {
      g_hotstart_restored = 1U;   /* 遙測 TELEM_FLAG_HOTSTART（地面站需特別標示） */
      FlashFlight_SetLastId(last_pkt.flight_id, 1U);
      EKF_in_flight = 1;          /* EKF 跳過發射台 ZUPT / launch 偵測 */
      current_fsm_state = hs.state;
      /* 起飛基準＝現在 − 封包記錄的「起飛後經過時間」。重啟後 HAL_GetTick() 由 0 重數，
       * 用 last_pkt.tick_ms（重啟前的開機絕對 tick）會得到毫無意義的負值迴繞，讓
       * FSM 失效保護時限（now − flight_start_ms）瞬間爆表。 */
      flight_start_tick = HAL_GetTick() - last_pkt.flight_tick_ms;
      FSM_Init(&g_fsm_ctx, hs.state, HAL_GetTick(), flight_start_tick, hs.drogue_fired);

      /* P0-x 校準政策：EKF_LoadCalibrationFromFlash 已改為「僅存參考值、不設
       * calibrated」（上電必重校）。空中熱啟動無法靜置 3 秒重校，故在此把 Flash
       * 參考偏置真正套用並標記校準完成 —— 陳舊偏置仍遠優於帶著全零偏置飛行。 */
      EKF_ApplyFlashCalibration();

      /* 注入空中 EKF 狀態（封包 EKF 高度/垂直速度/四元數）。
       * 此刻 EKF_Task 阻塞於空佇列（主迴圈尚未饋入 buffer），寫入安全。 */
      float hs_q[4] = { last_pkt.ekf_q0, last_pkt.ekf_q1, last_pkt.ekf_q2, last_pkt.ekf_q3 };
      EKF_HotRestartRestore((float)last_pkt.ekf_pos_z_cm / 100.0f,
                            (float)last_pkt.ekf_vel_z_cms / 100.0f, hs_q);

      /* 還原飛行摘要（最大高度/速度/G + 開傘高度）。★不呼叫 ExtremaTrack_Reset()——
       * 熱重啟要的是接回重啟前的數字，歸零等於把整場飛行的關鍵資料丟掉，而火箭若沒
       * 回收，這些就是唯一來源。注意 ARM 路徑才歸零，兩條路徑刻意分開。 */
      ExtremaTrack_Restore(last_pkt.max_alt_m, last_pkt.max_vel_ms, last_pkt.max_acc_cg,
                           last_pkt.drogue_alt_m, last_pkt.main_alt_m);

      printf("[FSM] [WARNING] HOT-RESTART：恢復狀態 %d（drogue_fired=%u），起飛後 %lu ms（封包開機 tick %lu ms）\r\n",
             (int)hs.state, hs.drogue_fired,
             (unsigned long)last_pkt.flight_tick_ms, (unsigned long)last_pkt.tick_ms);
      printf("[FSM] HOT-RESTART：飛行摘要已還原 max_alt=%um max_vel=%dm/s max_acc=%u.%02ug "
             "drogue_alt=%dm main_alt=%dm（-32768=未開傘）\r\n",
             (unsigned)last_pkt.max_alt_m, (int)last_pkt.max_vel_ms,
             (unsigned)(last_pkt.max_acc_cg / 100U), (unsigned)(last_pkt.max_acc_cg % 100U),
             (int)last_pkt.drogue_alt_m, (int)last_pkt.main_alt_m);
  } else {
      current_fsm_state = STATE_PAD; // 正常地面起飛（或驗證未過回 PAD 重校準）
      FSM_Init(&g_fsm_ctx, STATE_PAD, HAL_GetTick(), 0U, 0U);
      if (pkt_valid &&
          last_pkt.fsm_state >= STATE_BOOST && last_pkt.fsm_state <= STATE_DESCENT) {
          printf("[FSM] 偵測到飛行中封包但熱啟動驗證未過（起飛後 %lu ms，封包 baro %ld cm vs 當下 %ld cm），回 PAD 重校準。\r\n",
                 (unsigned long)last_pkt.flight_tick_ms,
                 (long)last_pkt.baro_alt_cm, (long)(baro_data.altitude * 100.0f));
      }
  }

#if FEATURE_LORA && FEATURE_FLASH
  /* === LoRa 參數恢復邏輯 === */
  uint8_t last_state = pkt_valid ? last_pkt.fsm_state : 0U;
  if (pkt_valid && (last_state == STATE_PAD_ARMED || (last_state >= STATE_BOOST && last_state <= STATE_MAIN_DEPLOY))) {
      FlashSysFlags_t sys_flags;
      if (Flash_ReadSysFlags(&sys_flags) == W25QXX_OK && sys_flags.lora.magic == 0x50544c4f) {
          printf("[LORA] Restoring parameters from Flash (state = %d)...\r\n", (int)last_state);
          cmd_lora_pause();

          if (lora433_ok) {
              LoRaE22_SetFreqMHz(sys_flags.lora.e22_freq_mhz);
              LoRaE22_SetPowerLevel(sys_flags.lora.e22_pwr);
              LoRaE22_SetAirRate(sys_flags.lora.e22_air);
          }

          if (lora920_ok) {
              LoRaE80_Reconfig(sys_flags.lora.e80_freq_hz, sys_flags.lora.e80_sf, sys_flags.lora.e80_bw,
                               sys_flags.lora.e80_cr, sys_flags.lora.e80_pwr_dbm, sys_flags.lora.e80_preamble);
          }

          cmd_lora_resume();
          printf("[LORA] Parameters restored successfully.\r\n");
          if (lora433_ok) LoRaE22_PrintConfig();
          if (lora920_ok) cmd_print_e80_params();
      } else {
          printf("[LORA] No valid LoRa parameters found in Flash.\r\n");
      }
  } else {
      printf("[LORA] Keep using default LoRa parameters (state = %d).\r\n", (int)last_state);
  }
#endif

  printf("\r\n============================================================\r\n");
  printf("[ROLE] %s  [LORA RADIO PARAMETERS]\r\n", IS_PRIMARY ? "PRIMARY_AV (主航電)" : "BACKUP_AV (備援航電)");
  if (lora433_ok) {
      LoRaE22_PrintConfig();
  } else {
      printf("[LORA433] E22 not detected — no parameters to show.\r\n");
  }
  if (lora920_ok) {
      LoRaE80_PrintConfig();
  } else {
      printf("[LORA920] E80 not detected — held in reset, no parameters to show.\r\n");
  }
  printf("============================================================\r\n\r\n");

  /* === LED 開機自檢：全亮 2 秒確認三顆 LED 正常，之後全滅交由主迴圈接管 ===
   * 分 4 × 500ms 餵狗，避免 IWDG 2.05s timeout 在此期間觸發 reset。 */
  HAL_GPIO_WritePin(GPIOE, LED_SYS_Pin | GPIO_PIN_3 | LED_STAT2_Pin, GPIO_PIN_SET);
  for (int _li = 0; _li < 4; _li++) {
      osDelay(500);
      HAL_IWDG_Refresh(&hiwdg);
  }
  HAL_GPIO_WritePin(GPIOE, LED_SYS_Pin | GPIO_PIN_3 | LED_STAT2_Pin, GPIO_PIN_RESET);

#if FEATURE_USER_BUTTONS
  /* 開機按住 USER_BT1 才完整 dump W25Q128 三分區到 USART2（讀取上一次飛行數據、驗證 Flash 狀態）。
   * USER_BT1 為上拉輸入 → 按下讀到 LOW (GPIO_PIN_RESET)。
   * 平時開機跳過 dump，可省下整顆 Flash 輸出 (~3–5 秒) 的初始化時間 (Item N)。 */
  if (HAL_GPIO_ReadPin(USER_BT1_GPIO_Port, USER_BT1_Pin) == GPIO_PIN_RESET) {
      SPI3_Bus_Lock();          /* 整段 dump 持 SPI3 鎖，保證輸出不被 E80 遙測交錯 */
      Flash_DumpAll();          /* P2：已併入 w25qxx.c，SPI handle 由驅動內部綁定 */
      SPI3_Bus_Unlock();
  }
#endif  /* FEATURE_USER_BUTTONS */

  /* 初始化採樣率監測器（Watch 視窗加入 g_sampling_rate 即可一次觀察全部 Hz） */
  SAMPLING_RATE_INIT();

  /* === RTOS 啟動提示音已停用，統一使用 main() 中的硬體開機提示音 === */
  /*
  htim2.Instance->ARR  = 499;
  htim2.Instance->CCR1 = 250;
  htim2.Instance->EGR  = TIM_EGR_UG;
  osDelay(100);
  htim2.Instance->CCR1 = 0;
  osDelay(100);
  htim2.Instance->ARR  = 249;
  htim2.Instance->CCR1 = 125;
  htim2.Instance->EGR  = TIM_EGR_UG;
  osDelay(100);
  htim2.Instance->CCR1 = 0;
  */

  /* === SD 卡初始化與 10 秒日誌啟動 === */
  extern FATFS SDFatFS;
  extern FIL SDFile;
  extern FIL SDImuFile;      /* S6：原始 IMU 二進位記錄檔（FATFS/App/fatfs.c） */
  extern char SDPath[4];
  sd_logging_active = 0;
  uint32_t sd_log_start_tick = 0;
  char sd_fname[16] = {0};   // 本次開機使用的 SD 檔名 (e.g. HIL_003.CSV)

  /* S6：sd_imu_bin_active/s_imu_buf 等狀態已提升為檔案層級全域（main.c 頂部，
   * sd_logging_active 旁），供 App_ExecuteRecovery() 也能在關檔時沖掉緩衝；此處
   * 只留純粹區域用途的檔名字串（與 CSV 共用同一個 fnum 配對命名）。開檔失敗只記錄
   * 不中止 CSV（_FS_LOCK=2 剛好用滿，見 ffconf.h/fatfs.c 註解）。 */
  char sd_imu_fname[16] = {0};

  /* ★ 卡壞/接觸不良時的瘋狂重啟根因：f_mount()→disk_initialize()→BSP_SD_Init()→
   * HAL_SD_Init()→SD_PowerON() 內建 ACMD41 輪詢迴圈上限 SDMMC_MAX_VOLT_TRIAL=0xFFFF
   * 次，卡片一直不回「就緒」時可整段卡住數秒（此 while 迴圈是 ST HAL 原生 busy-loop，
   * 中途完全不會回來讓我們餵狗）。IWDG 只有 ~2.05s（Reload=4095, /16），必炸表重開機；
   * 開機又立刻撞回同一段 → 無限重啟迴圈。此處暫時放寬 IWDG 視窗到 ~10s（Prescaler_256,
   * Reload=1250）撐過這段已知會卡住的一次性掛載動作，SD 區塊結束後立刻改回原本
   * 2.05s 視窗（飛行迴圈仍要求快速偵測當機並復位，不能用 10s 那組）。 */
  {
      IWDG_HandleTypeDef iwdg_wide = hiwdg;
      iwdg_wide.Init.Prescaler = IWDG_PRESCALER_256;
      iwdg_wide.Init.Reload    = 1250;   /* 32kHz/256=125Hz -> 1250/125 = 10s */
      HAL_IWDG_Init(&iwdg_wide);
  }

  // 為了防止 SDIO 在初始化、掛載與建立檔案時被高頻中斷干擾而超時失敗，在此期間暫停高頻中斷
  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);

  printf("\r\n[SD] 正在檢測 SD 卡與進行掛載...\r\n");
  HAL_IWDG_Refresh(&hiwdg);
  
  printf("[SD] 正在初始化 SDIO 介面並檢測實體 SD 卡插入狀態...\r\n");
  HAL_IWDG_Refresh(&hiwdg);
  
  // 1. 物理插卡檢測，避免無卡時 f_mount 總線超時卡死
  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET) {
      printf("[SD] [OK] 檢測到實體 SD 卡已插入！正在掛載文件系統...\r\n");
      HAL_IWDG_Refresh(&hiwdg);
      
      // 2. 立即掛載 SD 卡 (opt=1)，避免 lazy mount 導致 f_stat 遇到 FR_NOT_READY
      if (f_mount(&SDFatFS, SDPath, 1) == FR_OK) {
          printf("[SD] [OK] 文件系統掛載成功！正在建立 HIL_xxx.CSV 檔案...\r\n");
          HAL_IWDG_Refresh(&hiwdg);
          
          // 找到第一個不存在的 HIL_xxx.CSV，確保每次開機寫入全新獨立檔案
          FILINFO fno;
          uint16_t fnum;
          for (fnum = 1; fnum <= 999; fnum++) {
              snprintf(sd_fname, sizeof(sd_fname), "HIL_%03u.CSV", fnum);
              FRESULT fres = f_stat(sd_fname, &fno);
              if (fres == FR_NO_FILE) break;      // 檔案不存在，用這個名稱
              if (fres != FR_OK) {                // 真正的錯誤（卡片未就緒等）
                  printf("[SD] [ERROR] f_stat(%s) 失敗，錯誤碼: %d\r\n", sd_fname, (int)fres);
                  fnum = 1000;
                  break;
              }
              // fres == FR_OK：檔案已存在，繼續找下一個號碼
          }
          printf("[SD] 準備建立新檔案: %s\r\n", sd_fname);
          HAL_IWDG_Refresh(&hiwdg);

          FRESULT fopen_res = f_open(&SDFile, sd_fname, FA_CREATE_NEW | FA_WRITE);
          if (fopen_res == FR_OK) {
              sd_logging_active = 1;
              sd_log_start_tick = HAL_GetTick();
              printf("[SD] [SUCCESS] %s 建立成功！開始全程飛行記錄（起飛前/落地後 10Hz，飛行中 100Hz）...\r\n", sd_fname);
              char header[] = "tick_ms,bmi_ax,bmi_ay,bmi_az,bmi_gx,bmi_gy,bmi_gz,adxl_ax,adxl_ay,adxl_az,temp,press,alt,fsm,ekf_alt_cm,ekf_vel_cms,ekf_q0,ekf_q1,ekf_q2,ekf_q3\r\n";
              UINT bw;
              f_write(&SDFile, header, sizeof(header) - 1, &bw);
          } else {
              printf("[SD] [ERROR] 建立檔案 %s 失敗！錯誤碼: %d\r\n", sd_fname, (int)fopen_res);
          }

          /* S6：原始 IMU 二進位記錄，與 CSV 用同一個 fnum 配對命名（如 HIL_007.CSV +
           * IMU_007.BIN）。獨立於 CSV 是否成功——開檔失敗只記錄、不影響 CSV 飛行記錄。 */
          snprintf(sd_imu_fname, sizeof(sd_imu_fname), "IMU_%03u.BIN", fnum);
          FRESULT imu_fopen_res = f_open(&SDImuFile, sd_imu_fname, FA_CREATE_NEW | FA_WRITE);
          if (imu_fopen_res == FR_OK) {
              sd_imu_bin_active = 1;
              printf("[SD] [SUCCESS] %s 建立成功！開始原始 IMU 記錄（陀螺1000Hz/加速度400Hz，逐 frame 128B）...\r\n",
                     sd_imu_fname);
          } else {
              printf("[SD] [ERROR] 建立檔案 %s 失敗！錯誤碼: %d（原始 IMU 記錄停用，CSV 飛行記錄不受影響）\r\n",
                     sd_imu_fname, (int)imu_fopen_res);
          }
      } else {
          printf("[SD] [ERROR] 掛載 SD 卡文件系統失敗！\r\n");
      }
  } else {
      printf("[SD] [WARNING] 未檢測到實體 SD 卡插入（SDIO_DET 為高電平），跳過 SD 飛行記錄。\r\n");
  }

  // SD 卡掛載與開檔流程結束後，重新恢復高頻中斷
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);

  /* SD 一次性掛載動作（含可能卡住的 HAL_SD_Init）已結束，把 IWDG 視窗改回飛行迴圈
   * 要求的 ~2.05s（原 MX_IWDG_Init 設定），避免之後真正的當機被放寬視窗掩蓋太久。 */
  HAL_IWDG_Init(&hiwdg);

  /* FlashRing_Init 阻塞約 48s(960 sectors × ~50ms/sector)，BMP388（主迴圈讀取）
   * 在此期間無法累積計數。重置所有採樣率監測器，確保第一個 [RATE] 報告反映穩態採樣率。 */
  SAMPLING_RATE_INIT();
  HAL_IWDG_Refresh(&hiwdg);

  /* Infinite loop */
  uint32_t wake_tick = osKernelGetTickCount();
  static uint32_t main_task_accumulated_cycles = 0;
  static uint32_t main_task_last_calc_tick = 0;
  DWT_Init(); // Ensure DWT is initialized
  for(;;)
  {
    uint32_t loop_start_cycles = DWT->CYCCNT;
    /* --- 餵狗 (Feed the independent watchdog) --- */
    HAL_IWDG_Refresh(&hiwdg);

    /* === SD 卡全程飛行記錄（FSM 驅動，取代原 10 秒上限）===
     * 起飛前（地面等待，尚未 BOOST）：10 Hz —— 靜置無動態，省 SD 壽命；
     * 升空至落地前（BOOST..）：100 Hz —— 提高動態階段時間解析度；
     * 落地後 (g_touchdown_tick != 0)：降為 10 Hz；
     * 停止條件：落地後超過 SD_LANDED_LOG_TIMEOUT_MS；
     * 每秒 f_sync 一次，限制斷電造成的資料遺失 (Item E)。 */
    if (sd_logging_active) {
        uint8_t  landed    = (g_touchdown_tick != 0);
        uint8_t  in_flight = (current_fsm_state >= STATE_BOOST) && !landed;
        uint32_t rate_div  = in_flight ? 10U : 100U;   // 飛行 100Hz / 地面(起飛前)+落地後 10Hz

        uint8_t stop_request = 0;
        if (landed && (HAL_GetTick() - g_touchdown_tick) > SD_LANDED_LOG_TIMEOUT_MS) {
            stop_request = 1;
            printf("[SD] 落地後記錄逾時，停止記錄並安全關檔。\r\n");
        }

        if (stop_request) {
            /* IMU bin 與 CSV 共用同一次落地/逾時事件收尾；先沖緩衝關 IMU bin，
             * 再關 CSV + unmount（f_mount(NULL,...) 之後任何檔案 handle 都不再合法，
             * 順序不可顛倒）。 */
            if (sd_imu_bin_active) {
                if (s_imu_len > 0U) {
                    UINT imu_bw;
                    f_write(&SDImuFile, s_imu_buf, s_imu_len, &imu_bw);
                    s_imu_len = 0U;
                }
                f_close(&SDImuFile);
                sd_imu_bin_active = 0;
            }
            f_close(&SDFile);
            f_mount(NULL, SDPath, 1);
            sd_logging_active = 0;
            printf("[SD] [SUCCESS] 檔案已安全關閉並儲存 (%s)。\r\n", sd_fname);
        } else if (tick % rate_div == 0) {
            EKF_State_t sd_ekf = EKF_GetState();
            uint32_t elapsed = HAL_GetTick() - sd_log_start_tick;
            char csv_buf[256];
            int len = snprintf(csv_buf, sizeof(csv_buf),
                               "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                               (unsigned long)elapsed,
                               (int)(imu_data.ax * 1000.0f),
                               (int)(imu_data.ay * 1000.0f),
                               (int)(imu_data.az * 1000.0f),
                               (int)(imu_data.gx * 100.0f),
                               (int)(imu_data.gy * 100.0f),
                               (int)(imu_data.gz * 100.0f),
                               (int)(highg_data.ax * 1000.0f),
                               (int)(highg_data.ay * 1000.0f),
                               (int)(highg_data.az * 1000.0f),
                               (int)(baro_data.temperature * 100.0f),
                               (int)(baro_data.pressure),
                               (int)(baro_data.altitude * 100.0f),
                               (int)current_fsm_state,
                               (long)(sd_ekf.pos_z * 100.0f),
                               (long)(sd_ekf.vel_z * 100.0f),
                               (long)(sd_ekf.q[0] * 10000.0f),
                               (long)(sd_ekf.q[1] * 10000.0f),
                               (long)(sd_ekf.q[2] * 10000.0f),
                               (long)(sd_ekf.q[3] * 10000.0f));
            UINT bytes_written;
            FRESULT res = f_write(&SDFile, csv_buf, len, &bytes_written);
            if (res != FR_OK) {
                printf("[SD] [ERROR] 寫入數據失敗！res=%d，停止記錄並關檔。\r\n", res);
                if (sd_imu_bin_active) {   // SD 卡本身出狀況，IMU bin 一併收尾（理由同上）
                    if (s_imu_len > 0U) {
                        UINT imu_bw;
                        f_write(&SDImuFile, s_imu_buf, s_imu_len, &imu_bw);
                        s_imu_len = 0U;
                    }
                    f_close(&SDImuFile);
                    sd_imu_bin_active = 0;
                }
                f_close(&SDFile);
                f_mount(NULL, SDPath, 1);
                sd_logging_active = 0;
            } else if (tick % 1000 == 0) {
                f_sync(&SDFile);   // 每秒 flush FAT/目錄，限制斷電資料遺失
            }
        }
    }

    /* === S6：原始 IMU 二進位記錄倒出 —— 獨立於上方 CSV 生命週期（IMU bin 開檔失敗
     * 不影響 CSV，反之亦然）。滿 4096B 即寫；另在 1Hz 強制沖刷未滿的緩衝，把斷電
     * 造成的資料遺失上限控制在 ~1 秒內，與 CSV 的 f_sync 節奏一致。 === */
    if (sd_imu_bin_active) {
        uint8_t flush_now = (s_imu_len >= sizeof(s_imu_buf)) ||
                            (tick % 1000 == 0 && s_imu_len > 0U);
        if (flush_now) {
            UINT imu_bw;
            FRESULT imu_res = f_write(&SDImuFile, s_imu_buf, s_imu_len, &imu_bw);
            if (imu_res != FR_OK) {
                printf("[SD] [ERROR] IMU 二進位寫入失敗！res=%d，停止原始 IMU 記錄並關檔（CSV 不受影響）。\r\n",
                       (int)imu_res);
                f_close(&SDImuFile);
                sd_imu_bin_active = 0;
            } else {
                s_imu_len = 0U;
                if (tick % 1000 == 0) {
                    f_sync(&SDImuFile);   // 每秒 flush FAT/目錄，限制斷電資料遺失
                }
            }
        }
    }

    /* === EKF 雙速率同步組裝（陀螺 1000Hz / 加速度 400Hz，於陀螺新批次到達時執行） ===
     * 陀螺 batch 10 顆(=1000Hz×10ms)為主迴圈，加速度 batch 4 顆(=400Hz×10ms)零階保持
     * (ZOH)映射進去、以 has_accel 標記真正的新樣本；EKF_Task 據此姿態積分跑 1000Hz、
     * 位置/共變異數 Predict 跑 400Hz（見 ekf.c EKF_Task）。 */
    if (bmi088_ok && g_bmi_acc_new_batch_ready && g_bmi_gyro_new_batch_ready) {
        g_bmi_acc_new_batch_ready = 0;
        g_bmi_gyro_new_batch_ready = 0;

        uint8_t acc_ready_idx = g_bmi_acc_ready_batch_idx;
        uint8_t gyro_ready_idx = g_bmi_gyro_ready_batch_idx;

        /* 改善項目 A：高G感測器切換前置快照。
         * 在進入 EKF 樣本迴圈前鎖定 ADXL batch idx，避免迴圈執行期間 TIM3 ISR 完成新批
         * 次並更新 g_adxl_ready_batch_idx，導致同一個 10ms 窗格內讀到混合批次的資料。 */
        uint8_t adxl_hiG_avail = adxl375_ok && (g_adxl_new_batch_ready != 0);
        uint8_t adxl_hiG_bidx  = g_adxl_ready_batch_idx;
        (void)adxl_hiG_avail;
        (void)adxl_hiG_bidx;

        /* baro 去重：只在 S4 的 drdy 閘真的產生新樣本時（g_baro_seq 遞增）才標記本 frame
         * 有 baro，取代舊有 (i==0||i==2) 硬編位置的假設——那假設綁死在 400Hz 陀螺/200Hz
         * baro 的舊配置，1000Hz 陀螺/50Hz baro 下位置不再對應，且原本完全沒有去重檢查
         * （50Hz ODR 若讀取率≥ODR 會拍頻重複餵同一筆，人為壓低量測噪聲）。 */
        static uint32_t s_ekf_baro_seq_seen = 0;
        uint32_t baro_seq_now = g_baro_seq;
        uint8_t  baro_new = (baro_seq_now != s_ekf_baro_seq_seen) && bmp388_ok &&
                            ((g_sensor_fault_bits & SH_BIT_BMP388) == 0U);
        float    baro_snap = baro_data.altitude;
        s_ekf_baro_seq_seen = baro_seq_now;

        // Loop to generate EKF_GYRO_PER_FRAME(10) synchronized 1000Hz gyro samples for
        // the current 10ms frame; accel(400Hz) is zero-order-held between has_accel samples.
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        int   acc_prev_ai = -1;
        for (int j = 0; j < EKF_GYRO_PER_FRAME; j++) {
            // 1. Accel index map：10 顆陀螺樣本對應 4 顆加速度樣本，ai = 0,0,0,1,1,2,2,2,3,3，
            //    has_accel 恰在 4 個 ai 值各觸發一次（新樣本到達的那一刻）。
            int     ai      = (j * EKF_ACC_PER_FRAME) / EKF_GYRO_PER_FRAME;
            uint8_t has_acc = (ai != acc_prev_ai) ? 1U : 0U;
            acc_prev_ai = ai;

            if (has_acc) {
                float raw_ax = g_bmi_acc_batches[acc_ready_idx][ai].ax * 9.80665f;
                float raw_ay = g_bmi_acc_batches[acc_ready_idx][ai].ay * 9.80665f;
                float raw_az = g_bmi_acc_batches[acc_ready_idx][ai].az * 9.80665f;
                // 重映射至 body frame (X=右, Y=前, Z=上)，依 sensor_axis.h（唯一真相來源）。
                sensor_imu_to_body(raw_ax, raw_ay, raw_az, &ax, &ay, &az);
            }
            // has_acc=0：沿用上一輪的 ax/ay/az（ZOH），供 Mahony 重力向量參考——其變化時間
            // 常數遠慢於 2.5ms 保持期，ZOH 足夠；j=0 恰為 ai=0 的新樣本，恆不會用到初值。

            // 2. Gyro sample (1000Hz direct extraction，逐顆)
            float sgx = g_bmi_gyro_batches[gyro_ready_idx][j].gx;
            float sgy = g_bmi_gyro_batches[gyro_ready_idx][j].gy;
            float sgz = g_bmi_gyro_batches[gyro_ready_idx][j].gz;

            // body 角速度 (rad/s)，與 accel 同一映射 (sensor_axis.h)。
            float gx, gy, gz;
            sensor_imu_to_body(sgx * (M_PI / 180.0f), sgy * (M_PI / 180.0f), sgz * (M_PI / 180.0f),
                               &gx, &gy, &gz);

            // 3. Assemble the sample
            EKF_Sample_t* p_sample = &g_ekf_buffers[g_ekf_active_idx].samples[g_ekf_sample_count];
            p_sample->ax = ax;
            p_sample->ay = ay;
            p_sample->az = az;
            p_sample->gx = gx;
            p_sample->gy = gy;
            p_sample->gz = gz;
            p_sample->has_accel = has_acc;

            // 真實 ISR 時戳（DWT->CYCCNT，wrap-safe cycle domain）取代原本合成的 2.5ms 網格：
            // 舊算法 frame_start_cycles+cycles_offset 在 uint32 加法時會回繞（每 ~25.6s 一次
            // dt 落回 fallback 值），且未反映 TIM6/TIM7 互相搶佔造成的實際抖動；逐顆記錄
            // ISR 完成瞬間的 CYCCNT 兩個問題一併解決。
            p_sample->timestamp_cyc = g_bmi_gyro_batches[gyro_ready_idx][j].t_cyc;

            // 4. baro：每個新樣本只在 j==0 標記一次，避免同一筆 baro 被誤判成多筆新資訊。
            if (baro_new && j == 0) {
                p_sample->has_baro = 1;
                p_sample->baro_alt = baro_snap;
            } else {
                p_sample->has_baro = 0;
                p_sample->baro_alt = 0.0f;
            }

            g_ekf_sample_count++;

            // 5. If buffer is full (EKF_BUFFER_SIZE samples), send it to the EKF playback task
            if (g_ekf_sample_count >= EKF_BUFFER_SIZE) {
                // Get pointer to full buffer
                EKF_Buffer_t* p_full_buffer = &g_ekf_buffers[g_ekf_active_idx];

                // Non-blocking queue send；put 失敗 (queue 滿) 代表 EKF_Task 消費不及，計數供觀測
                if (osMessageQueuePut(xEKFQueue, &p_full_buffer, 0, 0) != osOK) {
                    g_ekf_queue_drops++;
                }

                // Switch buffer index
                g_ekf_active_idx = !g_ekf_active_idx;
                g_ekf_sample_count = 0;
            }
        }

        /* S6：原始 IMU 二進位記錄——把本 frame 剛組裝好的 10 顆陀螺 + 4 顆加速度原始
         * LSB（sensor frame，remap 前）封裝成 128B record 塞進 SRAM 環形緩衝，供下方
         * SD 區塊倒出到 IMU_xxx.BIN。與 sd_imu_bin_active 無關——緩衝生產與 SD 寫入
         * 分離，開檔失敗時緩衝會一路累積到滿後開始丟棄計數，不影響 CSV/EKF 本身。 */
        static uint32_t s_imu_last_gyro_cyc = 0;   // 上一個 frame 最後一顆陀螺的 t_cyc；0=尚無上一筆
        if ((uint32_t)(s_imu_len + sizeof(ImuRawRecord_t)) <= sizeof(s_imu_buf)) {
            ImuRawRecord_t rec;
            rec.magic[0] = IMU_RAW_LOG_MAGIC0;
            rec.magic[1] = IMU_RAW_LOG_MAGIC1;
            rec.ver      = IMU_RAW_LOG_VERSION;
            rec.n_gyro   = IMU_RAW_LOG_N_GYRO;
            rec.n_acc    = IMU_RAW_LOG_N_ACC;

            uint8_t rec_flags = 0U;
            if (s_imu_bin_drop_count != 0U) { rec_flags |= IMU_RAW_LOG_FLAG_BUF_DROP; s_imu_bin_drop_count = 0U; }
            if (baro_new)                    { rec_flags |= IMU_RAW_LOG_FLAG_BARO_NEW; }
            if (g_ekf_queue_drops != s_ekf_queue_drops_seen) { rec_flags |= IMU_RAW_LOG_FLAG_EKF_QUEUE_DROP; }
            s_ekf_queue_drops_seen = g_ekf_queue_drops;
            rec.flags = rec_flags;

            rec.seq       = s_imu_seq++;
            rec.t_cyc     = g_bmi_gyro_batches[gyro_ready_idx][0].t_cyc;
            rec.fsm_state = (uint8_t)current_fsm_state;
            rec._rsv      = 0U;
            rec.baro_temp_c_x100 = (int16_t)(baro_data.temperature * 100.0f);
            rec.baro_press_pa    = baro_new ? (uint32_t)baro_data.pressure : 0U;

            for (int gj = 0; gj < (int)IMU_RAW_LOG_N_GYRO; gj++) {
                rec.gyro[gj][0] = g_bmi_gyro_batches[gyro_ready_idx][gj].gyro_x_raw;
                rec.gyro[gj][1] = g_bmi_gyro_batches[gyro_ready_idx][gj].gyro_y_raw;
                rec.gyro[gj][2] = g_bmi_gyro_batches[gyro_ready_idx][gj].gyro_z_raw;

                uint32_t prev_cyc = (gj == 0) ? s_imu_last_gyro_cyc
                                               : g_bmi_gyro_batches[gyro_ready_idx][gj - 1].t_cyc;
                uint32_t dcyc = (prev_cyc == 0U && gj == 0)
                                    ? 0U   // 首筆記錄：無上一顆可比較，見標頭欄位註解
                                    : (g_bmi_gyro_batches[gyro_ready_idx][gj].t_cyc - prev_cyc);
                rec.gyro_dt_q[gj] = (uint16_t)(dcyc >> 3);
            }
            s_imu_last_gyro_cyc = g_bmi_gyro_batches[gyro_ready_idx][IMU_RAW_LOG_N_GYRO - 1].t_cyc;

            for (int ai2 = 0; ai2 < (int)IMU_RAW_LOG_N_ACC; ai2++) {
                rec.acc[ai2][0] = g_bmi_acc_batches[acc_ready_idx][ai2].accel_x_raw;
                rec.acc[ai2][1] = g_bmi_acc_batches[acc_ready_idx][ai2].accel_y_raw;
                rec.acc[ai2][2] = g_bmi_acc_batches[acc_ready_idx][ai2].accel_z_raw;
            }
            int32_t acc_off_cyc = (int32_t)(g_bmi_acc_batches[acc_ready_idx][0].t_cyc - rec.t_cyc);
            rec.acc_t_off_q = (int16_t)(acc_off_cyc / 8);

            rec.crc16 = crc16_ccitt_false((const uint8_t *)&rec, (uint16_t)(sizeof(rec) - 2U));

            memcpy(&s_imu_buf[s_imu_len], &rec, sizeof(rec));
            s_imu_len = (uint16_t)(s_imu_len + sizeof(rec));
        } else {
            s_imu_bin_drop_count++;   // 緩衝已滿（消費端 SD 寫入跟不上）：丟棄本筆，下一筆記錄標記 BUF_DROP
        }

        // Also update standard imu_data structure for standard telemetry and logging
        imu_data.accel_x_raw = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].accel_x_raw;
        imu_data.accel_y_raw = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].accel_y_raw;
        imu_data.accel_z_raw = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].accel_z_raw;
        
        // 重映射至 body frame (sensor_axis.h)；與 EKF 饋入完全一致。
        float raw_ax         = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].ax;
        float raw_ay         = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].ay;
        float raw_az         = g_bmi_acc_batches[acc_ready_idx][BMI_ACC_BATCH_SIZE - 1].az;
        sensor_imu_to_body(raw_ax, raw_ay, raw_az, &imu_data.ax, &imu_data.ay, &imu_data.az);

        /* --- 飛行滾動極值：最大合加速度（見 g_max_acc_g 宣告處說明）---
         * ★逐筆掃描整批，不是只看上面那筆 [BATCH_SIZE-1]：上面那行是「抽最後一筆給遙測/
         *   顯示用」的降頻，若極值也跟著只看最後一筆，400Hz 的取樣會被抽成 100Hz，峰值
         *   照樣漏掉——而峰值 G 正是這個欄位存在的唯一理由。
         * 模長對旋轉不變，故直接用 sensor frame 原始值算，不必先轉 body frame（省 4 次
         *   重映射；本區塊在 1kHz 主迴圈內，成本要看得見）。
         * 不設狀態閘：加速度峰值本來就發生在點火瞬間（FSM 可能還在 PAD_ARMED），
         *   以 ARM 時歸零作為起算點即可。 */
        for (uint8_t mi = 0; mi < BMI_ACC_BATCH_SIZE; mi++) {
            float sx = g_bmi_acc_batches[acc_ready_idx][mi].ax;
            float sy = g_bmi_acc_batches[acc_ready_idx][mi].ay;
            float sz = g_bmi_acc_batches[acc_ready_idx][mi].az;
            float mag = sqrtf(sx * sx + sy * sy + sz * sz);
            if (mag > g_max_acc_g) g_max_acc_g = mag;
        }

        imu_data.gyro_x_raw  = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gyro_x_raw;
        imu_data.gyro_y_raw  = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gyro_y_raw;
        imu_data.gyro_z_raw  = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gyro_z_raw;
        
        float raw_gx         = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gx;
        float raw_gy         = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gy;
        float raw_gz         = g_bmi_gyro_batches[gyro_ready_idx][BMI_GYRO_BATCH_SIZE - 1].gz;
        sensor_imu_to_body(raw_gx, raw_gy, raw_gz, &imu_data.gx, &imu_data.gy, &imu_data.gz);

        /* P0-D：BMI088 健康餵入（accel+gyro raw 簽章；卡死/失流偵測） */
        sensor_mon_feed(&g_mon_bmi088, HAL_GetTick(),
                        sensor_sig3(imu_data.accel_x_raw,
                                    imu_data.accel_y_raw,
                                    imu_data.accel_z_raw) ^
                        sensor_sig3(imu_data.gyro_x_raw,
                                    imu_data.gyro_y_raw,
                                    imu_data.gyro_z_raw),
                        1U /* BMI088 無範圍檢查 */);
    }

    /* === ADXL375 Ping-Pong 雙緩衝區批次數據提取 (400 Hz 中斷在背後默默採樣並填充) === */
    if (adxl375_ok && g_adxl_new_batch_ready) {
        g_adxl_new_batch_ready = 0;
        uint8_t ready_idx = g_adxl_ready_batch_idx;
        
        // 自 32 筆最新採樣數據中，提取最後一筆最新值供即時遙測傳送與顯示
        highg_data.x_raw = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].x_raw;
        highg_data.y_raw = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].y_raw;
        highg_data.z_raw = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].z_raw;
        
        float raw_adxl_ax = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].ax;
        float raw_adxl_ay = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].ay;
        float raw_adxl_az = g_adxl_batches[ready_idx][ADXL_BATCH_SIZE - 1].az;
        sensor_highg_to_body(raw_adxl_ax, raw_adxl_ay, raw_adxl_az,
                             &highg_data.ax, &highg_data.ay, &highg_data.az);

        /* P0-D：ADXL375 健康餵入（±200g 量程，>250g 持續 0.5s 視為範圍失效） */
        {
            uint8_t adxl_range_ok = (fabsf(highg_data.ax) <= 250.0f &&
                                     fabsf(highg_data.ay) <= 250.0f &&
                                     fabsf(highg_data.az) <= 250.0f) ? 1U : 0U;
            sensor_mon_feed(&g_mon_adxl375, HAL_GetTick(),
                            sensor_sig3(highg_data.x_raw,
                                        highg_data.y_raw,
                                        highg_data.z_raw),
                            adxl_range_ok);
        }
        
        // 提示：如需寫入 SD 卡，可在此處直接寫入整個 g_adxl_batches[ready_idx] 共 ADXL_BATCH_SIZE(4) 筆資料，即為 400 Hz HIL 實時存檔
    }

    /* === FSM 100Hz 定時執行（P0-A：自 BMI088 批次條件塊移出） ===
     * 舊版在 if (bmi088_ok && batch_ready) 內呼叫 → BMI088 初始化失敗或飛行中
     * 批次斷流時，FSM（含後續 P0-B 失效保護計時器）會整個停擺。
     * 改為無條件 tick 定時：感測器斷流時 FSM 仍持續運作（吃最後快照 + EKF 狀態）。 */
    if (tick % FSM_STEP_PERIOD_MS == 0) {
        FSM_Update();
    }

#if FEATURE_LINK
    /* 板間鏈路：每 LINK_TX_PERIOD_MS 廣播自身狀態（對稱獨立：供地面監看與加法協同，
     * 不抑制對端開傘）。 */
    if (tick % LINK_TX_PERIOD_MS == 0) {
        Link_PublishTick();
    }
#endif

    /* === GPS（USART6 NMEA）：每迴圈嘗試消化一整句 ===
     * 無整句就緒時僅檢查旗標，極輕量；循環 DMA + IDLE 事件已於背景組句。
     * 每迴圈呼叫可即時排空單句緩衝，避免連續輸出時被新句覆蓋。 */
    GPS_Update();

    /* === GPS → EKF：偵測到新的有效定位才提交一次水平位置量測 ===
     * 以 last_fix_tick 變化判斷新定位，避免每迴圈 (1kHz) 重複提交同一筆。
     * EKF_SubmitGPS 內部負責原點鎖定與 ENU 換算；發射台 (ZUPT) 階段提交的
     * 定位會於 EKF_Task 內被丟棄，僅飛行中真正融合。 */
    {
        const GPS_Data_t *gfix = GPS_GetData();
        static uint32_t last_gps_fused_tick = 0;
        if (gfix->fix_valid && gfix->last_fix_tick != last_gps_fused_tick) {
            last_gps_fused_tick = gfix->last_fix_tick;
            EKF_SubmitGPS(gfix->lat_1e6, gfix->lon_1e6, gfix->satellites, gfix->hacc_mm);
        }
    }

    /* === MMC5983MA 地磁計 @ 100 Hz 軟體 I2C 讀取 (每 10ms 一次讀取暫存器) === */
    if (mag_ok && (tick % 10 == 0)) {
        MMC5983_Read_Continuous();
    }

    /* === MMC5983MA 地磁計 EKF 融合 @ 10 Hz (每 100ms 提交一次軟體濾波後的純淨磁場資料) === */
    if (mag_ok && (tick % 100 == 0)) {
        float mx = 0.0f, my = 0.0f, mz = 0.0f;
        MMC5983_GetFilteredGauss(&mx, &my, &mz);
        /* 重映射至 body frame (sensor_axis.h)：mag X->-Y, Y->X, Z->-Z */
        float mx_body, my_body, mz_body;
        sensor_mag_to_body(mx, my, mz, &mx_body, &my_body, &mz_body);
        EKF_SubmitMag(mx_body, my_body, mz_body);
    }
    /* 每 10 秒重做一次 SET/RESET 橋偏校準以補溫漂；飛行中不做（地磁融合僅限發发射台）。
     * 暫時註解以進行排查，防止從 Flash 載入的硬鐵校正被覆蓋。 */
    // if (mag_ok && !EKF_in_flight && (tick % 10000 == 0) && (tick > 0)) {
    //     MMC5983_Recalibrate();
    // }

    /* === BMP388 @ 100Hz 輪詢 + drdy 閘 (晶片 ODR=50Hz，8x 壓力過採樣) ===
     * 輪詢率(100Hz) > ODR(50Hz)：保證不漏任何一顆新轉換結果。
     * drdy 閘：晶片內部時鐘與 MCU 各自獨立，讀取率若等於或高於 ODR 卻不看 drdy，
     * 會偶爾在同一顆結果上拍頻重複讀到 → 人為壓低量測噪聲、EKF/VF 誤判有新資訊。
     * g_baro_seq 只在真的讀到新樣本時遞增，供 EKF 組裝迴圈判斷本 frame 是否有新 baro。 */
    if (tick % 10 == 0 && bmp388_ok) {
        // 在任務讀取 BMP388 (SPI1) 期間，暫時關閉 TIM3 中斷，防止高優先權中斷搶佔 SPI1 匯流排
        __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
        uint8_t baro_rdy = BMP388_IsDataReady();
        if (baro_rdy) {
            BMP388_ReadData(&hspi1, &baro_data);
        }
        __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

        if (baro_rdy) {
            g_baro_seq++;
            RATE_TICK_BMP388();   /* 統計 BMP388 實際採樣率 → g_sampling_rate.bmp388.rate_hz（現應報 ~50Hz） */

            /* P0-D：BMP388 健康餵入（氣壓 26–110 kPa、溫度 −40~85°C；
             * 簽章取整數 Pa —— 卡死的感測器回傳逐位元相同的浮點值） */
            uint8_t bmp_range_ok = (baro_data.pressure    >= 26000.0f &&
                                    baro_data.pressure    <= 110000.0f &&
                                    baro_data.temperature >= -40.0f &&
                                    baro_data.temperature <=  85.0f) ? 1U : 0U;
            sensor_mon_feed(&g_mon_bmp388, HAL_GetTick(),
                            (int32_t)baro_data.pressure, bmp_range_ok);
        }
    }

    /* === Flash Ring Buffer 寫入：依 FSM 狀態分級寫入速率（改善項目 F） ===
     * Flash 在 SPI3，感測器在 SPI1，兩者獨立，無需停中斷。
     *   INIT/PAD（地面：初始化/待命，尚未武裝）：完全不寫入 —— 整池預算保留給飛行；
     *     池子已在開機時一次擦滿（FlashRing_InitEx），此階段不消耗。
     *   PAD_ARMED（已武裝待發）：1 Hz —— 記錄武裝後、起飛前的地面狀態（電壓/姿態/
     *     待命時間等），消耗量遠小於 100Hz 飛行段，武裝到起飛的等待時間內不影響
     *     飛行段可用池容量（見 flash_ring_math.h 發射檢核表）。
     *   BOOST..MAIN_DEPLOY（飛行各相位）：100 Hz —— 提高動態階段時間解析度，不變。
     *   LANDED（著陸後）：1 Hz —— 長時間尋標等待，大幅省 Flash 壽命
     * 飛行態（含 PAD_ARMED，見 FlashRing_SetEraseAllowed 呼叫處）禁止同步滾動擦除，
     * 僅消耗開機預擦好的 FLASH_RING_PREERASE_TARGET 池；限制改為「飛行時間是否超出
     * 該池容量」，見 flash_ring_math.h 內對應計算與發射檢核表要求。 */
    uint8_t  ring_enabled;
    uint32_t ring_period_ms;
    if (current_fsm_state == STATE_LANDED) {
        ring_enabled   = 1U;
        ring_period_ms = 1000;   /* 1 Hz */
    } else if (current_fsm_state >= STATE_BOOST && current_fsm_state <= STATE_MAIN_DEPLOY) {
        ring_enabled   = 1U;
        ring_period_ms = 10;     /* 100 Hz，飛行時不變 */
    } else if (current_fsm_state == STATE_PAD_ARMED) {
        ring_enabled   = 1U;
        ring_period_ms = 1000;   /* 1 Hz，武裝後開始記錄 */
    } else {
        ring_enabled   = 0U;     /* INIT/PAD：尚未武裝，整池保留給飛行 */
        ring_period_ms = 1000;   /* 值不重要（ring_enabled=0 時本迴圈不會進入寫入分支） */
    }
    if (g_flash_logging_active && ring_enabled && (tick % ring_period_ms == 0)) {
        /* Flash Ring Buffer：將目前感測器與估計數據打包為 128-byte 格式寫入 */
        FlashRingPacket_t ring_pkt = {0};
        ring_pkt.flight_id   = g_flash_current_flight_id;
        ring_pkt.tick_ms     = HAL_GetTick();
        
        // 抓取系統開傘狀態旗標
        uint8_t sys_flags = 0;
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET) sys_flags |= 0x01; // 副傘已點火
        if (g_main_deployed)                                      sys_flags |= 0x02; // 主傘已釋放（PD14 曾拉高，鎖存）
        ring_pkt.flags       = sys_flags;
        
        ring_pkt.bat_voltage_mv = g_bat_voltage_mv;   /* P0-E：改用 1Hz 快取（ADC 讀取移出 50Hz 寫入路徑） */
        
        ring_pkt.bmi_ax      = imu_data.accel_x_raw;
        ring_pkt.bmi_ay      = imu_data.accel_y_raw;
        ring_pkt.bmi_az      = imu_data.accel_z_raw;
        ring_pkt.bmi_gx      = imu_data.gyro_x_raw;
        ring_pkt.bmi_gy      = imu_data.gyro_y_raw;
        ring_pkt.bmi_gz      = imu_data.gyro_z_raw;
        
        ring_pkt.adxl_x      = highg_data.x_raw;
        ring_pkt.adxl_y      = highg_data.y_raw;
        ring_pkt.adxl_z      = highg_data.z_raw;
        
        ring_pkt.baro_temp_c_x100 = (int16_t)(baro_data.temperature * 100.0f);
        ring_pkt.baro_press_pa    = (uint32_t)(baro_data.pressure);
        ring_pkt.baro_alt_cm      = (int32_t)(baro_data.altitude * 100.0f);
        
        EKF_State_t sd_ekf = EKF_GetState();
        ring_pkt.ekf_pos_z_cm     = (int32_t)(sd_ekf.pos_z * 100.0f);
        ring_pkt.ekf_vel_z_cms    = (int32_t)(sd_ekf.vel_z * 100.0f);
        ring_pkt.ekf_q0           = sd_ekf.q[0];
        ring_pkt.ekf_q1           = sd_ekf.q[1];
        ring_pkt.ekf_q2           = sd_ekf.q[2];
        ring_pkt.ekf_q3           = sd_ekf.q[3];
        
        const GPS_Data_t *gps_pkt = GPS_GetData();
        if (gps_pkt->fix_valid) {
            ring_pkt.gps_lat      = gps_pkt->lat_1e6;
            ring_pkt.gps_lon      = gps_pkt->lon_1e6;
            ring_pkt.gps_alt_m    = (int16_t)gps_pkt->altitude_m;
            ring_pkt.gps_spd_cms  = (int16_t)(gps_pkt->speed_mps * 100.0f);
            ring_pkt.gps_sats     = gps_pkt->satellites;
            ring_pkt.gps_fix      = gps_pkt->fix_valid;
        }
        
        ring_pkt.fsm_state   = (uint8_t)current_fsm_state;

        /* ★熱重啟用：起飛後經過時間。tick_ms 是「開機以來」的絕對 tick，重啟後歸零，
         * 跨重啟完全無法用來判斷「這筆封包距離起飛多久」——真實飛行在發射台上通電
         * 校準/等待往往就超過 60s，若拿 tick_ms 去比 FSM_HOTSTART_MAX_TICK_MS 會把
         * 每一次空中熱重啟都誤判成「陳舊資料」而打回 PAD。未起飛的階段寫 0。 */
        ring_pkt.flight_tick_ms =
            ((current_fsm_state >= STATE_BOOST) && (flight_start_tick != 0U))
                ? (HAL_GetTick() - flight_start_tick) : 0U;

        /* ★熱重啟用：飛行摘要（最大高度/速度/G + 實際開傘高度）。這些平時只活在 RAM，
         * 飛行中一次 brownout/IWDG 重置就全歸零——而它們正是火箭無法回收時唯一能知道
         * 「飛多高／多快／幾 G／傘在什麼高度開」的資料。逐筆重複寫入（用的是原本
         * reserved 的空間，封包仍 128B、成本 0），還原見 ExtremaTrack_Restore()。
         * 單位/哨兵與下鏈 telemetry.h 同名欄位完全一致。 */
        ring_pkt.max_alt_m    = (g_max_alt_m > 0.0f)
                                ? (uint16_t)((g_max_alt_m > 65535.0f) ? 65535.0f : g_max_alt_m) : 0U;
        ring_pkt.max_vel_ms   = sat_alt_i16(g_max_vel_ms);
        ring_pkt.max_acc_cg   = (uint16_t)((g_max_acc_g * 100.0f > 65535.0f) ? 65535.0f
                                                                            : (g_max_acc_g * 100.0f));
        ring_pkt.drogue_alt_m = g_drogue_alt_m;
        ring_pkt.main_alt_m   = g_main_alt_m;


#if FEATURE_FLASH
        /* 先前完全忽略回傳值 → 寫入失敗靜默無感。現在計數；飛行態
         * （BOOST..MAIN_DEPLOY）刻意不 printf（UART ~9ms/行會延遲開傘），
         * 僅地面態限流列印，好在發射前就抓到 flash 異常。 */
        if (FlashRing_WritePacket(&ring_pkt) != W25QXX_OK) {
            g_flash_write_fail_count++;
            uint8_t in_flight = (current_fsm_state >= STATE_BOOST &&
                                 current_fsm_state <= STATE_MAIN_DEPLOY);
            if (!in_flight && (g_flash_write_fail_count % 20U) == 1U) {
                printf("[FLASH] WritePacket FAILED, fail_count=%lu\r\n",
                       (unsigned long)g_flash_write_fail_count);
            }
        }
#endif
    }

    /* === P0-E：1Hz 電池電壓 ADC 讀取（自 50Hz Flash 寫入路徑移出；唯一 ADC 讀取點） === */
    if (tick % 1000 == 250) {
        g_bat_voltage_mv = ADC_Read_Battery_mv();
    }

#if FEATURE_FLASH
    /* === P0-E：INIT/PAD 期背景預擦安全網（每 0.5s 擦 1 個 sector，單次最壞 ~400ms < IWDG 2.05s）===
     * 池子主要由開機一次性 bulk 預擦（FlashRing_InitEx，main.c 開機序列）擦滿，本迴圈只是
     * 補漏的安全網（例如 EraseAll 上行指令清池後）。★不含 PAD_ARMED：FlashRing_PreEraseOne()
     * 本身不查 s_ring_erase_allowed，若把武裝態也納入，武裝後仍可能觸發同步 sector 擦除
     * （~50~400ms 阻塞），與「武裝後隨時可能起飛」的假設衝突——見上方 FlashRing_SetEraseAllowed
     * 呼叫處的禁止窗口（PAD_ARMED..MAIN_DEPLOY）。 */
    if (tick % 500 == 250 && current_fsm_state <= STATE_PAD) {
        if (FlashRing_PreEraseOne() == 0U) {
            printf("[FLASH] pool=%lu/%u\r\n",
                   (unsigned long)FlashRing_GetPoolSectors(),
                   (unsigned)FLASH_RING_PREERASE_TARGET);
        }
    }
#endif

    /* === 每 100ms：10 Hz UART 遙測輸出 (減少 CPU 阻塞開銷)
     * P0-E：[TEST ONLY] 桌面測試模式：飛行階段不抑制 UART 輸出，以便即時觀測氣壓曲線。
     * 重要：實際起飛前必須改回 !(current_fsm_state >= STATE_BOOST && current_fsm_state <= STATE_MAIN_DEPLOY) 避免 printf 阻塞。 === */
    if (tick % 100 == 0) {
        /* UART 遙測輸出
         * 格式: bmi_ax(mg),bmi_ay(mg),bmi_az(mg),adxl_ax(mg),adxl_ay(mg),adxl_az(mg),
         *        temp(*100 degC),press(Pa),alt(cm)
         */
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               (int)(imu_data.ax * 1000.0f),
               (int)(imu_data.ay * 1000.0f),
               (int)(imu_data.az * 1000.0f),
               (int)(highg_data.ax * 1000.0f),
               (int)(highg_data.ay * 1000.0f),
               (int)(highg_data.az * 1000.0f),
               (int)(baro_data.temperature * 100.0f),
               (int)(baro_data.pressure),
               (int)(baro_data.altitude * 100.0f));
    }



    /* === 每 500ms 翻轉 LED 一次，形成溫和的 1 Hz 視覺心跳 (0.5s 亮 / 0.5s 暗) === */
    if (tick % 500 == 0) {
        HAL_GPIO_TogglePin(LED_SYS_GPIO_Port, LED_SYS_Pin);
    }

    /* === P0-D：1Hz 健康診斷行（sens=SH_BIT_BMI088/ADXL375/BMP388、ekf=EKF_HB_*、fsm=FlightState） === */
    if (tick % 1000 == 0) {
        printf("[HEALTH] sens=0x%02X ekf=0x%02X fsm=%u\r\n",
               (unsigned)g_sensor_fault_bits, (unsigned)EKF_GetHealthBits(), (unsigned)current_fsm_state);
    }

    /* === P1：0.2Hz 任務堆疊水位（osThreadGetStackSpace = 歷史最低剩餘 bytes）。
     * 值逼近 0 = 溢位前兆；HIL 斷言每個任務 ≥256 bytes 餘裕。 === */
    if (tick % 5000 == 0) {
        printf("[STACK] main=%lu ekf=%lu lora=%lu\r\n",
               (unsigned long)(defaultTaskHandle ? osThreadGetStackSpace(defaultTaskHandle) : 0),
               (unsigned long)(g_ekf_task_handle ? osThreadGetStackSpace(g_ekf_task_handle) : 0),
               (unsigned long)(g_lora_task_handle ? osThreadGetStackSpace(g_lora_task_handle) : 0));
    }

    /* === 每 10 秒：PAD 靜止階段發射前重要參數配置廣播 === */
    if (tick % 10000 == 0 && (current_fsm_state == STATE_INIT || current_fsm_state == STATE_PAD || current_fsm_state == STATE_PAD_ARMED)) {
#if FLIGHT_PROFILE_ELEVATOR
        const char *prof_msg = "ELEVATOR_TEST (WARNING: ELEVATOR PROFILE ACTIVE! DO NOT LAUNCH FOR REAL FLIGHT!)";
#else
        const char *prof_msg = "REAL_FLIGHT (OK FOR LAUNCH)";
#endif
        printf("\r\n==================== [PAD CONFIG BROADCAST (10s)] ====================\r\n");
        printf("[PAD_CFG] Role: %s | Firmware: %s | FSM State: %u (%s)\r\n",
               IS_PRIMARY ? "PRIMARY" : (IS_BACKUP ? "BACKUP" : "GROUND"),
               FIRMWARE_VERSION,
               (unsigned)current_fsm_state,
               (current_fsm_state == STATE_PAD_ARMED) ? "PAD_ARMED" : "PAD");
        printf("[PAD_CFG] Flight Profile: %s\r\n", prof_msg);
        printf("[PAD_CFG] Parachute Config: MainAlt=%.1fm | ApogeeFailsafe=%.1fs | MainWD=%.1fs\r\n",
               (double)FSM_FB_MAIN_ALT_M,
               (double)((IS_PRIMARY ? FSM_FAILSAFE_APOGEE_MS : FSM_FAILSAFE_APOGEE_BACKUP_MS) / 1000.0f),
               (double)(FSM_MAIN_WATCHDOG_MS / 1000.0f));
#if FEATURE_LORA
        printf("[PAD_CFG] LoRa Config: E22 433MHz Active | E80 920MHz Active\r\n");
#else
        printf("[PAD_CFG] LoRa Config: N/A (FEATURE_LORA=0)\r\n");
#endif
        printf("======================================================================\r\n\r\n");
    }

    /* === 電梯測試 profile 醒目提示（1Hz，僅地面狀態）：只要本板或對端（板間鏈路中繼）
     * 仍是電梯測試 profile 編譯，就大聲洗版警告——雙板換板/重燒最容易漏掉其中一片，
     * 靠這行避免忘記重燒就搬去真實發射。飛行中（BOOST..LANDED）刻意不印，理由同上方
     * PAD_CFG 區塊：printf 在飛行熱路徑會拖慢主迴圈（見本函式開頭註解）。 === */
    if (tick % 1000 == 0 && (current_fsm_state == STATE_INIT || current_fsm_state == STATE_PAD || current_fsm_state == STATE_PAD_ARMED)) {
        uint8_t self_elevator = 0U;
#if FLIGHT_PROFILE_ELEVATOR
        self_elevator = 1U;
#endif
        uint8_t peer_elevator = 0U;
#if FEATURE_LINK
        peer_elevator = (Link_GetPeer()->profile_flags & TELEM_PROFILE_SELF_ELEVATOR) ? 1U : 0U;
#endif
        if (self_elevator || peer_elevator) {
            printf("\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            printf("!!! [ELEVATOR_TEST_WARNING] %s%s%s STILL ON ELEVATOR-TEST PROFILE !!!\r\n",
                   self_elevator ? "THIS BOARD" : "",
                   (self_elevator && peer_elevator) ? " + " : "",
                   peer_elevator ? "PEER BOARD" : "");
            printf("!!! DO NOT LAUNCH FOR REAL FLIGHT -- RE-FLASH WITH FLIGHT_PROFILE_ELEVATOR=0 !!!\r\n");
            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n\r\n");
        }
    }

#if FEATURE_FLASH
    /* === ★2026-07-31：Flash 池未達標橫幅（1Hz，僅地面狀態）===
     * 開機不再自動擦除，所以「該擦而沒擦」不會有任何自然徵兆——沒有這行提醒，使用者只會
     * 在按下 ARM 之後才發現被擋。與電梯 profile 橫幅同一套節奏/條件（飛行中不印，理由同上：
     * printf 在飛行熱路徑會拖慢主迴圈）。旗標每輪重算，擦完即自動消失。
     * 注意 fail-open 條件與 FSM_Update 組 flash_pool_ready 時一致：flash 沒偵測到或已停止
     * 記錄時不擋 ARM，也就不該再喊「需要擦除」。 */
    if (tick % 1000 == 500 && (current_fsm_state == STATE_INIT ||
                               current_fsm_state == STATE_PAD  ||
                               current_fsm_state == STATE_PAD_ARMED)) {
        uint32_t pool_now = FlashRing_GetPoolSectors();
        g_flash_need_erase = (flash_ring_hw_ok && g_flash_logging_active &&
                              pool_now < FLASH_RING_PREERASE_TARGET) ? 1U : 0U;
        if (g_flash_need_erase) {
            printf("\r\n**********************************************************************\r\n");
            printf("*** [FLASH_NOT_READY] 已擦池 %lu/%u sectors —— ARM 已被擋下 ***\r\n",
                   (unsigned long)pool_now, (unsigned)FLASH_RING_PREERASE_TARGET);
            printf("*** 飛前請下 `flash erase`（整環全擦 ~3min，正規流程）              ***\r\n");
            printf("*** 或 `flash pool`（只補不足部分，bench 省時用）                   ***\r\n");
            printf("*** USB console 與 LoRa 上行文字指令皆可                            ***\r\n");
            printf("**********************************************************************\r\n\r\n");
        }
    }
#endif

    /* === FSM 飛行狀態 LED（每 100ms 更新）—— 電梯/台面測試用肉眼確認狀態機推進 ===
     * STAT1 (PE3) / STAT2 (PE4) 依 current_fsm_state 編碼（active-high；SYS=心跳另計）：
     *   PAD/INIT : 兩顆全暗            BOOST : STAT1 閃
     *   COAST    : STAT1 常亮          APOGEE: STAT1 亮 + STAT2 閃（PD13 已點副傘）
     *   DESCENT  : STAT2 閃            MAIN  : STAT2 常亮（PD14 已拉高釋放主傘）
     *   LANDED   : 兩顆一起閃
     * （GPS/磁力計狀態改看 [HEALTH]/[GPS] 序列輸出，不再佔這兩顆燈。） */
    if (tick % 100 == 0) {
        const GPIO_PinState blink = ((tick / 250) % 2 == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET; /* ~2Hz */
        GPIO_PinState s1 = GPIO_PIN_RESET, s2 = GPIO_PIN_RESET;
        switch (current_fsm_state) {
            case STATE_BOOST:       s1 = blink;          s2 = GPIO_PIN_RESET; break;
            case STATE_COAST:       s1 = GPIO_PIN_SET;   s2 = GPIO_PIN_RESET; break;
            case STATE_DEPLOY_DROGUE: s1 = GPIO_PIN_SET; s2 = blink;          break; /* 馬達運轉中 */
            case STATE_APOGEE:      s1 = GPIO_PIN_SET;   s2 = blink;          break; /* 僅存續 1 週期 */
            case STATE_DESCENT:     s1 = GPIO_PIN_RESET; s2 = blink;          break;
            case STATE_MAIN_DEPLOY: s1 = GPIO_PIN_RESET; s2 = GPIO_PIN_SET;   break;
            case STATE_LANDED:      s1 = blink;          s2 = blink;          break;
            default:                s1 = GPIO_PIN_RESET; s2 = GPIO_PIN_RESET; break; /* INIT/PAD */
        }
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, s1);              /* STAT1 / PE3 */
        HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin, s2); /* STAT2 / PE4 */
    }



    uint32_t loop_end_cycles = DWT->CYCCNT;
    uint32_t loop_elapsed_cycles = loop_end_cycles - loop_start_cycles;

    /* P0-E：單迴圈最大耗時水位（HIL 斷言飛行態 < 20ms 的依據） */
    static uint32_t loop_max_cycles = 0;
    if (loop_elapsed_cycles > loop_max_cycles) {
        loop_max_cycles = loop_elapsed_cycles;
    }

    uint32_t current_tick = HAL_GetTick();
    if (main_task_last_calc_tick == 0) {
        main_task_last_calc_tick = current_tick;
    }
    main_task_accumulated_cycles += loop_elapsed_cycles;
    if (current_tick - main_task_last_calc_tick >= 1000) {
        uint32_t elapsed_ms_time = current_tick - main_task_last_calc_tick;
        uint32_t total_possible_cycles = elapsed_ms_time * (SystemCoreClock / 1000);
        if (total_possible_cycles > 0) {
            g_main_task_cpu_usage = (float)main_task_accumulated_cycles / (float)total_possible_cycles * 100.0f;
        }
        /* P0-E：1Hz 印出本秒最大單迴圈耗時（µs）並重置水位 */
        printf("[LOOP] max_us=%lu\r\n",
               (unsigned long)(loop_max_cycles / (SystemCoreClock / 1000000U)));
        loop_max_cycles = 0;
        main_task_accumulated_cycles = 0;
        main_task_last_calc_tick = current_tick;
    }

    tick++;
    wake_tick += 1U;
    osDelayUntil(wake_tick);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM3)
  {
      if (adxl375_ok)
      {
          ADXL375_Data_t sensor_data;
          // 直接在中斷中以極速阻塞讀取 ADXL375 (SPI1, ~5.6us)
          if (ADXL375_ReadData(&hspi1, &sensor_data) == HAL_OK)
          {
              uint8_t current_batch = g_adxl_active_batch;
              g_adxl_batches[current_batch][g_adxl_sample_count].x_raw = sensor_data.x_raw;
              g_adxl_batches[current_batch][g_adxl_sample_count].y_raw = sensor_data.y_raw;
              g_adxl_batches[current_batch][g_adxl_sample_count].z_raw = sensor_data.z_raw;
              g_adxl_batches[current_batch][g_adxl_sample_count].ax = sensor_data.ax;
              g_adxl_batches[current_batch][g_adxl_sample_count].ay = sensor_data.ay;
              g_adxl_batches[current_batch][g_adxl_sample_count].az = sensor_data.az;

              g_adxl_sample_count++;
              
              // 在中斷內進行採樣率累加，回報精準的 400 Hz 物理採樣頻率
              RATE_TICK_ADXL375();

              if (g_adxl_sample_count >= ADXL_BATCH_SIZE)
              {
                  g_adxl_ready_batch_idx = current_batch;
                  g_adxl_active_batch = !current_batch;
                  g_adxl_sample_count = 0;
                  g_adxl_new_batch_ready = 1;
              }
          }
      }
  }
  else if (htim->Instance == TIM6)
  {
      if (bmi088_ok)
      {
          BMI088_Accel_t acc_raw;
          // TIM6 400 Hz 中斷只讀取 BMI088 加速度計 (SPI2)
          if (BMI088_ReadAccel(&hspi2, &acc_raw) == HAL_OK)
          {
              uint8_t current_batch = g_bmi_acc_active_batch;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].accel_x_raw = acc_raw.accel_x_raw;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].accel_y_raw = acc_raw.accel_y_raw;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].accel_z_raw = acc_raw.accel_z_raw;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].ax          = acc_raw.ax;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].ay          = acc_raw.ay;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].az          = acc_raw.az;
              g_bmi_acc_batches[current_batch][g_bmi_acc_sample_count].t_cyc       = DWT->CYCCNT;

              g_bmi_acc_sample_count++;

              // 累加 Accel 採樣計數
              RATE_TICK_BMI088_ACC();

              if (g_bmi_acc_sample_count >= BMI_ACC_BATCH_SIZE)
              {
                  g_bmi_acc_ready_batch_idx = current_batch;
                  g_bmi_acc_active_batch = !current_batch;
                  g_bmi_acc_sample_count = 0;
                  g_bmi_acc_new_batch_ready = 1;
              }
          }
      }
  }
  else if (htim->Instance == TIM7)
  {
      if (bmi088_ok)
      {
          BMI088_Gyro_t gyro_raw;
          // TIM7 1000 Hz 中斷只讀取 BMI088 陀螺儀 (SPI2)
          if (BMI088_ReadGyro(&hspi2, &gyro_raw) == HAL_OK)
          {
              uint8_t current_batch = g_bmi_gyro_active_batch;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gyro_x_raw = gyro_raw.gyro_x_raw;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gyro_y_raw = gyro_raw.gyro_y_raw;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gyro_z_raw = gyro_raw.gyro_z_raw;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gx         = gyro_raw.gx;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gy         = gyro_raw.gy;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].gz         = gyro_raw.gz;
              g_bmi_gyro_batches[current_batch][g_bmi_gyro_sample_count].t_cyc      = DWT->CYCCNT;

              g_bmi_gyro_sample_count++;

              // 累加 Gyro 採樣計數
              RATE_TICK_BMI088_GYRO();

              if (g_bmi_gyro_sample_count >= BMI_GYRO_BATCH_SIZE)
              {
                  g_bmi_gyro_ready_batch_idx = current_batch;
                  g_bmi_gyro_active_batch = !current_batch;
                  g_bmi_gyro_sample_count = 0;
                  g_bmi_gyro_new_batch_ready = 1;
              }
          }
      }
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
