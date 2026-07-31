/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    w25qxx.h
 * @brief   W25Qxx SPI Flash 驅動程式標頭檔
 *          適用晶片: W25Q16 / W25Q32 / W25Q64 / W25Q128
 *          SPI Bus  : SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI)
 *          CS Pin   : PA15 (FLASH_CSB)
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __W25QXX_H
#define __W25QXX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 *  硬體設定 — 若接線改變只需修改這裡
 * ============================================================ */
#define W25QXX_SPI_HANDLE       hspi3
#define W25QXX_CS_GPIO_PORT     GPIOA
#define W25QXX_CS_GPIO_PIN      GPIO_PIN_15
#define W25QXX_SPI_TIMEOUT_MS   100U

/* ============================================================
 *  W25Qxx 指令集 (Instruction Set)
 * ============================================================ */
#define W25QXX_CMD_WRITE_ENABLE         0x06U
#define W25QXX_CMD_WRITE_DISABLE        0x04U
#define W25QXX_CMD_READ_STATUS_REG1     0x05U
#define W25QXX_CMD_READ_STATUS_REG2     0x35U
#define W25QXX_CMD_WRITE_STATUS_REG     0x01U
#define W25QXX_CMD_READ_DATA            0x03U
#define W25QXX_CMD_FAST_READ            0x0BU
#define W25QXX_CMD_PAGE_PROGRAM         0x02U
#define W25QXX_CMD_SECTOR_ERASE_4KB    0x20U
#define W25QXX_CMD_BLOCK_ERASE_32KB    0x52U
#define W25QXX_CMD_BLOCK_ERASE_64KB    0xD8U
#define W25QXX_CMD_CHIP_ERASE           0xC7U
#define W25QXX_CMD_JEDEC_ID             0x9FU
#define W25QXX_CMD_READ_UID             0x4BU
#define W25QXX_CMD_POWER_DOWN           0xB9U
#define W25QXX_CMD_RELEASE_POWER_DOWN   0xABU

/* ============================================================
 *  Status Register 1 位元定義
 * ============================================================ */
#define W25QXX_SR1_BUSY     (1U << 0)   /* 1 = 正在執行寫入/擦除 */
#define W25QXX_SR1_WEL      (1U << 1)   /* 1 = Write Enable Latch 已設定 */

/* ============================================================
 *  記憶體規格常數
 * ============================================================ */
#define W25QXX_PAGE_SIZE        256U    /* 每頁 256 bytes */
#define W25QXX_SECTOR_SIZE      4096U   /* 每個 Sector 4 KB */
#define W25QXX_BLOCK_SIZE_32K   32768U  /* 32 KB Block */
#define W25QXX_BLOCK_SIZE_64K   65536U  /* 64 KB Block */

/* ============================================================
 *  JEDEC Manufacturer ID
 * ============================================================ */
#define W25QXX_MANUFACTURER_ID  0xEFU   /* Winbond */

/* ============================================================
 *  回傳狀態
 * ============================================================ */
typedef enum {
    W25QXX_OK           = 0,
    W25QXX_ERR_SPI      = 1,   /* SPI 傳輸失敗 */
    W25QXX_ERR_TIMEOUT  = 2,   /* 等待 BUSY 逾時 */
    W25QXX_ERR_ID       = 3,   /* JEDEC ID 不符 */
    W25QXX_ERR_PARAM    = 4,   /* 參數錯誤 */
    W25QXX_ERR_NO_POOL  = 5,   /* P0-E：飛行中預擦池耗盡，本筆丟棄（禁同步擦除） */
} W25QXX_StatusTypeDef;

/* ============================================================
 *  Flash 裝置資訊結構
 * ============================================================ */
typedef struct {
    uint8_t  ManufacturerID;    /* 0xEF = Winbond */
    uint16_t DeviceID;          /* e.g. 0x4017 = W25Q64 */
    uint32_t Capacity_KB;       /* 容量 (KB) */
    uint32_t SectorCount;       /* Sector 總數 */
    bool     Initialized;       /* 初始化成功旗標 */
} W25QXX_InfoTypeDef;

/* ============================================================
 *  外部 SPI handle 宣告 (由 CubeMX spi.c 定義)
 * ============================================================ */
extern SPI_HandleTypeDef W25QXX_SPI_HANDLE;

/* ============================================================
 *  函式宣告
 * ============================================================ */

/**
 * @brief  初始化 Flash 並讀取 JEDEC ID
 * @param  info  指向 W25QXX_InfoTypeDef 結構，初始化後填入裝置資訊
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_Init(W25QXX_InfoTypeDef *info);

/**
 * @brief  讀取 JEDEC ID (3 bytes: Manufacturer + Device)
 * @param  manufacturer  輸出 Manufacturer ID
 * @param  deviceID      輸出 Device ID (2 bytes)
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_ReadJEDEC_ID(uint8_t *manufacturer, uint16_t *deviceID);

/**
 * @brief  從指定位址讀取資料
 * @param  addr    起始位址 (24-bit)
 * @param  buf     輸出緩衝區
 * @param  len     要讀取的 byte 數
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_ReadData(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * @brief  寫入一頁資料 (最多 256 bytes，不可跨頁)
 * @param  addr    起始位址 (需對齊或注意跨頁)
 * @param  buf     輸入資料
 * @param  len     長度 (1~256)
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_WritePage(uint32_t addr, const uint8_t *buf, uint16_t len);

/**
 * @brief  寫入任意長度資料 (自動處理跨頁)
 * @note   寫入前請確保目標區域已被擦除 (值為 0xFF)
 * @param  addr    起始位址
 * @param  buf     輸入資料
 * @param  len     長度
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_WriteData(uint32_t addr, const uint8_t *buf, uint32_t len);

/**
 * @brief  擦除一個 Sector (4 KB)
 * @param  sectorAddr  Sector 起始位址 (需為 4096 的倍數)
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_EraseSector(uint32_t sectorAddr);

/**
 * @brief  擦除一個 64 KB Block (單次最壞 ~2s，遠快於 16 次 Sector Erase)
 * @param  blockAddr  Block 起始位址 (需為 65536 的倍數)
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_EraseBlock64K(uint32_t blockAddr);

/**
 * @brief  擦除整顆 Flash (時間較長，約 20~100 秒；單次阻塞不餵狗)
 * @note   會觸發 IWDG 重啟，一般改用 FlashRing_EraseAll() 逐塊擦除 + 餵狗
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_EraseChip(void);

/**
 * @brief  讀取 Status Register 1
 * @param  status  輸出 status byte
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef W25QXX_ReadStatusReg1(uint8_t *status);

/**
 * @brief  等待 Flash 完成操作 (輪詢 BUSY bit)
 * @param  timeout_ms  最長等待時間 (ms)
 * @retval W25QXX_OK 或 W25QXX_ERR_TIMEOUT
 */
W25QXX_StatusTypeDef W25QXX_WaitForReady(uint32_t timeout_ms);

/**
 * @brief  Flash 功能測試 — 在 UART2 輸出測試結果
 *         流程: 讀 JEDEC ID → Erase Sector 0 → 寫入測試資料 → 讀回比對
 */
void Flash_Test(void);

/* ============================================================
 *  Flash 記憶體分區與雙軌備份規劃 (Design Philosophy)
 * ============================================================
 *  本航電系統採用 W25Q128JV (16 MB SPI Flash)，整體空間劃分為三大分區：
 *  
 *  1. 系統旗標區 (System Flags Sector): 0x000000 - 0x000FFF (4KB, Sector 0)
 *     - 用於儲存地面的靜態偏置 (Calibration)、磁力計硬鐵偏置 (mag_offsets) 以及
 *       開傘事件、斷電計數器等。
 *     - 安全策略：為了防範 4KB Sector Erase 造成高頻感測器採樣 (1.6kHz~3.2kHz) 
 *       與 EKF (100Hz) 迴圈中斷阻塞 (~400ms)，飛行過程中絕不擦除寫入 Sector 0。
 *       僅在發射架靜止自檢校準、以及判定著陸成功 (STATE_LANDED) 後，才可進行擦除寫入。
 *  
 *  2. 任務總結區 (Mission Summary Sector): 0x001000 - 0x00FFFF (60KB, Sector 1 - 15)
 *     - 採用結構體 FlashMissionSummary_t 記錄飛行過程的極值 (最高高度、速度、過載等)
 *       與各狀態切換時間戳。
 *     - 為避免飛行擦除，該區亦限制僅在判定著陸成功 (STATE_LANDED) 後進行一次性寫入。
 *  
 *  3. 飛行數據環形緩衝區 (Flight Ring Buffer): 0x010000 - 0xFFFFFF (~15.9 MB)
 *     - 用於高頻記錄飛行中的姿態與感測器原始數據，不進行 Sector Erase，而是以 
 *       Page Write (1.5ms，不阻塞主任務) 寫入。開機時預先擦除前 10 個 Sector，
 *       並在寫入接近邊界時以背景滾動預擦方式確保寫入不被擦除阻塞。
 * ============================================================ */
/* 環形緩衝區幾何常數與位址數學已抽至 flash_ring_math.h（P1，host 可測，
 * tests/test_flash_ring.c）：FLASH_RINGBUF_ADDR/END/SIZE、FLASH_RING_PACKET_SIZE、
 * FLASH_RING_PREERASE_N/TARGET 皆定義於該檔。 */
#include "flash_ring_math.h"
_Static_assert(FLASH_RING_SECTOR_SIZE == W25QXX_SECTOR_SIZE,
               "flash_ring_math.h 擦除粒度必須與 W25QXX_SECTOR_SIZE 一致");

/* 靜態分區位址（P2 自 w25q128.h 併入；SysFlags/Summary 讀寫函式即操作此二區） */
#define FLASH_SYSFLAGS_ADDR      0x000000UL   /* 系統旗標區起始（Sector 0） */
#define FLASH_SYSFLAGS_SIZE      0x001000UL   /* 4 KB */
#define FLASH_SUMMARY_ADDR       0x001000UL   /* 任務總結區起始（Sectors 1-15） */
#define FLASH_SUMMARY_SIZE       0x00F000UL   /* 60 KB */

/* ============================================================
 *  結構體定義
 * ============================================================ */

/* 系統旗標區結構體 (62 Bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  fsm_state;          /* [0] 飛行狀態機當前狀態 */
    uint8_t  drogue_deployed;    /* [1] 副傘是否已部署 (1=是, 0=否) */
    uint8_t  main_deployed;      /* [2] 主傘是否已部署 (1=是, 0=否) */
    uint8_t  reboot_count;       /* [3] 空中斷電重啟次數計數 */
    uint32_t drogue_time_ms;     /* [4..7] 副傘部署開機時間戳 (ms) */
    uint32_t main_time_ms;       /* [8..11] 主傘部署開機時間戳 (ms) */
    uint16_t bat_voltage_mv;     /* [12..13] 開機/發射時電池電壓 (mV) */
    uint16_t self_test_errs;     /* [14..15] 開機感測器自檢錯誤碼 */
    struct __attribute__((packed)) {
        uint32_t magic;          /* magic = 0xC0DEB1A5 */
        float    baro_launchpad; /* 發射台基準氣壓高度 */
        float    accel_bias[3];  /* EKF 加速度計三軸偏置 (X, Y, Z) */
        float    gyro_bias[3];   /* EKF 陀螺儀三軸偏置 (X, Y, Z) */
    } calib;                     /* [16..47] 校準參數，對齊 FLASH_OFF_CALIB_PARAMS */
    float    mag_offsets[3];     /* [48..59] MMC5983MA 地磁計三軸硬鐵偏置 (Gauss) */
    struct __attribute__((packed)) {
        uint32_t magic;          /* magic = 0x50544c4f ('PTLO') */
        uint32_t e22_freq_mhz;
        uint8_t  e22_pwr;
        uint8_t  e22_air;
        uint8_t  reserved1[2];   /* alignment */
        uint32_t e80_freq_hz;
        uint8_t  e80_sf;
        uint8_t  e80_bw;
        uint8_t  e80_cr;
        int8_t   e80_pwr_dbm;
        uint16_t e80_preamble;
        uint8_t  reserved2[2];   /* alignment */
    } lora;
    uint16_t crc16;              /* 整個結構的 CRC-16 校驗碼 */
} FlashSysFlags_t;

/* 任務總結區結構體 (98 Bytes) */
typedef struct __attribute__((packed)) {
    float    max_altitude;       /* [0..3] 飛行過程最高相對高度 (m) */
    float    max_velocity;       /* [4..7] 飛行過程最大垂直速度 (m/s) */
    float    max_accel_bmi;      /* [8..11] 主感測器 BMI088 最大合加速度 (g) */
    float    max_accel_adxl;     /* [12..15] 高 G 感測器 ADXL375 最大合加速度 (g) */
    uint32_t state_timestamps[8];/* [16..47] FSM 各狀態切換時間戳 (ms)，對齊 FLASH_OFF_STATE_TIMESTAMPS */
    uint32_t flight_duration_ms; /* [48..51] 起飛至落地的總飛行時間 (ms)，對齊 FLASH_OFF_FLIGHT_DURATION */
    float    drogue_altitude;    /* [52..55] 副傘實際開傘相對高度 (m) */
    float    main_altitude;      /* [56..59] 主傘實際開傘相對高度 (m) */
    uint8_t  flight_reboot_count;/* [60] 飛行中經歷的總斷電次數 */
    uint8_t  reserved[3];        /* [61..63] 預留對齊 */
    double   land_gps_lat;       /* [64..71] 落地點 GPS 緯度，對齊 FLASH_OFF_LAND_GPS_LAT */
    double   land_gps_lon;       /* [72..79] 落地點 GPS 經度，對齊 FLASH_OFF_LAND_GPS_LON */
    double   launch_gps_lat;     /* [80..87] 發射點 GPS 緯度 */
    double   launch_gps_lon;     /* [88..95] 發射點 GPS 經度 */
    uint16_t crc16;              /* [96..97] 整個結構的 CRC-16 校驗碼 */
} FlashMissionSummary_t;

/* 飛行數據環形緩衝區封包格式 (128 Bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  magic[2];       /* [0..1]   0xAA 0x55 作為環形掃描標記 */
    uint16_t seq;            /* [2..3]   封包序號（滾動） */
    uint32_t flight_id;      /* [4..7]   飛行批次 ID（同一次 ARM/熱重啟沿用同一值） */
    uint32_t tick_ms;        /* [8..11]  HAL_GetTick() ms */
    uint8_t  fsm_state;      /* [12]     飛行狀態機 state */
    uint8_t  flags;          /* [13]     系統狀態旗標 */
    uint16_t bat_voltage_mv; /* [14..15] 電池電壓 (mV) */
    int16_t  bmi_ax;         /* [16..17] BMI088 Accel X raw LSB */
    int16_t  bmi_ay;         /* [18..19] BMI088 Accel Y raw */
    int16_t  bmi_az;         /* [20..21] BMI088 Accel Z raw */
    int16_t  bmi_gx;         /* [22..23] BMI088 Gyro X raw */
    int16_t  bmi_gy;         /* [24..25] BMI088 Gyro Y raw */
    int16_t  bmi_gz;         /* [26..27] BMI088 Gyro Z raw */
    int16_t  adxl_x;         /* [28..29] ADXL375 X raw LSB */
    int16_t  adxl_y;         /* [30..31] ADXL375 Y raw */
    int16_t  adxl_z;         /* [32..33] ADXL375 Z raw */
    int16_t  baro_temp_c_x100;/* [34..35] 氣壓計溫度 * 100 */
    uint32_t baro_press_pa;  /* [36..39] 氣壓計壓力 (Pa) */
    int32_t  baro_alt_cm;    /* [40..43] 氣壓計相對高度 (cm) */
    int32_t  ekf_pos_z_cm;   /* [44..47] EKF 估計相對高度 (cm) */
    int32_t  ekf_vel_z_cms;  /* [48..51] EKF 估計垂直速度 (cm/s) */
    float    ekf_q0;         /* [52..55] EKF 四元數 q0 */
    float    ekf_q1;         /* [56..59] EKF 四元數 q1 */
    float    ekf_q2;         /* [60..63] EKF 四元數 q2 */
    float    ekf_q3;         /* [64..67] EKF 四元數 q3 */
    int32_t  gps_lat;        /* [68..71] GPS 緯度 (lat * 1e6) */
    int32_t  gps_lon;        /* [72..75] GPS 經度 (lon * 1e6) */
    int16_t  gps_alt_m;      /* [76..77] GPS 海拔高度 (m) */
    int16_t  gps_spd_cms;    /* [78..79] GPS 地速 (cm/s) */
    uint8_t  gps_sats;       /* [80]     GPS 衛星數 */
    uint8_t  gps_fix;        /* [81]     GPS 定位品質 */
    uint8_t  reserved0[2];   /* [82..83] 對齊 flight_tick_ms 至 4-byte 邊界 */
    uint32_t flight_tick_ms; /* [84..87] ★起飛後經過時間 (ms)＝HAL_GetTick()−flight_start_tick；
                              *          未起飛(PAD_ARMED/LANDED 前)為 0。熱重啟判斷與
                              *          flight_start_tick 還原只能用這個欄位——tick_ms 是
                              *          「開機以來」的絕對 tick，重啟後歸零、跨重啟無意義。 */

    /* === [88..97] 跨熱重啟保存的飛行摘要（★2026-07-30 自 reserved 切出，封包仍是 128B）===
     * 這些值平時只活在 RAM（main.c g_max_*／g_drogue_alt_m／g_main_alt_m），飛行中一次
     * brownout/IWDG 重置就會全部歸零——而它們正是「火箭無法回收時唯一能知道飛多高／多快／
     * 幾 G／傘在什麼高度開」的資料，歸零等於整場飛行的關鍵數字消失。
     * 每筆 ring 封包都重複寫入（成本 0，本來就是 reserved 空間），熱重啟時由
     * FlashRing_GetLastPacket() 讀最後一筆還原（見 main.c hs.restore 區塊）。
     * ⚠ 語意與下鏈 telemetry.h 的同名欄位完全一致（單位/哨兵值都相同），改動要同步。 */
    uint16_t max_alt_m;      /* [88..89] 起飛後最大相對高度 (m) */
    int16_t  max_vel_ms;     /* [90..91] 起飛後最大垂直速度 (m/s) */
    uint16_t max_acc_cg;     /* [92..93] 起飛後最大合加速度 |a| (cg = 0.01g)，BMI088 */
    int16_t  drogue_alt_m;   /* [94..95] 副傘實際開傘相對高度 (m)；未開傘 = -32768 */
    int16_t  main_alt_m;     /* [96..97] 主傘實際開傘相對高度 (m)；未開傘 = -32768 */

    uint8_t  reserved[28];   /* [98..125] 預留，維持 128B sector 對齊 */
    uint16_t crc16;          /* [126..127] CRC-16/CCITT */
} FlashRingPacket_t;
_Static_assert(sizeof(FlashRingPacket_t) == FLASH_RING_PACKET_SIZE,
               "FlashRingPacket_t 必須等於 FLASH_RING_PACKET_SIZE");

/* ============================================================
 *  環形緩衝區 API
 * ============================================================ */

/**
 * @brief  初始化環形緩衝區：掃描寫入頭、預擦 FLASH_RING_PREERASE_TARGET 個 Sector
 *         輸出 [FLASH_RING] 訊息至 UART
 */
void FlashRing_Init(void);
void FlashRing_InitEx(void (*progress_cb)(uint32_t current, uint32_t total));
uint8_t FlashRing_GetErasePct(void);

/**
 * @brief  ★P0：僅掃描寫入頭，不擦除。開機序列須先呼叫此函式、用 FlashRing_GetLastPacket()
 *         讀出最後一筆封包並完成「是否為空中熱重啟」判斷後，才可呼叫 FlashRing_RunPreErase()
 *         或 FlashRing_SkipPreErase()。絕不可在讀最後一筆封包前跑 bulk 預擦，否則會把
 *         熱重啟判斷要讀的資料連同寫入頭所在 Sector 一起擦掉。
 */
void FlashRing_ScanWriteHeadOnly(void);

/**
 * @brief  ★P0：確定不是空中熱重啟後才呼叫，執行 960-sector bulk 預擦（行為同舊版
 *         FlashRing_InitEx 的預擦段）。
 */
void FlashRing_RunPreErase(void (*progress_cb)(uint32_t current, uint32_t total));

/**
 * @brief  ★2026-07-31：唯讀認領已擦區並更新池，全程零擦除（開機序列冷/熱兩條路徑共用）。
 *         開機不再自動 bulk 預擦，池改由使用者觸發的 `flash erase` / `flash pool` 建立，
 *         本函式負責重開機後把那片已擦區找回來。誤差方向安全：只會低估、不會高估。
 * @return 認領到的池大小（sectors），上限 FLASH_RING_PREERASE_TARGET
 */
uint32_t FlashRing_ProbePoolNoErase(void);

/**
 * @brief  ★2026-07-31：快速填池（`flash pool` 指令）。只補到 FLASH_RING_PREERASE_TARGET
 *         達標，已擦部分不重擦。飛前正規流程仍應走 `flash erase`（整環全擦）。
 * @return 補完後的池大小（sectors）
 */
uint32_t FlashRing_TopUpPool(void (*progress_cb)(uint32_t current, uint32_t total));

/**
 * @brief  ★P0：確定是空中熱重啟時呼叫，全程零擦除；改以唯讀探測把開機序列留在 flash 上的
 *         已擦區找回來當池（上限仍是 FLASH_RING_PREERASE_TARGET），讓重啟後能繼續記錄。
 *         呼叫端應緊接著呼叫 FlashRing_SetEraseAllowed(0) 關閉飛行中同步擦除——池耗盡時
 *         仍走既有的「池滿即丟包」保護，絕不在飛行中擦除。
 */
void FlashRing_SkipPreErase(void);

/**
 * @brief  寫入一筆飛行數據封包至環形緩衝區
 *         自動計算 CRC，當接近預擦邊界時進行滾動擦除
 * @param  pkt  封包指標
 * @retval W25QXX_OK 或錯誤碼
 */
W25QXX_StatusTypeDef FlashRing_WritePacket(FlashRingPacket_t *pkt);
 
/**
 * @brief  邊界安全地提取環形緩衝區中最後一筆寫入的有效數據封包
 * @param  pkt  輸出封包結構體
 * @retval W25QXX_OK 成功提取, W25QXX_ERR_ID 數據無效或未寫入
 */
W25QXX_StatusTypeDef FlashRing_GetLastPacket(FlashRingPacket_t *pkt);

/**
 * @brief  邊界安全地提取環形緩衝區中倒數第二筆寫入的有效數據封包，用於熱啟動差分速度估計
 * @param  pkt  輸出封包結構體
 * @retval W25QXX_OK 成功提取, W25QXX_ERR_ID 數據無效或未寫入
 */
W25QXX_StatusTypeDef FlashRing_GetSecondLastPacket(FlashRingPacket_t *pkt);

/**
 * @brief  只擦除飛行環形緩衝區（0x010000~0xFFFFFF，Block 1..255），保留 Sector 0
 *         系統旗標區（校準/mag/LoRa）與任務總結區。
 * @note   以 64KB Block Erase 逐塊擦除並每塊餵狗；每次 SPI 交易自帶 CS_LOW/CS_HIGH
 *         SPI3 鎖，呼叫端不需另外包外層鎖（包外層鎖只會讓 E80 920MHz LoRa 遙測整段
 *         期間發不出去）。呼叫端應先放寬 IWDG 視窗（單次 block erase 最壞 ~2s）。
 *         擦除後請呼叫 FlashRing_Init() 重新掃描寫入頭與預擦。
 * @retval W25QXX_OK 或錯誤碼
 */
/* ★2026-07-31：加上 progress_cb（可傳 NULL）。舊版整段擦除只每 32 塊印一行、且完全不更新
 * s_ring_erase_pct，板間鏈路/GUI 的進度一路停在 0，看起來像當機。現在每塊更新 pct 並回呼，
 * 文字進度另附「本塊耗時/平均/預估剩餘」供診斷擦除速度。 */
W25QXX_StatusTypeDef FlashRing_EraseAll(void (*progress_cb)(uint32_t current, uint32_t total));

/** @brief 取得目前寫入地址 */
uint32_t FlashRing_GetWriteAddr(void);

/** @brief 取得累計寫入封包數 */
uint32_t FlashRing_GetPacketCount(void);

/* === P0-E：飛行中擦除禁令 + PAD 期背景預擦池 === */

/** @brief 設定是否允許 WritePacket 內同步滾動擦除（飛行態 BOOST..MAIN_DEPLOY 設 0） */
void FlashRing_SetEraseAllowed(uint8_t allowed);

/** @brief 背景預擦一個 Sector（PAD/INIT 期每 1s 呼叫一次；單次最壞 ~400ms）
 *  @retval 1 = 池已達 FLASH_RING_PREERASE_TARGET（無動作），0 = 已擦一個或擦除失敗 */
uint8_t FlashRing_PreEraseOne(void);

/** @brief 取得目前預擦池大小（sectors） */
uint32_t FlashRing_GetPoolSectors(void);

/** @brief 取得因池耗盡而丟棄的封包數（飛行中擦除禁令生效時累計） */
uint32_t FlashRing_GetDropCount(void);

/** @brief 公用 CRC-16/CCITT 計算函數 */
uint16_t ring_crc16(const uint8_t *data, uint16_t len);

/* ============================================================
 *  靜態區 (Sector 0 / Sector 1-15) 讀寫 API
 * ============================================================ */
W25QXX_StatusTypeDef Flash_WriteSysFlags(FlashSysFlags_t *flags);
W25QXX_StatusTypeDef Flash_ReadSysFlags(FlashSysFlags_t *flags);
W25QXX_StatusTypeDef Flash_WriteMissionSummary(FlashMissionSummary_t *summary);

/** @brief 開機完整 Dump 三分區至 USART2（USER_BT1 按住開機觸發，需 FEATURE_USER_BUTTONS=1；
 *         預設關閉，見 board_config.h。P2 自 w25q128.c 併入）。
 *         呼叫端須持 SPI3 鎖（main.c 既有作法），內部每頁讀取自動餵狗。 */
void Flash_DumpAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H */
