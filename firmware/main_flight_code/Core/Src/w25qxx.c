/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    w25qxx.c
 * @brief   W25Qxx SPI Flash 驅動程式
 *          適用晶片: W25Q16 / W25Q32 / W25Q64 / W25Q128
 *
 *  接線:
 *    SPI1_SCK   → PA5
 *    SPI1_MISO  → PA6
 *    SPI1_MOSI  → PA7
 *    FLASH_CSB  → PA15  (軟體控制 CS)
 *
 *  Debug 輸出走 USART2 (PA2=TX, PA3=RX) → ESP32 TTL → Serial
 ******************************************************************************
 */
/* USER CODE END Header */

#include "w25qxx.h"
#include "spi3_bus.h"   /* SPI3 與 E80 920MHz LoRa 共用，CS 期間須持互斥鎖 */
#include "crc16.h"      /* P1：CRC-16/CCITT-FALSE 單一實作 */
#include "cmsis_os2.h"  /* WaitForReady 讓出 CPU 用 osDelay（排程器啟動後） */
#include <stdarg.h>
#include <stddef.h>     /* offsetof：Flash_DumpAll 欄位解析 */
#include <stdio.h>
#include <string.h>

/* ============================================================
 *  私有巨集
 * ============================================================ */
/* CS_LOW 前取得 SPI3 匯流排、CS_HIGH 後釋放：保證 Flash 在 CS 拉低的整段交易期間
 * 獨佔 SPI3，與 E80 920MHz LoRa 互斥（見 spi3_bus.h）。兩 macro 在每個函式內成對使用，
 * 鎖為遞迴+優先級繼承；mutex 建立前 (pre-scheduler) Lock/Unlock 為 no-op。 */
#define CS_LOW()   do { SPI3_Bus_Lock(); \
                        HAL_GPIO_WritePin(W25QXX_CS_GPIO_PORT, W25QXX_CS_GPIO_PIN, GPIO_PIN_RESET); } while (0)
#define CS_HIGH()  do { HAL_GPIO_WritePin(W25QXX_CS_GPIO_PORT, W25QXX_CS_GPIO_PIN, GPIO_PIN_SET); \
                        SPI3_Bus_Unlock(); } while (0)

/* BUSY bit 等待逾時 (一般寫入) */
#define W25QXX_WRITE_TIMEOUT_MS     500U
/* Sector Erase 最長等待 */
#define W25QXX_SECTOR_ERASE_TIMEOUT 400U
/* 64KB Block Erase 最長等待。W25Q128JV datasheet max ~2s；設 2000 等於零餘裕，
 * 晶片老化/高溫 + WaitForReady 1ms poll 粒度會偶發 ERR_TIMEOUT（進而害 FlashRing_Init
 * 被跳過、留下髒寫入頭），故留 2× 餘裕。 */
#define W25QXX_BLOCK64_ERASE_TIMEOUT 4000U
/* Chip Erase 最長等待 (100 秒) */
#define W25QXX_CHIP_ERASE_TIMEOUT   100000U

/* ============================================================
 *  內部 Debug 輸出 (透過 printf retarget 到 USART2)
 * ============================================================ */
static void flash_debug(const char *msg)
{
    printf("%s", msg);
}

static void flash_debugf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

/* ============================================================
 *  私有: 傳送/接收單一 byte (CS 必須在外部控制)
 * ============================================================ */
static HAL_StatusTypeDef spi_transmit(const uint8_t *pTx, uint16_t len)
{
    return HAL_SPI_Transmit(&W25QXX_SPI_HANDLE, (uint8_t *)pTx, len, W25QXX_SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef spi_receive(uint8_t *pRx, uint16_t len)
{
    return HAL_SPI_Receive(&W25QXX_SPI_HANDLE, pRx, len, W25QXX_SPI_TIMEOUT_MS);
}

/* ============================================================
 *  私有: 傳送 Write Enable 指令
 * ============================================================ */
static W25QXX_StatusTypeDef send_write_enable(void)
{
    uint8_t cmd = W25QXX_CMD_WRITE_ENABLE;
    CS_LOW();
    HAL_StatusTypeDef ret = spi_transmit(&cmd, 1);
    CS_HIGH();
    return (ret == HAL_OK) ? W25QXX_OK : W25QXX_ERR_SPI;
}

/* ============================================================
 *  W25QXX_ReadStatusReg1
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_ReadStatusReg1(uint8_t *status)
{
    uint8_t cmd = W25QXX_CMD_READ_STATUS_REG1;
    CS_LOW();
    if (spi_transmit(&cmd, 1) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    if (spi_receive(status, 1) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();
    return W25QXX_OK;
}

/* 緊輪詢次數上限：約 1ms（每次輪詢含一次 SPI 交易，遠快於此）。
 * page program 典型 tPP ~0.4-0.7ms，在此階段內就會結束，不再被 HAL_Delay(1) 的
 * tick 量化成 1~2ms（HAL_Delay 內部多加 1 tick 保證最小等待）。
 * sector/block erase 等真正需要毫秒級的操作，超過本上限後落入 w25qxx_yield_1ms()。 */
#define W25QXX_WAIT_SPIN_POLLS 200U

/* 排程器啟動前（W25QXX_Init 於 main.c 開機序列呼叫，早於 osKernelStart）osDelay 無效，
 * 須 fallback 到 HAL_Delay；判斷方式與 main.c 既有的 printf retarget 同一套慣例
 * （osKernelGetState()==osKernelRunning && __get_IPSR()==0，即排程器已跑且非中斷內）。 */
static inline void w25qxx_yield_1ms(void)
{
    if (osKernelGetState() == osKernelRunning && __get_IPSR() == 0U) {
        osDelay(1);
    } else {
        HAL_Delay(1);
    }
}

/* ============================================================
 *  W25QXX_WaitForReady
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_WaitForReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t spins = 0;
    uint8_t  status;

    while (1)
    {
        if (W25QXX_ReadStatusReg1(&status) != W25QXX_OK)
            return W25QXX_ERR_SPI;

        if (!(status & W25QXX_SR1_BUSY))
            return W25QXX_OK;  /* 不再 BUSY */

        if ((HAL_GetTick() - start) >= timeout_ms)
            return W25QXX_ERR_TIMEOUT;

        /* 前 W25QXX_WAIT_SPIN_POLLS 次緊迴圈直接重讀 SR1（無延遲）；
         * 短操作（page program）在此階段結束，長操作（erase）才讓出 CPU。 */
        if (++spins > W25QXX_WAIT_SPIN_POLLS) {
            w25qxx_yield_1ms();
        }
    }
}

/* ============================================================
 *  W25QXX_ReadJEDEC_ID
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_ReadJEDEC_ID(uint8_t *manufacturer, uint16_t *deviceID)
{
    uint8_t cmd = W25QXX_CMD_JEDEC_ID;
    uint8_t rx[3] = {0};

    CS_LOW();
    if (spi_transmit(&cmd, 1) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    if (spi_receive(rx, 3) != HAL_OK)    { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();

    *manufacturer = rx[0];
    *deviceID     = ((uint16_t)rx[1] << 8) | rx[2];
    return W25QXX_OK;
}

/* ============================================================
 *  W25QXX_Init
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_Init(W25QXX_InfoTypeDef *info)
{
    if (info == NULL) return W25QXX_ERR_PARAM;

    memset(info, 0, sizeof(W25QXX_InfoTypeDef));

    /* 確保 CS 閒置為 HIGH */
    CS_HIGH();
    HAL_Delay(10);

    /* 喚醒 (Release Power Down) */
    uint8_t cmd = W25QXX_CMD_RELEASE_POWER_DOWN;
    CS_LOW();
    spi_transmit(&cmd, 1);
    CS_HIGH();
    HAL_Delay(1);

    /* 讀取 JEDEC ID */
    W25QXX_StatusTypeDef status = W25QXX_ReadJEDEC_ID(&info->ManufacturerID, &info->DeviceID);
    if (status != W25QXX_OK) return status;

    if (info->ManufacturerID != W25QXX_MANUFACTURER_ID)
    {
        flash_debugf("[FLASH] Init FAIL: MfgID=0x%02X (expected 0xEF)\r\n", info->ManufacturerID);
        return W25QXX_ERR_ID;
    }

    /* 根據 Device ID 推算容量 */
    uint8_t density = (uint8_t)(info->DeviceID & 0xFF);  /* 低 byte = density code */
    if (density >= 0x11 && density <= 0x1B)
    {
        /* 2^density KB, density=0x11 → 2MB (W25Q16), 0x17 → 128MB (W25Q128) */
        uint32_t capacity_bytes = (1UL << density) * 1024UL;  /* 實際是 2^density Mbit /8 */
        /* Winbond density code: 0x11=2MB, 0x12=4MB, 0x13=8MB, 0x14=16MB,
                                 0x15=32MB, 0x16=64MB, 0x17=128MB           */
        /* 正確計算: capacity = 2^density bits / 8 */
        info->Capacity_KB  = (1UL << (density - 3));   /* KB */
        info->SectorCount  = info->Capacity_KB / 4;    /* 每 Sector 4 KB */
        (void)capacity_bytes;
    }

    info->Initialized = true;

    flash_debugf("[FLASH] Init OK | MfgID=0x%02X DevID=0x%04X Cap=%lu KB Sectors=%lu\r\n",
                 info->ManufacturerID, info->DeviceID,
                 (unsigned long)info->Capacity_KB,
                 (unsigned long)info->SectorCount);

    return W25QXX_OK;
}

/* ============================================================
 *  W25QXX_ReadData
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return W25QXX_ERR_PARAM;

    uint8_t cmd[4];
    cmd[0] = W25QXX_CMD_READ_DATA;
    cmd[1] = (uint8_t)((addr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((addr >>  8) & 0xFF);
    cmd[3] = (uint8_t)( addr        & 0xFF);

    /* 確保 Flash 不在 BUSY 狀態 */
    if (W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS) != W25QXX_OK)
        return W25QXX_ERR_TIMEOUT;

    CS_LOW();
    if (spi_transmit(cmd, 4) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }

    /* 分段接收 (HAL 單次最多 65535 bytes) */
    uint32_t remaining = len;
    uint8_t *ptr = buf;
    while (remaining > 0)
    {
        uint16_t chunk = (remaining > 65535U) ? 65535U : (uint16_t)remaining;
        if (spi_receive(ptr, chunk) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
        ptr       += chunk;
        remaining -= chunk;
    }
    CS_HIGH();
    return W25QXX_OK;
}

/* ============================================================
 *  W25QXX_WritePage  (最多 256 bytes，不可跨頁邊界)
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_WritePage(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0 || len > W25QXX_PAGE_SIZE) return W25QXX_ERR_PARAM;

    W25QXX_StatusTypeDef st;

    /* 等待 Flash 就緒 */
    st = W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS);
    if (st != W25QXX_OK) return st;

    /* Write Enable */
    st = send_write_enable();
    if (st != W25QXX_OK) return st;

    uint8_t cmd[4];
    cmd[0] = W25QXX_CMD_PAGE_PROGRAM;
    cmd[1] = (uint8_t)((addr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((addr >>  8) & 0xFF);
    cmd[3] = (uint8_t)( addr        & 0xFF);

    CS_LOW();
    if (spi_transmit(cmd, 4) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    if (spi_transmit(buf, len) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();

    /* 等待 Page Program 完成 */
    return W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS);
}

/* ============================================================
 *  W25QXX_WriteData  (任意長度，自動分頁)
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_WriteData(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0) return W25QXX_ERR_PARAM;

    W25QXX_StatusTypeDef st;
    uint32_t remaining = len;
    uint32_t cur_addr  = addr;
    const uint8_t *ptr = buf;

    while (remaining > 0)
    {
        /* 計算到下一個頁面邊界還有多少 byte */
        uint32_t page_offset   = cur_addr % W25QXX_PAGE_SIZE;
        uint32_t space_in_page = W25QXX_PAGE_SIZE - page_offset;
        uint16_t write_len     = (uint16_t)((remaining < space_in_page) ? remaining : space_in_page);

        st = W25QXX_WritePage(cur_addr, ptr, write_len);
        if (st != W25QXX_OK) return st;

        cur_addr  += write_len;
        ptr       += write_len;
        remaining -= write_len;
    }
    return W25QXX_OK;
}

/* ============================================================
 *  W25QXX_EraseSector (4 KB)
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_EraseSector(uint32_t sectorAddr)
{
    W25QXX_StatusTypeDef st;

    st = W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS);
    if (st != W25QXX_OK) return st;

    st = send_write_enable();
    if (st != W25QXX_OK) return st;

    uint8_t cmd[4];
    cmd[0] = W25QXX_CMD_SECTOR_ERASE_4KB;
    cmd[1] = (uint8_t)((sectorAddr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((sectorAddr >>  8) & 0xFF);
    cmd[3] = (uint8_t)( sectorAddr        & 0xFF);

    CS_LOW();
    if (spi_transmit(cmd, 4) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();

    return W25QXX_WaitForReady(W25QXX_SECTOR_ERASE_TIMEOUT);
}

/* ============================================================
 *  W25QXX_EraseBlock64K (64 KB)
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_EraseBlock64K(uint32_t blockAddr)
{
    W25QXX_StatusTypeDef st;

    st = W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS);
    if (st != W25QXX_OK) return st;

    st = send_write_enable();
    if (st != W25QXX_OK) return st;

    uint8_t cmd[4];
    cmd[0] = W25QXX_CMD_BLOCK_ERASE_64KB;
    cmd[1] = (uint8_t)((blockAddr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((blockAddr >>  8) & 0xFF);
    cmd[3] = (uint8_t)( blockAddr        & 0xFF);

    CS_LOW();
    if (spi_transmit(cmd, 4) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();

    return W25QXX_WaitForReady(W25QXX_BLOCK64_ERASE_TIMEOUT);
}

/* ============================================================
 *  W25QXX_EraseChip
 * ============================================================ */
W25QXX_StatusTypeDef W25QXX_EraseChip(void)
{
    W25QXX_StatusTypeDef st;

    st = W25QXX_WaitForReady(W25QXX_WRITE_TIMEOUT_MS);
    if (st != W25QXX_OK) return st;

    st = send_write_enable();
    if (st != W25QXX_OK) return st;

    uint8_t cmd = W25QXX_CMD_CHIP_ERASE;
    CS_LOW();
    if (spi_transmit(&cmd, 1) != HAL_OK) { CS_HIGH(); return W25QXX_ERR_SPI; }
    CS_HIGH();

    flash_debug("[FLASH] Chip Erase started, waiting...\r\n");
    return W25QXX_WaitForReady(W25QXX_CHIP_ERASE_TIMEOUT);
}

/* ============================================================
 *  Flash_Test — 完整讀寫驗證測試，結果輸出到 UART2
 * ============================================================ */
void Flash_Test(void)
{
    flash_debug("\r\n===== W25Qxx Flash Test Start =====\r\n");

    W25QXX_InfoTypeDef info;
    W25QXX_StatusTypeDef st;

    /* --- Step 1: Init --- */
    flash_debug("[TEST] Step 1: Init...\r\n");
    st = W25QXX_Init(&info);
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Init error %d\r\n", st);
        return;
    }

    /* --- Step 2: JEDEC ID --- */
    flash_debug("[TEST] Step 2: Read JEDEC ID...\r\n");
    uint8_t  mfg;
    uint16_t devID;
    W25QXX_ReadJEDEC_ID(&mfg, &devID);
    flash_debugf("[TEST]   Manufacturer = 0x%02X  DeviceID = 0x%04X\r\n", mfg, devID);

    /* 常見 Device ID 對照 */
    const char *model = "Unknown";
    switch (devID)
    {
        case 0x4011: model = "W25Q10";  break;
        case 0x4012: model = "W25Q20";  break;
        case 0x4013: model = "W25Q40";  break;
        case 0x4014: model = "W25Q80";  break;
        case 0x4015: model = "W25Q16";  break;
        case 0x4016: model = "W25Q32";  break;
        case 0x4017: model = "W25Q64";  break;
        case 0x4018: model = "W25Q128"; break;
        case 0x4019: model = "W25Q256"; break;
    }
    flash_debugf("[TEST]   Model = %s  Capacity = %lu KB\r\n", model, (unsigned long)info.Capacity_KB);

    /* --- Step 3: Erase Sector 0 --- */
    flash_debug("[TEST] Step 3: Erase Sector 0 (addr=0x000000)...\r\n");
    st = W25QXX_EraseSector(0x000000);
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Erase error %d\r\n", st);
        return;
    }
    flash_debug("[TEST]   Erase OK\r\n");

    /* --- Step 4: 確認擦除後全為 0xFF --- */
    flash_debug("[TEST] Step 4: Verify erase (first 16 bytes should be 0xFF)...\r\n");
    uint8_t readbuf[16];
    st = W25QXX_ReadData(0x000000, readbuf, sizeof(readbuf));
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Read error %d\r\n", st);
        return;
    }
    bool erase_ok = true;
    for (int i = 0; i < 16; i++)
    {
        if (readbuf[i] != 0xFF) { erase_ok = false; break; }
    }
    flash_debugf("[TEST]   Erase verify: %s\r\n", erase_ok ? "PASS" : "FAIL");

    /* --- Step 5: 寫入測試資料 --- */
    flash_debug("[TEST] Step 5: Write test pattern (16 bytes)...\r\n");
    uint8_t writebuf[16];
    for (int i = 0; i < 16; i++) writebuf[i] = (uint8_t)(i * 0x11);
    /* writebuf = 0x00, 0x11, 0x22, ..., 0xFF */

    st = W25QXX_WriteData(0x000000, writebuf, sizeof(writebuf));
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Write error %d\r\n", st);
        return;
    }
    flash_debug("[TEST]   Write OK\r\n");

    /* --- Step 6: 讀回比對 --- */
    flash_debug("[TEST] Step 6: Read back and verify...\r\n");
    memset(readbuf, 0, sizeof(readbuf));
    st = W25QXX_ReadData(0x000000, readbuf, sizeof(readbuf));
    if (st != W25QXX_OK)
    {
        flash_debugf("[TEST] FAIL: Read error %d\r\n", st);
        return;
    }

    bool verify_ok = (memcmp(writebuf, readbuf, sizeof(writebuf)) == 0);
    flash_debug("[TEST]   Read back hex: ");
    for (int i = 0; i < 16; i++)
    {
        char tmp[6];
        snprintf(tmp, sizeof(tmp), "%02X ", readbuf[i]);
        flash_debug(tmp);
    }
    flash_debug("\r\n");
    flash_debugf("[TEST]   Verify: %s\r\n", verify_ok ? "PASS" : "FAIL");

    /* --- 總結 --- */
    flash_debug("===========================\r\n");
    flash_debugf("[TEST] Overall: %s\r\n", verify_ok ? "ALL PASS" : "FAIL");
    flash_debug("===========================\r\n\r\n");
}

/* ============================================================
 *  環形緩衝區實作
 * ============================================================ */

extern IWDG_HandleTypeDef hiwdg;

static uint32_t s_ring_write_addr   = FLASH_RINGBUF_ADDR;
static uint32_t s_ring_erased_end   = FLASH_RINGBUF_ADDR;  /* 預擦區終止地址（exclusive） */
static uint32_t s_ring_packet_count = 0;
static uint16_t s_ring_seq          = 0;
static volatile uint8_t  s_ring_erase_allowed = 1;  /* P0-E：0 = 飛行中禁同步擦除 */
static volatile uint32_t s_ring_drop_count    = 0;  /* P0-E：池耗盡丟棄計數 */
static volatile uint8_t  s_ring_erase_pct     = 0;  /* 預擦即時進度百分比 0..100 */

/* P0-E：目前預擦池大小（bytes）。位址數學統一在 flash_ring_math.h（host 已測）；
 * erased_end 採正規化語意（恆 ∈ [BASE, END]），修復舊版 END+1 瞬時態在寫入指標
 * 迴繞後產生整環假池量、導致後續寫入全落未擦區的資料損毀 bug。 */
static uint32_t ring_pool_bytes(void)
{
    return ring_pool_bytes_calc(s_ring_write_addr, s_ring_erased_end);
}

uint16_t ring_crc16(const uint8_t *data, uint16_t len)
{
    return crc16_ccitt_false(data, len);   /* P1：統一至 crc16.h 單一實作（符號保留，多處引用） */
}

/* 掃描寫入頭：讀取 FLASH_RINGBUF_ADDR 的第一個 byte，若為 0xFF 則 Ring 為空、從頭開始；
 * 否則二分搜尋最後一個有資料的 Sector，再逐 slot 找首個空槽。純讀取，不擦除任何東西，
 * 讓呼叫端可以在決定「是否要 bulk 預擦」之前，先安全讀到既有的最後一筆封包。 */
static void ring_scan_write_head(void)
{
    uint8_t first_byte;
    W25QXX_ReadData(FLASH_RINGBUF_ADDR, &first_byte, 1);

    if (first_byte == 0xFF) {
        s_ring_write_addr = FLASH_RINGBUF_ADDR;
        printf("[FLASH_RING] Ring buffer empty, start from 0x%06lX\r\n", s_ring_write_addr);
    } else {
        /* 二分搜尋：找第一個內容全 0xFF 的 Sector */
        uint32_t lo_sec = 0;
        uint32_t hi_sec = FLASH_RINGBUF_SIZE / W25QXX_SECTOR_SIZE;  /* 3840 */
        while (lo_sec + 1 < hi_sec) {
            uint32_t mid = (lo_sec + hi_sec) / 2;
            W25QXX_ReadData(FLASH_RINGBUF_ADDR + mid * W25QXX_SECTOR_SIZE, &first_byte, 1);
            if (first_byte == 0xFF) hi_sec = mid;
            else                    lo_sec = mid;
        }
        /* 在 lo_sec 扇區內逐 slot 掃描 */
        uint32_t scan     = FLASH_RINGBUF_ADDR + lo_sec * W25QXX_SECTOR_SIZE;
        uint32_t scan_end = scan + W25QXX_SECTOR_SIZE;
        uint8_t  magic[2];
        s_ring_write_addr = scan;  /* fallback */
        while (scan + FLASH_RING_PACKET_SIZE <= scan_end) {
            W25QXX_ReadData(scan, magic, 2);
            if (magic[0] == 0xFF && magic[1] == 0xFF) {
                s_ring_write_addr = scan;
                break;
            }
            scan += FLASH_RING_PACKET_SIZE;
        }
        if (scan + FLASH_RING_PACKET_SIZE > scan_end) {
            /* 整個 Sector 已滿，從下一個 Sector 開始（滾動覆蓋） */
            uint32_t next = FLASH_RINGBUF_ADDR + (lo_sec + 1) * W25QXX_SECTOR_SIZE;
            s_ring_write_addr = (next <= FLASH_RINGBUF_END) ? next : FLASH_RINGBUF_ADDR;
        }
        printf("[FLASH_RING] Resumed from 0x%06lX\r\n", s_ring_write_addr);
    }
}

/* Bulk 預擦 FLASH_RING_PREERASE_TARGET 個 Sector，從目前寫入頭所在的 Sector 開始。
 * ★會擦掉寫入頭所在 Sector 內、寫入頭之前的既有資料——呼叫前必須確定不需要再讀那筆
 * 資料（即已排除空中熱重啟的可能）。 */
static void ring_bulk_preerase(void (*progress_cb)(uint32_t current, uint32_t total))
{
    uint32_t erase_addr = s_ring_write_addr & ~((uint32_t)(W25QXX_SECTOR_SIZE - 1));
    for (int i = 0; i < FLASH_RING_PREERASE_TARGET; i++) {
        W25QXX_EraseSector(erase_addr);
        HAL_IWDG_Refresh(&hiwdg);
        s_ring_erase_pct = (uint8_t)(((uint32_t)(i + 1) * 100U) / FLASH_RING_PREERASE_TARGET);
        if (progress_cb) {
            progress_cb(i + 1, FLASH_RING_PREERASE_TARGET);
        } else {
            if ((i + 1) % 96 == 0 || (i + 1) == FLASH_RING_PREERASE_TARGET) {
                printf("[FLASH_RING] Pre-erase %4d/%d @ 0x%06lX\r\n", i + 1, FLASH_RING_PREERASE_TARGET, erase_addr);
            }
        }
        erase_addr = ring_erase_advance(erase_addr);
    }
    s_ring_erased_end   = erase_addr;
    s_ring_packet_count = 0;
    s_ring_seq          = 0;
}

void FlashRing_InitEx(void (*progress_cb)(uint32_t current, uint32_t total))
{
    printf("[FLASH_RING] Init start...\r\n");
    ring_scan_write_head();
    ring_bulk_preerase(progress_cb);
    printf("[FLASH_RING] Ready. Write: 0x%06lX, Erased to: 0x%06lX\r\n",
           s_ring_write_addr, s_ring_erased_end);
}

void FlashRing_Init(void)
{
    FlashRing_InitEx(NULL);
}

/* ★P0：僅掃描寫入頭、不擦除。給開機序列在判斷「是否為空中熱重啟」之前呼叫——
 * 若在讀到上一筆封包前就跑 bulk 預擦，會把寫入頭所在 Sector（含熱重啟判斷要讀的
 * 最後一筆資料）擦掉，等於「先擦證據、後驗屍」。呼叫後應以 FlashRing_GetLastPacket()
 * 讀最後一筆封包、跑完熱重啟判斷，才視結果呼叫 FlashRing_RunPreErase()（地面開機）
 * 或 FlashRing_SkipPreErase()（空中熱重啟）。 */
void FlashRing_ScanWriteHeadOnly(void)
{
    printf("[FLASH_RING] Scan write head (no erase)...\r\n");
    ring_scan_write_head();
    printf("[FLASH_RING] Write head: 0x%06lX\r\n", s_ring_write_addr);
}

/* ★P0：確定不是空中熱重啟（地面開機／熱啟動驗證未過回 PAD）後才呼叫，執行原本的
 * 960-sector bulk 預擦。行為與舊版 FlashRing_InitEx() 的預擦段一致。 */
void FlashRing_RunPreErase(void (*progress_cb)(uint32_t current, uint32_t total))
{
    ring_bulk_preerase(progress_cb);
    printf("[FLASH_RING] Ready. Write: 0x%06lX, Erased to: 0x%06lX\r\n",
           s_ring_write_addr, s_ring_erased_end);
}

/* ★2026-07-31：唯讀認領已擦區。開機序列的「唯一」池來源（冷開機與空中熱重啟共用）：
 * 開機序列的第一步：上一次留在 flash 上的已擦區還好端端在那，只是 RAM 裡的 erased_end
 * 隨重啟沒了——本函式零擦除把它找回來，不足的部分再由 FlashRing_TopUpPool() 補（見 main.c）。
 *
 * 作法：先把寫入頭所在 sector 的剩餘空間計入池（必為 0xFF，理由見 ring_sector_end()），
 * 再逐 sector 讀首 2 bytes 確認 0xFF 才往前收（封包必以 0xAA 0x55 起頭、同 sector 內由
 * 低往高寫，故首格為空 ⇒ 整格未使用；與 ring_scan_write_head() 的二分搜尋同一套假設）。
 * 一遇非 0xFF 立即停手，池上限為 FLASH_RING_PREERASE_TARGET。全程零擦除，成本最壞
 * FLASH_RING_PREERASE_TARGET 次 2-byte 讀（毫秒級）。
 * 誤差方向是安全的：只會低估池（把「其實已擦」誤判為未擦），絕不會高估。 */
uint32_t FlashRing_ProbePoolNoErase(void)
{
    uint32_t end = ring_sector_end(s_ring_write_addr);
    uint32_t probed = 0;

    /* ★2026-08-01：迴圈上限改用「格數」而非「位元組數」。舊版以 pool_bytes + 4096 <= 1500*4096
     * 為界，當寫入頭落在 sector 中間時（只要曾寫過任何一筆封包就會如此），起始的半格讓池永遠
     * 停在 1499.x 格，floor 後恆為 1499 < 1500 ⇒ 環明明整個是 0xFF 卻回報「需要擦除」，且
     * 唯一解法是再全擦一次。現在直接以 GetPoolSectors() 的同一套 floor 語意當終止條件，
     * 需要時多探一格，確保「有 1500 個完整可用格」時就回報達標。
     * probed 硬上限保證終止：整環全 0xFF 時不會繞回起點造成 pool 歸零後無限迴圈。 */
    while (ring_pool_bytes_calc(s_ring_write_addr, end) / W25QXX_SECTOR_SIZE
               < (uint32_t)FLASH_RING_PREERASE_TARGET
           && probed <= (uint32_t)FLASH_RING_PREERASE_TARGET) {
        uint8_t head[2];
        if (W25QXX_ReadData(end, head, sizeof(head)) != W25QXX_OK) break;
        if (head[0] != 0xFF || head[1] != 0xFF) break;   /* 遇到既有資料：停手，絕不擦除 */
        end = ring_sector_end(end);
        if ((++probed % 64U) == 0U) HAL_IWDG_Refresh(&hiwdg);
    }
    HAL_IWDG_Refresh(&hiwdg);

    s_ring_erased_end = end;
    return ring_pool_bytes() / W25QXX_SECTOR_SIZE;
}

/* ★2026-07-31：快速填池（`flash pool` 指令 / bench 反覆測試用）。只把池補到達標，
 * 不動整環 —— 已被 ProbePool 認領的部分不重擦，只擦缺的那幾格。飛前正規流程仍應走
 * `flash erase`（整環全擦），本函式是「環已乾淨、只是池不足」時的省時路徑。
 * @return 補完後的池大小（sectors）。 */
uint32_t FlashRing_TopUpPool(void (*progress_cb)(uint32_t current, uint32_t total))
{
    uint32_t have = FlashRing_ProbePoolNoErase();
    const uint32_t want = FLASH_RING_PREERASE_TARGET;
    if (have >= want) {
        printf("[FLASH_RING] Pool already OK: %lu/%u sectors (no erase needed)\r\n",
               (unsigned long)have, (unsigned)want);
        return have;
    }

    const uint32_t need = want - have;
    const uint32_t sectors_per_block = W25QXX_BLOCK_SIZE_64K / W25QXX_SECTOR_SIZE;   /* 16 */
    printf("[FLASH_RING] Top-up pool: %lu/%u sectors, erasing %lu more...\r\n",
           (unsigned long)have, (unsigned)want, (unsigned long)need);

    const uint32_t t0 = HAL_GetTick();
    uint32_t done = 0, blocks_used = 0, sectors_used = 0;
    while (done < need) {
        const uint32_t target    = ring_erase_target(s_ring_erased_end);
        const uint32_t remaining = need - done;
        /* ★2026-07-31 速度：對齊且還缺 ≥16 格時改用 64KB block erase。
         * 同樣 64KB，1 次 block erase（datasheet typ 150ms）遠快於 16 次 sector erase
         * （16 × typ 45ms ≈ 720ms），整池 1500 格差距可達數十秒。尾巴不足一個 block、
         * 或起點未對齊時才回退到 sector erase，故最終邊界仍精準落在 want。 */
        uint32_t step;
        if ((target % W25QXX_BLOCK_SIZE_64K) == 0U && remaining >= sectors_per_block) {
            if (W25QXX_EraseBlock64K(target) != W25QXX_OK) {
                printf("[FLASH_RING] Top-up FAILED (block) @ 0x%06lX\r\n", target);
                break;
            }
            step = sectors_per_block;
            blocks_used++;
        } else {
            if (W25QXX_EraseSector(target) != W25QXX_OK) {
                printf("[FLASH_RING] Top-up FAILED (sector) @ 0x%06lX\r\n", target);
                break;
            }
            step = 1U;
            sectors_used++;
        }
        for (uint32_t k = 0; k < step; k++) {
            s_ring_erased_end = ring_erase_advance(s_ring_erased_end);
        }
        done += step;
        HAL_IWDG_Refresh(&hiwdg);
        s_ring_erase_pct = (uint8_t)((done * 100U) / need);
        if (progress_cb) progress_cb(done, need);
        if ((done % 160U) < step || done >= need) {
            const uint32_t elapsed = HAL_GetTick() - t0;
            printf("[FLASH_ERASE] top-up %lu/%lu sectors %u%% | 已用 %lus 預估剩餘 %lus\r\n",
                   (unsigned long)done, (unsigned long)need, (unsigned)s_ring_erase_pct,
                   (unsigned long)(elapsed / 1000U),
                   (unsigned long)(done ? ((need - done) * elapsed / done) / 1000U : 0U));
        }
    }

    have = ring_pool_bytes() / W25QXX_SECTOR_SIZE;
    printf("[FLASH_RING] Top-up done: pool=%lu/%u sectors，耗時 %lus（%lu 塊 64KB + %lu 格 4KB），"
           "write=0x%06lX erased_end=0x%06lX\r\n",
           (unsigned long)have, (unsigned)want,
           (unsigned long)((HAL_GetTick() - t0) / 1000U),
           (unsigned long)blocks_used, (unsigned long)sectors_used,
           s_ring_write_addr, s_ring_erased_end);
    return have;
}

/* ★P0：確定是空中熱重啟時呼叫，整段跳過 bulk 預擦（避免擦掉剛讀到的最後一筆飛行
 * 資料、也避免卡住主迴圈數十秒到數分鐘）。呼叫端應緊接著呼叫
 * FlashRing_SetEraseAllowed(0) 立即關閉同步擦除，不要等主迴圈第一輪才關。
 *
 * ★2026-07-31：舊版直接令 erased_end = write_addr（池=0），造成兩個問題：
 *   1) 池=0 + erase_allowed=0 ⇒ 熱重啟之後整段飛行 100% 丟包（見 FlashRing_WritePacket
 *      的 W25QXX_ERR_NO_POOL 分支）——但這個 0 是假的：開機序列（整環全擦／960-sector
 *      預擦）留下的已擦區還好端端在 flash 上，只是 RAM 裡的 erased_end 隨重啟沒了。
 *   2) write_addr 是 128B 對齊、不是 sector 對齊，破壞 erased_end 的 sector 對齊
 *      不變量：落地後 erase_allowed 一開，第一次滾動擦除的 target 就是寫入頭本身，
 *      會擦掉寫入頭所在整個 sector（重啟前最後 ≤31 筆，含熱重啟賴以判斷的那一筆），
 *      之後 erased_end 長期不對齊、池區間尾端會落進未擦 sector（NOR 覆寫靜默損毀）。
 * 現版：不擦任何東西，改以「唯讀探測」把已擦區找回來——先把寫入頭所在 sector 的
 * 剩餘空間計入池（必為 0xFF，理由見 ring_sector_end()），再逐 sector 讀首 2 bytes
 * 確認 0xFF 才往前收（封包必以 0xAA 0x55 起頭、同 sector 內由低往高寫，故首格為空
 * ⇒ 整格未使用；與 ring_scan_write_head() 的二分搜尋同一套假設）。一遇非 0xFF 立即
 * 停手，池上限仍是 FLASH_RING_PREERASE_TARGET。成本最壞 960 次 2-byte 讀（毫秒級），
 * 換回「熱重啟後仍能繼續記錄」，且全程零擦除。 */
void FlashRing_SkipPreErase(void)
{
    uint32_t pool = FlashRing_ProbePoolNoErase();
    s_ring_packet_count = 0;
    s_ring_seq          = 0;
    printf("[FLASH_RING] HOT-RESTART: no erase at all; verified pool=%lu/%u sectors, "
           "write=0x%06lX erased_end=0x%06lX\r\n",
           (unsigned long)pool, (unsigned)FLASH_RING_PREERASE_TARGET,
           s_ring_write_addr, s_ring_erased_end);
}

/* 只擦除環形緩衝區（Block 1..255, 0x010000~0xFFFFFF），保留 Sector 0（校準/mag/LoRa）
 * 與任務總結區（皆位於 Block 0）。以 64KB Block Erase 逐塊擦除（255 次，遠快於 4080 次
 * Sector Erase），每塊擦完餵一次狗。每次 SPI 交易經 CS_LOW/CS_HIGH 自行鎖/解鎖 SPI3，
 * 呼叫端不需另包外層鎖（見 main.c flash erase 指令處說明）；呼叫端應放寬 IWDG 視窗。
 * 擦完由呼叫端跑 FlashRing_Init() 重掃寫入頭。 */
W25QXX_StatusTypeDef FlashRing_EraseAll(void (*progress_cb)(uint32_t current, uint32_t total))
{
    const uint32_t total_blocks =
        (FLASH_RINGBUF_END + 1UL - FLASH_RINGBUF_ADDR) / W25QXX_BLOCK_SIZE_64K;   /* 255 */
    uint32_t addr = FLASH_RINGBUF_ADDR;   /* 0x010000，64KB 對齊 */
    uint32_t block_idx = 0;
    const uint32_t t0 = HAL_GetTick();

    s_ring_erase_pct = 0U;
    if (progress_cb) progress_cb(0U, total_blocks);

    while (addr <= FLASH_RINGBUF_END) {
        const uint32_t tb = HAL_GetTick();
        W25QXX_StatusTypeDef st = W25QXX_EraseBlock64K(addr);
        if (st != W25QXX_OK) {
            printf("[FLASH_RING] EraseAll FAILED @ 0x%06lX, err=%d（已完成 %lu/%lu 塊）\r\n",
                   addr, (int)st, (unsigned long)block_idx, (unsigned long)total_blocks);
            return st;
        }
        HAL_IWDG_Refresh(&hiwdg);
        block_idx++;

        /* ★2026-07-31：每一塊都更新進度。舊版整段 3 分鐘只在每 32 塊印一行純文字、
         * 且完全不動 s_ring_erase_pct，導致板間鏈路/GUI 的 erase_pct 一路停在 0，
         * 使用者看到的是「畫面凍住」。現在 pct 每塊更新，progress_cb 讓呼叫端把進度
         * 廣播出去（LoRa/板間鏈路仍在跑，見 main.c 的任務暫停策略）。 */
        s_ring_erase_pct = (uint8_t)((block_idx * 100U) / total_blocks);
        if (progress_cb) progress_cb(block_idx, total_blocks);

        /* 文字進度：每 16 塊一行，附「本塊耗時 / 平均 / 預估剩餘」——這三個數字才能
         * 回答「為什麼慢」：64KB block erase 的實際耗時取決於晶片磨損程度，
         * datasheet typ 150ms / max 2000ms，只有量出來才知道落在哪。 */
        if ((block_idx % 16U) == 0U || block_idx == total_blocks) {
            const uint32_t elapsed = HAL_GetTick() - t0;
            const uint32_t avg_ms  = elapsed / block_idx;
            printf("[FLASH_ERASE] %lu/%lu blocks %u%% | 本塊 %lums 平均 %lums 已用 %lus 預估剩餘 %lus\r\n",
                   (unsigned long)block_idx, (unsigned long)total_blocks,
                   (unsigned)s_ring_erase_pct,
                   (unsigned long)(HAL_GetTick() - tb), (unsigned long)avg_ms,
                   (unsigned long)(elapsed / 1000U),
                   (unsigned long)(((total_blocks - block_idx) * avg_ms) / 1000U));
        }
        addr += W25QXX_BLOCK_SIZE_64K;
    }

    const uint32_t elapsed = HAL_GetTick() - t0;
    s_ring_erase_pct = 100U;
    if (progress_cb) progress_cb(total_blocks, total_blocks);
    printf("[FLASH_ERASE] DONE：%lu 塊 / %lus（平均 %lums/塊，64KB/塊）\r\n",
           (unsigned long)block_idx, (unsigned long)(elapsed / 1000U),
           (unsigned long)(elapsed / (block_idx ? block_idx : 1U)));
    return W25QXX_OK;
}

W25QXX_StatusTypeDef FlashRing_WritePacket(FlashRingPacket_t *pkt)
{
    /* 若即將超出預擦區，滾動擦除下一個 Sector。
     * P0-E：飛行態（erase_allowed=0）禁止同步擦除 —— 最壞 ~400ms 阻塞主迴圈
     * （FSM 停擺、EKF 斷饋、持 SPI3 mutex），可能正落在頂點窗口。
     * 池耗盡時丟棄該筆並計數（PAD 期背景預擦 64 sectors ≈ 65s 飛行量，
     * 正常不應發生；發射檢核表須確認 [FLASH] pool 達標）。 */
    if (ring_pool_bytes() < FLASH_RING_PACKET_SIZE) {
        if (!s_ring_erase_allowed) {
            s_ring_drop_count++;
            if ((s_ring_drop_count % 100U) == 1U) {   /* 限流：首筆與每 100 筆印一次 */
                printf("[FLASH_RING] pool exhausted in flight, dropped=%lu\r\n",
                       (unsigned long)s_ring_drop_count);
            }
            return W25QXX_ERR_NO_POOL;
        }
        uint32_t target = ring_erase_target(s_ring_erased_end);
        W25QXX_StatusTypeDef ret = W25QXX_EraseSector(target);
        HAL_IWDG_Refresh(&hiwdg);
        if (ret != W25QXX_OK) {
            printf("[FLASH_RING] Rolling erase FAILED @ 0x%06lX, err=%d\r\n",
                   target, (int)ret);
            return ret;
        }
        s_ring_erased_end = ring_erase_advance(s_ring_erased_end);
    }

    /* 填入欄位，計算 CRC（覆蓋 bytes [0..77]） */
    pkt->magic[0] = 0xAA;
    pkt->magic[1] = 0x55;
    pkt->seq      = s_ring_seq++;
    pkt->crc16    = ring_crc16((const uint8_t *)pkt, FLASH_RING_PACKET_SIZE - 2);

    /* 寫入 Flash（W25QXX_WriteData 自動處理跨 Page） */
    W25QXX_StatusTypeDef ret = W25QXX_WriteData(s_ring_write_addr,
                                                  (const uint8_t *)pkt,
                                                  FLASH_RING_PACKET_SIZE);
    if (ret != W25QXX_OK) {
        printf("[FLASH_RING] Write FAILED @ 0x%06lX, err=%d\r\n",
               s_ring_write_addr, (int)ret);
        return ret;
    }

    s_ring_write_addr = ring_write_advance(s_ring_write_addr);

    s_ring_packet_count++;

    /* 每 100 筆 log 一次進度 */
    if (s_ring_packet_count % 100 == 0) {
        printf("[FLASH_RING] PKT#%lu @ 0x%06lX\r\n",
               s_ring_packet_count, s_ring_write_addr);
    }

    return W25QXX_OK;
}

uint32_t FlashRing_GetWriteAddr(void)   { return s_ring_write_addr; }
uint32_t FlashRing_GetPacketCount(void) { return s_ring_packet_count; }

/* === P0-E：飛行中擦除禁令 + PAD 期背景預擦池 === */

void FlashRing_SetEraseAllowed(uint8_t allowed)
{
    s_ring_erase_allowed = (allowed != 0U);
}

uint8_t FlashRing_PreEraseOne(void)
{
    if (ring_pool_bytes() >= (uint32_t)FLASH_RING_PREERASE_TARGET * W25QXX_SECTOR_SIZE) {
        return 1U;   /* 池已達標 */
    }
    uint32_t target = ring_erase_target(s_ring_erased_end);
    if (W25QXX_EraseSector(target) != W25QXX_OK) {
        return 0U;   /* 擦除失敗：下次再試 */
    }
    s_ring_erased_end = ring_erase_advance(s_ring_erased_end);
    return 0U;
}

uint32_t FlashRing_GetPoolSectors(void)
{
    return ring_pool_bytes() / W25QXX_SECTOR_SIZE;
}

uint8_t FlashRing_GetErasePct(void)
{
    return s_ring_erase_pct;
}

uint32_t FlashRing_GetDropCount(void)
{
    return s_ring_drop_count;
}

W25QXX_StatusTypeDef FlashRing_GetLastPacket(FlashRingPacket_t *pkt)
{
    if (pkt == NULL) return W25QXX_ERR_PARAM;

    uint32_t last_packet_addr = ring_last_packet_addr(s_ring_write_addr);

    // 從 Flash 中讀取最後一個封包
    W25QXX_StatusTypeDef ret = W25QXX_ReadData(last_packet_addr, (uint8_t*)pkt, FLASH_RING_PACKET_SIZE);
    if (ret != W25QXX_OK) return ret;

    // 校報魔術字節與數據完整性 (CRC-16)
    if (pkt->magic[0] == 0xAA && pkt->magic[1] == 0x55) {
        uint16_t calc = ring_crc16((const uint8_t *)pkt, FLASH_RING_PACKET_SIZE - 2);
        if (calc == pkt->crc16) {
            return W25QXX_OK;
        }
    }

    return W25QXX_ERR_ID; // 若尚未有任何有效寫入，回傳 ID 錯誤
}

W25QXX_StatusTypeDef FlashRing_GetSecondLastPacket(FlashRingPacket_t *pkt)
{
    if (pkt == NULL) return W25QXX_ERR_PARAM;

    uint32_t second_last_packet_addr =
        ring_prev_packet_addr(ring_last_packet_addr(s_ring_write_addr));

    // 從 Flash 中讀取倒數第二個封包
    W25QXX_StatusTypeDef ret = W25QXX_ReadData(second_last_packet_addr, (uint8_t*)pkt, FLASH_RING_PACKET_SIZE);
    if (ret != W25QXX_OK) return ret;

    // 校驗魔術字節與數據完整性 (CRC-16)
    if (pkt->magic[0] == 0xAA && pkt->magic[1] == 0x55) {
        uint16_t calc = ring_crc16((const uint8_t *)pkt, FLASH_RING_PACKET_SIZE - 2);
        if (calc == pkt->crc16) {
            return W25QXX_OK;
        }
    }

    return W25QXX_ERR_ID; // 若尚未有任何有效寫入，回傳 ID 錯誤
}

/* ============================================================
 *  靜態區 (Sector 0 / Sector 1-15) 讀寫實作
 * ============================================================ */

W25QXX_StatusTypeDef Flash_WriteSysFlags(FlashSysFlags_t *flags)
{
    if (flags == NULL) return W25QXX_ERR_PARAM;

    // 1. 計算並填充 CRC16 (覆蓋前 60 bytes)
    flags->crc16 = ring_crc16((const uint8_t *)flags, sizeof(FlashSysFlags_t) - 2);

    // 2. 擦除 Sector 0 (4KB) —— 警告：此操作會阻塞約 300ms
    W25QXX_StatusTypeDef ret = W25QXX_EraseSector(0x000000UL);
    if (ret != W25QXX_OK) return ret;

    // 3. 寫入資料
    return W25QXX_WriteData(0x000000UL, (const uint8_t *)flags, sizeof(FlashSysFlags_t));
}

W25QXX_StatusTypeDef Flash_ReadSysFlags(FlashSysFlags_t *flags)
{
    if (flags == NULL) return W25QXX_ERR_PARAM;

    // 1. 讀取 Sector 0 開頭資料
    W25QXX_StatusTypeDef ret = W25QXX_ReadData(0x000000UL, (uint8_t *)flags, sizeof(FlashSysFlags_t));
    if (ret != W25QXX_OK) return ret;

    // 2. 驗證 CRC16
    uint16_t calc = ring_crc16((const uint8_t *)flags, sizeof(FlashSysFlags_t) - 2);
    if (calc != flags->crc16) {
        return W25QXX_ERR_ID; // CRC 校驗失敗，可能尚未校準或資料損毀
    }

    return W25QXX_OK;
}

W25QXX_StatusTypeDef Flash_WriteMissionSummary(FlashMissionSummary_t *summary)
{
    if (summary == NULL) return W25QXX_ERR_PARAM;

    // 1. 計算並填充 CRC16 (覆蓋前 96 bytes)
    summary->crc16 = ring_crc16((const uint8_t *)summary, sizeof(FlashMissionSummary_t) - 2);

    // 2. 擦除 Sector 1 (4KB)
    W25QXX_StatusTypeDef ret = W25QXX_EraseSector(0x001000UL);
    if (ret != W25QXX_OK) return ret;

    // 3. 寫入資料
    return W25QXX_WriteData(0x001000UL, (const uint8_t *)summary, sizeof(FlashMissionSummary_t));
}

/* ============================================================
 *  Flash Dump（P2 自 w25q128.c 併入，雙驅動合一）
 * ============================================================ */

/* 輸出單行標準 Hex Dump（最多 16 bytes）至 USART2。
 * 格式: XXXXXX: XX XX XX XX XX XX XX XX  XX XX XX XX XX XX XX XX  |................| */
static void flash_print_hex_line(uint32_t addr, const uint8_t *data, uint16_t len)
{
    printf("%06X: ", (unsigned int)addr);
    for (int i = 0; i < 16; i++) {
        if (i < (int)len) printf("%02X ", data[i]);
        else               printf("   ");
        if (i == 7)        printf(" ");   /* 中間空格分隔高低 8 bytes */
    }
    printf(" |");
    for (int i = 0; i < (int)len; i++)
        printf("%c", (data[i] >= 0x20 && data[i] < 0x7F) ? data[i] : '.');
    printf("|\r\n");
}

/* 完整讀取三個記憶體分區並格式化輸出至 USART2。
 *   [1] 系統旗標區  (0x000000~0x000FFF, 4KB)   — 全量 Hex Dump + 欄位解析
 *   [2] 任務總結區  (0x001000~0x00FFFF, 60KB)  — 前 128 bytes Hex Dump + 欄位解析
 *   [3] Ring Buffer (0x010000~0xFFFFFF, ~15.9MB)— 前 3 封包 Hex Dump + 空滿判斷
 * 每讀完一頁 (256 bytes) 自動餵狗，確保不觸發 2s IWDG 超時。
 * 欄位偏移一律 offsetof 結構體推導（取代舊 w25q128.h 手寫偏移，佈局改動自動跟隨）。 */
void Flash_DumpAll(void)
{
    uint8_t buf[256];

    printf("\r\n");
    printf("========================================================\r\n");
    printf("  W25Q128 Flash Memory Dump\r\n");
    printf("  Total 16MB  (0x000000 ~ 0xFFFFFF)\r\n");
    printf("========================================================\r\n");

    /* --- JEDEC ID 驗證，確認晶片正常就緒 --- */
    uint8_t mfr = 0; uint16_t dev = 0;
    W25QXX_ReadJEDEC_ID(&mfr, &dev);
    uint32_t jedec = ((uint32_t)mfr << 16) | dev;
    printf("[JEDEC] 0x%06X  %s\r\n", (unsigned int)jedec,
           (jedec == 0xEF4018UL) ? "W25Q128JV OK" : "ID MISMATCH!");
    if (jedec != 0xEF4018UL) {
        printf("[FLASH] Flash 晶片未就緒，中止讀取。\r\n\r\n");
        return;
    }

    /* === [1] 系統旗標區：全量 Hex Dump，每頁餵狗 === */
    printf("\r\n[1] 系統旗標區  0x%06X ~ 0x%06X  (%u bytes)\r\n",
           (unsigned int)FLASH_SYSFLAGS_ADDR,
           (unsigned int)(FLASH_SYSFLAGS_ADDR + FLASH_SYSFLAGS_SIZE - 1),
           (unsigned int)FLASH_SYSFLAGS_SIZE);
    printf("--------------------------------------------------------\r\n");
    for (uint32_t off = 0; off < FLASH_SYSFLAGS_SIZE; off += 256) {
        W25QXX_ReadData(FLASH_SYSFLAGS_ADDR + off, buf, 256);
        HAL_IWDG_Refresh(&hiwdg);
        for (uint16_t ln = 0; ln < 256; ln += 16)
            flash_print_hex_line(FLASH_SYSFLAGS_ADDR + off + ln, buf + ln, 16);
    }

    /* 欄位解析（offsetof 推導，與 FlashSysFlags_t 佈局同步） */
    W25QXX_ReadData(FLASH_SYSFLAGS_ADDR, buf, sizeof(FlashSysFlags_t));
    HAL_IWDG_Refresh(&hiwdg);
    printf("  [欄位解析]\r\n");
    printf("  FSM State     @ +0x%04X : 0x%02X\r\n",
           (unsigned int)offsetof(FlashSysFlags_t, fsm_state),
           buf[offsetof(FlashSysFlags_t, fsm_state)]);
    printf("  Drogue/Main   @ +0x%04X : %u / %u  (reboot=%u)\r\n",
           (unsigned int)offsetof(FlashSysFlags_t, drogue_deployed),
           buf[offsetof(FlashSysFlags_t, drogue_deployed)],
           buf[offsetof(FlashSysFlags_t, main_deployed)],
           buf[offsetof(FlashSysFlags_t, reboot_count)]);
    printf("  Calib magic   @ +0x%04X : %02X %02X %02X %02X  (期望 A5 B1 DE C0)\r\n",
           (unsigned int)offsetof(FlashSysFlags_t, calib),
           buf[offsetof(FlashSysFlags_t, calib)+0], buf[offsetof(FlashSysFlags_t, calib)+1],
           buf[offsetof(FlashSysFlags_t, calib)+2], buf[offsetof(FlashSysFlags_t, calib)+3]);

    /* === [2] 任務總結區：前 128 bytes（涵蓋所有已定義欄位） === */
    printf("\r\n[2] 任務總結區  0x%06X ~ 0x%06X  (%u bytes，顯示前 128 bytes)\r\n",
           (unsigned int)FLASH_SUMMARY_ADDR,
           (unsigned int)(FLASH_SUMMARY_ADDR + FLASH_SUMMARY_SIZE - 1),
           (unsigned int)FLASH_SUMMARY_SIZE);
    printf("--------------------------------------------------------\r\n");
    W25QXX_ReadData(FLASH_SUMMARY_ADDR, buf, 128);
    HAL_IWDG_Refresh(&hiwdg);
    for (uint16_t ln = 0; ln < 128; ln += 16)
        flash_print_hex_line(FLASH_SUMMARY_ADDR + ln, buf + ln, 16);

    printf("  [欄位解析]\r\n");
    printf("  Max Altitude  @ +0x%04X : %02X %02X %02X %02X  (float, LE)\r\n",
           (unsigned int)offsetof(FlashMissionSummary_t, max_altitude),
           buf[offsetof(FlashMissionSummary_t, max_altitude)+0],
           buf[offsetof(FlashMissionSummary_t, max_altitude)+1],
           buf[offsetof(FlashMissionSummary_t, max_altitude)+2],
           buf[offsetof(FlashMissionSummary_t, max_altitude)+3]);
    printf("  Max Velocity  @ +0x%04X : %02X %02X %02X %02X  (float, LE)\r\n",
           (unsigned int)offsetof(FlashMissionSummary_t, max_velocity),
           buf[offsetof(FlashMissionSummary_t, max_velocity)+0],
           buf[offsetof(FlashMissionSummary_t, max_velocity)+1],
           buf[offsetof(FlashMissionSummary_t, max_velocity)+2],
           buf[offsetof(FlashMissionSummary_t, max_velocity)+3]);
    printf("  State Times   @ +0x%04X :\r\n",
           (unsigned int)offsetof(FlashMissionSummary_t, state_timestamps));
    for (int i = 0; i < 8; i++) {
        uint32_t base = offsetof(FlashMissionSummary_t, state_timestamps) + (uint32_t)i * 4U;
        uint32_t ts = (uint32_t)buf[base+0]
                    | (uint32_t)buf[base+1] << 8
                    | (uint32_t)buf[base+2] << 16
                    | (uint32_t)buf[base+3] << 24;
        printf("    [%d] %u ms\r\n", i, (unsigned int)ts);
    }

    /* === [3] Ring Buffer：前 3 封包 + 空滿判斷 === */
    const uint16_t ring_preview = (uint16_t)(FLASH_RING_PACKET_SIZE * 3);  /* 384 bytes */
    printf("\r\n[3] Ring Buffer  0x%06X ~ 0x%06X  (%u MB)\r\n",
           (unsigned int)FLASH_RINGBUF_ADDR,
           (unsigned int)FLASH_RINGBUF_END,
           (unsigned int)(FLASH_RINGBUF_SIZE >> 20));
    printf("    容量: %u 封包（飛行 50Hz ≈ %u 分鐘）\r\n",
           (unsigned int)(FLASH_RINGBUF_SIZE / FLASH_RING_PACKET_SIZE),
           (unsigned int)(FLASH_RINGBUF_SIZE / FLASH_RING_PACKET_SIZE / 50UL / 60UL));
    printf("    顯示前 %u bytes (前 3 封包)\r\n", ring_preview);
    printf("--------------------------------------------------------\r\n");
    W25QXX_ReadData(FLASH_RINGBUF_ADDR, buf, ring_preview);
    HAL_IWDG_Refresh(&hiwdg);

    uint8_t ring_has_data = 0;
    for (uint16_t i = 0; i < ring_preview; i++) {
        if (buf[i] != 0xFF) { ring_has_data = 1; break; }
    }
    printf("  狀態: %s\r\n", ring_has_data ? "有數據 (非全空白)" : "全空白 (0xFF，尚未寫入)");

    for (uint16_t ln = 0; ln < ring_preview; ln += 16) {
        uint16_t rem = ring_preview - ln;
        flash_print_hex_line(FLASH_RINGBUF_ADDR + ln, buf + ln, rem >= 16 ? 16 : rem);
    }

    printf("\r\n[FLASH] Dump 完畢。\r\n");
    printf("========================================================\r\n\r\n");
    HAL_IWDG_Refresh(&hiwdg);
}
