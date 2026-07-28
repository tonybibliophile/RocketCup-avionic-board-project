/*
 * test_flash_ring_watchdog.c — Flash 預擦池是否覆蓋飛安看門狗視窗（P1）
 * ===========================================================================
 *   cd tests && make run
 *
 * 機械化 flash_ring_math.h 對 FLASH_RING_PREERASE_TARGET 註解裡的手算警告：
 * 若 fsm.h 的 FSM_MAIN_WATCHDOG_MS / FSM_MAIN_INFLATE_MS 之後調整（換 profile、
 * 重推 OpenRocket），但忘記回頭重算 FLASH_RING_PREERASE_TARGET，飛行中 flash pool
 * 可能撐不到看門狗視窗結束就耗盡（100Hz 寫入卻無法同步擦除）——這裡讓那個「忘記」
 * 直接讓 host test 變紅，而不是留到上板才發現。
 *
 * flash_ring_math.h 刻意不 include board_config.h（維持 header-only、不依賴
 * profile 巨集），故此交叉檢查放在測試檔而非產品標頭裡；以兩種 profile 各編一份
 * （Makefile 的 test_flash_ring_watchdog_flight / _elevator 兩個 target）。
 */
#include <stdio.h>
#include "flash_ring_math.h"
#include "fsm.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

int main(void) {
#if FLIGHT_PROFILE_ELEVATOR
    printf("=== test_flash_ring_watchdog（電梯 profile，FLIGHT_PROFILE_ELEVATOR=1） ===\n");
#else
    printf("=== test_flash_ring_watchdog（飛行 profile，FLIGHT_PROFILE_ELEVATOR=0） ===\n");
#endif

    /* 「同步擦除被禁止」的最長視窗：main.c FlashRing_SetEraseAllowed 的判斷區間
     * 是 STATE_PAD_ARMED..STATE_MAIN_DEPLOY；看門狗保證 MAIN_DEPLOY 最晚於
     * FSM_MAIN_WATCHDOG_MS 觸發，充氣還要再等 FSM_MAIN_INFLATE_MS 才真正進入
     * MAIN_DEPLOY 完成（解除擦除禁令），故硬上限取兩者之和。 */
    const uint32_t window_ms = FSM_MAIN_WATCHDOG_MS + FSM_MAIN_INFLATE_MS;
    const uint32_t flight_hz = 100U;   /* main.c：STATE_BOOST..MAIN_DEPLOY 之 ring_period_ms=10 */
    const uint32_t packets_per_sector = FLASH_RING_SECTOR_SIZE / FLASH_RING_PACKET_SIZE;

    const uint64_t needed_packets = (uint64_t)window_ms * flight_hz / 1000U;
    const uint64_t needed_sectors = (needed_packets + packets_per_sector - 1U) / packets_per_sector; /* 無條件進位 */

    printf("  視窗=%lums(=%lu+%lu) 需求=%llu packets / %llu sectors，池子=%u sectors\n",
           (unsigned long)window_ms, (unsigned long)FSM_MAIN_WATCHDOG_MS, (unsigned long)FSM_MAIN_INFLATE_MS,
           (unsigned long long)needed_packets, (unsigned long long)needed_sectors,
           (unsigned)FLASH_RING_PREERASE_TARGET);

    check("FLASH_RING_PREERASE_TARGET 覆蓋本 profile 的看門狗視窗",
          (uint64_t)FLASH_RING_PREERASE_TARGET >= needed_sectors);

    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
