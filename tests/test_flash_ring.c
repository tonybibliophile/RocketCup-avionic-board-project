/*
 * test_flash_ring.c — Flash 環形緩衝區位址數學單元測試（P1，純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 驗證 flash_ring_math.h：
 *   [1] 幾何不變量：封包與 sector 恰好鋪滿整環
 *   [2] 池量計算：前向 / 迴繞 / 空池
 *   [3] 寫入指標推進與環尾迴繞
 *   [4] 擦除指標推進：正規化語意（恆 ∈ [BASE, END]，不再停 END+1）
 *   [5] 熱啟動回讀位址：最後一筆 / 倒數第二筆的迴繞
 *   [6] ring_span_in_pool 安全不變量
 *   [7] 迴繞 bug 回歸：以 byte 粒度假 flash 模擬兩整圈寫入＋預擦池，
 *       逐 byte 斷言「擦過才能寫、寫過不重寫」——
 *       舊版 erased_end 可停 END+1 的語意在寫入指標迴繞後產生整環假池量，
 *       此模擬會在第一圈結束時崩紅；正規化後全綠。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "flash_ring_math.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

#define BASE    FLASH_RINGBUF_ADDR
#define END     FLASH_RINGBUF_END
#define PKT     FLASH_RING_PACKET_SIZE
#define SECTOR  FLASH_RING_SECTOR_SIZE
#define POOL_TARGET_BYTES  ((uint32_t)FLASH_RING_PREERASE_TARGET * SECTOR)

static void test_geometry(void)
{
    printf("[1] 幾何不變量\n");
    check("環容量 = END+1-BASE", FLASH_RINGBUF_SIZE == END + 1UL - BASE);
    check("封包恰好鋪滿整環（128B × 130560）", FLASH_RINGBUF_SIZE % PKT == 0 &&
                                               FLASH_RINGBUF_SIZE / PKT == 130560UL);
    check("sector 恰好鋪滿整環（4KB × 4080）", FLASH_RINGBUF_SIZE % SECTOR == 0 &&
                                               FLASH_RINGBUF_SIZE / SECTOR == 4080UL);
    check("BASE sector 對齊", (BASE % SECTOR) == 0);
}

static void test_pool(void)
{
    printf("[2] 池量計算\n");
    check("空池（兩針相等）= 0", ring_pool_bytes_calc(BASE, BASE) == 0);
    check("前向：erased 領先 2 sector", ring_pool_bytes_calc(BASE, BASE + 2 * SECTOR) == 2 * SECTOR);
    check("迴繞：write 在環尾、erased 已繞回",
          ring_pool_bytes_calc(END + 1UL - PKT, BASE + SECTOR) == PKT + SECTOR);
    check("迴繞：write 中段、erased 繞回 BASE（= 環尾餘量）",
          ring_pool_bytes_calc(0xFFF000UL, BASE) == END + 1UL - 0xFFF000UL);
    /* 舊 bug 場景：write 剛迴繞回 BASE、erased_end 若停 END+1 會算出整環假池。
     * 正規化語意下 erased_end 此時必為 BASE → 池 = 0（誠實）。 */
    check("write 與 erased 同時在 BASE = 空池（舊 bug 場景的正規化結果）",
          ring_pool_bytes_calc(BASE, BASE) == 0);
}

static void test_write_advance(void)
{
    printf("[3] 寫入指標推進\n");
    check("中段 +128", ring_write_advance(BASE) == BASE + PKT);
    check("最後一格寫完 → 迴繞回 BASE", ring_write_advance(END + 1UL - PKT) == BASE);
    check("倒數第二格 → 最後一格", ring_write_advance(END + 1UL - 2 * PKT) == END + 1UL - PKT);
}

static void test_erase_advance(void)
{
    printf("[4] 擦除指標推進（正規化語意）\n");
    check("中段 +4KB", ring_erase_advance(BASE) == BASE + SECTOR);
    check("最後一個 sector 擦完 → 直接回 BASE（不停 END+1）",
          ring_erase_advance(END + 1UL - SECTOR) == BASE);
    check("防衛：傳入未正規化 END+1 → 視同 BASE 推進",
          ring_erase_advance(END + 1UL) == BASE + SECTOR);
    check("target 中段恆等", ring_erase_target(0x800000UL) == 0x800000UL);
    check("target 防衛收斂 END+1 → BASE", ring_erase_target(END + 1UL) == BASE);
}

static void test_packet_addr(void)
{
    printf("[5] 熱啟動回讀位址\n");
    check("中段：last = write - 128", ring_last_packet_addr(BASE + 10 * PKT) == BASE + 9 * PKT);
    check("write 已迴繞回 BASE：last = 環尾最後一格",
          ring_last_packet_addr(BASE) == END + 1UL - PKT);
    check("prev 中段", ring_prev_packet_addr(BASE + PKT) == BASE);
    check("prev 在 BASE → 環尾最後一格", ring_prev_packet_addr(BASE) == END + 1UL - PKT);
    check("倒數第二筆（write=BASE+80 → last=BASE → prev=環尾）",
          ring_prev_packet_addr(ring_last_packet_addr(BASE + PKT)) == END + 1UL - PKT);
}

static void test_span_in_pool(void)
{
    printf("[6] span_in_pool 安全不變量\n");
    uint32_t w = BASE, e = BASE + SECTOR;
    check("池內首筆", ring_span_in_pool(w, e, w, PKT) == 1);
    check("恰好填滿池尾", ring_span_in_pool(w, e, e - PKT, PKT) == 1);
    check("超出池尾 1 byte", ring_span_in_pool(w, e, e - PKT + 1, PKT) == 0);
    check("空池任何寫入都拒絕", ring_span_in_pool(BASE, BASE, BASE, 1) == 0);
    /* 跨環尾的池：write 在尾、erased 繞回 */
    w = END + 1UL - SECTOR; e = BASE + SECTOR;
    check("迴繞池：環尾段在池內", ring_span_in_pool(w, e, w, SECTOR) == 1);
    check("迴繞池：BASE 段在池內", ring_span_in_pool(w, e, BASE, PKT) == 1);
    check("迴繞池：跨縫 span 在池內", ring_span_in_pool(w, e, END + 1UL - PKT, 2 * PKT) == 1);
    check("迴繞池：超出繞回端", ring_span_in_pool(w, e, e - PKT + 1, PKT) == 0);
}

/* ------------------------------------------------------------------ */
/* [7] 假 flash 模擬：兩整圈寫入 + PAD 預擦池維持                       */
/* ------------------------------------------------------------------ */
#define ST_DIRTY   0   /* 未擦（出廠視為髒，模擬最壞情況） */
#define ST_ERASED  1   /* 已擦未寫 */
#define ST_WRITTEN 2   /* 已寫 */

static uint8_t *g_map;            /* FLASH_RINGBUF_SIZE bytes 狀態圖 */
static uint32_t g_violations;     /* 寫入未擦/重寫 byte 的次數 */

static void sim_erase(uint32_t addr)
{
    if (addr < BASE || addr + SECTOR > END + 1UL || (addr % SECTOR) != 0) {
        g_violations += 1000000;  /* 越界/未對齊擦除：重罰 */
        return;
    }
    memset(g_map + (addr - BASE), ST_ERASED, SECTOR);
}

static void sim_write(uint32_t addr, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint32_t off = addr - BASE + i;
        if (off >= FLASH_RINGBUF_SIZE || g_map[off] != ST_ERASED) {
            g_violations++;
            return;
        }
        g_map[off] = ST_WRITTEN;
    }
}

static void test_full_lap_sim(void)
{
    printf("[7] 假 flash 模擬：兩整圈（261,120 筆）寫入\n");
    g_map = (uint8_t *)calloc(FLASH_RINGBUF_SIZE, 1);   /* 全 ST_DIRTY */
    if (!g_map) { check("calloc 假 flash", 0); return; }
    g_violations = 0;

    /* 開機：空環，write 自 BASE 起，預擦 10 sectors（比照 FlashRing_Init） */
    uint32_t write = BASE;
    uint32_t erased_end = write & ~(SECTOR - 1UL);
    for (int i = 0; i < FLASH_RING_PREERASE_N; i++) {
        sim_erase(ring_erase_target(erased_end));
        erased_end = ring_erase_advance(erased_end);
    }

    const uint32_t total_pkts = 2UL * (FLASH_RINGBUF_SIZE / PKT);
    uint32_t span_fail = 0, ptr_oob = 0;

    for (uint32_t n = 0; n < total_pkts; n++) {
        /* PAD 背景預擦（比照 FlashRing_PreEraseOne）：每 50 筆擦 1 個直到池達標。
         * 與滾動擦除交錯，覆蓋兩種擦除路徑的迴繞。 */
        if ((n % 50U) == 0U &&
            ring_pool_bytes_calc(write, erased_end) < POOL_TARGET_BYTES) {
            sim_erase(ring_erase_target(erased_end));
            erased_end = ring_erase_advance(erased_end);
        }
        /* 池耗盡 → 滾動擦除（比照 FlashRing_WritePacket 的 PAD 路徑） */
        if (ring_pool_bytes_calc(write, erased_end) < PKT) {
            sim_erase(ring_erase_target(erased_end));
            erased_end = ring_erase_advance(erased_end);
        }
        /* 安全不變量：本筆寫入必須完整落在已擦池內 */
        if (!ring_span_in_pool(write, erased_end, write, PKT)) span_fail++;
        sim_write(write, PKT);
        write = ring_write_advance(write);

        if (write < BASE || write > END || erased_end < BASE || erased_end > END) ptr_oob++;
    }

    check("兩整圈零未擦寫入 / 零重寫（byte 粒度）", g_violations == 0);
    check("兩整圈 span_in_pool 恆成立", span_fail == 0);
    check("write / erased_end 恆在 [BASE, END] 正規化域", ptr_oob == 0);
    check("池量不超過目標上限 + 1 sector",
          ring_pool_bytes_calc(write, erased_end) <= POOL_TARGET_BYTES + SECTOR);

    free(g_map);
}

int main(void)
{
    printf("=== test_flash_ring：環形緩衝區位址數學（P1） ===\n");
    test_geometry();
    test_pool();
    test_write_advance();
    test_erase_advance();
    test_packet_addr();
    test_span_in_pool();
    test_full_lap_sim();
    printf("----------------------------------------\n");
    if (g_fail == 0) printf("ALL PASS：%d/%d 通過\n", g_total, g_total);
    else             printf("FAILED：%d/%d 失敗\n", g_fail, g_total);
    return g_fail ? 1 : 0;
}
