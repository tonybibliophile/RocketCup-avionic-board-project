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

/* 熱重啟池起點（FlashRing_SkipPreErase 的位址數學）：
 * 寫入頭所在 sector 的剩餘空間必為已擦（該 sector 進入寫入前已擦淨、封包由低往高寫），
 * 故 erased_end 收在下一個 sector 邊界 —— 既維持 sector 對齊不變量，也把那段殘量計入池。
 * 舊版直接令 erased_end = write_addr（128B 對齊）：池=0 且不對齊，落地後第一次滾動擦除
 * 會擦掉寫入頭所在 sector（連同重啟前最後 ≤31 筆），本節即該回歸的鎖。 */
static void test_hotstart_pool_start(void)
{
    printf("[5b] 熱重啟池起點（零擦除還原）\n");
    const uint32_t w = BASE + 3 * SECTOR + 7 * PKT;   /* sector 中段的寫入頭 */
    check("erased_end 收在下一個 sector 邊界", ring_sector_end(w) == BASE + 4 * SECTOR);
    check("恆 sector 對齊", (ring_sector_end(w) % SECTOR) == 0);
    check("同 sector 殘量計入池", ring_pool_bytes_calc(w, ring_sector_end(w)) == SECTOR - 7 * PKT);
    check("寫入頭恰在 sector 邊界 → 整個 sector 計入池",
          ring_pool_bytes_calc(BASE + SECTOR, ring_sector_end(BASE + SECTOR)) == SECTOR);
    check("環尾 sector → 迴繞回 BASE", ring_sector_end(END + 1UL - PKT) == BASE);
    check("舊版行為（erased_end = write_addr）在此為非對齊 ⇒ 已不再使用",
          ((BASE + 3 * SECTOR + 7 * PKT) % SECTOR) != 0);
    /* ★2026-08-01 回歸：FlashRing_ProbePoolNoErase() 的探測迴圈（w25qxx.c）。
     * 舊版以「位元組」為界：pool_bytes + SECTOR <= POOL_TARGET_BYTES。寫入頭落在 sector
     * 中間時（只要曾寫過任何一筆封包就會如此），起始半格讓池永遠停在 TARGET-0.x 格，
     * floor 後恆為 TARGET-1 ⇒ 環明明整個是 0xFF 卻回報「需要擦除」，且唯一解法是再全擦。
     * 本測試舊版還把該 off-by-one 寫成期望值（probed == TARGET-1）而長期全綠。
     * 新版以「格數」為界，與 FlashRing_GetPoolSectors() 的 floor 語意一致。 */
    static const uint32_t heads[] = {
        BASE,                              /* 剛全擦完：write 對齊 BASE */
        BASE + PKT,                        /* 寫過一筆：sector 中間 */
        BASE + 3 * SECTOR + 7 * PKT,       /* 中段、sector 中間 */
        BASE + 100 * SECTOR,               /* 中段、sector 對齊 */
        BASE + 4000 * SECTOR + PKT,        /* 近環尾 + sector 中間（探測需迴繞） */
    };
    for (unsigned i = 0; i < sizeof(heads) / sizeof(heads[0]); i++) {
        const uint32_t head = heads[i];
        uint32_t e = ring_sector_end(head);
        uint32_t n = 0;
        while (ring_pool_bytes_calc(head, e) / SECTOR < FLASH_RING_PREERASE_TARGET
               && n <= FLASH_RING_PREERASE_TARGET) {
            e = ring_sector_end(e);
            n++;
        }
        const uint32_t got = ring_pool_bytes_calc(head, e) / SECTOR;
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "write=0x%06lX 探測必達標（%lu >= %u 格）—— 不因半格 floor 卡在 %u",
                 (unsigned long)head, (unsigned long)got,
                 FLASH_RING_PREERASE_TARGET, FLASH_RING_PREERASE_TARGET - 1U);
        check(msg, got >= FLASH_RING_PREERASE_TARGET);
        snprintf(msg, sizeof(msg), "write=0x%06lX 探測迴圈有界終止", (unsigned long)head);
        check(msg, n <= FLASH_RING_PREERASE_TARGET);
        snprintf(msg, sizeof(msg), "write=0x%06lX 探測後 end 仍 sector 對齊", (unsigned long)head);
        check(msg, (e % SECTOR) == 0);
        snprintf(msg, sizeof(msg), "write=0x%06lX 池不超過目標 + 1 sector", (unsigned long)head);
        check(msg, ring_pool_bytes_calc(head, e) <= POOL_TARGET_BYTES + SECTOR);
    }
}

/* ★2026-08-01 回歸：ARM 後 flash 池被消耗，不得讓「有沒有擦過」的判定翻回「未擦除」。
 * `flash erase` 產生的池恰好是 TARGET 格、零餘裕；PAD_ARMED 以 1Hz 寫入（main.c ring_enabled），
 * 第一筆 128B 封包就讓 floor(池/SECTOR) 由 TARGET 掉到 TARGET-1。舊版 main.c 在 PAD_ARMED
 * 仍以 live 池重算 g_flash_need_erase ⇒ 擦完一按 ARM，一秒後又開始洗 [FLASH_NOT_READY]，
 * 且地面不會重新擦、再也回不去。本節鎖住該數值事實，正解見 main.c（PAD_ARMED 不重算）。 */
static void test_pool_consumed_after_arm(void)
{
    printf("[5c] ARM 後池消耗與「未擦除」判定\n");
    const uint32_t erased_end = BASE + (uint32_t)FLASH_RING_PREERASE_TARGET * SECTOR;
    check("flash erase 後池恰為目標（零餘裕）",
          ring_pool_bytes_calc(BASE, erased_end) / SECTOR == FLASH_RING_PREERASE_TARGET);
    /* 武裝後每秒一筆：寫入頭前進，池縮小 */
    uint32_t write = BASE;
    for (int sec = 1; sec <= 3; sec++) {
        write = ring_write_advance(write);
        check("★寫入第一筆後 live 池即低於目標（故 PAD_ARMED 不可用 live 池判定是否擦過）",
              ring_pool_bytes_calc(write, erased_end) / SECTOR
                  == (uint32_t)FLASH_RING_PREERASE_TARGET - 1U);
    }
    check("池確實只少了 3 筆封包（非真的沒擦）",
          ring_pool_bytes_calc(write, erased_end)
              == (uint32_t)FLASH_RING_PREERASE_TARGET * SECTOR - 3U * PKT);
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
    test_hotstart_pool_start();
    test_pool_consumed_after_arm();
    test_packet_addr();
    test_span_in_pool();
    test_full_lap_sim();
    printf("----------------------------------------\n");
    if (g_fail == 0) printf("ALL PASS：%d/%d 通過\n", g_total, g_total);
    else             printf("FAILED：%d/%d 失敗\n", g_fail, g_total);
    return g_fail ? 1 : 0;
}
