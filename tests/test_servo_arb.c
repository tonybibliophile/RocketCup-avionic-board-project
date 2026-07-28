/*
 * test_servo_arb.c — 主傘舵機時間錯開互斥握手情境測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 兩板各跑一份 ServoArb（主板優先），逐 cycle 交換 broadcast_arb（模擬 1 週期鏈路
 * 延遲），鎖定 D2 安全不變量：
 *   [1] 互斥：任一 cycle 絕不兩板同時 drive_servo（diode-OR PWM 不打架）
 *   [2] 錯開順序：主板整段驅動窗在副板之前，各驅動 4s、各釋放一次
 *   [3] 主板失效/鏈路斷：副板經 1s guard 自行驅動（主傘冗餘仍開）
 *   [4] 主板在線但未要求：副板照樣接手驅動（感測未達條件時的加法接手）
 *   [5] 台上安全：want=0（arm-interlock 未成立）→ 兩板皆不驅動
 */
#include <stdio.h>
#include <string.h>
#include "servo_arb.h"

static int g_fail = 0, g_total = 0;
static void check(const char *n, int c) {
    g_total++;
    if (c) printf("  [PASS] %s\n", n);
    else { printf("  [FAIL] %s\n", n); g_fail++; }
}

#define GUARD 1000U
#define DRIVE 4000U
#define STEP    10U

typedef struct {
    ServoArb_t arb;
    uint8_t  last_bcast;        /* 上一週期廣播（對端本週期觀察到的值） */
    int      drove;            /* 累計 drive_servo=1 的週期數 */
    int      releases;
    uint32_t first_drive_ms, last_drive_ms;
} Board;

static void board_init(Board *b, uint8_t is_primary) {
    memset(b, 0, sizeof(*b));
    ServoArb_Init(&b->arb, is_primary);
    b->last_bcast = SERVO_ARB_MSG_NONE;
    b->first_drive_ms = 0xFFFFFFFFu;
}

/* 跑兩板互斥模擬。link_up=0 表兩板互相看不到（peer_arb=NONE，模擬對端斷電/鏈路斷）。
 * 回傳「是否曾有任一 cycle 兩板同時 drive_servo」。 */
static int run_two(Board *A, Board *B, uint8_t wa, uint8_t wb, uint8_t link_up, uint32_t t_end) {
    int both_ever = 0;
    for (uint32_t t = 0; t < t_end; t += STEP) {
        uint8_t pa = link_up ? B->last_bcast : SERVO_ARB_MSG_NONE;   /* A 看到 B 上一輪廣播 */
        uint8_t pb = link_up ? A->last_bcast : SERVO_ARB_MSG_NONE;
        ServoArbAction_t oa = ServoArb_Step(&A->arb, wa, pa, t, GUARD, DRIVE);
        ServoArbAction_t ob = ServoArb_Step(&B->arb, wb, pb, t, GUARD, DRIVE);

        if (oa.drive_servo && ob.drive_servo) both_ever = 1;
        if (oa.drive_servo) { A->drove++; if (A->first_drive_ms == 0xFFFFFFFFu) A->first_drive_ms = t; A->last_drive_ms = t; }
        if (ob.drive_servo) { B->drove++; if (B->first_drive_ms == 0xFFFFFFFFu) B->first_drive_ms = t; B->last_drive_ms = t; }
        A->releases += oa.release_servo; B->releases += ob.release_servo;
        A->last_bcast = oa.broadcast_arb; B->last_bcast = ob.broadcast_arb;
    }
    return both_ever;
}

static void test_normal_staggered(void) {
    printf("[1] 正常：主先→副後，全程互斥\n");
    Board A, B; board_init(&A, 1); board_init(&B, 0);   /* A=主(優先), B=副 */
    int both = run_two(&A, &B, 1, 1, 1, 15000);
    check("互斥：任一 cycle 不兩板同時驅動", both == 0);
    check("主板有驅動", A.drove > 0);
    check("副板有驅動", B.drove > 0);
    check("主板驅動窗整段先於副板", A.last_drive_ms < B.first_drive_ms);
    check("主板驅動約 4s（~400 週期）", A.drove >= 399 && A.drove <= 400);
    check("副板驅動約 4s（~400 週期）", B.drove >= 399 && B.drove <= 400);
    check("主板釋放恰一次", A.releases == 1);
    check("副板釋放恰一次", B.releases == 1);
}

static void test_primary_dead_fallback(void) {
    printf("[2] 主板失效/鏈路斷：副板 guard 後自行驅動（冗餘）\n");
    Board A, B; board_init(&A, 1); board_init(&B, 0);
    int both = run_two(&A, &B, 0, 1, 0, 8000);   /* 只有副板 want、鏈路斷 */
    check("副板仍驅動約 4s（主傘仍開）", B.drove >= 399 && B.drove <= 400);
    check("主板未驅動", A.drove == 0);
    check("互斥仍成立", both == 0);
    check("副板首次驅動於 guard 後（~1s）", B.first_drive_ms >= GUARD && B.first_drive_ms <= GUARD + STEP);
}

static void test_primary_idle_backup_takes(void) {
    printf("[3] 主板在線但未要求：副板接手驅動\n");
    Board A, B; board_init(&A, 1); board_init(&B, 0);
    int both = run_two(&A, &B, 0, 1, 1, 8000);   /* 主板 want=0（在線廣播 NONE），副板 want=1 */
    check("副板接手驅動約 4s", B.drove >= 399 && B.drove <= 400);
    check("主板未驅動", A.drove == 0);
    check("互斥成立", both == 0);
}

static void test_priority_simultaneous(void) {
    printf("[4] 兩板同時想開：主板優先、副板讓位\n");
    Board A, B; board_init(&A, 1); board_init(&B, 0);
    run_two(&A, &B, 1, 1, 1, 15000);
    /* 主板先取得（first_drive 較早），副板讓位至主板 DONE 後才驅動 */
    check("主板先驅動（副板讓位）", A.first_drive_ms < B.first_drive_ms);
    check("副板首次驅動在主板完成之後", B.first_drive_ms > A.last_drive_ms);
}

static void test_pad_safety(void) {
    printf("[5] 台上安全：want=0（arm-interlock 未成立）兩板皆不驅動\n");
    Board A, B; board_init(&A, 1); board_init(&B, 0);
    run_two(&A, &B, 0, 0, 1, 8000);
    check("兩板皆不驅動舵機", A.drove == 0 && B.drove == 0);
    check("兩板皆不釋放", A.releases == 0 && B.releases == 0);
}

int main(void) {
    printf("=== test_servo_arb：主傘舵機時間錯開互斥握手 ===\n");
    test_normal_staggered();
    test_primary_dead_fallback();
    test_primary_idle_backup_takes();
    test_priority_simultaneous();
    test_pad_safety();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
