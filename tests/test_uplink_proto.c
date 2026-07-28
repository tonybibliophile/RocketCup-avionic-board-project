/*
 * test_uplink_proto.c — 上行命令封包協定單元測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 驗證 uplink_proto.h（地面站 TX 端 / 火箭 RX 端共用同一份）：
 *   [1] Build → Feed round-trip（cmd/arg/seq 正確還原）
 *   [2] CRC 拒絕（任一位元組翻轉）
 *   [3] 自動重新同步（前面塞垃圾仍能解到後續有效框）
 *   [4] 分段餵入（位元組拆開仍正確）
 *   [5] is_deploy 分類
 *   [6] 連續兩筆有效命令
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "uplink_proto.h"
#include "uplink_text_proto.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static void test_roundtrip(void)
{
    printf("[1] Build -> Feed round-trip\n");
    uint8_t f[UPLINK_FRAME_SIZE];
    uint8_t n = UplinkProto_Build(f, UPLINK_CMD_DEPLOY_DROGUE, 0x5A, 42);
    check("Build 回傳 7", n == UPLINK_FRAME_SIZE);
    check("sync0/1 正確", f[0] == 0x55 && f[1] == 0xAA);

    UplinkRx_t rx; UplinkRx_Init(&rx);
    uint8_t cmd = 0, arg = 0, seq = 0, got = 0;
    for (uint8_t i = 0; i < n; i++) got |= UplinkRx_Feed(&rx, f[i], &cmd, &arg, &seq);
    check("解出一筆", got == 1);
    check("cmd=DEPLOY_DROGUE", cmd == UPLINK_CMD_DEPLOY_DROGUE);
    check("arg=0x5A", arg == 0x5A);
    check("seq=42", seq == 42);
    check("ok 計數=1", rx.ok == 1);
}

static void test_crc_reject(void)
{
    printf("[2] CRC 拒絕\n");
    uint8_t f[UPLINK_FRAME_SIZE];
    UplinkProto_Build(f, UPLINK_CMD_ARM, 0, 1);
    f[3] ^= 0x01;   /* 翻轉 arg → CRC 不符 */

    UplinkRx_t rx; UplinkRx_Init(&rx);
    uint8_t cmd, arg, seq, got = 0;
    for (uint8_t i = 0; i < UPLINK_FRAME_SIZE; i++) got |= UplinkRx_Feed(&rx, f[i], &cmd, &arg, &seq);
    check("毀損框不交出", got == 0);
    check("crc_err 計數=1", rx.crc_err == 1);
}

static void test_resync(void)
{
    printf("[3] 自動重新同步\n");
    uint8_t f[UPLINK_FRAME_SIZE];
    UplinkProto_Build(f, UPLINK_CMD_DEPLOY_BOTH, 0, 7);

    UplinkRx_t rx; UplinkRx_Init(&rx);
    uint8_t cmd = 0, arg = 0, seq = 0, got = 0;
    uint8_t garbage[] = { 0x00, 0x55, 0x12, 0xFF, 0x55, 0x55 };  /* 含假 sync */
    for (size_t i = 0; i < sizeof(garbage); i++) (void)UplinkRx_Feed(&rx, garbage[i], &cmd, &arg, &seq);
    for (uint8_t i = 0; i < UPLINK_FRAME_SIZE; i++) got |= UplinkRx_Feed(&rx, f[i], &cmd, &arg, &seq);
    check("垃圾後仍解出有效框", got == 1 && cmd == UPLINK_CMD_DEPLOY_BOTH && seq == 7);
}

static void test_split_feed(void)
{
    printf("[4] 分段餵入\n");
    uint8_t f[UPLINK_FRAME_SIZE];
    UplinkProto_Build(f, UPLINK_CMD_DEPLOY_MAIN, 0, 99);

    UplinkRx_t rx; UplinkRx_Init(&rx);
    uint8_t cmd = 0, arg = 0, seq = 0, got = 0;
    /* 一次一個 byte（本來就是逐 byte），確認狀態跨呼叫保持 */
    for (uint8_t i = 0; i < UPLINK_FRAME_SIZE; i++) {
        uint8_t r = UplinkRx_Feed(&rx, f[i], &cmd, &arg, &seq);
        if (i < UPLINK_FRAME_SIZE - 1) check("未滿不交出", r == 0);
        got |= r;
    }
    check("最後一 byte 交出", got == 1 && cmd == UPLINK_CMD_DEPLOY_MAIN && seq == 99);
}

static void test_is_deploy(void)
{
    printf("[5] is_deploy 分類\n");
    check("DROGUE 屬 deploy", uplink_cmd_is_deploy(UPLINK_CMD_DEPLOY_DROGUE));
    check("MAIN 屬 deploy",   uplink_cmd_is_deploy(UPLINK_CMD_DEPLOY_MAIN));
    check("BOTH 屬 deploy",   uplink_cmd_is_deploy(UPLINK_CMD_DEPLOY_BOTH));
    check("ARM 非 deploy",   !uplink_cmd_is_deploy(UPLINK_CMD_ARM));
    check("PING 非 deploy",  !uplink_cmd_is_deploy(UPLINK_CMD_PING));
    check("RECALIB 非 deploy", !uplink_cmd_is_deploy(UPLINK_CMD_RECALIBRATE));
    check("RECOVERY 非 deploy", !uplink_cmd_is_deploy(UPLINK_CMD_RECOVERY));
}

static void test_two_frames(void)
{
    printf("[6] 連續兩筆\n");
    uint8_t a[UPLINK_FRAME_SIZE], b[UPLINK_FRAME_SIZE];
    UplinkProto_Build(a, UPLINK_CMD_ARM, 0, 1);
    UplinkProto_Build(b, UPLINK_CMD_DEPLOY_DROGUE, 0, 2);

    UplinkRx_t rx; UplinkRx_Init(&rx);
    uint8_t cmd = 0, arg = 0, seq = 0;
    int n_ok = 0;
    for (uint8_t i = 0; i < UPLINK_FRAME_SIZE; i++)
        if (UplinkRx_Feed(&rx, a[i], &cmd, &arg, &seq)) n_ok++;
    check("第一筆 ARM", n_ok == 1 && cmd == UPLINK_CMD_ARM && seq == 1);
    for (uint8_t i = 0; i < UPLINK_FRAME_SIZE; i++)
        if (UplinkRx_Feed(&rx, b[i], &cmd, &arg, &seq)) n_ok++;
    check("第二筆 DEPLOY", n_ok == 2 && cmd == UPLINK_CMD_DEPLOY_DROGUE && seq == 2);
    check("ok 計數=2", rx.ok == 2);
}

/* ---- 文字上行幀（uplink_text_proto.h）---- */
static int feed_text(const uint8_t *buf, int n, char *out, uint8_t *len, uint8_t *seq)
{
    UplinkTextRx_t rx; UplinkTextRx_Init(&rx);
    int got = 0;
    for (int i = 0; i < n; i++) if (UplinkTextRx_Feed(&rx, buf[i], out, len, seq)) got++;
    return got;
}

static void test_text_roundtrip(void)
{
    printf("[7] 文字幀 round-trip（帶參數指令）\n");
    const char *cmd = "cmd_mag_cal:131072,131072,131072";
    uint8_t f[UPLINK_TEXT_FRAME_MAX];
    uint8_t n = UplinkTextProto_Build(f, cmd, (uint8_t)strlen(cmd), 42);
    check("Build 長度 = 4+len+2", n == (uint8_t)(4 + strlen(cmd) + 2));
    check("sync0/1 = 0x55/0xBB", f[0] == 0x55 && f[1] == 0xBB);
    char out[UPLINK_TEXT_MAX + 1]; uint8_t len = 0, seq = 0;
    check("解出一筆", feed_text(f, n, out, &len, &seq) == 1);
    check("文字還原", strcmp(out, cmd) == 0);
    check("len 正確", len == (uint8_t)strlen(cmd));
    check("seq=42", seq == 42);
}

static void test_text_crc_and_resync(void)
{
    printf("[8] 文字幀 CRC 拒絕 + 前置垃圾 resync\n");
    uint8_t f[UPLINK_TEXT_FRAME_MAX];
    uint8_t n = UplinkTextProto_Build(f, "e80 sf 9", 8, 3);
    char out[UPLINK_TEXT_MAX + 1]; uint8_t len = 0, seq = 0;

    uint8_t bad[UPLINK_TEXT_FRAME_MAX];
    memcpy(bad, f, n); bad[n - 1] ^= 0xFF;   /* 翻 crc 高位 */
    check("毀損框不交出", feed_text(bad, n, out, &len, &seq) == 0);

    uint8_t g[80]; int gi = 0;
    g[gi++] = 0x00; g[gi++] = 0x55; g[gi++] = 0x12; g[gi++] = 0x55;  /* 假 sync0 混雜 */
    memcpy(&g[gi], f, n); gi += n;
    check("垃圾後仍解出", feed_text(g, gi, out, &len, &seq) == 1 && strcmp(out, "e80 sf 9") == 0);
}

static void test_text_maxlen_and_illegal_len(void)
{
    printf("[9] 文字幀 最大長度 + 非法 len 拒絕\n");
    char big[UPLINK_TEXT_MAX + 8];
    memset(big, 'A', sizeof(big)); big[UPLINK_TEXT_MAX + 4] = '\0';   /* 超過上限 */
    uint8_t f[UPLINK_TEXT_FRAME_MAX];
    uint8_t n = UplinkTextProto_Build(f, big, (uint8_t)(UPLINK_TEXT_MAX + 4), 1);
    check("超長自動夾到 MAX", f[3] == UPLINK_TEXT_MAX);
    char out[UPLINK_TEXT_MAX + 1]; uint8_t len = 0, seq = 0;
    check("夾到 MAX 後仍可解", feed_text(f, n, out, &len, &seq) == 1 && len == UPLINK_TEXT_MAX);

    /* 手工造一個 len=0 的非法幀，解析器應拒絕（不掛） */
    uint8_t z[7] = { 0x55, 0xBB, 5, 0, 0, 0, 0 };
    check("len=0 非法幀不交出", feed_text(z, 7, out, &len, &seq) == 0);
}

static void test_text_binary_coexist(void)
{
    printf("[10] 文字幀與二進制幀同流並排解析（互不干擾）\n");
    /* 模擬火箭端：同一位元組流餵兩個解析器，二進制 ARM 與文字命令交錯仍各自解出。 */
    uint8_t bf[UPLINK_FRAME_SIZE];
    UplinkProto_Build(bf, UPLINK_CMD_ARM, 0, 5);
    uint8_t tf[UPLINK_TEXT_FRAME_MAX];
    uint8_t tn = UplinkTextProto_Build(tf, "role", 4, 6);

    uint8_t stream[64]; int si = 0;
    memcpy(&stream[si], bf, UPLINK_FRAME_SIZE); si += UPLINK_FRAME_SIZE;
    memcpy(&stream[si], tf, tn); si += tn;

    UplinkRx_t brx; UplinkRx_Init(&brx);
    UplinkTextRx_t trx; UplinkTextRx_Init(&trx);
    uint8_t bcmd = 0, barg = 0, bseq = 0, bgot = 0;
    char tout[UPLINK_TEXT_MAX + 1]; uint8_t tlen = 0, tseq = 0, tgot = 0;
    for (int i = 0; i < si; i++) {
        bgot |= UplinkRx_Feed(&brx, stream[i], &bcmd, &barg, &bseq);
        tgot |= UplinkTextRx_Feed(&trx, stream[i], tout, &tlen, &tseq);
    }
    check("二進制 ARM 解出", bgot == 1 && bcmd == UPLINK_CMD_ARM && bseq == 5);
    check("文字 role 解出",  tgot == 1 && strcmp(tout, "role") == 0 && tseq == 6);
}

int main(void)
{
    printf("==== uplink_proto 測試 ====\n");
    test_roundtrip();
    test_crc_reject();
    test_resync();
    test_split_feed();
    test_is_deploy();
    test_two_frames();
    test_text_roundtrip();
    test_text_crc_and_resync();
    test_text_maxlen_and_illegal_len();
    test_text_binary_coexist();
    printf("---- %d/%d 通過 ----\n", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
