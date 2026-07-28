/*
 * test_ack_proto.c — 命令 ACK 幀協定單元測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 驗證 ack_proto.h（火箭 TX 端 / 地面站 RX 端共用同一份）：
 *   [1] Build → Feed round-trip（seq/status/echo 文字還原）
 *   [2] 空文字 ACK（len==0）邊界
 *   [3] CRC 拒絕
 *   [4] 前置垃圾自動 resync
 *   [5] 與遙測位元組流並排時（模擬）ACK 仍解得出（sync 0xAC/0xCA 不誤觸）
 *   [6] status 字串
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ack_proto.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static int feed(const uint8_t *buf, int n, uint8_t *seq, uint8_t *st, char *out, uint8_t *len)
{
    AckRx_t rx; AckRx_Init(&rx);
    int got = 0;
    for (int i = 0; i < n; i++) if (AckRx_Feed(&rx, buf[i], seq, st, out, len)) got++;
    return got;
}

static void test_roundtrip(void)
{
    printf("[1] Build -> Feed round-trip\n");
    uint8_t f[ACK_FRAME_MAX];
    uint8_t n = AckProto_Build(f, 42, ACK_OK, "e80 sf 9", 8);
    check("sync0/1 = 0xAC/0xCA", f[0] == 0xAC && f[1] == 0xCA);
    uint8_t seq = 0, st = 0, len = 0; char out[ACK_TEXT_MAX + 1];
    check("解出一筆", feed(f, n, &seq, &st, out, &len) == 1);
    check("seq=42", seq == 42);
    check("status=OK", st == ACK_OK);
    check("echo 還原", strcmp(out, "e80 sf 9") == 0 && len == 8);
}

static void test_empty_text(void)
{
    printf("[2] 空文字 ACK（len==0）\n");
    uint8_t f[ACK_FRAME_MAX];
    uint8_t n = AckProto_Build(f, 9, ACK_UNARMED, NULL, 0);
    check("長度 = 7", n == 7);
    uint8_t seq = 0, st = 0, len = 0; char out[ACK_TEXT_MAX + 1];
    check("解出一筆", feed(f, n, &seq, &st, out, &len) == 1);
    check("seq=9 status=UNARMED len=0", seq == 9 && st == ACK_UNARMED && len == 0);
    check("out 為空字串", out[0] == '\0');
}

static void test_crc_reject(void)
{
    printf("[3] CRC 拒絕\n");
    uint8_t f[ACK_FRAME_MAX];
    uint8_t n = AckProto_Build(f, 1, ACK_BADARG, "bad", 3);
    f[2] ^= 0x01;   /* 翻 seq → CRC 不符 */
    uint8_t seq, st, len; char out[ACK_TEXT_MAX + 1];
    check("毀損框不交出", feed(f, n, &seq, &st, out, &len) == 0);
}

static void test_resync(void)
{
    printf("[4] 前置垃圾 resync\n");
    uint8_t f[ACK_FRAME_MAX];
    uint8_t n = AckProto_Build(f, 7, ACK_REJECTED, "bench", 5);
    uint8_t g[80]; int gi = 0;
    g[gi++] = 0x00; g[gi++] = 0xAC; g[gi++] = 0x12; g[gi++] = 0xAC;  /* 假 sync0 混雜 */
    memcpy(&g[gi], f, n); gi += n;
    uint8_t seq = 0, st = 0, len = 0; char out[ACK_TEXT_MAX + 1];
    check("垃圾後仍解出", feed(g, gi, &seq, &st, out, &len) == 1 &&
                          seq == 7 && st == ACK_REJECTED && strcmp(out, "bench") == 0);
}

static void test_coexist_stream(void)
{
    printf("[5] 與遙測位元組流並排（ACK sync 不被非 sync 位元組誤觸）\n");
    /* 模擬地面站：遙測 sync 0xA5/0x5A 與雜訊在前，ACK 幀在後，AckRx 只解出 ACK。 */
    uint8_t f[ACK_FRAME_MAX];
    uint8_t n = AckProto_Build(f, 3, ACK_OK, "role", 4);
    uint8_t s[80]; int si = 0;
    /* 遙測 sync + 雜訊，含「孤立 0xAC 後接非 0xCA」以驗證 resync；但不放完整 0xAC/0xCA 假 sync
     * 緊鄰真幀（那會如同任何 sync 框架般吞掉真幀頭部——屬正常行為，靠 burst 重送化解）。 */
    uint8_t noise[] = { 0xA5, 0x5A, 0x11, 0x22, 0xAC, 0x00, 0xA5, 0x5A };
    memcpy(&s[si], noise, sizeof(noise)); si += (int)sizeof(noise);
    memcpy(&s[si], f, n); si += n;
    uint8_t seq = 0, st = 0, len = 0; char out[ACK_TEXT_MAX + 1];
    check("雜訊後解出正確 ACK", feed(s, si, &seq, &st, out, &len) == 1 &&
                                seq == 3 && strcmp(out, "role") == 0);
}

static void test_status_str(void)
{
    printf("[6] status 字串\n");
    check("OK",       strcmp(ack_status_str(ACK_OK), "OK") == 0);
    check("UNKNOWN",  strcmp(ack_status_str(ACK_UNKNOWN), "UNKNOWN") == 0);
    check("BADARG",   strcmp(ack_status_str(ACK_BADARG), "BADARG") == 0);
    check("UNARMED",  strcmp(ack_status_str(ACK_UNARMED), "UNARMED") == 0);
    check("REJECTED", strcmp(ack_status_str(ACK_REJECTED), "REJECTED") == 0);
}

int main(void)
{
    printf("==== ack_proto 測試 ====\n");
    test_roundtrip();
    test_empty_text();
    test_crc_reject();
    test_resync();
    test_coexist_stream();
    test_status_str();
    printf("---- %d/%d 通過 ----\n", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
