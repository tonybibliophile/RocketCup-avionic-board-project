/*
 * test_telem_rx.c — 下行遙測接收端解析契約測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 *   [1] TelemetryPacket_t 大小 = 99 bytes（與 TX/地面站解碼契約一致）
 *   [2] 組一筆有效封包 → TelemRx_Feed 往返一致（sync 對齊 + CRC）
 *   [3] 單一位元翻轉 → CRC 不符 → 不吐封包、crc_err++
 *   [4] 前綴雜訊 + 連續兩筆 → 正確對齊並解出 2 筆
 *   [5] 連續 sync0 不卡死對齊
 *   [6] 分段餵入（模擬 UART 分片）仍解出 1 筆
 *   [7] 兩包之間插入 1 個雜散位元組（E22 附加的 RSSI byte）→ 第二包仍收到
 *   [8] 假 sync 吞掉真封包起點 → TelemRx_Realign 回溯重對齊要救回真封包
 *   [9] 封包「中間」插入 1 個位元組（子封包邊界的 RSSI byte）→ 只賠這一包
 */
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "telem_rx.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

/* 組一筆有效 TelemetryPacket_t 到 buf（填代表性欄位 + sync + CRC）。回傳長度。 */
static uint16_t build_sample(uint8_t *buf) {
    TelemetryPacket_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.sync0       = TELEM_SYNC0;
    pkt.sync1       = TELEM_SYNC1;
    pkt.seq         = 0x2A;
    pkt.fsm_state   = 3;            /* STATE_COAST */
    pkt.tick_ms     = 123456;
    pkt.ekf_pos_z_cm = 25032;       /* 250.32 m */
    pkt.baro_alt_cm  = 24990;
    pkt.gps_lat_1e6 = 25033000;     /* 25.033000°N */
    pkt.gps_lon_1e6 = 121564000;    /* 121.564000°E */
    pkt.gps_alt_m   = 250;
    pkt.gps_sats    = 9;
    pkt.gps_fix     = 1;
    pkt.bat_mv      = 11850;
    pkt.flags       = TELEM_FLAG_DROGUE_FIRED | TELEM_FLAG_FAILSAFE;
    pkt.crc16 = crc16_ccitt_false((const uint8_t *)&pkt,
                                  (uint16_t)(sizeof(pkt) - 2));
    memcpy(buf, &pkt, sizeof(pkt));
    return (uint16_t)sizeof(pkt);
}

static void test_size(void) {
    printf("[1] 封包大小（解碼契約）\n");
    check("sizeof(TelemetryPacket_t) == 99", sizeof(TelemetryPacket_t) == 99);
    check("TELEM_PACKET_SIZE == 99",          TELEM_PACKET_SIZE == 99);
}

static void test_roundtrip(void) {
    printf("[2] 組包 -> Feed 往返一致\n");
    uint8_t buf[TELEM_PACKET_SIZE];
    uint16_t n = build_sample(buf);
    check("buf[0],buf[1] == sync", buf[0] == TELEM_SYNC0 && buf[1] == TELEM_SYNC1);

    TelemRx_t rx; TelemRx_Init(&rx);
    TelemetryPacket_t out; memset(&out, 0, sizeof(out));
    int got = 0;
    for (uint16_t i = 0; i < n; i++)
        if (TelemRx_Feed(&rx, buf[i], &out)) got++;
    check("恰好解出 1 筆", got == 1);
    check("rx.ok == 1",        rx.ok == 1);
    check("seq 一致",          out.seq == 0x2A);
    check("fsm_state 一致",    out.fsm_state == 3);
    check("tick_ms 一致",      out.tick_ms == 123456);
    check("ekf_pos_z_cm 一致", out.ekf_pos_z_cm == 25032);
    check("gps_lat_1e6 一致",  out.gps_lat_1e6 == 25033000);
    check("gps_lon_1e6 一致",  out.gps_lon_1e6 == 121564000);
    check("gps_sats 一致",     out.gps_sats == 9);
    check("flags 一致",        out.flags == (TELEM_FLAG_DROGUE_FIRED | TELEM_FLAG_FAILSAFE));
}

static void test_bad_crc(void) {
    printf("[3] 位元翻轉 → CRC 不符 → 不吐封包\n");
    uint8_t buf[TELEM_PACKET_SIZE];
    build_sample(buf);
    buf[10] ^= 0x01;               /* 翻轉酬載一個 bit */

    TelemRx_t rx; TelemRx_Init(&rx);
    TelemetryPacket_t out;
    int got = 0;
    for (uint16_t i = 0; i < TELEM_PACKET_SIZE; i++)
        if (TelemRx_Feed(&rx, buf[i], &out)) got++;
    check("壞 CRC 解出 0 筆", got == 0);
    check("crc_err == 1",     rx.crc_err == 1);
    check("ok == 0",          rx.ok == 0);
}

static void test_noise_and_two_frames(void) {
    printf("[4] 前綴雜訊 + 連續兩筆\n");
    uint8_t buf[TELEM_PACKET_SIZE];
    build_sample(buf);

    TelemRx_t rx; TelemRx_Init(&rx);
    TelemetryPacket_t out;
    int got = 0;
    const uint8_t noise[] = {0x00, 0xFF, 0x12, 0xA5, 0x99, 0x5A, 0x01};
    for (size_t i = 0; i < sizeof(noise); i++) TelemRx_Feed(&rx, noise[i], &out);
    for (uint16_t i = 0; i < TELEM_PACKET_SIZE; i++)
        if (TelemRx_Feed(&rx, buf[i], &out)) got++;
    for (uint16_t i = 0; i < TELEM_PACKET_SIZE; i++)
        if (TelemRx_Feed(&rx, buf[i], &out)) got++;
    check("雜訊後仍解出 2 筆", got == 2);
    check("rx.ok == 2",        rx.ok == 2);
}

static void test_consecutive_sync0(void) {
    printf("[5] 連續 sync0 不卡死對齊\n");
    uint8_t buf[TELEM_PACKET_SIZE];
    build_sample(buf);

    TelemRx_t rx; TelemRx_Init(&rx);
    TelemetryPacket_t out;
    int got = 0;
    TelemRx_Feed(&rx, TELEM_SYNC0, &out);   /* 多餘的 sync0 */
    TelemRx_Feed(&rx, TELEM_SYNC0, &out);   /* 再一個 sync0 */
    for (uint16_t i = 0; i < TELEM_PACKET_SIZE; i++)
        if (TelemRx_Feed(&rx, buf[i], &out)) got++;
    check("連續 sync0 後仍解出 1 筆", got == 1);
}

static void test_fragmented_feed(void) {
    printf("[6] 分段餵入（模擬 UART 分片）\n");
    uint8_t buf[TELEM_PACKET_SIZE];
    build_sample(buf);

    TelemRx_t rx; TelemRx_Init(&rx);
    TelemetryPacket_t out;
    int got = 0;
    /* 任意切成 3 段：1 / 40 / 其餘 */
    const uint16_t cut1 = 1, cut2 = 41;
    for (uint16_t i = 0; i < cut1; i++)            got += TelemRx_Feed(&rx, buf[i], &out);
    for (uint16_t i = cut1; i < cut2; i++)         got += TelemRx_Feed(&rx, buf[i], &out);
    for (uint16_t i = cut2; i < TELEM_PACKET_SIZE; i++) got += TelemRx_Feed(&rx, buf[i], &out);
    check("分段餵入仍解出 1 筆", got == 1);
}

/* E22 開啟 REG3 bit7 後，模組每收到一次「無線」封包就在 UART 尾端附加一個 RSSI 位元組。
 * 正常情況（一個應用封包 = 一個空中子封包）它落在兩包之間，接收端剝掉即可；
 * 但若封包被 E22 的 auto sub-packaging 拆成多個子封包，那個位元組會插進封包中間。
 * 這兩個案例鎖定「插入一個位元組」時解析器必須怎麼表現。 */
static void test_stray_byte_between_frames(void) {
    printf("[7] 兩包之間插入 1 個雜散位元組（RSSI byte 落在正確位置）\n");
    uint8_t buf[TELEM_PACKET_SIZE];
    build_sample(buf);

    TelemRx_t rx; TelemRx_Init(&rx);
    TelemetryPacket_t out;
    int got = 0;
    for (uint16_t i = 0; i < TELEM_PACKET_SIZE; i++) got += TelemRx_Feed(&rx, buf[i], &out);
    /* 0xA5 是最惡劣的值：剛好等於 sync0（-91dBm 換算就是 0xA5），必須不能卡死對齊 */
    got += TelemRx_Feed(&rx, TELEM_SYNC0, &out);
    for (uint16_t i = 0; i < TELEM_PACKET_SIZE; i++) got += TelemRx_Feed(&rx, buf[i], &out);
    check("兩包之間夾一個 0xA5 仍解出 2 筆", got == 2);
}

static void test_realign_after_false_sync(void) {
    printf("[8] 假 sync 吞掉真封包起點 → 回溯重對齊要救回來\n");
    uint8_t buf[TELEM_PACKET_SIZE];
    build_sample(buf);

    TelemRx_t rx; TelemRx_Init(&rx);
    TelemetryPacket_t out;
    int got = 0;

    /* 雜訊裡湊巧出現一組假的 A5 5A，緊接著才是真封包。
     * 解析器會先咬住假 sync 湊滿 117 bytes、CRC 失敗——而真封包的起點就落在
     * 這段緩衝的 offset 2。舊實作把整段丟光、從下一個位元組重找 sync，真封包
     * 連帶報銷（一次誤判賠兩包）；回溯重對齊要在緩衝內找回 offset 2 的 A5 5A。 */
    got += TelemRx_Feed(&rx, TELEM_SYNC0, &out);
    got += TelemRx_Feed(&rx, TELEM_SYNC1, &out);
    for (uint16_t i = 0; i < TELEM_PACKET_SIZE; i++) got += TelemRx_Feed(&rx, buf[i], &out);

    check("假 sync 記為 1 次 CRC 錯", rx.crc_err == 1);
    check("真封包仍解出（回溯重對齊生效）", got == 1);
    check("解出內容正確", out.seq == 0x2A && out.bat_mv == 11850);
}

static void test_stray_byte_inside_frame(void) {
    printf("[9] 封包中間插入 1 個位元組（子封包邊界的 RSSI byte）\n");
    uint8_t buf[TELEM_PACKET_SIZE];
    build_sample(buf);

    TelemRx_t rx; TelemRx_Init(&rx);
    TelemetryPacket_t out;
    int got = 0;

    /* 在 offset 64 插入一個位元組：內容已被撐開，這包無論如何都救不回來，
     * 契約是「只賠這一包」——下一包必須正常。 */
    for (uint16_t i = 0; i < 64; i++)                   got += TelemRx_Feed(&rx, buf[i], &out);
    got += TelemRx_Feed(&rx, 0x7F, &out);               /* 插入物 */
    for (uint16_t i = 64; i < TELEM_PACKET_SIZE; i++)   got += TelemRx_Feed(&rx, buf[i], &out);
    check("被插入的那包不會被交出", got == 0);
    check("計為 CRC 錯", rx.crc_err == 1);

    for (uint16_t i = 0; i < TELEM_PACKET_SIZE; i++)    got += TelemRx_Feed(&rx, buf[i], &out);
    check("下一包正常解出", got == 1);
}

int main(void) {
    printf("=== test_telem_rx：下行遙測接收端解析契約 ===\n");
    test_size();
    test_roundtrip();
    test_bad_crc();
    test_noise_and_two_frames();
    test_consecutive_sync0();
    test_fragmented_feed();
    test_stray_byte_between_frames();
    test_realign_after_false_sync();
    test_stray_byte_inside_frame();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
