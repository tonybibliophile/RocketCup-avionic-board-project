/*
 * test_link_proto.c — 板間鏈路封包契約測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 *   [1] LinkPacket_t 大小 = 53 bytes（含 echo-ACK ack_state + VF h/v 中繼 + erase_pct + erase_req），
 *       欄位 offset 逐一鎖定。★改動大小＝主/副兩板必須同時重燒（不影響地面下鏈 TelemetryPacket_t）。
 *   [2] LinkProto_Build → LinkRx_Feed 往返一致（含 sync 對齊與 CRC）
 *   [3] 單一位元翻轉 → CRC 不符 → 不吐封包
 *   [4] 前綴雜訊 / 連續兩筆 → 正確對齊並解出
 *   [5] 連續 sync0 不會卡死對齊
 */
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "link_proto.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static LinkStatus_t sample_status(void) {
    LinkStatus_t st;
    st.board_id    = LINK_BOARD_PRIMARY;
    st.seq         = 0x2A;
    st.fsm_state   = 3;            /* STATE_COAST */
    st.flags       = TELEM_FLAG_DROGUE_FIRED | TELEM_FLAG_FAILSAFE;
    st.tick_ms     = 123456;
    st.h_est_cm    = 25032;        /* 250.32 m */
    st.v_est_cms   = -1850;        /* -18.50 m/s */
    st.baro_alt_cm = 24990;
    st.a_z_cg      = -981;         /* -9.81 g (cg) */
    st.ack_state   = 6;            /* echo：我已採納對端 STATE_DESCENT */
    st.q_w         = 9950;
    st.q_x         = 100;
    st.q_y         = 50;
    st.q_z         = 0;
    st.main_arb    = 2;            /* SERVO_ARB_MSG_DRIVING */
    st.flash_ready = 1;
    st.erase_pct   = 75;
    st.erase_req   = 7;
    st.vf_h_cm     = 24950;        /* 249.50 m */
    st.vf_v_cms    = -1480;        /* -14.80 m/s */
    st.bmi_mag_cg  = 1015;         /* 10.15 g */
    st.adxl_mag_cg = 1032;         /* 10.32 g */
    st.profile_flags = TELEM_PROFILE_SELF_ELEVATOR;
    st.cmd_flags   = LINK_CMD_DEPLOY_DROGUE | LINK_CMD_DEPLOY_MAIN;
    return st;
}

static void test_layout(void) {
    printf("[1] 封包大小與欄位 offset（解碼契約）\n");
    check("sizeof(LinkPacket_t) == 53", sizeof(LinkPacket_t) == 53);
    check("LINK_PACKET_SIZE == 53",     LINK_PACKET_SIZE == 53);
#define OFF(field, expect) \
    check("offsetof " #field " == " #expect, offsetof(LinkPacket_t, field) == (expect))
    OFF(sync0,       0);
    OFF(sync1,       1);
    OFF(board_id,    2);
    OFF(seq,         3);
    OFF(fsm_state,   4);
    OFF(flags,       5);
    OFF(tick_ms,     6);
    OFF(h_est_cm,    10);
    OFF(v_est_cms,   14);
    OFF(baro_alt_cm, 18);
    OFF(a_z_cg,      22);
    OFF(ack_state,   24);
    OFF(q_w,         25);
    OFF(q_x,         27);
    OFF(q_y,         29);
    OFF(q_z,         31);
    OFF(main_arb,    33);
    OFF(flash_ready, 34);
    OFF(erase_pct,   35);
    OFF(vf_h_cm,     36);
    OFF(vf_v_cms,    40);
    OFF(bmi_mag_cg,  44);
    OFF(adxl_mag_cg, 46);
    OFF(profile_flags, 48);
    OFF(cmd_flags,   49);
    OFF(erase_req,   50);
    OFF(crc16,       51);
#undef OFF
}

static void test_roundtrip(void) {
    printf("[2] Build -> Feed 往返一致\n");
    LinkStatus_t st = sample_status();
    uint8_t buf[LINK_PACKET_SIZE];
    uint16_t n = LinkProto_Build(buf, &st);
    check("Build 回傳長度 == 53", n == LINK_PACKET_SIZE);
    check("buf[0],buf[1] == sync", buf[0] == LINK_SYNC0 && buf[1] == LINK_SYNC1);

    LinkRx_t rx; LinkRx_Init(&rx);
    LinkPacket_t out; memset(&out, 0, sizeof(out));
    int got = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    }
    check("恰好解出 1 筆", got == 1);
    check("board_id 一致",    out.board_id    == st.board_id);
    check("seq 一致",         out.seq         == st.seq);
    check("fsm_state 一致",   out.fsm_state   == st.fsm_state);
    check("flags 一致",       out.flags       == st.flags);
    check("tick_ms 一致",     out.tick_ms     == st.tick_ms);
    check("h_est_cm 一致",    out.h_est_cm    == st.h_est_cm);
    check("v_est_cms 一致",   out.v_est_cms   == st.v_est_cms);
    check("baro_alt_cm 一致", out.baro_alt_cm == st.baro_alt_cm);
    check("a_z_cg 一致",      out.a_z_cg      == st.a_z_cg);
    check("ack_state 一致",   out.ack_state   == st.ack_state);
    check("q_w 一致",         out.q_w         == st.q_w);
    check("q_x 一致",         out.q_x         == st.q_x);
    check("q_y 一致",         out.q_y         == st.q_y);
    check("q_z 一致",         out.q_z         == st.q_z);
    check("main_arb 一致",    out.main_arb    == st.main_arb);
    check("flash_ready 一致", out.flash_ready == st.flash_ready);
    check("erase_pct 一致",   out.erase_pct   == st.erase_pct);
    check("erase_req 一致",   out.erase_req   == st.erase_req);
    check("vf_h_cm 一致",     out.vf_h_cm     == st.vf_h_cm);
    check("vf_v_cms 一致",    out.vf_v_cms    == st.vf_v_cms);
    check("bmi_mag_cg 一致",  out.bmi_mag_cg  == st.bmi_mag_cg);
    check("adxl_mag_cg 一致", out.adxl_mag_cg == st.adxl_mag_cg);
    check("profile_flags 一致", out.profile_flags == st.profile_flags);
    check("cmd_flags 一致",   out.cmd_flags   == st.cmd_flags);
}

static void test_bad_crc(void) {
    printf("[3] 位元翻轉 → CRC 不符 → 不吐封包\n");
    LinkStatus_t st = sample_status();
    uint8_t buf[LINK_PACKET_SIZE];
    LinkProto_Build(buf, &st);
    buf[10] ^= 0x01;               /* 翻轉酬載一個 bit */

    LinkRx_t rx; LinkRx_Init(&rx);
    LinkPacket_t out;
    int got = 0;
    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++)
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    check("壞 CRC 解出 0 筆", got == 0);
}

static void test_noise_and_two_frames(void) {
    printf("[4] 前綴雜訊 + 連續兩筆\n");
    LinkStatus_t st = sample_status();
    uint8_t buf[LINK_PACKET_SIZE];
    LinkProto_Build(buf, &st);

    LinkRx_t rx; LinkRx_Init(&rx);
    LinkPacket_t out;
    int got = 0;
    const uint8_t noise[] = {0x00, 0xFF, 0x12, 0xC3, 0x99, 0x5A, 0x01};
    for (size_t i = 0; i < sizeof(noise); i++) LinkRx_Feed(&rx, noise[i], &out);
    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++)
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++)
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    check("雜訊後仍解出 2 筆", got == 2);
}

static void test_consecutive_sync0(void) {
    printf("[5] 連續 sync0 不卡死對齊\n");
    LinkStatus_t st = sample_status();
    uint8_t buf[LINK_PACKET_SIZE];
    LinkProto_Build(buf, &st);

    LinkRx_t rx; LinkRx_Init(&rx);
    LinkPacket_t out;
    int got = 0;
    LinkRx_Feed(&rx, LINK_SYNC0, &out);   /* 多餘的 sync0 */
    LinkRx_Feed(&rx, LINK_SYNC0, &out);   /* 再一個 sync0 */
    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++)
        if (LinkRx_Feed(&rx, buf[i], &out)) got++;
    check("連續 sync0 後仍解出 1 筆", got == 1);
}

static void test_qos_counters(void) {
    printf("[6] 鏈路品質統計（ok / crc_err / resync）\n");
    LinkStatus_t st = sample_status();
    uint8_t buf[LINK_PACKET_SIZE];
    LinkProto_Build(buf, &st);

    LinkRx_t rx; LinkRx_Init(&rx);
    LinkPacket_t out;
    check("初始統計歸零", rx.ok == 0 && rx.crc_err == 0 && rx.resync == 0);

    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++) LinkRx_Feed(&rx, buf[i], &out);
    check("好封包 → ok=1", rx.ok == 1 && rx.crc_err == 0);

    uint8_t bad[LINK_PACKET_SIZE];
    memcpy(bad, buf, LINK_PACKET_SIZE);
    bad[10] ^= 0x01;                    /* 翻轉酬載 → CRC 不符 */
    for (uint16_t i = 0; i < LINK_PACKET_SIZE; i++) LinkRx_Feed(&rx, bad[i], &out);
    check("壞 CRC → crc_err=1（ok 不變）", rx.crc_err == 1 && rx.ok == 1);

    uint32_t r0 = rx.resync;
    LinkRx_Feed(&rx, LINK_SYNC0, &out); /* sync0 後接非 sync1/非 sync0 */
    LinkRx_Feed(&rx, 0x00, &out);
    check("假 sync → resync++", rx.resync == r0 + 1);
}

int main(void) {
    printf("=== test_link_proto：板間鏈路封包契約 ===\n");
    test_layout();
    test_roundtrip();
    test_bad_crc();
    test_noise_and_two_frames();
    test_consecutive_sync0();
    test_qos_counters();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
