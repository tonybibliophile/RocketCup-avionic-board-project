/*
 * test_telemetry.c — 下行遙測封包契約測試（P1，純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 這份測試「就是」地面站解碼契約的機器驗證：
 *   [1] CRC-16/CCITT-FALSE 黃金向量："123456789" → 0x29B1（crc16.h 單一實作）
 *   [2] TelemetryPacket_t 大小 = 116 bytes（93 + 本板 VF 摘要 8B + peer baro/accel/VF 14B
 *       + peer_bench_arb 1B）
 *   [3] 每個欄位的 byte offset 逐一鎖定（GroundStation/telemetry_decoder.py
 *       的 struct.unpack 格式依據此表）
 *   [4] CRC 欄位語意：覆蓋前 sizeof-2 bytes
 * 任何人改動 telemetry.h 封包佈局，此測試必須紅 → 強迫同步更新解碼器。
 */
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "telemetry.h"
#include "crc16.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static void test_crc_golden(void) {
    printf("[1] CRC-16/CCITT-FALSE 黃金向量\n");
    const uint8_t v[] = "123456789";
    check("\"123456789\" -> 0x29B1", crc16_ccitt_false(v, 9) == 0x29B1);
    check("空輸入 -> init 0xFFFF",    crc16_ccitt_false(v, 0) == 0xFFFF);
}

static void test_packet_layout(void) {
    printf("[2] 封包大小與欄位 offset（地面站解碼契約）\n");
    check("sizeof(TelemetryPacket_t) == 116", sizeof(TelemetryPacket_t) == 116);
    check("TELEM_PACKET_SIZE == 116",         TELEM_PACKET_SIZE == 116);

#define OFF(field, expect) \
    check("offsetof " #field " == " #expect, offsetof(TelemetryPacket_t, field) == (expect))

    OFF(sync0,         0);
    OFF(sync1,         1);
    OFF(seq,           2);
    OFF(fsm_state,     3);
    OFF(tick_ms,       4);
    OFF(ekf_pos_z_cm,  8);
    OFF(ekf_vel_z_cms, 12);
    OFF(ekf_q0,        16);
    OFF(ekf_q1,        18);
    OFF(ekf_q2,        20);
    OFF(ekf_q3,        22);
    OFF(baro_alt_cm,   24);
    OFF(baro_press_pa, 28);
    OFF(imu_ax_mg,     32);
    OFF(imu_ay_mg,     34);
    OFF(imu_az_mg,     36);
    OFF(gyro_x_dps,    38);
    OFF(gyro_y_dps,    40);
    OFF(gyro_z_dps,    42);
    OFF(hg_ax_cg,      44);
    OFF(hg_ay_cg,      46);
    OFF(hg_az_cg,      48);
    OFF(mag_x_mg,      50);
    OFF(mag_y_mg,      52);
    OFF(mag_z_mg,      54);
    OFF(gps_lat_1e6,   56);
    OFF(gps_lon_1e6,   60);
    OFF(gps_alt_m,     64);
    OFF(gps_sats,      66);
    OFF(gps_fix,       67);
    OFF(bat_mv,        68);
    OFF(cpu_main_x10,  70);
    OFF(cpu_ekf_x10,   72);
    OFF(flags,         74);
    OFF(health_bits,   75);
    OFF(sensor_bits,   76);
    OFF(vf_pos_z_cm,   77);
    OFF(vf_vel_z_cms,  81);
    OFF(peer_fsm_state,85);
    OFF(peer_flags,    86);
    OFF(peer_h_cm,     87);
    OFF(peer_v_cms,    91);
    OFF(peer_baro_cm,  95);
    OFF(peer_link,     99);
    OFF(peer_loss_pmil,100);
    OFF(peer_az_cg,    102);
    OFF(peer_vf_h_cm,  104);
    OFF(peer_vf_v_cms, 108);
    OFF(arm_flags,     112);
    OFF(peer_bench_arb,113);
    OFF(crc16,         114);
#undef OFF
}

static void test_packet_crc_semantics(void) {
    printf("[3] 封包 CRC 語意（覆蓋前 sizeof-2 bytes，模擬地面站驗證流程）\n");
    TelemetryPacket_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.sync0        = TELEM_SYNC0;
    pkt.sync1        = TELEM_SYNC1;
    pkt.seq          = 42;
    pkt.fsm_state    = 3;            /* STATE_COAST */
    pkt.tick_ms      = 123456;
    pkt.ekf_pos_z_cm = 25032;        /* 250.32 m */
    pkt.flags        = TELEM_FLAG_DROGUE_FIRED | TELEM_FLAG_FAILSAFE;
    pkt.health_bits  = 0x01;
    pkt.sensor_bits  = 0x04;
    pkt.crc16 = crc16_ccitt_false((const uint8_t *)&pkt, (uint16_t)(sizeof(pkt) - 2));

    /* 地面站視角：raw bytes 進來，sync 對齊 + 重算 CRC 比對 */
    const uint8_t *raw = (const uint8_t *)&pkt;
    check("sync bytes 位於 [0],[1]", raw[0] == 0xA5 && raw[1] == 0x5A);
    uint16_t crc_calc = crc16_ccitt_false(raw, (uint16_t)(sizeof(pkt) - 2));
    uint16_t crc_recv = (uint16_t)(raw[114] | ((uint16_t)raw[115] << 8));  /* little-endian */
    check("重算 CRC == 封包尾 2 bytes (LE)", crc_calc == crc_recv);

    /* 位元翻轉必須被偵測 */
    TelemetryPacket_t bad = pkt;
    ((uint8_t *)&bad)[10] ^= 0x01;
    check("單一位元翻轉 → CRC 不符",
          crc16_ccitt_false((const uint8_t *)&bad, (uint16_t)(sizeof(bad) - 2)) != bad.crc16);
}

int main(void) {
    printf("=== test_telemetry：下行遙測封包契約（P1） ===\n");
    test_crc_golden();
    test_packet_layout();
    test_packet_crc_semantics();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
