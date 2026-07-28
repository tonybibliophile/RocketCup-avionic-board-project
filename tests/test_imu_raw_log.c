/*
 * test_imu_raw_log.c — SD 原始 IMU 二進位記錄封包契約測試（S6/S8，純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 *   [1] sizeof(ImuRawRecord_t) == 128（imu_raw_log.h 已有 _Static_assert，此處
 *       另以執行期斷言鎖定，並逐欄位 offsetof 對照 imu_raw_log.h 的欄位表）
 *   [2] CRC-16/CCITT-FALSE pack/decode 往返一致
 *   [3] 黃金位元組向量——與 ground_station/tools/imu_raw_decoder.py 的
 *       make_selftest_record(7) 用同一組固定輸入值，兩邊產出的 128 bytes
 *       必須逐位元組相同（用 `python3 imu_raw_decoder.py --selftest` 各自驗證
 *       CRC 往返，本檔額外鎖死跨語言位元組相容，任一邊改動封包版面會讓此測試變紅）。
 */
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "imu_raw_log.h"
#include "crc16.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static void test_layout(void) {
    printf("[1] 封包大小與欄位 offset\n");
    check("sizeof(ImuRawRecord_t) == 128", sizeof(ImuRawRecord_t) == 128);
#define OFF(field, expect) \
    check("offsetof " #field " == " #expect, offsetof(ImuRawRecord_t, field) == (expect))
    OFF(magic,             0);
    OFF(ver,                2);
    OFF(n_gyro,             3);
    OFF(n_acc,              4);
    OFF(flags,              5);
    OFF(seq,                6);
    OFF(t_cyc,              8);
    OFF(fsm_state,          12);
    OFF(_rsv,               13);
    OFF(baro_temp_c_x100,   14);
    OFF(baro_press_pa,      16);
    OFF(gyro,               20);
    OFF(acc,                80);
    OFF(gyro_dt_q,          104);
    OFF(acc_t_off_q,        124);
    OFF(crc16,              126);
#undef OFF
}

/* 與 ground_station/tools/imu_raw_decoder.py 的 make_selftest_record(seq) 用完全
 * 同一組固定輸入值——任一邊改動欄位順序/型別，本函式產出的 bytes 就會跟 Python
 * 端的黃金向量對不上。 */
static void build_golden_record(ImuRawRecord_t *rec, uint16_t seq) {
    memset(rec, 0, sizeof(*rec));
    rec->magic[0] = IMU_RAW_LOG_MAGIC0;
    rec->magic[1] = IMU_RAW_LOG_MAGIC1;
    rec->ver      = 1U;
    rec->n_gyro   = 10U;
    rec->n_acc    = 4U;
    rec->flags    = 0x08U;
    rec->seq      = seq;
    rec->t_cyc    = 123456789U;
    rec->fsm_state = 3U;
    rec->_rsv      = 0U;
    rec->baro_temp_c_x100 = 2512;
    rec->baro_press_pa    = 98412U;
    for (int i = 0; i < 10; i++) {
        rec->gyro[i][0] = (int16_t)(100 * i);
        rec->gyro[i][1] = (int16_t)(-50 * i);
        rec->gyro[i][2] = (int16_t)(25 * i);
        rec->gyro_dt_q[i] = (uint16_t)(21000 + i);
    }
    for (int i = 0; i < 4; i++) {
        rec->acc[i][0] = 1000;
        rec->acc[i][1] = 2000;
        rec->acc[i][2] = (int16_t)(-3000 + i);
    }
    rec->acc_t_off_q = -17;
    rec->crc16 = crc16_ccitt_false((const uint8_t *)rec, (uint16_t)(sizeof(*rec) - 2U));
}

static void test_crc_roundtrip(void) {
    printf("[2] CRC pack/decode 往返\n");
    ImuRawRecord_t rec;
    build_golden_record(&rec, 7);
    uint16_t crc_recalc = crc16_ccitt_false((const uint8_t *)&rec, (uint16_t)(sizeof(rec) - 2U));
    check("重算 CRC == 封包內 crc16 欄位", crc_recalc == rec.crc16);

    ImuRawRecord_t bad = rec;
    ((uint8_t *)&bad)[10] ^= 0x01U;
    uint16_t crc_bad = crc16_ccitt_false((const uint8_t *)&bad, (uint16_t)(sizeof(bad) - 2U));
    check("單一位元翻轉 → CRC 不符", crc_bad != bad.crc16);
}

static void test_golden_vector(void) {
    printf("[3] 跨語言黃金位元組向量（對照 imu_raw_decoder.py make_selftest_record(7)）\n");
    /* 由 `python3 ground_station/tools/imu_raw_decoder.py` 產生 make_selftest_record(7)
     * 的實際輸出後逐位元組轉錄，非手推——與 Python 端變成唯一真相來源同步的機制是
     * 「兩邊都用同一組固定輸入值」，見上方 build_golden_record() 頂端註解。 */
    static const uint8_t golden[128] = {
        0x52, 0x49, 0x01, 0x0A, 0x04, 0x08, 0x07, 0x00, 0x15, 0xCD, 0x5B, 0x07, 0x03, 0x00, 0xD0, 0x09,
        0x6C, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0xCE, 0xFF, 0x19, 0x00,
        0xC8, 0x00, 0x9C, 0xFF, 0x32, 0x00, 0x2C, 0x01, 0x6A, 0xFF, 0x4B, 0x00, 0x90, 0x01, 0x38, 0xFF,
        0x64, 0x00, 0xF4, 0x01, 0x06, 0xFF, 0x7D, 0x00, 0x58, 0x02, 0xD4, 0xFE, 0x96, 0x00, 0xBC, 0x02,
        0xA2, 0xFE, 0xAF, 0x00, 0x20, 0x03, 0x70, 0xFE, 0xC8, 0x00, 0x84, 0x03, 0x3E, 0xFE, 0xE1, 0x00,
        0xE8, 0x03, 0xD0, 0x07, 0x48, 0xF4, 0xE8, 0x03, 0xD0, 0x07, 0x49, 0xF4, 0xE8, 0x03, 0xD0, 0x07,
        0x4A, 0xF4, 0xE8, 0x03, 0xD0, 0x07, 0x4B, 0xF4, 0x08, 0x52, 0x09, 0x52, 0x0A, 0x52, 0x0B, 0x52,
        0x0C, 0x52, 0x0D, 0x52, 0x0E, 0x52, 0x0F, 0x52, 0x10, 0x52, 0x11, 0x52, 0xEF, 0xFF, 0x18, 0xCF,
    };
    ImuRawRecord_t rec;
    build_golden_record(&rec, 7);
    check("sizeof golden == sizeof(ImuRawRecord_t)", sizeof(golden) == sizeof(rec));
    check("C 端 pack 結果與 Python 黃金向量逐位元組相同",
          memcmp(&rec, golden, sizeof(golden)) == 0);
}

int main(void) {
    printf("=== test_imu_raw_log：SD 原始 IMU 記錄封包契約（S6） ===\n");
    test_layout();
    test_crc_roundtrip();
    test_golden_vector();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
