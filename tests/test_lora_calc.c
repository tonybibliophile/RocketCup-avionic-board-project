/*
 * test_lora_calc.c — LoRa 參數換算 + 鏈路統計單元測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 驗證 lora_calc.h（driver / 命令解析器 / 本測試共用同一份）：
 *   [1] 頻寬 index ↔ kHz
 *   [2] 符號時間 / LDRO 判定
 *   [3] LR1121 SetRfFrequency 參數位元組（直接 Hz 大端，非 SX126x Frf 公式）
 *   [4] E22-400 頻率(MHz) ↔ 通道(CH) + 邊界
 *   [5] LoRa 空中時間 time-on-air（與已知值比對）
 *   [6] 鏈路統計：封包 / CRC 錯誤 / RSSI、SNR 之 min/max/avg / 封包率
 *   [7] LR1121 CalibImage 頻段位元組（換頻段影像校準）
 */
#include <stdio.h>
#include <stdint.h>
#include "lora_calc.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static void test_bw(void)
{
    printf("[1] 頻寬 index -> kHz\n");
    check("BW125 idx=0x04", lora_bw_to_khz(0x04) == 125);
    check("BW250 idx=0x05", lora_bw_to_khz(0x05) == 250);
    check("BW500 idx=0x06", lora_bw_to_khz(0x06) == 500);
    check("BW62.5 idx=0x03", lora_bw_to_khz(0x03) == 63);
    check("未知 idx=0x7F -> 0", lora_bw_to_khz(0x7F) == 0);
    check("lora_bw_valid(0x04)=1", lora_bw_valid(0x04) == 1);
    check("lora_bw_valid(0x7F)=0", lora_bw_valid(0x7F) == 0);
}

static void test_symbol_ldro(void)
{
    printf("[2] 符號時間 / LDRO\n");
    /* Ts = 2^SF / BW。SF7@125k = 128/125k = 1.024ms = 1024us */
    check("Ts SF7@125k = 1024us", lora_symbol_time_us(7, 0x04) == 1024);
    /* SF12@125k = 4096/125k = 32.768ms */
    check("Ts SF12@125k = 32768us", lora_symbol_time_us(12, 0x04) == 32768);
    check("Ts 非法 SF=4 -> 0", lora_symbol_time_us(4, 0x04) == 0);
    check("Ts 非法 BW -> 0", lora_symbol_time_us(9, 0x7F) == 0);

    /* LDRO 門檻 16ms：SF11@125k=16.384ms 開、SF10@125k=8.192ms 關 */
    check("LDRO SF11@125k = 1", lora_ldro_required(11, 0x04) == 1);
    check("LDRO SF12@125k = 1", lora_ldro_required(12, 0x04) == 1);
    check("LDRO SF10@125k = 0", lora_ldro_required(10, 0x04) == 0);
    check("LDRO SF12@500k = 0 (8.192ms)", lora_ldro_required(12, 0x06) == 0);
}

static void test_freq_bytes(void)
{
    printf("[3] LR1121 頻率位元組（直接 Hz 大端）\n");
    uint8_t b[4];
    lr1121_freq_to_bytes(920000000UL, b);
    /* 920000000 = 0x36D61600 */
    check("920MHz b0=0x36", b[0] == 0x36);
    check("920MHz b1=0xD6", b[1] == 0xD6);
    check("920MHz b2=0x16", b[2] == 0x16);
    check("920MHz b3=0x00", b[3] == 0x00);
    lr1121_freq_to_bytes(915000000UL, b);
    /* 915000000 = 0x3689CAC0 */
    check("915MHz 大端正確",
          b[0] == 0x36 && b[1] == 0x89 && b[2] == 0xCA && b[3] == 0xC0);
}

static void test_calib_image(void)
{
    printf("[7] LR1121 CalibImage 頻段位元組（換頻段影像校準）\n");
    uint8_t cb[2];
    lr1121_calib_image_bytes(920000000UL, cb);
    /* 中心 920MHz ±10 → 910..930；4MHz 步階：floor(910/4)=227, ceil(930/4)=233 */
    check("920MHz f1=227 (908MHz)", cb[0] == 227);
    check("920MHz f2=233 (932MHz)", cb[1] == 233);
    /* 校準頻段須確實包住中心頻率 */
    check("f1*4 <= 920 <= f2*4",
          (uint32_t)cb[0] * 4u <= 920u && 920u <= (uint32_t)cb[1] * 4u);
    /* 低頻端保護：不下溢 */
    lr1121_calib_image_bytes(5000000UL, cb);
    check("5MHz 不下溢 f1=0", cb[0] == 0);
}

static void test_e22_ch(void)
{
    printf("[4] E22-400 MHz <-> CH\n");
    uint8_t ch = 0xFF;
    check("432MHz 合法", e22_mhz_to_ch(432, &ch) == 1 && ch == 22);
    check("410MHz 邊界 CH0", e22_mhz_to_ch(410, &ch) == 1 && ch == 0);
    check("493MHz 邊界 CH83", e22_mhz_to_ch(493, &ch) == 1 && ch == 83);
    check("409MHz 超下界拒絕", e22_mhz_to_ch(409, &ch) == 0);
    check("494MHz 超上界拒絕", e22_mhz_to_ch(494, &ch) == 0);
}

static void test_airtime(void)
{
    printf("[5] LoRa 空中時間（顯式表頭 + CRC on）\n");
    /* SF7, BW125, CR4/5, preamble=8, payload=79 → 約 138ms 量級。
     * 用整數公式自洽性檢查：與手算對照（Ts=1024us）。
     *  t_pre = (8+4.25)*1024 = 12.25*1024 = 12544us
     *  payloadSymb = 8 + ceil((8*79-4*7+28+16)/(4*7))*5
     *              = 8 + ceil((632-28+28+16)/28)*5 = 8 + ceil(648/28)*5
     *              = 8 + 24*5 = 128 ; ceil(648/28)=24 (23.14→24)
     *  t_payload = 128*1024 = 131072us ; ToA = 143616us */
    uint32_t toa = lora_time_on_air_us(7, 0x04, 1, 8, 79);
    check("SF7/BW125/CR45/79B ToA = 143616us", toa == 143616);

    /* 越高 SF 空中時間越長（單調） */
    uint32_t toa_sf9  = lora_time_on_air_us(9,  0x04, 1, 8, 79);
    uint32_t toa_sf12 = lora_time_on_air_us(12, 0x04, 1, 8, 79);
    check("SF9 > SF7", toa_sf9 > toa);
    check("SF12 > SF9", toa_sf12 > toa_sf9);

    /* 越大頻寬空中時間越短 */
    uint32_t toa_bw500 = lora_time_on_air_us(7, 0x06, 1, 8, 79);
    check("BW500 < BW125", toa_bw500 < toa);

    check("非法參數回傳 0", lora_time_on_air_us(7, 0x7F, 1, 8, 79) == 0);
    check("非法 CR 回傳 0", lora_time_on_air_us(7, 0x04, 9, 8, 79) == 0);
}

static void test_stats(void)
{
    printf("[6] 鏈路統計\n");
    lora_stats_t s;
    lora_stats_reset(&s);
    check("初始 pkt_ok=0", s.pkt_ok == 0);
    check("初始 crc_err=0", s.crc_err == 0);

    /* 三筆有效（920，有 RSSI/SNR），時間 1000/2000/3000ms */
    lora_stats_on_packet(&s, 1, 1, -80, 1, 40, 1000);
    lora_stats_on_packet(&s, 1, 1, -90, 1, 20, 2000);
    lora_stats_on_packet(&s, 1, 1, -85, 1, 30, 3000);
    /* 一筆 CRC 錯 */
    lora_stats_on_packet(&s, 0, 0, 0, 0, 0, 3500);

    check("pkt_ok=3", s.pkt_ok == 3);
    check("crc_err=1", s.crc_err == 1);
    check("rssi_last=-85", s.rssi_last == -85);
    check("rssi_min=-90", s.rssi_min == -90);
    check("rssi_max=-80", s.rssi_max == -80);
    check("rssi_avg=-85", lora_stats_rssi_avg(&s) == -85);
    check("snr_min=20", s.snr_min == 20);
    check("snr_max=40", s.snr_max == 40);
    check("snr_avg=30", lora_stats_snr_avg(&s) == 30);

    /* 封包率：3 筆跨 1000→3000ms = 2000ms → 1.5 pkt/s → rate_x10=15 */
    check("rate_x10=15 (1.5 pkt/s)", lora_stats_rate_x10(&s) == 15);

    /* has_rssi/has_snr = 0（沒有量測值的封包）不應污染 rssi_cnt 與平均。
     * 433 在 E22 未開 REG3 bit7、或該包沒剝到 RSSI 位元組時就是走這條路徑。 */
    lora_stats_t s2;
    lora_stats_reset(&s2);
    lora_stats_on_packet(&s2, 1, 0, 0, 0, 0, 1000);
    lora_stats_on_packet(&s2, 1, 0, 0, 0, 0, 2000);
    check("無量測 pkt_ok=2", s2.pkt_ok == 2);
    check("無量測 rssi_cnt=0", s2.rssi_cnt == 0);
    check("無量測 rssi_avg=0 (除零保護)", lora_stats_rssi_avg(&s2) == 0);

    /* 空統計封包率 0（除零保護） */
    lora_stats_t s3;
    lora_stats_reset(&s3);
    check("空統計 rate_x10=0", lora_stats_rate_x10(&s3) == 0);
}

int main(void)
{
    printf("==== lora_calc 測試 ====\n");
    test_bw();
    test_symbol_ldro();
    test_freq_bytes();
    test_e22_ch();
    test_airtime();
    test_stats();
    test_calib_image();
    printf("---- %d/%d 通過 ----\n", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
