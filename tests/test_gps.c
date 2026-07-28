/*
 * test_gps.c — GPS NMEA 純解析邏輯單元測試（P1 GPS 加固，純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 驗證 gps_parse.h：
 *   [1] 組句狀態機：'$' 對齊、CRLF 變體、'$' 前雜訊（UBX 二進位）不入句
 *   [2] 截斷旗標修復（本次 P1 的 bug fix）：超長句作廢且剩餘位元組不污染下一句
 *   [3] checksum：黃金句、壞 checksum、無 '*'、小寫 hex
 *   [4] 經緯度換算：ddmm.mmmm → deg×1e6 黃金值、半球正負號、格式錯誤回 0
 *   [5] GGA 解析：全欄位黃金值、無 fix 不更新座標
 *   [6] RMC 解析：地速 knots→m/s、狀態 V 不更新座標
 *   [7] gps_parse_sentence 分派與 ok/err 計數
 *   [8] 端到端：位元組流（雜訊+截斷+正常句）→ 最終資料正確
 *
 * 黃金句 checksum 與經緯度期望值由 Python 預先計算（見 commit 訊息）。
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "gps_parse.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static int feq(float a, float b, float tol) { return fabsf(a - b) <= tol; }

/* 黃金句（checksum 已驗算） */
#define GGA_GOLD  "$GPGGA,123519.00,2256.78901,N,12012.34567,E,1,08,0.9,545.4,M,46.9,M,,*6C"
#define GGA_SW    "$GNGGA,123519.00,2256.78901,S,12012.34567,W,2,12,0.9,545.4,M,46.9,M,,*75"
#define GGA_NOFIX "$GPGGA,123519.00,,,,,0,00,,,M,,M,,*45"
#define RMC_GOLD  "$GPRMC,123519.00,A,2256.78901,N,12012.34567,E,022.4,084.4,230394,003.1,W*41"
#define RMC_VOID  "$GPRMC,123519.00,V,,,,,,,230394,,*1D"
#define VTG_GOLD  "$GPVTG,084.4,T,077.8,M,022.4,N,041.5,K*4A"

#define LAT_GOLD  22946484   /* 22 + 56.78901/60 → ×1e6 四捨五入 */
#define LON_GOLD  120205761  /* 120 + 12.34567/60 → ×1e6 四捨五入 */

/* 把字串逐位元組餵入組句機，回傳完成的句數，最後一句留在 last（須夠大）。 */
static int feed_str(GpsLineAsm_t *a, const char *s, char *last)
{
    int n = 0;
    for (const char *p = s; *p; p++) {
        if (gps_line_feed(a, (uint8_t)*p)) {
            n++;
            if (last) memcpy(last, a->buf, (size_t)a->len + 1U);
        }
    }
    return n;
}

static void test_line_asm(void)
{
    printf("[1] 組句狀態機\n");
    GpsLineAsm_t a;
    char line[GPS_PARSE_LINE_MAX];

    gps_line_asm_init(&a);
    check("正常句 \\r\\n 結尾 → 恰一句",
          feed_str(&a, GGA_GOLD "\r\n", line) == 1);
    check("句內容含 '$' 開頭、不含 CRLF", line[0] == '$' && strcmp(line, GGA_GOLD) == 0);

    gps_line_asm_init(&a);
    check("單 \\n 結尾也成句", feed_str(&a, RMC_GOLD "\n", line) == 1 && strcmp(line, RMC_GOLD) == 0);

    gps_line_asm_init(&a);
    check("\\r\\n 之後的多餘 \\n 不產生空句", feed_str(&a, "$GPXXX*00\r\n\n\r\n", line) == 1);

    /* '$' 之前的雜訊（開機亂碼 / UBX 二進位 ACK，內含 \r \n 值的位元組）一律不入句 */
    gps_line_asm_init(&a);
    static const uint8_t ubx_ack[] = { 0xB5, 0x62, 0x05, 0x01, 0x02, 0x00,
                                       0x06, 0x00, 0x0D, 0x0A, 0x0D, 0x3A };  /* 含 \r\n 位元組值 */
    int got = 0;
    for (size_t i = 0; i < sizeof(ubx_ack); i++) got += gps_line_feed(&a, ubx_ack[i]);
    check("UBX 二進位（含 CR/LF 位元組）不產生句", got == 0);
    check("UBX 之後正常句不受影響",
          feed_str(&a, GGA_GOLD "\r\n", line) == 1 && strcmp(line, GGA_GOLD) == 0);

    /* 句中遺漏 CRLF，下一個 '$' 重新對齊 */
    gps_line_asm_init(&a);
    check("'$' 中途重對齊：前段丟棄、後句完整",
          feed_str(&a, "$GPGGA,123,brok" RMC_GOLD "\r\n", line) == 1 && strcmp(line, RMC_GOLD) == 0);

    /* 空句 "$\r\n" 不成句 */
    gps_line_asm_init(&a);
    check("空句 $\\r\\n 不成句", feed_str(&a, "$\r\n", line) == 0);
}

static void test_truncation(void)
{
    printf("[2] 截斷旗標（P1 修復：超長句作廢且不污染下一句）\n");
    GpsLineAsm_t a;
    char line[GPS_PARSE_LINE_MAX];
    char longsent[160];

    /* 組一句 150 字元的超長「句」：$ + 149 個 'A' */
    longsent[0] = '$';
    memset(longsent + 1, 'A', 149);
    longsent[150] = '\0';

    gps_line_asm_init(&a);
    check("超長句溢位 → 不成句", feed_str(&a, longsent, line) == 0);
    check("truncated 計數 = 1", a.truncated == 1);
    check("discard 旗標已置", a.discard == 1);

    /* 關鍵回歸：CRLF 收尾後接正常句，必須完整無污染 */
    check("截斷句 CRLF 不產生句", feed_str(&a, "\r\n", line) == 0);
    check("截斷後下一句完整無污染",
          feed_str(&a, GGA_GOLD "\r\n", line) == 1 && strcmp(line, GGA_GOLD) == 0);

    /* 舊版 bug 路徑：溢位後「剩餘位元組」直接遇 CRLF —— 不得拼出垃圾句 */
    gps_line_asm_init(&a);
    char tail_garbage[200];
    snprintf(tail_garbage, sizeof(tail_garbage), "%sGARBAGE,TAIL\r\n", longsent);
    check("溢位後殘尾 + CRLF 不成句", feed_str(&a, tail_garbage, line) == 0);

    /* 邊界：恰 95 字元（buf 上限內）要能成句 */
    gps_line_asm_init(&a);
    char fit[GPS_PARSE_LINE_MAX + 4];
    fit[0] = '$';
    memset(fit + 1, 'B', GPS_PARSE_LINE_MAX - 2);   /* 總長 95 = MAX-1 */
    fit[GPS_PARSE_LINE_MAX - 1] = '\0';
    strcat(fit, "\r\n");
    check("恰 MAX-1 字元成句不截斷",
          feed_str(&a, fit, line) == 1 && a.truncated == 0 && (int)strlen(line) == GPS_PARSE_LINE_MAX - 1);
}

static void test_checksum(void)
{
    printf("[3] NMEA checksum\n");
    check("GGA 黃金句通過", nmea_checksum_ok(GGA_GOLD) == 1);
    check("RMC 黃金句通過", nmea_checksum_ok(RMC_GOLD) == 1);
    check("壞 checksum 拒收", nmea_checksum_ok("$GPGGA,123519.00,,,,,0,00,,,M,,M,,*46") == 0);
    check("內容被竄改拒收", nmea_checksum_ok("$GPGGA,123619.00,,,,,0,00,,,M,,M,,*45") == 0);
    check("無 '*' 欄位拒收", nmea_checksum_ok("$GPGGA,123519.00,,,,,0,00,,,M,,M,,") == 0);
    check("小寫 hex 通過", nmea_checksum_ok("$GPGGA,123519.00,2256.78901,N,12012.34567,E,1,08,0.9,545.4,M,46.9,M,,*6c") == 1);
    check("非 hex 字元拒收", nmea_checksum_ok("$GPGGA,1*GZ") == 0);
    check("非 '$' 開頭拒收", nmea_checksum_ok("GPGGA,1*00") == 0);
}

static void test_latlon(void)
{
    printf("[4] 經緯度換算（ddmm.mmmm → deg×1e6）\n");
    check("緯度 N 黃金值", nmea_latlon_1e6("2256.78901", 'N') == LAT_GOLD);
    check("緯度 S 取負", nmea_latlon_1e6("2256.78901", 'S') == -LAT_GOLD);
    check("經度 E 黃金值", nmea_latlon_1e6("12012.34567", 'E') == LON_GOLD);
    check("經度 W 取負", nmea_latlon_1e6("12012.34567", 'W') == -LON_GOLD);
    check("無小數點 → 0", nmea_latlon_1e6("225678901", 'N') == 0);
    check("整數部過短 → 0", nmea_latlon_1e6("12.34", 'N') == 0);
    check("混入非數字 → 0", nmea_latlon_1e6("22a6.78901", 'N') == 0);
    check("赤道 0000.00000 → 0", nmea_latlon_1e6("0000.00000", 'N') == 0);
}

static void test_gga(void)
{
    printf("[5] GGA 解析\n");
    GPS_Data_t d;
    memset(&d, 0, sizeof(d));

    gps_parse_gga(&d, GGA_GOLD, 5000);
    check("fix_quality = 1", d.fix_quality == 1);
    check("satellites = 8", d.satellites == 8);
    check("lat 黃金值", d.lat_1e6 == LAT_GOLD);
    check("lon 黃金值", d.lon_1e6 == LON_GOLD);
    check("altitude 545.4m", feq(d.altitude_m, 545.4f, 0.01f));
    check("geoid_sep 46.9m", feq(d.geoid_sep_m, 46.9f, 0.01f));
    check("utc = 123519", d.utc_hhmmss == 123519);
    check("fix_valid = 1", d.fix_valid == 1);
    check("last_fix_tick = now", d.last_fix_tick == 5000);

    /* GN talker + 南/西半球 */
    memset(&d, 0, sizeof(d));
    gps_parse_gga(&d, GGA_SW, 100);
    check("GN talker 可解析（忽略前綴）", d.fix_quality == 2 && d.satellites == 12);
    check("S/W 半球為負", d.lat_1e6 == -LAT_GOLD && d.lon_1e6 == -LON_GOLD);

    /* 無 fix：座標不得被空欄清掉 */
    memset(&d, 0, sizeof(d));
    d.lat_1e6 = LAT_GOLD; d.lon_1e6 = LON_GOLD;
    gps_parse_gga(&d, GGA_NOFIX, 200);
    check("q=0 → fix_valid=0", d.fix_valid == 0);
    check("空欄不覆蓋既有座標", d.lat_1e6 == LAT_GOLD && d.lon_1e6 == LON_GOLD);
}

static void test_rmc(void)
{
    printf("[6] RMC 解析\n");
    GPS_Data_t d;
    memset(&d, 0, sizeof(d));

    gps_parse_rmc(&d, RMC_GOLD, 7000);
    check("狀態 A → fix_valid=1", d.fix_valid == 1);
    check("lat/lon 黃金值", d.lat_1e6 == LAT_GOLD && d.lon_1e6 == LON_GOLD);
    check("地速 22.4kn → 11.52m/s", feq(d.speed_mps, 11.5235f, 0.01f));
    check("航向 84.4°", feq(d.course_deg, 84.4f, 0.01f));
    check("last_fix_tick = now", d.last_fix_tick == 7000);

    /* 狀態 V：不更新座標/速度 */
    GPS_Data_t v;
    memset(&v, 0, sizeof(v));
    v.lat_1e6 = 111; v.speed_mps = 9.9f;
    gps_parse_rmc(&v, RMC_VOID, 8000);
    check("狀態 V → fix_valid=0", v.fix_valid == 0);
    check("V 不覆蓋座標與速度", v.lat_1e6 == 111 && feq(v.speed_mps, 9.9f, 1e-6f));
    check("V 仍更新 UTC", v.utc_hhmmss == 123519);
}

static void test_dispatch(void)
{
    printf("[7] gps_parse_sentence 分派與計數\n");
    GPS_Data_t d;
    memset(&d, 0, sizeof(d));

    check("GGA → GPS_SENT_GGA", gps_parse_sentence(&d, GGA_GOLD, 1) == GPS_SENT_GGA);
    check("RMC → GPS_SENT_RMC", gps_parse_sentence(&d, RMC_GOLD, 2) == GPS_SENT_RMC);
    check("ok 計數 = 2", d.sentences_ok == 2 && d.sentences_err == 0);

    check("VTG（checksum 過）→ SKIP", gps_parse_sentence(&d, VTG_GOLD, 3) == GPS_SENT_SKIP);
    check("SKIP 不動計數", d.sentences_ok == 2 && d.sentences_err == 0);

    check("壞 checksum → BAD", gps_parse_sentence(&d, "$GPGGA,bad*00", 4) == GPS_SENT_BAD);
    check("err 計數 = 1", d.sentences_err == 1);
}

static void test_end_to_end(void)
{
    printf("[8] 端到端：位元組流 → 資料\n");
    GpsLineAsm_t a;
    GPS_Data_t d;
    char line[GPS_PARSE_LINE_MAX];
    gps_line_asm_init(&a);
    memset(&d, 0, sizeof(d));

    /* 模擬真實流：UBX 殘渣 + 截斷長句 + GGA + 雜訊 + RMC */
    char stream[512];
    char longsent[140];
    longsent[0] = '$';
    memset(longsent + 1, 'X', 130);
    longsent[131] = '\0';
    snprintf(stream, sizeof(stream), "\xB5\x62\x05\x01%s\r\n%s\r\nnoise%s\r\n",
             longsent, GGA_GOLD, RMC_GOLD);

    int parsed = 0;
    uint32_t now = 1000;
    for (const char *p = stream; *p; p++) {
        if (gps_line_feed(&a, (uint8_t)*p)) {
            memcpy(line, a.buf, (size_t)a.len + 1U);
            gps_parse_sentence(&d, line, now);
            parsed++;
        }
    }
    check("僅 2 句完整成句（截斷句與雜訊被擋）", parsed == 2);
    check("ok=2 err=0", d.sentences_ok == 2 && d.sentences_err == 0);
    check("truncated = 1", a.truncated == 1);
    check("最終座標正確", d.lat_1e6 == LAT_GOLD && d.lon_1e6 == LON_GOLD);
    check("最終地速正確", feq(d.speed_mps, 11.5235f, 0.01f));
    check("fix_valid = 1", d.fix_valid == 1);
}

static void test_ubx_pvt(void)
{
    printf("[9] UBX-NAV-PVT 二進位解包與解析\n");
    GpsUbxAsm_t asm_ubx;
    GPS_Data_t d;
    gps_ubx_asm_init(&asm_ubx);
    memset(&d, 0, sizeof(d));

    UbxNavPvt_t raw_pvt;
    memset(&raw_pvt, 0, sizeof(raw_pvt));
    raw_pvt.fixType = 3;         /* 3D Fix */
    raw_pvt.flags = 0x01;        /* gnssFixOK */
    raw_pvt.numSV = 14;
    raw_pvt.lat = 229464840;     /* 22.946484 deg (*1e7) */
    raw_pvt.lon = 1202057610;    /* 120.205761 deg (*1e7) */
    raw_pvt.height = 592300;     /* mm */
    raw_pvt.hMSL = 545400;       /* 545.4 m in mm */
    raw_pvt.gSpeed = 11520;      /* 11.52 m/s in mm/s */
    raw_pvt.headMot = 8440000;   /* 84.4 deg in 1e-5 deg */
    raw_pvt.hour = 12;
    raw_pvt.min = 35;
    raw_pvt.sec = 19;

    uint8_t pkt[100];
    pkt[0] = 0xB5;
    pkt[1] = 0x62;
    pkt[2] = 0x01; /* Class NAV */
    pkt[3] = 0x07; /* ID PVT */
    pkt[4] = 0x5C; /* Len LSB (92) */
    pkt[5] = 0x00; /* Len MSB */
    memcpy(&pkt[6], &raw_pvt, sizeof(raw_pvt));

    /* 計算 Checksum Over Class, ID, Len, Payload */
    uint8_t cka = 0, ckb = 0;
    for (size_t i = 2; i < 98; i++) {
        cka = (uint8_t)(cka + pkt[i]);
        ckb = (uint8_t)(ckb + cka);
    }
    pkt[98] = cka;
    pkt[99] = ckb;

    /* 逐位元組餵入狀態機 */
    int ok = 0;
    for (size_t i = 0; i < sizeof(pkt); i++) {
        if (gps_ubx_feed(&asm_ubx, pkt[i])) {
            ok = 1;
            UbxNavPvt_t *p = (UbxNavPvt_t*)asm_ubx.payload_buf;
            gps_parse_ubx_pvt(&d, p, 12345);
        }
    }

    check("UBX-NAV-PVT 二進位解包成功", ok == 1);
    check("fix_valid = 1", d.fix_valid == 1);
    check("fix_quality = 3 (3D)", d.fix_quality == 3);
    check("satellites = 14", d.satellites == 14);
    check("lat_1e6 轉換正確", d.lat_1e6 == 22946484);
    check("lon_1e6 轉換正確", d.lon_1e6 == 120205761);
    check("altitude_m = 545.4m", feq(d.altitude_m, 545.4f, 0.01f));
    check("geoid_sep_m = 46.9m", feq(d.geoid_sep_m, 46.9f, 0.01f));
    check("speed_mps = 11.52m/s", feq(d.speed_mps, 11.52f, 0.01f));
    check("course_deg = 84.4°", feq(d.course_deg, 84.4f, 0.01f));
    check("utc_hhmmss = 123519", d.utc_hhmmss == 123519);
    check("last_fix_tick = 12345", d.last_fix_tick == 12345);
}

int main(void)
{
    printf("=== test_gps：GPS UBX 二進位與 NMEA 解析 ===\n");
    test_line_asm();
    test_truncation();
    test_checksum();
    test_latlon();
    test_gga();
    test_rmc();
    test_dispatch();
    test_end_to_end();
    test_ubx_pvt();
    printf("----------------------------------------\n");
    if (g_fail == 0) printf("ALL PASS：%d/%d 通過\n", g_total, g_total);
    else             printf("FAILED：%d/%d 失敗\n", g_fail, g_total);
    return g_fail ? 1 : 0;
}

