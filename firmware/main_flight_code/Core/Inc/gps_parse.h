/*
 * gps_parse.h — NMEA-0183 純解析邏輯（P1，header-only，host 可測）
 * ===========================================================================
 * 自 gps.c 抽出的純字串處理層：組句狀態機、checksum、欄位切割、經緯度換算、
 * GGA/RMC 句型解析。不依賴 HAL / RTOS（僅 stdint + string），由
 * tests/test_gps.c 驗證 —— 比照 sensor_axis.h / fsm.h 的 host 共測模式。
 *
 * gps.c 保留不純的部分：USART6 DMA 環形差分、ISR↔task 的 ready-buffer
 * 交接、UBX 鮑率協商、RATE_TICK_GPS()。
 *
 * 本次抽離同時修復截斷污染 bug（改進計劃.md P1 GPS 加固項）：
 *   舊版 GPS_FeedByte 溢位時僅清長度重來，超長句的「剩餘位元組」會被當成
 *   新句子累積，遇 CRLF 升級為待解析句；且任何不以 '$' 開頭的雜訊（開機
 *   亂碼、GPS_Init 後的 UBX 二進位 ACK）也會被組句，徒增 sentences_err
 *   並有機率拼出貌似合法的部分句。
 *   新版：未見 '$' 一律不累積；溢位置 discard 旗標丟棄至下一個 '$'。
 */
#ifndef GPS_PARSE_H
#define GPS_PARSE_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NMEA 單句最長 82 字元（含 $ 與 CRLF）。緩衝留 96 充足餘量。 */
#define GPS_PARSE_LINE_MAX  96U

#pragma pack(push, 1)
/* UBX-NAV-PVT (Class 0x01, ID 0x07) Payload 92 位元組結構 */
typedef struct {
    uint32_t iTOW;       /* GPS time of week (ms) */
    uint16_t year;       /* Year (UTC) */
    uint8_t  month;      /* Month (1..12) */
    uint8_t  day;        /* Day (1..31) */
    uint8_t  hour;       /* Hour (0..23) */
    uint8_t  min;        /* Min (0..59) */
    uint8_t  sec;        /* Sec (0..60) */
    uint8_t  valid;      /* Validity flags */
    uint32_t tAcc;       /* Time accuracy estimate (ns) */
    int32_t  nano;       /* Fraction of second (ns) */
    uint8_t  fixType;    /* GNSSfix Type: 0=No fix, 1=DR, 2=2D, 3=3D, 4=GNSS+DR, 5=Time */
    uint8_t  flags;      /* Fix status flags (bit 0 = gnssFixOK) */
    uint8_t  flags2;     /* Additional flags */
    uint8_t  numSV;      /* Number of satellites used */
    int32_t  lon;        /* Longitude (deg * 1e-7) */
    int32_t  lat;        /* Latitude (deg * 1e-7) */
    int32_t  height;     /* Height above ellipsoid (mm) */
    int32_t  hMSL;       /* Height above MSL (mm) */
    uint32_t hAcc;       /* Horizontal accuracy (mm) */
    uint32_t vAcc;       /* Vertical accuracy (mm) */
    int32_t  velN;       /* NED north velocity (mm/s) */
    int32_t  velE;       /* NED east velocity (mm/s) */
    int32_t  velD;       /* NED down velocity (mm/s) */
    int32_t  gSpeed;     /* Ground Speed (2D) (mm/s) */
    int32_t  headMot;    /* Heading of motion (2D) (deg * 1e-5) */
    uint32_t sAcc;       /* Speed accuracy estimate (mm/s) */
    uint32_t headAcc;    /* Heading accuracy estimate (deg * 1e-5) */
    uint16_t pDOP;       /* Position DOP (* 0.01) */
    uint16_t flags3;     /* Additional flags */
    uint8_t  reserved1[4];
    int32_t  headVeh;    /* Heading of vehicle (2D) */
    int16_t  magDec;     /* Magnetic declination */
    uint16_t magAcc;     /* Magnetic declination accuracy */
} UbxNavPvt_t;
#pragma pack(pop)

/* --- 解析後的 GPS 資料（自 gps.h 移入；欄位皆純資料，無 HAL 依賴） --- */
typedef struct {
    uint8_t  fix_valid;        /* 1 = RMC 狀態 'A' 或 GGA fix>0（有有效定位） */
    uint8_t  fix_quality;      /* GGA fix quality：0=invalid,1=GPS,2=DGPS...   */
    uint8_t  satellites;       /* GGA 使用衛星數                               */
    int32_t  lat_1e6;          /* 緯度 ×1e6 (deg)，+北 / −南                    */
    int32_t  lon_1e6;          /* 經度 ×1e6 (deg)，+東 / −西                    */
    uint32_t hacc_mm;          /* UBX-NAV-PVT hAcc：接收器自估水平精度 (mm)；
                                 * NMEA 路徑（無 UBX PVT）留 0，呼叫端須視 0 為
                                 * 「無估計」並自行 fallback。 */
    float    altitude_m;       /* GGA 海拔高度 (m, MSL)                         */
    float    geoid_sep_m;      /* GGA 大地水準面分離 (m)                        */
    float    speed_mps;        /* RMC 地速 (m/s，由 knots 換算)                 */
    float    course_deg;       /* RMC 航向 (deg true)                          */
    uint32_t utc_hhmmss;       /* UTC 時間 hhmmss（整數部分）                    */
    uint32_t last_fix_tick;    /* 最近一次有效定位的時刻 (ms，由呼叫端提供)      */
    uint32_t sentences_ok;     /* 通過 checksum 並成功解析的句數（診斷）         */
    uint32_t sentences_err;    /* checksum 失敗或格式錯誤的句數（診斷）          */
} GPS_Data_t;

/* --- 組句狀態機（位元組流 → 完整 NMEA 句） --- */
typedef struct {
    char     buf[GPS_PARSE_LINE_MAX];  /* 組裝中（句完成時就地 NUL 化供取用） */
    uint16_t len;
    uint8_t  in_sentence;   /* 1 = 已見句首 '$'，正在累積 */
    uint8_t  discard;       /* 1 = 句長溢位，丟棄至下一個 '$'（截斷旗標） */
    uint32_t truncated;     /* 診斷：截斷髒句次數 */
} GpsLineAsm_t;

static inline void gps_line_asm_init(GpsLineAsm_t *a)
{
    memset(a, 0, sizeof(*a));
}

/* 餵入一個位元組。回傳 1 = a->buf 內有一句完整句子（NUL-terminated，
 * 含句首 '$'，不含 CRLF），內容在下一個 '$' 餵入前有效。
 * 規則：
 *   '$'      → 無條件重新開始累積（句中遺漏 CRLF 也能重新對齊），清 discard
 *   CR / LF  → in_sentence 且未 discard 且 len>0 → 句完成；一律結束累積
 *   其他     → 未見 '$' 則忽略（開機雜訊 / UBX 二進位不入句）；
 *              溢位 → 置 discard、計 truncated，本句作廢
 */
static inline uint8_t gps_line_feed(GpsLineAsm_t *a, uint8_t b)
{
    if (b == '$') {
        a->len = 0;
        a->buf[a->len++] = '$';
        a->in_sentence = 1U;
        a->discard = 0U;
        return 0U;
    }

    if (!a->in_sentence) {
        return 0U;                       /* '$' 之前的雜訊一律不收 */
    }

    if (b == '\r' || b == '\n') {
        uint8_t done = (!a->discard && a->len > 1U) ? 1U : 0U;
        if (done) {
            a->buf[a->len] = '\0';       /* len 保留 = 句長，供呼叫端取用 */
        } else {
            a->len = 0U;
        }
        a->in_sentence = 0U;             /* 句已結束，等下一個 '$' */
        a->discard = 0U;
        return done;
    }

    if (a->discard) {
        return 0U;
    }

    if (a->len < (GPS_PARSE_LINE_MAX - 1U)) {
        a->buf[a->len++] = (char)b;
    } else {
        a->discard = 1U;                 /* 溢位：本句作廢，剩餘位元組不得污染下一句 */
        a->truncated++;
    }
    return 0U;
}

/* ------------------------------------------------------------------ */
/* NMEA 解析小工具                                                     */
/* ------------------------------------------------------------------ */

/* 驗證 "$....*HH" 的 XOR checksum。回傳 1 = 通過。 */
static inline uint8_t nmea_checksum_ok(const char *s)
{
    if (s[0] != '$') return 0;
    uint8_t sum = 0;
    const char *p = s + 1;
    while (*p && *p != '*') {
        sum ^= (uint8_t)(*p);
        p++;
    }
    if (*p != '*') return 0;                 /* 無 checksum 欄位 */
    uint8_t hi = 0, lo = 0;
    char c1 = p[1], c2 = p[2];
    if (c1 >= '0' && c1 <= '9') hi = c1 - '0';
    else if (c1 >= 'A' && c1 <= 'F') hi = c1 - 'A' + 10;
    else if (c1 >= 'a' && c1 <= 'f') hi = c1 - 'a' + 10;
    else return 0;
    if (c2 >= '0' && c2 <= '9') lo = c2 - '0';
    else if (c2 >= 'A' && c2 <= 'F') lo = c2 - 'A' + 10;
    else if (c2 >= 'a' && c2 <= 'f') lo = c2 - 'a' + 10;
    else return 0;
    return ((hi << 4) | lo) == sum;
}

/* 取第 n 個逗號分隔欄位（n=0 為句型 token，如 "$GPGGA"）。
 * 複製到 out（最多 cap-1 字元 + NUL）。回傳欄位長度（0 = 空欄或不存在）。 */
static inline int nmea_field(const char *s, int n, char *out, int cap)
{
    int field = 0;
    const char *p = s;
    while (field < n && *p) {
        if (*p == ',') field++;
        p++;
    }
    if (field != n) { out[0] = '\0'; return 0; }
    int len = 0;
    while (*p && *p != ',' && *p != '*' && len < cap - 1) {
        out[len++] = *p++;
    }
    out[len] = '\0';
    return len;
}

/* 解析 NMEA 經緯度 "ddmm.mmmm" / "dddmm.mmmm" → 度 ×1e6 (int32)，依半球套用正負號。
 * 手動解析（不依賴 strtod），用 double 做最終縮放以保留 ~8 位有效數字。 */
static inline int32_t nmea_latlon_1e6(const char *f, char hemi)
{
    const char *dot = strchr(f, '.');
    if (!dot) return 0;
    int intlen = (int)(dot - f);
    if (intlen < 3) return 0;               /* 至少 "mm." 之前要有度+分 */

    long deg = 0;
    int i = 0;
    for (; i < intlen - 2; i++) {
        if (f[i] < '0' || f[i] > '9') return 0;
        deg = deg * 10 + (f[i] - '0');
    }
    long min_int = 0;
    for (; i < intlen; i++) {
        if (f[i] < '0' || f[i] > '9') return 0;
        min_int = min_int * 10 + (f[i] - '0');
    }
    long frac = 0, fdiv = 1;
    const char *p = dot + 1;
    while (*p >= '0' && *p <= '9' && fdiv < 1000000L) {
        frac = frac * 10 + (*p - '0');
        fdiv *= 10;
        p++;
    }
    double minutes = (double)min_int + (double)frac / (double)fdiv;
    double degrees = (double)deg + minutes / 60.0;
    int32_t out = (int32_t)(degrees * 1e6 + 0.5);
    if (hemi == 'S' || hemi == 'W') out = -out;
    return out;
}

/* 極簡 atof：可選正負號 + 整數 + 小數，無指數。空字串回傳 0。 */
static inline float nmea_atof(const char *s)
{
    float sign = 1.0f;
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }
    float val = 0.0f;
    while (*s >= '0' && *s <= '9') { val = val * 10.0f + (float)(*s - '0'); s++; }
    if (*s == '.') {
        s++;
        float scale = 0.1f;
        while (*s >= '0' && *s <= '9') { val += (float)(*s - '0') * scale; scale *= 0.1f; s++; }
    }
    return sign * val;
}

static inline int nmea_atoi(const char *s)
{
    int v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
    return v;
}

/* 比對句型（忽略 talker ID 前綴：GP/GN/GL/GA 等）。type 例如 "GGA"。 */
static inline uint8_t nmea_is_type(const char *s, const char *type)
{
    /* s 形如 "$GPGGA"；句型三碼位於索引 3..5 */
    return (s[0] == '$' && strncmp(s + 3, type, 3) == 0);
}

/* ------------------------------------------------------------------ */
/* 句型解析                                                            */
/* ------------------------------------------------------------------ */

/* $--GGA: 時間,緯度,N/S,經度,E/W,fix品質,衛星數,HDOP,海拔,M,水準面分離,M,... */
static inline void gps_parse_gga(GPS_Data_t *d, const char *s, uint32_t now_ms)
{
    char buf[16], ns[2], ew[2];

    int q = 0;
    if (nmea_field(s, 6, buf, sizeof(buf)) > 0) q = nmea_atoi(buf);
    d->fix_quality = (uint8_t)q;

    if (nmea_field(s, 7, buf, sizeof(buf)) > 0) d->satellites = (uint8_t)nmea_atoi(buf);

    int latlen = nmea_field(s, 2, buf, sizeof(buf));
    nmea_field(s, 3, ns, sizeof(ns));
    if (latlen > 0 && ns[0]) {
        d->lat_1e6 = nmea_latlon_1e6(buf, ns[0]);
    }
    int lonlen = nmea_field(s, 4, buf, sizeof(buf));
    nmea_field(s, 5, ew, sizeof(ew));
    if (lonlen > 0 && ew[0]) {
        d->lon_1e6 = nmea_latlon_1e6(buf, ew[0]);
    }

    if (nmea_field(s, 1, buf, sizeof(buf)) > 0) d->utc_hhmmss = (uint32_t)nmea_atoi(buf);
    if (nmea_field(s, 9, buf, sizeof(buf)) > 0) d->altitude_m = nmea_atof(buf);
    if (nmea_field(s, 11, buf, sizeof(buf)) > 0) d->geoid_sep_m = nmea_atof(buf);

    if (q > 0) {
        d->fix_valid = 1;
        d->last_fix_tick = now_ms;
    } else {
        d->fix_valid = 0;
    }
}

/* $--RMC: 時間,狀態(A/V),緯度,N/S,經度,E/W,地速(knots),航向,日期,... */
static inline void gps_parse_rmc(GPS_Data_t *d, const char *s, uint32_t now_ms)
{
    char buf[16], st[2], ns[2], ew[2];

    nmea_field(s, 2, st, sizeof(st));
    uint8_t valid = (st[0] == 'A');

    if (nmea_field(s, 1, buf, sizeof(buf)) > 0) d->utc_hhmmss = (uint32_t)nmea_atoi(buf);

    if (valid) {
        int latlen = nmea_field(s, 3, buf, sizeof(buf));
        nmea_field(s, 4, ns, sizeof(ns));
        if (latlen > 0 && ns[0]) d->lat_1e6 = nmea_latlon_1e6(buf, ns[0]);

        int lonlen = nmea_field(s, 5, buf, sizeof(buf));
        nmea_field(s, 6, ew, sizeof(ew));
        if (lonlen > 0 && ew[0]) d->lon_1e6 = nmea_latlon_1e6(buf, ew[0]);

        if (nmea_field(s, 7, buf, sizeof(buf)) > 0) d->speed_mps = nmea_atof(buf) * 0.514444f; /* knots→m/s */
        if (nmea_field(s, 8, buf, sizeof(buf)) > 0) d->course_deg = nmea_atof(buf);

        d->fix_valid = 1;
        d->last_fix_tick = now_ms;
    } else {
        d->fix_valid = 0;
    }
}

/* 解析一句完整 NMEA。回傳：
 *   GPS_SENT_BAD  = checksum 失敗（sentences_err++）
 *   GPS_SENT_GGA / GPS_SENT_RMC = 成功解析（sentences_ok++）
 *   GPS_SENT_SKIP = checksum 通過但句型不支援（GSV/GSA/VTG...，不計數） */
#define GPS_SENT_BAD   0
#define GPS_SENT_SKIP  1
#define GPS_SENT_GGA   2
#define GPS_SENT_RMC   3
static inline uint8_t gps_parse_sentence(GPS_Data_t *d, const char *line, uint32_t now_ms)
{
    if (!nmea_checksum_ok(line)) {
        d->sentences_err++;
        return GPS_SENT_BAD;
    }
    if (nmea_is_type(line, "GGA")) {
        gps_parse_gga(d, line, now_ms);
        d->sentences_ok++;
        return GPS_SENT_GGA;
    }
    if (nmea_is_type(line, "RMC")) {
        gps_parse_rmc(d, line, now_ms);
        d->sentences_ok++;
        return GPS_SENT_RMC;
    }
    return GPS_SENT_SKIP;
}

/* ------------------------------------------------------------------ */
/* UBX 二進位解包與解析                                               */
/* ------------------------------------------------------------------ */

typedef enum {
    UBX_STEP_SYNC1 = 0,
    UBX_STEP_SYNC2,
    UBX_STEP_CLASS,
    UBX_STEP_ID,
    UBX_STEP_LEN_LSB,
    UBX_STEP_LEN_MSB,
    UBX_STEP_PAYLOAD,
    UBX_STEP_CK_A,
    UBX_STEP_CK_B
} UbxStep_t;

typedef struct {
    UbxStep_t step;
    uint8_t   msg_class;
    uint8_t   msg_id;
    uint16_t  payload_len;
    uint16_t  payload_idx;
    uint8_t   calc_ck_a;
    uint8_t   calc_ck_b;
    uint8_t   payload_buf[128];
} GpsUbxAsm_t;

static inline void gps_ubx_asm_init(GpsUbxAsm_t *a)
{
    memset(a, 0, sizeof(*a));
    a->step = UBX_STEP_SYNC1;
}

/* 餵入一個位元組至 UBX 二進位解包狀態機。
 * 若收滿一包完整的 UBX-NAV-PVT (Class 0x01, ID 0x07) 且 Checksum 通過，回傳 1。
 * 成功解出的 92 位元組 payload 在 a->payload_buf 中。
 */
static inline uint8_t gps_ubx_feed(GpsUbxAsm_t *a, uint8_t b)
{
    switch (a->step) {
    case UBX_STEP_SYNC1:
        if (b == 0xB5) {
            a->step = UBX_STEP_SYNC2;
        }
        break;

    case UBX_STEP_SYNC2:
        if (b == 0x62) {
            a->step = UBX_STEP_CLASS;
        } else if (b != 0xB5) {
            a->step = UBX_STEP_SYNC1;
        }
        break;

    case UBX_STEP_CLASS:
        a->msg_class = b;
        a->calc_ck_a = b;
        a->calc_ck_b = b;
        a->step = UBX_STEP_ID;
        break;

    case UBX_STEP_ID:
        a->msg_id = b;
        a->calc_ck_a = (uint8_t)(a->calc_ck_a + b);
        a->calc_ck_b = (uint8_t)(a->calc_ck_b + a->calc_ck_a);
        a->step = UBX_STEP_LEN_LSB;
        break;

    case UBX_STEP_LEN_LSB:
        a->payload_len = b;
        a->calc_ck_a = (uint8_t)(a->calc_ck_a + b);
        a->calc_ck_b = (uint8_t)(a->calc_ck_b + a->calc_ck_a);
        a->step = UBX_STEP_LEN_MSB;
        break;

    case UBX_STEP_LEN_MSB:
        a->payload_len |= ((uint16_t)b << 8);
        a->calc_ck_a = (uint8_t)(a->calc_ck_a + b);
        a->calc_ck_b = (uint8_t)(a->calc_ck_b + a->calc_ck_a);
        if (a->payload_len > sizeof(a->payload_buf)) {
            a->step = UBX_STEP_SYNC1;
        } else if (a->payload_len == 0) {
            a->step = UBX_STEP_CK_A;
        } else {
            a->payload_idx = 0;
            a->step = UBX_STEP_PAYLOAD;
        }
        break;

    case UBX_STEP_PAYLOAD:
        a->payload_buf[a->payload_idx++] = b;
        a->calc_ck_a = (uint8_t)(a->calc_ck_a + b);
        a->calc_ck_b = (uint8_t)(a->calc_ck_b + a->calc_ck_a);
        if (a->payload_idx >= a->payload_len) {
            a->step = UBX_STEP_CK_A;
        }
        break;

    case UBX_STEP_CK_A:
        if (b == a->calc_ck_a) {
            a->step = UBX_STEP_CK_B;
        } else {
            a->step = UBX_STEP_SYNC1;
        }
        break;

    case UBX_STEP_CK_B:
        a->step = UBX_STEP_SYNC1;
        if (b == a->calc_ck_b) {
            if (a->msg_class == 0x01 && a->msg_id == 0x07 && a->payload_len == sizeof(UbxNavPvt_t)) {
                return 1U;
            }
        }
        break;

    default:
        a->step = UBX_STEP_SYNC1;
        break;
    }
    return 0U;
}

/* 將解包後的 UBX-NAV-PVT payload 轉換並更新至 GPS_Data_t */
static inline void gps_parse_ubx_pvt(GPS_Data_t *d, const UbxNavPvt_t *pvt, uint32_t now_ms)
{
    uint8_t valid = ((pvt->fixType >= 2U) && (pvt->flags & 0x01U)) ? 1U : 0U;

    d->fix_quality = pvt->fixType;
    d->satellites = pvt->numSV;
    d->hacc_mm = pvt->hAcc;
    d->lat_1e6 = pvt->lat / 10;
    d->lon_1e6 = pvt->lon / 10;
    d->altitude_m = (float)pvt->hMSL / 1000.0f;
    d->geoid_sep_m = (float)(pvt->height - pvt->hMSL) / 1000.0f;
    d->speed_mps = (float)pvt->gSpeed / 1000.0f;
    d->course_deg = (float)pvt->headMot / 100000.0f;
    d->utc_hhmmss = (uint32_t)pvt->hour * 10000U + (uint32_t)pvt->min * 100U + (uint32_t)pvt->sec;

    if (valid) {
        d->fix_valid = 1U;
        d->last_fix_tick = now_ms;
    } else {
        d->fix_valid = 0U;
    }
    d->sentences_ok++;
}

#ifdef __cplusplus
}
#endif

#endif /* GPS_PARSE_H */

