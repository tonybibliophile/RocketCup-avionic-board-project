/*
 * lora_calc.h — LoRa 參數換算 + 鏈路統計（純邏輯，header-only，host 可測）
 * ===========================================================================
 * 自 lora_e80.c / gs_lora_test.c 抽出的純計算，不依賴 HAL / SPI（僅 stdint /
 * string）。由 tests/test_lora_calc.c 驗證。driver、命令解析器、host 測試共用
 * 同一份，避免三處各寫一份 BW 表 / LDRO 規則而漂移。
 *
 * 涵蓋：
 *   - LoRa 頻寬 index ↔ kHz（LR1121 與 SX126x 共用同一組 index 值）
 *   - 符號時間 / LDRO 判定（低資料率最佳化，符號 ≥16ms 時須開）
 *   - LR1121 SetRfFrequency 參數位元組（直接以 Hz 大端，非 SX126x 的 Frf 公式）
 *   - E22-400 頻率(MHz) ↔ 通道(CH)
 *   - LoRa 空中時間 (time-on-air) 估算（顯式表頭 + CRC on）
 *   - 雙鏈路接收統計（封包數 / CRC 錯誤 / RSSI、SNR 之 last/min/max/avg / 封包率）
 */
#ifndef LORA_CALC_H
#define LORA_CALC_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  LoRa 頻寬 index（LR1121 / SX126x 共用值）
 * ============================================================ */
/** @brief 頻寬 index → kHz。未知值回傳 0（呼叫端可據此判錯）。 */
static inline uint32_t lora_bw_to_khz(uint8_t bw_idx)
{
    switch (bw_idx) {
        case 0x00: return 8;     /* 7.81 kHz（四捨五入顯示用） */
        case 0x08: return 10;    /* 10.42 */
        case 0x01: return 16;    /* 15.63 */
        case 0x09: return 21;    /* 20.83 */
        case 0x02: return 31;    /* 31.25 */
        case 0x0A: return 42;    /* 41.67 */
        case 0x03: return 63;    /* 62.50 */
        case 0x04: return 125;
        case 0x05: return 250;
        case 0x06: return 500;
        default:   return 0;
    }
}

/** @brief 該 bw_idx 是否為合法 LoRa 頻寬。 */
static inline int lora_bw_valid(uint8_t bw_idx)
{
    return lora_bw_to_khz(bw_idx) != 0;
}

/** @brief LoRa 符號時間（微秒）= 2^SF / BW。BW 無效回傳 0。 */
static inline uint32_t lora_symbol_time_us(uint8_t sf, uint8_t bw_idx)
{
    uint32_t bw_khz = lora_bw_to_khz(bw_idx);
    if (bw_khz == 0 || sf < 5 || sf > 12) return 0;
    return ((uint32_t)1u << sf) * 1000u / bw_khz;
}

/** @brief 是否須開 LDRO（低資料率最佳化）：符號時間 ≥ 16ms 時須開。
 *  例：BW125 下 SF11/SF12 須開（16.384ms / 32.768ms）。 */
static inline uint8_t lora_ldro_required(uint8_t sf, uint8_t bw_idx)
{
    uint32_t ts = lora_symbol_time_us(sf, bw_idx);
    return (ts != 0 && ts >= 16000u) ? 1u : 0u;
}

/* ============================================================
 *  頻率參數
 * ============================================================ */
/** @brief LR1121 SetRfFrequency 參數：頻率(Hz)直接以大端 4 bytes 帶入
 *  （與 SX126x 的 Frf=f·2^25/32e6 公式不同；LR1121 直接吃 Hz）。 */
static inline void lr1121_freq_to_bytes(uint32_t hz, uint8_t out[4])
{
    out[0] = (uint8_t)(hz >> 24);
    out[1] = (uint8_t)(hz >> 16);
    out[2] = (uint8_t)(hz >> 8);
    out[3] = (uint8_t)(hz);
}

/** @brief LR1121 CalibImage(0x0111) 參數位元組（頻段影像校準）。
 *  以中心頻率 ±10MHz 取一涵蓋頻段，換算成 4MHz 步階的 [freq1(floor), freq2(ceil)]。
 *  換頻段後（尤其自 LR1121 開機預設 sub-G 跳到 920MHz）須校準一次，否則 RX 影像抑制
 *  未最佳化、靈敏度下降。與 TCXO 供電方式無關，E80 自供電振盪器亦需此步。 */
static inline void lr1121_calib_image_bytes(uint32_t center_hz, uint8_t out[2])
{
    uint32_t mhz = center_hz / 1000000u;
    uint32_t lo  = (mhz > 10u) ? (mhz - 10u) : 0u;
    uint32_t hi  = mhz + 10u;
    out[0] = (uint8_t)(lo / 4u);          /* floor：對應 ≤ 下界的頻率 */
    out[1] = (uint8_t)((hi + 3u) / 4u);   /* ceil ：對應 ≥ 上界的頻率 */
}

/** @brief E22-400 頻率(MHz) → 通道 CH（freq = 410 + CH）。
 *  合法回傳 1 並填 *ch（410~493 MHz / CH 0~83）；超範圍回傳 0。 */
static inline int e22_mhz_to_ch(uint32_t mhz, uint8_t *ch)
{
    if (mhz < 410u || mhz > 493u) return 0;
    if (ch) *ch = (uint8_t)(mhz - 410u);
    return 1;
}

/* ============================================================
 *  LoRa 空中時間（time-on-air）估算
 * ============================================================ */
/**
 * @brief 估算一筆 LoRa 封包的空中時間（微秒）。顯式表頭、CRC on、固定 LoRaWAN 公式。
 * @param sf       展頻因子 7~12
 * @param bw_idx   頻寬 index（見上）
 * @param cr       編碼率 1~4（1=4/5 … 4=4/8）
 * @param preamble 前導碼符號數
 * @param payload_len  payload 位元組數
 * @return 空中時間 (us)；參數非法回傳 0。
 * @note  Semtech LoRa 公式（DE=LDRO, IH=0 顯式, CRC=1）。整數運算、無浮點。
 */
static inline uint32_t lora_time_on_air_us(uint8_t sf, uint8_t bw_idx, uint8_t cr,
                                           uint16_t preamble, uint8_t payload_len)
{
    uint32_t ts = lora_symbol_time_us(sf, bw_idx);
    if (ts == 0 || cr < 1 || cr > 4) return 0;

    uint8_t de = lora_ldro_required(sf, bw_idx);

    /* 前導碼時間 = (preamble + 4.25) · Ts；用 ×4 保留 .25 再除回 */
    uint32_t t_pre = (((uint32_t)preamble * 4u + 17u) * ts) / 4u;

    /* payload 符號數（Semtech 公式，ceil 以整數實作） */
    int32_t num = 8 * (int32_t)payload_len - 4 * (int32_t)sf + 28 + 16 /*CRC*/ - 0 /*IH=0 顯式*/;
    int32_t den = 4 * ((int32_t)sf - 2 * (int32_t)de);
    int32_t ceil_term = 0;
    if (num > 0 && den > 0) ceil_term = (num + den - 1) / den;
    int32_t payload_symb = 8 + ceil_term * ((int32_t)cr + 4);
    if (payload_symb < 8) payload_symb = 8;

    return t_pre + (uint32_t)payload_symb * ts;
}

/* ============================================================
 *  鏈路接收統計（純邏輯；時間戳由呼叫端帶入，便於 host 測試）
 * ============================================================ */
typedef struct {
    uint32_t pkt_ok;        /* 有效封包數 */
    uint32_t crc_err;       /* CRC / 表頭錯誤數 */

    int16_t  rssi_last, rssi_min, rssi_max;
    int32_t  rssi_sum;
    uint32_t rssi_cnt;

    int16_t  snr_last, snr_min, snr_max;
    int32_t  snr_sum;
    uint32_t snr_cnt;

    uint32_t first_ms;      /* 第一筆有效封包時間戳 */
    uint32_t last_ms;       /* 最後一筆有效封包時間戳 */
} lora_stats_t;

static inline void lora_stats_reset(lora_stats_t *s)
{
    memset(s, 0, sizeof(*s));
    s->rssi_min = 32767;  s->rssi_max = -32768;
    s->snr_min  = 32767;  s->snr_max  = -32768;
}

/**
 * @brief 更新統計。crc_ok=0 只累加 crc_err；=1 累加封包並（若有效）更新 RSSI/SNR。
 * @param has_rssi/has_snr  0=本鏈路無此量測（如 E22 透傳），不納入統計
 * @param now_ms 呼叫端時間戳（firmware 帶 HAL_GetTick，測試帶模擬值）
 */
static inline void lora_stats_on_packet(lora_stats_t *s, int crc_ok,
                                        int has_rssi, int16_t rssi,
                                        int has_snr,  int16_t snr,
                                        uint32_t now_ms)
{
    if (!crc_ok) { s->crc_err++; return; }

    if (s->pkt_ok == 0) s->first_ms = now_ms;
    s->pkt_ok++;
    s->last_ms = now_ms;

    if (has_rssi) {
        s->rssi_last = rssi; s->rssi_sum += rssi; s->rssi_cnt++;
        if (rssi < s->rssi_min) s->rssi_min = rssi;
        if (rssi > s->rssi_max) s->rssi_max = rssi;
    }
    if (has_snr) {
        s->snr_last = snr; s->snr_sum += snr; s->snr_cnt++;
        if (snr < s->snr_min) s->snr_min = snr;
        if (snr > s->snr_max) s->snr_max = snr;
    }
}

/** @brief 封包率 ×10（pkt/s 的 10 倍，便於印出一位小數）。無資料回傳 0。 */
static inline uint32_t lora_stats_rate_x10(const lora_stats_t *s)
{
    uint32_t el = (s->last_ms > s->first_ms) ? (s->last_ms - s->first_ms) : 0u;
    if (s->pkt_ok == 0 || el == 0) return 0u;
    return s->pkt_ok * 10000u / el;
}

static inline int32_t lora_stats_rssi_avg(const lora_stats_t *s)
{
    return s->rssi_cnt ? (s->rssi_sum / (int32_t)s->rssi_cnt) : 0;
}
static inline int32_t lora_stats_snr_avg(const lora_stats_t *s)
{
    return s->snr_cnt ? (s->snr_sum / (int32_t)s->snr_cnt) : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* LORA_CALC_H */
