/*
 * gs_log.c — 地面站紀錄組裝 + CSV 格式化（純邏輯，host 可測）
 * ===========================================================================
 * CRC-16/CCITT-FALSE 重用 crc16.h 單一實作（與下行遙測、Flash ring、板間鏈路同參數）。
 * 全部以縮放整數輸出，無 %f（遵專案黃金法則）。
 */
#include "gs_log.h"
#include "crc16.h"
#include <string.h>
#include <stdio.h>

void GsLog_BuildRecord(GsLogRecord_t *rec,
                       uint8_t link_source, int16_t rssi_dbm, int16_t snr_cb,
                       uint32_t rx_tick_ms, uint32_t rx_utc_ms,
                       uint32_t aligned_utc_ms, int32_t offset_ms,
                       int32_t gs_lat_1e6, int32_t gs_lon_1e6,
                       int16_t gs_alt_m, uint8_t gs_sats, uint8_t gs_fix,
                       const TelemetryPacket_t *pkt)
{
    rec->magic0        = GS_LOG_MAGIC0;
    rec->magic1        = GS_LOG_MAGIC1;
    rec->link_source   = link_source;
    rec->_rsv          = 0U;
    rec->rssi_dbm      = rssi_dbm;
    rec->snr_cb        = snr_cb;
    rec->rx_tick_ms    = rx_tick_ms;
    rec->rx_utc_ms     = rx_utc_ms;
    rec->aligned_utc_ms = aligned_utc_ms;
    rec->offset_ms     = offset_ms;
    rec->gs_lat_1e6    = gs_lat_1e6;
    rec->gs_lon_1e6    = gs_lon_1e6;
    rec->gs_alt_m      = gs_alt_m;
    rec->gs_sats       = gs_sats;
    rec->gs_fix        = gs_fix;
    memcpy(&rec->pkt, pkt, sizeof(rec->pkt));

    rec->crc16 = crc16_ccitt_false((const uint8_t *)rec,
                                   (uint16_t)(sizeof(*rec) - 2));
}

uint8_t GsLog_RecordValid(const GsLogRecord_t *rec)
{
    if (rec->magic0 != GS_LOG_MAGIC0 || rec->magic1 != GS_LOG_MAGIC1) return 0U;
    uint16_t crc = crc16_ccitt_false((const uint8_t *)rec,
                                     (uint16_t)(sizeof(*rec) - 2));
    return (crc == rec->crc16) ? 1U : 0U;
}

/* 當日 UTC 毫秒 → "HH:MM:SS.mmm"（無浮點）。buf 須 >= 13 bytes。 */
static void gs_fmt_hms(char *buf, size_t cap, uint32_t ms_of_day)
{
    uint32_t hh  = ms_of_day / 3600000U;
    uint32_t rem = ms_of_day % 3600000U;
    uint32_t mm  = rem / 60000U;  rem %= 60000U;
    uint32_t ss  = rem / 1000U;
    uint32_t mmm = rem % 1000U;
    snprintf(buf, cap, "%02u:%02u:%02u.%03u",
             (unsigned)hh, (unsigned)mm, (unsigned)ss, (unsigned)mmm);
}

int GsLog_CsvHeader(char *out, size_t cap)
{
    return snprintf(out, cap,
        "rx_utc,aligned_utc,link_mhz,rssi_dbm,snr_cb,offset_ms,"
        "seq,fsm_state,rkt_tick_ms,ekf_alt_cm,ekf_vel_cms,baro_alt_cm,baro_pa,"
        "vf_alt_cm,vf_vel_cms,"
        "gps_lat_1e6,gps_lon_1e6,gps_alt_m,gps_sats,gps_fix,bat_mv,"
        "flags,health,sensor,"
        "peer_fsm,peer_flags,peer_h_cm,peer_v_cms,peer_baro_cm,peer_link,peer_loss_pmil,"
        "peer_az_cg,peer_vf_h_cm,peer_vf_v_cms,peer_bench_arb,"
        "gs_lat_1e6,gs_lon_1e6,gs_alt_m,gs_sats,gs_fix,"
        "rx_utc_ms,aligned_utc_ms\r\n");
}

int GsLog_FormatCsvRow(char *out, size_t cap, const GsLogRecord_t *rec)
{
    const TelemetryPacket_t *p = &rec->pkt;
    char rx_hms[16], al_hms[16];
    gs_fmt_hms(rx_hms, sizeof(rx_hms), rec->rx_utc_ms);
    gs_fmt_hms(al_hms, sizeof(al_hms), rec->aligned_utc_ms);

    unsigned link_mhz = (rec->link_source == GS_LINK_920) ? 920U : 433U;

    return snprintf(out, cap,
        "%s,%s,%u,%d,%d,%d,"
        "%u,%u,%u,%d,%d,%d,%u,"
        "%d,%d,"
        "%d,%d,%d,%u,%u,%u,"
        "0x%02X,0x%02X,0x%02X,"
        "%u,0x%02X,%d,%d,%d,0x%02X,%u,"
        "%d,%d,%d,%u,"
        "%d,%d,%d,%u,%u,"
        "%u,%u\r\n",
        rx_hms, al_hms, link_mhz,
        (int)rec->rssi_dbm, (int)rec->snr_cb, (int)rec->offset_ms,
        (unsigned)p->seq, (unsigned)p->fsm_state, (unsigned)p->tick_ms,
        (int)p->ekf_pos_z_cm, (int)p->ekf_vel_z_cms, (int)p->baro_alt_cm,
        (unsigned)p->baro_press_pa,
        (int)p->vf_pos_z_cm, (int)p->vf_vel_z_cms,
        (int)p->gps_lat_1e6, (int)p->gps_lon_1e6, (int)p->gps_alt_m,
        (unsigned)p->gps_sats, (unsigned)p->gps_fix, (unsigned)p->bat_mv,
        (unsigned)p->flags, (unsigned)p->health_bits, (unsigned)p->sensor_bits,
        (unsigned)p->peer_fsm_state, (unsigned)p->peer_flags,
        (int)p->peer_h_cm, (int)p->peer_v_cms, (int)p->peer_baro_cm,
        (unsigned)p->peer_link, (unsigned)p->peer_loss_pmil,
        (int)p->peer_az_cg, (int)p->peer_vf_h_cm, (int)p->peer_vf_v_cms,
        (unsigned)p->peer_bench_arb,
        (int)rec->gs_lat_1e6, (int)rec->gs_lon_1e6, (int)rec->gs_alt_m,
        (unsigned)rec->gs_sats, (unsigned)rec->gs_fix,
        (unsigned)rec->rx_utc_ms, (unsigned)rec->aligned_utc_ms);
}
