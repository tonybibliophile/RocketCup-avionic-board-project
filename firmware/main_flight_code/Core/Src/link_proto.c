/*
 * link_proto.c — 板間鏈路封包組裝 / 逐位元組解析（純邏輯，host 可測）
 * ===========================================================================
 * CRC-16/CCITT-FALSE 重用 crc16.h 單一實作（與下行遙測、Flash ring 同參數）。
 */
#include "link_proto.h"
#include "crc16.h"
#include <string.h>

uint16_t LinkProto_Build(uint8_t *out, const LinkStatus_t *st)
{
    LinkPacket_t pkt;

    pkt.sync0       = LINK_SYNC0;
    pkt.sync1       = LINK_SYNC1;
    pkt.board_id    = st->board_id;
    pkt.seq         = st->seq;
    pkt.fsm_state   = st->fsm_state;
    pkt.flags       = st->flags;
    pkt.tick_ms     = st->tick_ms;
    pkt.h_est_cm    = st->h_est_cm;
    pkt.v_est_cms   = st->v_est_cms;
    pkt.baro_alt_cm = st->baro_alt_cm;
    pkt.a_z_cg      = st->a_z_cg;
    pkt.ack_state   = st->ack_state;
    pkt.q_w         = st->q_w;
    pkt.q_x         = st->q_x;
    pkt.q_y         = st->q_y;
    pkt.q_z         = st->q_z;
    pkt.main_arb    = st->main_arb;
    pkt.flash_ready = st->flash_ready;
    pkt.erase_pct   = st->erase_pct;
    pkt.vf_h_cm     = st->vf_h_cm;
    pkt.vf_v_cms    = st->vf_v_cms;
    pkt.bmi_mag_cg  = st->bmi_mag_cg;
    pkt.adxl_mag_cg = st->adxl_mag_cg;
    pkt.profile_flags = st->profile_flags;
    pkt.cmd_flags   = st->cmd_flags;
    pkt.erase_req   = st->erase_req;

    /* CRC 覆蓋除最後 2 bytes(crc16 本身) 外的全部內容 */
    pkt.crc16 = crc16_ccitt_false((const uint8_t *)&pkt, (uint16_t)(sizeof(pkt) - 2));

    memcpy(out, &pkt, sizeof(pkt));
    return (uint16_t)sizeof(pkt);
}

void LinkRx_Init(LinkRx_t *rx)
{
    memset(rx, 0, sizeof(*rx));   /* idx=0 + 統計歸零 */
}

uint8_t LinkRx_Feed(LinkRx_t *rx, uint8_t b, LinkPacket_t *out)
{
    const uint8_t SIZE = (uint8_t)sizeof(LinkPacket_t);

    switch (rx->idx) {
        case 0U:                       /* 等待 sync0 */
            if (b == LINK_SYNC0) {
                rx->buf[0] = b;
                rx->idx = 1U;
            }
            return 0U;

        case 1U:                       /* 等待 sync1 */
            if (b == LINK_SYNC1) {
                rx->buf[1] = b;
                rx->idx = 2U;
            } else if (b == LINK_SYNC0) {
                rx->buf[0] = b;        /* 連續 sync0：停在 idx=1 重新對齊 */
                rx->idx = 1U;
            } else {
                rx->idx = 0U;
                rx->resync++;          /* sync0 後遇非法 byte → 重新對齊 */
            }
            return 0U;

        default:                       /* idx >= 2：累積酬載 + CRC */
            rx->buf[rx->idx++] = b;
            if (rx->idx >= SIZE) {
                rx->idx = 0U;          /* 不論成敗，下一筆從頭找 sync */
                uint16_t crc_calc = crc16_ccitt_false(rx->buf, (uint16_t)(SIZE - 2));
                uint16_t crc_recv = (uint16_t)(rx->buf[SIZE - 2] |
                                    ((uint16_t)rx->buf[SIZE - 1] << 8));
                if (crc_calc == crc_recv) {
                    rx->ok++;
                    memcpy(out, rx->buf, SIZE);
                    return 1U;
                }
                rx->crc_err++;
            }
            return 0U;
    }
}
