/*
 * link.c — 板間鏈路對端狀態（純邏輯，host 可測）
 * ===========================================================================
 * 無 HAL / RTOS 依賴。tick 差以 uint32_t 無號相減，自然處理 wrap（差值 < 2^31）。
 */
#include "link.h"
#include <string.h>

void LinkPeer_Init(LinkPeer_t *p)
{
    memset(p, 0, sizeof(*p));
}

void LinkPeer_OnPacket(LinkPeer_t *p, const LinkPacket_t *pkt, uint32_t now_ms)
{
    /* 鏈路品質：以 seq 差估丟包（uint8 wrap 用無號相減）。
     * d==0：重複（UART 有序，罕見）→ 不計；1<=d<=32：連續丟 d-1 筆；
     * d>32：視為對端重啟/長時失聯後 seq 跳變，不計以免灌爆 loss 統計。 */
    if (p->seq_valid) {
        uint8_t d = (uint8_t)(pkt->seq - p->last_seq);
        if (d >= 1U && d <= 32U) p->lost_count += (uint32_t)(d - 1U);
    }
    p->last_seq  = pkt->seq;
    p->seq_valid = 1U;
    p->rx_count++;

    p->valid          = 1U;
    p->board_id       = pkt->board_id;
    p->fsm_state      = pkt->fsm_state;
    p->flags          = pkt->flags;
    p->peer_ack_state = pkt->ack_state;
    p->peer_main_arb   = pkt->main_arb;
    p->peer_flash_ready = pkt->flash_ready;
    p->peer_erase_pct   = pkt->erase_pct;
    p->peer_erase_req   = pkt->erase_req;
    p->peer_tick_ms    = pkt->tick_ms;
    p->last_rx_ms     = now_ms;
    p->h_est_cm       = pkt->h_est_cm;
    p->v_est_cms      = pkt->v_est_cms;
    p->baro_alt_cm    = pkt->baro_alt_cm;
    p->a_z_cg         = pkt->a_z_cg;
    p->q_w            = pkt->q_w;
    p->q_x            = pkt->q_x;
    p->q_y            = pkt->q_y;
    p->q_z            = pkt->q_z;
    p->vf_h_cm        = pkt->vf_h_cm;
    p->vf_v_cms       = pkt->vf_v_cms;
    p->bmi_mag_cg     = pkt->bmi_mag_cg;
    p->adxl_mag_cg    = pkt->adxl_mag_cg;
    p->profile_flags  = pkt->profile_flags;

    /* 開傘旗標一旦收到即鎖存（供加法協同判斷對端是否已開傘；不清除） */
    if (pkt->flags & TELEM_FLAG_DROGUE_FIRED)  p->drogue_latched = 1U;
    if (pkt->flags & TELEM_FLAG_MAIN_DEPLOYED) p->main_latched   = 1U;

    /* 手動開傘命令中繼同樣鎖存：對端只要廣播過一次，之後丟包/失聯都不會漏掉命令。 */
    if (pkt->cmd_flags & LINK_CMD_DEPLOY_DROGUE) p->cmd_drogue_latched = 1U;
    if (pkt->cmd_flags & LINK_CMD_DEPLOY_MAIN)   p->cmd_main_latched   = 1U;
}

void LinkPeer_ClearDeployLatches(LinkPeer_t *p)
{
    p->drogue_latched     = 0U;
    p->main_latched       = 0U;
    p->cmd_drogue_latched = 0U;
    p->cmd_main_latched   = 0U;
}

uint8_t LinkPeer_Fresh(const LinkPeer_t *p, uint32_t now_ms, uint32_t timeout_ms)
{
    if (!p->valid) return 0U;
    return ((now_ms - p->last_rx_ms) < timeout_ms) ? 1U : 0U;
}

uint8_t LinkPeer_Synced(const LinkPeer_t *p, uint8_t my_state)
{
    if (!p->valid) return 0U;
    return (p->peer_ack_state == my_state) ? 1U : 0U;
}
