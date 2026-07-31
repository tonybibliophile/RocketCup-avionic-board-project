/*
 * test_link.c — 板間鏈路對端狀態追蹤情境測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 對稱獨立冗餘：本檔鎖定 LinkPeer 對端狀態追蹤（加法協同的資料基礎）：
 *   [1] freshness：未收包→失聯；收包後 timeout 內新鮮、逾時失聯
 *   [2] 開傘旗標鎖存：收過一次即維持，後續無旗標封包不清除
 *       （drogue_latched / main_latched 供 Phase D 加法協同判斷對端是否已開傘）
 *   [2b] 手動開傘命令中繼鎖存：cmd_flags 與 flags 分離（命令 vs 已開傘事實）
 *   [2c] DISARM 清除開傘鎖存：四個鎖存全清、QoS/freshness 不動
 */
#include <stdio.h>
#include <string.h>
#include "board_config.h"
#include "link.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

static LinkPacket_t make_pkt(uint8_t board_id, uint8_t fsm_state, uint8_t flags, uint32_t tick) {
    LinkPacket_t p; memset(&p, 0, sizeof(p));
    p.sync0 = LINK_SYNC0; p.sync1 = LINK_SYNC1;
    p.board_id = board_id; p.fsm_state = fsm_state; p.flags = flags; p.tick_ms = tick;
    return p;
}

static void test_freshness(void) {
    printf("[1] freshness（timeout=%u ms）\n", (unsigned)LINK_PEER_TIMEOUT_MS);
    LinkPeer_t pr; LinkPeer_Init(&pr);
    check("未收包 → 失聯", !LinkPeer_Fresh(&pr, 0, LINK_PEER_TIMEOUT_MS));
    LinkPacket_t pkt = make_pkt(LINK_BOARD_PRIMARY, 3, 0, 1000);
    LinkPeer_OnPacket(&pr, &pkt, 1000);
    check("收包當下 → 新鮮",            LinkPeer_Fresh(&pr, 1000, LINK_PEER_TIMEOUT_MS));
    check("timeout-1 內 → 新鮮",        LinkPeer_Fresh(&pr, 1000 + LINK_PEER_TIMEOUT_MS - 1, LINK_PEER_TIMEOUT_MS));
    check("達 timeout → 失聯",          !LinkPeer_Fresh(&pr, 1000 + LINK_PEER_TIMEOUT_MS, LINK_PEER_TIMEOUT_MS));
}

static void test_latch(void) {
    printf("[2] 開傘旗標鎖存\n");
    LinkPeer_t pr; LinkPeer_Init(&pr);
    LinkPacket_t a = make_pkt(LINK_BOARD_PRIMARY, 4, TELEM_FLAG_DROGUE_FIRED, 100);
    LinkPeer_OnPacket(&pr, &a, 100);
    check("收到 DROGUE_FIRED → drogue_latched", pr.drogue_latched == 1);
    check("尚未收 MAIN → main_latched=0",       pr.main_latched == 0);

    LinkPacket_t b = make_pkt(LINK_BOARD_PRIMARY, 5, 0, 200);   /* 後續無旗標 */
    LinkPeer_OnPacket(&pr, &b, 200);
    check("無旗標封包不清除 drogue 鎖存", pr.drogue_latched == 1);

    LinkPacket_t c = make_pkt(LINK_BOARD_PRIMARY, 6, TELEM_FLAG_MAIN_DEPLOYED, 300);
    LinkPeer_OnPacket(&pr, &c, 300);
    check("收到 MAIN_DEPLOYED → main_latched", pr.main_latched == 1);
    check("drogue 鎖存仍維持",                 pr.drogue_latched == 1);
}

/* ★手動開傘「命令」中繼（LINK_CMD_DEPLOY_*）：地面站只打得到主航電，副航電靠這條
 * 一起開傘。重點是與 flags 的 DROGUE_FIRED/MAIN_DEPLOYED（＝已開傘事實，含 FSM 自動
 * 開傘）分離——後者不得帶動對端，否則主航電提前 4s 的動態預測會把副航電牽走。 */
static void test_cmd_relay(void) {
    printf("[2b] 手動開傘命令中繼鎖存（LINK_CMD_DEPLOY_*）\n");
    LinkPeer_t pr; LinkPeer_Init(&pr);

    /* FSM 自動開傘（只有 flags，無 cmd_flags）不得被誤判為手動命令 */
    LinkPacket_t auto_fire = make_pkt(LINK_BOARD_PRIMARY, 5, TELEM_FLAG_DROGUE_FIRED, 100);
    LinkPeer_OnPacket(&pr, &auto_fire, 100);
    check("僅 DROGUE_FIRED（FSM 自動）→ cmd_drogue_latched 不亮", pr.cmd_drogue_latched == 0);
    check("僅 DROGUE_FIRED（FSM 自動）→ cmd_main_latched 不亮",   pr.cmd_main_latched == 0);

    LinkPacket_t cmd_d = make_pkt(LINK_BOARD_PRIMARY, 5, 0, 200);
    cmd_d.cmd_flags = LINK_CMD_DEPLOY_DROGUE;
    LinkPeer_OnPacket(&pr, &cmd_d, 200);
    check("收到 CMD_DEPLOY_DROGUE → cmd_drogue_latched", pr.cmd_drogue_latched == 1);
    check("尚未收 CMD_DEPLOY_MAIN → cmd_main_latched=0", pr.cmd_main_latched == 0);

    /* 命令鎖存：對端之後即使送出不含 cmd_flags 的封包（例如重開機後尚未重建），
     * 本板已收到的命令不得被清掉。 */
    LinkPacket_t blank = make_pkt(LINK_BOARD_PRIMARY, 5, 0, 300);
    LinkPeer_OnPacket(&pr, &blank, 300);
    check("無 cmd_flags 封包不清除 drogue 命令鎖存", pr.cmd_drogue_latched == 1);

    LinkPacket_t cmd_both = make_pkt(LINK_BOARD_PRIMARY, 7, 0, 400);
    cmd_both.cmd_flags = LINK_CMD_DEPLOY_DROGUE | LINK_CMD_DEPLOY_MAIN;
    LinkPeer_OnPacket(&pr, &cmd_both, 400);
    check("收到 CMD_DEPLOY_BOTH → main 命令也鎖存", pr.cmd_main_latched == 1);
    check("drogue 命令鎖存仍維持",                   pr.cmd_drogue_latched == 1);
}

/* DISARM 重置：四個開傘鎖存全清，其餘對端狀態與 QoS 統計不動（清掉會誤報失聯/丟包）。 */
static void test_clear_deploy_latches(void) {
    printf("[2c] DISARM 清除開傘鎖存（LinkPeer_ClearDeployLatches）\n");
    LinkPeer_t pr; LinkPeer_Init(&pr);

    LinkPacket_t p = make_pkt(LINK_BOARD_PRIMARY, 8,
                              TELEM_FLAG_DROGUE_FIRED | TELEM_FLAG_MAIN_DEPLOYED, 500);
    p.seq       = 9;
    p.cmd_flags = LINK_CMD_DEPLOY_DROGUE | LINK_CMD_DEPLOY_MAIN;
    LinkPeer_OnPacket(&pr, &p, 500);
    check("前置：四個鎖存皆亮",
          pr.drogue_latched && pr.main_latched &&
          pr.cmd_drogue_latched && pr.cmd_main_latched);

    LinkPeer_ClearDeployLatches(&pr);
    check("清除後 drogue_latched=0",     pr.drogue_latched == 0);
    check("清除後 main_latched=0",       pr.main_latched == 0);
    check("清除後 cmd_drogue_latched=0", pr.cmd_drogue_latched == 0);
    check("清除後 cmd_main_latched=0",   pr.cmd_main_latched == 0);
    check("freshness 不受影響",  LinkPeer_Fresh(&pr, 500, LINK_PEER_TIMEOUT_MS));
    check("QoS 統計不被清掉",    pr.rx_count == 1 && pr.seq_valid == 1);
    check("對端 fsm_state 保留", pr.fsm_state == 8);

    /* 清完之後對端仍在廣播 → 立刻重新鎖存。這正是 main.c 解除武裝期間要「持續」清、
     * 而非清一次就好的原因（兩板 DISARM 有 ~50ms 傳播差）。 */
    LinkPeer_OnPacket(&pr, &p, 600);
    check("對端仍在廣播 → 命令鎖存立刻回來", pr.cmd_drogue_latched == 1);
}

static void test_seq_loss(void) {
    printf("[3] 鏈路品質：seq 丟包估計\n");
    LinkPeer_t pr; LinkPeer_Init(&pr);
    for (uint8_t s = 0; s <= 2; s++) {
        LinkPacket_t p = make_pkt(LINK_BOARD_PRIMARY, 3, 0, 100u + (uint32_t)s * 50u);
        p.seq = s;
        LinkPeer_OnPacket(&pr, &p, 100u + (uint32_t)s * 50u);
    }
    check("連續 3 筆 → rx_count=3, lost=0", pr.rx_count == 3 && pr.lost_count == 0);

    LinkPacket_t skip = make_pkt(LINK_BOARD_PRIMARY, 3, 0, 300);
    skip.seq = 5;                       /* 跳過 3,4 → 丟 2 筆 */
    LinkPeer_OnPacket(&pr, &skip, 300);
    check("seq 2→跳到5 → lost+=2", pr.rx_count == 4 && pr.lost_count == 2);

    LinkPacket_t jump = make_pkt(LINK_BOARD_PRIMARY, 3, 0, 400);
    jump.seq = 100;                     /* d=95 > 32 → 視為對端重啟，不計 */
    LinkPeer_OnPacket(&pr, &jump, 400);
    check("大跳變(>32) 不灌爆 loss", pr.rx_count == 5 && pr.lost_count == 2);
}

static void test_synced(void) {
    printf("[4] echo-ACK 失同步偵測（LinkPeer_Synced）\n");
    LinkPeer_t pr; LinkPeer_Init(&pr);
    check("未收包 → 未同步", !LinkPeer_Synced(&pr, 2));

    LinkPacket_t p = make_pkt(LINK_BOARD_PRIMARY, 3, 0, 100);
    p.ack_state = 2;                    /* 對端回報「它認為我方在 state 2」 */
    LinkPeer_OnPacket(&pr, &p, 100);
    check("對端 echo == 我方狀態 → 同步",       LinkPeer_Synced(&pr, 2));
    check("我方前進到 3、對端仍回 2 → 失同步",  !LinkPeer_Synced(&pr, 3));

    LinkPacket_t p2 = make_pkt(LINK_BOARD_PRIMARY, 3, 0, 150);
    p2.ack_state = 3;                   /* 對端跟上 */
    LinkPeer_OnPacket(&pr, &p2, 150);
    check("對端跟上到 3 → 重新同步", LinkPeer_Synced(&pr, 3));
}

int main(void) {
    printf("=== test_link：對端狀態追蹤 ===\n");
    test_freshness();
    test_latch();
    test_cmd_relay();
    test_clear_deploy_latches();
    test_seq_loss();
    test_synced();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
