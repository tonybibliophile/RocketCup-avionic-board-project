/*
 * fsm.c — 飛行狀態機純邏輯（P0-A 自 main.c FSM_Update 逐字搬移，行為保存）
 * ===========================================================================
 * 不依賴 HAL / RTOS；所有條件式與參數值與原 main.c:1425-1562 完全一致，
 * 由 tests/test_fsm.c 黃金飛行剖面鎖定行為。硬體動作由呼叫端執行。
 */
#include "fsm.h"
#include "board_config.h"
#include <math.h>
#include <string.h>

FSM_HotStartDecision_t FSM_HotStartDecide(uint8_t pkt_valid,
                                          uint8_t pkt_fsm_state,
                                          uint32_t pkt_tick_ms,
                                          float pkt_baro_alt_m,
                                          float cur_baro_alt_m,
                                          uint8_t pkt_drogue_fired)
{
    FSM_HotStartDecision_t d;
    d.restore      = 0U;
    d.state        = STATE_PAD;
    d.drogue_fired = 0U;

#if defined(FEATURE_HOTSTART) && (!FEATURE_HOTSTART)
    return d;   /* 熱啟動機制已停用，一律回傳 PAD 狀態 */
#else
    if (!pkt_valid) {
        return d;
    }
    if (pkt_fsm_state < (uint8_t)STATE_BOOST || pkt_fsm_state > (uint8_t)STATE_DESCENT) {
        return d;   /* 正常地面開機（含上次飛行以 LANDED 收尾） */
    }
    if (pkt_tick_ms >= FSM_HOTSTART_MAX_TICK_MS) {
        return d;   /* 飛行 tick 不合理：陳舊/損毀資料 */
    }
    if (fabsf(pkt_baro_alt_m - cur_baro_alt_m) >= FSM_HOTSTART_MAX_ALT_DIFF_M) {
        return d;   /* 高度不連續：多半是上次飛行殘留封包在地面被讀到 */
    }

    d.restore      = 1U;
    d.drogue_fired = (pkt_drogue_fired != 0U) ? 1U : 0U;
    d.state        = (FlightState_t)pkt_fsm_state;
    /* 防二次點火：副傘已點火 → 不得恢復至會再點火的 COAST/APOGEE 之前 */
    if (d.drogue_fired && d.state < STATE_DESCENT) {
        d.state = STATE_DESCENT;
    }
    return d;
#endif
}

void FSM_Init(FSM_Context_t *ctx, FlightState_t s0, uint32_t now_ms,
              uint32_t flight_start_ms, uint8_t drogue_already_fired)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state            = s0;
    ctx->flight_start_ms  = flight_start_ms;
    ctx->state_entered_ms = now_ms;
    ctx->drogue_fired     = (drogue_already_fired != 0U);
}

void FSM_SetState(FSM_Context_t *ctx, FlightState_t target_state)
{
    if (ctx != NULL) {
        ctx->state = target_state;
    }
}

FSM_Action_t FSM_Step(FSM_Context_t *ctx, const FSM_Input_t *in)
{
    FSM_Action_t act;
    memset(&act, 0, sizeof(act));

    const float    h_est = in->h_est;   // 卡爾曼估計高度 (m)
    const float    v_est = in->v_est;   // 卡爾曼估計垂直速度 (m/s)
    const float    a_z   = in->a_z_g;   // 高 G 垂直加速度 (g)
    const uint32_t now   = in->now_ms;

    // 更新觀測到的最大高度
    if (h_est > ctx->max_altitude) {
        ctx->max_altitude = h_est;
    }

    const uint8_t baro_ok = ((in->sensor_bits & FSM_SB_BARO_FAULT) == 0U);

    /* === P0-B：頂點絕對失效保護（最後防線，不依賴任何感測器/EKF） ===
     * BOOST 與 COAST 皆生效：即使燒完判定失效卡在 BOOST、或 EKF 向上發散使
     * 頂點條件永不成立，起飛後失效保護仍強制點火副傘。
     * 主/副航電各用各的失效保護時限：主航電提前 DROGUE_LEAD_TIME_S 開傘，用
     * FSM_FAILSAFE_APOGEE_MS；副航電改為只在真頂點才開，用 FSM_FAILSAFE_APOGEE_BACKUP_MS
     * （= FSM_FAILSAFE_APOGEE_MS + DROGUE_LEAD_TIME_S），避免與副航電自身正常偵測搶跑。 */
    const uint32_t failsafe_apogee_ms = IS_PRIMARY ? FSM_FAILSAFE_APOGEE_MS
                                                    : FSM_FAILSAFE_APOGEE_BACKUP_MS;
    if ((ctx->state == STATE_BOOST || ctx->state == STATE_COAST) &&
        (now - ctx->flight_start_ms) >= failsafe_apogee_ms) {
        ctx->state            = STATE_DEPLOY_DROGUE;   // 走同一顆馬達狀態，非直接跳 APOGEE
        ctx->state_entered_ms = now;
        ctx->drogue_start_ms   = now;   // 記錄馬達啟動 tick
        ctx->drogue_fired     = 1U;
        ctx->failsafe_fired   = 1U;
        act.fire_drogue       = 1U;   // 強制啟動副傘 DC 馬達 (PD13 = HIGH)
        act.event             = FSM_EVT_APOGEE_FAILSAFE;
        ctx->last_vel_z = v_est;
        return act;
    }

    /* 背景 DC 馬達 8s 導通保護計時器 (解耦不卡 FSM 狀態轉移) */
    if (ctx->drogue_start_ms != 0U && (now - ctx->drogue_start_ms) >= FSM_DROGUE_MOTOR_RUN_MS) {
        act.release_drogue   = 1U;   // 滿 8 秒斷開 DC 馬達 (PD13 = LOW)
        ctx->drogue_start_ms = 0U;   // 關閉定時器
    }

    switch (ctx->state) {
        case STATE_PAD: {
            if (in->uplink_armed) {
                if (in->flash_pool_ready) {
                    ctx->state            = STATE_PAD_ARMED;
                    ctx->state_entered_ms = now;
                    act.event             = FSM_EVT_ARMED;
                } else {
                    act.arm_blocked_flash = 1U;   // 留在 STATE_PAD，回報原因（fail-open 由呼叫端組 flash_pool_ready 決定）
                }
            }
            break;
        }

        case STATE_PAD_ARMED: {
            if (!in->uplink_armed) {
                ctx->state            = STATE_PAD;
                ctx->state_entered_ms = now;
                act.event             = FSM_EVT_DISARMED;
                break;
            }

            /* a_z 路徑防手震：須連續 FSM_LIFTOFF_ACCEL_CONSEC_N 週期(200ms)超過
             * 門檻才算數。真實點火持續遠超此窗，手持晃動的瞬間尖峰通常 <100ms
             * （實測手震曾達 3.97g，單一取樣點無法分辨與馬達點火的差異）。 */
            if (a_z > FSM_LIFTOFF_ACCEL_G) {
                if (ctx->consec_liftoff_az < 255U) ctx->consec_liftoff_az++;
            } else {
                ctx->consec_liftoff_az = 0U;
            }
            uint8_t az_liftoff = (ctx->consec_liftoff_az >= FSM_LIFTOFF_ACCEL_CONSEC_N) ? 1U : 0U;

            uint8_t liftoff = 0U;
            if (in->est_calibrated && in->est_healthy) {
                // 起飛觸發條件：高G加速度 > 3.0g(連續200ms) 或高度 > 10.0m
                // 或 baro 相對高度 > 20.0m（P0-B 第三冗餘：ADXL 與估計器雙失效仍可偵測起飛）
                liftoff = (az_liftoff || h_est > FSM_LIFTOFF_ALT_M ||
                           (baro_ok && in->baro_alt_rel > FSM_LIFTOFF_BARO_ALT_M)) ? 1U : 0U;
            } else if (!in->est_healthy) {
                /* P0-C 降級：估計器 unhealthy（含估計器所在 task 死亡 → 永遠不會校準完成）時，
                 * 以不依賴估計器的雙路徑偵測起飛 —— 否則卡 PAD 連失效保護都不會武裝。 */
                liftoff = (az_liftoff ||
                           (baro_ok && in->baro_alt_rel > FSM_LIFTOFF_BARO_ALT_M)) ? 1U : 0U;
            }
            /* else：估計器健康但未校準 → 維持原行為等待校準完成（靜置僅需 3s） */
            if (liftoff) {
                ctx->state            = STATE_BOOST;
                ctx->flight_start_ms  = now;
                ctx->state_entered_ms = now;
                act.event = FSM_EVT_LIFTOFF;
            }
            break;
        }

        case STATE_BOOST: {
            // 馬達燒完判定：加速度 < 0.5g 連續 FSM_BURNOUT_ACCEL_CONSEC_N 週期(50ms)
            // 且 flight time > 1.5 秒。a_z_g 為單筆原始取樣（無濾波），連續週期防護
            // 比照起飛 a_z 路徑：單一取樣掉點不得使 BOOST 誤轉 COAST（無回頭路）。
            if (a_z < FSM_BURNOUT_ACCEL_G) {
                if (ctx->consec_burnout_az < 255U) ctx->consec_burnout_az++;
            } else {
                ctx->consec_burnout_az = 0U;
            }
            if (ctx->consec_burnout_az >= FSM_BURNOUT_ACCEL_CONSEC_N &&
                (now - ctx->state_entered_ms) > FSM_BURNOUT_MIN_MS) {
                ctx->state            = STATE_COAST;
                ctx->state_entered_ms = now;
                act.event = FSM_EVT_BURNOUT;
            }
            break;
        }

        case STATE_COAST: {
            // 動態頂點預估 (預估 4.0s 前開副傘)
            float decel = -9.80665f;
            if (ctx->last_vel_z != 0.0f) {
                float a_z_nav = (v_est - ctx->last_vel_z) / FSM_STEP_PERIOD_S; // 10ms 速度差所得加速度
                if (a_z_nav < -5.0f && a_z_nav > -25.0f) {
                    decel = a_z_nav;
                }
            }

            float t_to_apogee = -v_est / decel;

            /* P0-B：baro 原始趨勢交叉檢查（完全不依賴估計器）。
             * 追蹤 COAST 期 baro 相對高度滾動峰值，自峰值回落 ≥10m 並連續 20 週期
             * （200ms）即視為已過頂點。估計器向上發散時這是真正在頂點附近開傘的
             * 主路徑（失效保護計時器只是更晚的最後防線）。 */
            uint8_t baro_apogee = 0;
            if (baro_ok) {
                if (in->baro_alt_rel > ctx->max_alt_baro) {
                    ctx->max_alt_baro = in->baro_alt_rel;
                }
                if ((ctx->max_alt_baro - in->baro_alt_rel) >= FSM_BARO_APOGEE_DROP_M) {
                    if (ctx->consec_baro_drop < 255U) ctx->consec_baro_drop++;
                } else {
                    ctx->consec_baro_drop = 0U;
                }
                baro_apogee = (ctx->consec_baro_drop >= FSM_BARO_APOGEE_CONSEC) ? 1U : 0U;
            }

            // 頂點判定條件（估計器健康時皆生效；路徑1 由 FSM_APOGEE_DYNAMIC_PREDICT_ENABLED
            // 依 profile 開關 —— 該路徑的 decel fallback 假設「v_est 隨時間由大降到0」，僅真實彈道
            // COAST 段成立，電梯全程近似等速違反此假設，關閉見 fsm.h 該巨集註解）：
            // 1. 主路徑（僅主航電 IS_PRIMARY）：動態預測時間 <= DROGUE_LEAD_TIME_S(4.0s)，
            //    且仍處於上升狀態 (v_est > 0)——主航電提前開引傘。
            //    副航電不走此路徑，改為只在真正頂點才開（路徑 2/3/4，見下）。
            // 2. 真頂點路徑：垂直速度過零 (v_est < -FSM_APOGEE_VFALL_MPS)
            // 3. 真頂點路徑：高度自峰值下降超過 FSM_APOGEE_ALT_DROP_M
            // 4. P0-B：baro 原始趨勢交叉檢查（上方 baro_apogee，不依賴估計器，恆生效，亦屬真頂點路徑）
            // 同時 1~3 必須滿足起飛時間鎖（起飛後累計大於 3.0 秒）
            // P0-C：估計器 unhealthy 時停用 1~3（發散的 h_est/v_est 會誤點火），僅留 4 + 失效計時器
            uint8_t apogee_condition = 0;
            if (in->est_healthy) {
#if FSM_APOGEE_DYNAMIC_PREDICT_ENABLED
                if (IS_PRIMARY && t_to_apogee <= DROGUE_LEAD_TIME_S && v_est > 0.0f) {
                    apogee_condition = 1;                       /* 主路徑：動態預測（僅主航電提前開） */
                } else
#endif
                if (v_est < -FSM_APOGEE_VFALL_MPS) {
                    apogee_condition = 1;                       /* 真頂點：速度過零 */
                } else if ((ctx->max_altitude - h_est) >= FSM_APOGEE_ALT_DROP_M) {
                    apogee_condition = 1;                       /* 真頂點：高度自峰值回落 */
                }
            }
            if (baro_apogee) apogee_condition = 1;              /* P0-B：baro 趨勢交叉檢查（真頂點） */

            /* D1 加法 OR 互救：僅主航電接受——對端（副航電）已開副傘即視為到頂點，讓主航電
             * 跟著一起開（副航電只走真頂點路徑 2/3/4，其開傘信號等同「真頂點已到」，可信）。
             * 反向刻意停用：副航電不接受主航電的提前預測（路徑1）拉動，否則會被主航電
             * 提前 4s 的動態預測牽走，失去「副航電在真正頂點才開」的設計目的。副航電自身
             * 偵測若全失效，仍有 P0-B 頂點絕對失效保護 (FSM_FAILSAFE_APOGEE_MS) 兜底。
             * arm-interlock：本 case 僅在 STATE_COAST 執行（必經 BOOST 起飛+燒完），
             * 且下方 FSM_APOGEE_MIN_FLIGHT_MS 起飛時間鎖同樣套用於此路徑，彈射台不誤觸。 */
            if (IS_PRIMARY && in->peer_drogue_cmd) apogee_condition = 1;

            if (apogee_condition &&
                (now - ctx->flight_start_ms) > FSM_APOGEE_MIN_FLIGHT_MS) {
                ctx->consec_apogee_counts++;
                if (ctx->consec_apogee_counts >= FSM_APOGEE_CONSEC_N) { // 連續 5 個週期 (50ms) 成立以防雜訊
                    ctx->state            = STATE_DEPLOY_DROGUE;
                    ctx->state_entered_ms = now;
                    ctx->drogue_start_ms   = now;   // 記錄 DC 馬達啟動時間
                    ctx->drogue_fired     = 1U;
                    act.fire_drogue       = 1U;   // 啟動副傘 DC 馬達 (PD13 = HIGH)
                    act.event             = FSM_EVT_DEPLOY_DROGUE;
                    act.apogee_t_pred     = t_to_apogee;
                }
            } else {
                ctx->consec_apogee_counts = 0;
            }
            break;
        }

        case STATE_DEPLOY_DROGUE: {
            /* 轉移至 STATE_APOGEE 條件：必須確認已通過頂點並開始回落 (v_est <= 0.0f
             * 或 高度自歷史最高點回落 >= 0.5m)。提前開傘 (3.0s lead) 僅觸發副傘馬達點火，
             * 但狀態機必須確認真實回落才轉入 STATE_APOGEE 並記錄頂點最高高度。 */
            uint8_t is_past_peak = 0U;
            if (in->est_healthy) {
                is_past_peak = (v_est <= 0.0f || (ctx->max_altitude - h_est) >= 0.5f) ? 1U : 0U;
            } else if (baro_ok) {
                is_past_peak = (in->baro_alt_rel <= (ctx->max_alt_baro - 0.5f)) ? 1U : 0U;
            }

            uint32_t lead_ms = (uint32_t)(DROGUE_LEAD_TIME_S * 1000.0f);
            uint8_t lead_expired = ((now - ctx->state_entered_ms) >= lead_ms) ? 1U : 0U;

            uint8_t reached_apogee = (lead_expired || is_past_peak) ? 1U : 0U;

            if (reached_apogee) {
                ctx->state            = STATE_APOGEE;
                ctx->state_entered_ms = now;
                act.event             = FSM_EVT_DROGUE_DONE;
            }
            break;
        }

        case STATE_APOGEE:
            // 頂點狀態記錄：確認並記錄/鎖存最高高度，同一週期立即轉入 DESCENT
            if (h_est > ctx->max_altitude) {
                ctx->max_altitude = h_est;
            }
            if (baro_ok && in->baro_alt_rel > ctx->max_alt_baro) {
                ctx->max_alt_baro = in->baro_alt_rel;
            }
            ctx->state            = STATE_DESCENT;
            ctx->state_entered_ms = now;
            break;

        case STATE_DESCENT: {
            uint8_t main_trigger = 0U;

            /* P0-G 下降安全鎖：火箭/載具必須確實進入過零或開始下降 (v_est <= 0.0f
             * 或 baro 高度低於歷史最高點)，才允許觸發主傘，防止上升期誤闖 DESCENT
             * 且高度低於門檻時在高速上升中誤投主傘。 */
            uint8_t is_descending = 0U;
            if (in->est_healthy) {
                is_descending = (v_est <= 0.0f) ? 1U : 0U;
            } else if (baro_ok) {
                is_descending = (in->baro_alt_rel <= (ctx->max_alt_baro - 0.5f)) ? 1U : 0U;
            }

            if (in->est_healthy) {
                // 動態主傘部署高度計算：h_trigger = h_target + |v_fall| * t_delay (上限為 600m)
                float v_fall = (v_est < 0.0f) ? -v_est : 0.0f;
                float h_trigger_main = TARGET_MAIN_ALTITUDE + v_fall * MAIN_DEPLOY_DELAY_S;
                if (h_trigger_main > FSM_MAIN_MAX_ALT_LIMIT_M) {
                    h_trigger_main = FSM_MAIN_MAX_ALT_LIMIT_M;
                }
                main_trigger = (h_est <= h_trigger_main && h_est <= FSM_MAIN_MAX_ALT_LIMIT_M && is_descending) ? 1U : 0U;
            } else if (baro_ok) {
                /* P0-C 降級：baro 相對高度 ≤ FB門檻 且 ≤ 600m 且已進入下降 */
                main_trigger = (in->baro_alt_rel <= FSM_FB_MAIN_ALT_M &&
                                in->baro_alt_rel <= FSM_MAIN_MAX_ALT_LIMIT_M &&
                                is_descending) ? 1U : 0U;
            }

            // 連續 FSM_MAIN_TRIGGER_CONSEC_N 週期(50ms)防雜訊：比照頂點/起飛路徑，
            // h_est/baro 單筆離群值不得直接觸發主傘展開，須連續成立才算數。
            // 看門狗超時為最後防線，不受此防護延遲（避免看門狗本身被拖慢）。
            if (main_trigger) {
                if (ctx->consec_main_trigger < 255U) ctx->consec_main_trigger++;
            } else {
                ctx->consec_main_trigger = 0U;
            }
            uint8_t main_trigger_confirmed =
                (ctx->consec_main_trigger >= FSM_MAIN_TRIGGER_CONSEC_N) ? 1U : 0U;

            // 觸發條件：(高度低於觸發高度 且 已在下降，連續確認)，或是飛行總時間看門狗超時 (25秒/300秒)
            if (main_trigger_confirmed ||
                (now - ctx->flight_start_ms) > FSM_MAIN_WATCHDOG_MS) {
                ctx->state            = STATE_MAIN_DEPLOY;
                ctx->state_entered_ms = now;
                act.deploy_main       = 1U;   // 部署主傘：PD14 純 GPIO 拉高 SERVO_MAIN_HIGH_MS(1.5s)，不啟 PWM
                act.event             = FSM_EVT_MAIN_DEPLOY;
            }
            break;
        }

        case STATE_MAIN_DEPLOY:
            // 等待 3 秒讓主傘充氣張開，隨後進入落地偵測
            if (now - ctx->state_entered_ms >= FSM_MAIN_INFLATE_MS) {
                ctx->state            = STATE_LANDED;
                ctx->state_entered_ms = now;
                act.event             = FSM_EVT_MAIN_OPEN;
            }
            break;

        case STATE_LANDED:
            if (!ctx->touchdown_latched) {
                uint8_t touchdown = 0U;
                if (in->est_healthy) {
                    // 落地判定：下墜速度趨近零，且高度小於 20m（需 AND 同時成立，防空中誤判）
                    touchdown = (fabsf(v_est) < FSM_TOUCHDOWN_V_MPS &&
                                 h_est < FSM_TOUCHDOWN_ALT_M) ? 1U : 0U;
                } else if (baro_ok) {
                    /* P0-C 降級：2s 視窗內 baro 變化 < 2m 且高度 < 30m */
                    if (ctx->fb_td_ref_tick == 0U) {
                        ctx->fb_td_ref_alt  = in->baro_alt_rel;
                        ctx->fb_td_ref_tick = now;
                    } else if ((now - ctx->fb_td_ref_tick) >= FSM_FB_TOUCHDOWN_WIN_MS) {
                        touchdown = (fabsf(in->baro_alt_rel - ctx->fb_td_ref_alt) < FSM_FB_TOUCHDOWN_DELTA_M &&
                                     in->baro_alt_rel < FSM_FB_TOUCHDOWN_ALT_M) ? 1U : 0U;
                        ctx->fb_td_ref_alt  = in->baro_alt_rel;
                        ctx->fb_td_ref_tick = now;
                    }
                }
                if (touchdown) {
                    ctx->touchdown_latched = 1U;
                    act.start_buzzer       = 1U;   // 開啟板載尋標蜂鳴器（持續鳴叫，利於落點尋標）
                    act.event              = FSM_EVT_TOUCHDOWN;
                }
            }
            break;

        default:
            break;
    }

    ctx->last_vel_z = v_est; // 儲存速度歷史

    return act;
}
