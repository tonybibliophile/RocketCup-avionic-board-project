/**
 ******************************************************************************
 * @file    telemetry.c
 * @brief   共用二進制下行遙測封包打包器 (binary + CRC16)
 *
 * 由現有感測器 / EKF / 系統全域組裝一筆 TelemetryPacket_t（見 telemetry.h 欄位契約），
 * 結尾補 CRC-16/CCITT-FALSE。E22(433) 與 E80(920) 兩條鏈路發送同一份位元組。
 ******************************************************************************
 */
#include "telemetry.h"
#include "main.h"

#include "bmi088.h"
#include "adxl375.h"
#include "bmp388.h"
#include "ekf.h"
#include "gps.h"
#include "crc16.h"
#if FEATURE_LINK
#include "link_hw.h"   /* Link_GetPeer / Link_GetRx / Link_PeerFresh：中繼對端摘要 */
#endif

#include <math.h>      /* sqrtf：高G 模長 */
#include <string.h>

/* --- 來自 main.c 的系統全域（背景任務更新，遙測唯讀） --- */
extern BMI088_Data_t     imu_data;
extern ADXL375_Data_t    highg_data;
extern BMP388_Data_t     baro_data;
extern FlightState_t     current_fsm_state;
extern volatile float    g_main_task_cpu_usage;
extern volatile uint16_t g_bat_voltage_mv;   /* 飛控迴圈更新的最新電池電壓 (mV) */
extern uint8_t           adxl375_ok;
extern uint8_t           sd_logging_active;
extern volatile uint8_t  g_fsm_failsafe_fired;  /* P0-B：失效保護計時器強制點火鎖存 */
extern volatile uint8_t  g_sensor_fault_bits;   /* P0-D：感測器健康彙整位（SH_BIT_*） */
extern volatile uint8_t  g_hotstart_restored;   /* P0-F：熱啟動恢復鎖存 */
extern volatile uint8_t  g_main_deployed;       /* 主傘已部署鎖存（PD14 曾拉高；只高 1.5s 故需鎖存） */
extern volatile uint8_t  g_arm_blocked_flash;   /* ARM 被 flash pool 未達標擋下（見 FSM_Update） */
extern volatile float    g_vf_h_m;              /* 垂直濾波器 (VF) 高度 (m)，FEATURE_VFILTER=0 時恆為 0 */
extern volatile float    g_vf_v_ms;             /* 垂直濾波器 (VF) 垂直速度 (m/s) */
extern volatile float    g_max_alt_m;           /* 飛行滾動極值：最大相對高度 (m)，ARM 時歸零 */
extern volatile float    g_max_vel_ms;          /* 飛行滾動極值：最大垂直速度 (m/s) */
extern volatile float    g_max_acc_g;           /* 飛行滾動極值：最大合加速度 |a| (g)，BMI088 逐筆掃描 */
extern volatile int16_t  g_drogue_alt_m;        /* 副傘實際開傘相對高度 (m)；未開傘 = TELEM_DEPLOY_ALT_NA */
extern volatile int16_t  g_main_alt_m;          /* 主傘實際開傘相對高度 (m)；未開傘 = TELEM_DEPLOY_ALT_NA */
/* ★ 舊版由 __HAL_TIM_GET_COMPARE(&htim4, CH3) 推導 MAIN_DEPLOYED；主傘改為純 GPIO 拉高
 * 1.5s 後，TIM4 PWM 全程不啟動、腳位現況也只高 1.5s，故改讀 main.c 的鎖存旗標
 * g_main_deployed（見上方 extern），htim4 於本檔不再需要。 */

/* float → int16 飽和轉換，避免大數值 wrap 成錯誤負值 */
static int16_t sat_i16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

uint16_t telem_crc16(const uint8_t *data, uint16_t len)
{
    return crc16_ccitt_false(data, len);   /* P1：統一至 crc16.h 單一實作 */
}

uint16_t Telemetry_Build(uint8_t *out)
{
    static uint8_t s_seq = 0;
    TelemetryPacket_t pkt;

    pkt.sync0     = TELEM_SYNC0;
    pkt.sync1     = TELEM_SYNC1;
    pkt.seq       = s_seq++;
    pkt.fsm_state = (uint8_t)current_fsm_state;
    pkt.tick_ms   = HAL_GetTick();

    /* --- EKF 狀態（高度 / 速度 / 姿態四元數） --- */
    EKF_State_t ekf = EKF_GetState();
    pkt.ekf_pos_z_cm  = (int32_t)(ekf.pos_z * 100.0f);
    pkt.ekf_vel_z_cms = (int32_t)(ekf.vel_z * 100.0f);
    pkt.ekf_q0 = sat_i16(ekf.q[0] * 10000.0f);
    pkt.ekf_q1 = sat_i16(ekf.q[1] * 10000.0f);
    pkt.ekf_q2 = sat_i16(ekf.q[2] * 10000.0f);
    pkt.ekf_q3 = sat_i16(ekf.q[3] * 10000.0f);

    /* --- 氣壓計（只下鏈高度：氣壓與高度是同一個量，altitude 就是由 pressure 換算而來，
     * 下鏈兩個等於白付 4 bytes 空中時間。原始壓力仍完整寫進 Flash ring） --- */
    pkt.baro_alt_cm   = (int32_t)(baro_data.altitude * 100.0f);

    /* --- BMI088 加速度 (mg) / 角速度 (dps)，sensor frame --- */
    pkt.imu_ax_mg  = sat_i16(imu_data.ax * 1000.0f);
    pkt.imu_ay_mg  = sat_i16(imu_data.ay * 1000.0f);
    pkt.imu_az_mg  = sat_i16(imu_data.az * 1000.0f);
    pkt.gyro_x_dps = sat_i16(imu_data.gx);
    pkt.gyro_y_dps = sat_i16(imu_data.gy);
    pkt.gyro_z_dps = sat_i16(imu_data.gz);

    /* --- ADXL375 高G 模長 (cg = 0.01g)；±200g 量程故用 cg 以免 int16 溢位。
     * 只送模長不送三軸，理由見 telemetry.h hg_mag_cg 註解（三軸原始值仍進 Flash）。 --- */
    if (adxl375_ok) {
        pkt.hg_mag_cg = sat_i16(sqrtf(highg_data.ax * highg_data.ax +
                                      highg_data.ay * highg_data.ay +
                                      highg_data.az * highg_data.az) * 100.0f);
    } else {
        pkt.hg_mag_cg = 0;
    }

    /* --- GPS --- */
    const GPS_Data_t *g = GPS_GetData();
    pkt.gps_lat_1e6 = g->lat_1e6;
    pkt.gps_lon_1e6 = g->lon_1e6;
    pkt.gps_alt_m   = sat_i16(g->altitude_m);
    pkt.gps_sats    = g->satellites;
    pkt.gps_fix     = g->fix_valid;

    /* --- 電源 / CPU 佔用率 --- */
    pkt.bat_mv       = g_bat_voltage_mv;
    pkt.cpu_main_x10 = (uint16_t)(g_main_task_cpu_usage * 10.0f);

    /* --- 系統旗標 --- */
    uint8_t flags = 0;
    if (HAL_GPIO_ReadPin(FIRE_GPIO_Port, FIRE_Pin) == GPIO_PIN_SET)   flags |= TELEM_FLAG_DROGUE_FIRED;
    if (g_main_deployed)                                              flags |= TELEM_FLAG_MAIN_DEPLOYED;
    if (sd_logging_active)                                            flags |= TELEM_FLAG_SD_ACTIVE;
    if (GPS_IsStale(2000))                                            flags |= TELEM_FLAG_GPS_STALE;
    if (EKF_GetHealthBits() != 0U)                                    flags |= TELEM_FLAG_EKF_UNHEALTHY;
    if (g_sensor_fault_bits != 0U)                                    flags |= TELEM_FLAG_SENSOR_FAULT;
    if (g_fsm_failsafe_fired)                                         flags |= TELEM_FLAG_FAILSAFE;
    if (g_hotstart_restored)                                          flags |= TELEM_FLAG_HOTSTART;
    pkt.flags = flags;

    /* --- P1：完整健康位（flags 僅是「有/無問題」摘要，這裡給出哪一位故障） --- */
    pkt.health_bits = EKF_GetHealthBits();
    pkt.sensor_bits = g_sensor_fault_bits;

    /* --- 本板垂直濾波器 (VF)：與 EKF 並列下鏈，供地面站同屏比對 --- */
    pkt.vf_pos_z_cm  = (int32_t)(g_vf_h_m * 100.0f);
    pkt.vf_vel_z_cms = (int32_t)(g_vf_v_ms * 100.0f);

    /* --- 飛行滾動極值（見 telemetry.h 說明）：飛控迴圈全速率追蹤，此處只讀出。
     * 高度負值夾到 0（uint16），加速度本就非負。 --- */
    pkt.max_alt_m  = (g_max_alt_m > 0.0f)
                     ? (uint16_t)((g_max_alt_m > 65535.0f) ? 65535.0f : g_max_alt_m) : 0U;
    pkt.max_vel_ms = sat_i16(g_max_vel_ms);
    pkt.max_acc_cg = (uint16_t)((g_max_acc_g * 100.0f > 65535.0f) ? 65535.0f
                                                                  : (g_max_acc_g * 100.0f));

    /* --- 實際開傘高度：飛控迴圈在開傘那一刻就地鎖存，此處只讀出（未開傘為哨兵）。 --- */
    pkt.drogue_alt_m = g_drogue_alt_m;
    pkt.main_alt_m   = g_main_alt_m;

    /* --- 主/副協同：中繼對端(副板)摘要供地面雙板監看（僅主板有下鏈；FEATURE_LINK） --- */
#if FEATURE_LINK
    {
        const LinkPeer_t *pr  = Link_GetPeer();
        uint8_t plink = 0U;
        uint8_t lstat = Link_GetStatus();
        if (pr->valid)                     plink |= TELEM_PEER_EVER;
        if (Link_PeerFresh(HAL_GetTick()))  plink |= TELEM_PEER_FRESH;
        if (lstat & LINK_STATUS_LOST)       plink |= TELEM_PEER_LOST;
        if (lstat & LINK_STATUS_DESYNC)     plink |= TELEM_PEER_DESYNC;
        pkt.peer_fsm_state = pr->fsm_state;
        /* 開傘兩位改中繼「鎖存值」：對端的 DROGUE_FIRED 是它 PD13 的即時腳位狀態，導通窗
         * 一過就回 0（主 8s / 副 3s，見 fsm.h FSM_DROGUE_MOTOR_RUN_*_MS）。副板 3s 窗在
         * ~2Hz 下鏈只覆蓋約 6 包，全丟就等於地面永遠不知道副板開過引傘——而副板沒有
         * drogue_alt_m/main_alt_m 那種永久證據（那兩欄只記本板）。LinkPeer_OnPacket 本來
         * 就有 drogue_latched/main_latched 鎖存（見 link.c），這裡 OR 進來即可：開過之後
         * 每一包都帶著，抗丟包。
         * ★語意：peer_flags 的 bit0/bit1 = 「對端開過」，不是「對端此刻正在導通」。
         *   其餘位（FAILSAFE / EKF_UNHEALTHY / SENSOR_FAULT）維持即時值不動——那些本來
         *   就該看當下狀態。要看對端 PD13 即時導通請改看 [LINK] 診斷行的 flags 欄。 */
        uint8_t pflags = pr->flags;
        if (pr->drogue_latched) pflags |= TELEM_FLAG_DROGUE_FIRED;
        if (pr->main_latched)   pflags |= TELEM_FLAG_MAIN_DEPLOYED;
        pkt.peer_flags     = pflags;
        pkt.peer_baro_cm   = pr->baro_alt_cm;
        pkt.peer_link      = plink;
        pkt.peer_vf_h_cm   = pr->vf_h_cm;
        pkt.peer_vf_v_cms  = pr->vf_v_cms;
        pkt.peer_bench_arb = pr->peer_main_arb;
    }
#else
    pkt.peer_fsm_state = 0U;
    pkt.peer_flags     = 0U;
    pkt.peer_baro_cm   = 0;
    pkt.peer_link      = 0U;
    pkt.peer_vf_h_cm   = 0;
    pkt.peer_vf_v_cms  = 0;
    pkt.peer_bench_arb = 0U;
#endif

    /* --- ARM 被擋下原因 ---
     * ★2026-08-01：開機自動補擦到基本需求，未擦除的警告與 ARM 防護全數移除，故 bit1
     * （TELEM_ARM_NEED_ERASE）不再設定；bit0 也因 flash_pool_ready 恆為 1 而恆為 0。
     * 兩個位元保留在協定裡（封包長度不變 ⇒ 不需三板同燒），將來要重啟防護可直接復用。 */
    pkt.arm_flags = (uint8_t)(g_arm_blocked_flash ? TELEM_ARM_BLOCKED_FLASH_POOL : 0U);

    /* --- 電梯測試 profile 醒目標示：本板依編譯期巨集直填；對端（副板）由板間鏈路中繼
     * （僅主板下鏈才看得到對端，FEATURE_LINK=0 時 pr 已在上面全 0，peer 位自然不會設）。 --- */
    {
        uint8_t prof = 0U;
#if FLIGHT_PROFILE_ELEVATOR
        prof |= TELEM_PROFILE_SELF_ELEVATOR;
#endif
#if FEATURE_LINK
        if (Link_GetPeer()->profile_flags & TELEM_PROFILE_SELF_ELEVATOR) prof |= TELEM_PROFILE_PEER_ELEVATOR;
#endif
        pkt.profile_flags = prof;
    }

    /* --- CRC16 覆蓋除最後 2 bytes(crc16 本身) 外的全部內容 --- */
    pkt.crc16 = telem_crc16((const uint8_t *)&pkt, (uint16_t)(sizeof(pkt) - 2));

    memcpy(out, &pkt, sizeof(pkt));
    return (uint16_t)sizeof(pkt);
}
