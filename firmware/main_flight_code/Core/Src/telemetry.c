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
#include "mmc5983.h"
#include "sensor_axis.h"
#include "crc16.h"
#if FEATURE_LINK
#include "link_hw.h"   /* Link_GetPeer / Link_GetRx / Link_PeerFresh：中繼對端摘要 */
#endif

#include <string.h>

/* --- 來自 main.c 的系統全域（背景任務更新，遙測唯讀） --- */
extern BMI088_Data_t     imu_data;
extern ADXL375_Data_t    highg_data;
extern BMP388_Data_t     baro_data;
extern FlightState_t     current_fsm_state;
extern volatile float    g_main_task_cpu_usage;
extern volatile uint16_t g_bat_voltage_mv;   /* 飛控迴圈更新的最新電池電壓 (mV) */
extern uint8_t           adxl375_ok;
extern uint8_t           mag_ok;
extern uint8_t           sd_logging_active;
extern volatile uint8_t  g_fsm_failsafe_fired;  /* P0-B：失效保護計時器強制點火鎖存 */
extern volatile uint8_t  g_sensor_fault_bits;   /* P0-D：感測器健康彙整位（SH_BIT_*） */
extern volatile uint8_t  g_hotstart_restored;   /* P0-F：熱啟動恢復鎖存 */
extern volatile uint8_t  g_arm_blocked_flash;   /* ARM 被 flash pool 未達標擋下（見 FSM_Update） */
extern volatile float    g_vf_h_m;              /* 垂直濾波器 (VF) 高度 (m)，FEATURE_VFILTER=0 時恆為 0 */
extern volatile float    g_vf_v_ms;             /* 垂直濾波器 (VF) 垂直速度 (m/s) */
extern TIM_HandleTypeDef htim4;              /* PWM_Servo（主傘舵機）CH3 */

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

    /* --- 氣壓計 --- */
    pkt.baro_alt_cm   = (int32_t)(baro_data.altitude * 100.0f);
    pkt.baro_press_pa = (uint32_t)(baro_data.pressure);

    /* --- BMI088 加速度 (mg) / 角速度 (dps)，sensor frame --- */
    pkt.imu_ax_mg  = sat_i16(imu_data.ax * 1000.0f);
    pkt.imu_ay_mg  = sat_i16(imu_data.ay * 1000.0f);
    pkt.imu_az_mg  = sat_i16(imu_data.az * 1000.0f);
    pkt.gyro_x_dps = sat_i16(imu_data.gx);
    pkt.gyro_y_dps = sat_i16(imu_data.gy);
    pkt.gyro_z_dps = sat_i16(imu_data.gz);

    /* --- ADXL375 高G (cg = 0.01g)，sensor frame；±200g 量程故用 cg 以免 int16 溢位 --- */
    if (adxl375_ok) {
        pkt.hg_ax_cg = sat_i16(highg_data.ax * 100.0f);
        pkt.hg_ay_cg = sat_i16(highg_data.ay * 100.0f);
        pkt.hg_az_cg = sat_i16(highg_data.az * 100.0f);
    } else {
        pkt.hg_ax_cg = pkt.hg_ay_cg = pkt.hg_az_cg = 0;
    }

    /* --- MMC5983 磁場 (mGauss)，重映射至 body frame（與診斷 [MAG] 同來源） --- */
    if (mag_ok) {
        const MMC5983_Data_t *m = MMC5983_GetData();
        float mx, my, mz;
        sensor_mag_to_body(m->gauss[0], m->gauss[1], m->gauss[2], &mx, &my, &mz);
        pkt.mag_x_mg = sat_i16(mx * 1000.0f);
        pkt.mag_y_mg = sat_i16(my * 1000.0f);
        pkt.mag_z_mg = sat_i16(mz * 1000.0f);
    } else {
        pkt.mag_x_mg = pkt.mag_y_mg = pkt.mag_z_mg = 0;
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
    pkt.cpu_ekf_x10  = (uint16_t)(EKF_GetCPUUsage() * 10.0f);

    /* --- 系統旗標 --- */
    uint8_t flags = 0;
    if (HAL_GPIO_ReadPin(FIRE_GPIO_Port, FIRE_Pin) == GPIO_PIN_SET)   flags |= TELEM_FLAG_DROGUE_FIRED;
    if (__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3) >= 2000)         flags |= TELEM_FLAG_MAIN_DEPLOYED;
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
        pkt.peer_flags     = pr->flags;
        pkt.peer_h_cm      = pr->h_est_cm;
        pkt.peer_v_cms     = pr->v_est_cms;
        pkt.peer_baro_cm   = pr->baro_alt_cm;
        pkt.peer_link      = plink;
        uint32_t denom = pr->rx_count + pr->lost_count;
        pkt.peer_loss_pmil = denom ? (uint16_t)((1000UL * pr->lost_count) / denom) : 0U;
        pkt.peer_az_cg     = pr->a_z_cg;
        pkt.peer_vf_h_cm   = pr->vf_h_cm;
        pkt.peer_vf_v_cms  = pr->vf_v_cms;
        pkt.peer_bench_arb = pr->peer_main_arb;
    }
#else
    pkt.peer_fsm_state = 0U;
    pkt.peer_flags     = 0U;
    pkt.peer_h_cm      = 0;
    pkt.peer_v_cms     = 0;
    pkt.peer_baro_cm   = 0;
    pkt.peer_link      = 0U;
    pkt.peer_loss_pmil = 0U;
    pkt.peer_az_cg     = 0;
    pkt.peer_vf_h_cm   = 0;
    pkt.peer_vf_v_cms  = 0;
    pkt.peer_bench_arb = 0U;
#endif

    /* --- ARM 被擋下原因（fail-open 已在 FSM_Update 組 flash_pool_ready 時處理，這裡只回報） --- */
    pkt.arm_flags = g_arm_blocked_flash ? TELEM_ARM_BLOCKED_FLASH_POOL : 0U;

    /* --- CRC16 覆蓋除最後 2 bytes(crc16 本身) 外的全部內容 --- */
    pkt.crc16 = telem_crc16((const uint8_t *)&pkt, (uint16_t)(sizeof(pkt) - 2));

    memcpy(out, &pkt, sizeof(pkt));
    return (uint16_t)sizeof(pkt);
}
