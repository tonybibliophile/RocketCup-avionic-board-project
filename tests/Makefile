# 獨立純邏輯測試（純 host 編譯，與 firmware 共用同一份 header / .c）
#   make run    編譯並依序執行全部測試（任一失敗即中止、非零退出）
#   make clean  清除
CC      ?= cc
CFLAGS  := -I../firmware/main_flight_code/Core/Inc -Wall -Wextra -O2 -std=c11
LDLIBS  := -lm

# test_openrocket_fsm.c / test_fsm_stress_suite.c 刻意不列入 BINS：兩者是手動診斷工具
# （main() 恆 return 0，PASS/FAIL 只是印出來的文字、不反映在 exit code），不是本清單其餘
# 檔案那種 g_fail/exit(1) 把關的黃金測試。加進 `run:` 的 ./$$b || exit 1 迴圈只會製造
# 「一定綠燈」的假訊號，且前者依賴專案根目錄相對路徑讀 simulation_and_data/ 下的 CSV，
# 從 tests/ 執行會找錯檔案、靜默印錯誤後仍回傳 0。需要時手動執行：
#   cd .. && cc -Ifirmware/main_flight_code/Core/Inc -o /tmp/t_or tests/test_openrocket_fsm.c firmware/main_flight_code/Core/Src/fsm.c -lm && /tmp/t_or
BINS := test_sensor_axis test_fsm test_fsm_elevator test_ekf_guard test_sensor_health test_telemetry test_telem_rx test_gs_timesync test_gs_log test_gps test_flash_ring test_flash_ring_watchdog_flight test_flash_ring_watchdog_elevator test_link_proto test_link test_servo_arb test_lora_calc test_uplink_proto test_ack_proto test_vertical_filter test_imu_raw_log

.PHONY: run clean

run: $(BINS)
	@for b in $(BINS); do \
		echo "================ $$b ================"; \
		./$$b || exit 1; \
	done

test_sensor_axis: test_sensor_axis.c ../firmware/main_flight_code/Core/Inc/sensor_axis.h ../firmware/main_flight_code/Core/Inc/attitude_math.h
	$(CC) $(CFLAGS) -o $@ test_sensor_axis.c $(LDLIBS)

# 板上刻意停用熱啟動（board_config.h FEATURE_HOTSTART=0），但 FSM_HotStartDecide
# 的決策邏輯仍須驗證（golden test [8]），故 host 測試以 -DFEATURE_HOTSTART=1 強制開啟。
# ★必須明確 -DFLIGHT_PROFILE_ELEVATOR=0：board_config.h 的預設值會隨上板前準備哪個
# profile 而變動（電梯場測期間預設為 1），若不覆寫，這份「飛行剖面黃金測試」會悄悄
# 吃到電梯常數而非真實飛行常數，卻不會報錯——曾實際踩過（大量對不上的黃金值一度
# 誤判為邏輯改壞，實際是編譯吃錯 profile）。
test_fsm: test_fsm.c ../firmware/main_flight_code/Core/Src/fsm.c ../firmware/main_flight_code/Core/Inc/fsm.h ../firmware/main_flight_code/Core/Inc/board_config.h
	$(CC) $(CFLAGS) -DFEATURE_HOTSTART=1 -DFLIGHT_PROFILE_ELEVATOR=0 -o $@ test_fsm.c ../firmware/main_flight_code/Core/Src/fsm.c $(LDLIBS)

# 電梯場測 profile（FLIGHT_PROFILE_ELEVATOR=1）：門檻縮放 + 強制 baro 降級鏈，
# 驗證與飛行 profile 共用同一份 fsm.c 邏輯在電梯剖面下的行為（見 test_fsm_elevator.c）。
test_fsm_elevator: test_fsm_elevator.c ../firmware/main_flight_code/Core/Src/fsm.c ../firmware/main_flight_code/Core/Inc/fsm.h ../firmware/main_flight_code/Core/Inc/board_config.h
	$(CC) $(CFLAGS) -DFLIGHT_PROFILE_ELEVATOR=1 -o $@ test_fsm_elevator.c ../firmware/main_flight_code/Core/Src/fsm.c $(LDLIBS)

test_ekf_guard: test_ekf_guard.c ../firmware/main_flight_code/Core/Inc/ekf_guard.h
	$(CC) $(CFLAGS) -o $@ test_ekf_guard.c $(LDLIBS)

test_sensor_health: test_sensor_health.c ../firmware/main_flight_code/Core/Inc/sensor_health.h
	$(CC) $(CFLAGS) -o $@ test_sensor_health.c $(LDLIBS)

test_telemetry: test_telemetry.c ../firmware/main_flight_code/Core/Inc/telemetry.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_telemetry.c $(LDLIBS)

test_telem_rx: test_telem_rx.c ../firmware/main_flight_code/Core/Inc/telem_rx.h ../firmware/main_flight_code/Core/Inc/telemetry.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_telem_rx.c $(LDLIBS)

test_gs_timesync: test_gs_timesync.c ../firmware/main_flight_code/Core/Inc/gs_timesync.h
	$(CC) $(CFLAGS) -o $@ test_gs_timesync.c $(LDLIBS)

test_gs_log: test_gs_log.c ../firmware/main_flight_code/Core/Src/gs_log.c ../firmware/main_flight_code/Core/Inc/gs_log.h ../firmware/main_flight_code/Core/Inc/telemetry.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_gs_log.c ../firmware/main_flight_code/Core/Src/gs_log.c $(LDLIBS)

test_gps: test_gps.c ../firmware/main_flight_code/Core/Inc/gps_parse.h
	$(CC) $(CFLAGS) -o $@ test_gps.c $(LDLIBS)

test_flash_ring: test_flash_ring.c ../firmware/main_flight_code/Core/Inc/flash_ring_math.h
	$(CC) $(CFLAGS) -o $@ test_flash_ring.c $(LDLIBS)

# 池子是否覆蓋飛安看門狗視窗，兩種 profile 各驗一次（見 test_flash_ring_watchdog.c 頂部註解）。
test_flash_ring_watchdog_flight: test_flash_ring_watchdog.c ../firmware/main_flight_code/Core/Inc/flash_ring_math.h ../firmware/main_flight_code/Core/Inc/fsm.h ../firmware/main_flight_code/Core/Inc/board_config.h
	$(CC) $(CFLAGS) -DFLIGHT_PROFILE_ELEVATOR=0 -o $@ test_flash_ring_watchdog.c $(LDLIBS)

test_flash_ring_watchdog_elevator: test_flash_ring_watchdog.c ../firmware/main_flight_code/Core/Inc/flash_ring_math.h ../firmware/main_flight_code/Core/Inc/fsm.h ../firmware/main_flight_code/Core/Inc/board_config.h
	$(CC) $(CFLAGS) -DFLIGHT_PROFILE_ELEVATOR=1 -o $@ test_flash_ring_watchdog.c $(LDLIBS)

test_link_proto: test_link_proto.c ../firmware/main_flight_code/Core/Src/link_proto.c ../firmware/main_flight_code/Core/Inc/link_proto.h ../firmware/main_flight_code/Core/Inc/crc16.h ../firmware/main_flight_code/Core/Inc/telemetry.h
	$(CC) $(CFLAGS) -o $@ test_link_proto.c ../firmware/main_flight_code/Core/Src/link_proto.c $(LDLIBS)

test_link: test_link.c ../firmware/main_flight_code/Core/Src/link.c ../firmware/main_flight_code/Core/Src/link_proto.c ../firmware/main_flight_code/Core/Inc/link.h ../firmware/main_flight_code/Core/Inc/board_config.h
	$(CC) $(CFLAGS) -o $@ test_link.c ../firmware/main_flight_code/Core/Src/link.c ../firmware/main_flight_code/Core/Src/link_proto.c $(LDLIBS)

test_servo_arb: test_servo_arb.c ../firmware/main_flight_code/Core/Src/servo_arb.c ../firmware/main_flight_code/Core/Inc/servo_arb.h ../firmware/main_flight_code/Core/Inc/board_config.h
	$(CC) $(CFLAGS) -DFLIGHT_PROFILE_ELEVATOR=0 -o $@ test_servo_arb.c ../firmware/main_flight_code/Core/Src/servo_arb.c $(LDLIBS)

test_lora_calc: test_lora_calc.c ../firmware/main_flight_code/Core/Inc/lora_calc.h
	$(CC) $(CFLAGS) -o $@ test_lora_calc.c $(LDLIBS)

test_uplink_proto: test_uplink_proto.c ../firmware/main_flight_code/Core/Inc/uplink_proto.h ../firmware/main_flight_code/Core/Inc/uplink_text_proto.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_uplink_proto.c $(LDLIBS)

test_ack_proto: test_ack_proto.c ../firmware/main_flight_code/Core/Inc/ack_proto.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_ack_proto.c $(LDLIBS)

test_vertical_filter: test_vertical_filter.c ../firmware/main_flight_code/Core/Inc/vertical_filter.h
	$(CC) $(CFLAGS) -o $@ test_vertical_filter.c $(LDLIBS)

test_imu_raw_log: test_imu_raw_log.c ../firmware/main_flight_code/Core/Inc/imu_raw_log.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_imu_raw_log.c $(LDLIBS)

clean:
	rm -f $(BINS)
