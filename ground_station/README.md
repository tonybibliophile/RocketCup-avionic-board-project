# Ground Station — 地面站主機軟體

> **本資料夾只負責存放地面站「主機端軟體」（在 PC 上執行的 Python 工具）。**
> 硬體與韌體不放這裡：
> - 地面站硬體（KiCad PCB / footprint）→ [`../hardware/ground_station/`](../hardware/ground_station/)
> - 地面站 CubeMX 韌體設定 → [`../firmware/ground_station_mx/`](../firmware/ground_station_mx/)

## 內容

| 項目 | 說明 |
| :--- | :--- |
| `serial_link.py` | 串口連線與收發核心 |
| `telemetry_decoder.py` | 遙測封包解碼 |
| `gui/` | 上位機 GUI：`ground_gui.py`、`GUI_avionic.py`、`gui_monitor.py`、`requirements.txt` |
| `tools/` | 分析工具：`ekf_visualizer.py`、`realtime_charts.py`、`test_runner.py` |
| `logs/` | 執行時串口紀錄輸出（runtime 產物） |

## 使用

```bash
pip install -r gui/requirements.txt
python serial_link.py         # 命令列串口工具
python gui/ground_gui.py      # 地面站 GUI：接 ROLE_GROUND 板，雙鏈路(433/920)收報統計、
                               # RSSI/SNR 分頻率繪圖、火箭/地面站 GPS 分開顯示、主副航電狀態
python gui/GUI_avionic.py     # 航電板直連 GUI：USB-TTL 直接接航電板除錯輸出用
```
