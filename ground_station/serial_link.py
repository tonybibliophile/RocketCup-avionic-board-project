"""
serial_link.py — RocketCom 串口共用模組（P3）
===========================================================================
統一 test_runner.py / gui_monitor.py / ekf_visualizer.py 的 port/baud 預設、
自動偵測與開埠錯誤處理（原本三份腳本各有一套寫死預設與互異的偵測邏輯）。

GroundStation/telemetry_decoder.py 刻意不依賴本模組 —— 地面站筆電可能只
複製那一支檔案，保持自包含是功能不是重複。

用法：
    import serial_link
    ser = serial_link.open_serial()                  # 全自動
    ser = serial_link.open_serial("/dev/cu.x", 460800, timeout=0.5)
"""
import sys

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial：pip3 install -r requirements.txt")
    sys.exit(1)

DEFAULT_BAUD   = 460800                      # printf 重定向 USART2 的固定鮑率
PREFERRED_PORT = "/dev/cu.usbserial-0001"    # 實測過的 USB-TTL 橋接器（macOS）

# USB-to-UART 橋接器關鍵字（排除藍牙等虛擬埠）
_USB_KEYWORDS = ("usbserial", "usbmodem", "ch34", "cp210", "ftdi", "uart",
                 "ttyusb", "ttyacm")


def list_candidate_ports():
    """回傳所有串口裝置路徑（macOS 一律轉 cu.*，避免 tty.* 阻塞 open）。"""
    out = []
    for p in list_ports.comports():
        dev = p.device
        if sys.platform == "darwin" and "tty." in dev:
            dev = dev.replace("tty.", "cu.")
        out.append(dev)
    return out


def auto_port():
    """自動偵測：優先回傳當前實體連接的 USB CDC / USB Serial 裝置。"""
    candidates = list_candidate_ports()
    # 1. 優先匹配線上現存的 USB CDC / USB-Serial 裝置
    for dev in candidates:
        dev_low = dev.lower()
        if any(ignore in dev_low for ignore in ("bluetooth", "incoming-port", "wlan-debug", "debug-console")):
            continue
        if any(kw in dev_low for kw in _USB_KEYWORDS):
            return dev

    # 2. 次之：任何非虛擬串口
    for dev in candidates:
        dev_low = dev.lower()
        if not any(ignore in dev_low for ignore in ("bluetooth", "incoming-port", "wlan-debug", "debug-console")):
            return dev

    # 3. 備用首選
    import os
    if os.path.exists(PREFERRED_PORT):
        return PREFERRED_PORT

    return candidates[0] if candidates else None


def resolve_port(port=None):
    """port 為 None / "" / "AUTO"（不分大小寫）→ 自動偵測；否則原樣回傳。"""
    if port and str(port).upper() != "AUTO":
        return port
    return auto_port() or PREFERRED_PORT


def prompt_select_port():
    """在終端機列出所有可用串口供使用者互動式選擇。"""
    ports = list_candidate_ports()
    if not ports:
        print("[ERROR] 系統未檢測到任何可用串口！")
        return None
    print("\n" + "="*50)
    print(" 🔍 【串口選擇菜單】檢測到以下可用串口：")
    print("="*50)
    for i, dev in enumerate(ports, start=1):
        marker = " (推薦)" if "usbserial" in dev.lower() or "usbmodem" in dev.lower() else ""
        print(f"  [{i}] {dev}{marker}")
    print("="*50)
    
    if sys.stdin.isatty():
        try:
            choice = input(f"請輸入號碼 (1-{len(ports)}) 選擇串口 [預設 1]: ").strip()
            if not choice:
                return ports[0]
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                return ports[idx]
        except (ValueError, KeyboardInterrupt, EOFError):
            pass
    return ports[0]


def open_serial(port=None, baud=DEFAULT_BAUD, timeout=1.0, interactive=True):
    """解析 port 後開埠並清空輸入緩衝。若開啟失敗且在終端機下，提供互動選單讓使用者選擇。"""
    target_port = resolve_port(port)
    try:
        ser = serial.Serial(target_port, baud, timeout=timeout)
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        print(f"\n[ERROR] 無法開啟預設串口 {target_port}: {e}")
        if interactive and sys.stdin.isatty():
            selected = prompt_select_port()
            if selected and selected != target_port:
                try:
                    print(f"🔄 正在切換並開啟串口: {selected} ...")
                    ser = serial.Serial(selected, baud, timeout=timeout)
                    try:
                        ser.dtr = False
                        ser.rts = False
                    except Exception:
                        pass
                    ser.reset_input_buffer()
                    return ser
                except serial.SerialException as e2:
                    print(f"[ERROR] 開啟選擇的串口 {selected} 失敗: {e2}")
        avail = ", ".join(list_candidate_ports()) or "(無)"
        print(f"   可用串口: {avail}")
        raise
