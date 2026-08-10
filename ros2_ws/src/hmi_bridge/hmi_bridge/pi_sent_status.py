import serial, time, json

# GPIO UART on Pi 5 - GPIO 14/15
# Try ttyAMA0 instead of ttyAMA10
port = '/dev/ttyAMA0'
ser = serial.Serial(port, 115200, timeout=1)
print(f"Connected to {port} at 115200 baud")
print("Sending test data to CYD every 1 second...")
print("Press Ctrl+C to stop\n")

try:
    count = 0
    while True:
        count += 1
        status = {
            "sensors_running": True,
            "lidar_hz": 10.0,
            "imu_hz": 200.0,
            "lidar_ok": True,
            "imu_ok": True,
            "recording": False,
            "disk_gb": 45.2,
            "rec_duration": 0,
            "bag_name": ""
        }
        line = json.dumps(status) + "\n"
        ser.write(line.encode('utf-8'))
        ser.flush()
        print(f"[{count}] Sent: {line.strip()}")
        time.sleep(1.0)

        # read any incoming lines (commands from CYD)
        resp = ser.readline().decode('utf-8', errors='replace').strip()
        if resp:
            print(f"    RX from CYD: {resp}")
except KeyboardInterrupt:
    pass
finally:
    ser.close()