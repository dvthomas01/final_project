#!/usr/bin/env python3
import serial, time

PORT = "/dev/ttyTHS0"   # Jetson’s hardware UART
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.5)
print("Jetson TX/RX test – press Ctrl-C to quit")

try:
    while True:
        ser.write(b"ROTATE,1\n")            # ➊ send a line
        print("TX: ROTATE")

        resp = ser.readline().decode().strip()
        if resp:
            print("RX:", resp)           # expect “ACK HELLO”
        else:
            print("RX: (nothing)")

        time.sleep(1)                    # once per second
except KeyboardInterrupt:
    pass
finally:
    ser.close()
