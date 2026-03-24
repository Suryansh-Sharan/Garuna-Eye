import serial
import pynmea2
import socket
import json
import time

# ========== CONFIG ==========
SERIAL_PORT = "/dev/serial0"
BAUDRATE = 9600

LAPTOP_TAILSCALE_IP = "100.115.66.25"   # <-- PUT YOUR LAPTOP TAILSCALE IP
UDP_PORT = 6000
# ============================

# Serial GPS
gps = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)

# UDP socket (SENDER → no bind needed)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("📡 GPS UDP sender running...")

while True:
    try:
        line = gps.readline().decode("ascii", errors="ignore")

        if line.startswith("$GPGGA") or line.startswith("$GPRMC"):
            msg = pynmea2.parse(line)

            if hasattr(msg, "latitude") and msg.latitude != 0.0:
                payload = {
                    "lat": msg.latitude,
                    "lon": msg.longitude,
                    "alt": getattr(msg, "altitude", None),
                    "ts": time.time()
                }

                sock.sendto(
                    json.dumps(payload).encode(),
                    (LAPTOP_TAILSCALE_IP, UDP_PORT)
                )

                print("➡️ Sent:", payload)

    except pynmea2.ParseError:
        pass
    except Exception as e:
        print("❌ Error:", e)
