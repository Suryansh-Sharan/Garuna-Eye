import socket
import json
import os
from datetime import datetime
import keyboard   # pip install keyboard

# ================= CONFIG =================
UDP_PORT = 6000
WAYPOINT_DIR = "waypoints"
DEFAULT_ALT = 10  # meters
# ==========================================

os.makedirs(WAYPOINT_DIR, exist_ok=True)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.setblocking(False)

latest_gps = None
survivor_count = 1

print("📡 GPS receiver running")
print("🟢 Press 'G' to save survivor waypoint")
print("🔴 Ctrl+C to exit")

def save_waypoint(lat, lon, alt):
    global survivor_count

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"survivor{survivor_count}_{timestamp}.waypoints"
    path = os.path.join(WAYPOINT_DIR, filename)

    with open(path, "w") as f:
        f.write("QGC WPL 110\n")
        f.write(
            f"0\t1\t0\t16\t0\t0\t0\t0\t"
            f"{lat}\t{lon}\t{alt}\t1\n"
        )

    print(f"✅ Waypoint saved: {filename}")
    survivor_count += 1

try:
    while True:
        # --- Receive GPS ---
        try:
            data, _ = sock.recvfrom(1024)
            latest_gps = json.loads(data.decode())
        except BlockingIOError:
            pass

        # --- Key press ---
        if keyboard.is_pressed("g"):
            if latest_gps:
                save_waypoint(
                    latest_gps["lat"],
                    latest_gps["lon"],
                    DEFAULT_ALT
                )
                keyboard.wait("g")  # prevent multiple saves
            else:
                print("⚠️ No GPS fix yet")

except KeyboardInterrupt:
    print("\n🛑 Exiting")
