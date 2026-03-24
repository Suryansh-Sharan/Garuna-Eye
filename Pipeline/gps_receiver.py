import socket
import json
from datetime import datetime
from pynput import keyboard
import threading

# ========= CONFIG =========
UDP_PORT = 6000
BUFFER_SIZE = 2048
SAVE_FILE = "marked_locations.jsonl"

# ========= STATE =========
latest_gps = None
lock = threading.Lock()

# ========= UDP SOCKET =========
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))

print(f"📥 Listening for GPS on port {UDP_PORT}...")
print("👉 Press 'G' to save current GPS location\n")

# ========= KEYBOARD HANDLER =========
def on_press(key):
    global latest_gps
    try:
        if key.char.lower() == 'g':
            with lock:
                if latest_gps:
                    record = {
                        **latest_gps,
                        "saved_at": datetime.now().isoformat()
                    }
                    with open(SAVE_FILE, "a") as f:
                        f.write(json.dumps(record) + "\n")

                    print("📌 LOCATION SAVED:", record)
                else:
                    print("⚠️ No GPS fix yet")
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press)
listener.start()

# ========= MAIN LOOP =========
try:
    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        gps = json.loads(data.decode())

        if not gps.get("lat") or not gps.get("lon"):
            continue

        with lock:
            latest_gps = gps

        print(
            f"📍 {gps['lat']:.6f}, {gps['lon']:.6f} | "
            f"Alt: {gps.get('alt')} | Time: {gps.get('time')}"
        )

except KeyboardInterrupt:
    print("\n🛑 Receiver stopped")

finally:
    sock.close()
    listener.stop()
