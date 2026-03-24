# live_viewer.py
import cv2
import socket
import numpy as np

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 7001))

while True:
    data, _ = sock.recvfrom(65536)
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is not None:
        cv2.imshow("LIVE STREAM", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
