import cv2
import numpy as np
import socket

# Configurazione rete
UDP_IP = "0.0.0.0"   # ascolta su tutte le interfacce
UDP_PORT = 4210      # stessa porta di invio ESP32

# Indirizzo stream MJPEG ESP32 (modifica con il suo IP)
ESP32_STREAM_URL = "http://192.168.1.100:81/stream"

# Dimensione QVGA
WIDTH, HEIGHT = 320, 240

# Socket UDP non-bloccante
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)
print(f"[INFO] In ascolto UDP su {UDP_IP}:{UDP_PORT}")

# Apri stream MJPEG
cap = cv2.VideoCapture(ESP32_STREAM_URL)
if not cap.isOpened():
    raise RuntimeError(f"Impossibile aprire stream {ESP32_STREAM_URL}")

# Crea finestre ridimensionate
cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Frame", WIDTH*2, HEIGHT*2)
cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Mask", WIDTH*2, HEIGHT*2)

cx, cy = None, None

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.resize(frame, (WIDTH, HEIGHT))

    # maschera rosso in HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower1 = np.array([0, 120, 70])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 120, 70])
    upper2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)

    # prova a ricevere coordinate (non-bloccante)
    try:
        data, _ = sock.recvfrom(1024)
        s = data.decode().strip()
        x_str, y_str = s.split(',')
        cx, cy = int(round(float(x_str))), int(round(float(y_str)))
    except BlockingIOError:
        pass
    except Exception as e:
        print(f"[WARN] parsing '{data}': {e}")

    # disegna su frame e mask
    if cx is not None and cy is not None:
        cv2.circle(frame, (cx, cy), 5, (0,255,0), -1)
        cv2.circle(mask, (cx, cy), 5, 255, -1)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
sock.close()
cv2.destroyAllWindows()
