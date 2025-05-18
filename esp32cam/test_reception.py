import socket
import cv2
import numpy as np

# Parametri di rete (come prima)
UDP_IP = "0.0.0.0"
UDP_PORT = 4210

# Dimensione immagine di debug (QVGA)
WIDTH, HEIGHT = 320, 240

# Crea socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"[INFO] In ascolto UDP su {UDP_IP}:{UDP_PORT}")

# Finestra OpenCV
cv2.namedWindow("Debug Centroid", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Debug Centroid", WIDTH*2, HEIGHT*2)

# Loop di ricezione e visualizzazione
while True:
    # Ricevi dati
    data, addr = sock.recvfrom(1024)
    text = data.decode('utf-8').strip()
    
    # Prepara tela nera
    canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    
    try:
        # Estrai coordinate
        x_str, y_str = text.split(',')
        x, y = float(x_str), float(y_str)
        
        # Constrain entro limiti
        ix = max(0, min(WIDTH-1, int(round(x))))
        iy = max(0, min(HEIGHT-1, int(round(y))))
        
        # Disegna un cerchio rosso
        cv2.circle(canvas, (ix, iy), 5, (0,0,255), -1)
        cv2.putText(canvas, f"({ix},{iy})", (ix+10, iy-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    except Exception as e:
        # In caso di parse error, mostro messaggio
        cv2.putText(canvas, f"Parsing error: {text}", (10, HEIGHT//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
    
    # Visualizza
    cv2.imshow("Debug Centroid", canvas)
    
    # Esc per uscire
    if cv2.waitKey(1) & 0xFF == 27:
        break

sock.close()
cv2.destroyAllWindows()