import cv2
import requests
import numpy as np
import time

CAMERA_IP = "192.168.1.78"
STREAM_URL = f"http://{CAMERA_IP}:81/stream"
BOUNDARY = b"--123456789000000000000987654321"

def detect_red_object(frame):
    # Converti l'immagine da BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Definisci l'intervallo di colore rosso in HSV
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    # Crea maschere per entrambi gli intervalli di rosso
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Applica operazioni morfologiche per ridurre il rumore
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    
    # Crea un'immagine colorata per la maschera (rosso)
    mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    mask_colored[np.where((mask_colored == [255, 255, 255]).all(axis=2))] = [0, 0, 255]  # Colore rosso
    
    # Sovrappone la maschera all'immagine originale
    frame_with_mask = cv2.addWeighted(frame, 1.0, mask_colored, 0.5, 0.0)
    
    # Trova i contorni nella maschera
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Disegna i contorni e calcola il centro dell'oggetto rosso più grande
    largest_area = 0
    largest_contour = None
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > largest_area and area > 100:  # Ignora contorni troppo piccoli
            largest_area = area
            largest_contour = contour
    
    if largest_contour is not None:
        # Calcola il bounding box e il centro
        x, y, w, h = cv2.boundingRect(largest_contour)
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Disegna il rettangolo e il centro sull'immagine
        cv2.rectangle(frame_with_mask, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.circle(frame_with_mask, (center_x, center_y), 5, (0, 255, 0), -1)
        cv2.putText(frame_with_mask, f"Centro: ({center_x}, {center_y})", 
                   (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Mostra la maschera in una finestra separata (opzionale)
    cv2.imshow('Maschera Rosso', mask)
    
    return frame_with_mask

def read_camera_stream():
    print(f"Connessione a {STREAM_URL}")
    try:
        stream = requests.get(STREAM_URL, stream=True, timeout=10)
        
        if stream.status_code != 200:
            print(f"Errore: Impossibile connettersi (codice HTTP {stream.status_code})")
            return
        
        bytes_data = bytearray()
        frame_count = 0
        start_time = time.time()
        
        for chunk in stream.iter_content(chunk_size=1024):
            bytes_data.extend(chunk)
            
            # Cerca il boundary per identificare l'inizio di un nuovo frame
            boundary_pos = bytes_data.find(BOUNDARY)
            if boundary_pos != -1:
                # Cerca l'inizio del frame JPEG dopo il boundary
                start_pos = bytes_data.find(b'\xff\xd8', boundary_pos)
                end_pos = bytes_data.find(b'\xff\xd9', start_pos)
                
                if start_pos != -1 and end_pos != -1 and end_pos > start_pos:
                    # Estrai il frame JPEG
                    jpg = bytes_data[start_pos:end_pos + 2]
                    bytes_data = bytes_data[end_pos + 2:]  # Rimuovi il frame processato
                    
                    # Decodifica il frame
                    img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    
                    if img is not None:
                        # Rileva oggetto rosso e applica la maschera
                        img = detect_red_object(img)
                        
                        # Mostra l'immagine con la maschera sovrapposta
                        cv2.imshow('ESP32-CAM Stream', img)
                        frame_count += 1
                        
                        # Calcola FPS
                        elapsed_time = time.time() - start_time
                        if elapsed_time > 1:
                            fps = frame_count / elapsed_time
                            print(f"FPS: {fps:.2f}")
                            frame_count = 0
                            start_time = time.time()
                    else:
                        print("Frame non valido")
                    
                    # Premi 'q' per uscire
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                elif len(bytes_data) > 100000:
                    print("Buffer troppo grande, resetto")
                    bytes_data = bytearray()
        
    except requests.exceptions.RequestException as e:
        print(f"Errore di connessione: {e}")
    except Exception as e:
        print(f"Errore durante l'elaborazione: {e}")
    finally:
        stream.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        read_camera_stream()
    except Exception as e:
        print(f"Errore generale: {e}")
        cv2.destroyAllWindows()