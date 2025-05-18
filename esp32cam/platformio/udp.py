import socket
import cv2
import numpy as np

# UDP configuration
UDP_IP = "192.168.0.105"  # PC's IP address (update if different)
UDP_PORT = 4210           # Matches ESP32-CAM
BUFFER_SIZE = 65535       # Max UDP packet size

# Frame dimensions (QVGA)
WIDTH, HEIGHT = 320, 240

# Create UDP socket
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[INFO] Listening for UDP packets on {UDP_IP}:{UDP_PORT}")
except Exception as e:
    print(f"[ERROR] Failed to bind socket: {e}")
    exit(1)

# Buffer for frame data
frame_buffer = bytearray()
frame_count = 0
cx, cy = None, None

# Create windows
cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Frame", WIDTH * 2, HEIGHT * 2)
cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Mask", WIDTH * 2, HEIGHT * 2)

while True:
    try:
        # Receive UDP packet
        data, addr = sock.recvfrom(BUFFER_SIZE)
        frame_buffer.extend(data)
        print(f"[INFO] Received {len(data)} bytes from {addr}")

        # Look for JPEG markers (0xFF 0xD8 = start, 0xFF 0xD9 = end)
        if len(frame_buffer) > 1000:
            start_idx = frame_buffer.find(b'\xFF\xD8')
            end_idx = frame_buffer.find(b'\xFF\xD9', start_idx + 2)

            if start_idx != -1 and end_idx != -1:
                # Extract complete JPEG frame
                jpeg_data = frame_buffer[start_idx:end_idx + 2]
                frame_buffer = frame_buffer[end_idx + 2:]  # Keep remaining data

                # Decode JPEG
                img_array = np.frombuffer(jpeg_data, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                if frame is not None:
                    # Resize to QVGA (if needed)
                    frame = cv2.resize(frame, (WIDTH, HEIGHT))

                    # Red color detection in HSV
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    lower1 = np.array([0, 120, 70])
                    upper1 = np.array([10, 255, 255])
                    lower2 = np.array([170, 120, 70])
                    upper2 = np.array([180, 255, 255])
                    mask1 = cv2.inRange(hsv, lower1, upper1)
                    mask2 = cv2.inRange(hsv, lower2, upper2)
                    mask = cv2.bitwise_or(mask1, mask2)

                    # Draw coordinates (if received)
                    if cx is not None and cy is not None:
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                        cv2.circle(mask, (cx, cy), 5, 255, -1)

                    # Display frames
                    cv2.imshow("Frame", frame)
                    cv2.imshow("Mask", mask)

                    # Save frame for debugging
                    frame_count += 1
                    cv2.imwrite(f"frame_{frame_count}.jpg", frame)
                    print(f"[INFO] Saved frame_{frame_count}.jpg")

                else:
                    print("[WARN] Failed to decode JPEG frame")

    except socket.timeout:
        print("[WARN] Socket timeout, waiting for data...")
        continue
    except BlockingIOError:
        # No data available yet
        continue
    except Exception as e:
        print(f"[ERROR] Error processing packet: {e}")
        continue

    # Exit on ESC key
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Cleanup
sock.close()
cv2.destroyAllWindows()
print("[INFO] Receiver stopped")