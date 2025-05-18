import socket

# Parametri di ascolto
UDP_IP = "0.0.0.0"    # ascolta su tutte le interfacce di rete
UDP_PORT = 4210      # deve corrispondere a pc_port nello sketch ESP32

# Crea socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"In ascolto su UDP {UDP_IP}:{UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)   # buffer fino a 1 024 byte
    text = data.decode('utf-8').strip()
    try:
        x_str, y_str = text.split(',')
        x = float(x_str)
        y = float(y_str)
        print(f"Ricevuto da {addr[0]}:{addr[1]} → x={x:.2f}, y={y:.2f}")
    except Exception as e:
        print(f"Dati non validi '{text}' ({e})")
