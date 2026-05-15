import socket
import cv2
import struct
import threading
from motor import Ordinary_Car

# Control de motores
car = Ordinary_Car()

COMMANDS = {
    'w': (1500, 1500, 1500, 1500),   # adelante
    's': (-1500, -1500, -1500, -1500), # atras
    'a': (-1500, -1500, 1500, 1500),  # izquierda
    'd': (1500, 1500, -1500, -1500),  # derecha
    'q': (0, 0, 0, 0),               # stop
}

# ── Thread 1: recibir comandos de teclado ──
def handle_commands(conn):
    while True:
        try:
            cmd = conn.recv(1).decode()
            if not cmd:
                break # Si se corta la conexión, salir
                
            if cmd in COMMANDS:
                car.set_motor_model(*COMMANDS[cmd])
        except:
            # Apagar motores si hay error de red
            car.set_motor_model(0, 0, 0, 0)
            break

# ── Thread 2: transmitir video comprimido ──
def stream_video(conn):
    # Nota: Si tu cámara USB estéreo no es el índice 0, cámbialo a 1 o 2.
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    # Parámetro de compresión JPEG (0 a 100). 80 mantiene gran calidad y poco peso.
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

    while True:
        ret, frame = cap.read()
        if not ret:
            # Si falla la captura de un frame, ignóralo y sigue intentando
            continue 

        # 1. Comprimir el frame a formato JPEG al vuelo
        result, encimg = cv2.imencode('.jpg', frame, encode_param)
        if not result:
            continue
            
        # 2. Convertir la imagen comprimida a una cadena de bytes
        data = encimg.tobytes()
        
        # 3. Empaquetar el tamaño usando '<L' (Little-endian, igual que la PC)
        size = struct.pack('<L', len(data))
        
        try:
            # 4. Enviar el tamaño seguido de los datos de la imagen
            conn.sendall(size + data)
        except:
            break # Si el cliente se desconecta, salir del bucle
            
    cap.release()

# ── Main: esperar conexiones ──
def main():
    # Socket de comandos (puerto 9999)
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    cmd_sock.bind(('0.0.0.0', 9999))
    cmd_sock.listen(1)

    # Socket de video (puerto 9998)
    vid_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    vid_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    vid_sock.bind(('0.0.0.0', 9998))
    vid_sock.listen(1)

    print("Esperando conexion...")
    cmd_conn, _ = cmd_sock.accept()
    vid_conn, _ = vid_sock.accept()
    print("Cliente conectado!")

    t1 = threading.Thread(target=handle_commands, args=(cmd_conn,))
    t2 = threading.Thread(target=stream_video, args=(vid_conn,))
    
    t1.daemon = True
    t2.daemon = True
    
    t1.start()
    t2.start()
    
    t1.join()
    t2.join()

if __name__ == '__main__':
    main()