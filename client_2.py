import socket
import cv2
import struct
import threading
import sys
import tty
import termios
import numpy as np

PI_IP = '10.14.221.32'

# ── Thread Secundario: Mandar comandos con WASD ──
def send_commands():
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_sock.connect((PI_IP, 9999))

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("¡Conectado! WASD para mover, Q para stop, X para salir")

    try:
        tty.setraw(fd)
        while True:
            key = sys.stdin.read(1).lower()
            cmd_sock.send(key.encode())
            if key == 'x':
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        cmd_sock.close()

# ── Hilo Principal: Recibir y mostrar video ──
def receive_video():
    vid_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    vid_sock.settimeout(1.0) # <--- 1. AGREGAR ESTE LÍMITE DE TIEMPO
    vid_sock.connect((PI_IP, 9998))
    data = b''
    # Usamos '<L' para forzar 4 bytes estandarizados entre la Pi y la PC
    payload_size = struct.calcsize('<L') 

    try:
        while True:
            try: # <--- 2. ENVOLVER LA LECTURA EN UN TRY
                # 1. Leer el tamaño del mensaje
                while len(data) < payload_size:
                    packet = vid_sock.recv(4096)
                    # Cambiamos break por raise para manejar la desconexión limpiamente
                    if not packet: raise ConnectionError 
                    data += packet
                
                packed_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack('<L', packed_size)[0]

                # 2. Leer los bytes de la imagen JPEG
                while len(data) < msg_size:
                    packet = vid_sock.recv(4096)
                    if not packet: raise ConnectionError
                    data += packet
                
                frame_data = data[:msg_size]
                data = data[msg_size:]

            except socket.timeout: # <--- 3. ATRAPAR EL ERROR DE RED LENTA
                print("Señal débil o lag, saltando frame para no congelar...")
                data = b'' # Limpiar los datos a medias para recibir uno nuevo
                continue 
            except ConnectionError:
                print("Conexión perdida con la Raspberry Pi.")
                break

            # 3. Decodificar el JPEG comprimido a Matriz de OpenCV
            frame_np = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

            if frame is not None:
                h, w = frame.shape[:2]
                left  = frame[:, :w//2]
                right = frame[:, w//2:]

                cv2.imshow('Ojo Izquierdo', left)
                cv2.imshow('Ojo Derecho', right)
            
            # cv2.waitKey DEBE estar en el hilo principal
            if cv2.waitKey(1) & 0xFF == ord('x'):
                break
    finally:
        cv2.destroyAllWindows()
        vid_sock.close()
        

# ── Main ──
if __name__ == '__main__':
    # Lanzamos el teclado en segundo plano
    t_cmd = threading.Thread(target=send_commands)
    t_cmd.daemon = True 
    t_cmd.start()

    # Ejecutamos el video en el hilo principal
    receive_video()