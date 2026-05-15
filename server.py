import socket
import cv2
import struct
import pickle
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
            if cmd in COMMANDS:
                car.set_motor_model(*COMMANDS[cmd])
        except:
            car.set_motor_model(0, 0, 0, 0)
            break

# ── Thread 2: transmitir video ──
def stream_video(conn):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        data = pickle.dumps(frame)
        size = struct.pack('L', len(data))
        try:
            conn.sendall(size + data)
        except:
            break
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
