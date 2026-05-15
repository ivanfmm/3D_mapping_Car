import socket
import cv2
import struct
import pickle
import threading
import sys
import tty
import termios

PI_IP = '10.14.221.32'

# ── Thread 1: recibir y mostrar video ──
def receive_video():
    vid_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    vid_sock.connect((PI_IP, 9998))
    data = b''
    payload_size = struct.calcsize('L')

    while True:
        while len(data) < payload_size:
            data += vid_sock.recv(4096)
        packed_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack('L', packed_size)[0]

        while len(data) < msg_size:
            data += vid_sock.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = pickle.loads(frame_data)
        h, w = frame.shape[:2]
        left  = frame[:, :w//2]
        right = frame[:, w//2:]

        cv2.imshow('Ojo Izquierdo', left)
        cv2.imshow('Ojo Derecho', right)
        if cv2.waitKey(1) & 0xFF == ord('x'):
            break

    cv2.destroyAllWindows()

# ── Thread 2: mandar comandos con WASD ──
def send_commands():
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_sock.connect((PI_IP, 9999))

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("Conectado! WASD para mover, Q para stop, X para salir")

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

# ── Main ──
if __name__ == '__main__':
    t1 = threading.Thread(target=receive_video)
    t2 = threading.Thread(target=send_commands)
    t1.daemon = True
    t1.start()
    t2.start()
    t2.join()