import socket
import cv2
import struct
import threading
import time
from motor import Ordinary_Car

car = Ordinary_Car()
stop_event = threading.Event()

COMMANDS = {
    'w': (1500, 1500, 1500, 1500),
    's': (-1500, -1500, -1500, -1500),
    'a': (-1500, -1500, 1500, 1500),
    'd': (1500, 1500, -1500, -1500),
    'q': (0, 0, 0, 0),
    'x': (0, 0, 0, 0),
}


def stop_motors():
    try:
        car.set_motor_model(0, 0, 0, 0)
    except Exception as e:
        print("No se pudieron detener los motores:", e)


def close_socket(sock):
    try:
        sock.close()
    except Exception:
        pass


def handle_commands(conn):
    conn.settimeout(0.2)

    try:
        while not stop_event.is_set():
            try:
                data = conn.recv(1)
            except socket.timeout:
                continue

            if not data:
                break

            cmd = data.decode(errors="ignore").lower()

            if cmd in COMMANDS:
                car.set_motor_model(*COMMANDS[cmd])

            if cmd == 'x':
                break

    except Exception as e:
        print("Error en comandos:", e)

    finally:
        stop_event.set()
        stop_motors()
        close_socket(conn)


def stream_video(conn):
    cap = cv2.VideoCapture(0)

    try:
        if not cap.isOpened():
            print("No se pudo abrir la camara")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

        while not stop_event.is_set():
            ret, frame = cap.read()

            if not ret:
                time.sleep(0.05)
                continue

            result, encimg = cv2.imencode('.jpg', frame, encode_param)

            if not result:
                time.sleep(0.01)
                continue

            data = encimg.tobytes()
            size = struct.pack('<L', len(data))
            conn.sendall(size + data)

    except Exception as e:
        print("Error en video:", e)

    finally:
        stop_event.set()
        cap.release()
        close_socket(conn)


def main():
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    vid_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    cmd_conn = None
    vid_conn = None

    try:
        cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        cmd_sock.bind(('0.0.0.0', 9999))
        cmd_sock.listen(1)

        vid_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        vid_sock.bind(('0.0.0.0', 9998))
        vid_sock.listen(1)

        print("Esperando conexion...")
        cmd_conn, _ = cmd_sock.accept()
        vid_conn, _ = vid_sock.accept()
        print("Cliente conectado")

        t_cmd = threading.Thread(target=handle_commands, args=(cmd_conn,))
        t_vid = threading.Thread(target=stream_video, args=(vid_conn,))

        t_cmd.start()
        t_vid.start()

        t_cmd.join()
        t_vid.join()

    except KeyboardInterrupt:
        print("Servidor detenido por teclado")

    except Exception as e:
        print("Error en servidor:", e)

    finally:
        stop_event.set()
        stop_motors()
        close_socket(cmd_conn) if cmd_conn else None
        close_socket(vid_conn) if vid_conn else None
        close_socket(cmd_sock)
        close_socket(vid_sock)
        print("Servidor cerrado")


if __name__ == '__main__':
    main()
