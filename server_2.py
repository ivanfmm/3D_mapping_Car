import socket
import cv2
import struct
import threading
import time
from motor import Ordinary_Car

car = Ordinary_Car()
stop_event = threading.Event()

WIDTH, HEIGHT = 2560, 720  # frame completo: 2 lentes de 1280x720 lado a lado
FPS = 30
WATCHDOG_S = 1.0  # segundos sin recibir nada del cliente -> frenar

# Velocidades (PWM de 0 a 4095). Ajustar aqui si se siente lento o brusco.
SPEED_FWD  = 600    # avanzar / retroceder
SPEED_TURN = 900    # girar sobre su eje (antes 1500: daba vueltas como loco)
HOLD_S     = 0.6    # segundos que dura el movimiento tras la ultima tecla recibida

COMMANDS = {
    'w': ( SPEED_FWD,   SPEED_FWD,   SPEED_FWD,   SPEED_FWD),
    's': (-SPEED_FWD,  -SPEED_FWD,  -SPEED_FWD,  -SPEED_FWD),
    'a': (-SPEED_TURN, -SPEED_TURN,  SPEED_TURN,  SPEED_TURN),
    'd': ( SPEED_TURN,  SPEED_TURN, -SPEED_TURN, -SPEED_TURN),
    'q': (0, 0, 0, 0),
    'x': (0, 0, 0, 0),
}
MOVE_KEYS = {'w', 'a', 's', 'd'}


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
    """
    Modo "mantener presionado":
      - Al dejar apretada una tecla, el teclado la repite (~30 veces/s)
        y cada repeticion renueva el movimiento.
      - Si pasan HOLD_S sin recibir la tecla (la soltaste), se frena solo.
    Ademas, watchdog: si no llega nada (ni el latido 'h') en WATCHDOG_S, frena.
    """
    conn.settimeout(0.05)
    last_rx   = time.monotonic()
    last_move = 0.0
    current   = 'q'          # comando que tienen ahorita los motores
    watchdog_stopped = False

    def apply(cmd):
        # solo escribir al PCA9685 si el comando cambia (evita 30 escrituras/s)
        nonlocal current
        if cmd != current:
            car.set_motor_model(*COMMANDS[cmd])
            current = cmd

    try:
        while not stop_event.is_set():
            now = time.monotonic()

            # se solto la tecla -> frenar
            if current in MOVE_KEYS and now - last_move > HOLD_S:
                apply('q')

            # watchdog: el cliente se callo (PC congelada, wifi caido)
            if not watchdog_stopped and now - last_rx > WATCHDOG_S:
                print("Watchdog: sin senal del cliente, frenando motores")
                stop_motors()
                current = 'q'
                watchdog_stopped = True

            try:
                data = conn.recv(1)
            except socket.timeout:
                continue

            if not data:
                break

            last_rx = time.monotonic()
            watchdog_stopped = False
            cmd = data.decode(errors="ignore").lower()

            if cmd in MOVE_KEYS:
                last_move = last_rx
                apply(cmd)
            elif cmd in COMMANDS:   # 'q' o 'x'
                apply(cmd)

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

        # MJPG antes de la resolucion, igual que en la prueba que dio 2560x720
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        print(f"Camara: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")


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
