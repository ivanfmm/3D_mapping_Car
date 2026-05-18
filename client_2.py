import socket
import cv2
import struct
import threading
import sys
import os
import time
import csv
from pathlib import Path
from datetime import datetime

import numpy as np

if os.name == 'nt':
    import msvcrt
else:
    import select
    import tty
    import termios

PI_IP = '10.14.221.32'
SAVE_FRAMES = True
SAVE_ROOT = Path('captured_frames')
MAX_FRAME_SIZE = 50 * 1024 * 1024

stop_event = threading.Event()
command_socket = None
command_lock = threading.Lock()


def recv_exact(sock, size):
    data = b''

    while len(data) < size and not stop_event.is_set():
        packet = sock.recv(size - len(data))

        if not packet:
            return None

        data += packet

    return data if len(data) == size else None


def send_car_command(cmd):
    with command_lock:
        if command_socket is None:
            return

        try:
            command_socket.sendall(cmd.encode())
        except Exception:
            pass


def request_shutdown():
    already_stopping = stop_event.is_set()
    stop_event.set()

    if not already_stopping:
        send_car_command('q')
        send_car_command('x')


def create_capture_session():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    session_dir = SAVE_ROOT / f'session_{timestamp}'
    full_dir = session_dir / 'full'
    left_dir = session_dir / 'left'
    right_dir = session_dir / 'right'

    full_dir.mkdir(parents=True, exist_ok=True)
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    csv_path = session_dir / 'frames.csv'
    csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
    writer = csv.writer(csv_file)
    writer.writerow([
        'frame_index',
        'timestamp_unix',
        'timestamp_iso',
        'width',
        'height',
        'full_frame_path',
        'left_frame_path',
        'right_frame_path',
    ])

    print(f"Guardando fotogramas en esta computadora: {session_dir.resolve()}")

    return {
        'session_dir': session_dir,
        'full_dir': full_dir,
        'left_dir': left_dir,
        'right_dir': right_dir,
        'csv_file': csv_file,
        'writer': writer,
    }


def close_capture_session(session):
    if session is None:
        return

    try:
        session['csv_file'].flush()
        session['csv_file'].close()
    except Exception:
        pass


def save_frame(session, frame_index, frame_data, frame, left, right):
    if session is None:
        return

    now = time.time()
    timestamp_iso = datetime.fromtimestamp(now).isoformat(timespec='milliseconds')

    full_path = session['full_dir'] / f'frame_{frame_index:06d}.jpg'
    left_path = session['left_dir'] / f'left_{frame_index:06d}.jpg'
    right_path = session['right_dir'] / f'right_{frame_index:06d}.jpg'

    with open(full_path, 'wb') as file:
        file.write(frame_data)

    cv2.imwrite(str(left_path), left)
    cv2.imwrite(str(right_path), right)

    h, w = frame.shape[:2]
    session['writer'].writerow([
        frame_index,
        f'{now:.6f}',
        timestamp_iso,
        w,
        h,
        str(full_path),
        str(left_path),
        str(right_path),
    ])

    if frame_index % 30 == 0:
        session['csv_file'].flush()


def read_key_windows():
    while not stop_event.is_set():
        if msvcrt.kbhit():
            key = msvcrt.getch()

            if key in {b'\x00', b'\xe0'}:
                if msvcrt.kbhit():
                    msvcrt.getch()
                continue

            return key.decode(errors='ignore').lower()

        time.sleep(0.02)

    return None


def read_key_unix():
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)

    if ready:
        return sys.stdin.read(1).lower()

    return None


def send_commands():
    global command_socket

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    old_terminal_settings = None
    fd = None

    try:
        cmd_sock.connect((PI_IP, 9999))

        with command_lock:
            command_socket = cmd_sock

        print("Conectado. WASD para mover, Q para stop, X para salir")

        if os.name != 'nt':
            fd = sys.stdin.fileno()
            old_terminal_settings = termios.tcgetattr(fd)
            tty.setraw(fd)

        while not stop_event.is_set():
            key = read_key_windows() if os.name == 'nt' else read_key_unix()

            if not key:
                continue

            if key in {'w', 'a', 's', 'd', 'q', 'x'}:
                send_car_command(key)

            if key == 'x':
                request_shutdown()
                break

    except Exception as e:
        print("Error en comandos:", e)
        request_shutdown()

    finally:
        if old_terminal_settings is not None and fd is not None:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_terminal_settings)
            except Exception:
                pass

        with command_lock:
            command_socket = None

        try:
            cmd_sock.close()
        except Exception:
            pass


def receive_video():
    vid_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    payload_size = struct.calcsize('<L')
    frame_index = 0
    session = None

    try:
        vid_sock.connect((PI_IP, 9998))
        session = create_capture_session() if SAVE_FRAMES else None

        while not stop_event.is_set():
            packed_size = recv_exact(vid_sock, payload_size)

            if packed_size is None:
                print("Conexion de video cerrada")
                break

            msg_size = struct.unpack('<L', packed_size)[0]

            if msg_size <= 0 or msg_size > MAX_FRAME_SIZE:
                print("Tamano de frame invalido")
                break

            frame_data = recv_exact(vid_sock, msg_size)

            if frame_data is None:
                print("Frame incompleto o conexion perdida")
                break

            frame_np = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

            if frame is None:
                continue

            h, w = frame.shape[:2]
            left = frame[:, :w // 2]
            right = frame[:, w // 2:]

            if SAVE_FRAMES:
                save_frame(session, frame_index, frame_data, frame, left, right)

            cv2.imshow('Ojo Izquierdo', left)
            cv2.imshow('Ojo Derecho', right)

            frame_index += 1

            if cv2.waitKey(1) & 0xFF == ord('x'):
                request_shutdown()
                break

    except Exception as e:
        print("Error en video:", e)
        request_shutdown()

    finally:
        request_shutdown()
        close_capture_session(session)
        cv2.destroyAllWindows()

        try:
            vid_sock.close()
        except Exception:
            pass

        print(f"Fotogramas guardados: {frame_index}")


if __name__ == '__main__':
    t_cmd = threading.Thread(target=send_commands, daemon=True)
    t_cmd.start()

    receive_video()
