#!/usr/bin/env python3
"""Prueba del worker MediaPipe con webcam, sin ROS ni ZED.

Lanza mediapipe_worker.py como subproceso y habla su protocolo: envia los
frames BGR de la webcam, dibuja los 33 landmarks y las conexiones de
BlazePose, y mide los FPS extremo a extremo (captura + socket + inferencia).

Uso (con el venv del proyecto):
  ~/.virtualenvs/zed_dev/bin/python3 scripts/test_mediapipe_worker.py \
      --model models/pose_landmarker_full.task [--camera 0] [--no-show] \
      [--duration 15]
"""

import argparse
import os
import socket
import struct
import subprocess
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mediapipe_worker import (COUNT, HANDSHAKE, HEADER, LANDMARK, MAGIC,
                              NUM_LANDMARKS, recv_exact)

# Conexiones de BlazePose (verificadas contra el enum oficial
# PoseLandmarksConnections.POSE_LANDMARKS de mediapipe).
BONES = [
    (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8), (9, 10),
    (11, 12), (11, 13), (13, 15), (15, 17), (15, 19), (15, 21), (17, 19),
    (12, 14), (14, 16), (16, 18), (16, 20), (16, 22), (18, 20),
    (11, 23), (12, 24), (23, 24), (23, 25), (24, 26), (25, 27), (26, 28),
    (27, 29), (28, 30), (29, 31), (30, 32), (27, 31), (28, 32),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--confidence', type=float, default=0.5)
    parser.add_argument('--no-show', action='store_true',
                        help='Sin ventana (modo headless)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Segundos de prueba; 0 = hasta pulsar q/ESC')
    args = parser.parse_args()

    socket_path = f'/tmp/test_mp_{os.getpid()}.sock'
    worker = subprocess.Popen([
        sys.executable,
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     'mediapipe_worker.py'),
        '--socket', socket_path,
        '--model', args.model,
        '--min-pose-detection-confidence', str(args.confidence),
        '--min-pose-presence-confidence', str(args.confidence),
        '--min-tracking-confidence', str(args.confidence),
    ])
    try:
        # Esperar a que el worker cargue el modelo y acepte la conexion.
        conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        deadline = time.monotonic() + 30.0
        while True:
            try:
                conn.connect(socket_path)
                break
            except (FileNotFoundError, ConnectionRefusedError):
                if time.monotonic() > deadline or worker.poll() is not None:
                    raise RuntimeError(
                        'El worker no arranco (rc=%s)' % worker.poll())
                time.sleep(0.2)
        magic, version = HANDSHAKE.unpack(recv_exact(conn, HANDSHAKE.size))
        assert magic == MAGIC, f'Handshake inesperado: {magic:#x}'
        print(f'[test] Worker listo (protocolo v{version})')

        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            raise RuntimeError(f'No se pudo abrir la camara {args.camera}')

        frames, t0 = 0, time.monotonic()
        t_start = t0
        print('[test] Pulsa q o ESC para salir')
        while True:
            if args.duration > 0 and time.monotonic() - t_start > \
                    args.duration:
                break
            ok, frame = cap.read()
            if not ok:
                continue
            height, width = frame.shape[:2]
            timestamp_ms = int(time.monotonic() * 1000)
            conn.sendall(HEADER.pack(width, height, timestamp_ms))
            conn.sendall(frame.tobytes())

            count_data = recv_exact(conn, COUNT.size)
            num_poses = COUNT.unpack(count_data)[0]
            if num_poses > 0:
                payload = recv_exact(conn, NUM_LANDMARKS * LANDMARK.size)
                for i in range(NUM_LANDMARKS):
                    x, y, vis = LANDMARK.unpack_from(payload,
                                                     i * LANDMARK.size)
                    if vis >= args.confidence:
                        cv2.circle(frame, (int(x * width), int(y * height)),
                                   3, (0, 255, 0), -1)
                lms = [LANDMARK.unpack_from(payload, i * LANDMARK.size)
                       for i in range(NUM_LANDMARKS)]
                for a, b in BONES:
                    if (lms[a][2] >= args.confidence
                            and lms[b][2] >= args.confidence):
                        cv2.line(frame,
                                 (int(lms[a][0] * width),
                                  int(lms[a][1] * height)),
                                 (int(lms[b][0] * width),
                                  int(lms[b][1] * height)),
                                 (0, 255, 255), 2)

            frames += 1
            elapsed = time.monotonic() - t0
            if elapsed >= 2.0:
                print(f'[test] {frames / elapsed:.1f} FPS '
                      f'({width}x{height})')
                frames, t0 = 0, time.monotonic()

            if not args.no_show:
                cv2.imshow('test_mediapipe_worker', frame)
                if cv2.waitKey(1) in (ord('q'), 27):
                    break
    finally:
        cap.release()
        if not args.no_show:
            cv2.destroyAllWindows()
        try:
            conn.close()
        except Exception:
            pass
        worker.wait(timeout=10)
        print(f'[test] Worker terminado (rc={worker.returncode})')


if __name__ == '__main__':
    main()
