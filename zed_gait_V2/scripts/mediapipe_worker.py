#!/usr/bin/env python3
"""Worker de pose MediaPipe BlazePose para zed_gait_analysis_recorder.

Proceso hijo lanzado por MediaPipeBackend (C++). Recibe frames BGR por un
socket Unix y devuelve los 33 landmarks 2D + visibility de BlazePose (ADR-018).

Protocolo (binario, little-endian):
  C++->worker : [u32 width][u32 height][i64 timestamp_ms][W*H*3 bytes BGR]
  worker->C++ : [u32 num_poses] (+ si >0: 33 x [f32 x][f32 y][f32 visibility])
  handshake   : worker->C++ [u32 MAGIC][u32 version] tras aceptar al cliente

El worker termina cuando el cliente cierra la conexion (EOF), asi nunca
queda zombie si el nodo muere.
"""

import argparse
import os
import socket
import struct

MAGIC = 0x4D504931  # "MPI1"
VERSION = 1
NUM_LANDMARKS = 33

HEADER = struct.Struct('<IIq')   # width, height, timestamp_ms
HANDSHAKE = struct.Struct('<II')  # magic, version
COUNT = struct.Struct('<I')      # num_poses
LANDMARK = struct.Struct('<fff')  # x, y, visibility


def recv_exact(conn, nbytes):
    """Lee exactamente nbytes o devuelve None si el cliente cerro (EOF)."""
    data = bytearray()
    while len(data) < nbytes:
        chunk = conn.recv(nbytes - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--socket', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--min-pose-detection-confidence', type=float,
                        default=0.5)
    parser.add_argument('--min-pose-presence-confidence', type=float,
                        default=0.5)
    parser.add_argument('--min-tracking-confidence', type=float, default=0.5)
    parser.add_argument('--gpu', action='store_true',
                        help='Delegado GPU (OpenGL ES/EGL); por defecto CPU')
    args = parser.parse_args()

    import numpy as np
    import cv2
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision

    base = mp_python.BaseOptions(
        model_asset_path=args.model,
        delegate=(mp_python.BaseOptions.Delegate.GPU if args.gpu
                  else mp_python.BaseOptions.Delegate.CPU))
    options = vision.PoseLandmarkerOptions(
        base_options=base,
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=args.min_pose_detection_confidence,
        min_pose_presence_confidence=args.min_pose_presence_confidence,
        min_tracking_confidence=args.min_tracking_confidence,
        output_segmentation_masks=False,
    )
    # El landmarker vive toda la sesion; recrearlo por frame es el error de
    # rendimiento numero uno (guia, seccion 8.6).
    landmarker = vision.PoseLandmarker.create_from_options(options)

    if os.path.exists(args.socket):
        os.unlink(args.socket)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(args.socket)
    server.listen(1)
    print(f'[mediapipe_worker] Modelo cargado. Escuchando en {args.socket}',
          flush=True)

    conn, _ = server.accept()
    conn.sendall(HANDSHAKE.pack(MAGIC, VERSION))
    print('[mediapipe_worker] Cliente conectado', flush=True)

    import time
    prof_n, prof_t = 0, [0.0, 0.0, 0.0]  # recv, conversion, inferencia

    while True:
        t0 = time.perf_counter()
        header = recv_exact(conn, HEADER.size)
        if header is None:
            break
        width, height, timestamp_ms = HEADER.unpack(header)
        frame = recv_exact(conn, width * height * 3)
        if frame is None:
            break
        t1 = time.perf_counter()

        bgr = np.frombuffer(frame, dtype=np.uint8).reshape((height, width, 3))
        # BGR -> RGB es obligatorio antes de mp.Image (guia, seccion 8.2).
        # cv2 (SIMD) tarda ~2 ms; el flip de numpy tardaba ~28 ms a 720p.
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        t2 = time.perf_counter()

        result = landmarker.detect_for_video(mp_image, timestamp_ms)
        t3 = time.perf_counter()
        prof_n += 1
        prof_t[0] += t1 - t0
        prof_t[1] += t2 - t1
        prof_t[2] += t3 - t2
        if prof_n == 60:
            print(f'[mediapipe_worker] ms/frame: recv={prof_t[0]/60*1e3:.1f} '
                  f'rgb={prof_t[1]/60*1e3:.1f} infer={prof_t[2]/60*1e3:.1f}',
                  flush=True)
            prof_n, prof_t = 0, [0.0, 0.0, 0.0]

        if not result.pose_landmarks:
            conn.sendall(COUNT.pack(0))
            continue
        landmarks = result.pose_landmarks[0]
        payload = bytearray(COUNT.pack(1))
        for i in range(NUM_LANDMARKS):
            if i < len(landmarks):
                lm = landmarks[i]
                payload += LANDMARK.pack(lm.x, lm.y, lm.visibility)
            else:
                payload += LANDMARK.pack(0.0, 0.0, 0.0)
        conn.sendall(bytes(payload))

    landmarker.close()
    conn.close()
    server.close()
    os.unlink(args.socket)
    print('[mediapipe_worker] Conexion cerrada; worker terminado', flush=True)


if __name__ == '__main__':
    main()
