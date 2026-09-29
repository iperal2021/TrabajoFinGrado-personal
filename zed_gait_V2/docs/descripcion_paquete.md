# zed_gait_v2 — qué hace el paquete

Paquete ROS 2 (C++) del TFG para captura de sesiones de marcha con cámaras
ZED 2i. Un proceso `gait_recorder_node` por cámara: publica la imagen 2D
para la interfaz web y, bajo demanda, graba SVO2 (estéreo crudo + IMU) por
paciente en `~/Documents/patient_records/<id>/`. Máquina objetivo: Jetson
Orin NX.

Derivado de `zed_gait_analysis_recorder`. Los backends de pose
(`src/backends/`, `include/.../backends/`, `recording_session.*`,
`scripts/mediapipe_worker.py`, `models/`) se conservan **sin compilar**:
son material para la fase 2 (procesado offline del SVO a CSV de joints, sin
necesidad de tiempo real).

## Flujo por frame

```
sl::Camera::grab() ──> retrieveImage(LEFT) ──> BGR ──> JPEG (q=50)
     │                                                  │
     │                                    /<alias>/image_annotated/compressed
     └─ grab() bloqueante: cola implícita de 1, siempre el frame más reciente
```

- **Un nodo por cámara**: namespace `zed<serial>`; el launch autodetecta
  hasta 2 cámaras.
- **Depth desactivada en runtime** (`DEPTH_MODE::NONE`, `enable_depth=false`):
  el SVO2 graba estéreo crudo + IMU/mag/baro igualmente y la profundidad se
  recalcula al reproducir; cero coste de GPU en el recorder.
- **JPEG solo si hay suscriptores**: `imencode` es lo caro del preview.

## Flujo de sesión (topics fijos de `scripts/ros2_gait_monitor.html`)

1. La interfaz pide el ID de paciente y la fase (PRE/POST), publica
   `/gait_session_patient_id` + `/gait_session_phase` (String, el nodo los
   cachea) y después `/gait_record` = true (Bool).
2. `startRecording()`: exige `patient_id` cacheado; sanitiza el id
   (`[^A-Za-z0-9_-]` → `_`), crea `~/Documents/patient_records/<id>/` y
   graba `<id>_<FASE>_<stamp>.svo2` (H265, fallback H264; chequeo de espacio
   libre mínimo). Cada cámara graba su propio SVO2 con el mismo id/fase.
3. `/gait_record` = false → `stopRecording()`: cierra el SVO2 y la sesión
   queda **pendiente de aceptar**.
4. La interfaz publica `/gait_session_notes` (String) y
   `/gait_session_accept` (Bool):
   - `true` → se escribe `<mismo_base>.txt` (id, fase, notas, fecha, serial,
     resolución@fps, duración).
   - `false` → se borra el `.svo2`.
   - Si nunca llega el accept, el `.svo2` se conserva sin `.txt`.

El estado lo reporta cada nodo en `/<alias>/recording_state`
(`transient_local`): la interfaz lo usa para el badge REC.

## Lanzamiento

```bash
ros2 launch zed_gait_v2 zed_gait_v2.launch.py \
  [camera_serials:=sn1,sn2] [svo_path:=file.svo2] [svo_loop:=true] \
  [config_file:=...]
```

- Sin `svo_path`: autodetecta hasta 2 cámaras (con reintentos de apertura si
  falta alguna). Con `svo_path`: un único nodo `zed_svo` en modo simulación;
  por defecto termina al final del SVO, `svo_loop:=true` repite en bucle.
- El launch fija `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` y `ROS_DOMAIN_ID=0`:
  no hace falta exportar nada en el shell.
