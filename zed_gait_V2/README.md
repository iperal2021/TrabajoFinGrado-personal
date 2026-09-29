# zed_gait_v2

Nodo ROS 2 por cámara ZED 2i (TFG): publica la imagen 2D para la interfaz
web y graba sesiones SVO2 (estéreo crudo + IMU) por paciente en
`~/Documents/patient_records/<id>/`. Sin body tracking en línea: el
procesado del SVO a CSV de joints es la fase 2 (offline, sin tiempo real).

Derivado de `zed_gait_analysis_recorder`. Los backends de pose
(`src/backends/`, `include/.../backends/`, `recording_session.*`,
`scripts/mediapipe_worker.py`, `models/`) se conservan **sin compilar** como
material para la fase 2.

## Requisitos (máquina objetivo)

- Jetson Orin NX, JetPack 6.0, Ubuntu 22.04
- ROS 2 Humble + `rmw_cyclonedds_cpp`
- ZED SDK 5.2.2, CUDA, `pyzed`
- 1–2 cámaras ZED 2i por USB

## Compilación

```bash
cd zed_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select zed_gait_v2
source install/setup.bash
```

## Ejecución

```bash
# Autodetección (hasta 2 cámaras, namespace zed<serial>)
ros2 launch zed_gait_v2 zed_gait_v2.launch.py

# Selección por serial
ros2 launch zed_gait_v2 zed_gait_v2.launch.py camera_serials:=12345678

# Modo simulación con un SVO (pruebas sin cámara)
ros2 launch zed_gait_v2 zed_gait_v2.launch.py svo_path:=/ruta/archivo.svo2
```

## Interfaz web

`scripts/start.sh` arranca rosbridge (:9090) y un `http.server` (:8080) que
sirve `scripts/ros2_gait_monitor.html`. No arranca el nodo de cámara.

## Flujo de sesión

1. La interfaz publica `/gait_session_patient_id` (String) y
   `/gait_session_phase` (String, PRE/POST), y después `/gait_record`
   (Bool, true). Sin `patient_id` cacheado el nodo no arranca la grabación.
2. Cada cámara graba `<id>_<FASE>_<stamp>.svo2` en
   `~/Documents/patient_records/<id>/` (H265, fallback H264) hasta
   `/gait_record` = false. El SVO2 lleva estéreo crudo + IMU; la profundidad
   se recalcula al reproducirlo (en este nodo está desactivada).
3. Al parar, la interfaz publica `/gait_session_notes` (String) y
   `/gait_session_accept` (Bool): `true` escribe el `.txt` de metadatos
   junto al SVO2; `false` borra el `.svo2`.

## Topics

| Topic | Tipo | Descripción |
|---|---|---|
| `/gait_record` | `std_msgs/Bool` | Control global de grabación (todas las cámaras) |
| `/gait_session_patient_id` | `std_msgs/String` | ID de paciente (cacheado) |
| `/gait_session_phase` | `std_msgs/String` | PRE / POST (cacheado) |
| `/gait_session_notes` | `std_msgs/String` | Notas del fisio (cacheado) |
| `/gait_session_accept` | `std_msgs/Bool` | true = conservar + escribir .txt; false = borrar .svo2 |
| `/zed<sn>/image_annotated/compressed` | `sensor_msgs/CompressedImage` | Preview JPEG (q=50; solo si hay suscriptores) |
| `/zed<sn>/recording_state` | `std_msgs/Bool` | Estado de grabación (`transient_local`) |

## Archivos por sesión (en `~/Documents/patient_records/<id>/`)

```text
<id>_<PRE|POST>_<yyyymmdd_hhmmss>.svo2   # estéreo crudo + IMU
<id>_<PRE|POST>_<yyyymmdd_hhmmss>.txt    # metadatos (id, fase, notas, fecha,
                                         #  serial, resolución@fps, duración)
```

## Parámetros

Ver `config/default.yaml` (comentados).
