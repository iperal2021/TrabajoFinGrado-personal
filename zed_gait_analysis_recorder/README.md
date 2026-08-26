# zed_gait_analysis_recorder

Nodo ROS 2 por cámara ZED para el análisis de marcha (TFG): ejecuta body
pose, publica la imagen anotada y graba CSV (joints 3D), vídeo MP4 anotado y
SVO2 nativo por sesión.

Especificación: `plan/PLAN_v2_zed_gait_analysis_recorder.md`.

## Requisitos (máquina objetivo)

- Jetson Orin NX, JetPack 6.0, Ubuntu 22.04
- ROS 2 Humble + `rmw_cyclonedds_cpp`
- ZED SDK 5.2.2, CUDA 12.2, OpenCV 4.12 con CUDA, `pyzed`
- 1–2 cámaras ZED 2i por USB

## Compilación

```bash
cd ~/TFG/ros_ws
colcon build --packages-select zed_gait_analysis_recorder
source install/setup.bash
```

## Ejecución

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# Autodetección (hasta 2 cámaras, namespace zed<serial>)
ros2 launch zed_gait_analysis_recorder zed_gait_analysis_recorder.launch.py

# Selección por serial
ros2 launch zed_gait_analysis_recorder zed_gait_analysis_recorder.launch.py \
    camera_serials:=12345678,87654321

# Modo simulación con un SVO
ros2 launch zed_gait_analysis_recorder zed_gait_analysis_recorder.launch.py \
    svo_path:=/ruta/archivo.svo2
```

## Uso

```bash
# Iniciar la grabación en TODAS las cámaras
ros2 topic pub --once /gait_record std_msgs/msg/Bool "{data: true}"

# Detener
ros2 topic pub --once /gait_record std_msgs/msg/Bool "{data: false}"
```

| Topic | Tipo | Descripción |
|---|---|---|
| `/gait_record` | `std_msgs/Bool` | Control global de grabación |
| `/zed<sn>/image_annotated/compressed` | `sensor_msgs/CompressedImage` | Imagen anotada (siempre publicada) |
| `/zed<sn>/image_annotated` | `sensor_msgs/Image` | Cruda `bgr8` si `publish_raw_image:=true` |
| `/zed<sn>/recording_state` | `std_msgs/Bool` | Estado de grabación (`transient_local`) |

## Archivos por sesión (en `~/Documents/ZED/`)

```text
<yyyymmdd_hhmmss>_zed<sn>_pose.csv    # joints 3D a 20 Hz
<yyyymmdd_hhmmss>_zed<sn>_pose.mp4    # vídeo anotado (H.264 NVENC)
<yyyymmdd_hhmmss>_zed<sn>_pose.svo2   # imagen 2D + profundidad nativas
```

Mientras se graba, los archivos llevan sufijo `.partial` y se renombran al
cerrar correctamente. Un `.partial` que quede indica sesión incompleta.

### Formato CSV

```text
time_s,frame_ts_ns,<joint>_x,<joint>_y,<joint>_z,<joint>_conf,...,body_id,ai_model,camera_sn
```

- `time_s`: segundos desde el inicio de la grabación.
- `frame_ts_ns`: timestamp del frame (ns), para realinear las dos cámaras.
- XYZ en **metros**, en el **frame de la cámara** correspondiente.
- Joints con `<joint>_conf < 0.5`: inválidos; su XYZ es el último valor
  válido (o 0,0,0 si nunca lo hubo). La confianza real siempre se guarda.
- Filas sin persona: joints a 0, confianzas 0 y `body_id = -1`.

## Parámetros

Ver `config/default.yaml` (comentados) y la sección 4 del PLAN v2.

## Notas para la verificación en la Jetson

- Los nombres de joints de `BODY_38`/`BODY_18` usados en la cabecera del CSV
  deben contrastarse con el orden del enum del SDK 5.2.2 en la primera
  ejecución (índices siempre correctos; los nombres son cosméticos).
- Si el pipeline NVENC no abre, el nodo cae a codificación por software con
  un aviso en el log; revisar `gst-inspect-1.0 nvv4l2h264enc`.
