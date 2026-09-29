# Cómo añadir un backend de pose

El paquete soporta varios backends de body pose sin duplicar captura,
dibujo ni grabación (ADR-001/018). Hoy existen dos backends reales:
`zed_sdk` (C++ puro) y `mediapipe` (worker Python por socket). Este
documento explica cómo añadir un tercero (p. ej. OpenPose, YOLO, RTMPose).

## Contrato `PoseBackend`

Un backend es una clase que implementa
`include/zed_gait_analysis_recorder/pose_backend.hpp`:

```cpp
bool init(sl::Camera & camera, const BackendConfig & config, std::string & error);
PoseResult infer(sl::Camera & camera, const cv::Mat & bgr);
const SkeletonTopology & topology() const;
std::string modelId() const;
```

Reglas del contrato:

1. **`init` se llama una vez**, con la cámara ya abierta. Aquí se cargan
   modelos, se lanzan procesos y se cachean intrínsecos. Devuelve `false` +
   mensaje en `error` si no puede arrancar (el nodo fallará al iniciarse).
2. **`infer` se llama tras cada `grab()` con éxito**, con la imagen
   izquierda rectificada en BGR (la misma que el nodo dibuja y publica).
   Todo lo que el backend necesite de la cámara (p. ej. `retrieveMeasure`)
   sale del mismo `grab`: nunca mezcles frames.
3. **El `PoseResult` devuelve personas en coordenadas de cámara, metros**:
   por joint, `x/y/z` (frame de cámara), `u/v` (píxel), `confidence` en
   `[0,1]` y `valid` (false si confianza < umbral o XYZ no disponible).
   `frame_ts_ns` = timestamp del frame (`getTimestamp(TIME_REFERENCE::IMAGE)`).
   Sin detecciones: `persons` vacío (el nodo escribe la fila `body_id=-1`).
4. **`topology()`**: nombres de joint (cabecera del CSV) y pares de huesos
   (dibujo). Cada backend lleva su propio mapa de joints (resp. 107).
5. **`modelId()`**: identificador corto para la columna `ai_model` del CSV
   (p. ej. `mediapipe_full`).

El nodo se encarga de TODO lo demás: selección de persona principal,
política de joints inválidos (último válido), dibujo, publicación y
grabación (CSV 20 Hz, MP4, SVO2). Un backend no debe tocar topics ni
ficheros.

## Alta en el sistema

1. `include/.../backends/<nombre>_backend.hpp` y
   `src/backends/<nombre>_backend.cpp`.
2. Rama nueva en `createBackend()` (`src/gait_recorder_node.cpp`) y
   `#include` correspondiente.
3. Fuente en `CMakeLists.txt` (`add_executable`).
4. Parámetros propios en `config/default.yaml` con prefijo
   `<backend>_`, declarados en el nodo y pasados por `BackendConfig`
   (añade campos solo si son realmente necesarios).
5. Documenta en `README.md` cómo instalarlo y lanzarlo.

## Patrón de referencia: backend con runtime Python

`mediapipe_backend.cpp` + `scripts/mediapipe_worker.py` son la referencia
para modelos cuyo runtime serio es Python:

- El backend C++ lanza el worker con `posix_spawnp` (seguro con hilos DDS;
  el worker hereda stdout/stderr y sus errores aparecen en el log del nodo).
- Socket Unix `/tmp/zg_mp_<pid>.sock`, protocolo binario length-prefixed:
  cabecera de frame `[u32 w][u32 h][i64 ts_ms]` + BGR crudo; respuesta
  `[u32 num_poses]` + landmarks `f32`.
- Handshake inicial con magic+versión para saber que el modelo cargó.
- El worker termina al recibir EOF: si el nodo muere, no quedan zombies.
- Envío con `MSG_NOSIGNAL`: un worker muerto no tumba el nodo con SIGPIPE.
- La proyección 2D→3D (`depthAt`: mediana 5×5 sobre `MEASURE::DEPTH` +
  intrínsecos cacheados) vive en el backend C++, no en el worker: el Python
  solo hace inferencia.

## Checklist de verificación

1. `colcon build --symlink-install --packages-select zed_gait_analysis_recorder`.
2. Si hay worker Python, probarlo standalone con
   `scripts/test_<backend>_worker.py` (webcam) antes de ROS.
3. Con SVO: `ros2 launch ... svo_path:=<file> pose_backend:=<nombre>` y
   comprobar topic, FPS y una sesión de grabación completa (CSV/MP4/SVO).
4. Contrastar la topología (nombres y huesos) contra la documentación del
   modelo en la primera ejecución.
