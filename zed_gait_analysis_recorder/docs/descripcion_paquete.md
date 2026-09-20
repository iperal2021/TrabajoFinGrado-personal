# zed_gait_analysis_recorder — qué hace el paquete

Paquete ROS 2 (C++) del TFG para análisis de marcha de miembro inferior con
cámaras ZED 2i. Un proceso `gait_recorder_node` por cámara: captura vídeo,
estima la pose corporal, publica la imagen anotada y, bajo demanda, graba la
sesión (CSV de joints 3D + MP4 + SVO2). Máquina objetivo: Jetson Orin NX.

Especificación: `plan/PLAN_v2_zed_gait_analysis_recorder.md` (ADR-001…017)
y `plan/PLAN_mediapipe_backend.md` (ADR-018).

## Flujo por frame

```
sl::Camera::grab() ──> PoseBackend::infer() ──> política joints inválidos
      │                      │                         │
      │                      │                         └─> dibujo esqueleto ──> /<alias>/image_annotated/compressed
      │                      └─> (grabando) RecordingSession: CSV 20 Hz + MP4 + SVO2
      └─> cola implícita de 1: grab() bloqueante devuelve siempre el frame
          más reciente; si la inferencia va lenta se descartan frames (ADR-001)
```

- **Un nodo por cámara** (ADR-001): dos cámaras = dos procesos independientes
  bajo el mismo launch, namespace `zed<serial>` (ADR-014). Una persona
  objetivo por cámara: la de mayor confianza del frame.
- **Publica siempre**, haya detección o no (ADR-003).
- **Control de grabación global**: un `std_msgs/Bool` en `/gait_record`
  inicia/detiene todas las cámaras a la vez (ADR-010); cada nodo reporta su
  estado en `/<alias>/recording_state` (`transient_local`).

## Backends de pose (`PoseBackend`, única abstracción)

Interfaz: `init(camera, config)`, `infer(camera, left_image, need_3d)`,
`topology()`, `modelId()`. Cómo añadir más: `docs/add_pose_backend.md`.

- **`zed_sdk`** (ADR-009): Body Tracking del ZED SDK vía `retrieveBodies()`,
  BODY_18/34/38, 3D nativo. Necesita la profundidad activa en cada grab.
- **`mediapipe`** (ADR-018): BlazePose de 33 landmarks corriendo en un
  **worker Python separado** (`scripts/mediapipe_worker.py`) comunicado por
  socket Unix con protocolo binario (frame BGR → 33×(x,y,vis)). La XYZ se
  obtiene proyectando los landmarks 2D sobre la profundidad ZED del mismo
  grab (mediana en ventana 5×5, rango 1.5–10 m). El worker muere solo si el
  nodo cierra la conexión (no zombies).
  - Variantes `lite`/`full`/`heavy` (`.task` en `models/`,
    `scripts/download_models.sh`).
  - `mediapipe_gpu: true` = delegado GPU (OpenGL ES/EGL); si no, CPU.
  - `mediapipe_input_width: 640` reescala el frame antes del socket; los
    landmarks son normalizados [0,1], así que la proyección sigue
    haciéndose sobre el frame original.

## Optimizaciones de rendimiento (posteriores al plan)

- La profundidad ZED (~10 ms/grab) **se desactiva cuando no se graba** con
  el backend mediapipe (`need_3d`/`enable_depth`); al iniciar grabación se
  reactiva para la XYZ del CSV. Con `zed_sdk` siempre activa.
- Conversión BGR→RGB del worker con `cv2.cvtColor` (SIMD) en vez de numpy.
- Profiling por etapas: nodo (`[prof]` cada 60 frames) y worker
  (`ms/frame: recv/rgb/infer`).

## Grabación (`RecordingSession`, ADR-004/005/011/012/015)

Por sesión y cámara, en `~/Documents/ZED` (escritura atómica `.partial`):

- **CSV** a 20 Hz (`csv_sample_period_s`): joints 3D en metros, frame de
  cámara, confianza por joint; joints inválidos conservan confianza real y
  XYZ estimada del último valor válido (ADR-012).
- **MP4 H.264** con el esqueleto dibujado, NVENC vía GStreamer con fallback
  software (ADR-004); recibe **cada** frame procesado.
- **SVO2 nativo** (H265 con fallback H264), opcional.
- Máquina de estados `IDLE → RECORDING → (ERROR) → IDLE`; chequeo de
  espacio en disco periódico (ADR-015); cierre y renombrado seguro incluso
  con Ctrl-C.

## Lanzamiento

```bash
ros2 launch zed_gait_analysis_recorder zed_gait_analysis_recorder.launch.py \
  [camera_serials:=sn1,sn2] [svo_path:=file.svo2] [svo_loop:=true] \
  [pose_backend:=mediapipe] [mediapipe_gpu:=true] \
  [mediapipe_model_variant:=lite] [config_file:=...]
```

- Sin `svo_path`: autodetecta hasta 2 cámaras (arranque degradado con
  reintentos si falta alguna, ADR-002). Con `svo_path`: un único nodo
  `zed_svo` en modo simulación; por defecto termina al final del SVO,
  `svo_loop:=true` repite en bucle (pruebas).
- El launch fija `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` y `ROS_DOMAIN_ID=0`
  (ADR-008): no hace falta exportar nada en el shell.

## Decisiones de diseño clave (resumen ADR)

| Decisión | Elección |
|---|---|
| Arquitectura | Un nodo por cámara (ADR-001) |
| Cola de frames | Implícita de 1, sin backlog (ADR-001) |
| Publicación | Siempre, con o sin detección (ADR-003) |
| Vídeo | MP4 + H.264 + NVENC, fallback software (ADR-004) |
| Escritura | Atómica con `.partial` (ADR-005) |
| Lifecycle | Nodo ROS 2 normal (ADR-006) |
| Filtrado temporal | Ninguno en v1 (ADR-007) |
| Middleware | CycloneDDS (ADR-008) |
| Backend inicial | ZED SDK Body Tracking (ADR-009) |
| Control | Bool compartido `/gait_record` (ADR-010) |
| Backend alternativo | MediaPipe BlazePose vía worker Python + socket (ADR-018) |

Objetivos de rendimiento (Fase 1, Jetson): ≥20 FPS con 1 cámara, ≥10 con 2,
720p, CSV a 20 Hz exactos, profundidad útil 1.5–10 m.
