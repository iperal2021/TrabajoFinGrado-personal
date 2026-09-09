#ifndef ZED_GAIT_ANALYSIS_RECORDER__POSE_BACKEND_HPP_
#define ZED_GAIT_ANALYSIS_RECORDER__POSE_BACKEND_HPP_

#include <string>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

#include "zed_gait_analysis_recorder/pose_types.hpp"

namespace zed_gait
{

// Configuracion compartida de backends. Cada backend usa solo los campos
// que le aplican: zed_sdk usa detection_model/body_format; mediapipe usa
// model_*/python_path/script_path y depth_min/max_m (para su proyeccion).
struct BackendConfig
{
  // Comun: umbral de validez por joint [0, 1].
  float confidence_threshold{0.5F};

  // zed_sdk
  std::string detection_model{"HUMAN_BODY_MEDIUM"};
  std::string body_format{"BODY_38"};

  // mediapipe
  std::string model_path;
  std::string model_variant{"full"};
  std::string python_path{"python3"};
  std::string script_path;
  float depth_min_m{1.5F};
  float depth_max_m{10.0F};
};

// Interfaz minima de backend de pose (ADR-001/009/018). El nodo es dueno de
// sl::Camera y del bucle grab(); infer() se llama siempre tras un grab() con
// exito, de modo que imagen, profundidad y joints pertenecen al mismo frame.
//
// - El backend "zed_sdk" usa el Body Tracking del propio SDK
//   (retrieveBodies), que ya devuelve keypoints 3D.
// - El backend "mediapipe" envia el frame a un worker Python (BlazePose) por
//   un socket Unix y proyecta los landmarks 2D a 3D con la profundidad ZED.
class PoseBackend
{
public:
  virtual ~PoseBackend() = default;

  // Inicializa el backend sobre la camara ya abierta. Devuelve false y
  // rellena `error` si no puede arrancar.
  virtual bool init(
    sl::Camera & camera, const BackendConfig & config,
    std::string & error) = 0;

  // bgr: imagen izquierda rectificada ya convertida (la misma que el nodo
  // dibuja y publica), para que los backends no repitan la conversion.
  virtual PoseResult infer(sl::Camera & camera, const cv::Mat & bgr) = 0;

  virtual const SkeletonTopology & topology() const = 0;

  // Identificador corto para la columna ai_model del CSV
  // (p. ej. "zed_sdk_body_38", "mediapipe_full").
  virtual std::string modelId() const = 0;
};

}  // namespace zed_gait

#endif  // ZED_GAIT_ANALYSIS_RECORDER__POSE_BACKEND_HPP_
