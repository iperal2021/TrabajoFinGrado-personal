#ifndef ZED_GAIT_ANALYSIS_RECORDER__POSE_BACKEND_HPP_
#define ZED_GAIT_ANALYSIS_RECORDER__POSE_BACKEND_HPP_

#include <string>

#include <sl/Camera.hpp>

#include "zed_gait_analysis_recorder/pose_types.hpp"

namespace zed_gait
{

// Configuracion independiente del backend.
struct BackendConfig
{
  std::string detection_model{"HUMAN_BODY_MEDIUM"};
  std::string body_format{"BODY_38"};
  float confidence_threshold{0.5F};  // [0, 1]
};

// Interfaz minima de backend de pose (ADR-001/009). El nodo es dueno de
// sl::Camera y del bucle grab(); infer() se llama siempre tras un grab() con
// exito, de modo que imagen, profundidad y joints pertenecen al mismo frame.
//
// - El backend "zed_sdk" usa el Body Tracking del propio SDK
//   (retrieveBodies), que ya devuelve keypoints 3D.
// - Los backends externos futuros (OpenPose, MediaPipe, YOLO) usaran
//   left_image y la profundidad de la propia camara para proyectar 2D -> 3D.
class PoseBackend
{
public:
  virtual ~PoseBackend() = default;

  // Inicializa el backend sobre la camara ya abierta. Devuelve false y
  // rellena `error` si no puede arrancar.
  virtual bool init(
    sl::Camera & camera, const BackendConfig & config,
    std::string & error) = 0;

  virtual PoseResult infer(sl::Camera & camera, const sl::Mat & left_image) = 0;

  virtual const SkeletonTopology & topology() const = 0;

  // Identificador corto para la columna ai_model del CSV
  // (p. ej. "zed_sdk_body_38").
  virtual std::string modelId() const = 0;
};

}  // namespace zed_gait

#endif  // ZED_GAIT_ANALYSIS_RECORDER__POSE_BACKEND_HPP_
