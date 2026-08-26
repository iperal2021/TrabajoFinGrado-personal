#ifndef ZED_GAIT_ANALYSIS_RECORDER__POSE_TYPES_HPP_
#define ZED_GAIT_ANALYSIS_RECORDER__POSE_TYPES_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace zed_gait
{

// Joint de una persona: XYZ en metros (frame de la camara), pixel 2D y
// confianza normalizada [0, 1].
// valid == false cuando la confianza es menor que el umbral o la XYZ no es
// finita; estimated == true cuando la XYZ proviene del ultimo valor valido
// (politica de joints invalidos, ADR-012).
struct Joint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float u{0.0F};
  float v{0.0F};
  float confidence{0.0F};
  bool valid{false};
  bool estimated{false};
};

struct PersonPose
{
  int body_id{-1};
  float confidence{0.0F};
  std::vector<Joint> joints;
};

struct PoseResult
{
  std::vector<PersonPose> persons;
  // Timestamp del frame (ns, reloj de la camara ZED) para realinear camaras.
  int64_t frame_ts_ns{0};
};

// Topologia del esqueleto: nombres de joint (cabecera del CSV) y pares de
// indices de huesos (dibujo). La aporta cada backend (resp. 107).
struct SkeletonTopology
{
  std::vector<std::string> joint_names;
  std::vector<std::pair<int, int>> bones;
};

}  // namespace zed_gait

#endif  // ZED_GAIT_ANALYSIS_RECORDER__POSE_TYPES_HPP_
