#include "zed_gait_analysis_recorder/backends/zed_body_tracking_backend.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace zed_gait
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("zed_body_tracking_backend");

sl::BODY_FORMAT parseBodyFormat(const std::string & value)
{
  if (value == "BODY_18") {return sl::BODY_FORMAT::BODY_18;}
  if (value == "BODY_34") {return sl::BODY_FORMAT::BODY_34;}
  if (value == "BODY_38") {return sl::BODY_FORMAT::BODY_38;}
  RCLCPP_WARN(
    kLogger, "body_format '%s' desconocido; usando BODY_38", value.c_str());
  return sl::BODY_FORMAT::BODY_38;
}

sl::BODY_TRACKING_MODEL parseDetectionModel(const std::string & value)
{
  if (value == "HUMAN_BODY_FAST") {return sl::BODY_TRACKING_MODEL::HUMAN_BODY_FAST;}
  if (value == "HUMAN_BODY_ACCURATE") {return sl::BODY_TRACKING_MODEL::HUMAN_BODY_ACCURATE;}
  if (value == "HUMAN_BODY_MEDIUM") {return sl::BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM;}
  RCLCPP_WARN(
    kLogger, "body_detection_model '%s' desconocido; usando HUMAN_BODY_MEDIUM",
    value.c_str());
  return sl::BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM;
}

// Nombres de joint en minusculas para la cabecera del CSV. NOTA: contrastar
// el orden con el enum del SDK 5.2.2 en la primera ejecucion en la Jetson;
// los indices siempre son correctos, los nombres son cosmeticos.
std::vector<std::string> body38Names()
{
  return {
    "pelvis", "spine_1", "spine_2", "spine_3", "neck", "nose",
    "left_eye", "right_eye", "left_ear", "right_ear",
    "left_clavicle", "right_clavicle",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
    "left_big_toe", "right_big_toe", "left_small_toe", "right_small_toe",
    "left_heel", "right_heel",
    "left_hand_thumb_1", "right_hand_thumb_1",
    "left_hand_thumb_2", "right_hand_thumb_2",
    "left_hand_pinky_1", "right_hand_pinky_1",
    "left_hand_middle_1", "right_hand_middle_1"};
}

std::vector<std::string> body18Names()
{
  return {
    "nose", "neck",
    "right_shoulder", "right_elbow", "right_wrist",
    "left_shoulder", "left_elbow", "left_wrist",
    "right_hip", "right_knee", "right_ankle",
    "left_hip", "left_knee", "left_ankle",
    "right_eye", "left_eye", "right_ear", "left_ear"};
}

// Nombres genericos para formatos sin tabla verificada (BODY_34).
std::vector<std::string> genericNames(size_t count)
{
  std::vector<std::string> names;
  names.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    names.push_back("joint_" + std::to_string(i));
  }
  return names;
}

template<typename BoneEnum>
std::vector<std::pair<int, int>> toBoneIndices(
  const std::vector<std::pair<BoneEnum, BoneEnum>> & bones)
{
  std::vector<std::pair<int, int>> indices;
  indices.reserve(bones.size());
  for (const auto & bone : bones) {
    indices.emplace_back(static_cast<int>(bone.first), static_cast<int>(bone.second));
  }
  return indices;
}

SkeletonTopology buildTopology(sl::BODY_FORMAT format)
{
  SkeletonTopology topology;
  switch (format) {
    case sl::BODY_FORMAT::BODY_18:
      topology.joint_names = body18Names();
      topology.bones = toBoneIndices(sl::BODY_18_BONES);
      break;
    case sl::BODY_FORMAT::BODY_38:
      topology.joint_names = body38Names();
      topology.bones = toBoneIndices(sl::BODY_38_BONES);
      break;
    case sl::BODY_FORMAT::BODY_34:
    default:
      // BODY_34: nombres genericos hasta verificar el orden en el SDK.
      topology.joint_names = genericNames(34);
      topology.bones = toBoneIndices(sl::BODY_34_BONES);
      break;
  }
  return topology;
}

}  // namespace

bool ZedBodyTrackingBackend::init(
  sl::Camera & camera, const BackendConfig & config, std::string & error)
{
  sl::BodyTrackingParameters params;
  params.enable_tracking = true;
  params.enable_body_fitting = true;
  params.detection_model = parseDetectionModel(config.detection_model);
  params.body_format = parseBodyFormat(config.body_format);

  const sl::ERROR_CODE err = camera.enableBodyTracking(params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    error = "enableBodyTracking fallo: " + std::string(sl::toString(err).c_str());
    return false;
  }

  // El SDK trabaja con umbral entero 0-100.
  runtime_.detection_confidence_threshold =
    static_cast<int>(config.confidence_threshold * 100.0F);

  confidence_threshold_ = config.confidence_threshold;
  topology_ = buildTopology(params.body_format);

  std::string format_name = config.body_format;
  std::transform(
    format_name.begin(), format_name.end(), format_name.begin(), ::tolower);
  model_id_ = "zed_sdk_" + format_name;

  RCLCPP_INFO(
    kLogger, "Backend zed_sdk listo: modelo=%s formato=%s umbral=%.2f",
    config.detection_model.c_str(), config.body_format.c_str(),
    config.confidence_threshold);
  return true;
}

PoseResult ZedBodyTrackingBackend::infer(
  sl::Camera & camera, const cv::Mat & bgr)
{
  // El backend zed_sdk obtiene los joints del propio grab(); la imagen la
  // usan los backends externos (mediapipe, ...).
  (void)bgr;

  PoseResult result;
  const sl::ERROR_CODE err = camera.retrieveBodies(bodies_, runtime_);
  if (err != sl::ERROR_CODE::SUCCESS) {
    static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      kLogger, throttle_clock, 2000, "retrieveBodies fallo: %s",
      sl::toString(err).c_str());
    return result;
  }

  result.frame_ts_ns = static_cast<int64_t>(bodies_.timestamp.getNanoseconds());

  for (const auto & body : bodies_.body_list) {
    // Solo detecciones firmes; SEARCHING extrapola keypoints durante
    // oclusiones y no interesa para el analisis.
    if (body.tracking_state != sl::OBJECT_TRACKING_STATE::OK) {continue;}

    PersonPose person;
    person.body_id = static_cast<int>(body.id);
    // SDK 5.2.2: confianza en [0, 1]; si una version futura usa 0-100,
    // ajustar aqui el unico punto de conversion.
    person.confidence = std::clamp(body.confidence, 0.0F, 1.0F);

    const size_t n = body.keypoint.size();
    person.joints.resize(n);
    for (size_t i = 0; i < n; ++i) {
      const sl::float3 & p = body.keypoint[i];
      Joint joint;
      joint.x = p.x;
      joint.y = p.y;
      joint.z = p.z;
      if (i < body.keypoint_2d.size()) {
        joint.u = body.keypoint_2d[i].x;
        joint.v = body.keypoint_2d[i].y;
      }
      if (i < body.keypoint_confidence.size()) {
        joint.confidence = std::clamp(body.keypoint_confidence[i], 0.0F, 1.0F);
      }
      const bool finite =
        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
      joint.valid = finite && joint.confidence >= confidence_threshold_;
      person.joints[i] = joint;
    }
    result.persons.push_back(std::move(person));
  }
  return result;
}

}  // namespace zed_gait
