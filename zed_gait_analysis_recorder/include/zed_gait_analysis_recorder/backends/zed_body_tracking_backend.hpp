#ifndef ZED_GAIT_ANALYSIS_RECORDER__BACKENDS__ZED_BODY_TRACKING_BACKEND_HPP_
#define ZED_GAIT_ANALYSIS_RECORDER__BACKENDS__ZED_BODY_TRACKING_BACKEND_HPP_

#include <string>

#include "zed_gait_analysis_recorder/pose_backend.hpp"

namespace zed_gait
{

// Backend de pose basado en el Body Tracking del propio ZED SDK (ADR-009).
class ZedBodyTrackingBackend : public PoseBackend
{
public:
  bool init(
    sl::Camera & camera, const BackendConfig & config,
    std::string & error) override;

  PoseResult infer(sl::Camera & camera, const sl::Mat & left_image) override;

  const SkeletonTopology & topology() const override {return topology_;}

  std::string modelId() const override {return model_id_;}

private:
  sl::BodyTrackingRuntimeParameters runtime_;
  sl::Bodies bodies_;
  SkeletonTopology topology_;
  std::string model_id_{"zed_sdk"};
  float confidence_threshold_{0.5F};
};

}  // namespace zed_gait

#endif  // ZED_GAIT_ANALYSIS_RECORDER__BACKENDS__ZED_BODY_TRACKING_BACKEND_HPP_
