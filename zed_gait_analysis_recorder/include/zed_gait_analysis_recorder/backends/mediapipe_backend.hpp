#ifndef ZED_GAIT_ANALYSIS_RECORDER__BACKENDS__MEDIAPIPE_BACKEND_HPP_
#define ZED_GAIT_ANALYSIS_RECORDER__BACKENDS__MEDIAPIPE_BACKEND_HPP_

#include <sys/types.h>

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "zed_gait_analysis_recorder/pose_backend.hpp"

namespace zed_gait
{

// Backend de pose MediaPipe BlazePose (33 landmarks, ADR-018): lanza un
// worker Python (mediapipe_worker.py) como proceso hijo y habla con el por
// un socket Unix. La XYZ de cada joint se obtiene de la profundidad ZED del
// mismo grab: mediana de Z validos en ventana 5x5 + back-proyeccion con los
// intrinsecos de la imagen izquierda rectificada.
class MediaPipeBackend : public PoseBackend
{
public:
  MediaPipeBackend();
  ~MediaPipeBackend() override;

  bool init(
    sl::Camera & camera, const BackendConfig & config,
    std::string & error) override;

  PoseResult infer(sl::Camera & camera, const cv::Mat & bgr) override;

  const SkeletonTopology & topology() const override {return topology_;}

  std::string modelId() const override {return model_id_;}

private:
  bool connectWorker(std::string & error);
  void stopWorker();
  bool sendExact(const void * data, size_t nbytes);
  bool recvExact(void * data, size_t nbytes);
  // Cierra el socket y recolecta al worker muerto.
  void workerDied(const char * where);

  // Mediana de Z validos en la ventana 5x5 de (u, v) + back-proyeccion.
  bool depthAt(int u, int v, float & x, float & y, float & z) const;

  SkeletonTopology topology_;
  std::string model_id_{"mediapipe"};
  float confidence_threshold_{0.5F};

  // Worker
  pid_t worker_pid_{-1};
  int socket_fd_{-1};
  std::string socket_path_;
  int64_t last_timestamp_ms_{-1};

  // Proyeccion 3D: mapa de profundidad del mismo grab + intrinsecos
  // cacheados de la imagen izquierda rectificada.
  sl::Mat depth_map_;
  float fx_{0.0F};
  float fy_{0.0F};
  float cx_{0.0F};
  float cy_{0.0F};
  float depth_min_m_{1.5F};
  float depth_max_m_{10.0F};

  rclcpp::Logger logger_;
};

}  // namespace zed_gait

#endif  // ZED_GAIT_ANALYSIS_RECORDER__BACKENDS__MEDIAPIPE_BACKEND_HPP_
