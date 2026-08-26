#ifndef ZED_GAIT_ANALYSIS_RECORDER__RECORDING_SESSION_HPP_
#define ZED_GAIT_ANALYSIS_RECORDER__RECORDING_SESSION_HPP_

#include <cstdint>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

#include "zed_gait_analysis_recorder/pose_types.hpp"

namespace zed_gait
{

struct SessionConfig
{
  std::string directory;      // ya expandido (~ resuelto)
  std::string camera_alias;
  std::string stamp;          // yyyymmdd_hhmmss
  std::string model_id;       // columna ai_model del CSV
  int camera_serial{0};       // columna camera_sn del CSV
  cv::Size video_size;
  double video_fps{30.0};
  bool video_hw_encoder{true};
  int video_bitrate{4000000};
  bool svo_enabled{true};
  std::string svo_compression{"H265"};
  uint64_t min_free_space_mb{1024};
};

// Sesion de grabacion por camara: CSV (20 Hz), video MP4 anotado y SVO2.
// RAII: el destructor cierra lo que siga abierto. Escribe con sufijo
// .partial y renombra al cerrar correctamente (ADR-005).
//
// Ante un error de escritura (p. ej. disco lleno) la sesion pasa a ERROR,
// se cierra sin renombrar y NO se tumba el proceso (ADR-015).
class RecordingSession
{
public:
  enum class State { IDLE, RECORDING, ERROR };

  ~RecordingSession();

  // No copiable: los recursos (ficheros, encoder, SVO) son unicos.
  RecordingSession(const RecordingSession &) = delete;
  RecordingSession & operator=(const RecordingSession &) = delete;

  bool start(
    const SessionConfig & config, sl::Camera & camera,
    const SkeletonTopology & topology);

  // Un frame de video por frame procesado.
  void writeFrame(const cv::Mat & bgr);

  // Una fila de CSV por muestra. person == nullptr escribe la fila de
  // "sin persona" (joints a 0, body_id = -1) para mantener la alineacion
  // temporal entre camaras.
  void writeRow(
    double time_s, int64_t frame_ts_ns, const PersonPose * person,
    const SkeletonTopology & topology);

  // Cierre idempotente. Si el estado es RECORDING, renombra los .partial.
  void stop();

  State state() const {return state_;}

private:
  // Pasa a ERROR y cierra sin renombrar.
  void fail(const std::string & message);
  void closeAndRename(bool rename);
  bool hasFreeSpace() const;

  State state_{State::IDLE};
  std::ofstream csv_;
  cv::VideoWriter video_;
  bool svo_active_{false};
  sl::Camera * camera_{nullptr};  // no posee; la posee el nodo

  std::string model_id_;
  int camera_serial_{0};
  std::string directory_;
  uint64_t min_free_space_mb_{1024};

  uint64_t rows_since_flush_{0};
  uint64_t frames_since_space_check_{0};

  std::string csv_tmp_, csv_final_;
  std::string video_tmp_, video_final_;
  std::string svo_tmp_, svo_final_;
};

}  // namespace zed_gait

#endif  // ZED_GAIT_ANALYSIS_RECORDER__RECORDING_SESSION_HPP_
