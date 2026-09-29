#ifndef ZED_GAIT_ANALYSIS_RECORDER__GAIT_RECORDER_NODE_HPP_
#define ZED_GAIT_ANALYSIS_RECORDER__GAIT_RECORDER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sl/Camera.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace zed_gait
{

// Nodo grabador por camara (zed_gait_v2): publica la imagen 2D para la
// interfaz web (ros2_gait_monitor.html) y graba SVO2 (estereo crudo + IMU)
// por paciente. Sin body tracking: eso es la linea de trabajo offline
// (fase 2, procesado del SVO a CSV).
//
// Flujo de sesion (topics fijos de la interfaz):
//   1. La interfaz publica patient_id + phase y luego /gait_record=true.
//   2. El nodo graba <id>_<FASE>_<stamp>.svo2 en
//      ~/Documents/patient_records/<id>/ hasta /gait_record=false.
//   3. Al parar, la interfaz publica notes + /gait_session_accept:
//      accept=true escribe el .txt de metadatos; false borra el .svo2.
class GaitRecorderNode : public rclcpp::Node
{
public:
  GaitRecorderNode();
  ~GaitRecorderNode() override;

private:
  void openCamera();
  void processFrame();
  void publishCompressed(const cv::Mat & img);

  void cbRecord(const std_msgs::msg::Bool::SharedPtr msg);
  void cbAccept(const std_msgs::msg::Bool::SharedPtr msg);
  void cbPatientId(const std_msgs::msg::String::SharedPtr msg);
  void cbPhase(const std_msgs::msg::String::SharedPtr msg);
  void cbNotes(const std_msgs::msg::String::SharedPtr msg);
  void startRecording();
  void stopRecording();
  void publishState(bool recording);

  // ---- Parametros (ver config/default.yaml) ----
  int camera_serial_param_{0};
  std::string camera_alias_;
  std::string svo_path_;
  bool svo_loop_ = false;
  std::string resolution_;
  int fps_{60};
  int jpeg_quality_{50};
  std::string record_topic_;
  std::string recording_directory_;
  std::string svo_compression_;
  uint64_t min_free_space_mb_{1024};
  int open_retries_{5};
  double open_retry_delay_s_{2.0};

  // ---- Estado resuelto tras abrir la camara ----
  int camera_serial_{0};

  // ---- ROS 2 ----
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_record_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_accept_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_patient_id_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_phase_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_notes_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- ZED SDK. Depth desactivada: el SVO2 graba estereo crudo + IMU y
  // la profundidad se recalcula al reproducir; cero coste de GPU aqui. ----
  sl::Camera zed_;
  sl::Mat image_zed_;
  sl::RuntimeParameters runtime_params_;

  // ---- Metadatos de sesion cacheados (los publica la interfaz web) ----
  std::string patient_id_;
  std::string phase_;
  std::string notes_;

  // ---- Sesion de grabacion ----
  bool recording_{false};
  // Tras stopRecording la sesion queda pendiente de aceptar/descartar: las
  // notas y el accept llegan despues de parar, asi que el .txt se escribe
  // (o el .svo2 se borra) en cbAccept.
  bool session_pending_{false};
  std::string session_base_;    // ruta sin extension (.svo2 / .txt)
  std::string session_id_;      // id sanitizado (apto para ficheros)
  std::string session_phase_;   // PRE/POST en mayusculas
  std::string session_stamp_;   // yyyymmdd_hhmmss
  std::chrono::steady_clock::time_point session_start_{};
  double session_duration_s_{0.0};
};

}  // namespace zed_gait

#endif  // ZED_GAIT_ANALYSIS_RECORDER__GAIT_RECORDER_NODE_HPP_
