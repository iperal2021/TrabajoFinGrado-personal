#ifndef ZED_GAIT_ANALYSIS_RECORDER__GAIT_RECORDER_NODE_HPP_
#define ZED_GAIT_ANALYSIS_RECORDER__GAIT_RECORDER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sl/Camera.hpp>
#include <std_msgs/msg/bool.hpp>

#include "zed_gait_analysis_recorder/pose_backend.hpp"
#include "zed_gait_analysis_recorder/recording_session.hpp"

namespace zed_gait
{

// Un nodo por camara (ADR-001): captura + pose + dibujo + publicacion +
// grabacion en el mismo proceso. Un proceso por camara; el launch instancia
// uno por serial detectado o pedido.
class GaitRecorderNode : public rclcpp::Node
{
public:
  GaitRecorderNode();
  ~GaitRecorderNode() override;

private:
  void openCamera();
  void initBackend();
  void processFrame();

  void cbRecord(const std_msgs::msg::Bool::SharedPtr msg);
  void startRecording();
  void stopRecording();
  void publishState(bool recording);

  const PersonPose * selectMainPerson(const PoseResult & result) const;
  void applyInvalidJointPolicy(PersonPose & person);
  void drawSkeleton(cv::Mat & frame, const PersonPose & person) const;
  void publishCompressed(const cv::Mat & img);
  void publishRaw(const cv::Mat & img);

  // ---- Parametros (ver config/default.yaml) ----
  int camera_serial_param_{0};
  std::string camera_alias_;
  std::string svo_path_;
  std::string pose_backend_;
  std::string detection_model_;
  std::string body_format_;
  float confidence_threshold_{0.5F};
  std::string resolution_;
  int fps_{60};
  std::string depth_mode_;
  double depth_min_m_{1.5};
  double depth_max_m_{10.0};
  int jpeg_quality_{80};
  bool publish_raw_image_{false};
  std::string record_topic_;
  std::string recording_directory_;
  double csv_sample_period_s_{0.05};
  bool video_hw_encoder_{true};
  int video_bitrate_{4000000};
  bool svo_recording_{true};
  std::string svo_compression_;
  uint64_t min_free_space_mb_{1024};
  int open_retries_{5};
  double open_retry_delay_s_{2.0};

  // ---- Estado resuelto tras abrir la camara ----
  int camera_serial_{0};

  // ---- ROS 2 ----
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_record_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool state_published_{false};

  // ---- ZED SDK + backend ----
  sl::Camera zed_;
  sl::Mat image_zed_;
  std::unique_ptr<PoseBackend> backend_;
  SkeletonTopology topology_;
  std::string model_id_;

  // ---- Grabacion ----
  RecordingSession session_;
  rclcpp::Time record_start_{0, 0, RCL_ROS_TIME};
  double last_csv_time_s_{-1.0};

  // Politica de joints invalidos: ultimo valor valido por joint (ADR-012);
  // valid == false marca "sin valor previo".
  std::vector<Joint> last_valid_;
};

}  // namespace zed_gait

#endif  // ZED_GAIT_ANALYSIS_RECORDER__GAIT_RECORDER_NODE_HPP_
