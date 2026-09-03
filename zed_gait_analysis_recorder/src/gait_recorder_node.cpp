#include "zed_gait_analysis_recorder/gait_recorder_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <cv_bridge/cv_bridge.h>

#include "zed_gait_analysis_recorder/backends/zed_body_tracking_backend.hpp"

namespace zed_gait
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("gait_recorder_node");

sl::RESOLUTION parseResolution(const std::string & value)
{
  if (value == "HD2K") {return sl::RESOLUTION::HD2K;}
  if (value == "HD1080") {return sl::RESOLUTION::HD1080;}
  if (value == "VGA") {return sl::RESOLUTION::VGA;}
  if (value == "HD720") {return sl::RESOLUTION::HD720;}
  RCLCPP_WARN(kLogger, "resolution '%s' desconocida; usando HD720", value.c_str());
  return sl::RESOLUTION::HD720;
}

sl::DEPTH_MODE parseDepthMode(const std::string & value)
{
  if (value == "PERFORMANCE") {return sl::DEPTH_MODE::PERFORMANCE;}
  if (value == "QUALITY") {return sl::DEPTH_MODE::QUALITY;}
  if (value == "NEURAL") {return sl::DEPTH_MODE::NEURAL;}
  if (value == "NEURAL_LIGHT") {return sl::DEPTH_MODE::NEURAL_LIGHT;}
  if (value == "ULTRA") {return sl::DEPTH_MODE::ULTRA;}
  RCLCPP_WARN(kLogger, "depth_mode '%s' desconocido; usando ULTRA", value.c_str());
  return sl::DEPTH_MODE::ULTRA;
}

// Unica "factoria" del proyecto: un if. Backends futuros (resp. 29):
// "openpose", "mediapipe", "yolo".
std::unique_ptr<PoseBackend> createBackend(const std::string & name)
{
  if (name == "zed_sdk") {
    return std::make_unique<ZedBodyTrackingBackend>();
  }
  throw std::runtime_error(
          "pose_backend desconocido: '" + name + "' (disponibles: zed_sdk)");
}

// Expande '~' al HOME del usuario (p. ej. ~/Documents/ZED).
std::string expandUser(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    return (home != nullptr ? std::string(home) : ".") + path.substr(1);
  }
  return path;
}

std::string currentStamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now{};
  localtime_r(&now_c, &tm_now);
  std::ostringstream stamp;
  stamp << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
  return stamp.str();
}

}  // namespace

GaitRecorderNode::GaitRecorderNode()
: Node("gait_recorder_node")
{
  // ---- Parametros ----
  camera_serial_param_ =
    static_cast<int>(declare_parameter<int64_t>("camera_serial", 0));
  camera_alias_ = declare_parameter<std::string>("camera_alias", "");
  svo_path_ = declare_parameter<std::string>("svo_path", "");
  pose_backend_ = declare_parameter<std::string>("pose_backend", "zed_sdk");
  detection_model_ = declare_parameter<std::string>(
    "body_detection_model", "HUMAN_BODY_MEDIUM");
  body_format_ = declare_parameter<std::string>("body_format", "BODY_38");
  confidence_threshold_ = static_cast<float>(
    declare_parameter<double>("confidence_threshold", 0.5));
  resolution_ = declare_parameter<std::string>("resolution", "HD720");
  fps_ = declare_parameter<int>("fps", 60);
  depth_mode_ = declare_parameter<std::string>("depth_mode", "ULTRA");
  depth_min_m_ = declare_parameter<double>("depth_min_m", 1.5);
  depth_max_m_ = declare_parameter<double>("depth_max_m", 10.0);
  jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);
  publish_raw_image_ = declare_parameter<bool>("publish_raw_image", false);
  record_topic_ = declare_parameter<std::string>("record_topic", "/gait_record");
  recording_directory_ = declare_parameter<std::string>(
    "recording_directory", "~/Documents/ZED");
  csv_sample_period_s_ = declare_parameter<double>("csv_sample_period_s", 0.05);
  video_hw_encoder_ = declare_parameter<bool>("video_hw_encoder", true);
  video_bitrate_ = declare_parameter<int>("video_bitrate", 4000000);
  svo_recording_ = declare_parameter<bool>("svo_recording", true);
  svo_compression_ = declare_parameter<std::string>("svo_compression", "H265");
  min_free_space_mb_ = static_cast<uint64_t>(
    declare_parameter<int64_t>("min_free_space_mb", 1024));
  open_retries_ = declare_parameter<int>("open_retries", 5);
  open_retry_delay_s_ = declare_parameter<double>("open_retry_delay_s", 2.0);

  // ---- Camara (con reintentos, ADR-002) y backend ----
  openCamera();
  if (camera_alias_.empty()) {
    camera_alias_ = "zed" + std::to_string(camera_serial_);
  }
  initBackend();

  last_valid_.resize(topology_.joint_names.size());

  // ---- Topics (ADR-003/010) ----
  const std::string base = "/" + camera_alias_ + "/";
  pub_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    base + "image_annotated/compressed", 1);
  if (publish_raw_image_) {
    pub_raw_ = create_publisher<sensor_msgs::msg::Image>(
      base + "image_annotated", 1);
  }
  pub_state_ = create_publisher<std_msgs::msg::Bool>(
    base + "recording_state", rclcpp::QoS(1).transient_local());
  sub_record_ = create_subscription<std_msgs::msg::Bool>(
    record_topic_, 10,
    std::bind(&GaitRecorderNode::cbRecord, this, std::placeholders::_1));
  publishState(false);

  const auto period =
    std::chrono::milliseconds(std::max(1, 1000 / std::max(1, fps_)));
  timer_ =
    create_wall_timer(period, std::bind(&GaitRecorderNode::processFrame, this));

  RCLCPP_INFO(
    get_logger(),
    "[%s] Nodo listo: serial=%d backend=%s (%s) %s@%dfps depth=%s "
    "[%.1f-%.1f m] grabacion en %s",
    camera_alias_.c_str(), camera_serial_, pose_backend_.c_str(),
    model_id_.c_str(), resolution_.c_str(), fps_, depth_mode_.c_str(),
    depth_min_m_, depth_max_m_, expandUser(recording_directory_).c_str());
  RCLCPP_INFO(
    get_logger(), "[%s] Imagen en %s | control en %s", camera_alias_.c_str(),
    (base + "image_annotated/compressed").c_str(), record_topic_.c_str());
}

GaitRecorderNode::~GaitRecorderNode()
{
  // Cierra la sesion (renombrando .partial) antes de cerrar la camara,
  // para que el SVO quede bien cerrado tambien en Ctrl-C.
  session_.stop();
  if (zed_.isOpened()) {
    zed_.close();
  }
}

void GaitRecorderNode::openCamera()
{
  sl::InitParameters init_params;
  if (!svo_path_.empty()) {
    init_params.input.setFromSVOFile(svo_path_.c_str());
    RCLCPP_INFO(get_logger(), "Usando SVO: %s", svo_path_.c_str());
  } else if (camera_serial_param_ > 0) {
    init_params.input.setFromSerialNumber(
      static_cast<unsigned int>(camera_serial_param_));
    RCLCPP_INFO(
      get_logger(), "Abriendo camara con serial %d", camera_serial_param_);
  }

  init_params.camera_resolution = parseResolution(resolution_);
  init_params.camera_fps = fps_;
  init_params.depth_mode = parseDepthMode(depth_mode_);
  init_params.coordinate_units = sl::UNIT::METER;
  init_params.depth_minimum_distance = static_cast<float>(depth_min_m_);
  init_params.depth_maximum_distance = static_cast<float>(depth_max_m_);

  sl::ERROR_CODE err = sl::ERROR_CODE::FAILURE;
  for (int attempt = 1; attempt <= open_retries_; ++attempt) {
    err = zed_.open(init_params);
    if (err == sl::ERROR_CODE::SUCCESS) {
      break;
    }
    RCLCPP_WARN(
      get_logger(), "Apertura ZED fallida (%s). Intento %d/%d",
      sl::toString(err).c_str(), attempt, open_retries_);
    if (attempt < open_retries_) {
      rclcpp::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(open_retry_delay_s_)));
    }
  }
  if (err != sl::ERROR_CODE::SUCCESS) {
    throw std::runtime_error(
            "No se pudo abrir la camara ZED: " +
            std::string(sl::toString(err).c_str()));
  }

  camera_serial_ =
    static_cast<int>(zed_.getCameraInformation().serial_number);
  RCLCPP_INFO(get_logger(), "Camara abierta, serial: %d", camera_serial_);
}

void GaitRecorderNode::initBackend()
{
  backend_ = createBackend(pose_backend_);
  BackendConfig config{detection_model_, body_format_, confidence_threshold_};
  std::string error;
  if (!backend_->init(zed_, config, error)) {
    throw std::runtime_error(
            "Backend '" + pose_backend_ + "' no pudo iniciarse: " + error);
  }
  topology_ = backend_->topology();
  model_id_ = backend_->modelId();
}

// ---------------------------------------------------------------------------
// Bucle principal: grab() bloqueante -> siempre el frame mas reciente
// (cola implicita de 1, ADR-001).
// ---------------------------------------------------------------------------
void GaitRecorderNode::processFrame()
{
  const sl::ERROR_CODE grab_state = zed_.grab();

  if (grab_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
    RCLCPP_INFO(get_logger(), "Fin del SVO. Cerrando grabacion y nodo.");
    stopRecording();
    rclcpp::shutdown();
    return;
  }
  if (grab_state != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "grab() fallo: %s",
      sl::toString(grab_state).c_str());
    return;
  }

  zed_.retrieveImage(image_zed_, sl::VIEW::LEFT);

  // sl::Mat -> cv::Mat compartiendo memoria (sin copia).
  cv::Mat img_bgra(
    static_cast<int>(image_zed_.getHeight()),
    static_cast<int>(image_zed_.getWidth()),
    CV_8UC4, image_zed_.getPtr<sl::uchar1>());
  cv::Mat img;
  cv::cvtColor(img_bgra, img, cv::COLOR_BGRA2BGR);

  PoseResult result = backend_->infer(zed_, image_zed_);

  // Una persona objetivo (resp. 40): la de mayor confianza del frame.
  const PersonPose * main_person = selectMainPerson(result);
  PersonPose estimated;
  const PersonPose * row_person = nullptr;
  if (main_person != nullptr) {
    estimated = *main_person;
    applyInvalidJointPolicy(estimated);
    drawSkeleton(img, estimated);
    row_person = &estimated;
  }

  // Se publica siempre, con o sin deteccion (ADR-003).
  publishCompressed(img);
  if (pub_raw_) {
    publishRaw(img);
  }

  if (session_.state() == RecordingSession::State::RECORDING) {
    session_.writeFrame(img);

    const double time_s = (now() - record_start_).seconds();
    if (last_csv_time_s_ < 0.0 ||
      (time_s - last_csv_time_s_) >= csv_sample_period_s_)
    {
      session_.writeRow(time_s, result.frame_ts_ns, row_person, topology_);
      last_csv_time_s_ = time_s;
    }

    // La sesion pudo detenerse sola (disco lleno, ADR-015): se comunica.
    if (session_.state() == RecordingSession::State::ERROR &&
      state_published_)
    {
      publishState(false);
    }
  }
}

const PersonPose * GaitRecorderNode::selectMainPerson(
  const PoseResult & result) const
{
  const auto it = std::max_element(
    result.persons.begin(), result.persons.end(),
    [](const PersonPose & a, const PersonPose & b) {
      return a.confidence < b.confidence;
    });
  return it == result.persons.end() ? nullptr : &*it;
}

void GaitRecorderNode::applyInvalidJointPolicy(PersonPose & person)
{
  // Joints invalidos (confianza < umbral o XYZ no finita): se conserva la
  // confianza real (marca la invalidez en el CSV) y la XYZ se estima con el
  // ultimo valor valido; si nunca lo hubo, queda 0,0,0 (ADR-012).
  const size_t n =
    std::min(person.joints.size(), last_valid_.size());
  for (size_t i = 0; i < n; ++i) {
    Joint & joint = person.joints[i];
    if (joint.valid) {
      last_valid_[i] = joint;
    } else {
      if (last_valid_[i].valid) {  // valid guarda el ultimo XYZ bueno
        joint.x = last_valid_[i].x;
        joint.y = last_valid_[i].y;
        joint.z = last_valid_[i].z;
      }
      joint.estimated = true;
    }
  }
}

void GaitRecorderNode::drawSkeleton(
  cv::Mat & frame, const PersonPose & person) const
{
  const auto & joints = person.joints;
  for (const auto & bone : topology_.bones) {
    if (bone.first >= static_cast<int>(joints.size()) ||
      bone.second >= static_cast<int>(joints.size()))
    {
      continue;
    }
    const Joint & a = joints[bone.first];
    const Joint & b = joints[bone.second];
    if (!a.valid || !b.valid) {continue;}
    cv::line(
      frame, cv::Point(static_cast<int>(a.u), static_cast<int>(a.v)),
      cv::Point(static_cast<int>(b.u), static_cast<int>(b.v)),
      cv::Scalar(0, 255, 255), 2);
  }
  for (const auto & joint : joints) {
    if (!joint.valid) {continue;}
    cv::circle(
      frame, cv::Point(static_cast<int>(joint.u), static_cast<int>(joint.v)),
      3, cv::Scalar(0, 255, 0), -1);
  }
}

void GaitRecorderNode::publishCompressed(const cv::Mat & img)
{
  const std::vector<int> encode_param = {
    cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
  std::vector<uchar> compressed;
  if (!cv::imencode(".jpg", img, compressed, encode_param)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "imencode JPEG fallo");
    return;
  }
  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = now();
  msg.format = "jpeg";
  msg.data = std::move(compressed);
  pub_compressed_->publish(msg);
}

void GaitRecorderNode::publishRaw(const cv::Mat & img)
{
  auto msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  msg->header.stamp = now();
  pub_raw_->publish(*msg);
}

// ---------------------------------------------------------------------------
// Control de grabacion (ADR-010): true inicia, false detiene. Idempotente.
// ---------------------------------------------------------------------------
void GaitRecorderNode::cbRecord(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    startRecording();
  } else {
    stopRecording();
  }
}

void GaitRecorderNode::startRecording()
{
  if (session_.state() == RecordingSession::State::RECORDING) {
    RCLCPP_WARN(
      get_logger(), "[%s] Ya hay una grabacion activa; se ignora la orden",
      camera_alias_.c_str());
    return;
  }
  if (!image_zed_.isInit()) {
    RCLCPP_ERROR(
      get_logger(),
      "[%s] Todavia no hay frames de camara; no se puede iniciar",
      camera_alias_.c_str());
    return;
  }

  for (auto & j : last_valid_) {j.valid = false;}
  last_csv_time_s_ = -1.0;
  record_start_ = now();

  SessionConfig config;
  config.directory = expandUser(recording_directory_);
  config.camera_alias = camera_alias_;
  config.stamp = currentStamp();
  config.model_id = model_id_;
  config.camera_serial = camera_serial_;
  config.video_size = cv::Size(
    static_cast<int>(image_zed_.getWidth()),
    static_cast<int>(image_zed_.getHeight()));
  config.video_fps = static_cast<double>(fps_);
  config.video_hw_encoder = video_hw_encoder_;
  config.video_bitrate = video_bitrate_;
  config.svo_enabled = svo_recording_;
  config.svo_compression = svo_compression_;
  config.min_free_space_mb = min_free_space_mb_;

  if (session_.start(config, zed_, topology_)) {
    publishState(true);
    RCLCPP_INFO(get_logger(), "[%s] Grabacion iniciada", camera_alias_.c_str());
  } else {
    RCLCPP_ERROR(
      get_logger(), "[%s] No se pudo iniciar la grabacion",
      camera_alias_.c_str());
    publishState(false);
  }
}

void GaitRecorderNode::stopRecording()
{
  if (session_.state() == RecordingSession::State::IDLE) {
    return;
  }
  session_.stop();
  publishState(false);
  RCLCPP_INFO(get_logger(), "[%s] Grabacion detenida", camera_alias_.c_str());
}

void GaitRecorderNode::publishState(bool recording)
{
  std_msgs::msg::Bool msg;
  msg.data = recording;
  pub_state_->publish(msg);
  state_published_ = recording;
}

}  // namespace zed_gait
