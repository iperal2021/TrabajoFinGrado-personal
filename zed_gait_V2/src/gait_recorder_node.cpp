#include "zed_gait_analysis_recorder/gait_recorder_node.hpp"

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace zed_gait
{

namespace fs = std::filesystem;

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

// Expande '~' al HOME del usuario (p. ej. ~/Documents/patient_records).
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

// Deja el id de paciente apto para directorio/fichero: [A-Za-z0-9_-].
std::string sanitizeId(const std::string & id)
{
  std::string out;
  out.reserve(id.size());
  for (const unsigned char c : id) {
    out +=
      (std::isalnum(c) != 0 || c == '-' || c == '_') ?
      static_cast<char>(c) : '_';
  }
  return out;
}

// Lee records_dir de la config del servidor del monitor web
// (gait_monitor_server.py la escribe al crear/seleccionar la carpeta).
// JSON plano: basta una regex, sin dependencia de parser.
std::string configuredRecordsDir()
{
  const char * xdg = std::getenv("XDG_CONFIG_HOME");
  const char * home = std::getenv("HOME");
  std::string base;
  if (xdg != nullptr && *xdg != '\0') {
    base = xdg;
  } else if (home != nullptr) {
    base = std::string(home) + "/.config";
  } else {
    return "";
  }
  std::ifstream f(base + "/gait_monitor/config.json");
  if (!f) {return "";}
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string content = ss.str();
  std::smatch m;
  const std::regex re(R"json("records_dir"\s*:\s*"([^"]+)")json");
  if (std::regex_search(content, m, re)) {return m[1].str();}
  return "";
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
  svo_loop_ = declare_parameter<bool>("svo_loop", false);
  resolution_ = declare_parameter<std::string>("resolution", "HD720");
  fps_ = declare_parameter<int>("fps", 60);
  jpeg_quality_ = declare_parameter<int>("jpeg_quality", 50);
  record_topic_ = declare_parameter<std::string>("record_topic", "/gait_record");
  recording_directory_ = declare_parameter<std::string>(
    "recording_directory", "");
  svo_compression_ = declare_parameter<std::string>("svo_compression", "H265");
  min_free_space_mb_ = static_cast<uint64_t>(
    declare_parameter<int64_t>("min_free_space_mb", 1024));
  open_retries_ = declare_parameter<int>("open_retries", 5);
  open_retry_delay_s_ = declare_parameter<double>("open_retry_delay_s", 2.0);
  start_delay_ms_ = declare_parameter<int>("start_delay_ms", 500);

  // ---- Camara (con reintentos) ----
  openCamera();
  if (camera_alias_.empty()) {
    camera_alias_ = "zed" + std::to_string(camera_serial_);
  }
  runtime_params_.enable_depth = false;  // recorder puro: ver comentario hpp

  // ---- Topics ----
  const std::string base = "/" + camera_alias_ + "/";
  pub_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    base + "image_annotated/compressed", 1);
  pub_state_ = create_publisher<std_msgs::msg::Bool>(
    base + "recording_state", rclcpp::QoS(1).transient_local());

  sub_record_ = create_subscription<std_msgs::msg::Bool>(
    record_topic_, 10,
    std::bind(&GaitRecorderNode::cbRecord, this, std::placeholders::_1));
  // Topics fijos de la interfaz web (ros2_gait_monitor.html). Absolutos:
  // el nodo va namespaced (zed<serial>) y el control es global.
  sub_accept_ = create_subscription<std_msgs::msg::Bool>(
    "/gait_session_accept", 10,
    std::bind(&GaitRecorderNode::cbAccept, this, std::placeholders::_1));
  sub_patient_id_ = create_subscription<std_msgs::msg::String>(
    "/gait_session_patient_id", 10,
    std::bind(&GaitRecorderNode::cbPatientId, this, std::placeholders::_1));
  sub_phase_ = create_subscription<std_msgs::msg::String>(
    "/gait_session_phase", 10,
    std::bind(&GaitRecorderNode::cbPhase, this, std::placeholders::_1));
  publishState(false);

  const auto period =
    std::chrono::milliseconds(std::max(1, 1000 / std::max(1, fps_)));
  timer_ =
    create_wall_timer(period, std::bind(&GaitRecorderNode::processFrame, this));

  const std::string dir_info = recording_directory_.empty() ?
    "auto (config del monitor o ~/Documents/patient_records)" :
    expandUser(recording_directory_);
  RCLCPP_INFO(
    get_logger(),
    "[%s] Nodo listo: serial=%d %s@%dfps (depth off) | imagen en %s | "
    "grabacion en %s (control: %s)",
    camera_alias_.c_str(), camera_serial_, resolution_.c_str(), fps_,
    (base + "image_annotated/compressed").c_str(),
    dir_info.c_str(), record_topic_.c_str());
}

GaitRecorderNode::~GaitRecorderNode()
{
  // Cierra la grabacion antes que la camara para que el SVO2 quede bien
  // cerrado tambien en Ctrl-C.
  if (recording_) {
    zed_.disableRecording();
    recording_ = false;
  }
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
  // Depth desactivada en runtime: el SVO2 graba estereo crudo + IMU y la
  // profundidad se recalcula al reproducir; no hace falta GPU aqui.
  init_params.depth_mode = sl::DEPTH_MODE::NONE;
  init_params.coordinate_units = sl::UNIT::METER;

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

// ---------------------------------------------------------------------------
// Bucle principal: grab() bloqueante -> siempre el frame mas reciente.
// ---------------------------------------------------------------------------
void GaitRecorderNode::processFrame()
{
  const sl::ERROR_CODE grab_state = zed_.grab(runtime_params_);

  if (grab_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
    if (svo_loop_) {
      RCLCPP_INFO(get_logger(), "Fin del SVO. Reiniciando reproduccion.");
      zed_.setSVOPosition(0);
      return;
    }
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

  publishCompressed(img);
}

void GaitRecorderNode::publishCompressed(const cv::Mat & img)
{
  // imencode JPEG es lo caro del preview: sin suscriptores (nadie mirando
  // el monitor web) no se codifica.
  if (pub_compressed_->get_subscription_count() == 0) {return;}
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

// ---------------------------------------------------------------------------
// Control de grabacion: true inicia, false detiene. Idempotente.
// ---------------------------------------------------------------------------
void GaitRecorderNode::cbRecord(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    // La interfaz publica patient_id, phase y record=true en rafaga por
    // topics DISTINTOS y DDS no garantiza orden entre topics: si este true
    // se procesa primero, la sesion arrancaria con metadatos viejos o
    // vacios (visto: "Sin patient_id cacheado" + arranque con la fase de
    // la sesion anterior). Se retrasa el arranque un margen corto para que
    // los metadatos de ESTA sesion esten cacheados.
    if (!recording_ && !pending_start_) {
      pending_start_ = create_wall_timer(
        std::chrono::milliseconds(start_delay_ms_),
        [this]() {
          pending_start_.reset();  // one-shot
          startRecording();
        });
    }
  } else {
    if (pending_start_) {
      pending_start_.reset();  // stop antes de que arrancara: se cancela
    }
    stopRecording();
  }
}

void GaitRecorderNode::cbPatientId(const std_msgs::msg::String::SharedPtr msg)
{
  patient_id_ = msg->data;
}

void GaitRecorderNode::cbPhase(const std_msgs::msg::String::SharedPtr msg)
{
  phase_ = msg->data;
}

void GaitRecorderNode::startRecording()
{
  if (recording_) {
    RCLCPP_WARN(
      get_logger(), "[%s] Ya hay una grabacion activa; se ignora la orden",
      camera_alias_.c_str());
    return;
  }
  if (patient_id_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "[%s] Sin patient_id cacheado: la interfaz debe publicarlo antes de "
      "grabar; no se inicia", camera_alias_.c_str());
    return;
  }

  const std::string id = sanitizeId(patient_id_);
  // Fase en minusculas: misma convencion que el .txt del servidor
  // (<id>_<pre|post>_<sello>.txt).
  const std::string phase = phase_.empty() ? "na" : phase_;
  // Directorio: parametro si se indico; si no, el que eligio la interfaz
  // (config de gait_monitor_server.py); si no hay config, el historico.
  std::string base_dir = expandUser(recording_directory_);
  if (base_dir.empty()) {
    base_dir = configuredRecordsDir();
  }
  if (base_dir.empty()) {
    base_dir = expandUser("~/Documents/patient_records");
  }
  const std::string dir = base_dir + "/" + id;

  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) {
    RCLCPP_ERROR(
      get_logger(), "[%s] No se pudo crear el directorio %s: %s",
      camera_alias_.c_str(), dir.c_str(), ec.message().c_str());
    return;
  }
  const auto space = fs::space(dir, ec);
  if (!ec && space.available < (min_free_space_mb_ << 20U)) {
    RCLCPP_ERROR(
      get_logger(), "[%s] Espacio libre insuficiente en %s (minimo %lu MB)",
      camera_alias_.c_str(), dir.c_str(), min_free_space_mb_);
    return;
  }

  if (session_pending_) {
    RCLCPP_WARN(
      get_logger(),
      "[%s] La sesion anterior quedo sin aceptar/descartar; su .txt no se "
      "escribio", camera_alias_.c_str());
  }

  session_stamp_ = currentStamp();
  // El alias (= zed<serial> por defecto) hace unico el fichero por camara:
  // sin el, dos nodos grabarian sobre la misma ruta y el SVO2 quedaria
  // corrupto (ilegible en ZED Explorer).
  session_base_ =
    dir + "/" + id + "_" + phase + "_" + session_stamp_ + "_" + camera_alias_;
  const std::string svo_file = session_base_ + ".svo2";

  sl::SVO_COMPRESSION_MODE mode = sl::SVO_COMPRESSION_MODE::H265;
  if (svo_compression_ == "H264") {
    mode = sl::SVO_COMPRESSION_MODE::H264;
  }
  sl::RecordingParameters rec_params(svo_file.c_str(), mode);
  sl::ERROR_CODE err = zed_.enableRecording(rec_params);
  if (err != sl::ERROR_CODE::SUCCESS &&
    mode == sl::SVO_COMPRESSION_MODE::H265)
  {
    RCLCPP_WARN(
      get_logger(), "SVO H265 no disponible (%s); probando H264",
      sl::toString(err).c_str());
    rec_params = sl::RecordingParameters(
      svo_file.c_str(), sl::SVO_COMPRESSION_MODE::H264);
    err = zed_.enableRecording(rec_params);
  }
  if (err != sl::ERROR_CODE::SUCCESS) {
    RCLCPP_ERROR(
      get_logger(), "[%s] No se pudo iniciar la grabacion SVO: %s",
      camera_alias_.c_str(), sl::toString(err).c_str());
    publishState(false);
    return;
  }

  recording_ = true;
  session_pending_ = false;
  session_start_ = std::chrono::steady_clock::now();
  publishState(true);
  RCLCPP_INFO(
    get_logger(), "[%s] Grabacion iniciada: %s", camera_alias_.c_str(),
    svo_file.c_str());
}

void GaitRecorderNode::stopRecording()
{
  if (!recording_) {
    return;
  }
  zed_.disableRecording();
  recording_ = false;
  session_duration_s_ = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - session_start_).count();
  // Pendiente de accept: el .txt se escribe (o el .svo2 se borra) cuando
  // la interfaz publique /gait_session_accept.
  session_pending_ = true;
  publishState(false);
  // Evidencia en el log de que el SVO2 ya esta en disco (y su tamano) en el
  // momento del stop; si falta o esta vacio, se ve aqui.
  std::error_code ec_size;
  const auto svo_size =
    fs::file_size(session_base_ + ".svo2", ec_size);
  if (ec_size) {
    RCLCPP_ERROR(
      get_logger(), "[%s] Tras parar, el SVO2 no existe: %s.svo2",
      camera_alias_.c_str(), session_base_.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "[%s] Grabacion detenida (%.1f s): %s.svo2 guardado (%.1f MB); "
      "pendiente de aceptar/descartar",
      camera_alias_.c_str(), session_duration_s_, session_base_.c_str(),
      static_cast<double>(svo_size) / 1048576.0);
  }
}

void GaitRecorderNode::cbAccept(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!session_pending_) {
    RCLCPP_WARN(
      get_logger(), "[%s] accept recibido sin sesion pendiente; se ignora",
      camera_alias_.c_str());
    return;
  }
  session_pending_ = false;

  const std::string svo_file = session_base_ + ".svo2";
  if (!msg->data) {
    std::error_code ec;
    fs::remove(svo_file, ec);
    RCLCPP_INFO(
      get_logger(), "[%s] Sesion descartada: %s eliminado",
      camera_alias_.c_str(), svo_file.c_str());
    return;
  }

  // Aceptada: el SVO2 ya esta en disco desde el stop. El .txt de
  // observaciones lo escribe el servidor del monitor (POST
  // /api/session/notes); aqui no se duplica.
  RCLCPP_INFO(
    get_logger(), "[%s] Sesion aceptada: %s", camera_alias_.c_str(),
    svo_file.c_str());
}

void GaitRecorderNode::publishState(bool recording)
{
  std_msgs::msg::Bool msg;
  msg.data = recording;
  pub_state_->publish(msg);
}

}  // namespace zed_gait
