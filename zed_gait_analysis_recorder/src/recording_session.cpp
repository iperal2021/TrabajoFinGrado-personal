#include "zed_gait_analysis_recorder/recording_session.hpp"

#include <filesystem>
#include <iomanip>
#include <system_error>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace zed_gait
{

namespace fs = std::filesystem;

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("recording_session");
}  // namespace

RecordingSession::~RecordingSession()
{
  stop();
}

bool RecordingSession::hasFreeSpace() const
{
  std::error_code ec;
  const auto space = fs::space(directory_, ec);
  if (ec) {
    RCLCPP_WARN(
      kLogger, "No se pudo consultar el espacio libre de %s: %s",
      directory_.c_str(), ec.message().c_str());
    return true;  // no se bloquea la grabacion por no poder medir
  }
  return space.available >= (min_free_space_mb_ << 20U);
}

bool RecordingSession::start(
  const SessionConfig & config, sl::Camera & camera,
  const SkeletonTopology & topology)
{
  if (state_ == State::RECORDING) {
    RCLCPP_WARN(kLogger, "Ya hay una sesion activa; se ignora el inicio");
    return false;
  }

  directory_ = config.directory;
  min_free_space_mb_ = config.min_free_space_mb;
  model_id_ = config.model_id;
  camera_serial_ = config.camera_serial;
  camera_ = &camera;

  std::error_code ec;
  fs::create_directories(directory_, ec);
  if (ec) {
    RCLCPP_ERROR(
      kLogger, "No se pudo crear el directorio %s: %s", directory_.c_str(),
      ec.message().c_str());
    return false;
  }
  if (!hasFreeSpace()) {
    RCLCPP_ERROR(
      kLogger, "Espacio libre insuficiente en %s (minimo %lu MB)",
      directory_.c_str(), min_free_space_mb_);
    return false;
  }

  // Nombres: .partial hasta el cierre correcto (ADR-005). El .partial va
  // ANTES de la extension real: el backend FFmpeg de OpenCV deduce el
  // contenedor por la extension (un ".mp4.partial" no abre).
  const std::string base =
    directory_ + "/" + config.stamp + "_" + config.camera_alias + "_pose";
  csv_final_ = base + ".csv";
  csv_tmp_ = base + ".partial.csv";
  video_final_ = base + ".mp4";
  video_tmp_ = base + ".partial.mp4";
  svo_final_ = base + ".svo2";
  svo_tmp_ = base + ".partial.svo2";

  // ---- CSV con cabecera ----
  csv_.open(csv_tmp_, std::ios::out | std::ios::trunc);
  if (!csv_) {
    RCLCPP_ERROR(kLogger, "No se pudo abrir el CSV %s", csv_tmp_.c_str());
    return false;
  }
  csv_ << std::fixed;
  csv_ << "time_s,frame_ts_ns";
  for (const auto & name : topology.joint_names) {
    csv_ << ',' << name << "_x," << name << "_y," << name << "_z," << name
         << "_conf";
  }
  csv_ << ",body_id,ai_model,camera_sn\n";

  // ---- Video: NVENC via GStreamer, fallback a software (ADR-004) ----
  if (config.video_hw_encoder) {
    const std::string pipeline =
      "appsrc ! videoconvert ! video/x-raw,format=I420 ! nvv4l2h264enc "
      "bitrate=" + std::to_string(config.video_bitrate) +
      " insert-sps-pps=true ! h264parse ! mp4mux ! filesink location=" +
      video_tmp_;
    video_.open(
      pipeline, cv::CAP_GSTREAMER, 0.0, config.video_fps, config.video_size,
      true);
    if (video_.isOpened()) {
      RCLCPP_INFO(kLogger, "Codec de video: H.264 por hardware (NVENC)");
    }
  }
  if (!video_.isOpened()) {
    RCLCPP_WARN(
      kLogger,
      "NVENC no disponible; codificando por software (mp4v via FFmpeg). "
      "En la Jetson revisar 'gst-inspect-1.0 nvv4l2h264enc'.");
    // CAP_FFMPEG explicito: con CAP_GSTREAMER el fallback muere en runtime
    // si faltan elementos de codificacion (visto en OpenCV 4.5.4 de apt).
    // Objeto fresco: reutilizar un VideoWriter con un open() fallido es
    // propenso a errores.
    video_ = cv::VideoWriter();
    video_.open(
      video_tmp_, cv::CAP_FFMPEG, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
      config.video_fps, config.video_size, true);
  }
  if (!video_.isOpened()) {
    RCLCPP_ERROR(kLogger, "No se pudo abrir el VideoWriter");
    csv_.close();
    // Inicio fallido: no se deja ningun artefacto atras.
    std::error_code ec;
    fs::remove(csv_tmp_, ec);
    return false;
  }

  // ---- SVO nativo (imagen 2D + profundidad) ----
  if (config.svo_enabled) {
    sl::SVO_COMPRESSION_MODE mode = sl::SVO_COMPRESSION_MODE::H265;
    if (config.svo_compression == "H264") {
      mode = sl::SVO_COMPRESSION_MODE::H264;
    }
    sl::RecordingParameters rec_params(svo_tmp_.c_str(), mode);
    sl::ERROR_CODE rec_err = camera.enableRecording(rec_params);
    if (rec_err != sl::ERROR_CODE::SUCCESS &&
      mode == sl::SVO_COMPRESSION_MODE::H265)
    {
      RCLCPP_WARN(
        kLogger, "SVO H265 no disponible (%s); probando H264",
        sl::toString(rec_err).c_str());
      rec_params = sl::RecordingParameters(
        svo_tmp_.c_str(), sl::SVO_COMPRESSION_MODE::H264);
      rec_err = camera.enableRecording(rec_params);
    }
    if (rec_err == sl::ERROR_CODE::SUCCESS) {
      svo_active_ = true;
    } else {
      // Se sigue sin SVO pero el fallo queda registrado (ADR-015).
      RCLCPP_WARN(
        kLogger, "SVO desactivado para esta sesion: %s",
        sl::toString(rec_err).c_str());
    }
  }

  rows_since_flush_ = 0;
  frames_since_space_check_ = 0;
  video_frames_written_ = 0;
  session_start_ = std::chrono::steady_clock::now();
  state_ = State::RECORDING;

  RCLCPP_INFO(
    kLogger, "Sesion iniciada: %s [.csv/.mp4%s] (video %dx%d @ %.0f fps)",
    base.c_str(), svo_active_ ? "/.svo2" : "", config.video_size.width,
    config.video_size.height, config.video_fps);
  return true;
}

void RecordingSession::writeFrame(const cv::Mat & bgr)
{
  if (state_ != State::RECORDING || !video_.isOpened()) {return;}

  video_.write(bgr);
  ++video_frames_written_;

  // Chequeo periodico de espacio: cv::VideoWriter no reporta disco lleno.
  if (++frames_since_space_check_ >= 60) {
    frames_since_space_check_ = 0;
    if (!hasFreeSpace()) {
      fail("Disco casi lleno: se detiene la sesion");
    }
  }
}

void RecordingSession::writeRow(
  double time_s, int64_t frame_ts_ns, const PersonPose * person,
  const SkeletonTopology & topology)
{
  if (state_ != State::RECORDING) {return;}

  csv_ << std::setprecision(3) << time_s << ',' << frame_ts_ns;

  const size_t n = topology.joint_names.size();
  for (size_t i = 0; i < n; ++i) {
    if (person != nullptr && i < person->joints.size()) {
      const Joint & joint = person->joints[i];
      csv_ << ',' << std::setprecision(4) << joint.x
           << ',' << std::setprecision(4) << joint.y
           << ',' << std::setprecision(4) << joint.z
           << ',' << std::setprecision(2) << joint.confidence;
    } else {
      // Sin persona (o joint ausente): fila neutral que mantiene la
      // alineacion temporal entre camaras.
      csv_ << ",0,0,0,0";
    }
  }
  csv_ << ',' << (person != nullptr ? person->body_id : -1)
       << ',' << model_id_ << ',' << camera_serial_ << '\n';

  // Flush cada ~1 s (20 filas a 20 Hz) para no bloquear el bucle.
  if (++rows_since_flush_ >= 20) {
    csv_.flush();
    rows_since_flush_ = 0;
  }
  if (!csv_.good()) {
    fail("Escritura de CSV fallida (¿disco lleno?)");
  }
}

void RecordingSession::fail(const std::string & message)
{
  RCLCPP_ERROR(kLogger, "%s", message.c_str());
  state_ = State::ERROR;
  closeAndRename(false);  // se conservan los .partial como señal de incompleto
}

void RecordingSession::closeAndRename(bool rename)
{
  if (csv_.is_open()) {
    csv_.flush();
    csv_.close();
  }
  if (video_.isOpened()) {
    video_.release();
  }
  if (svo_active_ && camera_ != nullptr) {
    camera_->disableRecording();
    svo_active_ = false;
  }

  if (rename) {
    const std::pair<const std::string *, const std::string *> pairs[] = {
      {&csv_tmp_, &csv_final_},
      {&video_tmp_, &video_final_},
      {&svo_tmp_, &svo_final_},
    };
    for (const auto & [tmp, final] : pairs) {
      std::error_code ec;
      if (fs::exists(*tmp, ec)) {
        fs::rename(*tmp, *final, ec);
        if (ec) {
          RCLCPP_WARN(
            kLogger, "No se pudo renombrar %s: %s", tmp->c_str(),
            ec.message().c_str());
        }
      }
    }
  }
}

void RecordingSession::stop()
{
  if (state_ == State::IDLE) {return;}

  const bool clean = (state_ == State::RECORDING);
  closeAndRename(clean);
  state_ = State::IDLE;

  if (clean) {
    const double elapsed_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - session_start_).count();
    RCLCPP_INFO(
      kLogger,
      "Sesion cerrada: %s (%lu frames de video en %.1f s = %.1f fps "
      "efectivos)", csv_final_.c_str(), video_frames_written_, elapsed_s,
      elapsed_s > 0.0 ? video_frames_written_ / elapsed_s : 0.0);
  } else {
    RCLCPP_WARN(
      kLogger, "Sesion cerrada en ERROR; quedan archivos .partial");
  }
}

}  // namespace zed_gait
