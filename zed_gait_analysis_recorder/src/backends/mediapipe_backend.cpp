#include "zed_gait_analysis_recorder/backends/mediapipe_backend.hpp"

#include <spawn.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <string>
#include <thread>
#include <utility>
#include <vector>

extern char ** environ;

namespace zed_gait
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("mediapipe_backend");

constexpr uint32_t kMagic = 0x4D504931;  // "MPI1"
constexpr size_t kNumLandmarks = 33;
constexpr int kDepthWindowRadius = 2;  // ventana 5x5
constexpr int kWorkerStartTimeoutS = 30;

// Topologia BlazePose: 33 nombres y 35 conexiones, verificados contra
// PoseLandmark y PoseLandmarksConnections.POSE_LANDMARKS del paquete
// mediapipe (google-ai-edge/mediapipe).
SkeletonTopology buildBlazePoseTopology()
{
  SkeletonTopology topology;
  topology.joint_names = {
    "nose", "left_eye_inner", "left_eye", "left_eye_outer",
    "right_eye_inner", "right_eye", "right_eye_outer",
    "left_ear", "right_ear", "mouth_left", "mouth_right",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_pinky", "right_pinky",
    "left_index", "right_index", "left_thumb", "right_thumb",
    "left_hip", "right_hip", "left_knee", "right_knee",
    "left_ankle", "right_ankle", "left_heel", "right_heel",
    "left_foot_index", "right_foot_index"};
  topology.bones = {
    {0, 1}, {1, 2}, {2, 3}, {3, 7}, {0, 4}, {4, 5}, {5, 6}, {6, 8}, {9, 10},
    {11, 12}, {11, 13}, {13, 15}, {15, 17}, {15, 19}, {15, 21}, {17, 19},
    {12, 14}, {14, 16}, {16, 18}, {16, 20}, {16, 22}, {18, 20},
    {11, 23}, {12, 24}, {23, 24}, {23, 25}, {24, 26}, {25, 27}, {26, 28},
    {27, 29}, {28, 30}, {29, 31}, {30, 32}, {27, 31}, {28, 32}};
  return topology;
}

}  // namespace

MediaPipeBackend::MediaPipeBackend()
: logger_(rclcpp::get_logger("mediapipe_backend"))
{
}

MediaPipeBackend::~MediaPipeBackend()
{
  stopWorker();
}

bool MediaPipeBackend::init(
  sl::Camera & camera, const BackendConfig & config, std::string & error)
{
  // Intrinsecos de la imagen izquierda rectificada (back-proyeccion).
  const auto left_cam =
    camera.getCameraInformation()
    .camera_configuration.calibration_parameters.left_cam;
  fx_ = left_cam.fx;
  fy_ = left_cam.fy;
  cx_ = left_cam.cx;
  cy_ = left_cam.cy;
  depth_min_m_ = config.depth_min_m;
  depth_max_m_ = config.depth_max_m;
  confidence_threshold_ = config.confidence_threshold;
  topology_ = buildBlazePoseTopology();
  model_id_ = "mediapipe_" + config.model_variant;

  socket_path_ = "/tmp/zg_mp_" + std::to_string(::getpid()) + ".sock";

  const std::string threshold = std::to_string(config.confidence_threshold);
  const std::vector<std::string> args = {
    config.script_path,
    "--socket", socket_path_,
    "--model", config.model_path,
    "--min-pose-detection-confidence", threshold,
    "--min-pose-presence-confidence", threshold,
    "--min-tracking-confidence", threshold,
  };
  std::vector<char *> argv;
  argv.push_back(const_cast<char *>(config.python_path.c_str()));
  for (const auto & arg : args) {
    argv.push_back(const_cast<char *>(arg.c_str()));
  }
  argv.push_back(nullptr);

  // posix_spawnp: fork+exec seguro en un proceso con hilos (DDS). El worker
  // hereda stdout/stderr: sus errores se ven en el log del nodo.
  const int rc = ::posix_spawnp(
    &worker_pid_, config.python_path.c_str(), nullptr, nullptr, argv.data(),
    environ);
  if (rc != 0) {
    error = "posix_spawnp('" + config.python_path + "') fallo: " +
      std::string(std::strerror(rc));
    return false;
  }

  RCLCPP_INFO(
    logger_, "Worker lanzado (pid %d): %s %s", static_cast<int>(worker_pid_),
    config.python_path.c_str(), config.script_path.c_str());
  return connectWorker(error);
}

bool MediaPipeBackend::connectWorker(std::string & error)
{
  socket_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    error = "socket(AF_UNIX) fallo";
    return false;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  // El worker tarda unos segundos en cargar el modelo: reintentos con
  // vigilancia de que el proceso siga vivo.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::seconds(kWorkerStartTimeoutS);
  while (true) {
    if (::connect(
        socket_fd_, reinterpret_cast<sockaddr *>(&addr),
        sizeof(addr)) == 0)
    {
      break;
    }
    int status = 0;
    if (::waitpid(worker_pid_, &status, WNOHANG) == worker_pid_) {
      worker_pid_ = -1;
      error = "el worker termino al arrancar (¿mediapipe instalado en el "
              "python indicado? ¿existe el modelo " + socket_path_ + "?)";
      stopWorker();
      return false;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      error = "timeout esperando al worker en " + socket_path_;
      stopWorker();
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // Handshake [u32 magic][u32 version] con timeout; luego, sin timeout.
  const timeval rcv_timeout{kWorkerStartTimeoutS, 0};
  ::setsockopt(
    socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));
  uint32_t handshake[2] = {0, 0};
  const bool ok = recvExact(handshake, sizeof(handshake));
  const timeval no_timeout{0, 0};
  ::setsockopt(
    socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &no_timeout, sizeof(no_timeout));
  if (!ok || handshake[0] != kMagic) {
    error = "handshake del worker invalido";
    stopWorker();
    return false;
  }
  RCLCPP_INFO(logger_, "Worker conectado (protocolo v%u)", handshake[1]);
  return true;
}

bool MediaPipeBackend::sendExact(const void * data, size_t nbytes)
{
  const auto * ptr = static_cast<const char *>(data);
  size_t sent = 0;
  while (sent < nbytes) {
    // MSG_NOSIGNAL: un worker muerto no debe tumbar el nodo con SIGPIPE.
    const ssize_t n =
      ::send(socket_fd_, ptr + sent, nbytes - sent, MSG_NOSIGNAL);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

bool MediaPipeBackend::recvExact(void * data, size_t nbytes)
{
  auto * ptr = static_cast<char *>(data);
  size_t received = 0;
  while (received < nbytes) {
    const ssize_t n = ::recv(socket_fd_, ptr + received, nbytes - received, 0);
    if (n <= 0) {
      return false;
    }
    received += static_cast<size_t>(n);
  }
  return true;
}

void MediaPipeBackend::workerDied(const char * where)
{
  RCLCPP_ERROR(
    logger_, "Worker MediaPipe no disponible en %s; backend inactivo",
    where);
  stopWorker();
}

void MediaPipeBackend::stopWorker()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);  // EOF: el worker termina solo
    socket_fd_ = -1;
  }
  if (worker_pid_ > 0) {
    bool exited = false;
    for (int i = 0; i < 50 && !exited; ++i) {
      int status = 0;
      exited = (::waitpid(worker_pid_, &status, WNOHANG) == worker_pid_);
      if (!exited) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    if (!exited) {
      ::kill(worker_pid_, SIGTERM);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      int status = 0;
      if (::waitpid(worker_pid_, &status, WNOHANG) != worker_pid_) {
        ::kill(worker_pid_, SIGKILL);
        ::waitpid(worker_pid_, &status, 0);
      }
    }
    worker_pid_ = -1;
  }
  if (!socket_path_.empty()) {
    ::unlink(socket_path_.c_str());
    socket_path_.clear();
  }
}

PoseResult MediaPipeBackend::infer(sl::Camera & camera, const cv::Mat & bgr)
{
  PoseResult result;
  if (socket_fd_ < 0) {
    return result;  // worker caido: ya se aviso; el nodo sigue publicando
  }

  // Timestamp del frame: el modo VIDEO exige ms estrictamente crecientes.
  const uint64_t ts_ns =
    camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
  result.frame_ts_ns = static_cast<int64_t>(ts_ns);
  int64_t timestamp_ms = static_cast<int64_t>(ts_ns / 1000000ULL);
  if (timestamp_ms <= last_timestamp_ms_) {
    timestamp_ms = last_timestamp_ms_ + 1;
  }
  last_timestamp_ms_ = timestamp_ms;

  // Enviar [u32 width][u32 height][i64 timestamp_ms] + frame BGR.
  const uint32_t width = static_cast<uint32_t>(bgr.cols);
  const uint32_t height = static_cast<uint32_t>(bgr.rows);
  char header[16];
  std::memcpy(header, &width, 4);
  std::memcpy(header + 4, &height, 4);
  std::memcpy(header + 8, &timestamp_ms, 8);
  const size_t frame_bytes = static_cast<size_t>(width) * height * 3;
  if (!sendExact(header, sizeof(header)) ||
    !sendExact(bgr.data, frame_bytes))
  {
    workerDied("send");
    return result;
  }

  // Recibir [u32 num_poses] y, si hay persona, 33 x [f32 x][f32 y][f32 vis].
  uint32_t num_poses = 0;
  if (!recvExact(&num_poses, sizeof(num_poses))) {
    workerDied("recv");
    return result;
  }
  if (num_poses == 0) {
    return result;
  }
  float landmarks[kNumLandmarks * 3];
  if (!recvExact(landmarks, sizeof(landmarks))) {
    workerDied("recv");
    return result;
  }

  // Profundidad del mismo grab para la back-proyeccion.
  if (camera.retrieveMeasure(depth_map_, sl::MEASURE::DEPTH) !=
    sl::ERROR_CODE::SUCCESS)
  {
    static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      kLogger, throttle_clock, 2000, "retrieveMeasure fallo");
  }

  PersonPose person;
  person.body_id = 0;  // num_poses = 1 fijo (resp. 23/31)
  person.joints.resize(kNumLandmarks);
  float confidence_sum = 0.0F;

  for (size_t i = 0; i < kNumLandmarks; ++i) {
    const float xn = landmarks[i * 3];
    const float yn = landmarks[i * 3 + 1];
    Joint joint;
    joint.confidence = std::clamp(landmarks[i * 3 + 2], 0.0F, 1.0F);
    confidence_sum += joint.confidence;

    const int u = std::clamp(
      static_cast<int>(std::lround(xn * width)), 0,
      static_cast<int>(width) - 1);
    const int v = std::clamp(
      static_cast<int>(std::lround(yn * height)), 0,
      static_cast<int>(height) - 1);
    joint.u = static_cast<float>(u);
    joint.v = static_cast<float>(v);

    if (joint.confidence >= confidence_threshold_) {
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      if (depthAt(u, v, x, y, z)) {
        joint.x = x;
        joint.y = y;
        joint.z = z;
        joint.valid = true;
      }
    }
    person.joints[i] = joint;
  }
  person.confidence = confidence_sum / static_cast<float>(kNumLandmarks);
  result.persons.push_back(std::move(person));
  return result;
}

bool MediaPipeBackend::depthAt(int u, int v, float & x, float & y, float & z)
const
{
  const int w = static_cast<int>(depth_map_.getWidth());
  const int h = static_cast<int>(depth_map_.getHeight());

  // Mediana de los Z validos de la ventana 5x5 (resp. 37 autorizo vecindad).
  float window[25];
  size_t count = 0;
  for (int dv = -kDepthWindowRadius; dv <= kDepthWindowRadius; ++dv) {
    for (int du = -kDepthWindowRadius; du <= kDepthWindowRadius; ++du) {
      const int uu = u + du;
      const int vv = v + dv;
      if (uu < 0 || vv < 0 || uu >= w || vv >= h) {
        continue;
      }
      float depth = 0.0F;
      if (depth_map_.getValue<float>(
          static_cast<size_t>(uu), static_cast<size_t>(vv),
          &depth) != sl::ERROR_CODE::SUCCESS)
      {
        continue;
      }
      if (std::isfinite(depth) && depth >= depth_min_m_ &&
        depth <= depth_max_m_)
      {
        window[count++] = depth;
      }
    }
  }
  if (count == 0) {
    return false;
  }
  std::nth_element(window, window + count / 2, window + count);
  z = window[count / 2];
  x = (static_cast<float>(u) - cx_) * z / fx_;
  y = (static_cast<float>(v) - cy_) * z / fy_;
  return true;
}

}  // namespace zed_gait
