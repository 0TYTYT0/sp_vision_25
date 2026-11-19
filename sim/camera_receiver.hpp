#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace sim
{
enum class ProgramStatus : std::uint16_t
{
  pose_result = 2,
  processing_error = 3,
};

struct Corner
{
  float image_x;
  float image_y;
};

struct ArmorObservation
{
  std::uint64_t id;
  std::int32_t team;
  std::int32_t robot;
  std::int32_t armor;
  std::array<Corner, 4> corners;
};

struct CameraFrame
{
  std::uint64_t id = 0;
  std::uint64_t connection_id = 0;
  std::int64_t capture_time_us = 0;
  std::array<float, 3> camera_position;
  std::array<float, 4> camera_rotation;
  std::vector<ArmorObservation> armors;
};

struct PoseStatus
{
  ProgramStatus status = ProgramStatus::pose_result;
  std::uint32_t solved = 0;
  std::uint32_t processing_us = 0;
  std::uint64_t target_id = 0;
  float distance = 0;
  std::array<float, 3> world_position{};
  std::array<float, 3> camera_rotation_vector{};
};

class CameraReceiver
{
public:
  explicit CameraReceiver(std::uint16_t port);
  ~CameraReceiver();

  CameraReceiver(const CameraReceiver &) = delete;
  CameraReceiver & operator=(const CameraReceiver &) = delete;

  std::optional<CameraFrame> wait_for_frame();
  void publish_status(const CameraFrame & frame, const PoseStatus & status);

private:
  class Client;
  std::shared_ptr<Client> client_;
  std::atomic_bool stopping_{false};
  std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<CameraFrame> latest_;
  std::exception_ptr failure_;
  std::thread worker_;

  void receive(std::uint16_t port);
};
}  // namespace sim
