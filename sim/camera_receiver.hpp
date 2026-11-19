#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace sim
{
struct CameraFrame
{
  std::uint64_t id = 0;
  std::int64_t capture_time_us = 0;
  std::uint16_t width = 0;
  std::uint16_t height = 0;
  std::vector<std::uint8_t> bgr;
};

class CameraReceiver
{
public:
  explicit CameraReceiver(std::uint16_t port);
  ~CameraReceiver();

  CameraReceiver(const CameraReceiver &) = delete;
  CameraReceiver & operator=(const CameraReceiver &) = delete;

  std::optional<CameraFrame> wait_for_frame();

private:
  std::atomic_bool stopping_{false};
  std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<CameraFrame> latest_;
  std::exception_ptr failure_;
  std::thread worker_;

  void receive(std::uint16_t port);
};
}  // namespace sim
