#include "camera_receiver.hpp"

#include <winsock2.h>

#include <array>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace sim
{
namespace
{
constexpr std::size_t header_size = 32;
constexpr std::uint32_t max_frame_size = 16 * 1024 * 1024;

std::runtime_error socket_error(const char * operation)
{
  return std::runtime_error(
    std::string(operation) + " failed (Winsock " + std::to_string(WSAGetLastError()) + ")");
}

class Winsock
{
public:
  Winsock()
  {
    WSADATA data{};
    const int error = WSAStartup(MAKEWORD(2, 2), &data);
    if (error != 0) {
      throw std::runtime_error("WSAStartup failed (" + std::to_string(error) + ")");
    }
  }

  ~Winsock() { WSACleanup(); }
};

class Socket
{
public:
  explicit Socket(SOCKET socket) : socket_(socket) {}
  ~Socket()
  {
    if (socket_ != INVALID_SOCKET) {
      closesocket(socket_);
    }
  }

  Socket(const Socket &) = delete;
  Socket & operator=(const Socket &) = delete;

  SOCKET get() const { return socket_; }

private:
  SOCKET socket_;
};

int poll_read(SOCKET socket)
{
  fd_set sockets;
  FD_ZERO(&sockets);
  FD_SET(socket, &sockets);
  timeval timeout{0, 100000};
  return select(0, &sockets, nullptr, nullptr, &timeout);
}

bool read_exact(
  SOCKET socket, std::uint8_t * bytes, std::size_t size, const std::atomic_bool & stopping)
{
  while (size != 0 && !stopping) {
    const int readable = poll_read(socket);
    if (readable < 0) {
      return false;
    }
    if (readable == 0) {
      continue;
    }

    const int received = recv(socket, reinterpret_cast<char *>(bytes), static_cast<int>(size), 0);
    if (received <= 0) {
      return false;
    }
    bytes += received;
    size -= received;
  }
  return size == 0;
}

template <typename Integer>
Integer read_little_endian(const std::uint8_t * bytes)
{
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < sizeof(Integer); ++i) {
    value |= static_cast<std::uint64_t>(bytes[i]) << (8 * i);
  }
  return static_cast<Integer>(value);
}

bool read_frame(SOCKET socket, CameraFrame & frame, const std::atomic_bool & stopping)
{
  std::array<std::uint8_t, header_size> header;
  if (!read_exact(socket, header.data(), header.size(), stopping)) {
    return false;
  }

  const auto * bytes = header.data();
  if (
    header[0] != 'R' || header[1] != 'B' || header[2] != 'T' || header[3] != 'F' ||
    read_little_endian<std::uint16_t>(bytes + 4) != 1 ||
    read_little_endian<std::uint16_t>(bytes + 6) != 1) {
    return false;
  }

  const auto capture_time = read_little_endian<std::uint64_t>(bytes + 16);
  frame.width = read_little_endian<std::uint16_t>(bytes + 24);
  frame.height = read_little_endian<std::uint16_t>(bytes + 26);
  const auto payload_size = read_little_endian<std::uint32_t>(bytes + 28);
  const auto expected_size = static_cast<std::uint64_t>(frame.width) * frame.height * 3;
  if (
    capture_time > std::numeric_limits<std::int64_t>::max() || expected_size == 0 ||
    payload_size != expected_size || payload_size > max_frame_size) {
    return false;
  }

  frame.id = read_little_endian<std::uint64_t>(bytes + 8);
  frame.capture_time_us = static_cast<std::int64_t>(capture_time);
  frame.bgr.resize(payload_size);
  return read_exact(socket, frame.bgr.data(), frame.bgr.size(), stopping);
}
}  // namespace

CameraReceiver::CameraReceiver(std::uint16_t port)
: worker_([this, port] {
    try {
      receive(port);
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        failure_ = std::current_exception();
      }
      ready_.notify_one();
    }
  })
{
}

CameraReceiver::~CameraReceiver()
{
  stopping_ = true;
  worker_.join();
}

std::optional<CameraFrame> CameraReceiver::wait_for_frame()
{
  std::unique_lock<std::mutex> lock(mutex_);
  ready_.wait_for(lock, std::chrono::milliseconds(20), [this] { return latest_ || failure_; });
  if (failure_) {
    std::rethrow_exception(failure_);
  }
  return std::exchange(latest_, std::nullopt);
}

void CameraReceiver::receive(std::uint16_t port)
{
  Winsock winsock;
  Socket listener(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
  if (listener.get() == INVALID_SOCKET) {
    throw socket_error("socket");
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(port);
  if (bind(listener.get(), reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0) {
    throw socket_error("bind");
  }
  if (listen(listener.get(), 1) != 0) {
    throw socket_error("listen");
  }

  while (!stopping_) {
    const int readable = poll_read(listener.get());
    if (readable < 0) {
      throw socket_error("select");
    }
    if (readable == 0) {
      continue;
    }

    Socket client(accept(listener.get(), nullptr, nullptr));
    if (client.get() == INVALID_SOCKET) {
      continue;
    }

    CameraFrame frame;
    while (read_frame(client.get(), frame, stopping_)) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto previous = std::exchange(latest_, std::move(frame));
        if (previous) {
          frame.bgr = std::move(previous->bgr);
        }
      }
      ready_.notify_one();
    }
  }
}
}  // namespace sim
