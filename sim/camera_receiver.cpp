#include "camera_receiver.hpp"

#include <winsock2.h>

#include <array>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace sim
{
namespace
{
constexpr std::size_t header_size = 108;
constexpr std::size_t observation_size = 52;
constexpr std::size_t status_size = 72;
constexpr std::uint16_t max_observations = 64;

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

template <typename Integer>
void write_little_endian(std::uint8_t * bytes, Integer value)
{
  for (std::size_t i = 0; i < sizeof(Integer); ++i) {
    bytes[i] = static_cast<std::uint8_t>(value >> (8 * i));
  }
}

float read_float(const std::uint8_t * bytes)
{
  const auto bits = read_little_endian<std::uint32_t>(bytes);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void write_float(std::uint8_t * bytes, float value)
{
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  write_little_endian(bytes, bits);
}

bool read_frame(SOCKET socket, CameraFrame & frame, const std::atomic_bool & stopping)
{
  std::array<std::uint8_t, header_size> header;
  if (!read_exact(socket, header.data(), header.size(), stopping)) {
    return false;
  }

  const auto * bytes = header.data();
  const auto count = read_little_endian<std::uint16_t>(bytes + 6);
  if (
    header[0] != 'R' || header[1] != 'B' || header[2] != 'O' || header[3] != 'B' ||
    read_little_endian<std::uint16_t>(bytes + 4) != 4 || count > max_observations) {
    return false;
  }

  frame.id = read_little_endian<std::uint64_t>(bytes + 8);
  const auto timestamp = read_little_endian<std::uint64_t>(bytes + 16);
  if (frame.id == 0 || timestamp > static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::duration::max()).count())) {
    return false;
  }
  frame.capture_time_us = static_cast<std::int64_t>(timestamp);
  for (std::size_t i = 0; i < frame.camera_position.size(); ++i) {
    frame.camera_position[i] = read_float(bytes + 24 + i * 4);
  }
  for (std::size_t i = 0; i < frame.camera_rotation.size(); ++i) {
    frame.camera_rotation[i] = read_float(bytes + 36 + i * 4);
  }

  for (std::size_t i = 0; i < frame.aim_origin.size(); ++i) {
    frame.aim_origin[i] = read_float(bytes + 52 + i * 4);
    frame.aim_up[i] = read_float(bytes + 64 + i * 4);
    frame.aim_forward[i] = read_float(bytes + 76 + i * 4);
    frame.muzzle_forward[i] = read_float(bytes + 96 + i * 4);
  }
  frame.projectile_speed = read_float(bytes + 88);
  frame.current_yaw = read_float(bytes + 92);

  frame.armors.resize(count);
  for (auto & armor : frame.armors) {
    std::array<std::uint8_t, observation_size> observation;
    if (!read_exact(socket, observation.data(), observation.size(), stopping)) {
      return false;
    }
    const auto * record = observation.data();
    armor.id = read_little_endian<std::uint64_t>(record);
    armor.team = read_little_endian<std::int32_t>(record + 8);
    armor.robot = read_little_endian<std::int32_t>(record + 12);
    armor.armor = read_little_endian<std::int32_t>(record + 16);
    for (std::size_t i = 0; i < armor.corners.size(); ++i) {
      const auto * point = record + 20 + i * 8;
      armor.corners[i] = {read_float(point), read_float(point + 4)};
    }
  }
  return true;
}
}  // namespace

class CameraReceiver::Client
{
public:
  Client(SOCKET listener, std::uint64_t id)
  : socket(accept(listener, nullptr, nullptr)), connection_id(id)
  {
  }

  Socket socket;
  const std::uint64_t connection_id;
};

CameraReceiver::CameraReceiver(std::uint16_t port)
: worker_([this, port] {
    try {
      receive(port);
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        client_.reset();
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

void CameraReceiver::publish_status(const CameraFrame & frame, const PoseStatus & status)
{
  std::shared_ptr<Client> client;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!client_ || client_->connection_id != frame.connection_id) {
      return;
    }
    client = client_;
  }

  std::array<std::uint8_t, status_size> packet{'R', 'B', 'T', 'S'};
  write_little_endian(packet.data() + 4, std::uint16_t{4});
  write_little_endian(packet.data() + 6, static_cast<std::uint16_t>(status.status));
  write_little_endian(packet.data() + 8, frame.id);
  write_little_endian(packet.data() + 16, status.solved);
  write_little_endian(packet.data() + 20, status.processing_us);
  write_little_endian(packet.data() + 24, status.target_id);
  write_float(packet.data() + 32, status.distance);
  for (std::size_t i = 0; i < status.world_position.size(); ++i) {
    write_float(packet.data() + 36 + i * 4, status.world_position[i]);
    write_float(packet.data() + 48 + i * 4, status.camera_rotation_vector[i]);
  }
  const std::uint32_t flags = (status.aim_valid ? 1u : 0u) | (status.fire_requested ? 2u : 0u);
  write_little_endian(packet.data() + 60, flags);
  write_float(packet.data() + 64, status.yaw);
  write_float(packet.data() + 68, status.pitch);
  const auto * data = reinterpret_cast<const char *>(packet.data());
  int remaining = static_cast<int>(packet.size());
  while (remaining > 0) {
    const int sent = send(client->socket.get(), data, remaining, 0);
    if (sent <= 0) {
      shutdown(client->socket.get(), SD_BOTH);
      return;
    }
    data += sent;
    remaining -= sent;
  }
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

  std::uint64_t connection_id = 0;
  while (!stopping_) {
    const int readable = poll_read(listener.get());
    if (readable < 0) {
      throw socket_error("select");
    }
    if (readable == 0) {
      continue;
    }

    auto client = std::make_shared<Client>(listener.get(), ++connection_id);
    const SOCKET socket = client->socket.get();
    if (socket == INVALID_SOCKET) {
      continue;
    }

    const DWORD send_timeout_ms = 100;
    if (
      setsockopt(
        socket, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char *>(&send_timeout_ms),
        sizeof(send_timeout_ms)) != 0) {
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      client_ = client;
    }

    CameraFrame frame;
    frame.connection_id = client->connection_id;
    while (read_frame(socket, frame, stopping_)) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto previous = std::exchange(latest_, std::move(frame));
        if (previous) {
          frame.armors = std::move(previous->armors);
        }
      }
      ready_.notify_one();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    client_.reset();
  }
}
}  // namespace sim
