#include <windows.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "camera_receiver.hpp"

namespace
{
std::atomic_bool stop_requested{false};

BOOL WINAPI handle_console_event(DWORD event)
{
  if (event == CTRL_C_EVENT || event == CTRL_BREAK_EVENT) {
    stop_requested = true;
    return TRUE;
  }
  return FALSE;
}

std::uint16_t read_port(int argc, char ** argv)
{
  if (argc == 1) {
    return 5600;
  }
  if (argc != 3 || std::string(argv[1]) != "--port") {
    throw std::invalid_argument("Usage: robottest_vision [--port PORT]");
  }
  const std::string value = argv[2];
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("Port must be between 1 and 65535");
  }
  const auto port = std::stoul(value);
  if (port == 0 || port > std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("Port must be between 1 and 65535");
  }
  return static_cast<std::uint16_t>(port);
}

struct CameraPose
{
  cv::Vec3d position;
  cv::Vec3d rotation_vector;
};

struct Solution
{
  const sim::ArmorObservation * armor;
  cv::Vec3d position;
  cv::Vec3d camera_rotation_vector;
  double distance;
};

std::optional<CameraPose> solve(const sim::ArmorObservation & armor)
{
  std::vector<cv::Point3d> model;
  std::vector<cv::Point2d> image;
  model.reserve(armor.corners.size());
  image.reserve(armor.corners.size());
  for (const auto & corner : armor.corners) {
    if (
      !std::isfinite(corner.image_x) || !std::isfinite(corner.image_y) ||
      !std::isfinite(corner.model_x) || !std::isfinite(corner.model_y)) {
      return std::nullopt;
    }
    model.emplace_back(corner.model_x, corner.model_y, 0);
    image.emplace_back(corner.image_x, corner.image_y);
  }

  std::vector<cv::Vec3d> rotations;
  std::vector<cv::Vec3d> translations;
  try {
    cv::solvePnPGeneric(
      model, image, cv::Matx33d::eye(), cv::noArray(), rotations, translations, false,
      cv::SOLVEPNP_IPPE);
  } catch (const cv::Exception &) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < translations.size(); ++i) {
    const auto & position = translations[i];
    if (!cv::checkRange(position) || position[2] <= 0 || !cv::checkRange(rotations[i])) {
      continue;
    }
    cv::Matx33d rotation;
    cv::Rodrigues(rotations[i], rotation);
    bool in_front = true;
    for (const auto & point : model) {
      if (position[2] + rotation(2, 0) * point.x + rotation(2, 1) * point.y <= 0) {
        in_front = false;
        break;
      }
    }
    if (in_front) {
      return CameraPose{position, rotations[i]};
    }
  }
  return std::nullopt;
}

std::optional<cv::Vec3d> to_world(const sim::CameraFrame & frame, const cv::Vec3d & camera)
{
  const auto & q = frame.camera_rotation;
  const cv::Vec4d rotation(q[0], q[1], q[2], q[3]);
  const double norm = cv::norm(rotation);
  const auto & p = frame.camera_position;
  const cv::Vec3d origin(p[0], p[1], p[2]);
  if (!std::isfinite(norm) || norm < 0.5 || norm > 1.5 || !cv::checkRange(origin)) {
    return std::nullopt;
  }

  // OpenCV uses +Y down. The transmitted quaternion rotates Unity camera-local axes.
  const cv::Vec3d local(camera[0], -camera[1], camera[2]);
  const cv::Vec3d axis(q[0] / norm, q[1] / norm, q[2] / norm);
  const cv::Vec3d cross = 2.0 * axis.cross(local);
  return origin + local + (q[3] / norm) * cross + axis.cross(cross);
}

int run(std::uint16_t port)
{
  sim::CameraReceiver receiver(port);
  std::cerr << "Waiting for RoboTest corner observations on 127.0.0.1:" << port << '\n';
  std::uint64_t connection = 0;
  std::optional<std::uint64_t> tracked_id;

  while (!stop_requested) {
    const auto frame = receiver.wait_for_frame();
    if (!frame) {
      continue;
    }
    if (frame->connection_id != connection) {
      connection = frame->connection_id;
      tracked_id.reset();
    }

    const auto start = std::chrono::steady_clock::now();
    sim::PoseStatus status;
    std::optional<Solution> target;
    for (const auto & armor : frame->armors) {
      const auto camera = solve(armor);
      if (!camera) {
        continue;
      }
      const auto world = to_world(*frame, camera->position);
      const double distance = cv::norm(camera->position);
      if (
        !world || !cv::checkRange(*world) || !std::isfinite(distance) ||
        distance > std::numeric_limits<float>::max()) {
        continue;
      }
      ++status.solved;
      if (
        !target || tracked_id == armor.id ||
        (tracked_id != target->armor->id && distance < target->distance)) {
        target = Solution{&armor, *world, camera->rotation_vector, distance};
      }
    }

    std::cout << "frame=" << frame->id << " capture_us=" << frame->capture_time_us
              << " observed=" << frame->armors.size() << " solved=" << status.solved;
    if (target) {
      const auto & armor = *target->armor;
      tracked_id = armor.id;
      status.target_id = armor.id;
      status.distance = static_cast<float>(target->distance);
      for (std::size_t i = 0; i < status.world_position.size(); ++i) {
        status.world_position[i] = static_cast<float>(target->position[static_cast<int>(i)]);
        status.camera_rotation_vector[i] =
          static_cast<float>(target->camera_rotation_vector[static_cast<int>(i)]);
      }
      std::cout << " target=" << armor.team << "/" << armor.robot << "/" << armor.armor
                << " distance_m=" << status.distance << " world=" << target->position
                << " camera_rvec=" << target->camera_rotation_vector;
    } else {
      tracked_id.reset();
      if (!frame->armors.empty()) {
        status.status = sim::ProgramStatus::processing_error;
      }
    }
    status.processing_us =
      static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::steady_clock::now() - start)
                                   .count());
    receiver.publish_status(*frame, status);
    std::cout << std::endl;
  }
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "robottest_vision [--port PORT]\n";
    return 0;
  }
  try {
    if (!SetConsoleCtrlHandler(handle_console_event, TRUE)) {
      throw std::runtime_error("Could not register console stop handler");
    }
    return run(read_port(argc, argv));
  } catch (const std::exception & error) {
    std::cerr << "robottest_vision: " << error.what() << '\n';
    return 1;
  }
}
