#include <windows.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <list>
#include <memory>
#include <iostream>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor_dimensions.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/math_tools.hpp"
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
  auto_aim::ArmorName name;
  cv::Vec3d position;
  cv::Vec3d camera_rotation_vector;
  double distance;
};

std::optional<CameraPose> solve(const sim::ArmorObservation & armor)
{
  constexpr double half_width = auto_aim::SMALL_ARMOR_WIDTH / 2;
  constexpr double half_height = auto_aim::LIGHTBAR_LENGTH / 2;
  // Match the upstream corner order while keeping the pose reply's right/down axes.
  static const std::array<cv::Point3d, 4> model{{
    {-half_width, -half_height, 0}, {half_width, -half_height, 0},
    {half_width, half_height, 0}, {-half_width, half_height, 0},
  }};
  std::array<cv::Point2d, 4> image;
  for (std::size_t i = 0; i < armor.corners.size(); ++i) {
    const auto & corner = armor.corners[i];
    if (!std::isfinite(corner.image_x) || !std::isfinite(corner.image_y)) {
      return std::nullopt;
    }
    image[i] = {corner.image_x, corner.image_y};
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

// Unity right/up/forward -> framework forward/left/up.
Eigen::Vector3d framework_point(double x, double y, double z) { return {z, -x, y}; }

auto_aim::ArmorName armor_name(int robot)
{
  if (robot >= 1 && robot <= 5) {
    return static_cast<auto_aim::ArmorName>(robot - 1);
  }
  switch (robot) {
    case 7:
      return auto_aim::sentry;
    case 10:
      return auto_aim::base;
    case 11:
      return auto_aim::outpost;
    default:
      return auto_aim::not_armor;
  }
}

std::string config_path()
{
  std::wstring filename(32768, L'\0');
  const auto length =
    GetModuleFileNameW(nullptr, filename.data(), static_cast<DWORD>(filename.size()));
  if (length == 0 || length == filename.size()) {
    throw std::runtime_error("Could not resolve executable directory");
  }
  filename.resize(length);
  return (std::filesystem::path(filename).parent_path() / "robottest_aim.yaml").string();
}

void write_aim(sim::PoseStatus & status, const sim::CameraFrame & frame, const io::Command & command)
{
  if (!command.control || !std::isfinite(command.yaw) || !std::isfinite(command.pitch)) {
    return;
  }
  const auto & u = frame.aim_up;
  const auto & f = frame.aim_forward;
  cv::Vec3d up(u[0], u[1], u[2]);
  cv::Vec3d forward(f[0], f[1], f[2]);
  const double up_length = cv::norm(up);
  if (!std::isfinite(up_length) || up_length < 0.5) {
    return;
  }
  up *= 1.0 / up_length;
  forward -= up * forward.dot(up);
  const double forward_length = cv::norm(forward);
  if (!std::isfinite(forward_length) || forward_length < 0.5) {
    return;
  }
  forward *= 1.0 / forward_length;
  const cv::Vec3d right = up.cross(forward);

  const double horizontal = std::cos(command.pitch);
  const cv::Vec3d direction(
    -std::sin(command.yaw) * horizontal, -std::sin(command.pitch),
    std::cos(command.yaw) * horizontal);
  const double x = direction.dot(right);
  const double z = direction.dot(forward);
  const double planar_length = std::hypot(x, z);
  constexpr double degrees = 180.0 / CV_PI;
  const double yaw =
    planar_length > 1e-6 ? std::atan2(x, z) * degrees : frame.current_yaw;
  const double pitch = std::atan2(-direction.dot(up), planar_length) * degrees;
  if (std::isfinite(yaw) && std::abs(yaw) <= 180 && std::isfinite(pitch)) {
    status.aim_valid = true;
    status.yaw = static_cast<float>(yaw);
    status.pitch = static_cast<float>(pitch);
  }
}

bool should_fire(
  auto_aim::Shooter & shooter, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const io::Command & command,
  const sim::CameraFrame & frame)
{
  const auto & v = frame.muzzle_forward;
  const Eigen::Vector3d muzzle = framework_point(v[0], v[1], v[2]);
  const double muzzle_length = muzzle.norm();
  if (!std::isfinite(muzzle_length) || muzzle_length < 0.5) {
    return false;
  }

  const double horizontal = std::cos(command.pitch);
  const Eigen::Vector3d desired(
    std::cos(command.yaw) * horizontal, std::sin(command.yaw) * horizontal,
    -std::sin(command.pitch));
  const Eigen::Vector3d actual_angles(
    std::atan2(muzzle.y(), muzzle.x()), std::atan2(-muzzle.z(), std::hypot(muzzle.x(), muzzle.y())),
    0);

  // The upstream shooter checks yaw stability. The simulator also requires barrel alignment.
  constexpr double alignment_limit = CV_PI / 180.0;
  const bool stable = shooter.shoot(command, aimer, targets, actual_angles);
  return stable && desired.dot(muzzle / muzzle_length) >= std::cos(alignment_limit);
}

int run(std::uint16_t port)
{
  sim::CameraReceiver receiver(port);
  const std::string config = config_path();
  std::cerr << "Waiting for RoboTest corner observations on 127.0.0.1:" << port << '\n';
  std::uint64_t connection = 0;
  std::int64_t last_capture = 0;
  std::unique_ptr<auto_aim::Tracker> tracker;
  std::unique_ptr<auto_aim::Aimer> aimer;
  std::unique_ptr<auto_aim::Shooter> shooter;

  while (!stop_requested) {
    const auto frame = receiver.wait_for_frame();
    if (!frame) {
      continue;
    }
    if (frame->connection_id != connection || frame->capture_time_us <= last_capture) {
      connection = frame->connection_id;
      tracker = std::make_unique<auto_aim::Tracker>(config, nullptr, cv::Point2f{});
      aimer = std::make_unique<auto_aim::Aimer>(config);
      shooter = std::make_unique<auto_aim::Shooter>(config);
    }
    last_capture = frame->capture_time_us;
    const auto timestamp = std::chrono::steady_clock::time_point(
      std::chrono::microseconds(frame->capture_time_us));
    const auto start = std::chrono::steady_clock::now();

    std::list<auto_aim::Armor> observations;
    std::vector<Solution> solutions;
    sim::PoseStatus status;
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

      const auto name = armor_name(armor.robot);
      if (name == auto_aim::not_armor) {
        continue;
      }

      cv::Matx33d rotation;
      cv::Rodrigues(camera->rotation_vector, rotation);
      // The marker plane's +Z points into the armor, toward the robot center.
      const cv::Vec3d normal(rotation(0, 2), rotation(1, 2), rotation(2, 2));
      const auto normal_endpoint = to_world(*frame, camera->position + normal);
      if (!normal_endpoint) {
        continue;
      }
      const cv::Vec3d normal_world = *normal_endpoint - *world;
      std::vector<cv::Point2f> points;
      for (const auto & corner : armor.corners) {
        points.emplace_back(corner.image_x, corner.image_y);
      }
      auto & observation = observations.emplace_back(
        armor.team == 100 ? 0 : 1, 0, 1.f, cv::Rect{}, std::move(points));
      observation.name = name;
      observation.type = auto_aim::small;
      observation.priority = auto_aim::fifth;
      observation.xyz_in_world = framework_point((*world)[0], (*world)[1], (*world)[2]);
      observation.ypd_in_world = tools::xyz2ypd(observation.xyz_in_world);
      observation.ypr_in_world = Eigen::Vector3d(
        std::atan2(-normal_world[0], normal_world[2]), 0, 0);
      solutions.push_back({&armor, name, *world, camera->rotation_vector, distance});
    }

    auto targets = tracker->track(observations, timestamp, false);
    if (status.solved == 0 && !frame->armors.empty()) {
      status.status = sim::ProgramStatus::processing_error;
    } else if (!targets.empty()) {
      const auto name = targets.front().name;
      const Solution * observed = nullptr;
      for (const auto & solution : solutions) {
        if (solution.name == name && (!observed || solution.distance < observed->distance)) {
          observed = &solution;
        }
      }
      if (observed != nullptr) {
        status.target_id = observed->armor->id;
        status.distance = static_cast<float>(observed->distance);
        for (int axis = 0; axis < 3; ++axis) {
          status.world_position[axis] = static_cast<float>(observed->position[axis]);
          status.camera_rotation_vector[axis] =
            static_cast<float>(observed->camera_rotation_vector[axis]);
        }
      }

      const auto & p = frame->aim_origin;
      const auto origin = framework_point(p[0], p[1], p[2]);
      if (origin.allFinite() && std::isfinite(frame->projectile_speed) &&
          frame->projectile_speed > 0) {
        targets.front() = targets.front().relative_to(origin);
        const auto command = aimer->aim(targets, timestamp, frame->projectile_speed, false);
        write_aim(status, *frame, command);
        if (status.aim_valid && observed != nullptr) {
          status.fire_requested = should_fire(*shooter, *aimer, targets, command, *frame);
        }
      }
    }

    std::cout << "frame=" << frame->id << " capture_us=" << frame->capture_time_us
              << " observed=" << frame->armors.size() << " solved=" << status.solved
              << " tracker=" << tracker->state() << " valid=" << status.aim_valid;
    if (status.aim_valid) {
      std::cout << " yaw_deg=" << status.yaw << " pitch_deg=" << status.pitch
                << " fire=" << status.fire_requested;
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
