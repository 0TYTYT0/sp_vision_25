#include <windows.h>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "camera_receiver.hpp"

#ifdef SP_VISION_SIM_DETECTOR
#include <chrono>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/yolos/yolov5.hpp"
#include "tools/img_tools.hpp"
#endif

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

struct Options
{
  std::uint16_t port = 5600;
  std::uint64_t max_frames = 0;
  bool receive_only = false;
  bool preview = true;
  std::filesystem::path config;
  std::filesystem::path video;
};

std::uint64_t parse_integer(const std::string & value, const std::string & option)
{
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("Expected a nonnegative integer for " + option);
  }
  return std::stoull(value);
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "--receive-only") {
      options.receive_only = true;
    } else if (option == "--no-preview") {
      options.preview = false;
    } else if (
      option == "--port" || option == "--max-frames" || option == "--config" ||
      option == "--video") {
      if (++i >= argc) {
        throw std::invalid_argument("Missing value for " + option);
      }
      if (option == "--port") {
        const auto port = parse_integer(argv[i], option);
        if (port == 0 || port > std::numeric_limits<std::uint16_t>::max()) {
          throw std::invalid_argument("Port must be between 1 and 65535");
        }
        options.port = static_cast<std::uint16_t>(port);
      } else if (option == "--max-frames") {
        options.max_frames = parse_integer(argv[i], option);
      } else if (option == "--config") {
        options.config = std::filesystem::absolute(argv[i]);
      } else {
        options.video = std::filesystem::absolute(argv[i]);
      }
    } else {
      throw std::invalid_argument("Unknown option " + option);
    }
  }
  return options;
}

#ifdef SP_VISION_SIM_DETECTOR
std::filesystem::path executable_directory()
{
  std::wstring path(32768, L'\0');
  const auto length = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
  if (length == 0 || length == path.size()) {
    throw std::runtime_error("Could not determine executable directory");
  }
  path.resize(length);
  return std::filesystem::path(path).parent_path();
}
#endif

void print_frame(const sim::CameraFrame & frame)
{
  std::cout << "frame=" << frame.id << " capture_us=" << frame.capture_time_us
            << " size=" << frame.width << "x" << frame.height;
}

int run(Options options)
{
#ifndef SP_VISION_SIM_DETECTOR
  if (!options.config.empty() || !options.video.empty()) {
    throw std::invalid_argument("Config and video input require SP_VISION_SIM_DETECTOR");
  }
  options.receive_only = true;
#else
  std::unique_ptr<auto_aim::YOLOV5> detector;
  if (!options.receive_only) {
    std::filesystem::current_path(executable_directory());
    if (options.config.empty()) {
      options.config = "vision.yaml";
    }
    detector = std::make_unique<auto_aim::YOLOV5>(options.config.string(), false);
  }

  cv::VideoCapture video;
  if (!options.video.empty() && !video.open(options.video.string())) {
    throw std::runtime_error("Could not open video " + options.video.string());
  }
  const bool preview = options.preview;
  constexpr char window[] = "RoboTest Vision";
  bool preview_opened = false;
#endif

  std::unique_ptr<sim::CameraReceiver> receiver;
  if (options.video.empty()) {
    receiver = std::make_unique<sim::CameraReceiver>(options.port);
    std::cerr << "Waiting for RoboTest on 127.0.0.1:" << options.port << '\n';
  }

  std::uint64_t processed = 0;
  while (!stop_requested && (options.max_frames == 0 || processed < options.max_frames)) {
    auto frame = receiver ? receiver->wait_for_frame() : std::nullopt;
    if (frame) {
      print_frame(*frame);
    }

#ifdef SP_VISION_SIM_DETECTOR
    cv::Mat image;
    if (frame) {
      image = cv::Mat(frame->height, frame->width, CV_8UC3, frame->bgr.data());
    } else if (!receiver) {
      if (!video.read(image)) {
        break;
      }
      std::cout << "frame=" << processed + 1 << " video_ms=" << video.get(cv::CAP_PROP_POS_MSEC)
                << " size=" << image.cols << "x" << image.rows;
    }

    if (!image.empty()) {
      if (detector) {
        const auto started = std::chrono::steady_clock::now();
        const auto detections = detector->detect(image, -1);
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - started);
        std::cout << " detections=" << detections.size() << " inference_us=" << elapsed.count();
        for (const auto & armor : detections) {
          std::cout << " [" << auto_aim::COLORS.at(armor.color) << " "
                    << auto_aim::ARMOR_NAMES.at(armor.name) << " " << armor.confidence;
          for (const auto & point : armor.points) {
            std::cout << " " << point.x << "," << point.y;
          }
          std::cout << "]";
          if (preview) {
            tools::draw_points(image, armor.points, {0, 255, 0});
          }
        }
      }
      if (preview) {
        cv::imshow(window, image);
        preview_opened = true;
      }
      if (!frame) {
        ++processed;
        std::cout << std::endl;
      }
    }
#endif

    if (frame) {
      ++processed;
      std::cout << std::endl;
    }

#ifdef SP_VISION_SIM_DETECTOR
    if (
      preview_opened &&
      (cv::waitKey(1) == 27 || cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) < 1)) {
      break;
    }
#endif
  }
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "robottest_vision [--port 5600 | --video FILE] [--config FILE]\n"
                 "                 [--receive-only] [--no-preview] [--max-frames N]\n";
    return 0;
  }

  try {
    if (!SetConsoleCtrlHandler(handle_console_event, TRUE)) {
      throw std::runtime_error("Could not register console stop handler");
    }
    return run(parse_options(argc, argv));
  } catch (const std::exception & error) {
    std::cerr << "robottest_vision: " << error.what() << '\n';
    return 1;
  }
}
