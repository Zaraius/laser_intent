#include "laser_projection_core/backends.hpp"
#include <algorithm>
#include <dlfcn.h>
#include <cstdint>
#include <chrono>
#include <thread>

namespace laser_projection {
namespace {
struct HeliosPoint {
  std::uint16_t x, y;
  std::uint8_t r, g, b, i;
  HeliosPoint(std::uint16_t x_value, std::uint16_t y_value,
              std::uint8_t red, std::uint8_t green, std::uint8_t blue)
      : x(x_value), y(y_value), r(red), g(green), b(blue), i(0) {}
};
}

HeliosBackend::HeliosBackend(std::string library_path) : library_path_(std::move(library_path)) {}
HeliosBackend::~HeliosBackend() {
  if (initialized_ && close_devices_) close_devices_();
  if (library_) dlclose(library_);
}

bool HeliosBackend::initialize() {
  library_ = dlopen(library_path_.c_str(), RTLD_NOW);
  if (!library_) return false;
  open_devices_ = reinterpret_cast<OpenDevices>(dlsym(library_, "OpenDevices"));
  get_status_ = reinterpret_cast<GetStatus>(dlsym(library_, "GetStatus"));
  write_frame_ = reinterpret_cast<WriteFrame>(dlsym(library_, "WriteFrame"));
  close_devices_ = reinterpret_cast<CloseDevices>(dlsym(library_, "CloseDevices"));
  if (!open_devices_ || !get_status_ || !write_frame_) return false;
  initialized_ = open_devices_() > 0;
  return initialized_;
}

bool HeliosBackend::send_frame(const std::vector<LaserPoint>& points, double scan_rate) {
  if (!initialized_ || !write_frame_ || points.empty()) return false;
  std::vector<HeliosPoint> frame;
  frame.reserve(points.size());
  for (const auto& point : points) {
    const auto convert = [](double value) {
      return static_cast<std::uint16_t>(std::clamp((value + 1.0) * 32767.5, 0.0, 65535.0));
    };
    frame.emplace_back(convert(point.x), convert(point.y),
      static_cast<std::uint8_t>(point.blank ? 0 : point.color.r),
      static_cast<std::uint8_t>(point.blank ? 0 : point.color.g),
      static_cast<std::uint8_t>(point.blank ? 0 : point.color.b));
  }
  if (get_status_) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (get_status_(device_) != 1) {
      if (std::chrono::steady_clock::now() >= deadline) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return write_frame_(device_, static_cast<int>(scan_rate), 0,
                      frame.data(), static_cast<int>(frame.size())) == 0;
}

void HeliosBackend::blackout() {
  if (initialized_ && write_frame_) {
    HeliosPoint point{0, 0, 0, 0, 0};
    write_frame_(device_, 30000, 0, &point, 1);
  }
}

}  // namespace laser_projection
