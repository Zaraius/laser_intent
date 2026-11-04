#include "laser_projection_core/backends.hpp"
#include <algorithm>
#include <dlfcn.h>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace laser_projection {

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
  write_frame_high_resolution_ = reinterpret_cast<WriteFrameHighResolution>(
      dlsym(library_, "WriteFrameHighResolution"));
  close_devices_ = reinterpret_cast<CloseDevices>(dlsym(library_, "CloseDevices"));
  if (!open_devices_ || !get_status_ || !write_frame_high_resolution_) {
    std::fprintf(stderr,
                 "[HeliosBackend] API library must export OpenDevices, GetStatus, "
                 "and WriteFrameHighResolution\n");
    return false;
  }
  initialized_ = open_devices_() > 0;
  return initialized_;
}

bool HeliosBackend::send_frame(const std::vector<LaserPoint>& points, double scan_rate) {
  if (!initialized_ || !write_frame_high_resolution_ || points.empty()) {
    std::fprintf(stderr, "[HeliosBackend] Cannot send frame: backend is uninitialized or frame is empty\n");
    return false;
  }
  if (!std::isfinite(scan_rate) || scan_rate <= 0.0) {
    std::fprintf(stderr, "[HeliosBackend] Cannot send frame: invalid scan rate %.3f\n", scan_rate);
    return false;
  }
  std::vector<detail::HeliosHighResolutionPoint> frame;
  frame.reserve(points.size());
  for (const auto& point : points) {
    const auto convert = [](double value) {
      return static_cast<std::uint16_t>(std::lround(
          std::clamp((value + 1.0) * 32767.5, 0.0, 65535.0)));
    };
    const auto channel = [blank = point.blank](std::uint8_t value) {
      return static_cast<std::uint16_t>(blank ? 0 : value * 257u);
    };
    frame.push_back({convert(point.x), convert(point.y), channel(point.color.r),
                     channel(point.color.g), channel(point.color.b)});
  }
  if (get_status_) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    int status = get_status_(device_);
    while (status != 1) {
      if (std::chrono::steady_clock::now() >= deadline) {
        std::fprintf(stderr, "[HeliosBackend] Timed out waiting for device readiness (status %d)\n",
                     status);
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      status = get_status_(device_);
    }
  }
  const int result = write_frame_high_resolution_(
      static_cast<unsigned int>(device_), static_cast<int>(scan_rate), 0,
      frame.data(), static_cast<int>(frame.size()));
  if (result != 1) {
    std::fprintf(stderr,
                 "[HeliosBackend] WriteFrameHighResolution failed "
                 "(result %d, points %zu, scan rate %.3f)\n",
                 result, frame.size(), scan_rate);
    return false;
  }
  return true;
}

void HeliosBackend::blackout() {
  if (initialized_ && write_frame_high_resolution_) {
    detail::HeliosHighResolutionPoint point{32768, 32768, 0, 0, 0};
    write_frame_high_resolution_(static_cast<unsigned int>(device_), 30000, 0, &point, 1);
  }
}

}  // namespace laser_projection
