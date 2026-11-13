#pragma once

#include "laser_projection_core/projector.hpp"
#include <cstdint>
#include <string>

namespace laser_projection {

namespace detail {
struct HeliosHighResolutionPoint {
  std::uint16_t x, y, r, g, b;
};
}

class UdpBackend final : public LaserBackend {
public:
  UdpBackend(std::string host, unsigned short port);
  bool initialize() override;
  bool send_frame(const std::vector<LaserPoint>& points, double scan_rate) override;
  bool blackout() override;
private:
  std::string host_;
  unsigned short port_;
  int socket_{-1};
};

class HeliosBackend final : public LaserBackend {
public:
  explicit HeliosBackend(std::string library_path);
  ~HeliosBackend() override;
  bool initialize() override;
  bool send_frame(const std::vector<LaserPoint>& points, double scan_rate) override;
  bool blackout() override;
private:
  std::string library_path_;
  void* library_{nullptr};
  int device_{0};
  using OpenDevices = int (*)();
  using GetStatus = int (*)(int);
  using WriteFrameHighResolution = int (*)(
      unsigned int, int, std::uint8_t, detail::HeliosHighResolutionPoint*, int);
  using CloseDevices = void (*)();
  OpenDevices open_devices_{nullptr};
  GetStatus get_status_{nullptr};
  bool initialized_{false};
  WriteFrameHighResolution write_frame_high_resolution_{nullptr};
  CloseDevices close_devices_{nullptr};
};

}  // namespace laser_projection
