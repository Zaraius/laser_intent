#pragma once

#include "laser_projection_core/projector.hpp"
#include <string>

namespace laser_projection {

class UdpBackend final : public LaserBackend {
public:
  UdpBackend(std::string host, unsigned short port);
  bool initialize() override;
  bool send_frame(const std::vector<LaserPoint>& points, double scan_rate) override;
  void blackout() override;
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
  void blackout() override;
private:
  std::string library_path_;
  void* library_{nullptr};
  int device_{0};
  using OpenDevices = int (*)();
  using GetStatus = int (*)(int);
  using WriteFrame = int (*)(int, int, int, const void*, int);
  using CloseDevices = void (*)();
  OpenDevices open_devices_{nullptr};
  GetStatus get_status_{nullptr};
  bool initialized_{false};
  WriteFrame write_frame_{nullptr};
  CloseDevices close_devices_{nullptr};
};

}  // namespace laser_projection
