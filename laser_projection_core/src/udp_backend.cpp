#include "laser_projection_core/backends.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>

namespace laser_projection {

UdpBackend::UdpBackend(std::string host, unsigned short port)
    : host_(std::move(host)), port_(port) {}

bool UdpBackend::initialize() {
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  return socket_ >= 0;
}

bool UdpBackend::send_frame(const std::vector<LaserPoint>& points, double scan_rate) {
  if (socket_ < 0) return false;
  std::ostringstream message;
  message << "LASER_FRAME " << scan_rate << ' ' << points.size() << '\n';
  for (const auto& point : points)
    message << point.x << ' ' << point.y << ' ' << static_cast<int>(point.color.r) << ' '
            << static_cast<int>(point.color.g) << ' ' << static_cast<int>(point.color.b)
            << ' ' << point.blank << '\n';
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  if (inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1) return false;
  const auto data = message.str();
  return sendto(socket_, data.data(), data.size(), 0,
                reinterpret_cast<sockaddr*>(&address), sizeof(address)) ==
         static_cast<ssize_t>(data.size());
}

void UdpBackend::blackout() {
  if (socket_ >= 0) send_frame({}, 0.0);
}

}  // namespace laser_projection
