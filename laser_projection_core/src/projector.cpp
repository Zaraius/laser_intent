#include "laser_projection_core/projector.hpp"
#include <algorithm>
#include <cmath>

namespace laser_projection {

static Point3D rotate_inverse(const Point3D& point, const Quaternion& q) {
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (norm < 1e-12) return point;
  const double x = -q.x / norm, y = -q.y / norm, z = -q.z / norm, w = q.w / norm;
  return {
    (1 - 2 * (y * y + z * z)) * point.x + 2 * (x * y - z * w) * point.y +
      2 * (x * z + y * w) * point.z,
    2 * (x * y + z * w) * point.x + (1 - 2 * (x * x + z * z)) * point.y +
      2 * (y * z - x * w) * point.z,
    2 * (x * z - y * w) * point.x + 2 * (y * z + x * w) * point.y +
      (1 - 2 * (x * x + y * y)) * point.z};
}

std::optional<Point3D> PlaneSurface::intersect(const Ray& ray) const {
  if (std::abs(ray.direction.z) < 1e-9) return std::nullopt;
  const double t = (z_ - ray.origin.z) / ray.direction.z;
  if (t < 0.0) return std::nullopt;
  return Point3D{ray.origin.x + t * ray.direction.x,
                 ray.origin.y + t * ray.direction.y, z_};
}

Projector::Projector(std::shared_ptr<const ProjectionSurface> surface,
                     std::shared_ptr<LaserBackend> backend, Calibration calibration)
    : surface_(std::move(surface)), backend_(std::move(backend)), calibration_(calibration) {}

bool Projector::initialize() { return backend_->initialize(); }

bool Projector::show_path(const std::vector<Point3D>& path, ProjectorPose projector_pose,
                          PathOptions options, double scan_rate) {
  last_frame_.clear();
  if (path.empty() || options.point_spacing <= 0.0) {
    backend_->blackout();
    return false;
  }
  std::vector<Point3D> projected;
  for (const auto& target : path) {
    auto point = surface_->intersect({projector_pose.position,
      {target.x - projector_pose.position.x, target.y - projector_pose.position.y,
       target.z - projector_pose.position.z}});
    if (point) projected.push_back(*point);
  }
  if (projected.empty()) {
    backend_->blackout();
    return false;
  }

  auto append = [&](Point3D p, bool blank) {
    const auto local = rotate_inverse(
        {p.x - projector_pose.position.x, p.y - projector_pose.position.y,
         p.z - projector_pose.position.z}, projector_pose.orientation);
    last_frame_.push_back({local.x * calibration_.scale_x + calibration_.offset_x,
                           local.y * calibration_.scale_y + calibration_.offset_y,
                           options.color, blank});
  };
  append(projected.front(), true);
  if (projected.size() == 1) {
    for (int i = 0; i < 8; ++i) append(projected.front(), false);
  }
  for (size_t i = 0; i + 1 < projected.size(); ++i) {
    const auto a = projected[i];
    const auto b = projected[i + 1];
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / options.point_spacing)));
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / steps;
      append({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z}, false);
    }
  }
  if (options.arrows && projected.size() > 1) {
    const auto tip = projected.back();
    const auto from = projected[projected.size() - 2];
    const double length = std::hypot(tip.x - from.x, tip.y - from.y);
    if (length > 1e-9) {
      const double size = std::min(0.2, length * 0.5);
      const double ux = (tip.x - from.x) / length;
      const double uy = (tip.y - from.y) / length;
      const Point3D left{tip.x - size * (ux - uy), tip.y - size * (uy + ux), tip.z};
      const Point3D right{tip.x - size * (ux + uy), tip.y - size * (uy - ux), tip.z};
      append(left, true);
      append(tip, false);
      append(right, true);
      append(tip, false);
    }
  }
  return backend_->send_frame(last_frame_, scan_rate);
}

}  // namespace laser_projection
