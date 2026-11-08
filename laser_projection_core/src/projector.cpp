#include "laser_projection_core/projector.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

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
                     std::shared_ptr<LaserBackend> backend, Calibration calibration,
                     Calibration2D surface_calibration)
    : surface_(std::move(surface)), backend_(std::move(backend)),
      calibration_(calibration), surface_calibration_(std::move(surface_calibration)) {}

void Calibration2D::fit(
    const std::vector<CalibrationPoint2D>& correspondences) {
  fitted_ = false;
  if (correspondences.size() != 4)
    throw std::invalid_argument("2D calibration requires exactly four point pairs");

  double orientation = 0.0;
  Point2D mirror_min{std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity()};
  Point2D mirror_max{-std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()};
  for (std::size_t i = 0; i < correspondences.size(); ++i) {
    const auto& a = correspondences[i].surface;
    const auto& b = correspondences[(i + 1) % correspondences.size()].surface;
    const auto& c = correspondences[(i + 2) % correspondences.size()].surface;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(correspondences[i].mirror.x) ||
        !std::isfinite(correspondences[i].mirror.y))
      throw std::invalid_argument("2D calibration points must be finite");
    mirror_min.x = std::min(mirror_min.x, correspondences[i].mirror.x);
    mirror_min.y = std::min(mirror_min.y, correspondences[i].mirror.y);
    mirror_max.x = std::max(mirror_max.x, correspondences[i].mirror.x);
    mirror_max.y = std::max(mirror_max.y, correspondences[i].mirror.y);
    const double cross = (b.x - a.x) * (c.y - b.y) -
                         (b.y - a.y) * (c.x - b.x);
    if (std::abs(cross) < 1e-12 ||
        (orientation != 0.0 && cross * orientation < 0.0))
      throw std::invalid_argument(
          "Surface calibration corners must form a convex, non-degenerate quadrilateral in order");
    orientation = cross;
  }

  double equations[8][9]{};
  for (std::size_t i = 0; i < 4; ++i) {
    const auto& source = correspondences[i].surface;
    const auto& target = correspondences[i].mirror;
    const std::size_t row = i * 2;
    equations[row][0] = source.x;
    equations[row][1] = source.y;
    equations[row][2] = 1.0;
    equations[row][6] = -target.x * source.x;
    equations[row][7] = -target.x * source.y;
    equations[row][8] = target.x;
    equations[row + 1][3] = source.x;
    equations[row + 1][4] = source.y;
    equations[row + 1][5] = 1.0;
    equations[row + 1][6] = -target.y * source.x;
    equations[row + 1][7] = -target.y * source.y;
    equations[row + 1][8] = target.y;
  }

  for (std::size_t column = 0; column < 8; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1; row < 8; ++row)
      if (std::abs(equations[row][column]) >
          std::abs(equations[pivot][column])) pivot = row;
    if (std::abs(equations[pivot][column]) < 1e-12)
      throw std::invalid_argument("Surface calibration corners are degenerate");
    if (pivot != column)
      for (std::size_t j = column; j <= 8; ++j)
        std::swap(equations[pivot][j], equations[column][j]);
    const double divisor = equations[column][column];
    for (std::size_t j = column; j <= 8; ++j)
      equations[column][j] /= divisor;
    for (std::size_t row = 0; row < 8; ++row) {
      if (row == column) continue;
      const double factor = equations[row][column];
      for (std::size_t j = column; j <= 8; ++j)
        equations[row][j] -= factor * equations[column][j];
    }
  }

  std::array<double, 9> candidate{};
  for (std::size_t i = 0; i < 8; ++i) candidate[i] = equations[i][8];
  candidate[8] = 1.0;
  homography_ = candidate;
  mirror_min_ = mirror_min;
  mirror_max_ = mirror_max;
  fitted_ = true;
}

std::optional<Point2D> Calibration2D::map(Point2D surface) const {
  if (!fitted_ || !std::isfinite(surface.x) || !std::isfinite(surface.y))
    return std::nullopt;
  const double denominator = homography_[6] * surface.x +
                             homography_[7] * surface.y + homography_[8];
  if (!std::isfinite(denominator) || std::abs(denominator) < 1e-12)
    return std::nullopt;
  Point2D result{
      (homography_[0] * surface.x + homography_[1] * surface.y + homography_[2]) /
          denominator,
      (homography_[3] * surface.x + homography_[4] * surface.y + homography_[5]) /
          denominator};
  if (!std::isfinite(result.x) || !std::isfinite(result.y)) return std::nullopt;
  return result;
}

bool Calibration2D::contains(Point2D mirror) const {
  constexpr double tolerance = 1e-9;
  return fitted_ && std::isfinite(mirror.x) && std::isfinite(mirror.y) &&
         mirror.x >= mirror_min_.x - tolerance &&
         mirror.x <= mirror_max_.x + tolerance &&
         mirror.y >= mirror_min_.y - tolerance &&
         mirror.y <= mirror_max_.y + tolerance;
}

bool Projector::initialize() { return backend_->initialize(); }

bool Projector::show_path(const std::vector<Point3D>& path, ProjectorPose projector_pose,
                          PathOptions options, double scan_rate) {
  return show_paths({path}, projector_pose, options, scan_rate);
}

bool Projector::show_paths(const std::vector<std::vector<Point3D>>& paths,
                           ProjectorPose projector_pose, PathOptions options,
                           double scan_rate) {
  last_frame_.clear();
  const bool valid_input = !paths.empty() &&
      std::isfinite(options.point_spacing) && options.point_spacing > 0.0 &&
      std::isfinite(scan_rate) && scan_rate > 0.0 &&
      std::all_of(paths.begin(), paths.end(), [](const auto& path) {
        return std::all_of(path.begin(), path.end(), [](const Point3D& point) {
          return std::isfinite(point.x) && std::isfinite(point.y) &&
                 std::isfinite(point.z);
        });
      });
  if (!valid_input) {
    backend_->blackout();
    return false;
  }
  bool mapping_valid = true;
  std::vector<std::vector<Point3D>> projected_paths;
  for (const auto& path : paths) {
    std::vector<Point3D> projected;
    for (const auto& target : path) {
      if (surface_calibration_.fitted()) {
        const auto mapped = surface_calibration_.map({target.x, target.y});
        if (!mapped || !surface_calibration_.contains(*mapped)) {
          mapping_valid = false;
          continue;
        }
        projected.push_back({mapped->x, mapped->y, target.z});
      } else {
        auto point = surface_->intersect({projector_pose.position,
          {target.x - projector_pose.position.x, target.y - projector_pose.position.y,
           target.z - projector_pose.position.z}});
        if (point) projected.push_back(*point);
      }
    }
    if (!projected.empty()) projected_paths.push_back(std::move(projected));
  }
  if (!mapping_valid || projected_paths.empty()) {
    backend_->blackout();
    return false;
  }

  auto append = [&](Point3D p, bool blank) {
    if (surface_calibration_.fitted()) {
      last_frame_.push_back({p.x, p.y, options.color, blank});
      return;
    }
    const auto local = rotate_inverse(
        {p.x - projector_pose.position.x, p.y - projector_pose.position.y,
         p.z - projector_pose.position.z}, projector_pose.orientation);
    last_frame_.push_back({local.x * calibration_.scale_x + calibration_.offset_x,
                           local.y * calibration_.scale_y + calibration_.offset_y,
                           options.color, blank});
  };
  Point3D frame_end = projected_paths.front().front();
  auto append_blank_travel = [&](const Point3D& start, const Point3D& end) {
    const double distance = std::hypot(end.x - start.x, end.y - start.y);
    if (distance < 1e-9) return;
    const int steps = std::max(
        1, static_cast<int>(std::ceil(distance / options.point_spacing)));
    append(start, true);
    for (int step = 1; step <= steps; ++step) {
      const double t = static_cast<double>(step) / steps;
      append({start.x + t * (end.x - start.x),
              start.y + t * (end.y - start.y),
              start.z + t * (end.z - start.z)}, true);
    }
  };
  for (const auto& projected : projected_paths) {
    if (last_frame_.empty()) append(projected.front(), true);
    else append_blank_travel(frame_end, projected.front());
    if (projected.size() == 1) {
      for (int i = 0; i < 8; ++i) append(projected.front(), false);
      frame_end = projected.front();
    }
    for (size_t i = 0; i + 1 < projected.size(); ++i) {
      const auto a = projected[i];
      const auto b = projected[i + 1];
      const double distance = std::hypot(b.x - a.x, b.y - a.y);
      const int steps = std::max(1, static_cast<int>(std::ceil(distance / options.point_spacing)));
      for (int step = 0; step <= steps; ++step) {
        const double t = static_cast<double>(step) / steps;
        const Point3D point{a.x + t * (b.x - a.x),
                            a.y + t * (b.y - a.y),
                            a.z + t * (b.z - a.z)};
        append(point, false);
        frame_end = point;
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

        auto append_stroke = [&](const Point3D& start, const Point3D& end) {
          append(start, true);
          const double distance = std::hypot(end.x - start.x, end.y - start.y);
          const int steps = std::max(1, static_cast<int>(std::ceil(
              distance / options.point_spacing)));
          for (int step = 0; step <= steps; ++step) {
            const double t = static_cast<double>(step) / steps;
            const Point3D point{start.x + t * (end.x - start.x),
                                start.y + t * (end.y - start.y),
                                start.z + t * (end.z - start.z)};
            append(point, false);
            frame_end = point;
          }
        };

        append(tip, true);
        append_stroke(left, tip);

        append(tip, true);
        append_stroke(right, tip);
      }
    }
  }
  append_blank_travel(frame_end, projected_paths.front().front());
  const bool valid_frame = mapping_valid && last_frame_.size() >= 2 &&
      std::all_of(last_frame_.begin(), last_frame_.end(), [](const LaserPoint& point) {
        return std::isfinite(point.x) && std::isfinite(point.y);
      });
  const bool in_mirror_field = !surface_calibration_.fitted() ||
      std::all_of(last_frame_.begin(), last_frame_.end(), [](const LaserPoint& point) {
        return point.x >= -1.0 && point.x <= 1.0 &&
               point.y >= -1.0 && point.y <= 1.0;
      });
  if (!valid_frame || !in_mirror_field) {
    last_frame_.clear();
    backend_->blackout();
    return false;
  }
  return backend_->send_frame(last_frame_, scan_rate);
}

bool Projector::show_raw_frame(const std::vector<LaserPoint>& points, Color color,
                               double scan_rate) {
  auto colored_points = points;
  for (auto& point : colored_points)
    point.color = point.blank ? Color{0, 0, 0} : color;
  return show_raw_frame(colored_points, scan_rate);
}

bool Projector::show_raw_frame(const std::vector<LaserPoint>& points, double scan_rate) {
  last_frame_.clear();
  const bool valid_points = !points.empty() &&
      points.front().blank && points.back().blank &&
      std::all_of(points.begin(), points.end(), [](const LaserPoint& point) {
        return std::isfinite(point.x) && std::isfinite(point.y) &&
               point.x >= -1.0 && point.x <= 1.0 &&
               point.y >= -1.0 && point.y <= 1.0;
      });
  if (!valid_points || !std::isfinite(scan_rate) || scan_rate <= 0.0) {
    backend_->blackout();
    return false;
  }

  last_frame_ = points;
  for (auto& point : last_frame_)
    if (point.blank) point.color = {0, 0, 0};
  return backend_->send_frame(last_frame_, scan_rate);
}

}  // namespace laser_projection
