#pragma once

#include <memory>
#include <optional>
#include <array>
#include <utility>
#include <vector>

namespace laser_projection {

struct Point2D { double x{}; double y{}; };
struct Point3D { double x{}; double y{}; double z{}; };
struct Color { unsigned char r{}; unsigned char g{}; unsigned char b{}; };
struct LaserPoint {
  double x{}; double y{}; Color color{}; bool blank{};
};
struct Ray { Point3D origin; Point3D direction; };
struct Quaternion { double x{}; double y{}; double z{}; double w{1.0}; };
struct ProjectorPose { Point3D position; Quaternion orientation; };

class ProjectionSurface {
public:
  virtual ~ProjectionSurface() = default;
  virtual std::optional<Point3D> intersect(const Ray&) const = 0;
};

class PlaneSurface final : public ProjectionSurface {
public:
  explicit PlaneSurface(double z = 0.0) : z_(z) {}
  std::optional<Point3D> intersect(const Ray&) const override;
private:
  double z_;
};

class LaserBackend {
public:
  virtual ~LaserBackend() = default;
  virtual bool initialize() = 0;
  virtual bool send_frame(const std::vector<LaserPoint>& points, double scan_rate) = 0;
  virtual bool blackout() = 0;
};

struct Calibration {
  double scale_x{1.0};
  double scale_y{1.0};
  double offset_x{};
  double offset_y{};
};

struct CalibrationPoint2D {
  Point2D surface;
  Point2D mirror;
};

class Calibration2D {
public:
  void fit(const std::vector<CalibrationPoint2D>& correspondences);
  std::optional<Point2D> map(Point2D surface) const;
  bool contains(Point2D mirror) const;
  std::optional<std::pair<Point3D, Point3D>> clip_surface_segment(
      Point3D start, Point3D end) const;
  std::optional<std::pair<Point3D, Point3D>> clip_segment(
      Point3D start, Point3D end) const;
  bool fitted() const { return fitted_; }
private:
  std::array<double, 9> homography_{};
  std::array<Point2D, 4> surface_corners_{};
  Point2D mirror_min_{};
  Point2D mirror_max_{};
  double surface_winding_{};
  bool fitted_{false};
};

struct PathOptions {
  Color color{0, 255, 0};
  double point_spacing{0.05};
  bool arrows{false};
};

class Projector {
public:
  Projector(std::shared_ptr<const ProjectionSurface> surface,
            std::shared_ptr<LaserBackend> backend,
            Calibration calibration = {}, Calibration2D surface_calibration = {});
  bool initialize();
  bool blackout();
  bool has_surface_calibration() const { return surface_calibration_.fitted(); }
  bool show_path(const std::vector<Point3D>& path, ProjectorPose projector_pose,
                 PathOptions options = {}, double scan_rate = 30000.0);
  bool show_paths(const std::vector<std::vector<Point3D>>& paths,
                  ProjectorPose projector_pose, PathOptions options = {},
                  double scan_rate = 30000.0);
  // Bypasses surface projection and calibration; x/y must be in [-1, 1].
  bool show_raw_frame(const std::vector<LaserPoint>& points, Color color, double scan_rate = 15000.0);
  // Sends raw points with their individual colors; x/y must be in [-1, 1].
  bool show_raw_frame(const std::vector<LaserPoint>& points, double scan_rate = 15000.0);
  const std::vector<LaserPoint>& last_frame() const { return last_frame_; }
private:
  std::shared_ptr<const ProjectionSurface> surface_;
  std::shared_ptr<LaserBackend> backend_;
  Calibration calibration_;
  Calibration2D surface_calibration_;
  std::vector<LaserPoint> last_frame_;
};

}  // namespace laser_projection
