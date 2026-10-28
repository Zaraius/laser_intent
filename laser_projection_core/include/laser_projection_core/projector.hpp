#pragma once

#include <memory>
#include <optional>
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
  virtual void blackout() = 0;
};

struct Calibration {
  double scale_x{1.0};
  double scale_y{1.0};
  double offset_x{};
  double offset_y{};
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
            Calibration calibration = {});
  bool initialize();
  bool show_path(const std::vector<Point3D>& path, ProjectorPose projector_pose,
                 PathOptions options = {}, double scan_rate = 30000.0);
  bool show_paths(const std::vector<std::vector<Point3D>>& paths,
                  ProjectorPose projector_pose, PathOptions options = {},
                  double scan_rate = 30000.0);
  const std::vector<LaserPoint>& last_frame() const { return last_frame_; }
private:
  std::shared_ptr<const ProjectionSurface> surface_;
  std::shared_ptr<LaserBackend> backend_;
  Calibration calibration_;
  std::vector<LaserPoint> last_frame_;
};

}  // namespace laser_projection
