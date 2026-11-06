#pragma once

#include "laser_projection_core/projector.hpp"
#include <cstddef>
#include <string>
#include <vector>

namespace laser_projection {

struct SvgRenderOptions {
  Point2D bottom_left{-1.0, -1.0};
  Point2D top_right{1.0, 1.0};
  double scale_x{1.0};
  double scale_y{1.0};
  double rotation_degrees{};
  double hatch_spacing{0.1};
  double point_spacing{0.05};
  std::size_t max_points{500000};
};

struct SvgRenderResult {
  std::vector<LaserPoint> points;
  std::vector<std::string> warnings;
};

SvgRenderResult render_svg_file(const std::string& path,
                                const SvgRenderOptions& options = {});

}  // namespace laser_projection
