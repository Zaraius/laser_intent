#include "laser_projection_core/backends.hpp"
#include "laser_projection_core/projector.hpp"
#include "laser_projection_core/svg.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

constexpr const char* kUsage =
    "Usage: laser_svg_sender <file.svg> [options]\n"
    "  --bottom-left X Y   Target box bottom-left in raw units (default -1 -1)\n"
    "  --top-right X Y     Target box top-right in raw units (default 1 1)\n"
    "  --scale-x VALUE     X scale around target-box center (default 1)\n"
    "  --scale-y VALUE     Y scale around target-box center (default 1)\n"
    "  --hatch-spacing N   Fill hatch spacing in raw units (default 0.1)\n"
    "  --point-spacing N   Maximum point spacing in raw units (default 0.05)\n"
    "  --max-points N      Maximum generated frame points (default 500000)\n"
    "  --scan-rate N       DAC scan rate (default 15000)\n"
    "  --host ADDRESS      UDP destination IPv4 address (default 127.0.0.1)\n"
    "  --port PORT         UDP destination port (default 2345)\n"
    "  --help              Show this help";

double parse_double(const std::string& value, const char* option) {
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result))
    throw std::invalid_argument(std::string("Invalid value for ") + option);
  return result;
}

std::string required_value(int& index, int argc, char** argv, const char* option) {
  if (++index >= argc)
    throw std::invalid_argument(std::string("Missing value for ") + option);
  return argv[index];
}

void parse_pair(int& index, int argc, char** argv, laser_projection::Point2D& point,
                const char* option) {
  const auto x = required_value(index, argc, argv, option);
  const auto y = required_value(index, argc, argv, option);
  point.x = parse_double(x, option);
  point.y = parse_double(y, option);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << kUsage << '\n';
    return 2;
  }

  try {
    std::string svg_path;
    std::string host = "127.0.0.1";
    unsigned long port = 2345;
    double scan_rate = 15000.0;
    laser_projection::SvgRenderOptions options;
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--help") {
        std::cout << kUsage << '\n';
        return 0;
      } else if (argument == "--bottom-left") {
        parse_pair(i, argc, argv, options.bottom_left, "--bottom-left");
      } else if (argument == "--top-right") {
        parse_pair(i, argc, argv, options.top_right, "--top-right");
      } else if (argument == "--scale-x") {
        options.scale_x = parse_double(required_value(i, argc, argv, "--scale-x"),
                                       "--scale-x");
      } else if (argument == "--scale-y") {
        options.scale_y = parse_double(required_value(i, argc, argv, "--scale-y"),
                                       "--scale-y");
      } else if (argument == "--hatch-spacing") {
        options.hatch_spacing = parse_double(
            required_value(i, argc, argv, "--hatch-spacing"), "--hatch-spacing");
      } else if (argument == "--point-spacing") {
        options.point_spacing = parse_double(
            required_value(i, argc, argv, "--point-spacing"), "--point-spacing");
      } else if (argument == "--max-points") {
        const auto text = required_value(i, argc, argv, "--max-points");
        if (text.empty() || text.front() == '-')
          throw std::invalid_argument("max-points must be a positive integer");
        std::size_t consumed = 0;
        const auto limit = std::stoull(text, &consumed);
        if (consumed != text.size() || limit < 2)
          throw std::invalid_argument("max-points must be at least 2");
        if (limit > std::numeric_limits<std::size_t>::max())
          throw std::invalid_argument("max-points is too large");
        options.max_points = static_cast<std::size_t>(limit);
      } else if (argument == "--scan-rate") {
        scan_rate = parse_double(required_value(i, argc, argv, "--scan-rate"),
                                 "--scan-rate");
      } else if (argument == "--host") {
        host = required_value(i, argc, argv, "--host");
      } else if (argument == "--port") {
        const auto text = required_value(i, argc, argv, "--port");
        std::size_t consumed = 0;
        port = std::stoul(text, &consumed);
        if (consumed != text.size() || port == 0 || port > 65535)
          throw std::invalid_argument("UDP port must be in [1, 65535]");
      } else if (!argument.empty() && argument.front() == '-') {
        throw std::invalid_argument("Unknown option: " + argument);
      } else if (svg_path.empty()) {
        svg_path = argument;
      } else {
        throw std::invalid_argument("Only one SVG input path may be specified");
      }
    }

    if (svg_path.empty()) throw std::invalid_argument("An SVG input path is required");
    if (!std::isfinite(scan_rate) || scan_rate <= 0.0)
      throw std::invalid_argument("Scan rate must be positive");
    if (host.empty()) throw std::invalid_argument("UDP host must not be empty");

    auto rendered = laser_projection::render_svg_file(svg_path, options);
    for (const auto& warning : rendered.warnings)
      std::cerr << "Warning: " << warning << '\n';

    auto backend = std::make_shared<laser_projection::UdpBackend>(
        host, static_cast<unsigned short>(port));
    laser_projection::Projector projector(
        std::make_shared<laser_projection::PlaneSurface>(), backend);
    if (!projector.initialize())
      throw std::runtime_error("Could not initialize UDP backend");
    if (!projector.show_raw_frame(rendered.points, scan_rate))
      throw std::runtime_error("UDP backend failed to send the SVG frame");
    std::cout << "Sent " << rendered.points.size() << " laser points from "
              << svg_path << " to " << host << ':' << port << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "laser_svg_sender: " << error.what() << '\n'
              << "Run with --help for usage.\n";
    return 1;
  }
}
