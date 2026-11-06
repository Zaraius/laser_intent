#include "laser_projection_core/svg.hpp"

#include <libxml/parser.h>
#include <libxml/tree.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace laser_projection {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Transform {
  double a{1.0}, b{}, c{}, d{1.0}, e{}, f{};

  Point2D apply(Point2D point) const {
    return {a * point.x + c * point.y + e,
            b * point.x + d * point.y + f};
  }
};

Transform multiply(const Transform& left, const Transform& right) {
  return {
      left.a * right.a + left.c * right.b,
      left.b * right.a + left.d * right.b,
      left.a * right.c + left.c * right.d,
      left.b * right.c + left.d * right.d,
      left.a * right.e + left.c * right.f + left.e,
      left.b * right.e + left.d * right.f + left.f};
}

struct Subpath {
  std::vector<Point2D> points;
  bool closed{};
};

struct Shape {
  std::vector<Subpath> paths;
  std::optional<Color> fill;
  std::optional<Color> stroke;
  bool evenodd{};
};

struct Style {
  std::string fill{"black"};
  std::string stroke{"none"};
  std::string fill_rule{"nonzero"};
  bool visible{true};
};

struct StrokePath {
  std::vector<Point2D> points;
  Color color;
};

std::string trim(std::string value) {
  const auto first = std::find_if_not(value.begin(), value.end(),
      [](unsigned char ch) { return std::isspace(ch) != 0; });
  const auto last = std::find_if_not(value.rbegin(), value.rend(),
      [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
  return first < last ? std::string(first, last) : std::string{};
}

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

std::optional<std::string> attribute(xmlNode* node, const char* name) {
  xmlChar* value = xmlGetProp(node, BAD_CAST name);
  if (!value) return std::nullopt;
  std::string result(reinterpret_cast<const char*>(value));
  xmlFree(value);
  return result;
}

double parse_number(const std::string& value, const char* label) {
  const char* begin = value.c_str();
  char* end = nullptr;
  const double result = std::strtod(begin, &end);
  if (end == begin || !std::isfinite(result))
    throw std::runtime_error(std::string("Invalid SVG ") + label + ": " + value);
  return result;
}

double numeric_attribute(xmlNode* node, const char* name, double fallback = 0.0) {
  const auto value = attribute(node, name);
  return value ? parse_number(*value, name) : fallback;
}

bool is_command_token(const std::string& token) {
  return token.size() == 1 &&
      std::isalpha(static_cast<unsigned char>(token.front())) != 0;
}

std::vector<std::string> tokenize_numbers(const std::string& value) {
  std::vector<std::string> tokens;
  const char* cursor = value.c_str();
  while (*cursor) {
    if (std::isspace(static_cast<unsigned char>(*cursor)) || *cursor == ',') {
      ++cursor;
      continue;
    }
    if (std::isalpha(static_cast<unsigned char>(*cursor))) {
      tokens.emplace_back(1, *cursor++);
      continue;
    }
    char* end = nullptr;
    std::strtod(cursor, &end);
    if (end == cursor)
      throw std::runtime_error("Invalid SVG number list: " + value);
    tokens.emplace_back(cursor, static_cast<std::size_t>(end - cursor));
    cursor = end;
  }
  return tokens;
}

double token_number(const std::vector<std::string>& tokens, std::size_t& index) {
  if (index >= tokens.size() || is_command_token(tokens[index]))
    throw std::runtime_error("SVG path command has too few arguments");
  return parse_number(tokens[index++], "path coordinate");
}

double token_arc_flag(std::vector<std::string>& tokens, std::size_t& index) {
  if (index >= tokens.size() || is_command_token(tokens[index]))
    throw std::runtime_error("SVG arc command has too few arguments");
  auto& token = tokens[index];
  if (!token.empty() && (token.front() == '0' || token.front() == '1')) {
    const double flag = token.front() - '0';
    if (token.size() == 1) ++index;
    else token.erase(token.begin());
    return flag;
  }
  const double flag = token_number(tokens, index);
  if (flag != 0.0 && flag != 1.0)
    throw std::runtime_error("SVG arc flags must be 0 or 1");
  return flag;
}

void append_cubic(Subpath& path, Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
  constexpr int steps = 20;
  for (int step = 1; step <= steps; ++step) {
    const double t = static_cast<double>(step) / steps;
    const double u = 1.0 - t;
    path.points.push_back({
        u * u * u * p0.x + 3.0 * u * u * t * p1.x +
            3.0 * u * t * t * p2.x + t * t * t * p3.x,
        u * u * u * p0.y + 3.0 * u * u * t * p1.y +
            3.0 * u * t * t * p2.y + t * t * t * p3.y});
  }
}

void append_quadratic(Subpath& path, Point2D p0, Point2D p1, Point2D p2) {
  constexpr int steps = 16;
  for (int step = 1; step <= steps; ++step) {
    const double t = static_cast<double>(step) / steps;
    const double u = 1.0 - t;
    path.points.push_back({
        u * u * p0.x + 2.0 * u * t * p1.x + t * t * p2.x,
        u * u * p0.y + 2.0 * u * t * p1.y + t * t * p2.y});
  }
}

void append_arc(Subpath& path, Point2D start, double rx, double ry,
                double rotation, bool large_arc, bool sweep, Point2D end) {
  rx = std::abs(rx);
  ry = std::abs(ry);
  if (rx <= 1e-12 || ry <= 1e-12 ||
      (std::abs(start.x - end.x) < 1e-12 &&
       std::abs(start.y - end.y) < 1e-12)) {
    path.points.push_back(end);
    return;
  }

  const double phi = rotation * kPi / 180.0;
  const double cos_phi = std::cos(phi);
  const double sin_phi = std::sin(phi);
  const double dx = (start.x - end.x) / 2.0;
  const double dy = (start.y - end.y) / 2.0;
  const double x_prime = cos_phi * dx + sin_phi * dy;
  const double y_prime = -sin_phi * dx + cos_phi * dy;
  const double radius_scale = x_prime * x_prime / (rx * rx) +
                              y_prime * y_prime / (ry * ry);
  if (radius_scale > 1.0) {
    const double factor = std::sqrt(radius_scale);
    rx *= factor;
    ry *= factor;
  }

  const double numerator = std::max(
      0.0, rx * rx * ry * ry - rx * rx * y_prime * y_prime -
               ry * ry * x_prime * x_prime);
  const double denominator = rx * rx * y_prime * y_prime +
                             ry * ry * x_prime * x_prime;
  const double sign = large_arc == sweep ? -1.0 : 1.0;
  const double coefficient = denominator <= 1e-18
      ? 0.0 : sign * std::sqrt(numerator / denominator);
  const double cx_prime = coefficient * rx * y_prime / ry;
  const double cy_prime = coefficient * -ry * x_prime / rx;
  const double cx = cos_phi * cx_prime - sin_phi * cy_prime +
                    (start.x + end.x) / 2.0;
  const double cy = sin_phi * cx_prime + cos_phi * cy_prime +
                    (start.y + end.y) / 2.0;
  const auto vector_angle = [](double ux, double uy, double vx, double vy) {
    const double dot = ux * vx + uy * vy;
    const double det = ux * vy - uy * vx;
    return std::atan2(det, dot);
  };
  const double ux = (x_prime - cx_prime) / rx;
  const double uy = (y_prime - cy_prime) / ry;
  const double vx = (-x_prime - cx_prime) / rx;
  const double vy = (-y_prime - cy_prime) / ry;
  double start_angle = std::atan2(uy, ux);
  double sweep_angle = vector_angle(ux, uy, vx, vy);
  if (!sweep && sweep_angle > 0.0) sweep_angle -= 2.0 * kPi;
  if (sweep && sweep_angle < 0.0) sweep_angle += 2.0 * kPi;
  const int steps = std::max(1, static_cast<int>(
      std::ceil(std::abs(sweep_angle) / (kPi / 18.0))));
  for (int step = 1; step <= steps; ++step) {
    const double angle = start_angle + sweep_angle * step / steps;
    const double x = rx * std::cos(angle);
    const double y = ry * std::sin(angle);
    path.points.push_back({
        cos_phi * x - sin_phi * y + cx,
        sin_phi * x + cos_phi * y + cy});
  }
  path.points.back() = end;
}

std::vector<Subpath> parse_path(const std::string& data) {
  auto tokens = tokenize_numbers(data);
  std::vector<Subpath> paths;
  std::size_t index = 0;
  std::size_t active = std::numeric_limits<std::size_t>::max();
  char command = 0;
  Point2D current{};
  Point2D start{};
  Point2D previous_cubic{};
  Point2D previous_quadratic{};
  char previous_command = 0;
  auto ensure_active = [&]() -> Subpath& {
    if (active == std::numeric_limits<std::size_t>::max()) {
      paths.push_back({{current}, false});
      active = paths.size() - 1;
      start = current;
    }
    return paths[active];
  };
  auto read_point = [&](bool relative) {
    Point2D point{token_number(tokens, index), token_number(tokens, index)};
    if (relative) {
      point.x += current.x;
      point.y += current.y;
    }
    return point;
  };

  while (index < tokens.size()) {
    if (is_command_token(tokens[index])) command = tokens[index++].front();
    else if (!command)
      throw std::runtime_error("SVG path data must start with a command");

    const bool relative = std::islower(static_cast<unsigned char>(command)) != 0;
    const char upper = static_cast<char>(
        std::toupper(static_cast<unsigned char>(command)));
    if (upper == 'Z') {
      if (active != std::numeric_limits<std::size_t>::max()) {
        auto& path = paths[active];
        if (!path.points.empty() &&
            (path.points.back().x != start.x || path.points.back().y != start.y))
          path.points.push_back(start);
        path.closed = true;
        current = start;
        active = std::numeric_limits<std::size_t>::max();
      }
      previous_command = command;
      command = 0;
      continue;
    }

    if (upper != 'M' && upper != 'L' && upper != 'H' && upper != 'V' &&
        upper != 'C' && upper != 'S' && upper != 'Q' && upper != 'T' &&
        upper != 'A')
      throw std::runtime_error(std::string("Unsupported SVG path command: ") + command);
    if (index >= tokens.size() || is_command_token(tokens[index]))
      throw std::runtime_error("SVG path command has no coordinates");

    bool first_move = upper == 'M';
    do {
      if (upper == 'M' && first_move) {
        const auto point = read_point(relative);
        paths.push_back({{point}, false});
        active = paths.size() - 1;
        start = point;
        current = point;
        first_move = false;
        previous_command = command;
        continue;
      }
      auto& path = ensure_active();
      if (upper == 'M') {
        current = read_point(relative);
        path.points.push_back(current);
      } else if (upper == 'L') {
        current = read_point(relative);
        path.points.push_back(current);
      } else if (upper == 'H') {
        double x = token_number(tokens, index);
        if (relative) x += current.x;
        current = {x, current.y};
        path.points.push_back(current);
      } else if (upper == 'V') {
        double y = token_number(tokens, index);
        if (relative) y += current.y;
        current = {current.x, y};
        path.points.push_back(current);
      } else if (upper == 'C') {
        const Point2D p0 = current;
        const auto p1 = read_point(relative);
        const auto p2 = read_point(relative);
        const auto p3 = read_point(relative);
        append_cubic(path, p0, p1, p2, p3);
        current = p3;
        previous_cubic = p2;
      } else if (upper == 'S') {
        const Point2D p0 = current;
        const Point2D p1 = (previous_command == 'C' || previous_command == 'c' ||
                            previous_command == 'S' || previous_command == 's')
            ? Point2D{2.0 * current.x - previous_cubic.x,
                      2.0 * current.y - previous_cubic.y} : current;
        const auto p2 = read_point(relative);
        const auto p3 = read_point(relative);
        append_cubic(path, p0, p1, p2, p3);
        current = p3;
        previous_cubic = p2;
      } else if (upper == 'Q') {
        const auto control = read_point(relative);
        const auto end = read_point(relative);
        append_quadratic(path, current, control, end);
        current = end;
        previous_quadratic = control;
      } else if (upper == 'T') {
        const Point2D control =
            (previous_command == 'Q' || previous_command == 'q' ||
             previous_command == 'T' || previous_command == 't')
                ? Point2D{2.0 * current.x - previous_quadratic.x,
                          2.0 * current.y - previous_quadratic.y}
                : current;
        const auto end = read_point(relative);
        append_quadratic(path, current, control, end);
        current = end;
        previous_quadratic = control;
      } else if (upper == 'A') {
        const double rx = token_number(tokens, index);
        const double ry = token_number(tokens, index);
        const double rotation = token_number(tokens, index);
        const double large_arc = token_arc_flag(tokens, index);
        const double sweep = token_arc_flag(tokens, index);
        const auto end = read_point(relative);
        append_arc(path, current, rx, ry, rotation,
                   large_arc == 1.0, sweep == 1.0, end);
        current = end;
      }
      previous_command = command;
    } while (index < tokens.size() && !is_command_token(tokens[index]));

    if (upper == 'M') command = relative ? 'l' : 'L';
  }
  return paths;
}

Transform parse_transform(const std::string& value) {
  Transform result;
  std::size_t position = 0;
  while (position < value.size()) {
    while (position < value.size() &&
           (std::isspace(static_cast<unsigned char>(value[position])) ||
            value[position] == ',')) ++position;
    if (position == value.size()) break;
    const auto open = value.find('(', position);
    const auto close = open == std::string::npos ? std::string::npos :
        value.find(')', open + 1);
    if (open == std::string::npos || close == std::string::npos)
      throw std::runtime_error("Malformed SVG transform: " + value);
    const auto name = trim(value.substr(position, open - position));
    const auto args = tokenize_numbers(value.substr(open + 1, close - open - 1));
    std::vector<double> numbers;
    numbers.reserve(args.size());
    for (const auto& arg : args) {
      if (is_command_token(arg))
        throw std::runtime_error("Invalid SVG transform arguments: " + value);
      numbers.push_back(parse_number(arg, "transform"));
    }
    Transform local;
    if (name == "matrix" && numbers.size() == 6) {
      local = {numbers[0], numbers[1], numbers[2], numbers[3],
               numbers[4], numbers[5]};
    } else if (name == "translate" && (numbers.size() == 1 || numbers.size() == 2)) {
      local.e = numbers[0];
      local.f = numbers.size() == 2 ? numbers[1] : 0.0;
    } else if (name == "scale" && (numbers.size() == 1 || numbers.size() == 2)) {
      local.a = numbers[0];
      local.d = numbers.size() == 2 ? numbers[1] : numbers[0];
    } else if (name == "rotate" && (numbers.size() == 1 || numbers.size() == 3)) {
      const double angle = numbers[0] * kPi / 180.0;
      local.a = std::cos(angle);
      local.b = std::sin(angle);
      local.c = -std::sin(angle);
      local.d = std::cos(angle);
      if (numbers.size() == 3) {
        const Transform to_origin{1, 0, 0, 1, -numbers[1], -numbers[2]};
        const Transform from_origin{1, 0, 0, 1, numbers[1], numbers[2]};
        local = multiply(from_origin, multiply(local, to_origin));
      }
    } else if (name == "skewX" && numbers.size() == 1) {
      local.c = std::tan(numbers[0] * kPi / 180.0);
    } else if (name == "skewY" && numbers.size() == 1) {
      local.b = std::tan(numbers[0] * kPi / 180.0);
    } else {
      throw std::runtime_error("Unsupported or malformed SVG transform: " + name);
    }
    result = multiply(result, local);
    position = close + 1;
  }
  return result;
}

std::map<std::string, std::string> style_properties(xmlNode* node) {
  std::map<std::string, std::string> properties;
  for (const char* name : {"fill", "stroke", "fill-rule", "visibility",
                           "display", "opacity", "fill-opacity", "stroke-opacity"}) {
    if (auto value = attribute(node, name)) properties[name] = trim(*value);
  }
  if (auto style = attribute(node, "style")) {
    std::istringstream input(*style);
    std::string declaration;
    while (std::getline(input, declaration, ';')) {
      const auto colon = declaration.find(':');
      if (colon == std::string::npos) continue;
      properties[trim(declaration.substr(0, colon))] =
          trim(declaration.substr(colon + 1));
    }
  }
  return properties;
}

Style child_style(const Style& parent, xmlNode* node, bool& opacity_warning) {
  Style style = parent;
  const auto properties = style_properties(node);
  auto apply_inherited = [&](const char* key, std::string& destination) {
    const auto it = properties.find(key);
    if (it != properties.end() && it->second != "inherit")
      destination = it->second;
  };
  apply_inherited("fill", style.fill);
  apply_inherited("stroke", style.stroke);
  apply_inherited("fill-rule", style.fill_rule);
  const auto visibility = properties.find("visibility");
  if (visibility != properties.end()) {
    if (visibility->second == "hidden" || visibility->second == "collapse")
      style.visible = false;
    else if (visibility->second == "visible")
      style.visible = true;
  }
  for (const char* key : {"opacity", "fill-opacity", "stroke-opacity"}) {
    const auto it = properties.find(key);
    if (it != properties.end() && it->second != "inherit" &&
        std::abs(parse_number(it->second, key) - 1.0) > 1e-12)
      opacity_warning = true;
  }
  return style;
}

std::optional<Color> parse_color(std::string value) {
  value = lower(trim(value));
  if (value == "none" || value == "transparent") return std::nullopt;
  static const std::map<std::string, std::array<unsigned char, 3>> named{
      {"black", {0, 0, 0}}, {"white", {255, 255, 255}},
      {"red", {255, 0, 0}}, {"green", {0, 128, 0}},
      {"blue", {0, 0, 255}}, {"yellow", {255, 255, 0}},
      {"cyan", {0, 255, 255}}, {"aqua", {0, 255, 255}},
      {"magenta", {255, 0, 255}}, {"fuchsia", {255, 0, 255}},
      {"gray", {128, 128, 128}}, {"grey", {128, 128, 128}},
      {"silver", {192, 192, 192}}, {"maroon", {128, 0, 0}},
      {"olive", {128, 128, 0}}, {"lime", {0, 255, 0}},
      {"teal", {0, 128, 128}}, {"navy", {0, 0, 128}},
      {"purple", {128, 0, 128}}, {"orange", {255, 165, 0}}};
  if (const auto it = named.find(value); it != named.end())
    return Color{it->second[0], it->second[1], it->second[2]};
  if (!value.empty() && value.front() == '#') {
    const auto hex = value.substr(1);
    if (!std::all_of(hex.begin(), hex.end(), [](unsigned char ch) {
          return std::isxdigit(ch) != 0;
        }))
      throw std::runtime_error("Unsupported SVG color: " + value);
    auto byte = [&](std::size_t index) {
      return static_cast<unsigned char>(std::strtoul(hex.substr(index, 2).c_str(),
                                                      nullptr, 16));
    };
    if (hex.size() == 3) {
      auto nibble = [](char ch) {
        const char text[] = {ch, '\0'};
        return static_cast<unsigned char>(std::strtoul(text, nullptr, 16));
      };
      return Color{static_cast<unsigned char>(nibble(hex[0]) * 17u),
                   static_cast<unsigned char>(nibble(hex[1]) * 17u),
                   static_cast<unsigned char>(nibble(hex[2]) * 17u)};
    }
    if (hex.size() == 6)
      return Color{byte(0), byte(2), byte(4)};
    throw std::runtime_error("Unsupported SVG color: " + value);
  }
  if (value.rfind("rgb(", 0) == 0 && value.back() == ')') {
    std::array<unsigned char, 3> channels{};
    const auto body = value.substr(4, value.size() - 5);
    std::string components;
    for (char ch : body) components.push_back(ch == ',' ? ' ' : ch);
    std::istringstream input(components);
    for (std::size_t i = 0; i < 3; ++i) {
      std::string component;
      if (!(input >> component))
        throw std::runtime_error("Invalid SVG rgb() color: " + value);
      const bool percent = !component.empty() && component.back() == '%';
      if (percent) component.pop_back();
      double channel = parse_number(component, "color channel");
      if (percent) channel *= 2.55;
      channels[i] = static_cast<unsigned char>(
          std::lround(std::clamp(channel, 0.0, 255.0)));
    }
    std::string extra;
    if (input >> extra)
      throw std::runtime_error("Invalid SVG rgb() color: " + value);
    return Color{channels[0], channels[1], channels[2]};
  }
  throw std::runtime_error("Unsupported SVG color: " + value);
}

std::vector<Point2D> parse_points(const std::string& value) {
  const auto tokens = tokenize_numbers(value);
  if (tokens.size() % 2 != 0)
    throw std::runtime_error("SVG polyline/polygon needs coordinate pairs");
  std::vector<Point2D> points;
  points.reserve(tokens.size() / 2);
  for (std::size_t i = 0; i < tokens.size(); i += 2)
    points.push_back({parse_number(tokens[i], "point coordinate"),
                      parse_number(tokens[i + 1], "point coordinate")});
  return points;
}

void append_ellipse(Subpath& path, double cx, double cy, double rx, double ry) {
  constexpr int steps = 64;
  for (int step = 0; step < steps; ++step) {
    const double angle = 2.0 * kPi * step / steps;
    path.points.push_back({cx + rx * std::cos(angle),
                           cy + ry * std::sin(angle)});
  }
  path.points.push_back(path.points.front());
  path.closed = true;
}

std::vector<Subpath> element_geometry(xmlNode* node, const std::string& tag) {
  std::vector<Subpath> paths;
  if (tag == "path") {
    if (auto data = attribute(node, "d")) paths = parse_path(*data);
  } else if (tag == "line") {
    paths.push_back({{{numeric_attribute(node, "x1"), numeric_attribute(node, "y1")},
                      {numeric_attribute(node, "x2"), numeric_attribute(node, "y2")}}, false});
  } else if (tag == "polyline" || tag == "polygon") {
    const auto points = attribute(node, "points");
    if (points) {
      auto coordinates = parse_points(*points);
      const bool closed = tag == "polygon";
      if (closed && coordinates.size() > 1 &&
          (coordinates.front().x != coordinates.back().x ||
           coordinates.front().y != coordinates.back().y))
        coordinates.push_back(coordinates.front());
      paths.push_back({std::move(coordinates), closed});
    }
  } else if (tag == "rect") {
    const double x = numeric_attribute(node, "x");
    const double y = numeric_attribute(node, "y");
    const double width = numeric_attribute(node, "width");
    const double height = numeric_attribute(node, "height");
    if (width > 0.0 && height > 0.0) {
      double rx = numeric_attribute(node, "rx");
      double ry = numeric_attribute(node, "ry");
      if (rx == 0.0) rx = ry;
      if (ry == 0.0) ry = rx;
      rx = std::clamp(rx, 0.0, width / 2.0);
      ry = std::clamp(ry, 0.0, height / 2.0);
      Subpath path;
      if (rx == 0.0 || ry == 0.0) {
        path.points = {{x, y}, {x + width, y}, {x + width, y + height},
                       {x, y + height}, {x, y}};
      } else {
        constexpr int corner_steps = 8;
        const std::array<Point2D, 4> centers{{
            {x + width - rx, y + ry}, {x + width - rx, y + height - ry},
            {x + rx, y + height - ry}, {x + rx, y + ry}}};
        const std::array<double, 4> starts{{-kPi / 2.0, 0.0, kPi / 2.0, kPi}};
        for (std::size_t corner = 0; corner < centers.size(); ++corner) {
          for (int step = 0; step < corner_steps; ++step) {
            const double angle = starts[corner] + (kPi / 2.0) * step / corner_steps;
            path.points.push_back({centers[corner].x + rx * std::cos(angle),
                                   centers[corner].y + ry * std::sin(angle)});
          }
        }
        path.points.push_back(path.points.front());
      }
      path.closed = true;
      paths.push_back(std::move(path));
    }
  } else if (tag == "circle") {
    const double radius = numeric_attribute(node, "r");
    if (radius > 0.0) {
      Subpath path;
      append_ellipse(path, numeric_attribute(node, "cx"),
                     numeric_attribute(node, "cy"), radius, radius);
      paths.push_back(std::move(path));
    }
  } else if (tag == "ellipse") {
    const double rx = numeric_attribute(node, "rx");
    const double ry = numeric_attribute(node, "ry");
    if (rx > 0.0 && ry > 0.0) {
      Subpath path;
      append_ellipse(path, numeric_attribute(node, "cx"),
                     numeric_attribute(node, "cy"), rx, ry);
      paths.push_back(std::move(path));
    }
  }
  return paths;
}

void visit(xmlNode* node, const Transform& parent_transform,
           const Style& parent_style, bool& opacity_warning,
           std::vector<std::string>& warnings, std::vector<Shape>& shapes) {
  for (xmlNode* current = node; current; current = current->next) {
    if (current->type != XML_ELEMENT_NODE) continue;
    const std::string tag(reinterpret_cast<const char*>(current->name));
    if (tag == "defs" || tag == "metadata" || tag == "title" ||
        tag == "desc" ||
        tag == "pattern" || tag == "linearGradient" || tag == "radialGradient")
      continue;
    if (tag == "clipPath" || tag == "mask") {
      warnings.push_back("SVG clipping and masks are not applied");
      continue;
    }

    for (const char* effect : {"clip-path", "mask", "filter"}) {
      if (const auto value = attribute(current, effect);
          value && *value != "none")
        warnings.push_back(std::string("SVG effect ") + effect + " is ignored");
    }

    const Style style = child_style(parent_style, current, opacity_warning);
    const auto properties = style_properties(current);
    const auto display = properties.find("display");
    if (display != properties.end() && display->second == "none")
      continue;
    if (!style.visible) continue;
    Transform transform = parent_transform;
    if (const auto local = attribute(current, "transform"))
      transform = multiply(parent_transform, parse_transform(*local));

    const bool is_shape = tag == "path" || tag == "line" ||
        tag == "polyline" || tag == "polygon" || tag == "rect" ||
        tag == "circle" || tag == "ellipse";
    if (is_shape) {
      auto paths = element_geometry(current, tag);
      paths.erase(std::remove_if(paths.begin(), paths.end(),
          [](const Subpath& path) { return path.points.empty(); }), paths.end());
      if (paths.empty()) continue;
      auto fill = parse_color(style.fill);
      auto stroke = parse_color(style.stroke);
      if (!fill && !stroke) continue;
      for (auto& path : paths)
        for (auto& point : path.points) point = transform.apply(point);
      shapes.push_back({std::move(paths), fill, stroke,
                        lower(style.fill_rule) == "evenodd"});
      continue;
    }

    if (tag == "svg" || tag == "g" || tag == "a") {
      visit(current->children, transform, style, opacity_warning, warnings, shapes);
      continue;
    }
    warnings.push_back("Ignoring unsupported SVG element <" + tag + ">");
  }
}

struct Bounds {
  double min_x{std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};

  void include(Point2D point) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  bool valid() const {
    return std::isfinite(min_x) && std::isfinite(min_y) &&
           std::isfinite(max_x) && std::isfinite(max_y);
  }
};

void map_shape(Shape& shape, const Bounds& source, const SvgRenderOptions& options,
               double global_shrink) {
  const double source_width = source.max_x - source.min_x;
  const double source_height = source.max_y - source.min_y;
  const double target_width = options.top_right.x - options.bottom_left.x;
  const double target_height = options.top_right.y - options.bottom_left.y;
  double fit_scale = std::numeric_limits<double>::infinity();
  if (source_width > 1e-12) fit_scale = target_width / source_width;
  if (source_height > 1e-12)
    fit_scale = std::min(fit_scale, target_height / source_height);
  if (!std::isfinite(fit_scale))
    throw std::runtime_error("SVG artwork has no measurable width or height");
  const Point2D source_center{(source.min_x + source.max_x) / 2.0,
                              (source.min_y + source.max_y) / 2.0};
  const Point2D target_center{(options.bottom_left.x + options.top_right.x) / 2.0,
                              (options.bottom_left.y + options.top_right.y) / 2.0};
  const double scale_x = options.scale_x * global_shrink;
  const double scale_y = options.scale_y * global_shrink;
  for (auto& path : shape.paths) {
    for (auto& point : path.points) {
      point.x = target_center.x +
          (point.x - source_center.x) * fit_scale * scale_x;
      point.y = target_center.y +
          (source_center.y - point.y) * fit_scale * scale_y;
    }
  }
}

void rotate_shape(Shape& shape, Point2D center, double cosine, double sine) {
  for (auto& path : shape.paths) {
    for (auto& point : path.points) {
      const double dx = point.x - center.x;
      const double dy = center.y - point.y;
      point.x = center.x + cosine * dx - sine * dy;
      point.y = center.y - (sine * dx + cosine * dy);
    }
  }
}

std::vector<std::pair<double, double>> hatch_intervals(
    const Shape& shape, double scan_y) {
  struct Crossing { double x; int winding; };
  std::vector<Crossing> crossings;
  for (const auto& path : shape.paths) {
    if (path.points.size() < 3) continue;
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto& a = path.points[i];
      const auto& b = path.points[(i + 1) % path.points.size()];
      if ((a.y <= scan_y && b.y > scan_y) ||
          (b.y <= scan_y && a.y > scan_y)) {
        const double x = a.x + (scan_y - a.y) *
            (b.x - a.x) / (b.y - a.y);
        crossings.push_back({x, b.y > a.y ? 1 : -1});
      }
    }
  }
  std::sort(crossings.begin(), crossings.end(),
      [](const Crossing& left, const Crossing& right) {
        return left.x < right.x;
      });

  std::vector<std::pair<double, double>> intervals;
  int winding = 0;
  bool inside = false;
  double start = 0.0;
  for (std::size_t i = 0; i < crossings.size();) {
    std::size_t end = i + 1;
    int total_winding = crossings[i].winding;
    int crossing_count = 1;
    while (end < crossings.size() &&
           std::abs(crossings[end].x - crossings[i].x) < 1e-10) {
      total_winding += crossings[end].winding;
      ++crossing_count;
      ++end;
    }
    const bool was_inside = inside;
    if (shape.evenodd) {
      if (crossing_count % 2 != 0) inside = !inside;
    } else {
      winding += total_winding;
      inside = winding != 0;
    }
    if (!was_inside && inside) start = crossings[i].x;
    else if (was_inside && !inside)
      intervals.emplace_back(start, crossings[i].x);
    i = end;
  }
  return intervals;
}

void append_point(std::vector<LaserPoint>& frame, Point2D point, Color color,
                  bool blank, std::size_t max_points) {
  if (frame.size() >= max_points)
    throw std::runtime_error("SVG frame exceeds max_points limit");
  frame.push_back({point.x, point.y, blank ? Color{0, 0, 0} : color, blank});
}

std::size_t step_count(double distance, double spacing, std::size_t max_points) {
  const double count = std::max(1.0, std::ceil(distance / spacing));
  if (!std::isfinite(count) || count > static_cast<double>(max_points))
    throw std::runtime_error("SVG path exceeds max_points limit");
  return static_cast<std::size_t>(count);
}

void append_blank_travel(std::vector<LaserPoint>& frame, Point2D from, Point2D to,
                         double spacing, std::size_t max_points) {
  const double distance = std::hypot(to.x - from.x, to.y - from.y);
  const auto steps = step_count(distance, spacing, max_points);
  append_point(frame, from, {}, true, max_points);
  for (std::size_t step = 1; step <= steps; ++step) {
    const double t = static_cast<double>(step) / static_cast<double>(steps);
    append_point(frame, {from.x + t * (to.x - from.x),
                         from.y + t * (to.y - from.y)}, {}, true, max_points);
  }
}

void append_lit_path(std::vector<LaserPoint>& frame, const StrokePath& path,
                     double spacing, std::size_t max_points) {
  if (path.points.empty()) return;
  append_point(frame, path.points.front(), path.color, false, max_points);
  if (path.points.size() == 1) {
    for (int i = 0; i < 7; ++i)
      append_point(frame, path.points.front(), path.color, false, max_points);
    return;
  }
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const auto& from = path.points[i - 1];
    const auto& to = path.points[i];
    const double distance = std::hypot(to.x - from.x, to.y - from.y);
    const auto steps = step_count(distance, spacing, max_points);
    for (std::size_t step = 1; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      append_point(frame, {from.x + t * (to.x - from.x),
                           from.y + t * (to.y - from.y)},
                   path.color, false, max_points);
    }
  }
}

void validate_options(const SvgRenderOptions& options) {
  const std::array<double, 9> values{{
      options.bottom_left.x, options.bottom_left.y,
      options.top_right.x, options.top_right.y,
      options.scale_x, options.scale_y, options.rotation_degrees,
      options.hatch_spacing, options.point_spacing}};
  if (!std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value); }))
    throw std::invalid_argument("SVG render options must be finite");
  if (options.bottom_left.x < -1.0 || options.bottom_left.y < -1.0 ||
      options.top_right.x > 1.0 || options.top_right.y > 1.0 ||
      options.bottom_left.x >= options.top_right.x ||
      options.bottom_left.y >= options.top_right.y)
    throw std::invalid_argument(
        "SVG target box must be ordered and entirely within [-1, 1]");
  if (options.scale_x <= 0.0 || options.scale_y <= 0.0)
    throw std::invalid_argument("SVG X/Y scale factors must be positive");
  if (options.hatch_spacing <= 0.0 || options.point_spacing <= 0.0)
    throw std::invalid_argument("SVG hatch and point spacing must be positive");
  if (options.max_points < 2)
    throw std::invalid_argument("SVG max_points must be at least 2");
}

}  // namespace

SvgRenderResult render_svg_file(const std::string& path,
                                const SvgRenderOptions& options) {
  validate_options(options);
  if (path.empty()) throw std::invalid_argument("SVG path must not be empty");
  xmlDocPtr document = xmlReadFile(path.c_str(), nullptr,
                                   XML_PARSE_NONET | XML_PARSE_NOBLANKS);
  if (!document) {
    const auto* error = xmlGetLastError();
    throw std::runtime_error("Could not parse SVG '" + path + "'" +
        (error && error->message ? ": " + trim(error->message) : ""));
  }

  SvgRenderResult result;
  std::vector<Shape> shapes;
  bool opacity_warning = false;
  const Style default_style;
  visit(xmlDocGetRootElement(document), {}, default_style, opacity_warning,
        result.warnings, shapes);
  xmlFreeDoc(document);
  if (opacity_warning)
    result.warnings.push_back(
        "SVG opacity is ignored; solid RGB colors are preserved");
  if (shapes.empty())
    throw std::runtime_error("SVG contains no supported visible vector geometry");

  Bounds source;
  for (const auto& shape : shapes)
    for (const auto& subpath : shape.paths)
      for (const auto point : subpath.points) source.include(point);
  if (!source.valid())
    throw std::runtime_error("SVG contains no measurable vector geometry");
  if (!std::isfinite(options.rotation_degrees))
    throw std::invalid_argument("SVG rotation must be finite");
  const Point2D source_center{(source.min_x + source.max_x) / 2.0,
                              (source.min_y + source.max_y) / 2.0};
  const double rotation =
      std::remainder(options.rotation_degrees, 360.0) * kPi / 180.0;
  const double cosine = std::cos(rotation);
  const double sine = std::sin(rotation);
  if (std::abs(options.rotation_degrees) > 1e-12) {
    for (auto& shape : shapes)
      rotate_shape(shape, source_center, cosine, sine);
    source = {};
    for (const auto& shape : shapes)
      for (const auto& subpath : shape.paths)
        for (const auto point : subpath.points) source.include(point);
  }
  const double source_width = source.max_x - source.min_x;
  const double source_height = source.max_y - source.min_y;
  if (!std::isfinite(source_width) || !std::isfinite(source_height))
    throw std::runtime_error("SVG bounds exceed the supported numeric range");
  if (source_width <= 1e-12 && source_height <= 1e-12)
    throw std::runtime_error("SVG artwork has zero width and height");

  const Point2D center{(options.bottom_left.x + options.top_right.x) / 2.0,
                       (options.bottom_left.y + options.top_right.y) / 2.0};
  const double target_width = options.top_right.x - options.bottom_left.x;
  const double target_height = options.top_right.y - options.bottom_left.y;
  double fit_scale = std::numeric_limits<double>::infinity();
  if (source_width > 1e-12) fit_scale = target_width / source_width;
  if (source_height > 1e-12)
    fit_scale = std::min(fit_scale, target_height / source_height);
  if (!std::isfinite(fit_scale) || fit_scale <= 0.0)
    throw std::runtime_error("SVG artwork cannot be scaled into the target box");

  const double extent_x = source_width * fit_scale / 2.0;
  const double extent_y = source_height * fit_scale / 2.0;
  const double available_x = std::min(1.0 - center.x, 1.0 + center.x);
  const double available_y = std::min(1.0 - center.y, 1.0 + center.y);
  double global_shrink = 1.0;
  if (extent_x > 0.0 && options.scale_x > available_x / extent_x)
    global_shrink = std::min(global_shrink,
                              (available_x / extent_x) / options.scale_x);
  if (extent_y > 0.0 && options.scale_y > available_y / extent_y)
    global_shrink = std::min(global_shrink,
                              (available_y / extent_y) / options.scale_y);
  if (global_shrink < 1.0) {
    global_shrink *= 1.0 - 1e-12;
    result.warnings.push_back(
        "SVG was uniformly reduced to keep the scaled artwork inside [-1, 1]");
  }
  for (auto& shape : shapes)
    map_shape(shape, source, options, global_shrink);

  std::vector<StrokePath> strokes;
  auto append_stroke = [&](StrokePath stroke) {
    if (strokes.size() >= options.max_points)
      throw std::runtime_error("SVG fill/stroke geometry exceeds max_points limit");
    strokes.push_back(std::move(stroke));
  };
  for (const auto& shape : shapes) {
    if (shape.fill) {
      Bounds fill_bounds;
      for (const auto& path : shape.paths)
        for (const auto point : path.points) fill_bounds.include(point);
      if (fill_bounds.valid() &&
          fill_bounds.max_y - fill_bounds.min_y > 1e-12) {
        double first_scan_y = fill_bounds.min_y + options.hatch_spacing / 2.0;
        if (first_scan_y > fill_bounds.max_y)
          first_scan_y = (fill_bounds.min_y + fill_bounds.max_y) / 2.0;
        const double row_count_value =
            std::floor((fill_bounds.max_y - first_scan_y) /
                       options.hatch_spacing) + 1.0;
        if (!std::isfinite(row_count_value) ||
            row_count_value > static_cast<double>(options.max_points))
          throw std::runtime_error("SVG fill exceeds max_points limit");
        const auto row_count = static_cast<std::size_t>(row_count_value);
        for (std::size_t row = 0; row < row_count; ++row) {
          const double scan_y = first_scan_y +
              static_cast<double>(row) * options.hatch_spacing;
          auto intervals = hatch_intervals(shape, scan_y);
          if (row % 2 != 0) std::reverse(intervals.begin(), intervals.end());
          for (const auto& [left, right] : intervals) {
            if (right - left <= 1e-12) continue;
            if (row % 2 == 0)
              append_stroke({{{left, scan_y}, {right, scan_y}}, *shape.fill});
            else
              append_stroke({{{right, scan_y}, {left, scan_y}}, *shape.fill});
          }
        }
      }
    }
    if (shape.stroke) {
      for (const auto& subpath : shape.paths) {
        if (subpath.points.empty()) continue;
        auto points = subpath.points;
        if (subpath.closed &&
            (points.front().x != points.back().x ||
             points.front().y != points.back().y))
          points.push_back(points.front());
        append_stroke({std::move(points), *shape.stroke});
      }
    }
  }
  if (strokes.empty())
    throw std::runtime_error("SVG contains no drawable fill or stroke paths");

  std::vector<StrokePath> sampled;
  for (auto& stroke : strokes) {
    if (stroke.points.size() <= 1) {
      sampled.push_back(std::move(stroke));
      continue;
    }
    StrokePath path;
    path.color = stroke.color;
    path.points.push_back(stroke.points.front());
    for (std::size_t i = 1; i < stroke.points.size(); ++i) {
      const auto& from = stroke.points[i - 1];
      const auto& to = stroke.points[i];
      const double distance = std::hypot(to.x - from.x, to.y - from.y);
      const auto steps = step_count(distance, options.point_spacing,
                                    options.max_points);
      if (path.points.size() + steps > options.max_points)
        throw std::runtime_error("SVG frame exceeds max_points limit");
      for (std::size_t step = 1; step <= steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(steps);
        path.points.push_back({from.x + t * (to.x - from.x),
                               from.y + t * (to.y - from.y)});
      }
    }
    sampled.push_back(std::move(path));
  }

  auto& frame = result.points;
  Point2D first = sampled.front().points.front();
  Point2D current = first;
  append_point(frame, first, {}, true, options.max_points);
  for (const auto& stroke : sampled) {
    if (stroke.points.empty()) continue;
    const auto start = stroke.points.front();
    if (std::hypot(start.x - current.x, start.y - current.y) > 1e-12)
      append_blank_travel(frame, current, start, options.point_spacing,
                          options.max_points);
    append_lit_path(frame, stroke, options.point_spacing, options.max_points);
    current = stroke.points.back();
  }
  append_blank_travel(frame, current, first, options.point_spacing,
                      options.max_points);
  return result;
}

}  // namespace laser_projection
