#include "laser_projection_core/projector.hpp"
#include "laser_projection_core/svg.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {

class CaptureBackend final : public laser_projection::LaserBackend {
public:
  bool initialize() override { return true; }
  bool send_frame(const std::vector<laser_projection::LaserPoint>& points,
                  double) override {
    frame = points;
    return true;
  }
  bool blackout() override { return true; }
  std::vector<laser_projection::LaserPoint> frame;
};

class TemporarySvg {
public:
  explicit TemporarySvg(const std::string& content) {
    static unsigned int counter = 0;
    path_ = std::filesystem::temp_directory_path() /
        ("laser_projection_test_" + std::to_string(++counter) + ".svg");
    std::ofstream output(path_);
    output << content;
  }
  ~TemporarySvg() { std::filesystem::remove(path_); }
  std::string path() const { return path_.string(); }

private:
  std::filesystem::path path_;
};

}  // namespace

TEST(Svg, ParsesPathCommandsTransformsAndFitsToBox) {
  const TemporarySvg svg(
      "<svg xmlns='http://www.w3.org/2000/svg'><g transform='translate(5,2) scale(2)'>"
      "<path d='M0 0 H10 v10 l-10 0 z' fill='none' stroke='#f00'/>"
      "</g></svg>");
  laser_projection::SvgRenderOptions options;
  options.bottom_left = {-0.5, -0.5};
  options.top_right = {0.5, 0.5};
  options.point_spacing = 0.1;
  const auto rendered = laser_projection::render_svg_file(svg.path(), options);

  ASSERT_GT(rendered.points.size(), 10u);
  EXPECT_TRUE(rendered.points.front().blank);
  EXPECT_TRUE(rendered.points.back().blank);
  EXPECT_TRUE(std::all_of(rendered.points.begin(), rendered.points.end(),
      [](const auto& point) {
        return point.x >= -0.5 && point.x <= 0.5 &&
               point.y >= -0.5 && point.y <= 0.5;
      }));
  const auto red_points = std::count_if(rendered.points.begin(), rendered.points.end(),
      [](const auto& point) {
        return !point.blank && point.color.r == 255 &&
               point.color.g == 0 && point.color.b == 0;
      });
  const auto first_lit = std::find_if(rendered.points.begin(), rendered.points.end(),
      [](const auto& point) { return !point.blank; });
  ASSERT_NE(first_lit, rendered.points.end());
  EXPECT_NEAR(first_lit->x, -0.5, 1e-9);
  EXPECT_NEAR(first_lit->y, 0.5, 1e-9);
  EXPECT_EQ(first_lit->color.r, 255);
  EXPECT_EQ(first_lit->color.g, 0);
  EXPECT_EQ(first_lit->color.b, 0);
  EXPECT_GT(red_points, 10);
}

TEST(Svg, HatchesFilledShapesAndPreservesElementColors) {
  const TemporarySvg svg(
      "<svg><rect x='0' y='0' width='2' height='1' fill='rgb(0 0 255)' "
      "stroke='none'/><path d='M3 0 L4 1' fill='none' "
      "style='stroke: #00ff00'/></svg>");
  laser_projection::SvgRenderOptions options;
  options.point_spacing = 0.05;
  options.hatch_spacing = 0.1;
  const auto rendered = laser_projection::render_svg_file(svg.path(), options);

  const auto blue_points = std::count_if(rendered.points.begin(), rendered.points.end(),
      [](const auto& point) {
        return !point.blank && point.color.r == 0 &&
               point.color.g == 0 && point.color.b == 255;
      });
  const auto green_points = std::count_if(rendered.points.begin(), rendered.points.end(),
      [](const auto& point) {
        return !point.blank && point.color.r == 0 &&
               point.color.g == 255 && point.color.b == 0;
      });
  EXPECT_GT(blue_points, 10);
  EXPECT_GT(green_points, 1);
}

TEST(Svg, ScalesWholeImageIntoRawFieldAfterAxisScaling) {
  const TemporarySvg svg(
      "<svg><rect x='0' y='0' width='1' height='1' fill='none' "
      "stroke='white'/></svg>");
  laser_projection::SvgRenderOptions options;
  options.bottom_left = {0.8, -0.4};
  options.top_right = {1.0, 0.4};
  options.scale_x = 2.0;
  options.scale_y = 2.0;
  options.point_spacing = 0.05;
  const auto rendered = laser_projection::render_svg_file(svg.path(), options);

  ASSERT_FALSE(rendered.warnings.empty());
  EXPECT_NEAR(rendered.points.front().x, 0.8, 1e-9);
  EXPECT_NEAR(rendered.points.front().y, 0.1, 1e-9);
  EXPECT_TRUE(std::all_of(rendered.points.begin(), rendered.points.end(),
      [](const auto& point) {
        return point.x >= -1.0 && point.x <= 1.0 &&
               point.y >= -1.0 && point.y <= 1.0;
      }));
}

TEST(Svg, RotatesArtworkCounterclockwiseAroundItsCenter) {
      const TemporarySvg svg(
          "<svg><path d='M0 0 L2 1' fill='none' stroke='white'/></svg>");
      laser_projection::SvgRenderOptions options;
      options.rotation_degrees = 90.0;
      const auto rendered = laser_projection::render_svg_file(svg.path(), options);
      const auto first_lit = std::find_if(rendered.points.begin(), rendered.points.end(),
          [](const auto& point) { return !point.blank; });
      const auto last_lit = std::find_if(rendered.points.rbegin(), rendered.points.rend(),
          [](const auto& point) { return !point.blank; });
      ASSERT_NE(first_lit, rendered.points.end());
      ASSERT_NE(last_lit, rendered.points.rend());
      EXPECT_NEAR(first_lit->x, -0.5, 1e-9);
      EXPECT_NEAR(first_lit->y, -1.0, 1e-9);
      EXPECT_NEAR(last_lit->x, 0.5, 1e-9);
      EXPECT_NEAR(last_lit->y, 1.0, 1e-9);
}

TEST(Svg, RejectsInvalidBoxAndExcessiveFrame) {
  const TemporarySvg svg(
      "<svg><path d='M0 0 L1 1' fill='none' stroke='black'/></svg>");
  laser_projection::SvgRenderOptions invalid_box;
  invalid_box.bottom_left.x = -1.1;
  EXPECT_THROW(laser_projection::render_svg_file(svg.path(), invalid_box),
               std::invalid_argument);

  laser_projection::SvgRenderOptions limited;
  limited.max_points = 3;
  EXPECT_THROW(laser_projection::render_svg_file(svg.path(), limited),
               std::runtime_error);
}

TEST(Svg, ProjectorKeepsPerPointColorsForRawFrames) {
  auto backend = std::make_shared<CaptureBackend>();
  laser_projection::Projector projector(
      std::make_shared<laser_projection::PlaneSurface>(), backend);
  const std::vector<laser_projection::LaserPoint> points{
      {0.0, 0.0, {}, true},
      {0.1, 0.0, {255, 0, 0}, false},
      {0.2, 0.0, {0, 255, 0}, false},
      {0.0, 0.0, {}, true}};

  ASSERT_TRUE(projector.show_raw_frame(points, 15000.0));
  ASSERT_EQ(backend->frame.size(), points.size());
  EXPECT_EQ(backend->frame[1].color.r, 255);
  EXPECT_EQ(backend->frame[1].color.g, 0);
  EXPECT_EQ(backend->frame[2].color.r, 0);
  EXPECT_EQ(backend->frame[2].color.g, 255);
  EXPECT_EQ(backend->frame.front().color.r, 0);
  EXPECT_EQ(backend->frame.back().color.g, 0);
}

TEST(Svg, SupportsSmoothCurvesQuadraticsAndArcs) {
  const TemporarySvg svg(
      "<svg><path d='M0 10 C1 0 2 0 3 10 S5 20 6 10 "
      "Q7 0 8 10 T10 10 A1 1 0 0111 11 Z' "
      "fill='none' stroke='black'/></svg>");
  const auto rendered = laser_projection::render_svg_file(svg.path());
  EXPECT_GT(rendered.points.size(), 50u);
  EXPECT_TRUE(rendered.points.front().blank);
  EXPECT_TRUE(rendered.points.back().blank);
}
