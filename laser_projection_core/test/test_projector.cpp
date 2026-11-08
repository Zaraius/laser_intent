#include "laser_projection_core/projector.hpp"
#include "laser_projection_core/backends.hpp"
#include <algorithm>
#include <cstdint>
#include <gtest/gtest.h>
#include <stdexcept>

using namespace laser_projection;

extern "C" int FakeHeliosFrameSize();
extern "C" std::uint16_t FakeHeliosFrameValue(int index, int channel);

class TestBackend final : public LaserBackend {
public:
  bool initialize() override { return true; }
  bool send_frame(const std::vector<LaserPoint>& p, double) override { frame = p; return true; }
  void blackout() override { ++blackout_count; }
  std::vector<LaserPoint> frame;
  int blackout_count{};
};

TEST(Core, PlaneIntersectionAndHeight) {
  PlaneSurface plane;
  auto result = plane.intersect({{0, 0, 1}, {1, 0, -1}});
  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->x, 1.0);
}

TEST(Core, PathSamplingAndColor) {
  auto backend = std::make_shared<TestBackend>();
  Projector projector(std::make_shared<PlaneSurface>(), backend);
  ASSERT_TRUE(projector.show_path({{0, 0, 0}, {1, 0, 0}}, {{0, 0, 1}, {}},
                                  {{255, 0, 0}, 0.25}));
  EXPECT_EQ(backend->frame.front().color.r, 255);
  EXPECT_GE(backend->frame.size(), 5u);
  EXPECT_TRUE(backend->frame.front().blank);
}

TEST(Core, OptionalArrowAddsBlankedTravel) {
  auto backend = std::make_shared<TestBackend>();
  Projector projector(std::make_shared<PlaneSurface>(), backend);
  ASSERT_TRUE(projector.show_path({{0, 0, 0}, {1, 0, 0}}, {{0, 0, 1}, {}},
                                  {{255, 0, 0}, 0.25, true}));

  // Initial blank and lit shaft.
  ASSERT_GE(backend->frame.size(), 14u);
  EXPECT_TRUE(backend->frame[0].blank);
  for (size_t i = 1; i <= 5; ++i) EXPECT_FALSE(backend->frame[i].blank);

  // The shaft-to-left-arm travel is blank at both ends.
  EXPECT_TRUE(backend->frame[6].blank);
  EXPECT_TRUE(backend->frame[7].blank);
  EXPECT_FALSE(backend->frame[8].blank);

  // Find the blank transition between the two lit arm strokes.
  size_t second_transition = 8;
  while (second_transition < backend->frame.size() &&
         !backend->frame[second_transition].blank) {
    ++second_transition;
  }
  ASSERT_LT(second_transition + 2, backend->frame.size());
  EXPECT_TRUE(backend->frame[second_transition].blank);
  EXPECT_TRUE(backend->frame[second_transition + 1].blank);
  EXPECT_FALSE(backend->frame[second_transition + 2].blank);

  // The repeated frame returns to its first point with the laser blanked.
  EXPECT_GE(std::count_if(backend->frame.begin(), backend->frame.end(),
                          [](const LaserPoint& point) { return point.blank; }),
            10);
  EXPECT_TRUE(backend->frame.back().blank);
}

TEST(Core, MultiplePathsKeepBlankedTravelBetweenArrows) {
  auto backend = std::make_shared<TestBackend>();
  Projector projector(std::make_shared<PlaneSurface>(), backend);
  ASSERT_TRUE(projector.show_paths(
      {{{0, 0, 0}, {1, 0, 0}}, {{0, 1, 0}, {1, 1, 0}}},
      {{0, 0, 1}, {}}, {{255, 0, 0}, 0.25, true}));

  EXPECT_GT(std::count_if(backend->frame.begin(), backend->frame.end(),
                          [](const LaserPoint& point) { return point.blank; }),
            10);
  EXPECT_GT(backend->frame.size(), 32u);
  EXPECT_TRUE(backend->frame[0].blank);
  EXPECT_TRUE(backend->frame.back().blank);
}

TEST(Core, FrameReturnToFirstPointIsBlanked) {
  auto backend = std::make_shared<TestBackend>();
  Projector projector(std::make_shared<PlaneSurface>(), backend);
  ASSERT_TRUE(projector.show_path({{0, 0, 0}, {1, 0, 0}},
                                  {{0, 0, 1}, {}}, {{255, 0, 0}, 0.25}));

  ASSERT_GE(backend->frame.size(), 11u);
  EXPECT_TRUE(backend->frame.front().blank);
  EXPECT_FALSE(backend->frame[5].blank);
  for (size_t i = 6; i < backend->frame.size(); ++i)
    EXPECT_TRUE(backend->frame[i].blank);
  EXPECT_NEAR(backend->frame.back().x, backend->frame.front().x, 1e-9);
  EXPECT_NEAR(backend->frame.back().y, backend->frame.front().y, 1e-9);
}

TEST(Core, ProjectorOrientationChangesLocalOutput) {
  auto backend = std::make_shared<TestBackend>();
  Projector projector(std::make_shared<PlaneSurface>(), backend);
  const double half = 0.7071067811865476;
  ASSERT_TRUE(projector.show_path({{1, 0, 0}}, {{0, 0, 1}, {0, 0, half, half}},
                                  {{255, 0, 0}, 0.25}));
  EXPECT_NEAR(backend->frame.front().x, 0.0, 1e-9);
  EXPECT_NEAR(backend->frame.front().y, -1.0, 1e-9);
}

TEST(Core, SinglePointProducesLitSamples) {
  auto backend = std::make_shared<TestBackend>();
  Projector projector(std::make_shared<PlaneSurface>(), backend);
  ASSERT_TRUE(projector.show_path({{0, 0, 0}}, {{0, 0, 1}, {}}));
  EXPECT_EQ(backend->frame.size(), 9u);
  EXPECT_FALSE(backend->frame.back().blank);
}

TEST(Core, RawFrameBypassesProjectionAndCalibration) {
  auto backend = std::make_shared<TestBackend>();
  Projector projector(std::make_shared<PlaneSurface>(), backend);
  const std::vector<LaserPoint> points{
      {-0.75, 0.5, {255, 0, 0}, true},
      {0.25, -0.5, {0, 255, 0}, false},
      {0.9, 0.8, {0, 0, 255}, true}};

  const Color color{12, 34, 56};
  ASSERT_TRUE(projector.show_raw_frame(points, color));
  ASSERT_EQ(backend->frame.size(), points.size());
  EXPECT_DOUBLE_EQ(backend->frame[1].x, 0.25);
  EXPECT_DOUBLE_EQ(backend->frame[1].y, -0.5);
  EXPECT_FALSE(backend->frame[1].blank);
  EXPECT_EQ(backend->frame[1].color.r, color.r);
  EXPECT_EQ(backend->frame[1].color.g, color.g);
  EXPECT_EQ(backend->frame[1].color.b, color.b);
  EXPECT_TRUE(backend->frame.front().blank);
  EXPECT_TRUE(backend->frame.back().blank);
  EXPECT_EQ(backend->frame.front().color.r, 0);
  EXPECT_EQ(backend->frame.front().color.g, 0);
  EXPECT_EQ(backend->frame.front().color.b, 0);
}

TEST(Core, RawFrameRejectsOutOfRangeCoordinates) {
  auto backend = std::make_shared<TestBackend>();
  Projector projector(std::make_shared<PlaneSurface>(), backend);
  const std::vector<LaserPoint> points{
      {0.0, 0.0, {0, 0, 0}, true},
      {1.01, 0.0, {255, 255, 255}, false},
      {0.0, 0.0, {0, 0, 0}, true}};

  EXPECT_FALSE(projector.show_raw_frame(points, {255, 255, 255}));
  EXPECT_EQ(backend->blackout_count, 1);
  EXPECT_TRUE(backend->frame.empty());
}

TEST(Core, FourCornerCalibrationMapsMeasuredCornersToMirrorField) {
  Calibration2D calibration;
  calibration.fit({{{0.2, 0.3}, {-1.0, -1.0}},
                   {{2.4, 0.1}, {1.0, -1.0}},
                   {{2.1, 1.8}, {1.0, 1.0}},
                   {{-0.1, 1.7}, {-1.0, 1.0}}});
  const std::vector<Point2D> expected{{-1.0, -1.0}, {1.0, -1.0},
                                      {1.0, 1.0}, {-1.0, 1.0}};
  const std::vector<Point2D> measured{{0.2, 0.3}, {2.4, 0.1},
                                      {2.1, 1.8}, {-0.1, 1.7}};
  for (std::size_t i = 0; i < measured.size(); ++i) {
    const auto mapped = calibration.map(measured[i]);
    ASSERT_TRUE(mapped);
    EXPECT_NEAR(mapped->x, expected[i].x, 1e-9);
    EXPECT_NEAR(mapped->y, expected[i].y, 1e-9);
  }
}

TEST(Core, FourCornerCalibrationSupportsInsetMirrorRange) {
  Calibration2D calibration;
  calibration.fit({{{0, 0}, {-0.7, -0.6}}, {{2, 0}, {0.7, -0.6}},
                   {{2, 1}, {0.7, 0.6}}, {{0, 1}, {-0.7, 0.6}}});
  const auto mapped = calibration.map({1, 0.5});
  ASSERT_TRUE(mapped);
  EXPECT_NEAR(mapped->x, 0.0, 1e-9);
  EXPECT_NEAR(mapped->y, 0.0, 1e-9);
  EXPECT_TRUE(calibration.contains({0.7, 0.6}));
  EXPECT_FALSE(calibration.contains({0.8, 0.0}));

  auto backend = std::make_shared<TestBackend>();
  Projector projector(std::make_shared<PlaneSurface>(), backend, {}, calibration);
  EXPECT_FALSE(projector.show_path({{0, 0.5, 0}, {2.1, 0.5, 0}}, {}));
  EXPECT_TRUE(backend->frame.empty());
}

TEST(Core, ProjectorUsesFourCornerCalibrationWithoutProjectorPose) {
  auto backend = std::make_shared<TestBackend>();
  Calibration2D calibration;
  calibration.fit({{{0, 0}, {-1, -1}}, {{2, 0}, {1, -1}},
                   {{2, 1}, {1, 1}}, {{0, 1}, {-1, 1}}});
  Projector projector(std::make_shared<PlaneSurface>(), backend, {}, calibration);
  ASSERT_TRUE(projector.show_path({{0, 0, 0}, {2, 0, 0}}, {}));
  ASSERT_FALSE(backend->frame.empty());
  EXPECT_NEAR(backend->frame.front().x, -1.0, 1e-9);
  EXPECT_NEAR(backend->frame.front().y, -1.0, 1e-9);
  EXPECT_NEAR(backend->frame[1].x, -1.0, 1e-9);
  EXPECT_NEAR(backend->frame.back().x, -1.0, 1e-9);
}

TEST(Core, FourCornerCalibrationRejectsPathOutsideMirrorField) {
  auto backend = std::make_shared<TestBackend>();
  Calibration2D calibration;
  calibration.fit({{{0, 0}, {-1, -1}}, {{2, 0}, {1, -1}},
                   {{2, 1}, {1, 1}}, {{0, 1}, {-1, 1}}});
  Projector projector(std::make_shared<PlaneSurface>(), backend, {}, calibration);
  EXPECT_FALSE(projector.show_path({{0, 0.5, 0}, {2.1, 0.5, 0}}, {}));
  EXPECT_TRUE(backend->frame.empty());
}

TEST(Core, FourCornerCalibrationRejectsDegenerateCorners) {
  Calibration2D calibration;
  EXPECT_THROW(calibration.fit({{{0, 0}, {-1, -1}}, {{1, 0}, {1, -1}},
                                {{2, 0}, {1, 1}}, {{3, 0}, {-1, 1}}}),
               std::invalid_argument);
}

TEST(Helios, RawCoordinatesUseHighResolutionFieldWithoutWrapping) {
  HeliosBackend backend(FAKE_HELIOS_LIBRARY);
  ASSERT_TRUE(backend.initialize());
  const std::vector<LaserPoint> points{
      {-1.0, -1.0, {}, true},
      {0.0, 0.0, {255, 128, 64}, false},
      {0.0, 0.5, {255, 128, 64}, false},
      {0.0, -0.3, {255, 128, 64}, false},
      {0.3, 0.2, {255, 128, 64}, false},
      {0.4, 0.2, {255, 128, 64}, false},
      {0.49, 0.2, {255, 128, 64}, false},
      {0.0, 0.0, {99, 99, 99}, false},
      {0.0, 0.0, {100, 100, 100}, false},
      {1.0, 1.0, {}, true}};

  ASSERT_TRUE(backend.send_frame(points, 15000.0));
  ASSERT_EQ(FakeHeliosFrameSize(), static_cast<int>(points.size()));
  EXPECT_EQ(FakeHeliosFrameValue(0, 0), 0);
  EXPECT_EQ(FakeHeliosFrameValue(0, 1), 0);
  EXPECT_EQ(FakeHeliosFrameValue(1, 0), 32768);
  EXPECT_EQ(FakeHeliosFrameValue(1, 1), 32768);
  EXPECT_EQ(FakeHeliosFrameValue(2, 1), 49151);
  EXPECT_EQ(FakeHeliosFrameValue(3, 1), 22937);
  EXPECT_LT(FakeHeliosFrameValue(3, 1), FakeHeliosFrameValue(2, 1));
  EXPECT_NE(FakeHeliosFrameValue(3, 1), FakeHeliosFrameValue(4, 1));
  EXPECT_LT(FakeHeliosFrameValue(4, 0), FakeHeliosFrameValue(5, 0));
  EXPECT_LT(FakeHeliosFrameValue(5, 0), FakeHeliosFrameValue(6, 0));
  EXPECT_EQ(FakeHeliosFrameValue(1, 2), 65535);
  EXPECT_EQ(FakeHeliosFrameValue(1, 3), 32896);
  EXPECT_EQ(FakeHeliosFrameValue(1, 4), 16448);
  EXPECT_EQ(FakeHeliosFrameValue(7, 2), 25443);
  EXPECT_EQ(FakeHeliosFrameValue(8, 2), 25700);
  EXPECT_LT(FakeHeliosFrameValue(7, 2), FakeHeliosFrameValue(8, 2));
  EXPECT_EQ(FakeHeliosFrameValue(9, 0), 65535);
  EXPECT_EQ(FakeHeliosFrameValue(9, 1), 65535);
  EXPECT_EQ(FakeHeliosFrameValue(9, 2), 0);
}
