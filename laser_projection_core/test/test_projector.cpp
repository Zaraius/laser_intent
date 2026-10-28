#include "laser_projection_core/projector.hpp"
#include <algorithm>
#include <gtest/gtest.h>

using namespace laser_projection;

class TestBackend final : public LaserBackend {
public:
  bool initialize() override { return true; }
  bool send_frame(const std::vector<LaserPoint>& p, double) override { frame = p; return true; }
  void blackout() override {}
  std::vector<LaserPoint> frame;
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

  // No other points are blank: the shaft and both arms are illuminated.
  EXPECT_EQ(std::count_if(backend->frame.begin(), backend->frame.end(),
                          [](const LaserPoint& point) { return point.blank; }),
            5);
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
