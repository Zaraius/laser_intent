#include "laser_projection_core/projector.hpp"
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
  EXPECT_GT(backend->frame.size(), 5u);
  EXPECT_TRUE(backend->frame[backend->frame.size() - 4].blank);
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
