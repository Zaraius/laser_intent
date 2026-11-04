#include <cstdint>
#include <vector>

namespace {
struct FakeHeliosPoint {
  std::uint16_t x, y, r, g, b;
};

std::vector<FakeHeliosPoint> last_frame;
}

extern "C" int OpenDevices() { return 1; }
extern "C" int GetStatus(int) { return 1; }
extern "C" void CloseDevices() {}

extern "C" int WriteFrameHighResolution(unsigned int, int, std::uint8_t,
                                        FakeHeliosPoint* points, int count) {
  last_frame.assign(points, points + count);
  return 1;
}

extern "C" int FakeHeliosFrameSize() {
  return static_cast<int>(last_frame.size());
}

extern "C" std::uint16_t FakeHeliosFrameValue(int index, int channel) {
  const auto& point = last_frame.at(static_cast<std::size_t>(index));
  switch (channel) {
    case 0: return point.x;
    case 1: return point.y;
    case 2: return point.r;
    case 3: return point.g;
    case 4: return point.b;
    default: return 0;
  }
}
