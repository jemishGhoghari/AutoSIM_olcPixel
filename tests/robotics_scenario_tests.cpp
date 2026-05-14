#include <cassert>
#include <cmath>
#include <vector>

#include "robotics_scenario.h"

int main() {
    using namespace autonomous_driving;

    const RectangleObstacle obstacle{10.0f, -5.0f, 10.0f, 10.0f};
    float hitDistance = 0.0f;
    bool hit = rayIntersectsObstacle({0.0f, 0.0f}, 0.0f, obstacle, 100.0f, hitDistance);
    assert(hit);
    assert(std::fabs(hitDistance - 10.0f) < 1e-5f);

    const std::vector<RectangleObstacle> obstacles{obstacle, {30.0f, -5.0f, 10.0f, 10.0f}};
    const float nearest = nearestObstacleDistance({0.0f, 0.0f}, 0.0f, obstacles, 100.0f);
    assert(std::fabs(nearest - 10.0f) < 1e-5f);

    RangeSensorConfig config;
    config.rays = 3;
    config.fieldOfViewDegrees = 0.0f;
    config.maxRange = 100.0f;
    config.noiseStdDev = 0.0f;
    config.dropoutRate = 0.0f;
    auto readings = simulateRangeScan({0.0f, 0.0f}, 0.0f, obstacles, config);
    assert(readings.size() == 3);
    for (const auto& reading : readings) {
        assert(reading.hit);
        assert(!reading.droppedOut);
        assert(std::fabs(reading.idealRange - 10.0f) < 1e-5f);
        assert(std::fabs(reading.measuredRange - 10.0f) < 1e-5f);
    }

    config.dropoutRate = 1.0f;
    readings = simulateRangeScan({0.0f, 0.0f}, 0.0f, obstacles, config);
    for (const auto& reading : readings) {
        assert(reading.droppedOut);
        assert(std::fabs(reading.measuredRange - config.maxRange) < 1e-5f);
    }

    return 0;
}
