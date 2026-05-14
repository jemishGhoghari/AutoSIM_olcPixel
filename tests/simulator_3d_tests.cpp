#include <cassert>
#include <cmath>
#include <vector>

#include "simulator_3d.h"
#include "utils.h"

int main() {
    using namespace autonomous_driving;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float width = 0.0f;
    float depth = 0.0f;
    float height = 0.0f;
    bool ok = extractBox<float>("10x20x30x40x50x60", x, y, z, width, depth, height);
    assert(ok);
    assert(std::fabs(x - 10.0f) < 1e-6f);
    assert(std::fabs(y - 20.0f) < 1e-6f);
    assert(std::fabs(z - 30.0f) < 1e-6f);
    assert(std::fabs(width - 40.0f) < 1e-6f);
    assert(std::fabs(depth - 50.0f) < 1e-6f);
    assert(std::fabs(height - 60.0f) < 1e-6f);

    ok = extractBox<float>("10x20x30x40x50", x, y, z, width, depth, height);
    assert(!ok);

    const BoxObstacle3D box{{0.0f, 0.0f, 20.0f}, {10.0f, 10.0f, 10.0f}};
    float hitDistance = 0.0f;
    const bool hit = rayIntersectsBox({0.0f, 0.0f, 0.0f}, directionFromAngles(0.0f, 0.0f), box, 100.0f, hitDistance);
    assert(hit);
    assert(std::fabs(hitDistance - 15.0f) < 1e-5f);

    const std::vector<BoxObstacle3D> boxes{box, {{0.0f, 0.0f, 50.0f}, {10.0f, 10.0f, 10.0f}}};
    const float nearest = nearestBoxDistance({0.0f, 0.0f, 0.0f}, directionFromAngles(0.0f, 0.0f), boxes, 100.0f);
    assert(std::fabs(nearest - 15.0f) < 1e-5f);

    RangeSensorConfig config;
    config.rays = 3;
    config.fieldOfViewDegrees = 0.0f;
    config.maxRange = 100.0f;
    config.noiseStdDev = 0.0f;
    config.dropoutRate = 0.0f;
    auto readings = simulateRangeScan3D({0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, boxes, config);
    assert(readings.size() == 3);
    for (const auto& reading : readings) {
        assert(reading.hit);
        assert(!reading.droppedOut);
        assert(std::fabs(reading.idealRange - 15.0f) < 1e-5f);
        assert(std::fabs(reading.measuredRange - 15.0f) < 1e-5f);
    }

    Camera3D camera;
    camera.position = {0.0f, 0.0f, 0.0f};
    camera.pitchRadians = 0.0f;
    camera.yawRadians = 0.0f;
    camera.focalLength = 100.0f;
    const ProjectedPoint projected = projectPoint({0.0f, 0.0f, 10.0f}, camera, 800.0f, 600.0f);
    assert(projected.visible);
    assert(std::fabs(projected.screen.x - 400.0f) < 1e-5f);
    assert(std::fabs(projected.screen.y - 300.0f) < 1e-5f);

    return 0;
}
