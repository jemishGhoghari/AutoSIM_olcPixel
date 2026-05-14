#ifndef ROBOTICS_SCENARIO_H
#define ROBOTICS_SCENARIO_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace autonomous_driving
{

struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
};

struct RectangleObstacle {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
};

struct RangeSensorConfig {
    int rays = 31;
    float fieldOfViewDegrees = 180.0f;
    float maxRange = 350.0f;
    float noiseStdDev = 0.0f;
    float dropoutRate = 0.0f;
    std::uint32_t randomSeed = 7u;
};

struct RangeReading {
    float rayAngleRadians = 0.0f;
    float idealRange = 0.0f;
    float measuredRange = 0.0f;
    bool hit = false;
    bool droppedOut = false;
};

constexpr float kPi = 3.14159265358979323846f;

inline float degreesToRadians(float degrees) {
    return degrees * kPi / 180.0f;
}

inline float clampFloat(float value, float low, float high) {
    return std::max(low, std::min(value, high));
}

inline bool containsPoint(const RectangleObstacle& obstacle, Point2D point) {
    return point.x >= obstacle.x && point.x <= obstacle.x + obstacle.width &&
           point.y >= obstacle.y && point.y <= obstacle.y + obstacle.height;
}

inline bool rayIntersectsObstacle(
    Point2D origin,
    float angleRadians,
    const RectangleObstacle& obstacle,
    float maxRange,
    float& hitDistance
) {
    const float dx = std::cos(angleRadians);
    const float dy = std::sin(angleRadians);
    float tMin = 0.0f;
    float tMax = maxRange;

    const auto updateInterval = [&](float rayOrigin, float rayDirection, float minBound, float maxBound,
                                    float& currentMin, float& currentMax) -> bool {
        constexpr float kEpsilon = 1e-6f;
        if (std::fabs(rayDirection) < kEpsilon) {
            return rayOrigin >= minBound && rayOrigin <= maxBound;
        }

        float t1 = (minBound - rayOrigin) / rayDirection;
        float t2 = (maxBound - rayOrigin) / rayDirection;
        if (t1 > t2) {
            std::swap(t1, t2);
        }

        currentMin = std::max(currentMin, t1);
        currentMax = std::min(currentMax, t2);
        return currentMin <= currentMax;
    };

    const bool intersectsX = updateInterval(origin.x, dx, obstacle.x, obstacle.x + obstacle.width, tMin, tMax);
    const bool intersectsY = updateInterval(origin.y, dy, obstacle.y, obstacle.y + obstacle.height, tMin, tMax);

    if (!intersectsX || !intersectsY || tMax < 0.0f || tMin > maxRange) {
        return false;
    }

    hitDistance = clampFloat(tMin < 0.0f ? tMax : tMin, 0.0f, maxRange);
    return true;
}

inline float nearestObstacleDistance(
    Point2D origin,
    float angleRadians,
    const std::vector<RectangleObstacle>& obstacles,
    float maxRange
) {
    float nearest = maxRange;
    for (const auto& obstacle : obstacles) {
        float candidate = maxRange;
        if (rayIntersectsObstacle(origin, angleRadians, obstacle, maxRange, candidate)) {
            nearest = std::min(nearest, candidate);
        }
    }
    return nearest;
}

inline std::vector<RangeReading> simulateRangeScan(
    Point2D origin,
    float headingRadians,
    const std::vector<RectangleObstacle>& obstacles,
    const RangeSensorConfig& config
) {
    const int rayCount = std::max(1, config.rays);
    const float maxRange = std::max(0.0f, config.maxRange);
    const float halfFov = degreesToRadians(config.fieldOfViewDegrees) * 0.5f;
    const float step = rayCount == 1 ? 0.0f : (halfFov * 2.0f) / static_cast<float>(rayCount - 1);
    const float dropoutRate = clampFloat(config.dropoutRate, 0.0f, 1.0f);

    std::mt19937 rng(config.randomSeed);
    std::normal_distribution<float> noise(0.0f, std::max(0.0f, config.noiseStdDev));
    std::uniform_real_distribution<float> dropout(0.0f, 1.0f);

    std::vector<RangeReading> readings;
    readings.reserve(static_cast<std::size_t>(rayCount));

    for (int i = 0; i < rayCount; ++i) {
        const float rayAngle = headingRadians - halfFov + step * static_cast<float>(i);
        const float idealRange = nearestObstacleDistance(origin, rayAngle, obstacles, maxRange);
        const bool hit = idealRange < maxRange;
        const bool droppedOut = dropoutRate > 0.0f && dropout(rng) < dropoutRate;
        float measuredRange = maxRange;

        if (!droppedOut) {
            measuredRange = clampFloat(idealRange + noise(rng), 0.0f, maxRange);
        }

        readings.push_back({rayAngle, idealRange, measuredRange, hit, droppedOut});
    }

    return readings;
}

} // namespace autonomous_driving

#endif // ROBOTICS_SCENARIO_H
