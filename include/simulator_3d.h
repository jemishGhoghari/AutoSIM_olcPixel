#ifndef SIMULATOR_3D_H
#define SIMULATOR_3D_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "robotics_scenario.h"

namespace autonomous_driving
{

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Camera3D {
    Vec3 position{0.0f, 120.0f, -420.0f};
    float yawRadians = 0.0f;
    float pitchRadians = degreesToRadians(-12.0f);
    float focalLength = 520.0f;
    float nearPlane = 1.0f;
};

struct ThirdPersonCameraConfig {
    float distance = 360.0f;
    float height = 145.0f;
    float orbitRadians = 0.0f;
    float minDistance = 140.0f;
    float maxDistance = 900.0f;
    float minHeight = 45.0f;
    float maxHeight = 420.0f;
};

struct BoxObstacle3D {
    Vec3 center{0.0f, 0.0f, 0.0f};
    Vec3 size{1.0f, 1.0f, 1.0f};
};

struct VehicleModelPart3D {
    Vec3 localCenter{0.0f, 0.0f, 0.0f};
    Vec3 size{1.0f, 1.0f, 1.0f};
};

struct ProjectedPoint {
    Vec2 screen;
    bool visible = false;
    float depth = 0.0f;
};

struct RangeReading3D {
    float yawRadians = 0.0f;
    float pitchRadians = 0.0f;
    float idealRange = 0.0f;
    float measuredRange = 0.0f;
    bool hit = false;
    bool droppedOut = false;
};

inline Vec3 add(Vec3 left, Vec3 right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

inline Vec3 subtract(Vec3 left, Vec3 right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

inline Vec3 multiply(Vec3 value, float scalar) {
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

inline float dot(Vec3 left, Vec3 right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

inline float length(Vec3 value) {
    return std::sqrt(dot(value, value));
}

inline Vec3 normalize(Vec3 value) {
    const float magnitude = length(value);
    if (magnitude <= 1e-6f) {
        return {0.0f, 0.0f, 1.0f};
    }

    return multiply(value, 1.0f / magnitude);
}

inline Vec3 rotateAroundY(Vec3 value, float yawRadians) {
    const float cosYaw = std::cos(yawRadians);
    const float sinYaw = std::sin(yawRadians);
    return {
        value.x * cosYaw + value.z * sinYaw,
        value.y,
        -value.x * sinYaw + value.z * cosYaw
    };
}

inline Vec3 directionFromAngles(float yawRadians, float pitchRadians) {
    const float cosPitch = std::cos(pitchRadians);
    return normalize({std::sin(yawRadians) * cosPitch, std::sin(pitchRadians), std::cos(yawRadians) * cosPitch});
}

inline Vec3 rotateWorldToCamera(Vec3 point, const Camera3D& camera) {
    Vec3 translated = subtract(point, camera.position);

    const float cosYaw = std::cos(-camera.yawRadians);
    const float sinYaw = std::sin(-camera.yawRadians);
    Vec3 yawed{
        translated.x * cosYaw - translated.z * sinYaw,
        translated.y,
        translated.x * sinYaw + translated.z * cosYaw
    };

    const float cosPitch = std::cos(-camera.pitchRadians);
    const float sinPitch = std::sin(-camera.pitchRadians);
    return {
        yawed.x,
        yawed.y * cosPitch - yawed.z * sinPitch,
        yawed.y * sinPitch + yawed.z * cosPitch
    };
}

inline Camera3D makeThirdPersonCamera(Vec3 target, float vehicleYawRadians, ThirdPersonCameraConfig config) {
    config.distance = clampFloat(config.distance, config.minDistance, config.maxDistance);
    config.height = clampFloat(config.height, config.minHeight, config.maxHeight);

    const float cameraYawAroundTarget = vehicleYawRadians + config.orbitRadians;
    const Vec3 forward = directionFromAngles(cameraYawAroundTarget, 0.0f);
    const Vec3 position = add(subtract(target, multiply(forward, config.distance)), {0.0f, config.height, 0.0f});
    const Vec3 toTarget = subtract(target, position);
    const float horizontal = std::sqrt(toTarget.x * toTarget.x + toTarget.z * toTarget.z);

    Camera3D camera;
    camera.position = position;
    camera.yawRadians = std::atan2(toTarget.x, toTarget.z);
    camera.pitchRadians = std::atan2(toTarget.y, horizontal);
    return camera;
}

inline ProjectedPoint projectPoint(Vec3 point, const Camera3D& camera, float screenWidth, float screenHeight) {
    const Vec3 cameraSpace = rotateWorldToCamera(point, camera);
    if (cameraSpace.z <= camera.nearPlane) {
        return {{0.0f, 0.0f}, false, cameraSpace.z};
    }

    return {{
        screenWidth * 0.5f + (cameraSpace.x * camera.focalLength) / cameraSpace.z,
        screenHeight * 0.5f - (cameraSpace.y * camera.focalLength) / cameraSpace.z
    }, true, cameraSpace.z};
}

inline std::array<Vec3, 8> orientedBoxCorners(const VehicleModelPart3D& part, Vec3 worldOrigin, float yawRadians) {
    const Vec3 half = multiply(part.size, 0.5f);
    const std::array<Vec3, 8> localCorners{{
        {part.localCenter.x - half.x, part.localCenter.y - half.y, part.localCenter.z - half.z},
        {part.localCenter.x + half.x, part.localCenter.y - half.y, part.localCenter.z - half.z},
        {part.localCenter.x + half.x, part.localCenter.y + half.y, part.localCenter.z - half.z},
        {part.localCenter.x - half.x, part.localCenter.y + half.y, part.localCenter.z - half.z},
        {part.localCenter.x - half.x, part.localCenter.y - half.y, part.localCenter.z + half.z},
        {part.localCenter.x + half.x, part.localCenter.y - half.y, part.localCenter.z + half.z},
        {part.localCenter.x + half.x, part.localCenter.y + half.y, part.localCenter.z + half.z},
        {part.localCenter.x - half.x, part.localCenter.y + half.y, part.localCenter.z + half.z}
    }};

    std::array<Vec3, 8> corners{};
    for (std::size_t i = 0; i < localCorners.size(); ++i) {
        corners[i] = add(worldOrigin, rotateAroundY(localCorners[i], yawRadians));
    }
    return corners;
}

inline std::array<Vec3, 8> boxCorners(const BoxObstacle3D& box) {
    const Vec3 half = multiply(box.size, 0.5f);
    return {{
        {box.center.x - half.x, box.center.y - half.y, box.center.z - half.z},
        {box.center.x + half.x, box.center.y - half.y, box.center.z - half.z},
        {box.center.x + half.x, box.center.y + half.y, box.center.z - half.z},
        {box.center.x - half.x, box.center.y + half.y, box.center.z - half.z},
        {box.center.x - half.x, box.center.y - half.y, box.center.z + half.z},
        {box.center.x + half.x, box.center.y - half.y, box.center.z + half.z},
        {box.center.x + half.x, box.center.y + half.y, box.center.z + half.z},
        {box.center.x - half.x, box.center.y + half.y, box.center.z + half.z}
    }};
}

inline std::vector<VehicleModelPart3D> defaultVehicleModel3D() {
    return {
        {{0.0f, 18.0f, 0.0f}, {70.0f, 26.0f, 120.0f}},
        {{0.0f, 42.0f, -8.0f}, {46.0f, 28.0f, 58.0f}},
        {{-42.0f, 12.0f, -38.0f}, {16.0f, 24.0f, 24.0f}},
        {{42.0f, 12.0f, -38.0f}, {16.0f, 24.0f, 24.0f}},
        {{-42.0f, 12.0f, 38.0f}, {16.0f, 24.0f, 24.0f}},
        {{42.0f, 12.0f, 38.0f}, {16.0f, 24.0f, 24.0f}},
        {{0.0f, 28.0f, 72.0f}, {30.0f, 16.0f, 18.0f}}
    };
}

inline const std::array<std::array<int, 2>, 12>& boxEdges() {
    static const std::array<std::array<int, 2>, 12> edges{{
        {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
        {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
        {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}
    }};
    return edges;
}

inline bool rayIntersectsBox(Vec3 origin, Vec3 direction, const BoxObstacle3D& box, float maxRange, float& hitDistance) {
    const Vec3 half = multiply(box.size, 0.5f);
    const Vec3 minimum = subtract(box.center, half);
    const Vec3 maximum = add(box.center, half);
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

    if (!updateInterval(origin.x, direction.x, minimum.x, maximum.x, tMin, tMax) ||
        !updateInterval(origin.y, direction.y, minimum.y, maximum.y, tMin, tMax) ||
        !updateInterval(origin.z, direction.z, minimum.z, maximum.z, tMin, tMax) ||
        tMax < 0.0f || tMin > maxRange) {
        return false;
    }

    hitDistance = clampFloat(tMin < 0.0f ? tMax : tMin, 0.0f, maxRange);
    return true;
}

inline float nearestBoxDistance(Vec3 origin, Vec3 direction, const std::vector<BoxObstacle3D>& boxes, float maxRange) {
    float nearest = maxRange;
    for (const auto& box : boxes) {
        float candidate = maxRange;
        if (rayIntersectsBox(origin, direction, box, maxRange, candidate)) {
            nearest = std::min(nearest, candidate);
        }
    }
    return nearest;
}

inline std::vector<RangeReading3D> simulateRangeScan3D(
    Vec3 origin,
    float yawRadians,
    float pitchRadians,
    const std::vector<BoxObstacle3D>& boxes,
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

    std::vector<RangeReading3D> readings;
    readings.reserve(static_cast<std::size_t>(rayCount));

    for (int i = 0; i < rayCount; ++i) {
        const float rayYaw = yawRadians - halfFov + step * static_cast<float>(i);
        const Vec3 direction = directionFromAngles(rayYaw, pitchRadians);
        const float idealRange = nearestBoxDistance(origin, direction, boxes, maxRange);
        const bool hit = idealRange < maxRange;
        const bool droppedOut = dropoutRate > 0.0f && dropout(rng) < dropoutRate;
        float measuredRange = maxRange;

        if (!droppedOut) {
            measuredRange = clampFloat(idealRange + noise(rng), 0.0f, maxRange);
        }

        readings.push_back({rayYaw, pitchRadians, idealRange, measuredRange, hit, droppedOut});
    }

    return readings;
}

} // namespace autonomous_driving

#endif // SIMULATOR_3D_H
