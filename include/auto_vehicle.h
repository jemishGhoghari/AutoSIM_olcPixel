#ifndef AUTO_VEHICLE_H
#define AUTO_VEHICLE_H

#include "olcPixelGameEngine.h"

namespace autonomous_driving
{

// Global Methods and Structures
struct InitialConfig{
    std::string CarPngPath;
    int32_t width, height;
};

void getYAMLData(const std::string& yaml_path, InitialConfig& gameConfig);
bool extractWidthHeight(const std::string& window_size, int32_t& width, int32_t& height);

class AutonomousVehicle : public olc::PixelGameEngine
{
private:
    // Car Position
    olc::vf2d position;
    // Acceleration
    float acceleration;
    // Heading Angle
    float angle;
    // Max Velocity
    float max_vel;
    // Rotational Velocity
    float rotional_velocity;

    // Game Configuartions 
    autonomous_driving::InitialConfig gameConfig;

    // Sprite for Loading PNG Images
    olc::Sprite* NewCar;

    // Decal: GPU Accelerated Sprite
    olc::Decal* gpuCarRender;

public:
    AutonomousVehicle(const autonomous_driving::InitialConfig& gameConfig);
    ~AutonomousVehicle();

    bool OnUserCreate() override;
    bool OnUserUpdate(float fElapsedTime) override;
};
} // namespace autonomous_driving

#endif