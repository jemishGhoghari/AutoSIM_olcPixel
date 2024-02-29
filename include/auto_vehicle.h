#ifndef AUTO_VEHICLE_H
#define AUTO_VEHICLE_H

#include <cmath>
#include "olcPixelGameEngine.h"
#include "logger.h"
#include "vehicle.h"

namespace autonomous_driving
{

// Global Methods and Structures
struct InitialConfig{
    std::string CarPngPath;
    int32_t width, height;
    float_t xPosition, yPosition;
};

void getYAMLData(const std::string& yaml_path, InitialConfig& gameConfig);

template <typename T>
bool extractWidthHeight(const std::string& window_size, T& width, T& height);

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
    // Vehicle Velocity
    float velocity;
    // Rotational Velocity
    float rotational_velocity;
    // Vehicle COG
    olc::vi2d center_of_gravity;

    // Game Configuartions 
    autonomous_driving::InitialConfig gameConfig;

    // Sprite for Loading PNG Images
    olc::Sprite* NewCar;

    // Decal: GPU Accelerated Sprite
    olc::Decal* gpuCarRender;

    // Logger
    logger::Logger data_log;

public:
    AutonomousVehicle(const autonomous_driving::InitialConfig& gameConfig);
    ~AutonomousVehicle();

    bool OnUserCreate() override;
    bool OnUserUpdate(float fElapsedTime) override;
    void move(float fElapsedTime);
    void move_forward(float fElapsedTime);
    void move_backward(float fElapsedTime);
    void rotate_car(bool left, bool right);
    void universal_boundaries();
};
} // namespace autonomous_driving

#endif