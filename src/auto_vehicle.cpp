#define OLC_PGE_APPLICATION
#include "auto_vehicle.h"
#include "utils.h"
#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

////////////////////////////////////////////////////////:- Autonomous Vehicle Class -:////////////////////////////////////////////////////////
autonomous_driving::AutonomousVehicle::AutonomousVehicle(const autonomous_driving::InitialConfig& gameConfig) {
    sAppName = "Autonomous Driving";
    this->gameConfig = gameConfig;
}

autonomous_driving::AutonomousVehicle::~AutonomousVehicle() = default;

bool autonomous_driving::AutonomousVehicle::OnUserCreate() {
    // Initialize Position
    position = {gameConfig.xPosition, gameConfig.yPosition};
    acceleration = 1.0f;
    angle = 0.0f;
    velocity = 0;
    rotational_velocity = 0.1;
    max_vel = 500;

    data_log.log(logger::LogLevel::INFO, "Car Loaded. Car started....");
    data_log.log(logger::LogLevel::INFO, "Robotics scenario loaded with " +
                 std::to_string(gameConfig.obstacles.size()) + " obstacle(s) and " +
                 std::to_string(std::max(1, gameConfig.rangeSensor.rays)) + " range rays.");
    
    // Car Image loading
    gpuCarRender = new olc::Decal(new olc::Sprite(this->gameConfig.CarPngPath));

    center_of_gravity = {gpuCarRender->sprite->width/2, gpuCarRender->sprite->height/2};

    return true;
}

bool autonomous_driving::AutonomousVehicle::OnUserUpdate(float fElapsedTime) {
    Clear(olc::WHITE);

    // Keyboard Events
    if (GetKey(olc::Key::UP).bHeld) {
        move_forward(fElapsedTime);
    } else if (GetKey(olc::Key::DOWN).bHeld) {
        move_backward(fElapsedTime);
    }

    if (GetKey(olc::Key::LEFT).bHeld) {
        rotate_car(true, false);
    } else if (GetKey(olc::Key::RIGHT).bHeld) {
        rotate_car(false, true);
    }

    if (!GetKey(olc::Key::UP).bHeld && !GetKey(olc::Key::DOWN).bHeld) {
        if (velocity < 0) {
            velocity += 0.5;
        }
        if (velocity > 0) {
            velocity -= 0.5;
        }
        move(fElapsedTime);
    }

    universal_boundaries(); // Set universal boundaries keep vehicle on the Screen

    last_scan = simulateRangeScan(
        get_sensor_origin(),
        angle - kPi / 2.0f,
        gameConfig.obstacles,
        gameConfig.rangeSensor
    );

    data_log.log(logger::LogLevel::INFO, "Velocity: " + std::to_string(velocity));
    
    if (gameConfig.showRoboticsOverlay) {
        draw_robotics_overlay();
    }

    DrawRotatedDecal(position, gpuCarRender, angle, center_of_gravity);
    return true;
}

void autonomous_driving::AutonomousVehicle::move(float fElapsedTime) {
    float_t verticle = std::cos(angle) * velocity;
    float_t horizontal = std::sin(angle) * velocity;
    
    position.y -= verticle * fElapsedTime;
    position.x -= horizontal * fElapsedTime;
}

void autonomous_driving::AutonomousVehicle::move_forward(float fElapsedTime) {
    velocity = std::min(velocity + acceleration, max_vel);
    move(fElapsedTime);
}

void autonomous_driving::AutonomousVehicle::move_backward(float fElapsedTime) {
    velocity = std::max(velocity - acceleration, -max_vel/2);
    move(fElapsedTime);
}

void autonomous_driving::AutonomousVehicle::rotate_car(bool left, bool right) {
    if (left) angle += rotational_velocity;
    if (right) angle -= rotational_velocity; 
}

autonomous_driving::Point2D autonomous_driving::AutonomousVehicle::get_sensor_origin() const {
    return {position.x, position.y};
}

void autonomous_driving::AutonomousVehicle::draw_robotics_overlay() {
    for (const auto& obstacle : gameConfig.obstacles) {
        FillRect(
            static_cast<int32_t>(obstacle.x),
            static_cast<int32_t>(obstacle.y),
            static_cast<int32_t>(obstacle.width),
            static_cast<int32_t>(obstacle.height),
            olc::Pixel(70, 70, 70)
        );
        DrawRect(
            static_cast<int32_t>(obstacle.x),
            static_cast<int32_t>(obstacle.y),
            static_cast<int32_t>(obstacle.width),
            static_cast<int32_t>(obstacle.height),
            olc::RED
        );
    }

    const Point2D sensorOrigin = get_sensor_origin();
    int hits = 0;
    int dropouts = 0;
    for (const auto& reading : last_scan) {
        if (reading.hit) {
            ++hits;
        }
        if (reading.droppedOut) {
            ++dropouts;
        }

        const float range = reading.measuredRange;
        const int endX = static_cast<int>(sensorOrigin.x + std::cos(reading.rayAngleRadians) * range);
        const int endY = static_cast<int>(sensorOrigin.y + std::sin(reading.rayAngleRadians) * range);
        const olc::Pixel color = reading.droppedOut ? olc::DARK_GREY : (reading.hit ? olc::RED : olc::BLUE);
        DrawLine(static_cast<int>(sensorOrigin.x), static_cast<int>(sensorOrigin.y), endX, endY, color);
        FillCircle(endX, endY, reading.hit && !reading.droppedOut ? 3 : 1, color);
    }

    std::ostringstream hud;
    hud << "Range sensor: " << last_scan.size() << " rays | hits " << hits
        << " | dropouts " << dropouts << " | noise σ " << std::fixed << std::setprecision(1)
        << gameConfig.rangeSensor.noiseStdDev << " px";
    DrawString(10, 10, hud.str(), olc::BLACK, 2);
}

void autonomous_driving::AutonomousVehicle::universal_boundaries() {
    if (static_cast<int>(position.x) >= ScreenWidth()) {
        position.x = 0;
    }

    if (static_cast<int>(position.x) < 0) {
        position.x = ScreenWidth();
    }

    if (static_cast<int>(position.y >= ScreenHeight())) {
        position.y = 0;
    }

    if (static_cast<int>(position.y < 0)) {
        position.y = ScreenHeight();
    }
}

////////////////////////////////////////////////////////////:- Global functions -:////////////////////////////////////////////////////////////
void autonomous_driving::getYAMLData(const std::string& yaml_path, InitialConfig& gameConfig){
    YAML::Node config = YAML::LoadFile(yaml_path);
    gameConfig.CarPngPath = config["car_png"].as<std::string>();

    if (!extractWidthHeight<int32_t>(config["window_size"].as<std::string>(), gameConfig.width, gameConfig.height)){
        std::cerr << "Cannot Extract Width and Height" << std::endl;
    }

    if (!extractWidthHeight<float_t>(config["initial_position"].as<std::string>(), gameConfig.xPosition, gameConfig.yPosition)){
        std::cerr << "Cannot Extract Position" << std::endl;
    }

    if (config["robotics_overlay"]) {
        gameConfig.showRoboticsOverlay = config["robotics_overlay"].as<bool>();
    }

    if (config["obstacles"]) {
        for (const auto& obstacleNode : config["obstacles"]) {
            RectangleObstacle obstacle;
            if (extractRectangle<float>(
                    obstacleNode.as<std::string>(),
                    obstacle.x,
                    obstacle.y,
                    obstacle.width,
                    obstacle.height)) {
                gameConfig.obstacles.push_back(obstacle);
            } else {
                std::cerr << "Cannot Extract Obstacle: " << obstacleNode.as<std::string>() << std::endl;
            }
        }
    }

    if (config["range_sensor"]) {
        const YAML::Node sensor = config["range_sensor"];
        if (sensor["rays"]) {
            gameConfig.rangeSensor.rays = sensor["rays"].as<int>();
        }
        if (sensor["fov_degrees"]) {
            gameConfig.rangeSensor.fieldOfViewDegrees = sensor["fov_degrees"].as<float>();
        }
        if (sensor["max_range"]) {
            gameConfig.rangeSensor.maxRange = sensor["max_range"].as<float>();
        }
        if (sensor["noise_stddev"]) {
            gameConfig.rangeSensor.noiseStdDev = sensor["noise_stddev"].as<float>();
        }
        if (sensor["dropout_rate"]) {
            gameConfig.rangeSensor.dropoutRate = sensor["dropout_rate"].as<float>();
        }
        if (sensor["random_seed"]) {
            gameConfig.rangeSensor.randomSeed = sensor["random_seed"].as<std::uint32_t>();
        }
    }
    
    std::cout << "[INFO]: PNG Loaded from " << gameConfig.CarPngPath << std::endl;
    std::printf("[INFO]: Window Size is %dx%d \n", gameConfig.width, gameConfig.height);
    std::printf("[INFO]: Initial position is %fx%f \n", gameConfig.xPosition, gameConfig.yPosition);
    std::printf("[INFO]: Robotics scenario has %zu obstacle(s), %d rays, %.1f px max range, %.2f dropout rate \n",
                gameConfig.obstacles.size(),
                std::max(1, gameConfig.rangeSensor.rays),
                gameConfig.rangeSensor.maxRange,
                clampFloat(gameConfig.rangeSensor.dropoutRate, 0.0f, 1.0f));
}



////////////////////////////////////////////////////////////:- Main Method -:////////////////////////////////////////////////////////////
int main(int argc, char* argv[]){
    std::string path;
    if (argc>1){
        path = argv[1];
    } else {
        path = "../configs/initializeGame.yaml";
    }
    
    // Initial Game Config
    autonomous_driving::InitialConfig gameConfig;

    // Load Game configurations
    autonomous_driving::getYAMLData(path, gameConfig);

    auto new_game = std::make_unique<autonomous_driving::AutonomousVehicle>(gameConfig);
    if (new_game->Construct(gameConfig.width, gameConfig.height, 1, 1, false, true)){
        new_game->Start();
    }
    return 0;
}