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
#include <string>

////////////////////////////////////////////////////////:- Autonomous Vehicle Class -:////////////////////////////////////////////////////////
autonomous_driving::AutonomousVehicle::AutonomousVehicle(const autonomous_driving::InitialConfig& gameConfig) {
    sAppName = "AutoSIM — Autonomous Driving";
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
    data_log.log(logger::LogLevel::INFO, "Scenario '" + gameConfig.scenarioName + "' loaded in " +
                 simulatorModeToString(gameConfig.simulatorMode) + " mode with " +
                 std::to_string(gameConfig.obstacles.size()) + " 2D obstacle(s), " +
                 std::to_string(gameConfig.obstacles3D.size()) + " 3D obstacle(s), and " +
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

    if (GetKey(olc::Key::TAB).bPressed) {
        gameConfig.showGuiOverlay = !gameConfig.showGuiOverlay;
    }

    if (GetKey(olc::Key::F1).bPressed) {
        gameConfig.showRoboticsOverlay = !gameConfig.showRoboticsOverlay;
    }

    if (GetKey(olc::Key::F2).bPressed) {
        gameConfig.simulatorMode = gameConfig.simulatorMode == SimulatorMode::Planar2D
            ? SimulatorMode::Preview3D
            : SimulatorMode::Planar2D;
    }

    if (gameConfig.simulatorMode == SimulatorMode::Preview3D) {
        last_scan_3d = simulateRangeScan3D(
            get_sensor_origin_3d(),
            kPi - angle,
            0.0f,
            gameConfig.obstacles3D,
            gameConfig.rangeSensor
        );
        draw_3d_preview();
    } else {
        last_scan = simulateRangeScan(
            get_sensor_origin(),
            angle - kPi / 2.0f,
            gameConfig.obstacles,
            gameConfig.rangeSensor
        );

        if (gameConfig.showRoboticsOverlay) {
            draw_robotics_overlay();
        }

        DrawRotatedDecal(position, gpuCarRender, angle, center_of_gravity);
    }

    data_log.log(logger::LogLevel::INFO, "Velocity: " + std::to_string(velocity));

    if (gameConfig.showGuiOverlay) {
        draw_gui_overlay();
    }
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

autonomous_driving::Vec3 autonomous_driving::AutonomousVehicle::get_sensor_origin_3d() const {
    return {position.x - ScreenWidth() * 0.5f, 12.0f, position.y};
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


void autonomous_driving::AutonomousVehicle::draw_3d_preview() {
    const auto project = [&](Vec3 point) {
        return projectPoint(point, gameConfig.camera3D, static_cast<float>(ScreenWidth()), static_cast<float>(ScreenHeight()));
    };

    const float gridExtent = 1200.0f;
    const float gridStep = 100.0f;
    for (float value = -gridExtent; value <= gridExtent; value += gridStep) {
        const ProjectedPoint a = project({-gridExtent, 0.0f, value});
        const ProjectedPoint b = project({gridExtent, 0.0f, value});
        const ProjectedPoint c = project({value, 0.0f, -100.0f});
        const ProjectedPoint d = project({value, 0.0f, gridExtent});
        if (a.visible && b.visible) {
            DrawLine(static_cast<int>(a.screen.x), static_cast<int>(a.screen.y), static_cast<int>(b.screen.x), static_cast<int>(b.screen.y), olc::VERY_DARK_GREY);
        }
        if (c.visible && d.visible) {
            DrawLine(static_cast<int>(c.screen.x), static_cast<int>(c.screen.y), static_cast<int>(d.screen.x), static_cast<int>(d.screen.y), olc::VERY_DARK_GREY);
        }
    }

    for (const auto& box : gameConfig.obstacles3D) {
        const auto corners = boxCorners(box);
        for (const auto& edge : boxEdges()) {
            const ProjectedPoint start = project(corners[static_cast<std::size_t>(edge[0])]);
            const ProjectedPoint end = project(corners[static_cast<std::size_t>(edge[1])]);
            if (start.visible && end.visible) {
                DrawLine(static_cast<int>(start.screen.x), static_cast<int>(start.screen.y),
                         static_cast<int>(end.screen.x), static_cast<int>(end.screen.y), olc::DARK_RED);
            }
        }
    }

    const Vec3 vehicleOrigin = get_sensor_origin_3d();
    const ProjectedPoint vehicle = project(vehicleOrigin);
    if (vehicle.visible) {
        FillCircle(static_cast<int>(vehicle.screen.x), static_cast<int>(vehicle.screen.y), 5, olc::GREEN);
    }

    if (gameConfig.showRoboticsOverlay) {
        for (const auto& reading : last_scan_3d) {
            const Vec3 direction = directionFromAngles(reading.yawRadians, reading.pitchRadians);
            const Vec3 endPoint = add(vehicleOrigin, multiply(direction, reading.measuredRange));
            const ProjectedPoint end = project(endPoint);
            if (vehicle.visible && end.visible) {
                const olc::Pixel color = reading.droppedOut ? olc::DARK_GREY : (reading.hit ? olc::RED : olc::BLUE);
                DrawLine(static_cast<int>(vehicle.screen.x), static_cast<int>(vehicle.screen.y),
                         static_cast<int>(end.screen.x), static_cast<int>(end.screen.y), color);
            }
        }
    }
}

void autonomous_driving::AutonomousVehicle::draw_gui_overlay() {
    int hits = 0;
    int dropouts = 0;
    if (gameConfig.simulatorMode == SimulatorMode::Preview3D) {
        for (const auto& reading : last_scan_3d) {
            if (reading.hit) { ++hits; }
            if (reading.droppedOut) { ++dropouts; }
        }
    } else {
        for (const auto& reading : last_scan) {
            if (reading.hit) { ++hits; }
            if (reading.droppedOut) { ++dropouts; }
        }
    }

    FillRect(8, ScreenHeight() - 86, 690, 76, olc::Pixel(245, 245, 245));
    DrawRect(8, ScreenHeight() - 86, 690, 76, olc::DARK_GREY);

    std::ostringstream hud;
    hud << "Scenario: " << gameConfig.scenarioName
        << " | Mode: " << simulatorModeToString(gameConfig.simulatorMode)
        << " | Sensor hits " << hits << " dropouts " << dropouts;
    DrawString(16, ScreenHeight() - 78, hud.str(), olc::BLACK, 1);
    DrawString(16, ScreenHeight() - 58, "Controls: Arrow keys drive | F1 sensor overlay | F2 2D/3D preview | TAB GUI", olc::BLACK, 1);
    DrawString(16, ScreenHeight() - 38, "3D preview is wireframe projection on olcPGE; use authored YAML boxes for scenarios.", olc::DARK_BLUE, 1);
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
autonomous_driving::SimulatorMode autonomous_driving::parseSimulatorMode(const std::string& text) {
    if (text == "3d" || text == "3D" || text == "preview_3d" || text == "3d_preview") {
        return SimulatorMode::Preview3D;
    }

    return SimulatorMode::Planar2D;
}

std::string autonomous_driving::simulatorModeToString(SimulatorMode mode) {
    switch (mode) {
        case SimulatorMode::Preview3D:
            return "3d_preview";
        case SimulatorMode::Planar2D:
        default:
            return "2d";
    }
}

void autonomous_driving::getYAMLData(const std::string& yaml_path, InitialConfig& gameConfig){
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (config["scenario_name"]) {
        gameConfig.scenarioName = config["scenario_name"].as<std::string>();
    }
    if (config["simulator_mode"]) {
        gameConfig.simulatorMode = parseSimulatorMode(config["simulator_mode"].as<std::string>());
    }
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

    if (config["gui_overlay"]) {
        gameConfig.showGuiOverlay = config["gui_overlay"].as<bool>();
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

    if (config["obstacles_3d"]) {
        for (const auto& obstacleNode : config["obstacles_3d"]) {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            float width = 0.0f;
            float depth = 0.0f;
            float height = 0.0f;
            if (extractBox<float>(obstacleNode.as<std::string>(), x, y, z, width, depth, height)) {
                gameConfig.obstacles3D.push_back({{x, y + height * 0.5f, z}, {width, height, depth}});
            } else {
                std::cerr << "Cannot Extract 3D Obstacle: " << obstacleNode.as<std::string>() << std::endl;
            }
        }
    }

    if (config["camera_3d"]) {
        const YAML::Node camera = config["camera_3d"];
        if (camera["position"]) {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            std::vector<float> values;
            if (extractDelimitedValues<float>(camera["position"].as<std::string>(), 'x', values) && values.size() == 3) {
                x = values[0]; y = values[1]; z = values[2];
                gameConfig.camera3D.position = {x, y, z};
            } else {
                std::cerr << "Cannot Extract 3D Camera Position: " << camera["position"].as<std::string>() << std::endl;
            }
        }
        if (camera["yaw_degrees"]) {
            gameConfig.camera3D.yawRadians = degreesToRadians(camera["yaw_degrees"].as<float>());
        }
        if (camera["pitch_degrees"]) {
            gameConfig.camera3D.pitchRadians = degreesToRadians(camera["pitch_degrees"].as<float>());
        }
        if (camera["focal_length"]) {
            gameConfig.camera3D.focalLength = camera["focal_length"].as<float>();
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
    std::printf("[INFO]: Scenario '%s' uses mode %s with %zu 2D obstacle(s), %zu 3D obstacle(s), %d rays, %.1f px max range, %.2f dropout rate \n",
                gameConfig.scenarioName.c_str(),
                simulatorModeToString(gameConfig.simulatorMode).c_str(),
                gameConfig.obstacles.size(),
                gameConfig.obstacles3D.size(),
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