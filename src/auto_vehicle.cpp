#define OLC_PGE_APPLICATION
#include "auto_vehicle.h"
#include "yaml-cpp/yaml.h"

////////////////////////////////////////////////////////:- Autonomous Vehicle Class -:////////////////////////////////////////////////////////
autonomous_driving::AutonomousVehicle::AutonomousVehicle(const autonomous_driving::InitialConfig& gameConfig) {
    sAppName = "Autonomous Driving";
    this->gameConfig = gameConfig;
}

autonomous_driving::AutonomousVehicle::~AutonomousVehicle() = default;

bool autonomous_driving::AutonomousVehicle::OnUserCreate() {
    // Initialize Position
    position = {gameConfig.xPosition, gameConfig.yPosition};
    acceleration = 0.1f;
    angle = 0.0f;
    velocity = 500;
    rotational_velocity = 0.1;

    std::cout << "[INFO]: Car Loaded. Car started...." << std::endl;
    
    // Car Image loading
    gpuCarRender = new olc::Decal(new olc::Sprite(this->gameConfig.CarPngPath));
    return true;
}

bool autonomous_driving::AutonomousVehicle::OnUserUpdate(float fElapsedTime) {
    Clear(olc::WHITE);

    // Keyboard Events
    if (GetKey(olc::Key::UP).bHeld) {
        move_forward(fElapsedTime);
    }
    if (GetKey(olc::Key::LEFT).bHeld) {
        rotate_car(true, false);
    }
    if (GetKey(olc::Key::RIGHT).bHeld) {
        rotate_car(false, true);
    }

    universal_boundaries(); // Set universal boundaries keep vehicle on the Screen

    DrawRotatedDecal(position, gpuCarRender, angle, {30.0, 64.0});
    return true;
}

void autonomous_driving::AutonomousVehicle::move(float fElapsedTime) {
    // float_t radians = M_PI * angle / 180.0;

    float_t verticle = std::cos(angle) * velocity;
    float_t horizontal = std::sin(angle) * velocity;
    
    position.y += verticle * fElapsedTime;
    position.x += horizontal * fElapsedTime;
}

void autonomous_driving::AutonomousVehicle::move_forward(float fElapsedTime) {
    move(fElapsedTime);
}

void autonomous_driving::AutonomousVehicle::move_backward(float fElapsedTime) {
    move(fElapsedTime);
}

void autonomous_driving::AutonomousVehicle::rotate_car(bool left, bool right) {
    if (left) angle += rotational_velocity;
    if (right) angle -= rotational_velocity; 
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
    
    std::cout << "[INFO]: PNG Loaded from " << gameConfig.CarPngPath << std::endl;
    std::printf("[INFO]: Window Size is %dx%d \n", gameConfig.width, gameConfig.height);
    std::printf("[INFO]: Initial position is %fx%f \n", gameConfig.xPosition, gameConfig.yPosition);
}

template <typename T>
bool autonomous_driving::extractWidthHeight(const std::string& window_size, T& width, T& height){
    size_t xPos = window_size.find('x');
    if (xPos != std::string::npos && xPos < window_size.length()){
        width = std::stoll(window_size.substr(0, xPos));
        height = std::stoll(window_size.substr(xPos + 1));
        return true;
    } else
    {
        return false;
    }   
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

    std::unique_ptr new_game = std::make_unique<autonomous_driving::AutonomousVehicle>(gameConfig);
    if (new_game->Construct(gameConfig.width, gameConfig.height, 1, 1, false, true)){
        new_game->Start();
    }
    return 0;
}