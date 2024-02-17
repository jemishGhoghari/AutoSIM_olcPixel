#define OLC_PGE_APPLICATION
#include "auto_vehicle.h"
#include "yaml-cpp/yaml.h"

////////////////////////////////////////////////////////:- Autonomous Vehicle Class -:////////////////////////////////////////////////////////
autonomous_driving::AutonomousVehicle::AutonomousVehicle(const autonomous_driving::InitialConfig& gameConfig){
    sAppName = "Autonomous Driving";
    this->gameConfig = gameConfig;
}

autonomous_driving::AutonomousVehicle::~AutonomousVehicle() = default;

bool autonomous_driving::AutonomousVehicle::OnUserCreate(){
    // Initialize Position
    position = {200.0f, 200.0f};
    acceleration = 0.1f;

    std::cout << "[INFO]: Car Loaded. Car started...." << std::endl;
    
    // Car Image loading
    gpuCarRender = new olc::Decal(new olc::Sprite(this->gameConfig.CarPngPath));
    return true;
}

bool autonomous_driving::AutonomousVehicle::OnUserUpdate(float fElapsedTime){
    Clear(olc::WHITE);
    position.x += 1;
    DrawDecal(position, gpuCarRender);
    return true;
}

////////////////////////////////////////////////////////////:- Global functions -:////////////////////////////////////////////////////////////
void autonomous_driving::getYAMLData(const std::string& yaml_path, InitialConfig& gameConfig){
    YAML::Node config = YAML::LoadFile(yaml_path);
    gameConfig.CarPngPath = config["car_png"].as<std::string>();

    if (!extractWidthHeight(config["window_size"].as<std::string>(), gameConfig.width, gameConfig.height)){
        std::cerr << "Cannot Extract Width and Height" << std::endl;
    }
    
    std::cout << "[INFO]: PNG Loaded from " << gameConfig.CarPngPath << std::endl;
    std::printf("[INFO]: Window Size is %dx%d \n", gameConfig.width, gameConfig.height);
}

bool autonomous_driving::extractWidthHeight(const std::string& window_size, int32_t& width, int32_t& height){
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
        path = "../Configs/initializeGame.yaml";
    }
    
    // Initial Game Config
    autonomous_driving::InitialConfig gameConfig;

    // Load Game configurations
    autonomous_driving::getYAMLData(path, gameConfig);

    std::unique_ptr new_game = std::make_unique<autonomous_driving::AutonomousVehicle>(gameConfig);
    if (new_game->Construct(gameConfig.width, gameConfig.height, 1, 1)){
        new_game->Start();
    }
    return 0;
}