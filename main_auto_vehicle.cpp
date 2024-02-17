#define OLC_PGE_APPLICATION
#include "auto_vehicle.h"

int main(){
    std::unique_ptr new_game = std::make_unique<autonomous_driving::AutonomousVehicle>();
    if (new_game->Construct(500, 500, 2, 2)){
        new_game->Start();
    }
    return 0;
}