#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

class MyGameClass : public olc::PixelGameEngine
{
private:
    /* data */
public:
    MyGameClass(/* args */);
    ~MyGameClass();

    bool OnUserCreate() override;
    bool OnUserUpdate(float fElapsedTime) override;
};

MyGameClass::MyGameClass(/* args */)
{
    sAppName = "My New Game";
}

MyGameClass::~MyGameClass() = default;

bool MyGameClass::OnUserCreate() {
    return true;
}

bool MyGameClass::OnUserUpdate(float fElapsedTime) {
    // called once per frame
    for (int x = 0; x < ScreenWidth(); x++)
        for (int y = 0; y < ScreenHeight(); y++)
            Draw(x, y, olc::Pixel(rand() % 255, rand() % 255, rand()% 255));
    return true;
}

int main(){
    std::unique_ptr new_game = std::make_unique<MyGameClass>();
    if (new_game->Construct(500, 500, 4, 4)){
        new_game->Start();
    }
    
    return 0;
}