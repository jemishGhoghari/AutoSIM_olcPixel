#ifndef AUTO_VEHICLE_H
#define AUTO_VEHICLE_H

#include <cmath>
#include "olcPixelGameEngine.h"
#include "logger.h"
#include "vehicle.h"
#include "utils.h"

namespace autonomous_driving
{
/**
 * @brief Structure representing the initial configuration of a vehicle.
 */
struct InitialConfig{
    std::string CarPngPath; /**< Path to the car PNG file. */
    int32_t width; /**< Width of the vehicle. */
    int32_t height; /**< Height of the vehicle. */
    float_t xPosition; /**< X-coordinate of the initial position of the vehicle. */
    float_t yPosition; /**< Y-coordinate of the initial position of the vehicle. */
};

/**
 * @brief Reads YAML data from a file and populates the provided InitialConfig object.
 * 
 * This function reads YAML data from the specified file path and uses it to populate the provided
 * InitialConfig object. The YAML data should be in a format compatible with the structure of the
 * InitialConfig object.
 * 
 * @param yaml_path The path to the YAML file.
 * @param gameConfig The InitialConfig object to be populated with the YAML data.
 */
void getYAMLData(const std::string& yaml_path, InitialConfig& gameConfig);

/**
 * @brief Extracts the width and height from a given window size string.
 * 
 * This function takes a window size string and extracts the width and height values from it.
 * The extracted values are stored in the provided width and height variables.
 * 
 * @param window_size The window size string to extract the width and height from.
 * @param width The variable to store the extracted width value.
 * @param height The variable to store the extracted height value.
 * @return true if the extraction was successful, false otherwise.
 */
template <typename T>
bool extractWidthHeight(const std::string& window_size, T& width, T& height);

class AutonomousVehicle : public olc::PixelGameEngine
{
private:
    /**
     * @brief Represents a 2D vector in the olcPixelGameEngine library.
     * 
     * The `olc::vf2d` class is used to represent a 2D vector in the olcPixelGameEngine library.
     * It provides functions and operators for performing common vector operations such as addition,
     * subtraction, scalar multiplication, dot product, and normalization.
     * 
     * @note This class is part of the olcPixelGameEngine library and is not a standard C++ class.
     * 
     * @see olc::PixelGameEngine
     */
    olc::vf2d position;

    /**
     * @brief The acceleration of the vehicle.
     */
    float acceleration;

    /**
     * @brief The angle of the vehicle.
     */
    float angle;

    /**
     * @brief The maximum velocity of the vehicle.
     */
    float max_vel;

    /**
     * @brief The velocity of the vehicle.
     */
    float velocity;
  
    /**
     * @brief The rotational velocity of the vehicle.
     */
    float rotational_velocity;

    /**
     * Represents a 2D vector with integer components.
     * 
     * This class is used to store the coordinates of the center of gravity.
     * 
     * @note This class is part of the olcPixelGameEngine library.
     */
    olc::vi2d center_of_gravity;

    /**
     * @brief Represents the initial configuration for autonomous driving.
     * 
     * This class stores the initial configuration parameters required for autonomous driving.
     * It provides accessors and mutators for setting and retrieving the configuration values.
     */
    autonomous_driving::InitialConfig gameConfig;

    /**
     * @class olc::Sprite
     * @brief Represents a graphical sprite that can be rendered on the screen.
     *
     * The `olc::Sprite` class provides functionality to load, manipulate, and render graphical sprites.
     * Sprites can be used to represent various objects in a game or application.
     */
    olc::Sprite* NewCar;

    /**
     * @brief Represents a GPU accelerated sprite that can be rendered on the screen.
     * 
     * The `olc::Decal` class is used to represent a GPU accelerated sprite that can be rendered on the screen.
     * It provides efficient rendering of sprites by utilizing the graphics processing unit (GPU).
     * 
     * @see olc::Sprite
     */
    olc::Decal* gpuCarRender;

    /**
     * @brief The logger::Logger class is responsible for logging data.
     * 
     * This class provides functionality to log data for analysis and debugging purposes.
     * It can be used to log various types of data such as sensor readings, vehicle states, etc.
     * 
     * Usage example:
     * @code
     * logger::Logger data_log;
     * data_log.log("Sensor data: " + std::to_string(sensor_data));
     * @endcode
     */
    logger::Logger data_log;

public:
    AutonomousVehicle(const autonomous_driving::InitialConfig& gameConfig);
    ~AutonomousVehicle();

    /**
     * @brief This function is called once when the user creates the vehicle.
     * 
     * @return bool - Returns true if the function executed successfully, false otherwise.
     */
    bool OnUserCreate() override;

    /**
     * @brief Handles the user update event.
     *
     * This function is called on each frame update to handle user input and update the game state.
     *
     * @param fElapsedTime The elapsed time since the last frame update.
     * @return True if the update was successful, false otherwise.
     */
    bool OnUserUpdate(float fElapsedTime) override;

    /**
     * Moves the vehicle based on the elapsed time.
     *
     * @param fElapsedTime The elapsed time since the last update.
     */
    void move(float fElapsedTime);

    /**
     * Moves the vehicle forward by a specified amount of time.
     *
     * @param fElapsedTime The amount of time to move the vehicle forward.
     */
    void move_forward(float fElapsedTime);

    /**
     * Moves the vehicle backward.
     *
     * @param fElapsedTime The elapsed time since the last frame.
     */
    void move_backward(float fElapsedTime);

    /**
     * Rotates the car in the specified direction.
     *
     * @param left Specifies whether the car should rotate left.
     * @param right Specifies whether the car should rotate right.
     */
    void rotate_car(bool left, bool right);

    /**
     * @brief Sets the universal boundaries for the vehicle.
     * 
     * This function is responsible for setting the universal boundaries for the vehicle.
     * It defines the limits within which the vehicle can move or operate.
     * 
     * @return void
     */
    void universal_boundaries();
};
} // namespace autonomous_driving

#endif