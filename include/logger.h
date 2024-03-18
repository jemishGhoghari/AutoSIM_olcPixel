#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace autonomous_driving
{
/**
 * @brief Namespace for autonomous driving related functionality.
 */
namespace logger
{

/**
 * @file logger.h
 * @brief Contains the declaration of the Logger class and the LogLevel enum class.
 */

/**
 * @brief The LogLevel enum class represents different levels of logging.
 */
enum class LogLevel {
    INFO, /**< Information level logging. */
    WARNING, /**< Warning level logging. */
    ERROR /**< Error level logging. */
};

/**
 * @class Logger
 * @brief The Logger class is responsible for logging messages with different log levels.
 */
class Logger {
private:
    /**
     * @brief Gets the current time as a string.
     * @return The current time as a string.
     */
    std::string getCurrentTime() const;

public:
    /**
     * @brief Logs a message with the specified log level.
     * @param level The log level.
     * @param message The message to be logged.
     */
    void log(LogLevel level, const std::string& message);
};

} // namespace logger
} // namespace autonomous_driving

#endif // LOGGER_H