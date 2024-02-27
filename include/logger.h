#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace autonomous_driving
{
namespace logger
{

enum class LogLevel {
    INFO,
    WARNING,
    ERROR
};

class Logger {
private:
    std::string getCurrentTime() const;

public:
    void log(LogLevel level, const std::string& message);
};

} // namespace logger
} // namespace autonomous_driving

#endif // LOGGER_H