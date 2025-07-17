#ifndef UTILS_H
#define UTILS_H

#include <sstream>
#include <string>

namespace autonomous_driving {

template <typename T>
bool extractWidthHeight(const std::string& window_size, T& width, T& height) {
    size_t xPos = window_size.find('x');
    if (xPos == std::string::npos || xPos >= window_size.length()) {
        return false;
    }

    std::stringstream first(window_size.substr(0, xPos));
    std::stringstream second(window_size.substr(xPos + 1));

    T w{};
    T h{};
    if (!(first >> w) || !(second >> h)) {
        return false;
    }

    width = w;
    height = h;
    return true;
}

} // namespace autonomous_driving

#endif // UTILS_H
