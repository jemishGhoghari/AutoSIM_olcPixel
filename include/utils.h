#ifndef UTILS_H
#define UTILS_H

#include <sstream>
#include <string>
#include <vector>

namespace autonomous_driving {

template <typename T>
bool extractDelimitedValues(const std::string& text, char delimiter, std::vector<T>& values) {
    values.clear();
    std::stringstream stream(text);
    std::string token;

    while (std::getline(stream, token, delimiter)) {
        if (token.empty()) {
            return false;
        }

        std::stringstream tokenStream(token);
        T value{};
        if (!(tokenStream >> value)) {
            values.clear();
            return false;
        }

        values.push_back(value);
    }

    return !values.empty();
}

template <typename T>
bool extractWidthHeight(const std::string& window_size, T& width, T& height) {
    std::vector<T> values;
    if (!extractDelimitedValues<T>(window_size, 'x', values) || values.size() != 2) {
        return false;
    }

    width = values[0];
    height = values[1];
    return true;
}

template <typename T>
bool extractRectangle(const std::string& rectangle, T& x, T& y, T& width, T& height) {
    std::vector<T> values;
    if (!extractDelimitedValues<T>(rectangle, 'x', values) || values.size() != 4) {
        return false;
    }

    x = values[0];
    y = values[1];
    width = values[2];
    height = values[3];
    return true;
}

template <typename T>
bool extractBox(const std::string& box, T& x, T& y, T& z, T& width, T& depth, T& height) {
    std::vector<T> values;
    if (!extractDelimitedValues<T>(box, 'x', values) || values.size() != 6) {
        return false;
    }

    x = values[0];
    y = values[1];
    z = values[2];
    width = values[3];
    depth = values[4];
    height = values[5];
    return true;
}

} // namespace autonomous_driving

#endif // UTILS_H
