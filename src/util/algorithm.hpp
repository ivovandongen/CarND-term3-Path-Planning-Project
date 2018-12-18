#pragma once

#include <algorithm>
#include <cmath>

namespace util {

template<class T>
T clamp(T input, T lowerBounds, T upperBounds) {
    return std::max<T>(std::min<T>(input, upperBounds), lowerBounds);
}

template<class T>
T diff_wrapped_abs(T a, T b, T max) {
    return max / 2 - std::abs(std::abs(a - b) - max / 2);
}

template<class T>
T diff_wrapped(T a, T b, T max) {
    double difference = b - a;
    while (difference < -max / T{2}) difference += max;
    while (difference > max / T{2}) difference -= max;
    return difference;
}

template<class T>
bool in_front(T a, T b, T max) {
    auto diff = diff_wrapped(a, b, max);
    return diff < 0 ;
}


} // namespace util
