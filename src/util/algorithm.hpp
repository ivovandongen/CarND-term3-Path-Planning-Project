#pragma once

#include <algorithm>
#include <cmath>

namespace util {

template<class T>
T clamp(T input, T lowerBounds, T upperBounds) {
    return std::max<T>(std::min<T>(input, upperBounds), lowerBounds);
}

template<class T>
T diff_wrapped(T a, T b, T max) {
    return std::min(
            std::abs(std::fmod(a, max) - std::fmod(b, max)),
            std::abs(std::fmod(a, max) + max - std::fmod(b, max))
    );
}

} // namespace util
