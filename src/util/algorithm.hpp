#pragma once

#include <algorithm>

namespace util {

template<class T>
T clamp(T input, T lowerBounds, T upperBounds) {
    return std::max<T>(std::min<T>(input, upperBounds), lowerBounds);
}

} // namespace util
