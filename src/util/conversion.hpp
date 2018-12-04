#pragma once

namespace util {

inline double mphToMs(double mph) {
    return mph * .44704;
}

inline double msToMph(double ms) {
    return ms * 2.23694;
}

} // namespace util
