#pragma once

#include <chrono>

namespace util {

using timestamp = long long int;

inline timestamp unix_ts() {
    using namespace std::chrono;
    milliseconds ms = duration_cast<milliseconds>(
            system_clock::now().time_since_epoch()
    );
    return ms.count();
}

} // timestamp
