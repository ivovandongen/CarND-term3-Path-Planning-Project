#pragma once

#include <algorithm>
#include <iostream>
#include <vector>

namespace util {

template<class T>
inline bool contains(const std::vector<T> &collection, const T &&t) {
    return std::find(collection.begin(), collection.end(), t) != collection.end();
}

} // namespace util

template<class T>
std::ostream &operator<<(std::ostream &stream, const std::vector<T> &v) {
    stream << "{";

    for (size_t i = 0; i < v.size(); i++) {
        stream << v[i];

        if (i < (v.size() - 1)) {
            stream << ", ";
        }
    }

    return stream << "}";
}
