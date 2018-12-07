#pragma once

#include <algorithm>
#include <vector>

namespace util {

template<class T>
inline bool contains(const std::vector<T> &collection, const T &&t) {
    return std::find(collection.begin(), collection.end(), t) != collection.end();
}

} // namespace util
