#pragma once

#include <math.h>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

inline double deg2rad(double x) { return x * pi() / 180; }

inline double rad2deg(double x) { return x * 180 / pi(); }

inline double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

static double distance(const std::vector<double> &x, const std::vector<double> &y) {
    assert(x.size() == y.size());
    if (x.size() < 2) {
        return 0;
    }

    double size = 0;
    for (size_t i = 1; i < x.size(); i++) {
        size += distance(x[i - 1], y[i - 1], x[i], y[i]);
    }

    return size;
}