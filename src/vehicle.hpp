#pragma once

#include <map.hpp>

class Vehicle {
public:
    const static int EGO_ID = -1;

    Vehicle(int id, double x, double y, double s, double d, double yaw, double v);

    Vehicle(int id, double x, double y, double s, double d, double yaw, double vx, double vy);

    virtual ~Vehicle() = default;

    int id() const {
        return id_;
    }

    double x() const {
        return x_;
    }

    double y() const {
        return y_;
    }

    double s() const {
        return s_;
    }

    double d() const {
        return d_;
    }

    double yaw() const {
        return yaw_;
    }

    double v() const {
        return v_;
    }

    int lane() const;

    Vehicle stateIn(const Map&, double secs) const;

private:
    int id_;
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double v_;
};