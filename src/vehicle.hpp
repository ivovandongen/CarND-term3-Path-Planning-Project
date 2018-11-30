#pragma once

class Vehicle {
public:
    Vehicle(double x, double y, double s, double d, double yaw, double v)
            : x_(x), y_(y), s_(s), d_(d), yaw_(yaw), v_(v) {
    }

    virtual ~Vehicle() = default;

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

private:
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double v_;
};