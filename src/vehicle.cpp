#include "vehicle.hpp"

#include <constants.hpp>

#include <cmath>

Vehicle::Vehicle(int id, double x, double y, double s, double d, double yaw, double v)
        : id_(id), x_(x), y_(y), s_(s), d_(d), yaw_(yaw), v_(v) {
}

Vehicle::Vehicle(int id, double x, double y, double s, double d, double yaw, double vx, double vy)
        : id_(id), x_(x), y_(y), s_(s), d_(d), yaw_(yaw), v_(sqrt(vx * vx + vy * vy)) {
}

int Vehicle::lane() const {
    return int(d() / LANE_WIDTH);
}

Vehicle Vehicle::stateIn(const Map &map, double secs) const {
    double newS = s_ + (v_ * secs);
    auto xy = map.getXY(newS, d_);
    return {
            id_,
            xy[0],
            xy[1],
            newS,
            d_,
            yaw_, // TODO: yaw?
            v_
    };
}
