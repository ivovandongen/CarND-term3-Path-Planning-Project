#include "vehicle.hpp"

#include <constants.hpp>

#include <cmath>

inline double velocity(double vx, double vy) {
    return sqrt(vx * vx + vy * vy);
}

Vehicle::Vehicle(int id, double s, double d, double yaw, double v)
        : id_(id), s_(s), d_(d), yaw_(yaw), v_(v) {
}

Vehicle::Vehicle(int id, double s, double d, double yaw, double vx, double vy)
        : id_(id), s_(s), d_(d), yaw_(yaw), v_(velocity(vx, vy)) {
}

int Vehicle::lane() const {
    return int(d() / LANE_WIDTH);
}

Vehicle Vehicle::stateIn(const Map &map, double secs) const {
    double newS = s_ + (v_ * secs);

    auto builder = VehicleBuilder::newBuilder(*this);
    if (coordinates_) {
        auto xy = map.getXY(newS, d_);
        builder.withCoordinates({xy[0], xy[1]});
    }
    return builder.withS(newS).build();
}

cartesian::Coordinates Vehicle::coordinates(const Map &map) const {
    if (coordinates_) {
        return *coordinates_;
    } else {
        auto xy = map.getXY(s_, d_);
        return {xy[0], xy[1]};
    }
}

VehicleBuilder VehicleBuilder::newEgoBuilder() {
    return VehicleBuilder{Vehicle::EGO_ID};
}

VehicleBuilder VehicleBuilder::newBuilder(int id) {
    return VehicleBuilder{id};
}

VehicleBuilder VehicleBuilder::newBuilder(const Vehicle &vehicle) {
    return VehicleBuilder{vehicle};
}

VehicleBuilder &VehicleBuilder::withS(double s) {
    vehicle_.s_ = s;
    vehicle_.coordinates_ = {};
    return *this;
}

VehicleBuilder &VehicleBuilder::withD(double d) {
    vehicle_.d_ = d;
    vehicle_.coordinates_ = {};
    return *this;
}


VehicleBuilder &VehicleBuilder::withYaw(double yaw) {
    vehicle_.yaw_ = yaw;
    return *this;
}

VehicleBuilder &VehicleBuilder::withV(double v) {
    vehicle_.v_ = v;
    return *this;
}

VehicleBuilder &VehicleBuilder::withV(double vx, double vy) {
    vehicle_.v_ = velocity(vx, vy);
    return *this;
}

VehicleBuilder &VehicleBuilder::withCoordinates(cartesian::Coordinates coordinates) {
    vehicle_.coordinates_ = coordinates;
    return *this;
}

Vehicle &&VehicleBuilder::build() {
    return std::move(vehicle_);
}
