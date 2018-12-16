#include "vehicle.hpp"

#include <constants.hpp>

#include <cmath>

inline double velocity(double vx, double vy) {
    return sqrt(vx * vx + vy * vy);
}

int Vehicle::lane() const {
    return int(d() / LANE_WIDTH);
}

Vehicle Vehicle::stateIn(const Map &map, double secs) const {
    double newS = s_ + (v_ * secs);
    return VehicleBuilder::newBuilder(*this)
            .withS(newS)
            .build();
}

cartesian::Coordinates Vehicle::coordinates(const Map &map) const {
    if (coordinates_) {
        return *coordinates_;
    } else {
        auto xy = map.getXY(s_, d_);
        return {xy[0], xy[1]};
    }
}

cartesian::Coordinates Vehicle::coordinates(const Map &map) {
    if (!coordinates_) {
        auto xy = map.getXY(s_, d_);
        coordinates_ = cartesian::Coordinates{xy[0], xy[1]};
    }
    return *coordinates_;
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

VehicleBuilder &VehicleBuilder::withA(double a) {
    vehicle_.a_ = a;
    return *this;
}

VehicleBuilder &VehicleBuilder::withCoordinates(cartesian::Coordinates coordinates) {
    vehicle_.coordinates_ = coordinates;
    return *this;
}

Vehicle &&VehicleBuilder::build() {
    return std::move(vehicle_);
}
