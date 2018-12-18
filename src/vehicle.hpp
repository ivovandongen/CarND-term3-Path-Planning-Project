#pragma once

#include <constants.hpp>
#include <coordinates.hpp>
#include <map.hpp>
#include <util/algorithm.hpp>

#include <tl/optional.hpp>

#include <cmath>

class VehicleBuilder;

class Vehicle {
public:
    const static int EGO_ID = -1;

    virtual ~Vehicle() = default;

    int id() const {
        return id_;
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

    double a() const {
        return a_;
    }

    tl::optional<cartesian::Coordinates> coordinates() const {
        return coordinates_;
    }

    cartesian::Coordinates coordinates(const Map &) const;

    cartesian::Coordinates coordinates(const Map &map);

    void coordinates(cartesian::Coordinates coordinates) {
        coordinates_ = coordinates;
    }

    int lane() const;

    bool isInFrontOf(const Vehicle &other) const {
        return util::in_front(s_, other.s(), MAX_S);
    }

    bool isBehindOf(const Vehicle &other) const {
        return !isInFrontOf(other);
    }

    Vehicle stateIn(const Map &, double secs) const;

private:

    explicit Vehicle(int id) : id_(id) {}

    friend class VehicleBuilder;

    int id_;
    double s_{};
    double d_{};
    double yaw_{};
    double v_{};
    double a_{};
    tl::optional<cartesian::Coordinates> coordinates_;
};

class VehicleBuilder {
public:
    static VehicleBuilder newEgoBuilder();

    static VehicleBuilder newBuilder(int id);

    static VehicleBuilder newBuilder(const Vehicle &vehicle);

    VehicleBuilder &withS(double s);

    VehicleBuilder &withD(double d);

    VehicleBuilder &withYaw(double yaw);

    VehicleBuilder &withV(double v);

    VehicleBuilder &withV(double vx, double vy);

    VehicleBuilder &withA(double a);

    VehicleBuilder &withCoordinates(cartesian::Coordinates coordinates);

    Vehicle &&build();

private:
    Vehicle vehicle_;

    explicit VehicleBuilder(int id) : vehicle_(id) {}

    explicit VehicleBuilder(const Vehicle &vehicle) : vehicle_(vehicle) {}

};