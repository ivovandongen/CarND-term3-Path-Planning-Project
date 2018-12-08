#pragma once

#include <coordinates.hpp>
#include <map.hpp>

#include <tl/optional.hpp>

class VehicleBuilder;

class Vehicle {
public:
    const static int EGO_ID = -1;

    Vehicle(int id, double s, double d, double yaw, double v);

    Vehicle(int id, double s, double d, double yaw, double vx, double vy);

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

    tl::optional<cartesian::Coordinates> coordinates() const {
        return coordinates_;
    }

    cartesian::Coordinates coordinates(const Map &) const;

    void coordinates(cartesian::Coordinates coordinates) {
        coordinates_ = coordinates;
    }

    int lane() const;

    Vehicle stateIn(const Map &, double secs) const;

private:

    explicit Vehicle(int id) : id_(id) {}

    friend class VehicleBuilder;

    int id_;
    double s_{};
    double d_{};
    double yaw_{};
    double v_{};
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

    VehicleBuilder &withCoordinates(cartesian::Coordinates coordinates);

    Vehicle &&build();

private:
    Vehicle vehicle_;

    explicit VehicleBuilder(int id) : vehicle_(id) {}

    explicit VehicleBuilder(const Vehicle &vehicle) : vehicle_(vehicle) {}

};