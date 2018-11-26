#include "coordinates.hpp"

#include <math.h>

namespace cartesian {
    Coordinates Coordinates::toGlobal(const double x, const double y, const double ref_x, const double ref_y,
                                      const double ref_angle) {
        return {
                (x * cos(ref_angle) - y * sin(ref_angle)) + ref_x,
                (x * sin(ref_angle) + y * cos(ref_angle)) + ref_y
        };
    }

    Coordinates Coordinates::toLocal(const double x, const double y, const double ref_x, const double ref_y,
                                     const double ref_angle) {
        double deltaX = x - ref_x;
        double deltaY = y - ref_y;
        return {
                deltaX * cos(-ref_angle) - deltaY * sin(-ref_angle),
                deltaX * sin(-ref_angle) + deltaY * cos(-ref_angle)
        };

    }

    Coordinates::Coordinates(double x, double y) : x_(x), y_(y) {}

    Coordinates::~Coordinates() = default;

    Coordinates Coordinates::toGlobal(const double ref_x, const double ref_y,
                                      const double ref_angle) const {
        return Coordinates::toGlobal(x_, y_, ref_x, ref_y, ref_angle);
    }

    Coordinates Coordinates::toLocal(const double ref_x, const double ref_y,
                                     const double ref_angle) const {
        return Coordinates::toLocal(x_, y_, ref_x, ref_y, ref_angle);
    }

}