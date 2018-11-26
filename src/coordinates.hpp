#pragma once

namespace cartesian {
    class Coordinates {
    public:
        Coordinates(double x, double y);

        virtual ~Coordinates();

        static Coordinates toGlobal(double x, double y, double ref_x, double ref_y,
                                    double ref_angle);

        static Coordinates toLocal(double x, double y, double ref_x, double ref_y,
                                   double ref_angle);

        Coordinates toGlobal(double ref_x, double ref_y, double ref_angle) const;

        Coordinates toLocal(double ref_x, double ref_y, double ref_angle) const;

        double x() const {
            return x_;
        }

        double y() {
            return y_;
        }

    private:
        double x_;
        double y_;
    };
}
