#pragma once

#include <vector>

class Map {

public:

    static Map loadFromFile(const std::string &fileName);

    Map(std::vector<double> x, std::vector<double> y,
        std::vector<double> s, std::vector<double> dx, std::vector<double> dy);

    virtual ~Map();

    /**
     * Find closest waypoint, regardless of heading.
     * @param x
     * @param y
     * @return
     */
    uint32_t closestWaypoint(double x, double y) const;

    /**
     * Find closest waypoint in the same heading.
     * @param x
     * @param y
     * @param theta
     * @return
     */
    uint32_t nextWaypoint(double x, double y, double theta) const;

    /**
     * Transform from Cartesian x,y coordinates to Frenet s,d coordinates
     * @param x
     * @param y
     * @param theta
     * @return
     */
    std::vector<double> getFrenet(double x, double y, double theta) const;

    /**
     * Transform from Cartesian x,y and acc (vx,vy) coordinates to Frenet s_dot,d_dot
     * @param x
     * @param y
     * @param vx
     * @param vy
     * @return
     */
    std::vector<double> getFrenetDot(double x, double y, double vx, double vy) const;

    /**
     * Transform from Frenet s,d coordinates to Cartesian x,y
     * @param s
     * @param d
     * @return
     */
    std::vector<double> getXY(double s, double d) const;

private:
    const std::vector<double> maps_x;
    const std::vector<double> maps_y;
    const std::vector<double> maps_s;
    const std::vector<double> maps_dx;
    const std::vector<double> maps_dy;
};


