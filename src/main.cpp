#include <coordinates.hpp>
#include <map.hpp>
#include <trigonometry.hpp>
#include <vehicle.hpp>

#include <uWS.h>
#include <json.hpp>
#include <spline.h>

#include <cassert>
#include <math.h>
#include <chrono>
#include <thread>
#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string getJsonData(const string &s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// TODO: Find a place for these constants/state
int lane = 1;
const int lane_width = 4;
const double max_vel = 49.5;

// Max s-value on track before wrapping around
const double max_s = 6945.554;

struct Path {
    std::vector<double> x_vals;
    std::vector<double> y_vals;
    double target_vel;
};

Path calculatePath(double car_x, double car_y, double car_yaw) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    double dist_inc = 0.5;
    for (int i = 0; i < 50; i++) {
        next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
        next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
    }

    return {std::move(next_x_vals), std::move(next_y_vals)};
}

Path calculatePath(const Map &map, double car_s, double car_d) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    double dist_inc = 0.5;
    for (int i = 0; i < 50; i++) {
        double next_s = car_s + (i + 1) * dist_inc;
        double next_d = lane_width * 1.5;

        auto xy = map.getXY(next_s, next_d);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }

    return {std::move(next_x_vals), std::move(next_y_vals)};
}

Path calculatePath(const Map &map, const Vehicle &ego,
                   double ref_vel,
                   double target_vel,
                   const std::vector<double> &previous_path_x,
                   const std::vector<double> &previous_path_y) {

    size_t prev_size = previous_path_x.size();

//    std::cout << "Current speed: " << ego.v() << std::endl;

    // Way points
    vector<double> way_pts_x;
    vector<double> way_pts_y;

    // Reference state
    double ref_x = ego.x();
    double ref_y = ego.y();
    double ref_yaw = deg2rad(ego.yaw());

    if (prev_size < 2) {
        // Not enough previous path points to use as a reference, use
        // the car position and yaw to get the initial points
        double prev_car_x = ego.x() - cos(ego.yaw());
        double prev_car_y = ego.y() - sin(ego.yaw());

        way_pts_x.push_back(prev_car_x);
        way_pts_x.push_back(ego.x());

        way_pts_y.push_back(prev_car_y);
        way_pts_y.push_back(ego.y());
    } else {
        // Use the end of the remaining previous path
        // to start off the next point calculations

        // Last point
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        // Point before
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        way_pts_x.push_back(ref_x_prev);
        way_pts_x.push_back(ref_x);

        way_pts_y.push_back(ref_y_prev);
        way_pts_y.push_back(ref_y);
    }

    // Add way points
    for (size_t i = 0; i < 3; i++) {
        auto wp = map.getXY(ego.s() + ((i + 1) * 30), (2 + 4 * lane));
        way_pts_x.push_back(wp[0]);
        way_pts_y.push_back(wp[1]);
    }

    assert(way_pts_x.size() == way_pts_y.size());

    // Transform points to local coordinate space
    for (size_t i = 0; i < way_pts_x.size(); i++) {
        auto local = cartesian::Coordinates::toLocal(way_pts_x[i], way_pts_y[i], ref_x, ref_y, ref_yaw);
        way_pts_x[i] = local.x();
        way_pts_y[i] = local.y();
    }

    tk::spline spline;
    spline.set_points(way_pts_x, way_pts_y);

    vector<double> next_x_vals;
    next_x_vals.reserve(50);
    vector<double> next_y_vals;
    next_y_vals.reserve(50);

    // Add all remaining points from last iteration
    for (size_t i = 0; i < prev_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Determine target x,y,dist
    double target_x = 30;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    // Interpolate the spline at set intervals
    double x_add_on = 0;
    for (size_t i = prev_size; i < 50; i++) {
        if (ref_vel < target_vel) {
            ref_vel += .224;
        } else if (ref_vel > target_vel) {
            ref_vel -= .224;
        }
//        std::cout << "Target speed: " << ref_vel << std::endl;

        double N = (target_dist / (.02 * ref_vel / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = spline(x_point);

        x_add_on = x_point;

        auto global = cartesian::Coordinates::toGlobal(x_point, y_point, ref_x, ref_y, ref_yaw);
        next_x_vals.push_back(global.x());
        next_y_vals.push_back(global.y());
    }

    std::cout << "====" << ref_vel << std::endl;

    assert(next_x_vals.size() == next_y_vals.size());
    assert(next_x_vals.size() == 50);

    return {std::move(next_x_vals), std::move(next_y_vals), ref_vel};
}

int main() {
    uWS::Hub h;

    // Read waypoint map
    Map map = Map::loadFromFile("../data/highway_map.csv");

    // Some state
    double ref_vel = 0;

    h.onMessage([&map, &ref_vel](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = getJsonData(data);

            if (!s.empty()) {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    // The ego car state
                    Vehicle ego{car_x, car_y, car_s, car_d, car_yaw, car_speed};

                    //["sensor_fusion"] A 2d vector of cars and then that car's
                    // 0 car's unique ID,
                    // 1 car's x position in map coordinates,
                    // 2 car's y position in map coordinates,
                    // 3 car's x velocity in m/s,
                    // 4 car's y velocity in m/s,
                    // 5 car's s position in frenet coordinates,
                    // 6 car's d position in frenet coordinates.

                    double target_speed = max_vel;

                    // Check the way ahead
                    for (auto &car:sensor_fusion) {
                        double d = car[6];
                        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
                            // We're in the same lane
                            double vx = car[3];
                            double vy = car[4];
                            double speed = sqrt(vx * vx + vy * vy);
                            double s = car[5];

                            // Project s out
                            s += ((double) previous_path_x.size() * 0.02 * speed);
                            if ((s > car_s) && ((s - ego.s()) < 60)) {
                                std::cout << "Nearing collision with: " << car[0] << std::endl;
                                // The car is getting too close
                                target_speed = std::min(target_speed, speed * 2.24);
                            }
                        }
                    }

                    std::cout << "Target speed: " << target_speed << std::endl;


                    Path path = calculatePath(map,
                                              ego,
                                              ref_vel,
                                              target_speed,
                                              previous_path_x,
                                              previous_path_y);

                    // Store state for next iteration
                    ref_vel = path.target_vel;

                    json msgJson;
                    msgJson["next_x"] = path.x_vals;
                    msgJson["next_y"] = path.y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
