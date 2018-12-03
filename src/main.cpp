#include <constants.hpp>
#include <coordinates.hpp>
#include <map.hpp>
#include <prediction.hpp>
#include <trajectory.hpp>
#include <trigonometry.hpp>
#include <vehicle.hpp>

#include <uWS.h>
#include <json.hpp>

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

vector<Vehicle> parseSensorFusionData(json &json) {
    std::vector<Vehicle> vehicles;

    //["sensor_fusion"] A 2d vector of cars and then that car's
    for (auto &data : json) {
        // 0 car's unique ID,
        // 1 car's x position in map coordinates,
        // 2 car's y position in map coordinates,
        // 3 car's x velocity in m/s,
        // 4 car's y velocity in m/s,
        // 5 car's s position in frenet coordinates,
        // 6 car's d position in frenet coordinates.
        vehicles.emplace_back(
                data[0],
                data[1],
                data[2],
                data[5],
                data[6],
                0., // TODO: calculate yaw
                data[3],
                data[4]
        );
    }


    return vehicles;
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

            auto jsonData = getJsonData(data);

            if (!jsonData.empty()) {
                auto j = json::parse(jsonData);

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
                    Vehicle ego{Vehicle::EGO_ID, car_x, car_y, car_s, car_d, car_yaw, car_speed * 0.44704};

                    // Parse sensor fusion data
                    std::vector<Vehicle> traffic = parseSensorFusionData(sensor_fusion);

                    double target_speed = MAX_VELOCITY;
                    int target_lane = ego.lane();

                    auto predictions = prediction::predictions(map, ego, traffic, 5);

                    // TODO: move to behaviour
                    if (!predictions.free_ahead) {
                        // The car is getting too close
                        if (predictions.free_left) {
                            // CL LEFT
                            std::cout << "CL LEFT" << std::endl;
                            target_lane -= 1;
                        } else if (predictions.free_right) {
                            // CL RIGHT
                            std::cout << "CL RIGHT" << std::endl;
                            target_lane += 1;
                        } else {
                            // KL follow
                            std::cout << "KL" << std::endl;
                            double distance = predictions.ahead->s() - ego.s();
                            target_speed = std::min(
                                    target_speed,
                                    distance < 10
                                    ? predictions.ahead->v() * 2.24 * .9
                                    : predictions.ahead->v() * 2.24
                            );
                        }
                    }

                    std::cout << "Position: s:" << ego.s() << " d:" << ego.d() << std::endl;
                    std::cout << predictions << std::endl;
                    if (!predictions.free_ahead) {
                        std::cout << "Distance to car: " << predictions.ahead->s() - ego.s() << std::endl;
                    }
                    std::cout << "Target speed: " << target_speed << std::endl;
                    std::cout << "Target lane: " << target_lane << " (current: " << ego.lane() << ")" << std::endl;

                    std::cout << std::endl;


                    trajectory::Path path = trajectory::calculatePath(map,
                                                                      ego,
                                                                      ref_vel,
                                                                      target_speed,
                                                                      target_lane,
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
