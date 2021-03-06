#include <behaviour.hpp>
#include <coordinates.hpp>
#include <constants.hpp>
#include <map.hpp>
#include <prediction.hpp>
#include <trajectory.hpp>
#include <trigonometry.hpp>
#include <vehicle.hpp>
#include <util/conversion.hpp>
#include <fmt/behaviour.hpp>
#include <fmt/coordinates.hpp>
#include <fmt/prediction.hpp>
#include <fmt/collections.hpp>

#include <uWS.h>
#include <json.hpp>
#include <fmt/printf.h>

#include <cassert>
#include <cmath>
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
        // TODO: calculate yaw?
        vehicles.push_back(
                VehicleBuilder::newBuilder((int) data[0])
                        .withS(data[5])
                        .withD(data[6])
                        .withV(data[3], data[4])
                        .withCoordinates({data[1], data[2]})
                        .build()
        );
    }

    return vehicles;
}

int main() {
    // Some static assertions
    assert(std::abs(MAX_VELOCITY_MS - util::mphToMs(MAX_VELOCITY_MPH)) < .1);

    uWS::Hub h;

    // Read waypoint map
    Map map = Map::loadFromFile("../data/highway_map.csv");

    behaviour::Behaviour behaviour(map);

    // Some state
    behaviour::State previousState = behaviour.state();
    double ref_vel = 0;

    h.onMessage([&map, &behaviour, &ref_vel, &previousState](
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
                    double car_speed_ms = util::mphToMs(j[1]["speed"]);

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    // The ego car state
                    Vehicle ego = VehicleBuilder::newEgoBuilder()
                            .withS(car_s)
                            .withD(car_d)
                            .withYaw(car_yaw)
                            .withV(car_speed_ms)
                            .withCoordinates({car_x, car_y})
                            .build();

                    // Parse sensor fusion data
                    std::vector<Vehicle> traffic = parseSensorFusionData(sensor_fusion);

                    // Get predictions
                    auto predictions = prediction::predictions(map, traffic, 5, .5);

                    // Get target behaviour (throttles internally)
                    auto targetState = behaviour.nextState(ego, predictions);

                    // Some debug logging
                    if (previousState.action() != targetState.action() ||
                        previousState.target().v() != targetState.target().v()) {

                        fmt::print("Position: s:{} d:{}\n", ego.s(), ego.d());
                        fmt::print("Predictions: {}\n", predictions);
                        fmt::print("Target state: {}\n", targetState);
                        std::cout << std::endl;
                    }


                    // Calculate trajectory
                    trajectory::Trajectory previousPath{
                            std::move(previous_path_x),
                            std::move(previous_path_y)
                    };

                    trajectory::Trajectory path = trajectory::calculateTrajectory(map,
                                                                                  ego,
                                                                                  targetState,
                                                                                  previousPath,
                                                                                  TRAJECTORY_POINTS
                                                                                  );

                    // Store state for next iteration
                    previousState = targetState;

                    // Return message
                    json msgJson;
                    msgJson["next_x"] = path.x;
                    msgJson["next_y"] = path.y;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

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
