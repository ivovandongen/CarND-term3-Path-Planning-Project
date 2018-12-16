#pragma once

static const int TRAJECTORY_POINTS = 50;

constexpr int LANE_WIDTH = 4;
constexpr int NUM_LANES = 3;
constexpr double MAX_VELOCITY_MPH = 49;
constexpr double MAX_VELOCITY_MS = 21.905; // (22.12848 == 49.5)
constexpr double MAX_ACCELERATION_MS_SQUARED = 4.5;
constexpr double SENSOR_RANGE = 30;

// Max s-value on track before wrapping around
constexpr double MAX_S = 6945.554;