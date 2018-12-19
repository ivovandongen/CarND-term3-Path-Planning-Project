#pragma once

#include <prediction.hpp>

#include <fmt/vehicle.hpp>

#include <fmt/format.h>

namespace fmt {

template<>
struct formatter<prediction::Waypoint> {
    template<typename ParseContext>
    constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

    template<typename FormatContext>
    auto format(const prediction::Waypoint &p, FormatContext &ctx) {
        using namespace prediction;
        return format_to(ctx.out(),
                         "(ts: {}, state: {})",
                         p.ts,
                         p.state);
    }
};

template<>
struct formatter<prediction::Trajectory> {
    template<typename ParseContext>
    constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

    template<typename FormatContext>
    auto format(const prediction::Trajectory &t, FormatContext &ctx) {
        using namespace prediction;
        return format_to(ctx.out(),
                         "(probability: {}, trajectory: {})",
                         t.probability,
                         t.trajectory
        );
    }
};

template<>
struct formatter<prediction::Prediction> {
    template<typename ParseContext>
    constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

    template<typename FormatContext>
    auto format(const prediction::Prediction &p, FormatContext &ctx) {
        using namespace prediction;
        return format_to(ctx.out(),
                         "(Vehicle: {}, trajectories: {})",
                         p.vehicle_id,
                         p.trajectories
        );
    }
};

} // namespace fmt
