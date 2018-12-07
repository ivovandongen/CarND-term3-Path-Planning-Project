#pragma once

#include <behaviour.hpp>

#include <fmt/format.h>

template<>
struct fmt::formatter<behaviour::Action> {
    template<typename ParseContext>
    constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

    template<typename FormatContext>
    auto format(const behaviour::Action &a, FormatContext &ctx) {
        using namespace behaviour;
        switch (a) {
            case Action::INIT:
                return format_to(ctx.out(), "INIT");
            case Action::KEEP_LANE:
                return format_to(ctx.out(), "KEEP_LANE");
            case Action::CHANGE_LANE_LEFT:
                return format_to(ctx.out(), "CHANGE_LANE_LEFT");
            case Action::CHANGE_LANE_RIGHT:
                return format_to(ctx.out(), "CHANGE_LANE_RIGHT");
        }
    }
};

template<>
struct fmt::formatter<behaviour::State> {
    template<typename ParseContext>
    constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

    template<typename FormatContext>
    auto format(const behaviour::State &s, FormatContext &ctx) {
        using namespace behaviour;
        return format_to(ctx.out(),
                         "(Action: {}, time: {}, speed: {}mph, lane: {})",
                         s.action(),
                         s.ts(),
                         util::msToMph(s.speed()),
                         s.lane()
        );
    }
};
