#pragma once

#include <coordinates.hpp>

#include <fmt/format.h>

template<>
struct fmt::formatter<cartesian::Coordinates> {
    template<typename ParseContext>
    constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

    template<typename FormatContext>
    auto format(const cartesian::Coordinates &c, FormatContext &ctx) {
        using namespace cartesian;
        return format_to(ctx.out(), "{}x{}", c.x(), c.y());
    }
};
