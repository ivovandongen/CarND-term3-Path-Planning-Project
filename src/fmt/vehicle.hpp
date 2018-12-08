#pragma once

#include <vehicle.hpp>

#include <fmt/format.h>

template<>
struct fmt::formatter<Vehicle> {
    template<typename ParseContext>
    constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

    template<typename FormatContext>
    auto format(const Vehicle &v, FormatContext &ctx) {
        return format_to(ctx.out(),
                         "(id:{}, lane:{}, s:{}, d:{}, v:{})",
                         v.id(),
                         v.lane(),
                         v.s(),
                         v.d(),
                         v.v());
    }
};



