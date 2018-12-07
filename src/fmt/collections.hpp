#pragma once

#include <fmt/format.h>

#include <sstream>
#include <vector>

template<class T>
struct fmt::formatter<std::vector<T>> {
    template<typename ParseContext>
    constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

    template<typename FormatContext>
    auto format(const std::vector<T> &v, FormatContext &ctx) {
        std::stringstream ss;
        for (size_t i = 0; i < v.size(); i++) {
            ss << fmt::format("{}", v[i]);
            if (i < (v.size() - 1)) {
                ss << ", ";
            }
        }
        return format_to(ctx.out(), "{{{}}}", ss.str());
    }
};
